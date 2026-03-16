"""Target fusion engine — duplicate merging and cross-drone fusion."""

import math
import time
import numpy as np
from typing import Dict, List, Optional, Tuple

import config
from modules.core.logger import SwarmLogger
from modules.core.geo_math import GeoMath
from modules.swarm.target import TrackedTarget, TargetRegistry, IMMUTABLE_STATES
from modules.swarm.target_fusion import (
    TargetFusionManager, weighted_covariance_fusion, should_fuse_targets,
)
from config import (
    SWARM_MERGE_DISTANCE_M, SWARM_FUSION_HARD_CAP_M, SWARM_FUSION_MAX_SIGMA_M,
    DUPLICATE_MERGE_DISTANCE_M, DUPLICATE_VELOCITY_SIMILARITY_MPS,
    MERGE_DISTANCE_BASE_M, MERGE_DISTANCE_MAX_M, MERGE_DISTANCE_SIGMA_SCALE,
    ENABLE_ATTACK_IMMUTABILITY,
)


def _sigma_from_cov(cov) -> Optional[float]:
    """Extract max sigma from covariance matrix."""
    try:
        if cov is None:
            return None
        v0 = float(cov[0][0]) if len(cov) > 0 else 0
        v1 = float(cov[1][1]) if len(cov) > 1 and len(cov[0]) > 1 else 0
        return math.sqrt(max(0.0, max(v0, v1)))
    except Exception:
        return None


class FusionEngine:
    """Two-layer target fusion engine:
        1. _merge_duplicates: Merge nearby targets seen by same drone
        2. fuse_cross_drone: Merge same target seen by different drones
    """

    def __init__(self, registry: TargetRegistry, fusion_threshold_m: float = None):
        self._registry = registry
        threshold = fusion_threshold_m if fusion_threshold_m else SWARM_MERGE_DISTANCE_M
        self._ekf_manager = TargetFusionManager(fusion_threshold_m=threshold)
        # Merge backoff spam protection: (id1, id2) → {"next_attempt": float, "penalty": float}
        self._merge_backoff: Dict[Tuple[str, str], Dict] = {}

    @property
    def ekf_manager(self) -> TargetFusionManager:
        """Direct access to EKF fusion manager."""
        return self._ekf_manager

    # ------------------------------------------------------------------
    # Dynamic merge threshold
    # ------------------------------------------------------------------

    @staticmethod
    def dynamic_merge_threshold(t1: TrackedTarget, t2: TrackedTarget) -> float:
        """Calculate dynamic merge threshold based on two targets' covariance."""
        s1 = _sigma_from_cov(t1.covariance)
        s2 = _sigma_from_cov(t2.covariance)

        if s1 is not None and s2 is not None:
            avg = (s1 + s2) / 2.0
        elif s1 is not None:
            avg = s1
        elif s2 is not None:
            avg = s2
        else:
            avg = 0.0

        threshold = MERGE_DISTANCE_BASE_M + avg * MERGE_DISTANCE_SIGMA_SCALE
        if t1.is_group and t2.is_group:
            avg_members = (int(t1.group_member_count or 1) + int(t2.group_member_count or 1)) / 2.0
            threshold += min(6.0, avg_members * 1.5)
        return min(threshold, MERGE_DISTANCE_MAX_M)

    # ------------------------------------------------------------------
    # Duplicate merging (CONFIRMED targets)
    # ------------------------------------------------------------------

    def merge_duplicates(self, ownership_mgr, identity_index=None) -> List[str]:
        """Merge spatially nearby CONFIRMED targets. Return removed IDs."""
        removed = []
        now = time.time()
        with self._registry.lock:
            all_ids = list(t_id for t_id, t in self._registry.items()
                           if t.track_state == "CONFIRMED")

            for i in range(len(all_ids)):
                id1 = all_ids[i]
                if id1 in removed:
                    continue
                t1 = self._registry.get(id1)
                if not t1 or t1.lat is None or t1.lon is None:
                    continue

                for j in range(i + 1, len(all_ids)):
                    id2 = all_ids[j]
                    if id2 in removed:
                        continue
                    t2 = self._registry.get(id2)
                    if not t2 or t2.lat is None or t2.lon is None:
                        continue

                    dist = GeoMath.haversine_distance(t1.lat, t1.lon, t2.lat, t2.lon)
                    threshold = self.dynamic_merge_threshold(t1, t2)

                    should_fuse, _, _ = should_fuse_targets(
                        (t1.lat, t1.lon), (t2.lat, t2.lon),
                        cov1=t1.covariance, cov2=t2.covariance,
                        threshold_m=threshold,
                    )
                    if not should_fuse:
                        continue

                    # Group safety
                    if not self._groups_compatible(t1, t2, now):
                        continue

                    # Attack protection
                    if self._both_in_attack(t1, t2):
                        continue

                    # Sigma quality
                    if not self._sigma_quality_ok(t1, t2, dist):
                        continue

                    # Velocity similarity
                    if not self._velocity_compatible(id1, id2, dist):
                        continue

                    # Immutability protection
                    pair = tuple(sorted([id1, id2]))
                    if self._merge_blocked_by_immutability(t1, t2, dist, pair):
                        continue

                    # Merge
                    keep_id, drop_id = self._select_canonical(id1, id2, t1, t2)
                    self._execute_merge(keep_id, drop_id, dist, ownership_mgr, identity_index)
                    removed.append(drop_id)

        return removed

    # ------------------------------------------------------------------
    # Cross-drone fusion
    # ------------------------------------------------------------------

    def fuse_cross_drone(self, ownership_mgr, identity_index=None) -> int:
        """Merge same targets seen by different drones."""
        fused_count = 0
        to_remove = []

        with self._registry.lock:
            now = time.time()
            ids = list(t_id for t_id, t in self._registry.items())

            for i in range(len(ids)):
                id1 = ids[i]
                if id1 in to_remove:
                    continue
                t1 = self._registry.get(id1)
                if not t1:
                    continue

                for j in range(i + 1, len(ids)):
                    id2 = ids[j]
                    if id2 in to_remove:
                        continue
                    t2 = self._registry.get(id2)
                    if not t2:
                        continue

                    # Group safety
                    if not self._groups_compatible(t1, t2, now):
                        continue

                    # Attack protection
                    if t1.is_immutable() or t2.is_immutable():
                        continue
                    if t1.approved_for_attack or t2.approved_for_attack:
                        continue

                    # Same drone/different tracker — no merge
                    if (t1.assigned_drone_port is not None
                            and t1.assigned_drone_port == t2.assigned_drone_port
                            and t1.tracker_id is not None and t2.tracker_id is not None
                            and t1.tracker_id != t2.tracker_id):
                        continue

                    # Fusion test
                    threshold = self.dynamic_merge_threshold(t1, t2)
                    should_merge, mahal, dist = should_fuse_targets(
                        (t1.lat, t1.lon), (t2.lat, t2.lon),
                        t1.covariance[:2, :2] if t1.covariance is not None else None,
                        t2.covariance[:2, :2] if t2.covariance is not None else None,
                        threshold_m=threshold,
                    )
                    
                    # --- Merge decision log ---
                    if not should_merge:
                        # Log rejection reason
                        SwarmLogger.log("FUSION_DETAIL", "FusionEngine",
                            f"MERGE_REJECT: {id1}+{id2} | dist={dist:.1f}m mahal={mahal:.1f}σ | "
                            f"threshold={threshold:.1f}m",
                            "FUSION")
                        continue

                    # Canonical selection
                    keep_id, drop_id = id1, id2
                    keep_t, drop_t = t1, t2
                    protected = {"ENGAGED", "LOCKED", "CONFIRMED_ATTACK",
                                 "CENTERING", "ATTACKING", "ECHO_WAIT"}

                    if t2.status in protected and t1.status not in protected:
                        keep_id, drop_id = id2, id1
                        keep_t, drop_t = t2, t1
                    elif t2.status in protected and t1.status in protected:
                        if t1.first_seen_time > t2.first_seen_time:
                            keep_id, drop_id = id2, id1
                            keep_t, drop_t = t2, t1

                    # Covariance fusion
                    if drop_t.covariance is not None:
                        try:
                            measurements = [
                                {"position": [keep_t.lat, keep_t.lon, 0], "covariance": keep_t.covariance},
                                {"position": [drop_t.lat, drop_t.lon, 0], "covariance": drop_t.covariance},
                            ]
                            fused_pos, fused_cov = weighted_covariance_fusion(measurements)
                            keep_t.lat, keep_t.lon = fused_pos[0], fused_pos[1]
                            keep_t.covariance = fused_cov
                        except Exception:
                            pass

                    # Transfer drone assignment
                    self._transfer_drone_on_merge(keep_id, keep_t, drop_id, drop_t, ownership_mgr)

                    # Merge local ID map and update identity index
                    for port, lid in drop_t.drone_local_ids.items():
                        keep_t.drone_local_ids[port] = lid
                        keep_t.drone_last_seen_time[port] = max(
                            keep_t.drone_last_seen_time.get(port, 0.0),
                            drop_t.drone_last_seen_time.get(port, 0.0),
                        )
                        keep_t.identity_last_update_time[port] = max(
                            keep_t.identity_last_update_time.get(port, 0.0),
                            drop_t.identity_last_update_time.get(port, 0.0),
                        )
                        # Update identity index: (port, lid) → keep_id
                        if identity_index is not None:
                            identity_index.update(port, lid, keep_id)
                    if drop_t.is_group:
                        keep_t.is_group = True
                        for port, count in getattr(drop_t, "group_count_by_port", {}).items():
                            keep_t.group_count_by_port[port] = max(
                                int(keep_t.group_count_by_port.get(port, 0) or 0),
                                int(count or 0),
                            )
                        for port, member_ids in drop_t.group_member_local_ids.items():
                            merged = set(keep_t.group_member_local_ids.get(port, ()))
                            merged.update(member_ids)
                            keep_t.group_member_local_ids[port] = tuple(sorted(merged))
                            keep_t.group_count_by_port[port] = max(
                                int(keep_t.group_count_by_port.get(port, 0) or 0),
                                len(keep_t.group_member_local_ids[port]),
                            )
                        for port, gid in getattr(drop_t, "group_id_by_port", {}).items():
                            if gid is not None and (
                                port not in keep_t.group_id_by_port
                                or drop_t.identity_last_update_time.get(port, 0.0)
                                >= keep_t.identity_last_update_time.get(port, 0.0)
                            ):
                                keep_t.group_id_by_port[port] = gid
                        keep_t.group_member_count = max(
                            int(keep_t.group_member_count or 0),
                            int(drop_t.group_member_count or 0),
                        )
                        keep_t.refresh_group_summary()

                    # Create alias
                    self._registry.add_alias(drop_id, keep_id)

                    to_remove.append(drop_id)
                    fused_count += 1
                    
                    # --- Merge success log ---
                    sigma_new = (keep_t.covariance[0, 0] + keep_t.covariance[1, 1]) ** 0.5 if keep_t.covariance is not None else 0.0
                    SwarmLogger.log("FUSION_DETAIL", "FusionEngine",
                        f"MERGE_OK: {drop_id} → {keep_id} | dist={dist:.1f}m mahal={mahal:.1f}σ | "
                        f"new_pos=({keep_t.lat:.6f},{keep_t.lon:.6f}) σ={sigma_new:.1f}m",
                        "FUSION")

            for tid in to_remove:
                self._ekf_manager.remove_target(tid)
                self._registry.remove(tid)

        return fused_count

    # ------------------------------------------------------------------
    # EKF batch processing
    # ------------------------------------------------------------------

    def process_ekf_batch(self, target_coasting_status: Dict[str, bool] = None):
        """EKF fusion batch — called in periodic loop.
        
        Args:
            target_coasting_status: {target_id: is_coasting} dict (Phase 1)
            
        Fix: Coast flag now only resets when actual observation arrives.
        For coasting targets, predict-only step is executed.
        """
        if target_coasting_status is None:
            target_coasting_status = {}
        
        try:
            # --- STEP 1: Process actual observations ---
            results = self._ekf_manager.process_observations(target_coasting_status)
            
            with self._registry.lock:
                # Apply actual observation results
                for t_id, (fused_pos, fused_cov, fused_vel) in results.items():
                    t = self._registry.get(t_id)
                    if t:
                        old_lat, old_lon = t.lat, t.lon
                        
                        gps = self._ekf_manager.ned_to_gps(fused_pos)
                        t.lat = gps[0]
                        t.lon = gps[1]
                        t.last_velocity = fused_vel
                        
                        # Actual observation arrived → exit coast mode
                        if t.is_coasting:
                            t.is_coasting = False
                            t.coast_start_time = None
                        t.coast_distance_traveled = 0.0
                        
                        # Log
                        sigma_out = (fused_cov[0, 0] + fused_cov[1, 1]) ** 0.5 if fused_cov.shape[0] >= 2 else 0.0
                        vel_mag = np.linalg.norm(fused_vel) if fused_vel is not None else 0.0
                        if old_lat is not None and old_lon is not None:
                            drift = GeoMath.haversine_distance(old_lat, old_lon, t.lat, t.lon)
                            SwarmLogger.log("EKF_OUTPUT", "FusionEngine",
                                f"{t_id}: EKF UPDATED | "
                                f"gps=({t.lat:.6f},{t.lon:.6f}) | "
                                f"ned=({fused_pos[0]:.1f},{fused_pos[1]:.1f})m | "
                                f"σ_out={sigma_out:.1f}m | Δ={drift:.1f}m | vel={vel_mag:.1f}m/s",
                                "FILTER")
                        else:
                            SwarmLogger.log("EKF_OUTPUT", "FusionEngine",
                                f"{t_id}: EKF INITIAL | "
                                f"gps=({t.lat:.6f},{t.lon:.6f}) | σ_out={sigma_out:.1f}m",
                                "FILTER")
                
                # --- STEP 2: Predict-only for coasting targets ---
                # For targets without observation but coasting, update position
                # using velocity model (EKF prediction step)
                for t_id, is_coasting in target_coasting_status.items():
                    if not is_coasting:
                        continue
                    if t_id in results:
                        continue  # Already updated with observation
                    
                    t = self._registry.get(t_id)
                    kf = self._ekf_manager.filters.get(t_id)
                    if t and kf:
                        pred_pos, pred_cov = kf.predict(is_coasting=True)
                        gps = self._ekf_manager.ned_to_gps(pred_pos)
                        
                        old_lat, old_lon = t.lat, t.lon
                        t.lat = gps[0]
                        t.lon = gps[1]
                        t.last_velocity = kf.get_velocity()
                        
                        # Update coast distance
                        if old_lat is not None and old_lon is not None:
                            step_dist = GeoMath.haversine_distance(old_lat, old_lon, t.lat, t.lon)
                            t.coast_distance_traveled += step_dist
                        
                        sigma_out = (pred_cov[0, 0] + pred_cov[1, 1]) ** 0.5 if pred_cov.shape[0] >= 2 else 0.0
                        SwarmLogger.log("EKF_OUTPUT", "FusionEngine",
                            f"{t_id}: COAST PREDICT | "
                            f"gps=({t.lat:.6f},{t.lon:.6f}) | "
                            f"σ_out={sigma_out:.1f}m | coast_dist={t.coast_distance_traveled:.1f}m",
                            "FILTER")
                        
        except Exception as e:
            SwarmLogger.log("ERROR", "FUSION", f"EKF Process Error: {e}", "FUSION")

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _targets_share_identity_evidence(
        t1: TrackedTarget, t2: TrackedTarget, current_time: Optional[float] = None
    ) -> bool:
        """Whether two targets have direct local-ID evidence of being the same family."""
        current_time = time.time() if current_time is None else current_time
        common_ports = set(t1.drone_local_ids).intersection(set(t2.drone_local_ids))
        for port in common_ports:
            if not (
                t1.has_recent_identity(port, current_time, config.TRACK_IDENTITY_CONFLICT_WINDOW_SEC)
                and t2.has_recent_identity(port, current_time, config.TRACK_IDENTITY_CONFLICT_WINDOW_SEC)
            ):
                continue

            members1 = set(int(mid) for mid in (t1.group_member_local_ids.get(port, ()) or ()))
            members2 = set(int(mid) for mid in (t2.group_member_local_ids.get(port, ()) or ()))
            if members1 and members2 and (members1 & members2):
                return True
            if t1.is_group and t2.is_group:
                continue

            lid1 = t1.drone_local_ids.get(port)
            lid2 = t2.drone_local_ids.get(port)
            if lid1 is not None and lid2 is not None and int(lid1) == int(lid2):
                return True
        return False

    @staticmethod
    def _targets_have_conflicting_local_identity(
        t1: TrackedTarget, t2: TrackedTarget, current_time: Optional[float] = None
    ) -> bool:
        """Whether two targets have evidence of being distinct families on the same drone.

        If the same drone currently sees both targets under different local IDs
        or disjoint group-member sets, they must not be merged even if spatially
        close. This is the exact failure mode that can collapse two nearby 2x
        groups into one canonical target.
        """
        current_time = time.time() if current_time is None else current_time
        common_ports = set(t1.drone_local_ids).intersection(set(t2.drone_local_ids))
        if not common_ports:
            return False

        for port in common_ports:
            if not (
                t1.has_recent_identity(port, current_time, config.TRACK_IDENTITY_CONFLICT_WINDOW_SEC)
                and t2.has_recent_identity(port, current_time, config.TRACK_IDENTITY_CONFLICT_WINDOW_SEC)
            ):
                continue
            lid1 = t1.drone_local_ids.get(port)
            lid2 = t2.drone_local_ids.get(port)
            members1 = set(int(mid) for mid in (t1.group_member_local_ids.get(port, ()) or ()))
            members2 = set(int(mid) for mid in (t2.group_member_local_ids.get(port, ()) or ()))

            if members1 and members2:
                if members1.isdisjoint(members2):
                    return True
                continue

            if lid1 is not None and lid2 is not None and int(lid1) != int(lid2):
                return True

        return False

    @staticmethod
    def _groups_compatible(
        t1: TrackedTarget, t2: TrackedTarget, current_time: Optional[float] = None
    ) -> bool:
        """Are group targets compatible?"""
        current_time = time.time() if current_time is None else current_time
        if FusionEngine._targets_have_conflicting_local_identity(t1, t2, current_time):
            return False
        if t1.is_group or t2.is_group:
            if t1.is_group != t2.is_group:
                return False
            count_diff = abs(int(t1.group_member_count or 0) - int(t2.group_member_count or 0))
            if count_diff > 1:
                return False
            if count_diff > 0 and not FusionEngine._targets_share_identity_evidence(t1, t2, current_time):
                return False
        return True

    @staticmethod
    def _both_in_attack(t1: TrackedTarget, t2: TrackedTarget) -> bool:
        """Are both in attack pipeline?"""
        a1 = t1.is_in_attack_pipeline() or t1.approved_for_attack
        a2 = t2.is_in_attack_pipeline() or t2.approved_for_attack
        return a1 and a2

    @staticmethod
    def _sigma_quality_ok(t1: TrackedTarget, t2: TrackedTarget, dist: float) -> bool:
        """Does sigma quality pass threshold?"""
        max_sigma = SWARM_FUSION_MAX_SIGMA_M
        if max_sigma <= 0:
            return True
        s1 = _sigma_from_cov(t1.covariance)
        s2 = _sigma_from_cov(t2.covariance)
        if (s1 is not None and s1 > max_sigma) or (s2 is not None and s2 > max_sigma):
            return dist <= 3.0
        return True

    def _velocity_compatible(self, id1: str, id2: str, dist: float) -> bool:
        """Velocity similarity check."""
        pos1 = self._ekf_manager.get_target_position(id1)
        pos2 = self._ekf_manager.get_target_position(id2)
        if not (pos1 and pos2):
            return True
        kf1 = self._ekf_manager.filters.get(id1)
        kf2 = self._ekf_manager.filters.get(id2)
        if not (kf1 and kf2):
            return True
        vel_diff = np.linalg.norm(kf1.get_velocity() - kf2.get_velocity())
        if vel_diff > DUPLICATE_VELOCITY_SIMILARITY_MPS:
            bypass = DUPLICATE_MERGE_DISTANCE_M / 3.0
            if dist < bypass:
                SwarmLogger.log("MERGE", "LEADER",
                                f"MERGE: {id1} & {id2} velocity bypass ({dist:.1f}m)", "FUSION")
                return True
            return False
        return True

    def _merge_blocked_by_immutability(self, t1, t2, dist, pair) -> bool:
        """Immutability protection and backoff check."""
        if not ENABLE_ATTACK_IMMUTABILITY or dist < 5.0:
            return False

        if not (t1.status in IMMUTABLE_STATES or t2.status in IMMUTABLE_STATES
                or t1.approved_for_attack or t2.approved_for_attack):
            return False

        now = time.time()
        backoff = self._merge_backoff.get(pair, {
            "next_attempt": 0.0,
            "penalty": config.SWARM_MERGE_BACKOFF_INIT_S,
        })
        if now < backoff["next_attempt"]:
            return True

        SwarmLogger.log(
            "IMMUTABLE", "LEADER",
            f"MERGE ABORTED: {pair[0]} ({t1.status}) or {pair[1]} ({t2.status}) protected",
            "MERGE",
        )
        new_penalty = min(backoff["penalty"] * 2.0, config.SWARM_MERGE_BACKOFF_MAX_S)
        self._merge_backoff[pair] = {"next_attempt": now + new_penalty, "penalty": new_penalty}
        return True

    def _select_canonical(self, id1, id2, t1, t2):
        """Select canonical (kept) target for merge."""
        protected = {"ENGAGED", "LOCKED", "CONFIRMED_ATTACK",
                     "CENTERING", "ATTACKING", "ECHO_WAIT"}
        keep_id, drop_id = id1, id2
        if t2.status in protected and t1.status not in protected:
            keep_id, drop_id = id2, id1
        elif t2.status in protected and t1.status in protected:
            if t2.observation_count > t1.observation_count:
                keep_id, drop_id = id2, id1
        elif t2.observation_count > t1.observation_count:
            keep_id, drop_id = id2, id1
        return keep_id, drop_id

    def _execute_merge(self, keep_id, drop_id, dist, ownership_mgr, identity_index=None):
        """Merge two targets."""
        keep = self._registry.get(keep_id)
        drop = self._registry.get(drop_id)
        if not keep or not drop:
            return

        SwarmLogger.log(
            "MERGE", "LEADER",
            f"MERGE: {keep_id} ← {drop_id} | Dist={dist:.1f}m | "
            f"KeepObs={keep.observation_count} DropObs={drop.observation_count}",
            "FUSION",
        )

        # Transfer drone assignment
        self._transfer_drone_on_merge(keep_id, keep, drop_id, drop, ownership_mgr)

        # Merge local ID map and update identity index
        for port, lid in drop.drone_local_ids.items():
            if port not in keep.drone_local_ids:
                keep.drone_local_ids[port] = lid
            keep.drone_last_seen_time[port] = max(
                keep.drone_last_seen_time.get(port, 0.0),
                drop.drone_last_seen_time.get(port, 0.0),
            )
            keep.identity_last_update_time[port] = max(
                keep.identity_last_update_time.get(port, 0.0),
                drop.identity_last_update_time.get(port, 0.0),
            )
            # Update identity index: (port, lid) → keep_id
            if identity_index is not None:
                identity_index.update(port, lid, keep_id)
                SwarmLogger.log("MERGE", "LEADER",
                    f"Identity: Drone_{port} local_id={lid} → {keep_id}", "FUSION")

        # Transfer group info
        if drop.is_group:
            keep.is_group = True
            keep.group_member_count = max(
                int(keep.group_member_count or 0),
                int(drop.group_member_count or 0),
            )
            for port, member_ids in drop.group_member_local_ids.items():
                merged = set(keep.group_member_local_ids.get(port, ()))
                merged.update(member_ids)
                keep.group_member_local_ids[port] = tuple(sorted(merged))
            for port, gid in getattr(drop, "group_id_by_port", {}).items():
                if gid is not None and (
                    port not in keep.group_id_by_port
                    or drop.identity_last_update_time.get(port, 0.0)
                    >= keep.identity_last_update_time.get(port, 0.0)
                ):
                    keep.group_id_by_port[port] = gid
            preferred_port = keep.assigned_drone_port or keep.owner_port or drop.assigned_drone_port
            if preferred_port in keep.group_id_by_port:
                keep.group_id_local = keep.group_id_by_port[preferred_port]
            elif keep.group_id_local is None:
                keep.group_id_local = drop.group_id_local

        # Cleanup
        self._ekf_manager.remove_target(drop_id)
        self._registry.add_alias(drop_id, keep_id)
        self._registry.remove(drop_id)

    def _transfer_drone_on_merge(self, keep_id, keep_t, drop_id, drop_t, ownership_mgr):
        """Transfer drone assignment during merge."""
        if not drop_t.assigned_drone_port:
            return

        port = drop_t.assigned_drone_port
        # Is this drone tracking another target?
        busy_elsewhere = False
        for t_id, t in self._registry.items():
            if t_id == drop_id:
                continue
            if t.assigned_drone_port == port and t.track_state != "DELETED":
                if t.status in {"PENDING", "ACTIVE"} or t.is_in_attack_pipeline():
                    busy_elsewhere = True
                    break

        if keep_t.assigned_drone_port is None and not busy_elsewhere:
            ownership_mgr.release(port)
            ownership_mgr.assign(port, keep_id)
            keep_t.tracker_id = drop_t.tracker_id
        elif keep_t.assigned_drone_port is not None and not busy_elsewhere:
            ownership_mgr.drone_active_target[port] = keep_id
            ownership_mgr.target_owner.pop(drop_id, None)
            drop_t.assigned_drone_port = None
            SwarmLogger.log("MERGE", "LEADER",
                            f"Drone_{port} Transferred ({drop_id} → {keep_id})", "FUSION")
        else:
            ownership_mgr.release(port)
            SwarmLogger.log("MERGE", "LEADER",
                            f"Drone_{port} Released ({drop_id} → {keep_id} full)", "FUSION")
