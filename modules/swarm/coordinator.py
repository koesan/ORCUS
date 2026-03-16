"""Main swarm coordinator — thin orchestration layer.

All business logic is delegated to sub-modules.
This file only routes incoming messages and runs the periodic loop.
"""

import math
import time
import threading
import weakref
import numpy as np
from typing import Dict, List, Optional, Tuple

import config
from modules.core.logger import SwarmLogger
from modules.core.geo_math import GeoMath
from modules.swarm.target import (
    TrackedTarget, TargetRegistry, IMMUTABLE_STATES,
    OWNERSHIP_FREE, OWNERSHIP_OWNED, OWNERSHIP_RESERVED,
)
from modules.swarm.state_machine import TargetLifecycle
from modules.swarm.identity import LocalIdentityIndex
from modules.swarm.ownership import OwnershipManager
from modules.swarm.assignment import AssignmentEngine
from modules.swarm.fusion_engine import FusionEngine
from modules.swarm.protocol import AttackProtocol
from modules.swarm.battlespace import BattlespaceView
from modules.swarm.target_fusion import should_fuse_targets

from config import (
    SWARM_MERGE_DISTANCE_M, SWARM_ATTACK_MIN_CONFIDENCE,
    ASSIGNMENT_LOOP_INTERVAL_S, SWARM_ALIGNMENT_TOLERANCE_M,
    SWARM_LOCAL_ID_INDEX_MAX_JUMP_M, SWARM_GEO_COOLDOWN_S,
    ENABLE_SPATIAL_FALLBACK, TRACKER_ID_ALPHA,
    DUPLICATE_MERGE_DISTANCE_M, LEADER_TIMEOUT_S,
    ENABLE_CAPACITY_GUARD, ENABLE_ATTACK_IMMUTABILITY,
    MERGE_DISTANCE_BASE_M, MERGE_DISTANCE_MAX_M, MERGE_DISTANCE_SIGMA_SCALE,
)


class SwarmManager:
    """Swarm intelligence coordinator — API compatible with legacy SwarmManager."""

    _instances = weakref.WeakSet()

    def __init__(self, drone_manager):
        self.drone_manager = drone_manager

        # Sub-modules
        self.registry = TargetRegistry()
        self.lifecycle = TargetLifecycle(self.registry)
        self.identity = LocalIdentityIndex()
        self.ownership = OwnershipManager(self.registry)
        self.assignment = AssignmentEngine(self.registry, drone_manager)
        self.fusion = FusionEngine(self.registry)
        self.protocol = AttackProtocol(self.registry, self.ownership, self.assignment)
        self.battlespace = BattlespaceView(self.registry, drone_manager, self.fusion)

        # Compatibility: fields expected by old API
        self.targets = self.registry._targets
        self.drone_targets: Dict[int, list] = {}
        self.target_aliases = self.registry._aliases
        self.drone_active_target = self.ownership.drone_active_target
        self.target_owner = self.ownership.target_owner
        self.lock = self.registry.lock
        self.attack_mode_active = False

        # Heartbeat monitoring
        self.drone_heartbeat_ts: Dict[int, float] = {}

        # Geo-mismatch cooldown
        self.drone_mismatch_cooldowns: Dict[int, Dict[str, float]] = {}

        # Log rate-limit
        self._immutable_log_ts: Dict = {}
        self._immutable_log_target: Dict = {}
        self._action_log_ts: Dict = {}

        # Periodic loop
        self._loop_running = True
        self._loop_thread = threading.Thread(
            target=self._periodic_loop,
            name="SwarmAssignmentLoop",
            daemon=True,
        )

        SwarmLogger.init_log()
        SwarmLogger.log("INFO", "LEADER",
                        "Battlespace Manager v4 (Modular Refactor) Started.", "INIT")
        self._loop_thread.start()
        self._instances.add(self)

    # ==================================================================
    # PUBLIC API — Fully compatible with legacy module
    # ==================================================================

    def report_target(self, drone_port, lat, lon, confidence,
                      tracker_id=None, raw_data=None, covariance=None):
        """Process incoming target detection and return action."""
        if raw_data != "HEARTBEAT" and confidence < SWARM_ATTACK_MIN_CONFIDENCE:
            return {"action": "SEARCH", "message": "Low Confidence"}

        with self.lock:
            self.identity.cleanup(active_targets=self.targets)

            # Message routing
            if isinstance(raw_data, dict) and "action" in raw_data:
                msg = raw_data.get("action")
                t_id = raw_data.get("target_id")
                if msg == "REQUEST_LOCK":
                    return self.protocol.handle_lock_request(drone_port, t_id, raw_data)
                elif msg == "LOCKED":
                    return self.protocol.handle_lock_confirmation(drone_port, t_id)

            # Heartbeat
            if raw_data == "HEARTBEAT":
                return self._handle_heartbeat(drone_port)

            # Leader verification
            final_lat, final_lon = lat, lon
            is_edge = False
            if raw_data and isinstance(raw_data, dict):
                final_lat, final_lon, is_edge = self._leader_verify(
                    lat, lon, raw_data
                )

            current_time = time.time()

            # Pruning
            self.lifecycle.prune(
                current_time, self.attack_mode_active,
                drone_connected_check=self._is_drone_connected,
                release_drone=self.ownership.release,
                remove_fusion_target=self.fusion.ekf_manager.remove_target,
            )

            # Matching
            best_id, match_reason = self._match_target(
                drone_port, final_lat, final_lon, tracker_id, covariance, raw_data,
            )

            target_id = None

            if best_id:
                target_id = self._update_existing_target(
                    best_id, match_reason, drone_port, final_lat, final_lon,
                    tracker_id, covariance, raw_data, is_edge, current_time,
                )
            else:
                target_id = self._create_new_target(
                    drone_port, final_lat, final_lon, tracker_id, covariance,
                    raw_data, is_edge, current_time,
                )

            if not target_id or target_id not in self.targets:
                return {"action": "SEARCH", "message": "Target lost"}

            # EKF registration - with confidence (Phase 2)
            self._record_fusion(target_id, final_lat, final_lon, covariance, drone_port, confidence)

            # TENTATIVE redirect
            target = self.targets[target_id]
            if target.track_state == "TENTATIVE":
                redirect = self._tentative_redirect(target_id, drone_port, final_lat, final_lon,
                                                     tracker_id, covariance, raw_data)
                if redirect:
                    return redirect

            # Result
            return self._build_report_response(target, target_id, drone_port, tracker_id)

    def check_mission_updates(self, drone_port, raw_data=None):
        """Process incoming telemetry/detections and return action."""
        needs_prune = True

        # Route incoming data
        if raw_data is not None:
            if isinstance(raw_data, dict) and "detected_targets" in raw_data:
                det = raw_data.get("detected_targets", [])
                needs_prune = not bool(det)
                self.drone_targets[drone_port] = det
                for item in det:
                    lat, lon = item.get("world_xyz", (0, 0))[:2]
                    conf = item.get("confidence", 0)
                    tid = item.get("track_id")
                    cov_list = item.get("covariance")
                    cov_arr = np.array(cov_list) if cov_list else None
                    self.report_target(drone_port, lat, lon, conf, tid,
                                       raw_data=item, covariance=cov_arr)

            elif isinstance(raw_data, dict) and "action" in raw_data:
                return self._dispatch_action(drone_port, raw_data)

            elif isinstance(raw_data, str) and raw_data == "HEARTBEAT":
                needs_prune = False
                self.report_target(drone_port, 0, 0, 0, None, raw_data="HEARTBEAT")

        current_time = time.time()
        with self.lock:
            if needs_prune:
                self.lifecycle.prune(
                    current_time, self.attack_mode_active,
                    drone_connected_check=self._is_drone_connected,
                    release_drone=self.ownership.release,
                    remove_fusion_target=self.fusion.ekf_manager.remove_target,
                )
            return self._build_mission_action(drone_port)

    def get_battlespace_state(self):
        """Battlespace state for Web UI."""
        return self.battlespace.get_state(self.ownership)

    def approve_attack(self, target_id):
        """Single target attack approval."""
        return self.protocol.approve_attack(target_id)

    def approve_all_targets(self):
        """Approve all CONFIRMED targets."""
        with self.lock:
            self.attack_mode_active = True
            SwarmLogger.log("USER_CMD", "LEADER", "ATTACK MODE ACTIVATED", "USER")

            eligible = []
            for t_id, t in self.targets.items():
                if t.track_state == "DELETED" or t.status == "LOST":
                    continue
                if t.approved_for_attack:
                    continue
                if t.lat is None:
                    continue
                stab = t.consecutive_obs / (1.0 + t.id_switch_count + t.drop_count)
                eligible.append((t_id, t, stab))

            eligible.sort(key=lambda x: x[2], reverse=True)

            approved_targets = []
            count = 0
            for t_id, t, stab in eligible:
                dup = False
                for approved in approved_targets:
                    a_t = approved["target"]
                    if a_t.is_group != t.is_group:
                        continue
                    if t.is_group and abs(int(t.group_member_count or 0) - int(a_t.group_member_count or 0)) > 1:
                        continue
                    exclusion = self._attack_family_exclusion_radius(t, a_t)
                    if GeoMath.haversine_distance(t.lat, t.lon, a_t.lat, a_t.lon) < exclusion:
                        dup = True
                        break
                if dup:
                    continue

                if t.status == "PENDING" or (t.track_state == "CONFIRMED" and not t.approved_for_attack):
                    t.approved_for_attack = True
                    t.status = "ECHO_WAIT"
                    SwarmLogger.log("USER_CMD", "LEADER", f"MASS APPROVE: {t_id} → ECHO_WAIT", "USER")
                    count += 1
                    approved_targets.append({"target": t, "id": t_id, "stab": stab})

            return count

    @staticmethod
    def _attack_family_exclusion_radius(t1, t2) -> float:
        """Conservative family radius for batch attack approval."""
        if getattr(t1, "is_group", False) and getattr(t2, "is_group", False):
            avg_members = (int(t1.group_member_count or 1) + int(t2.group_member_count or 1)) / 2.0
            if SwarmManager._targets_share_identity_evidence(t1, t2):
                return max(DUPLICATE_MERGE_DISTANCE_M, 14.0 + 3.0 * avg_members)
            return max(10.0, DUPLICATE_MERGE_DISTANCE_M * 0.75)
        return DUPLICATE_MERGE_DISTANCE_M

    @staticmethod
    def _targets_share_identity_evidence(t1, t2) -> bool:
        """Whether two targets have direct local-ID evidence of being the same family."""
        common_ports = set(getattr(t1, "drone_local_ids", {})).intersection(
            set(getattr(t2, "drone_local_ids", {}))
        )
        for port in common_ports:
            members1 = set(int(mid) for mid in (t1.group_member_local_ids.get(port, ()) or ()))
            members2 = set(int(mid) for mid in (t2.group_member_local_ids.get(port, ()) or ()))
            if members1 and members2 and (members1 & members2):
                return True
            if getattr(t1, "is_group", False) and getattr(t2, "is_group", False):
                continue

            lid1 = t1.drone_local_ids.get(port)
            lid2 = t2.drone_local_ids.get(port)
            if lid1 is not None and lid2 is not None and int(lid1) == int(lid2):
                return True
        return False

    def release_drone_assignment(self, drone_port):
        """Release drone assignment (R1/R32)."""
        with self.lock:
            released = [t_id for t_id, t in self.targets.items()
                        if t.assigned_drone_port == drone_port]
            self.ownership.release(drone_port)
            self.drone_heartbeat_ts.pop(drone_port, None)
            if released:
                SwarmLogger.log("LIFECYCLE", "LEADER",
                                f"Drone_{drone_port} RELEASED: targets={released}", "TRACK")

    def handle_drone_disconnect(self, port):
        """Handle drone disconnect — ownership release, heartbeat cleanup.

        Called by fleet_manager.connect_drone_async() on disconnect.
        """
        with self.lock:
            # Ownership release
            released = [t_id for t_id, t in self.targets.items()
                        if t.assigned_drone_port == port]
            self.ownership.release(port)

            # Heartbeat cleanup
            self.drone_heartbeat_ts.pop(port, None)

            # Clear drone targets
            self.drone_targets.pop(port, None)

            # Clear mismatch cooldown
            self.drone_mismatch_cooldowns.pop(port, None)

            if released:
                SwarmLogger.log("LIFECYCLE", "LEADER",
                                f"Drone_{port} DISCONNECTED — Released targets: {released}",
                                "DISCONNECT")
            else:
                SwarmLogger.log("LIFECYCLE", "LEADER",
                                f"Drone_{port} DISCONNECTED — No active targets.",
                                "DISCONNECT")

    def is_drone_busy(self, drone_port):
        """Is drone busy?"""
        return self.ownership.is_drone_busy(drone_port)

    def assign_attack_targets(self):
        """Optimal assignment."""
        return self.assignment.assign_attack_targets(self.ownership)

    def fuse_cross_drone_targets(self):
        """Cross-drone fusion (with identity index)."""
        return self.fusion.fuse_cross_drone(self.ownership, self.identity)

    def get_assigned_local_id(self, drone_port):
        """Local ID of drone's target."""
        with self.lock:
            for t_id, t in self.targets.items():
                if t.assigned_drone_port == drone_port and t.status == "ENGAGED":
                    return t.drone_local_ids.get(drone_port)
        return None

    def get_drone_targets_buffer(self):
        """Drone detection buffer."""
        return self.drone_targets

    def reset_runtime_state(self):
        """Soft reset volatile swarm state while preserving drone links."""
        with self.lock:
            self.registry._targets.clear()
            self.registry._aliases.clear()
            self.registry._alias_ttl.clear()
            self.registry._counter = 0
            self.drone_targets.clear()
            self.drone_heartbeat_ts.clear()
            self.drone_active_target.clear()
            self.target_owner.clear()
            self.attack_mode_active = False
            self.identity._index.clear()
            self.identity._log_debounce.clear()
            self.identity._id_history.clear()
            self.fusion._merge_backoff.clear()
            self.fusion.ekf_manager.reset_runtime_state()
            self.battlespace.reset_runtime_state()
        SwarmLogger.log("RESET", "LEADER",
                        "Soft reset completed: radar, targets, fusion and ownership cleared.",
                        "SYSTEM")

    # Compatibility properties
    @property
    def fusion_manager(self):
        return self.fusion.ekf_manager

    @property
    def local_identity_index(self):
        return self.identity._index

    # Backward compat — used in old tests
    def _prune_targets(self, current_time):
        """Backward compat wrapper for lifecycle.prune."""
        return self.lifecycle.prune(
            current_time, self.attack_mode_active,
            drone_connected_check=self._is_drone_connected,
            release_drone=self.ownership.release,
            remove_fusion_target=self.fusion.ekf_manager.remove_target,
        )

    def _release_drone(self, port):
        """Backward compat wrapper for ownership.release."""
        self.ownership.release(port)

    def is_target_in_attack_pipeline(self, target):
        """Backward compat."""
        return target.is_in_attack_pipeline()

    # ==================================================================
    # INTERNAL HELPERS
    # ==================================================================

    def _is_drone_connected(self, port: int) -> bool:
        """Is drone still connected?"""
        return (port in self.drone_manager.drones
                and self.drone_manager.drones[port] != "connecting")

    def _is_drone_stopping(self, port: int) -> bool:
        """Don't publish new attack to drone in STOP/RTL transition."""
        controllers = getattr(self.drone_manager, "drone_controllers", {}) or {}
        controller = controllers.get(port)
        return bool(getattr(controller, "is_stopping", False))

    def _handle_heartbeat(self, drone_port):
        """Process heartbeat message."""
        for t_id, t in self.targets.items():
            is_protected = (t.status == "ENGAGED"
                            or (ENABLE_ATTACK_IMMUTABILITY and t.status in IMMUTABLE_STATES))
            if t.assigned_drone_port == drone_port and is_protected:
                t.last_seen_time = time.time()
                self.drone_heartbeat_ts[drone_port] = time.time()
                local_id = t.drone_local_ids.get(drone_port)
                SwarmLogger.log("HEARTBEAT", f"DRONE_{drone_port}",
                                f"Target {t_id} still {t.status}", "FAILSAFE")
                return {"action": "ATTACK", "target_id": t_id, "tracker_id": local_id}
        return {"action": "SEARCH", "message": "No active target for heartbeat"}

    def _leader_verify(self, lat, lon, raw_data):
        """Leader verification — RGI cross-check."""
        final_lat, final_lon = lat, lon
        is_edge = False

        try:
            if "drone_gps" not in raw_data or "bbox" not in raw_data:
                return final_lat, final_lon, is_edge

            d_lat, d_lon, d_alt = raw_data["drone_gps"]
            cx, cy, w, h = raw_data["bbox"]

            W = float(config.CAMERA_WIDTH)
            H = float(config.CAMERA_HEIGHT)
            margin_x = W * config.SWARM_EDGE_FILTER_MARGIN_RATIO
            margin_y = H * config.SWARM_EDGE_FILTER_MARGIN_RATIO
            area_thresh = (W * config.SWARM_EDGE_FILTER_AREA_RATIO) * (H * config.SWARM_EDGE_FILTER_AREA_RATIO)

            edge_x = cx < margin_x or cx > (W - margin_x)
            edge_y = cy < margin_y or cy > (H - margin_y)
            is_edge = (edge_x or edge_y) and (w * h) < area_thresh

            # For group targets, bbox center does not represent the same physical
            # point as group centroid. Especially from oblique angles, bbox center
            # shifts left/right of the group and corrupts world centroid, producing
            # false duplicates. Therefore, bbox-based RGI override is not applied
            # for group targets; detection layer determines group center.
            if raw_data.get("is_group"):
                return final_lat, final_lon, is_edge

            if "attitude" in raw_data:
                roll, pitch, yaw = raw_data["attitude"]
            else:
                heading = raw_data.get("heading", 0)
                roll, pitch = 0.0, 0.0
                yaw = math.radians(heading) if heading else 0.0

            gcx, gcy = GeoMath.bbox_ground_contact_point(
                cx, cy, w, h, config.CAMERA_GROUND_CONTACT_Y_RATIO
            )
            result = GeoMath.ray_ground_intersection(d_lat, d_lon, d_alt,
                                                      roll, pitch, yaw, gcx, gcy)
            v_lat, v_lon = result[0], result[1]

            if v_lat is not None:
                diff = GeoMath.haversine_distance(lat, lon, v_lat, v_lon)
                if diff > SWARM_ALIGNMENT_TOLERANCE_M:
                    final_lat, final_lon = v_lat, v_lon
                else:
                    final_lat = (lat + v_lat) / 2
                    final_lon = (lon + v_lon) / 2

        except Exception as e:
            SwarmLogger.log("ERROR", "SwarmCoordinator",
                            f"Leader Verification error: {e}", "LEADER")

        return final_lat, final_lon, is_edge

    @staticmethod
    def _sigma_from_covariance(covariance) -> Optional[float]:
        try:
            cov = np.array(covariance, dtype=np.float64)
            if cov.ndim != 2 or cov.shape[0] < 2 or cov.shape[1] < 2:
                return None
            return float(np.sqrt(max(0.0, max(float(cov[0, 0]), float(cov[1, 1])))))
        except Exception:
            return None

    def _association_threshold(self, target, incoming_covariance, incoming_is_group: bool,
                               same_drone: bool = False) -> float:
        """Adaptive association threshold for incoming reports.

        This is intentionally covariance-driven, not a blind merge-distance bump.
        Different viewing geometry produces different centroid errors; group
        observations get a modest bonus because their centroid is an abstraction,
        not a point target.
        """
        s_existing = self._sigma_from_covariance(target.covariance)
        s_incoming = self._sigma_from_covariance(incoming_covariance)

        sigmas = [s for s in (s_existing, s_incoming) if s is not None]
        avg_sigma = sum(sigmas) / len(sigmas) if sigmas else 0.0

        threshold = MERGE_DISTANCE_BASE_M + avg_sigma * MERGE_DISTANCE_SIGMA_SCALE
        if incoming_is_group and getattr(target, "is_group", False):
            avg_members = (int(target.group_member_count or 1) + 1) / 2.0
            threshold += min(6.0, 2.0 + avg_members * 1.5)
        if same_drone:
            threshold += 2.0

        return min(threshold, max(MERGE_DISTANCE_MAX_M, DUPLICATE_MERGE_DISTANCE_M))

    @staticmethod
    def _same_drone_group_id_match(target, drone_port: int, incoming_group_id, current_time: float) -> bool:
        """Group IDs are not stable enough to be used as family evidence."""
        return False

    @staticmethod
    def _conflicts_with_same_drone_identity(target, drone_port: int,
                                            tracker_id, incoming_member_ids: set,
                                            current_time: float) -> bool:
        """Block matching/duplicate collapse when same drone has distinct evidence.

        A drone simultaneously observing two nearby groups must keep them
        separate unless there is direct overlap evidence.
        """
        if not target.has_recent_identity(drone_port, current_time, config.TRACK_IDENTITY_CONFLICT_WINDOW_SEC):
            return False
        existing_local_id = target.drone_local_ids.get(drone_port)
        existing_members = set(target.group_member_local_ids.get(drone_port, ()) or ())

        if incoming_member_ids and existing_members:
            return incoming_member_ids.isdisjoint(existing_members)

        if tracker_id is None or existing_local_id is None:
            return False

        try:
            return int(tracker_id) != int(existing_local_id)
        except (TypeError, ValueError):
            return False

    def _match_target(self, drone_port, lat, lon, tracker_id, covariance, raw_data=None):
        """Match incoming detection with existing targets."""
        current_time = time.time()
        best_id = None
        best_score = float("inf")
        reason = ""
        incoming_is_group = bool(isinstance(raw_data, dict) and raw_data.get("is_group"))
        incoming_group_count = int(raw_data.get("group_member_count", 1)) if isinstance(raw_data, dict) else 1
        incoming_group_id = raw_data.get("group_id") if isinstance(raw_data, dict) else None
        incoming_member_ids = set()
        if isinstance(raw_data, dict):
            for mid in raw_data.get("member_ids", []) or []:
                try:
                    incoming_member_ids.add(int(mid))
                except (TypeError, ValueError):
                    continue

        # 1. Local ID Index
        if tracker_id is not None and not incoming_is_group:
            idx_tid = self.identity.lookup(drone_port, tracker_id)
            if idx_tid and idx_tid in self.targets:
                cand = self.targets[idx_tid]
                if cand.track_state != "DELETED" and cand.status != "LOST":
                    try:
                        d = GeoMath.haversine_distance(lat, lon, cand.lat, cand.lon)
                    except Exception:
                        d = 0.0
                    if d <= SWARM_LOCAL_ID_INDEX_MAX_JUMP_M:
                        return idx_tid, "LOCAL_ID_INDEX"
        elif tracker_id is not None and incoming_is_group:
            idx_tid = self.identity.lookup(drone_port, tracker_id)
            if idx_tid and idx_tid in self.targets:
                cand = self.targets[idx_tid]
                if (cand.track_state != "DELETED"
                        and cand.status != "LOST"
                        and cand.is_group):
                    count_diff = abs(int(cand.group_member_count or 0) - incoming_group_count)
                    if count_diff == 0 and not self._conflicts_with_same_drone_identity(
                        cand, drone_port, tracker_id, incoming_member_ids, current_time
                    ):
                        try:
                            d = GeoMath.haversine_distance(lat, lon, cand.lat, cand.lon)
                        except Exception:
                            d = 0.0
                        threshold = self._association_threshold(
                            cand, covariance, True, same_drone=True
                        )
                        if d <= min(SWARM_LOCAL_ID_INDEX_MAX_JUMP_M, threshold):
                            return idx_tid, "LOCAL_GROUP_ID_INDEX"

        # 2. Spatial-first association
        if ENABLE_SPATIAL_FALLBACK:
            for t_id, t in self.targets.items():
                if t.track_state == "DELETED":
                    continue
                if incoming_is_group != bool(t.is_group):
                    continue
                if incoming_is_group and t.is_group:
                    if drone_port in t.drone_local_ids and self._conflicts_with_same_drone_identity(
                        t, drone_port, tracker_id, incoming_member_ids, current_time
                    ):
                        continue
                    existing_members = set(t.group_member_local_ids.get(drone_port, ()))
                    same_family_evidence = bool(
                        incoming_member_ids and existing_members and (incoming_member_ids & existing_members)
                    )
                    if same_family_evidence:
                        return t_id, "GROUP_MEMBER_OVERLAP"
                    group_bias = 1.0
                    count_diff = abs(int(t.group_member_count or 0) - incoming_group_count)
                    if count_diff > 1:
                        continue
                    if count_diff > 0 and not same_family_evidence:
                        continue
                else:
                    group_bias = 1.0
                threshold = self._association_threshold(
                    t, covariance, incoming_is_group, same_drone=(drone_port in t.drone_local_ids)
                )
                ok, mahal, raw_dist = should_fuse_targets(
                    (lat, lon), (t.lat, t.lon),
                    cov1=covariance, cov2=t.covariance,
                    threshold_m=threshold,
                )
                dist_m = GeoMath.haversine_distance(lat, lon, t.lat, t.lon)
                if not ok and dist_m < threshold:
                    ok = True
                    raw_dist = dist_m / 10.0
                if not ok:
                    continue

                score = mahal * group_bias
                if tracker_id is not None:
                    mapped = t.drone_local_ids.get(drone_port)
                    if mapped == tracker_id:
                        score *= (1.0 - TRACKER_ID_ALPHA)

                if score < best_score:
                    best_score = score
                    best_id = t_id
                    reason = "SPATIAL_MAHALANOBIS"
        else:
            if tracker_id is not None:
                for t_id, t in self.targets.items():
                    if t.assigned_drone_port == drone_port and t.tracker_id == tracker_id:
                        return t_id, "ID_PERSIST"

        return best_id, reason

    def _update_existing_target(self, best_id, reason, drone_port,
                                 lat, lon, tracker_id, covariance,
                                 raw_data, is_edge, current_time):
        """Update existing target."""
        target = self.targets[best_id]

        # Ownership check
        is_owner = (target.owner_port == drone_port)
        has_owner = (target.owner_port is not None)
        quality = self.ownership.calculate_quality(raw_data or {})

        if has_owner and not is_owner:
            self.ownership.process_secondary_observation(target, drone_port, raw_data or {})
            target.update_position(lat, lon, covariance, is_edge=is_edge)
            immutable_identity = (
                target.assigned_drone_port is not None
                and target.assigned_drone_port != drone_port
                and (target.is_in_attack_pipeline() or target.status in IMMUTABLE_STATES)
            )
            if not immutable_identity:
                self._update_group_metadata(target, best_id, drone_port, raw_data)
                if tracker_id is not None:
                    self._update_local_id(target, best_id, drone_port, tracker_id, current_time)
            return best_id

        # Owner observation
        target.update_position(lat, lon, covariance, is_edge=is_edge)
        target.record_observation(drone_port, quality, is_owner=True)

        # RESERVED → OWNED reclamation
        if target.ownership_state == OWNERSHIP_RESERVED:
            target.ownership_state = OWNERSHIP_OWNED
            target.owner_lost_time = None
            # BUG2 FIX: update assigned_drone_port - drone reclaimed ownership
            if target.assigned_drone_port != drone_port:
                old_assignee = target.assigned_drone_port
                target.assigned_drone_port = drone_port
                SwarmLogger.log("OWNERSHIP", "LEADER",
                    f"{best_id}: RE-ASSIGNED {old_assignee} → {drone_port} (reclaimed)",
                    "OWNERSHIP")

        # Update group metadata
        self._update_group_metadata(target, best_id, drone_port, raw_data)

        # TENTATIVE → CONFIRMED promotion
        just_confirmed = self.lifecycle.check_confirmation(
            target, current_time, is_searching=not self.attack_mode_active
        )
        if just_confirmed and self.attack_mode_active and target.assigned_drone_port is not None:
            target.approved_for_attack = True
            target.status = "ENGAGED"

        # Local ID update (with hysteresis)
        if tracker_id is not None:
            self._update_local_id(target, best_id, drone_port, tracker_id, current_time)

        return best_id

    def _create_new_target(self, drone_port, lat, lon, tracker_id,
                            covariance, raw_data, is_edge, current_time):
        """Create new target or merge with nearby existing target."""
        incoming_is_group = bool(isinstance(raw_data, dict) and raw_data.get("is_group"))
        incoming_group_count = int(raw_data.get("group_member_count", 1)) if isinstance(raw_data, dict) else 1
        incoming_group_id = raw_data.get("group_id") if isinstance(raw_data, dict) else None
        incoming_member_ids = set()
        if isinstance(raw_data, dict):
            for mid in raw_data.get("member_ids", []) or []:
                try:
                    incoming_member_ids.add(int(mid))
                except (TypeError, ValueError):
                    continue

        # Nearby duplicate check
        for eid, et in self.targets.items():
            if et.track_state == "DELETED" or et.lat is None:
                continue
            if incoming_is_group != bool(et.is_group):
                continue
            if incoming_is_group and et.is_group:
                if drone_port in et.drone_local_ids and self._conflicts_with_same_drone_identity(
                    et, drone_port, tracker_id, incoming_member_ids, current_time
                ):
                    continue
                existing_members = set(et.group_member_local_ids.get(drone_port, ()))
                same_family_evidence = bool(
                    incoming_member_ids and existing_members and (incoming_member_ids & existing_members)
                )
                if same_family_evidence:
                    duplicate_threshold = max(DUPLICATE_MERGE_DISTANCE_M, 20.0)
                else:
                    duplicate_threshold = self._association_threshold(
                        et, covariance, True, same_drone=(drone_port in et.drone_local_ids)
                    )
                count_diff = abs(int(et.group_member_count or 0) - incoming_group_count)
                if count_diff > 1:
                    continue
                if count_diff > 0 and not same_family_evidence:
                    continue
            else:
                duplicate_threshold = self._association_threshold(et, covariance, False)
            dist = GeoMath.haversine_distance(lat, lon, et.lat, et.lon)
            if dist < duplicate_threshold:
                et.update_position(lat, lon, covariance, is_edge=is_edge)
                self._update_group_metadata(et, eid, drone_port, raw_data)
                if tracker_id is not None:
                    self._update_local_id(et, eid, drone_port, tracker_id, current_time)
                quality = self.ownership.calculate_quality(raw_data or {})
                et.record_observation(drone_port, quality, is_owner=False)
                return eid

        # New target
        t_id = self.registry.generate_id()
        new_t = TrackedTarget(t_id, lat, lon, current_time,
                              tracker_id=tracker_id, covariance=covariance,
                              source_drone_port=drone_port)

        # Initial ownership
        has_existing = any(
            t.assigned_drone_port == drone_port and t.track_state != "DELETED"
            for t in self.targets.values()
        )
        if ENABLE_CAPACITY_GUARD and has_existing:
            new_t.ownership_state = OWNERSHIP_FREE
            new_t.owner_port = drone_port
            new_t.assigned_drone_port = None
        else:
            new_t.ownership_state = OWNERSHIP_OWNED
            new_t.owner_port = drone_port
            new_t.assigned_drone_port = drone_port

        quality = self.ownership.calculate_quality(raw_data or {})
        new_t.record_observation(drone_port, quality, is_owner=True)

        if is_edge:
            new_t.observation_count = 0
            new_t.consecutive_obs = 0
            new_t._edge_accumulator = config.SWARM_EDGE_FILTER_FRACTIONAL_CREDIT

        self.registry.add(new_t)
        self.ownership.target_owner[t_id] = drone_port
        if new_t.assigned_drone_port == drone_port:
            self.ownership.drone_active_target[drone_port] = t_id

        if tracker_id is not None:
            self.identity.update(drone_port, tracker_id, t_id, now=current_time)

        self._update_group_metadata(new_t, t_id, drone_port, raw_data)

        SwarmLogger.log(
            "TARGET", "LEADER",
            f"NEW TARGET (TENTATIVE): {t_id} (TID:{tracker_id}) @ ({lat:.6f}, {lon:.6f}) | Owner: Drone_{drone_port}",
            "TARGET",
        )

        return t_id

    def _update_local_id(self, target, t_id, drone_port, tracker_id, current_time):
        """Update local ID with hysteresis."""
        old = target.drone_local_ids.get(drone_port)

        if (old is not None and old != tracker_id
                and target.is_in_attack_pipeline()
                and target.assigned_drone_port == drone_port):
            # ID change during attack: requires 10 consecutive frames
            hyst = target.local_id_hysteresis.setdefault(drone_port, {"id": None, "count": 0})
            if hyst["id"] == tracker_id:
                hyst["count"] += 1
            else:
                hyst["id"] = tracker_id
                hyst["count"] = 1
            if hyst["count"] >= 10:
                target.drone_local_ids[drone_port] = tracker_id
                target.identity_last_update_time[drone_port] = current_time
                target.tracker_id = tracker_id
                self.identity.log_id_update(drone_port, t_id, old, tracker_id)
                hyst["count"] = 0
        else:
            target.drone_local_ids[drone_port] = tracker_id
            target.identity_last_update_time[drone_port] = current_time
            if target.assigned_drone_port == drone_port:
                target.tracker_id = tracker_id
            self.identity.update(drone_port, tracker_id, t_id, now=current_time)
            if old is not None and old != tracker_id:
                self.identity.log_id_update(drone_port, t_id, old, tracker_id)

    def _record_fusion(self, target_id, lat, lon, covariance, drone_port, confidence: float = 1.0):
        """EKF fusion record.
        
        Args:
            confidence: Detection confidence (0-1)
        """
        try:
            if covariance is not None and hasattr(covariance, "shape"):
                obs_cov = np.array(covariance, dtype=np.float64)
            else:
                obs_cov = np.eye(3) * 4.0

            t = self.targets.get(target_id)
            if t:
                self.fusion.ekf_manager.add_observation(
                    target_id, (lat, lon, 0.0), obs_cov, drone_port,
                    confidence=confidence,  # Phase 2
                    is_group=t.is_group,
                    group_member_count=t.group_member_count,
                )
        except Exception as e:
            SwarmLogger.log("ERROR", "SwarmCoordinator",
                            f"add_observation error: {e}", "LEADER")

    def _tentative_redirect(self, target_id, drone_port, lat, lon, tracker_id, covariance, raw_data=None):
        """Redirect TENTATIVE target to nearby CONFIRMED target.
        
        Fix: When redirect succeeds, return canonical target info.
        Instead of None, return canonical target response → caller takes correct action.
        """
        for t_check_id, t_check in self.targets.items():
            if (t_check.assigned_drone_port == drone_port
                    and t_check.track_state == "CONFIRMED"
                    and t_check_id != target_id):
                dist = GeoMath.haversine_distance(
                    self.targets[target_id].lat, self.targets[target_id].lon,
                    t_check.lat, t_check.lon,
                )
                if self.targets[target_id].is_group != t_check.is_group:
                    continue
                if self.targets[target_id].is_group and abs(
                    int(self.targets[target_id].group_member_count or 0) - int(t_check.group_member_count or 0)
                ) > 1:
                    continue
                if dist < 20.0:
                    t_check.update_position(lat, lon, covariance)
                    if tracker_id is not None:
                        self._update_local_id(t_check, t_check_id, drone_port, tracker_id, time.time())
                    if raw_data:
                        self._update_group_metadata(t_check, t_check_id, drone_port, raw_data)
                    del self.targets[target_id]
                    SwarmLogger.log("REDIRECT", "LEADER",
                        f"TENTATIVE {target_id} → CONFIRMED {t_check_id} (dist={dist:.1f}m)",
                        "LIFECYCLE")
                    # Return canonical target's response
                    return self._build_report_response(t_check, t_check_id, drone_port, tracker_id)

        return {"action": "SEARCH", "target_id": target_id,
                "tracker_id": tracker_id, "message": "Awaiting confirmation"}

    def _build_report_response(self, target, target_id, drone_port, tracker_id):
        """report_target result output.
        
        Fix: Filtered position is read from EKF.
        If EKF not yet processed, target.lat is used as fallback.
        """
        # Get filtered position from EKF (if available)
        filtered_pos = self.fusion.ekf_manager.get_target_position(target_id)
        if filtered_pos is not None:
            gps_pos, cov = filtered_pos
            lat_filtered = float(gps_pos[0])
            lon_filtered = float(gps_pos[1])
            covariance_filtered = cov
        else:
            # Fallback: if EKF not yet created, use target values
            lat_filtered = target.lat
            lon_filtered = target.lon
            covariance_filtered = target.covariance
        
        # Drone's local ID for this target
        drone_local_id = self._resolve_local_target_id(target, drone_port)
        
        # Base response - always include filtered position
        base_response = {
            "target_id": target_id,
            "tracker_id": target.tracker_id,
            # Drone's local ID → global ID mapping
            "local_id": drone_local_id,
            # Filtered position info
            "lat": lat_filtered,
            "lon": lon_filtered,
            "covariance": covariance_filtered.tolist() if covariance_filtered is not None else None,
        }
        
        if target.assigned_drone_port == drone_port:
            if target.approved_for_attack and drone_local_id is not None:
                target.status = "ENGAGED"
                base_response["action"] = "ATTACK"
            elif target.approved_for_attack and drone_local_id is None:
                base_response["action"] = "HOVER"
                base_response["message"] = "Awaiting local representative"
            else:
                target.status = "PENDING"
                base_response["action"] = "HOVER"
        elif target.assigned_drone_port is None:
            base_response["action"] = "TRACK"
            base_response["message"] = "Unassigned"
        else:
            # BUG1 FIX: For RESERVED target, return HOVER to observation-sending drone
            # This allows drone to enter TRACKING mode and reclaim ownership
            if target.ownership_state == OWNERSHIP_RESERVED:
                base_response["action"] = "HOVER"
                base_response["message"] = "Reserved target - reclaiming"
            # MERGE FIX: if registered in drone_local_ids → HOVER (can track target)
            elif drone_port in target.drone_local_ids:
                base_response["action"] = "HOVER"
                base_response["message"] = f"Merged target - tracking {target.assigned_drone_port}'s target"
            else:
                base_response["action"] = "TRACK"
                base_response["owner"] = target.assigned_drone_port
                
        # Log (rate-limited) - Data sent to drone
        sigma = 0.0
        if covariance_filtered is not None and hasattr(covariance_filtered, 'shape'):
            sigma = (covariance_filtered[0, 0] + covariance_filtered[1, 1]) ** 0.5
        SwarmLogger.log("RESPONSE", "Coordinator",
            f"→DRONE_{drone_port}: {base_response['action']} | "
            f"target={target_id} | lat={lat_filtered:.6f} lon={lon_filtered:.6f} | σ={sigma:.1f}m",
            "FILTER")
            
        return base_response

    def _update_group_metadata(self, target, target_id, drone_port, raw_data):
        """Update group-related metadata while preserving per-drone member ids."""
        if not isinstance(raw_data, dict) or not raw_data.get("is_group"):
            return

        old_count = target.group_member_count
        was_group = target.is_group
        target.is_group = True
        incoming_count = int(raw_data.get("group_member_count", old_count or 0) or 0)
        incoming_group_id = raw_data.get("group_id", target.group_id_local)
        if incoming_group_id is not None:
            target.group_id_by_port[drone_port] = incoming_group_id
            target.identity_last_update_time[drone_port] = time.time()
            preferred_port = target.assigned_drone_port or target.owner_port or drone_port
            target.group_id_local = target.group_id_by_port.get(preferred_port, incoming_group_id)

        member_ids = raw_data.get("member_ids") or []
        cleaned_member_ids = []
        for mid in member_ids:
            try:
                cleaned_member_ids.append(int(mid))
            except (TypeError, ValueError):
                continue
        if cleaned_member_ids:
            target.group_member_local_ids[drone_port] = tuple(sorted(set(cleaned_member_ids)))
            target.identity_last_update_time[drone_port] = time.time()
            incoming_count = max(incoming_count, len(target.group_member_local_ids[drone_port]))

        target.group_count_by_port[drone_port] = incoming_count
        target.refresh_group_summary()

        if not was_group:
            SwarmLogger.log("GROUP", "LEADER",
                f"{target_id}: Became GROUP x{target.group_member_count} (GID:{target.group_id_local})",
                "CLUSTER")
        elif old_count != target.group_member_count:
            SwarmLogger.log("GROUP", "LEADER",
                f"{target_id}: Group size changed x{old_count}→x{target.group_member_count}",
                "CLUSTER")

    def _resolve_local_target_id(self, target, drone_port) -> Optional[int]:
        """Return the actionable local tracker id for a drone.

        For singles this is the normal local tracker id. For groups we prefer a
        member id seen by the assigned drone and fall back to any known local id.
        """
        local_id = target.drone_local_ids.get(drone_port)
        if not target.is_group:
            return local_id

        group_members = target.group_member_local_ids.get(drone_port, ())
        if local_id in group_members:
            return local_id
        if group_members:
            return int(group_members[0])
        if local_id is not None:
            return local_id
        return None

    def _dispatch_action(self, drone_port, raw_data):
        """Route action messages from check_mission_updates."""
        msg = raw_data.get("action")
        t_id = raw_data.get("target_id")

        if msg == "REQUEST_LOCK":
            return self.protocol.handle_lock_request(drone_port, t_id, raw_data)
        elif msg == "LOCKED":
            return self.protocol.handle_lock_confirmation(drone_port, t_id)
        elif msg == "ECHO_TARGET":
            return self.protocol.handle_echo_target(drone_port, t_id, raw_data)
        elif msg == "LOCKED_DATA":
            return self.protocol.handle_locked_data(drone_port, t_id, raw_data, self.fusion.ekf_manager)
        elif msg == "ATTACKING":
            return self.protocol.handle_attacking(drone_port, t_id)
        elif msg == "MISMATCH":
            self.protocol.handle_mismatch(drone_port, t_id, self.drone_mismatch_cooldowns)
            return None

    def _build_mission_action(self, drone_port):
        """Action decision for check_mission_updates."""
        if self._is_drone_stopping(drone_port):
            return "STOP", None, None

        # Immutable target check
        for t_id, t in self.targets.items():
            if t.assigned_drone_port == drone_port and t.is_immutable():
                local_id = self._resolve_local_target_id(t, drone_port)
                action = self._immutable_action(t, t_id, local_id)
                self._log_immutable(drone_port, t_id, action[0])
                return action

        # Active target
        active_id = self.ownership.get_active_target(drone_port)
        if active_id and active_id in self.targets:
            t = self.targets[active_id]
            local_id = self._resolve_local_target_id(t, drone_port)

            if local_id is None and t.status not in IMMUTABLE_STATES:
                self.ownership.drone_active_target[drone_port] = None
            else:
                action = self._target_action(t, active_id, local_id, drone_port)
                if action:
                    return action

        # Search for new unassigned target
        if self.ownership.get_active_target(drone_port) is not None:
            return "SEARCH", None, None

        new_id = self._find_direct_reassignment(drone_port)
        if new_id:
            t = self.targets[new_id]
            self.ownership.assign(drone_port, new_id)
            local_id = self._resolve_local_target_id(t, drone_port)
            lat_f, lon_f = self._get_filtered_position(new_id, t)
            return "REASSIGN", new_id, local_id, lat_f, lon_f

        return "SEARCH", None, None

    def _get_filtered_position(self, target_id: str, target) -> Tuple[float, float]:
        """Get filtered position from EKF, or use target values if unavailable.
        
        Returns:
            (lat, lon) tuple - filtered or fallback position
        """
        filtered_pos = self.fusion.ekf_manager.get_target_position(target_id)
        if filtered_pos is not None:
            gps_pos, cov = filtered_pos
            lat_f, lon_f = float(gps_pos[0]), float(gps_pos[1])
            
            # Raw vs filtered diff log (only if significant diff)
            if target.lat is not None and target.lon is not None:
                diff_m = GeoMath.haversine_distance(target.lat, target.lon, lat_f, lon_f)
                if diff_m > 0.5:  # Only log diffs over 0.5m
                    # Calculate sigma from covariance
                    sigma = 0.0
                    if cov is not None and hasattr(cov, 'shape') and cov.shape[0] >= 2:
                        sigma = (cov[0, 0] + cov[1, 1]) ** 0.5
                    SwarmLogger.log("FILTER", "Coordinator",
                        f"{target_id}: RAW→FILTER Δ={diff_m:.1f}m | "
                        f"raw=({target.lat:.6f},{target.lon:.6f}) | "
                        f"filtered=({lat_f:.6f},{lon_f:.6f}) | σ={sigma:.1f}m",
                        "FILTER")
            return lat_f, lon_f
        
        # Fallback: if EKF not yet created
        SwarmLogger.log("FILTER_FALLBACK", "Coordinator",
            f"{target_id}: No EKF, using raw data | lat={target.lat:.6f} lon={target.lon:.6f}",
            "FILTER")
        return target.lat, target.lon

    def _immutable_action(self, target, t_id, local_id):
        """Action for immutable target.
        
        Fix: Filtered position is read from EKF.
        """
        # Get filtered position from EKF
        lat_f, lon_f = self._get_filtered_position(t_id, target)
        
        if target.status in ("CENTERING", "ATTACKING"):
            return "ATTACK", t_id, local_id, lat_f, lon_f
        elif target.status == "CONFIRMED_ATTACK":
            return "CENTER", t_id, local_id, lat_f, lon_f
        elif target.status == "ECHO_WAIT":
            return "ATTACK_ASSIGN", t_id, local_id, lat_f, lon_f
        else:
            return "ATTACK", t_id, local_id, lat_f, lon_f

    def _target_action(self, target, t_id, local_id, drone_port):
        """Determine action based on target status.
        
        Fix: Filtered position is read from EKF.
        """
        # Get filtered position from EKF
        lat_f, lon_f = self._get_filtered_position(t_id, target)
        
        if target.status in ("CENTERING", "ATTACKING"):
            return "ATTACK", t_id, local_id, lat_f, lon_f
        elif target.status == "CONFIRMED_ATTACK":
            return "CENTER", t_id, local_id, lat_f, lon_f
        elif target.status == "ECHO_WAIT":
            return "ATTACK_ASSIGN", t_id, local_id, lat_f, lon_f
        elif target.status == "ENGAGED":
            return "ATTACK", t_id, local_id, lat_f, lon_f
        elif target.status == "PENDING":
            if target.approved_for_attack:
                if target.consecutive_obs >= 12 and target.id_switch_count <= 1:
                    target.status = "ECHO_WAIT"
                    return "ATTACK_ASSIGN", t_id, local_id, lat_f, lon_f
            return "HOVER", t_id, local_id, lat_f, lon_f
        return None

    def _log_immutable(self, drone_port, t_id, action_name):
        """Immutable log rate-limit."""
        now = time.time()
        last_ts = self._immutable_log_ts.get(drone_port, 0)
        last_tid = self._immutable_log_target.get(drone_port, "")
        if now - last_ts >= 1.0 or t_id != last_tid:
            SwarmLogger.log("STATE", "PUBLISH",
                            f"Drone_{drone_port} IMMUTABLE: {action_name} -> Target {t_id}",
                            "ACTION")
            self._immutable_log_ts[drone_port] = now
            self._immutable_log_target[drone_port] = t_id

    def _find_direct_reassignment(self, drone_port):
        """Find nearby unassigned target seen by drone."""
        vehicle = self.drone_manager.drones.get(drone_port)
        if vehicle is None or vehicle == "connecting":
            return None
        try:
            loc = vehicle.location.global_relative_frame
            d_lat, d_lon = loc.lat, loc.lon
        except Exception:
            return None

        best_id = None
        best_dist = float("inf")
        for t_id, t in self.targets.items():
            if t.assigned_drone_port is not None:
                continue
            if t.status == "LOST" or t.track_state != "CONFIRMED":
                continue
            if drone_port not in t.drone_local_ids:
                continue
            if self._conflicts_with_active_family(t_id, t):
                continue
            # Mismatch cooldown check
            if drone_port in self.drone_mismatch_cooldowns:
                last = self.drone_mismatch_cooldowns[drone_port].get(t_id, 0)
                if time.time() - last < SWARM_GEO_COOLDOWN_S:
                    continue
            dist = GeoMath.haversine_distance(d_lat, d_lon, t.lat, t.lon)
            if dist < best_dist:
                best_dist = dist
                best_id = t_id

        return best_id

    def _conflicts_with_active_family(self, candidate_id: str, candidate_target) -> bool:
        """Reject candidates that overlap with an already assigned same-family target."""
        for other_id, other in self.targets.items():
            if other_id == candidate_id:
                continue
            if other.assigned_drone_port is None:
                continue
            if other.status == "LOST" or other.track_state == "DELETED":
                continue
            if candidate_target.is_group != other.is_group:
                continue
            if candidate_target.is_group and abs(
                int(candidate_target.group_member_count or 0) - int(other.group_member_count or 0)
            ) > 1:
                continue
            count_diff = abs(
                int(candidate_target.group_member_count or 0) - int(other.group_member_count or 0)
            ) if candidate_target.is_group else 0
            if count_diff > 0 and not self._targets_share_identity_evidence(candidate_target, other):
                continue
            dist = GeoMath.haversine_distance(candidate_target.lat, candidate_target.lon, other.lat, other.lon)
            if candidate_target.is_group:
                if self._targets_share_identity_evidence(candidate_target, other):
                    exclusion = 12.0 + 2.0 * max(
                        int(candidate_target.group_member_count or 1),
                        int(other.group_member_count or 1),
                    )
                else:
                    exclusion = 10.0
            else:
                exclusion = 10.0
            if dist < exclusion:
                return True
        return False

    # ==================================================================
    # PERIODIC OPTIMIZATION LOOP
    # ==================================================================

    def _periodic_loop(self):
        """Periodic: radar log, EKF batch, owner loss, merge, Hungarian, fusion."""
        SwarmLogger.log("INFO", "LEADER", "Proactive Assignment Loop Started.", "OPT_LOOP")

        while self._loop_running:
            try:
                time.sleep(ASSIGNMENT_LOOP_INTERVAL_S)

                now = time.time()
                self.lifecycle.prune(
                    now, self.attack_mode_active,
                    drone_connected_check=self._is_drone_connected,
                    release_drone=self.ownership.release,
                    remove_fusion_target=self.fusion.ekf_manager.remove_target,
                )

                # --- Root cause fix: Update leader drone reference ---
                self._update_leader_reference()

                # Radar snapshot
                self.battlespace.log_radar_snapshot()

                # EKF batch - collect coasting status (Phase 1)
                with self.lock:
                    coasting_status = {t_id: t.is_coasting for t_id, t in self.targets.items()}
                self.fusion.process_ekf_batch(coasting_status)

                # Owner loss detection
                with self.lock:
                    for t_id, t in list(self.targets.items()):
                        if (t.ownership_state == OWNERSHIP_OWNED
                                and t.owner_port is not None):
                            since = now - t.last_observation_time
                            quality = t.observation_quality.get(t.owner_port, 0)
                            if since > 3.0 and quality > 0:
                                self.ownership.process_owner_loss(t)

                # Duplicate merge (with identity index - drone local IDs updated after merge)
                self.fusion.merge_duplicates(self.ownership, self.identity)

                # Hungarian assignment
                with self.lock:
                    pending = [t_id for t_id, t in self.targets.items()
                               if t.status == "PENDING" and t.assigned_drone_port is None]
                    idle = set(p for p, v in self.drone_manager.drones.items()
                               if v != "connecting") - set(
                        t.assigned_drone_port for t in self.targets.values()
                        if t.assigned_drone_port is not None
                        and t.status in ("PENDING", "ENGAGED")
                    )

                if pending and idle:
                    assignments = self.assign_attack_targets()
                    if assignments:
                        SwarmLogger.log("ASSIGN", "OPT_LOOP",
                                        f"Optimal Assignment: {assignments}", "ASSIGN")

                # Cross-drone fusion
                fused = self.fuse_cross_drone_targets()
                if fused > 0:
                    SwarmLogger.log("FUSION", "OPT_LOOP",
                                    f"{fused} targets merged via fusion.", "FUSION")

            except Exception as e:
                SwarmLogger.log("ERROR", "OPT_LOOP",
                                f"Optimization Error: {e}", "OPT_LOOP")

            # Stale drone cleanup
            try:
                self._prune_stale_drones()
            except Exception as e:
                SwarmLogger.log("ERROR", "OPT_LOOP",
                                f"Heartbeat prune error: {e}", "OPT_LOOP")

    def _prune_stale_drones(self):
        """Release drone assignments with expired heartbeat."""
        now = time.time()
        with self.lock:
            for t_id, t in self.targets.items():
                port = t.assigned_drone_port
                if port is None:
                    continue
                last_hb = self.drone_heartbeat_ts.get(port)
                if last_hb is not None and (now - last_hb) > LEADER_TIMEOUT_S:
                    if ((t.status in IMMUTABLE_STATES)
                            or (t.status == "ENGAGED" and t.approved_for_attack)):
                        if self._is_drone_connected(port):
                            SwarmLogger.log(
                                "GUARD", "LEADER",
                                f"HEARTBEAT BYPASS: {t_id} stays {t.status} despite stale heartbeat "
                                f"({now - last_hb:.1f}s) on Drone_{port}",
                                "FAILSAFE",
                            )
                            continue
                    SwarmLogger.log("FAILSAFE", "LEADER",
                                    f"Drone_{port} heartbeat stale ({now - last_hb:.1f}s). Releasing {t_id}.",
                                    "FAILSAFE")
                    self.ownership.release(port)

    def _update_leader_reference(self):
        """Find leader drone position and set as NED reference for TargetFusionManager.
        
        Root cause fix: Provides a consistent reference point for NED conversions.
        Leader drone is determined as the drone tracking the most active targets.
        """
        leader_port = None
        leader_lat = None
        leader_lon = None
        max_targets = 0
        
        with self.lock:
            # Calculate each drone's active target count
            drone_target_counts: Dict[int, int] = {}
            for t_id, t in self.targets.items():
                if t.assigned_drone_port is not None and t.status in ("ENGAGED", "PENDING"):
                    port = t.assigned_drone_port
                    drone_target_counts[port] = drone_target_counts.get(port, 0) + 1
            
            # Select drone with most targets as leader
            for port, count in drone_target_counts.items():
                if count > max_targets:
                    max_targets = count
                    leader_port = port
            
            # If no active targets, select first connected drone as leader
            if leader_port is None:
                for port, vehicle in self.drone_manager.drones.items():
                    if vehicle != "connecting":
                        leader_port = port
                        break
            
            # Get leader drone position
            if leader_port is not None:
                vehicle = self.drone_manager.drones.get(leader_port)
                if vehicle and vehicle != "connecting":
                    try:
                        loc = vehicle.location.global_relative_frame
                        if loc and loc.lat is not None and loc.lon is not None:
                            leader_lat = loc.lat
                            leader_lon = loc.lon
                    except Exception:
                        pass
        
        # Update reference
        if leader_lat is not None and leader_lon is not None:
            self.fusion.ekf_manager.set_leader_reference(leader_lat, leader_lon)

    def stop(self):
        """Stop periodic loop."""
        self._loop_running = False
        thread = getattr(self, "_loop_thread", None)
        if thread and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=ASSIGNMENT_LOOP_INTERVAL_S * 2.0)
