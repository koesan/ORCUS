"""7-step attack protocol handlers."""

import time
from typing import Dict, Optional, Tuple

from modules.core.logger import SwarmLogger
from modules.core.geo_math import GeoMath
from modules.swarm.target import TrackedTarget, TargetRegistry, IMMUTABLE_STATES
from config import SWARM_MERGE_DISTANCE_M


class AttackProtocol:
    """Drone-leader 7-step attack handshake protocol.

    Flow:
        1. Leader sends ATTACK_ASSIGN
        2. Drone sends REQUEST_LOCK
        3. Leader returns ATTACK_APPROVED / SEARCH
        4. Drone confirms LOCKED
        5. Drone sends ECHO_TARGET
        6. Leader cross-validates → ATTACK_CONFIRMED / SEARCH
        7. Drone reports ATTACKING (dive — fire-and-forget)
    """

    def __init__(self, registry: TargetRegistry, ownership_mgr, assignment_eng):
        self._registry = registry
        self._ownership = ownership_mgr
        self._assignment = assignment_eng

    # ------------------------------------------------------------------
    # Step 2: Lock request
    # ------------------------------------------------------------------

    def handle_lock_request(self, drone_port: int, target_id: str, raw_data: Dict) -> Dict:
        """Process drone lock request."""
        with self._registry.lock:
            # Alias resolution
            target_id = self._registry.resolve_alias(target_id)

            if target_id not in self._registry:
                return {"action": "SEARCH", "target_id": None}

            t = self._registry.get(target_id)

            # Is drone already locked to another target?
            current = self._ownership.get_active_target(drone_port)
            current = self._ownership.get_active_target(drone_port)
            if current is not None and current != target_id:
                SwarmLogger.log(
                    "LOCK_BLOCKED", "HANDSHAKE",
                    f"Lock REJECTED: Drone_{drone_port} already has {current}",
                    "LEADER",
                )
                return {"action": "SEARCH", "target_id": None}

            # Is target in attack pipeline by another drone? (IMMUTABLE guard)
            # Check ALL attack states instead of just "LOCKED" — prevents race condition
            if (
                t.status in IMMUTABLE_STATES
                and t.assigned_drone_port is not None
                and t.assigned_drone_port != drone_port
            ):
                SwarmLogger.log(
                    "LOCK_BLOCKED", "HANDSHAKE",
                    f"Lock REJECTED: {target_id} in {t.status} by Drone_{t.assigned_drone_port}",
                    "LEADER",
                )
                return {"action": "SEARCH", "target_id": None}

            # Ghost check — is there an active/in-attack target nearby?
            for other_id, other in self._registry.items():
                if other_id == target_id:
                    continue
                if other.is_group != t.is_group:
                    continue
                if t.is_group:
                    count_diff = abs(int(other.group_member_count or 0) - int(t.group_member_count or 0))
                    if count_diff > 1:
                        continue
                    if count_diff > 0 and not self._targets_share_identity_evidence(t, other):
                        continue
                other_owner = other.assigned_drone_port
                if other_owner in (None, drone_port):
                    continue
                if other.status not in ("LOCKED", "CENTERING", "ATTACKING", "CONFIRMED_ATTACK", "ENGAGED", "DIVING"):
                    continue
                dist = GeoMath.haversine_distance(t.lat, t.lon, other.lat, other.lon)
                if dist < self._family_exclusion_radius(t, other):
                    SwarmLogger.log(
                        "WARN", "HANDSHAKE",
                        f"Lock REJECTED: {target_id} too close ({dist:.1f}m) to active {other_id} by Drone_{other_owner}",
                        "LEADER",
                    )
                    return {"action": "SEARCH", "target_id": None}

            # Approve
            self._ownership.assign(drone_port, target_id)
            local_id = t.drone_local_ids.get(drone_port)
            if t.is_group and local_id is None:
                members = t.group_member_local_ids.get(drone_port, ())
                local_id = int(members[0]) if members else None
            SwarmLogger.log(
                "INFO", "HANDSHAKE",
                f"Lock APPROVED: {target_id} by Drone_{drone_port}",
                "LEADER",
            )
            return {"action": "ATTACK_APPROVED", "target_id": target_id, "local_id": local_id}

    # ------------------------------------------------------------------
    # Step 3b: Lock confirmation
    # ------------------------------------------------------------------

    def handle_lock_confirmation(self, drone_port: int, target_id: str) -> Dict:
        """Process drone's LOCKED confirmation."""
        with self._registry.lock:
            target_id = self._registry.resolve_alias(target_id)
            t = self._registry.get(target_id)

            if t and t.assigned_drone_port == drone_port:
                t.status = "LOCKED"
                SwarmLogger.log("INFO", "HANDSHAKE",
                                f"{target_id} LOCKED by Drone_{drone_port}", "LEADER")
                return {"action": "ACK", "target_id": target_id}

        return {"action": "SEARCH", "target_id": None}

    # ------------------------------------------------------------------
    # Step 2-5: Echo validation
    # ------------------------------------------------------------------

    def handle_echo_target(self, drone_port: int, target_id: str, raw_data: Dict):
        """Echo validation — cross-check and ghost detection."""
        with self._registry.lock:
            t = self._registry.get(target_id)
            if not t:
                SwarmLogger.log("PROTOCOL", f"DRONE_{drone_port}",
                                f"ECHO REJECTED: {target_id} no longer exists", "LEADER")
                return "SEARCH", None, None

            if t.assigned_drone_port != drone_port:
                SwarmLogger.log("PROTOCOL", f"DRONE_{drone_port}",
                                f"ECHO REJECTED: {target_id} assigned to Drone_{t.assigned_drone_port}",
                                "LEADER")
                return "SEARCH", None, None

            drone_gps = raw_data.get("drone_gps")
            local_tracker_id = raw_data.get("local_tracker_id")
            SwarmLogger.log("PROTOCOL", f"DRONE_{drone_port}",
                            f"ECHO received: {target_id} | GPS={drone_gps} | LocalID={local_tracker_id}",
                            "LEADER")

            # Ghost check — is there an active attack nearby?
            for other_id, other in self._registry.items():
                if other_id == target_id:
                    continue
                if other.status in ("CENTERING", "ATTACKING", "CONFIRMED_ATTACK"):
                    dist = GeoMath.haversine_distance(t.lat, t.lon, other.lat, other.lon)
                    if dist < SWARM_MERGE_DISTANCE_M:
                        SwarmLogger.log("PROTOCOL", f"DRONE_{drone_port}",
                                        f"ECHO REJECTED: {target_id} ghost ({dist:.1f}m to {other_id})",
                                        "LEADER")
                        # Find alternative target
                        alt = self._assignment.find_nearest_unassigned(drone_port, exclude_id=target_id)
                        if alt:
                            alt_t = self._registry.get(alt)
                            self._ownership.release(drone_port)
                            self._ownership.assign(drone_port, alt)
                            alt_t.status = "ECHO_WAIT"
                            alt_t.approved_for_attack = True
                            alt_local = alt_t.drone_local_ids.get(drone_port)
                            return "REASSIGN", alt, alt_local
                        else:
                            self._ownership.release(drone_port)
                            return "SEARCH", None, None

            # Does the same drone have another active attack?
            for other_id, other in self._registry.items():
                if other_id == target_id:
                    continue
                if (other.assigned_drone_port == drone_port
                        and other.status in ("CENTERING", "ATTACKING", "CONFIRMED_ATTACK")):
                    SwarmLogger.log("PROTOCOL", f"DRONE_{drone_port}",
                                    f"ECHO REJECTED: already attacking {other_id}", "LEADER")
                    self._ownership.release(drone_port)
                    return "SEARCH", None, None

            # All checks passed — approve
            t.status = "CONFIRMED_ATTACK"
            local_id = t.drone_local_ids.get(drone_port)
            if t.is_group and local_id is None:
                members = t.group_member_local_ids.get(drone_port, ())
                local_id = int(members[0]) if members else None
            SwarmLogger.log("PROTOCOL", f"DRONE_{drone_port}",
                            f"ATTACK CONFIRMED: {target_id} → CENTER", "LEADER")
            return "ATTACK_CONFIRMED", target_id, local_id

    # ------------------------------------------------------------------
    # LOCKED_DATA — position update during attack
    # ------------------------------------------------------------------

    def handle_locked_data(self, drone_port: int, target_id: str, raw_data: Dict, fusion_manager=None):
        """Update target position during attack.
        
        FIX: Raw target_gps is not written directly to t.lat/t.lon.
        Instead, it's added to EKF via fusion_manager (TargetFusionManager).
        Filtered position is already updated via periodic process_ekf_batch.
        """
        with self._registry.lock:
            t = self._registry.get(target_id)
            if not t or t.assigned_drone_port != drone_port:
                return "SEARCH", None, None

            target_gps = raw_data.get("target_gps")
            if target_gps and target_gps[0] is not None:
                # Add raw data to EKF (if fusion_manager exists)
                if fusion_manager is not None:
                    try:
                        import numpy as np
                        obs_cov = np.eye(3) * 4.0  # Default covariance
                        fusion_manager.add_observation(
                            target_id, (target_gps[0], target_gps[1], 0.0), obs_cov, drone_port
                        )
                        SwarmLogger.log("FUSION_DETAIL", "Protocol",
                            f"{target_id}: LOCKED_DATA → added to EKF | raw=({target_gps[0]:.6f}, {target_gps[1]:.6f})",
                            "FUSION")
                    except Exception as e:
                        SwarmLogger.log("ERROR", "Protocol",
                            f"handle_locked_data fusion error: {e}", "FUSION")
                else:
                    # Fallback: Old behavior (if no fusion_manager)
                    t.lat, t.lon = target_gps[0], target_gps[1]
                
                now = time.time()
                t.last_seen_time = now
                t.last_observation_time = now

        return "ATTACK", target_id, None

    # ------------------------------------------------------------------
    # Step 7: Dive notification
    # ------------------------------------------------------------------

    def handle_attacking(self, drone_port: int, target_id: str):
        """Drone reports entering dive — fire-and-forget."""
        with self._registry.lock:
            t = self._registry.get(target_id)
            if t and t.assigned_drone_port == drone_port:
                t.status = "ATTACKING"
                SwarmLogger.log("ATTACK_FINAL", f"DRONE_{drone_port}",
                                f"{target_id} | DIVING — autonomous kamikaze", "LEADER")
        return "ATTACK", target_id, None

    # ------------------------------------------------------------------
    # Batch attack approval
    # ------------------------------------------------------------------

    def approve_attack(self, target_id: str) -> bool:
        """Single target attack approval."""
        with self._registry.lock:
            t = self._registry.get(target_id)
            if t and t.track_state == "CONFIRMED":
                t.approved_for_attack = True
                t.status = "ENGAGED"
                SwarmLogger.log("APPROVE", "LEADER",
                                f"{target_id} approved for attack", "ATTACK")
                return True
        return False

    def approve_all(self):
        """Approve all CONFIRMED targets."""
        with self._registry.lock:
            for t_id, t in self._registry.items():
                if t.track_state == "CONFIRMED" and not t.approved_for_attack:
                    t.approved_for_attack = True
                    t.status = "ENGAGED"
                    SwarmLogger.log("APPROVE", "LEADER",
                                    f"{t_id} approved (batch)", "ATTACK")

    # ------------------------------------------------------------------
    # MISMATCH handling
    # ------------------------------------------------------------------

    def handle_mismatch(self, drone_port: int, target_id: str,
                        mismatch_cooldowns: Dict):
        """Handle geo-validation mismatch."""
        SwarmLogger.log("MISMATCH", "LEADER",
                        f"Drone_{drone_port} rejected {target_id} (Geo-Validation fail)",
                        "PROTOCOL")
        t = self._registry.get(target_id)
        if t and t.assigned_drone_port == drone_port:
            self._ownership.release(drone_port)
        if drone_port not in mismatch_cooldowns:
            mismatch_cooldowns[drone_port] = {}
        mismatch_cooldowns[drone_port][target_id] = time.time()

    @staticmethod
    def _targets_share_identity_evidence(t1, t2) -> bool:
        """Direct overlap evidence only; group-id labels are not reliable enough."""
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

    @staticmethod
    def _family_exclusion_radius(t1, t2) -> float:
        """Conservative exclusion radius for same-family attack locking."""
        if getattr(t1, "is_group", False) and getattr(t2, "is_group", False):
            avg_members = (int(t1.group_member_count or 1) + int(t2.group_member_count or 1)) / 2.0
            return max(SWARM_MERGE_DISTANCE_M, 14.0 + 4.0 * avg_members)
        return max(SWARM_MERGE_DISTANCE_M, 12.0)
