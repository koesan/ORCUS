"""Leader-side communication facade."""

from __future__ import annotations

import time
from typing import Dict, Optional, Tuple

from config import (
    LEADER_TIMEOUT_S,
    SWARM_GEO_COOLDOWN_S,
    SWARM_MERGE_DISTANCE_M,
    SWARM_TARGET_ACTIONABLE_GRACE_S,
    TRACK_OBSERVER_STALE_TIMEOUT_SEC,
)
from modules.core.comm import LinkSessionPhase
from modules.core.geo_math import GeoMath
from modules.core.logger import SwarmLogger
from modules.swarm.target_fusion import sigma_from_cov
from modules.swarm.target import (
    IMMUTABLE_STATES,
    targets_share_identity_evidence,
    attack_family_exclusion_radius,
)


class LeaderLink:
    """Single leader-side communication surface."""

    def __init__(self, owner):
        self.owner = owner

    def _log_local_id_fallback(self, target, drone_port: int, fallback_local_id: int, reason: str) -> None:
        target_id = getattr(target, "target_id", None) or getattr(target, "id", None)
        SwarmLogger.log(
            "RECOVERY",
            "LEADER",
            f"{target_id}: Local-ID fallback -> {fallback_local_id} for Drone_{drone_port} ({reason})",
            "TRACK",
        )
        SwarmLogger.log_structured(
            "leader_local_id_fallback",
            "TRACK",
            level="RECOVERY",
            drone_id=drone_port,
            target_id=target_id,
            local_track_id=fallback_local_id,
            decision_reason=reason,
            world_estimate=(target.lat, target.lon),
        )

    def next_command_metadata(self, drone_port: int, action: str, target_id=None) -> Dict[str, object]:
        previous_epoch = self.owner._command_epoch_by_drone.get(drone_port, 0)
        if action in {"ASSIGN", "REASSIGN", "STOP"}:
            epoch = previous_epoch + 1
        elif target_id is None and action in {"SEARCH", "HOLD"}:
            epoch = previous_epoch + 1 if previous_epoch == 0 else previous_epoch
        else:
            epoch = previous_epoch if previous_epoch > 0 else 1
        self.owner._command_epoch_by_drone[drone_port] = epoch
        session_id = None
        if action == "ASSIGN":
            session_id = f"{drone_port}:{epoch}:{next(self.owner._session_counter)}"
        else:
            state = self.owner.drone_link_state.get(drone_port)
            if state is not None and state.session.session_id:
                session_id = state.session.session_id
        return {
            "command_id": f"{drone_port}:{epoch}:{next(self.owner._command_counter)}",
            "epoch": epoch,
            "issued_at": time.time(),
            "stale_after_s": max(1.0, LEADER_TIMEOUT_S * 0.5),
            "session_id": session_id,
        }

    def build_command_payload(self, drone_port: int, action_tuple):
        if not isinstance(action_tuple, (tuple, list)) or not action_tuple:
            return action_tuple
        action = action_tuple[0]
        target_id = action_tuple[1] if len(action_tuple) > 1 else None
        local_id = action_tuple[2] if len(action_tuple) > 2 else None
        lat = action_tuple[3] if len(action_tuple) > 3 else None
        lon = action_tuple[4] if len(action_tuple) > 4 else None
        payload = {
            "action": action,
            "target_id": target_id,
            "tracker_id": local_id,
            "local_id": local_id,
            "lat": lat,
            "lon": lon,
        }
        payload.update(self.next_command_metadata(drone_port, action, target_id=target_id))
        self.owner._record_command_publish(
            drone_port,
            action,
            payload.get("command_id"),
            target_id=target_id,
            local_id=local_id,
            session_id=payload.get("session_id"),
        )
        return payload

    def resolve_local_target_id(self, target, drone_port) -> Optional[int]:
        now = time.time()
        local_id = target.drone_local_ids.get(drone_port)
        if not target.is_group:
            if local_id is not None:
                self.owner.remember_actionable_local_id(drone_port, target.id, local_id)
                return local_id
            recent_seen = target.drone_last_seen_time.get(drone_port)
            if (
                (target.approved_for_attack or target.is_in_attack_pipeline())
                and target.status != "PENDING"
                and recent_seen is not None
                and (now - float(recent_seen)) <= TRACK_OBSERVER_STALE_TIMEOUT_SEC
                and target.assigned_drone_port == drone_port
                and target.tracker_id is not None
            ):
                fallback = int(target.tracker_id)
                self._log_local_id_fallback(target, drone_port, fallback, "recent_tracker_id_reuse")
                self.owner.remember_actionable_local_id(drone_port, target.id, fallback)
                return fallback
            if target.approved_for_attack or target.is_in_attack_pipeline():
                cached_local_id = self.owner.get_cached_actionable_local_id(drone_port, target.id)
                if cached_local_id is not None:
                    self._log_local_id_fallback(target, drone_port, cached_local_id, "sticky_actionable_cache")
                    return cached_local_id
            return local_id

        recent_seen = target.drone_last_seen_time.get(drone_port)
        local_is_recent = (
            target.has_recent_identity(drone_port, now, SWARM_TARGET_ACTIONABLE_GRACE_S)
            or (recent_seen is not None and (now - float(recent_seen)) <= TRACK_OBSERVER_STALE_TIMEOUT_SEC)
        )
        if local_id is not None:
            if target.is_in_attack_pipeline():
                self.owner.remember_actionable_local_id(drone_port, target.id, local_id)
                return local_id
            if local_is_recent:
                self.owner.remember_actionable_local_id(drone_port, target.id, local_id)
                return local_id
        if (
            (target.approved_for_attack or target.is_in_attack_pipeline())
            and target.status != "PENDING"
            and recent_seen is not None
            and (now - float(recent_seen)) <= TRACK_OBSERVER_STALE_TIMEOUT_SEC
            and target.assigned_drone_port == drone_port
            and target.tracker_id is not None
        ):
            fallback = int(target.tracker_id)
            self._log_local_id_fallback(target, drone_port, fallback, "recent_tracker_id_reuse")
            self.owner.remember_actionable_local_id(drone_port, target.id, fallback)
            return fallback
        if target.approved_for_attack or target.is_in_attack_pipeline():
            cached_local_id = self.owner.get_cached_actionable_local_id(drone_port, target.id)
            if cached_local_id is not None:
                self._log_local_id_fallback(target, drone_port, cached_local_id, "sticky_actionable_cache")
                return cached_local_id
        if (
            target.is_group
            and target.assigned_drone_port == drone_port
            and recent_seen is not None
            and (now - float(recent_seen)) <= TRACK_OBSERVER_STALE_TIMEOUT_SEC
        ):
            if target.tracker_id is not None:
                fallback = int(target.tracker_id)
                self._log_local_id_fallback(target, drone_port, fallback, "recent_group_tracker_reuse")
                self.owner.remember_actionable_local_id(drone_port, target.id, fallback)
                return fallback
            cached_local_id = self.owner.get_cached_actionable_local_id(drone_port, target.id)
            if cached_local_id is not None:
                self._log_local_id_fallback(target, drone_port, cached_local_id, "recent_group_sticky_cache")
                return cached_local_id
        return None

    def get_filtered_position(self, target_id: str, target) -> Tuple[float, float]:
        filtered_pos = self.owner.fusion.ekf_manager.get_target_position(target_id)
        if filtered_pos is not None:
            gps_pos, cov = filtered_pos
            lat_f, lon_f = float(gps_pos[0]), float(gps_pos[1])
            if target.lat is not None and target.lon is not None:
                diff_m = GeoMath.haversine_distance(target.lat, target.lon, lat_f, lon_f)
                if diff_m > 0.5:
                    sigma = 0.0
                    if cov is not None and hasattr(cov, "shape") and cov.shape[0] >= 2:
                        sigma = (cov[0, 0] + cov[1, 1]) ** 0.5
                    SwarmLogger.log(
                        "FILTER",
                        "Coordinator",
                        f"{target_id}: HAM→FİLTRE Δ={diff_m:.1f}m | ham=({target.lat:.6f},{target.lon:.6f}) | "
                        f"filtreli=({lat_f:.6f},{lon_f:.6f}) | σ={sigma:.1f}m",
                        "FILTER",
                    )
            return lat_f, lon_f

        SwarmLogger.log(
            "FILTER_FALLBACK",
            "Coordinator",
            f"{target_id}: EKF yok, ham veri kullanılıyor | lat={target.lat:.6f} lon={target.lon:.6f}",
            "FILTER",
        )
        return target.lat, target.lon

    def immutable_action(self, target, target_id, local_id):
        lat_f, lon_f = self.get_filtered_position(target_id, target)
        if target.status in ("STAGING", "CENTERING", "EXECUTING"):
            return "EXECUTE", target_id, local_id, lat_f, lon_f
        if target.status == "READY":
            return "VERIFY", target_id, local_id, lat_f, lon_f
        if target.status == "ASSIGNED":
            return "ASSIGN", target_id, local_id, lat_f, lon_f
        return "EXECUTE", target_id, local_id, lat_f, lon_f

    def target_action(self, target, target_id, local_id, drone_port):
        lat_f, lon_f = self.get_filtered_position(target_id, target)
        if target.status in ("STAGING", "CENTERING", "EXECUTING"):
            return "EXECUTE", target_id, local_id, lat_f, lon_f
        if target.status == "READY":
            return "VERIFY", target_id, local_id, lat_f, lon_f
        if target.status == "ASSIGNED":
            return "ASSIGN", target_id, local_id, lat_f, lon_f
        if target.status == "PENDING":
            if target.approved_for_attack and target.consecutive_obs >= 12 and target.id_switch_count <= 1:
                target.status = "ASSIGNED"
                return "ASSIGN", target_id, local_id, lat_f, lon_f
            return "HOVER", target_id, local_id, lat_f, lon_f
        return None

    def conflicts_with_active_family(self, candidate_id: str, candidate_target) -> bool:
        for other_id, other in self.owner.targets.items():
            if other_id == candidate_id:
                continue
            if other.assigned_drone_port is None:
                continue
            if other.status == "LOST" or other.track_state == "DELETED":
                continue
            if candidate_target.is_group != other.is_group:
                continue
            if candidate_target.is_group and abs(int(candidate_target.group_member_count or 0) - int(other.group_member_count or 0)) > 1:
                continue
            count_diff = abs(int(candidate_target.group_member_count or 0) - int(other.group_member_count or 0)) if candidate_target.is_group else 0
            if count_diff > 0 and not targets_share_identity_evidence(candidate_target, other):
                continue
            dist = GeoMath.haversine_distance(candidate_target.lat, candidate_target.lon, other.lat, other.lon)
            if candidate_target.is_group:
                if targets_share_identity_evidence(candidate_target, other):
                    exclusion = 12.0 + 2.0 * max(int(candidate_target.group_member_count or 1), int(other.group_member_count or 1))
                else:
                    exclusion = 10.0
            else:
                exclusion = 10.0
            if dist < exclusion:
                return True
        return False

    def find_direct_reassignment(self, drone_port):
        vehicle = self.owner.drone_manager.drones.get(drone_port)
        if vehicle is None or vehicle == "connecting":
            return None
        try:
            loc = vehicle.location.global_relative_frame
            d_lat, d_lon = loc.lat, loc.lon
        except Exception:
            return None

        best_id = None
        best_dist = float("inf")
        now = time.time()
        for target_id, target in self.owner.targets.items():
            if target.assigned_drone_port is not None:
                continue
            if target.status == "LOST" or target.track_state != "CONFIRMED":
                continue
            local_id = target.drone_local_ids.get(drone_port)
            recent_seen = target.drone_last_seen_time.get(drone_port)
            if local_id is None:
                continue
            if recent_seen is None or (now - float(recent_seen)) > TRACK_OBSERVER_STALE_TIMEOUT_SEC:
                continue
            if self.conflicts_with_active_family(target_id, target):
                continue
            if drone_port in self.owner.drone_mismatch_cooldowns:
                last = self.owner.drone_mismatch_cooldowns[drone_port].get(target_id, 0)
                if time.time() - last < SWARM_GEO_COOLDOWN_S:
                    continue
            dist = GeoMath.haversine_distance(d_lat, d_lon, target.lat, target.lon)
            if dist < best_dist:
                best_dist = dist
                best_id = target_id
        return best_id

    def build_mission_action(self, drone_port):
        owner = self.owner
        if owner._is_drone_stopping(drone_port):
            owner._log_action_decision(drone_port, "STOP", "drone stopping/RTL")
            return "STOP", None, None

        phase = (owner._get_controller_phase(drone_port) or "").upper()
        if phase in {"RTL", "PAUSED", "IDLE", "COMPLETE", "ERROR", "TERMINAL_HOLD"}:
            owner._log_action_decision(drone_port, "STOP", f"controller phase {phase or 'UNKNOWN'}")
            return "STOP", None, None

        recovery_active = owner._drone_in_recovery_window(drone_port)
        if phase in {"ATTACK_ABORT", "RETURN_TO_SCAN"}:
            owner._log_action_decision(drone_port, "SEARCH", f"controller phase {phase}")
            return "SEARCH", None, None

        for target_id, target in owner.targets.items():
            if target.assigned_drone_port == drone_port and target.is_immutable():
                local_id = self.resolve_local_target_id(target, drone_port)
                owner._reserve_attack_corridor(drone_port, target_id, target)
                action = self.immutable_action(target, target_id, local_id)
                if recovery_active and action[0] in {"EXECUTE", "VERIFY", "ASSIGN"}:
                    action = ("HOVER", target_id, local_id, action[3], action[4])
                owner._log_immutable(drone_port, target_id, action[0])
                reason = f"immutable status {target.status}"
                if recovery_active:
                    reason = f"{reason} (recovery window suppresses attack)"
                owner._log_action_decision(drone_port, action[0], reason, target_id, local_id)
                return action

        active_id = owner.ownership.get_active_target(drone_port)
        released_active_target = None
        if active_id and active_id in owner.targets:
            target = owner.targets[active_id]
            local_id = self.resolve_local_target_id(target, drone_port)
            now = time.time()
            if local_id is None and target.status not in IMMUTABLE_STATES:
                recent_seen = target.drone_last_seen_time.get(drone_port)
                if (
                    recent_seen is not None
                    and (now - float(recent_seen)) <= TRACK_OBSERVER_STALE_TIMEOUT_SEC
                ):
                    lat_f, lon_f = self.get_filtered_position(active_id, target)
                    owner._log_action_decision(
                        drone_port,
                        "HOVER",
                        f"active target {active_id} continuity hold",
                        active_id,
                        None,
                    )
                    return "HOVER", active_id, None, lat_f, lon_f
                if owner._has_actionable_grace(target, drone_port, now):
                    lat_f, lon_f = self.get_filtered_position(active_id, target)
                    owner._log_action_decision(drone_port, "HOVER", f"active target {active_id} sticky grace", active_id, None)
                    return "HOVER", active_id, None, lat_f, lon_f
                owner.ownership.release(drone_port)
                released_active_target = active_id
            else:
                owner._reserve_attack_corridor(drone_port, active_id, target)
                action = self.target_action(target, active_id, local_id, drone_port)
                if action:
                    if recovery_active and action[0] in {"EXECUTE", "VERIFY", "ASSIGN"}:
                        action = ("HOVER", active_id, local_id, action[3], action[4])
                    reason = f"target status {target.status}"
                    if recovery_active:
                        reason = f"{reason} (recovery window suppresses attack)"
                    owner._log_action_decision(drone_port, action[0], reason, active_id, local_id)
                    return action

        retained_id = owner.ownership.get_active_target(drone_port)
        if retained_id is not None:
            retained_target = owner.targets.get(retained_id)
            if retained_target is not None and retained_target.lat is not None and retained_target.lon is not None:
                local_id = self.resolve_local_target_id(retained_target, drone_port)
                lat_f, lon_f = self.get_filtered_position(retained_id, retained_target)
                owner._log_action_decision(drone_port, "HOVER", "active target retained pending actionable publish", retained_id, local_id)
                return "HOVER", retained_id, local_id, lat_f, lon_f
            owner._log_action_decision(drone_port, "SEARCH", "active target retained but no actionable publish")
            return "SEARCH", None, None

        new_id = self.find_direct_reassignment(drone_port)
        if new_id:
            target = owner.targets[new_id]
            owner.ownership.assign(drone_port, new_id)
            local_id = self.resolve_local_target_id(target, drone_port)
            lat_f, lon_f = self.get_filtered_position(new_id, target)
            owner._log_action_decision(drone_port, "REASSIGN", "direct reassignment", new_id, local_id)
            return "REASSIGN", new_id, local_id, lat_f, lon_f

        if released_active_target is not None:
            owner._log_action_decision(drone_port, "SEARCH", f"active target {released_active_target} missing local_id")
            return "SEARCH", None, None

        owner._log_action_decision(drone_port, "SEARCH", "no actionable target")
        return "SEARCH", None, None

    def approve_attack(self, target_id: str) -> bool:
        with self.owner.lock:
            target = self.owner.targets.get(target_id)
            if target and target.track_state == "CONFIRMED":
                target.approved_for_attack = True
                local_id = None
                if target.assigned_drone_port is not None:
                    local_id = self.resolve_local_target_id(target, target.assigned_drone_port)
                if target.assigned_drone_port is not None and local_id is not None:
                    target.status = "ASSIGNED"
                elif target.status not in IMMUTABLE_STATES:
                    target.status = "PENDING"
                SwarmLogger.log("APPROVE", "LEADER", f"{target_id} approved for attack", "ATTACK")
                return True
        return False

    def handle_assignment_accept(self, drone_port: int, target_id: str, raw_data: Dict) -> Dict:
        session_id = raw_data.get("session_id")
        with self.owner.lock:
            target_id = self.owner.registry.resolve_alias(target_id)
            if target_id not in self.owner.registry:
                return {"action": "SEARCH", "target_id": None}
            target = self.owner.registry.get(target_id)
            current = self.owner.ownership.get_active_target(drone_port)
            if current is not None and current != target_id:
                SwarmLogger.log("LOCK_BLOCKED", "HANDSHAKE", f"Lock REJECTED: Drone_{drone_port} already has {current}", "LEADER")
                return {"action": "SEARCH", "target_id": None}
            if target.status in IMMUTABLE_STATES and target.assigned_drone_port not in (None, drone_port):
                SwarmLogger.log(
                    "LOCK_BLOCKED",
                    "HANDSHAKE",
                    f"Lock REJECTED: {target_id} in {target.status} by Drone_{target.assigned_drone_port}",
                    "LEADER",
                )
                return {"action": "SEARCH", "target_id": None}
            for other_id, other in self.owner.registry.items():
                if other_id == target_id:
                    continue
                if other.is_group != target.is_group:
                    continue
                if target.is_group:
                    count_diff = abs(int(other.group_member_count or 0) - int(target.group_member_count or 0))
                    if count_diff > 1:
                        continue
                    if count_diff > 0 and not targets_share_identity_evidence(target, other):
                        continue
                other_owner = other.assigned_drone_port
                if other_owner in (None, drone_port):
                    continue
                if other.status not in ("VERIFYING", "STAGING", "CENTERING", "EXECUTING", "READY", "DIVING"):
                    continue
                dist = GeoMath.haversine_distance(target.lat, target.lon, other.lat, other.lon)
                if dist < attack_family_exclusion_radius(target, other):
                    SwarmLogger.log(
                        "WARNING",
                        "HANDSHAKE",
                        f"Lock REJECTED: {target_id} too close ({dist:.1f}m) to active {other_id} by Drone_{other_owner}",
                        "LEADER",
                    )
                    return {"action": "SEARCH", "target_id": None}
            self.owner.ownership.assign(drone_port, target_id)
            local_id = self.resolve_local_target_id(target, drone_port)
            self.owner._record_session_event(
                drone_port,
                LinkSessionPhase.ACKED,
                target_id=target_id,
                local_id=local_id,
                session_id=session_id,
            )
            SwarmLogger.log("INFO", "HANDSHAKE", f"Lock APPROVED: {target_id} by Drone_{drone_port}", "LEADER")
            return {
                "action": "ASSIGNMENT_ACCEPTED",
                "target_id": target_id,
                "local_id": local_id,
                "session_id": self.owner.drone_link_state.get(drone_port).session.session_id,
            }

    def handle_verify_ready(self, drone_port: int, target_id: str) -> Dict:
        with self.owner.lock:
            target_id = self.owner.registry.resolve_alias(target_id)
            target = self.owner.registry.get(target_id)
            if target and target.assigned_drone_port == drone_port:
                target.status = "VERIFYING"
                self.owner._record_session_event(
                    drone_port,
                    LinkSessionPhase.VERIFYING,
                    target_id=target_id,
                    local_id=self.resolve_local_target_id(target, drone_port),
                )
                SwarmLogger.log("INFO", "HANDSHAKE", f"{target_id} VERIFYING by Drone_{drone_port}", "LEADER")
                return {
                    "action": "ACK",
                    "target_id": target_id,
                    "session_id": self.owner.drone_link_state.get(drone_port).session.session_id,
                }
        return {"action": "SEARCH", "target_id": None}

    def handle_lock_report(self, drone_port: int, target_id: str, raw_data: Dict):
        """Verify a lock against the leader-side target reference."""
        _VERIFY_THRESHOLD_M = 15.0
        _VERIFY_THRESHOLD_MAX_M = 30.0
        session_id = raw_data.get("session_id")
        locked_lat = raw_data.get("locked_lat")
        locked_lon = raw_data.get("locked_lon")

        with self.owner.lock:
            target_id = self.owner.registry.resolve_alias(target_id)
            target = self.owner.registry.get(target_id)
            if not target:
                return {"action": "VERIFY_REJECTED", "target_id": target_id, "reason": "target_not_found"}
            if target.assigned_drone_port != drone_port:
                return {"action": "VERIFY_REJECTED", "target_id": target_id, "reason": "wrong_drone"}

            # Conflict check: reject same-family conflicts, but do not churn to a new
            # target during VERIFY. Reassignment here caused drones to bounce between
            # targets before ever entering autonomous attack.
            for other_id, other in self.owner.registry.items():
                if other_id == target_id:
                    continue
                if other.status == "EXECUTING" and other.assigned_drone_port is not None:
                    if other.assigned_drone_port != drone_port:
                        dist = GeoMath.haversine_distance(target.lat, target.lon, other.lat, other.lon)
                        if dist < SWARM_MERGE_DISTANCE_M:
                            return {"action": "VERIFY_REJECTED", "target_id": target_id, "reason": "conflict"}

            # Position-based verification
            filtered_pos = self.owner.fusion.ekf_manager.get_target_position(target_id)
            filtered_cov = None
            if filtered_pos is not None:
                gps_pos, filtered_cov = filtered_pos
                target_lat, target_lon = float(gps_pos[0]), float(gps_pos[1])
            else:
                target_lat, target_lon = target.lat, target.lon
            if locked_lat is not None and locked_lon is not None:
                lock_dist = GeoMath.haversine_distance(locked_lat, locked_lon, target_lat, target_lon)
            else:
                lock_dist = 0.0

            verify_threshold = _VERIFY_THRESHOLD_M
            recent_seen = float(target.drone_last_seen_time.get(drone_port, 0.0) or 0.0)
            same_drone_fresh = recent_seen > 0.0 and (time.time() - recent_seen) <= TRACK_OBSERVER_STALE_TIMEOUT_SEC
            sigma = sigma_from_cov(filtered_cov if filtered_cov is not None else target.covariance) or 0.0
            if same_drone_fresh and sigma > (_VERIFY_THRESHOLD_M * 0.5):
                verify_threshold = min(
                    _VERIFY_THRESHOLD_MAX_M,
                    max(_VERIFY_THRESHOLD_M, sigma * 1.5),
                )

            if lock_dist > verify_threshold:
                SwarmLogger.log("REJECT", "LEADER",
                                f"{target_id}: VERIFY_REJECTED for Drone_{drone_port} | "
                                f"lock_dist={lock_dist:.1f}m > {verify_threshold:.1f}m", "ATTACK")
                return {"action": "VERIFY_REJECTED", "target_id": target_id, "reason": "position_mismatch"}

            # APPROVED — set target to EXECUTING (autonomous mode)
            target.status = "EXECUTING"
            local_id = self.resolve_local_target_id(target, drone_port)
            self.owner._record_session_event(
                drone_port,
                LinkSessionPhase.CLOSED,
                target_id=target_id,
                local_id=local_id,
                session_id=session_id,
            )
            SwarmLogger.log("SUCCESS", "LEADER",
                            f"{target_id}: VERIFY_APPROVED for Drone_{drone_port} | "
                            f"lock_dist={lock_dist:.1f}m — entering autonomous mode", "ATTACK")
            return {
                "action": "VERIFY_APPROVED",
                "target_id": target_id,
                "local_id": local_id,
                "target_lat": target_lat,
                "target_lon": target_lon,
            }

    def handle_execution_update(self, drone_port: int, target_id: str, raw_data: Dict, fusion_manager):
        """Handle autonomous telemetry from drone during attack.

        Updates target observation times and feeds GPS to fusion EKF.
        Returns ACK only — no commands during autonomous mode.
        """
        with self.owner.lock:
            target_id = self.owner.registry.resolve_alias(target_id)
            target = self.owner.registry.get(target_id)
            if not target or target.assigned_drone_port != drone_port:
                return {"action": "ACK", "target_id": target_id}

            target_gps = raw_data.get("target_gps")
            if target_gps and target_gps[0] is not None:
                try:
                    import numpy as np
                    fusion_manager.add_observation(
                        target_id,
                        (target_gps[0], target_gps[1], 0.0),
                        np.eye(3) * 4.0, drone_port)
                except Exception as exc:
                    SwarmLogger.log("ERROR", "Protocol",
                                    f"Autonomous update fusion error: {exc}", "FUSION")
                now = time.time()
                target.last_seen_time = now
                target.last_observation_time = now

        return {"action": "ACK", "target_id": target_id}

    def handle_execution_start(self, drone_port: int, target_id: str):
        with self.owner.lock:
            target = self.owner.registry.get(target_id)
            if target and target.assigned_drone_port == drone_port:
                target.status = "EXECUTING"
                self.owner._record_session_event(
                    drone_port,
                    LinkSessionPhase.CLOSED,
                    target_id=target_id,
                )
        return {
            "action": "EXECUTE",
            "target_id": target_id,
            "session_id": self.owner.drone_link_state.get(drone_port).session.session_id if drone_port in self.owner.drone_link_state else None,
        }

    def handle_impact_confirmed(self, drone_port: int, target_id: str, raw_data: Dict):
        with self.owner.lock:
            target_id = self.owner.registry.resolve_alias(target_id)
            target = self.owner.registry.get(target_id)
            if not target:
                return {"action": "ACK", "target_id": target_id}
            if target.assigned_drone_port not in (None, drone_port):
                return {"action": "ACK", "target_id": target_id}
            coverage = float(raw_data.get("coverage", 0.0) or 0.0)
            altitude = float(raw_data.get("altitude", 0.0) or 0.0)
            
            # Store completed mission for UI map plotting
            if hasattr(self.owner, "completed_missions") and target.lat and target.lon:
                self.owner.completed_missions.append({
                    "target_id": target_id,
                    "lat": target.lat,
                    "lon": target.lon,
                    "drone_port": drone_port,
                    "ts": time.time(),
                    "coverage": coverage,
                    "altitude": altitude,
                    "reason": str(raw_data.get("reason", "") or ""),
                })
                
            target.status = "DESTROYED"
            target.track_state = "DELETED"
            target.approved_for_attack = False
            self.owner.ownership.release(drone_port)
            self.owner.registry.remove(target_id)
            SwarmLogger.log(
                "SUCCESS",
                "LEADER",
                f"{target_id}: IMPACT_CONFIRMED by Drone_{drone_port} | coverage={coverage:.1f}% alt={altitude:.1f}m — finalizing target",
                "ATTACK",
            )
            return {"action": "ACK", "target_id": target_id}

    def handle_mismatch(self, drone_port: int, target_id: str, mismatch_cooldowns: Dict):
        SwarmLogger.log("MISMATCH", "LEADER", f"Drone_{drone_port} rejected {target_id} (Geo-Validation fail)", "PROTOCOL")
        target = self.owner.registry.get(target_id)
        if target and target.assigned_drone_port == drone_port:
            self.owner.ownership.release(drone_port)
        mismatch_cooldowns.setdefault(drone_port, {})[target_id] = time.time()

    def dispatch_inbound(self, drone_port: int, raw_data):
        """Route inbound drone messages to appropriate handler."""
        msg = raw_data.get("action")
        target_id = raw_data.get("target_id")
        if msg == "ASSIGNMENT_ACCEPT":
            return self.handle_assignment_accept(drone_port, target_id, raw_data)
        if msg == "VERIFY_READY":
            return self.handle_verify_ready(drone_port, target_id)
        if msg == "LOCK_REPORT":
            return self.handle_lock_report(drone_port, target_id, raw_data)
        if msg in ("EXECUTION_UPDATE", "AUTONOMOUS_UPDATE"):
            return self.handle_execution_update(drone_port, target_id, raw_data, self.owner.fusion.ekf_manager)
        if msg == "MISSION_EVENT" and raw_data.get("event") == "execution_start":
            return self.handle_execution_start(drone_port, target_id)
        if msg == "IMPACT_CONFIRMED":
            return self.handle_impact_confirmed(drone_port, target_id, raw_data)
        if msg == "MISMATCH":
            self.handle_mismatch(drone_port, target_id, self.owner.drone_mismatch_cooldowns)
            self.owner._mark_drone_recovering(drone_port, f"mismatch on {target_id}")
            return None
        return None
