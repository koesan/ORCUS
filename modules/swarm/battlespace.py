"""Radar, UI data provider and log management."""

import time
from typing import Dict, Optional, Tuple

from modules.core.logger import SwarmLogger
from modules.core.geo_math import GeoMath
from modules.swarm.target import TrackedTarget, TargetRegistry


class BattlespaceView:
    """Provides target and drone state data for UI/radar."""

    def __init__(self, registry: TargetRegistry, drone_manager, fusion_engine=None):
        self._registry = registry
        self._drone_manager = drone_manager
        self._fusion_engine = fusion_engine
        # Radar log rate-limit
        self._radar_log_last_ts = 0.0
        self._radar_log_interval = 3.0
        # Position drift tracking: tid → (last_lat, last_lon)
        self._prev_positions: Dict[str, tuple] = {}
        # UI-only smoothing: keeps radar stable without affecting control logic
        self._display_positions: Dict[str, Tuple[float, float]] = {}
        self._display_alpha = 0.35

    # ------------------------------------------------------------------
    # Web UI — main state object
    # ------------------------------------------------------------------

    def get_state(self, ownership_mgr=None) -> Dict:
        """Return full battlespace state for Web UI."""
        now = time.time()

        # Clean up expired aliases
        self._registry.cleanup_aliases()

        with self._registry.lock:
            state = {"targets": {}, "drones": {}}

            # Leader detection (lowest port)
            active_ports = sorted(
                p for p in self._drone_manager.drones
                if self._drone_manager.drones[p] != "connecting"
            )
            leader_port = active_ports[0] if active_ports else None

            # Targets (excluding LOST)
            for t_id, t in self._iter_display_targets():

                cov_list = t.covariance.tolist() if hasattr(t.covariance, "tolist") else t.covariance
                disp_lat, disp_lon = self._get_display_position(t_id, t)

                # Leader distance
                dist_leader = -1
                if leader_port and leader_port in self._drone_manager.drones:
                    try:
                        veh = self._drone_manager.drones[leader_port]
                        loc = veh.location.global_relative_frame
                        if loc:
                            dist_leader = GeoMath.haversine_distance(loc.lat, loc.lon, disp_lat, disp_lon)
                    except Exception:
                        pass

                state["targets"][t_id] = {
                    "lat": disp_lat,
                    "lon": disp_lon,
                    "status": t.status,
                    "track_state": t.track_state,
                    "assigned_to": t.assigned_drone_port,
                    "age": now - t.first_seen_time,
                    "observation_count": t.observation_count,
                    "seen_by_drones": list(t.drone_local_ids.keys()),
                    "local_ids": t.drone_local_ids,
                    "covariance": cov_list,
                    "distance_to_leader": dist_leader,
                    "is_group": t.is_group,
                    "group_member_count": t.group_member_count,
                }

            # Drones
            for port, vehicle in self._drone_manager.drones.items():
                if vehicle == "connecting":
                    continue
                try:
                    loc = vehicle.location.global_relative_frame
                    if not loc:
                        continue
                except Exception:
                    continue

                controller = self._drone_manager.drone_controllers.get(port)
                action = "IDLE"
                engaged_tid = None
                current_tid = None

                if controller:
                    raw_tid = getattr(controller, "current_target_id", None)
                    if raw_tid:
                        current_tid = self._registry.resolve_alias(raw_tid)
                    if controller.tracking_mode:
                        action = "TRACKING"
                    elif controller.is_mission_active:
                        action = "MISSION"

                    if getattr(controller, "attack_approved", False):
                        action = "ENGAGING"
                        engaged_tid = current_tid

                role = "LEADER" if port == leader_port else "FOLLOWER"

                state["drones"][port] = {
                    "lat": loc.lat,
                    "lon": loc.lon,
                    "alt": loc.alt,
                    "heading": vehicle.heading,
                    "action": action,
                    "role": role,
                    "engaged_target_id": engaged_tid,
                    "current_target_id": current_tid,
                    "lock_status": getattr(controller, "lock_status", None),
                }

            return state

    def reset_runtime_state(self):
        """Clear UI/radar caches without touching live drone connections."""
        self._radar_log_last_ts = 0.0
        self._prev_positions.clear()
        self._display_positions.clear()

    # ------------------------------------------------------------------
    # Radar log (rate-limited) — enriched with σ, drift, drone state
    # ------------------------------------------------------------------

    def log_radar_snapshot(self):
        """Log all targets and drone states — rate-limited.
        
        Enhanced: Camera vs Radar inconsistency analysis, group comparison.
        """
        now = time.time()
        if now - self._radar_log_last_ts < self._radar_log_interval:
            return
        self._radar_log_last_ts = now

        with self._registry.lock:
            if len(self._registry) == 0:
                return

            # --- Statistics collection ---
            total_targets = 0
            confirmed_targets = 0
            pending_targets = 0
            group_targets = 0
            total_group_members = 0
            seen_by_multi_drone = 0
            
            summaries = []
            for t_id, t in self._iter_display_targets():
                disp_lat, disp_lon = self._get_display_position(t_id, t)
                    
                total_targets += 1
                if t.track_state == "CONFIRMED":
                    confirmed_targets += 1
                elif t.track_state == "TENTATIVE":
                    pending_targets += 1
                    
                if t.is_group:
                    group_targets += 1
                    total_group_members += t.group_member_count
                    
                if len(t.drone_local_ids) > 1:
                    seen_by_multi_drone += 1

                local_ids = ",".join(
                    f"{p}:{lid}" for p, lid in t.drone_local_ids.items()
                ) if t.drone_local_ids else "-"

                lat_s = f"{disp_lat:.6f}" if disp_lat is not None else "-"
                lon_s = f"{disp_lon:.6f}" if disp_lon is not None else "-"
                state = f"{t.track_state}/{t.status}/{t.ownership_state}"
                assigned = f"D{t.assigned_drone_port}" if t.assigned_drone_port else "FREE"
                stab = f"obs={t.observation_count} consec={t.consecutive_obs} sw={t.id_switch_count}"

                group_s = ""
                if t.is_group:
                    group_s = f" GRP[{t.group_id_local}]x{t.group_member_count}"

                # Covariance uncertainty (position σ in meters)
                try:
                    import numpy as np
                    cov = t.covariance
                    if hasattr(cov, 'shape') and cov.shape[0] >= 2:
                        sigma = (cov[0, 0] + cov[1, 1]) ** 0.5
                    else:
                        sigma = 0.0
                except Exception:
                    sigma = 0.0

                # Position drift since last snapshot
                drift_s = ""
                prev = self._prev_positions.get(t_id)
                if prev and disp_lat is not None and disp_lon is not None:
                    try:
                        drift_m = GeoMath.haversine_distance(prev[0], prev[1], disp_lat, disp_lon)
                        if drift_m > 0.5:  # Only show meaningful drift
                            drift_s = f" Δ={drift_m:.1f}m"
                    except Exception:
                        pass
                if disp_lat is not None and disp_lon is not None:
                    self._prev_positions[t_id] = (disp_lat, disp_lon)

                summaries.append(
                    f"{t_id}[{state}] @{lat_s},{lon_s} | "
                    f"Local:[{local_ids}] | {assigned} | {stab}{group_s} | "
                    f"σ={sigma:.1f}m{drift_s}"
                )

            if summaries:
                SwarmLogger.log(
                    "RADAR", "LEADER",
                    f"TARGETS({len(summaries)}): " + " | ".join(summaries),
                    "BATTLESPACE",
                )

            # --- SUMMARY STATISTICS (for camera comparison) ---
            SwarmLogger.log(
                "RADAR_DETAIL", "LEADER",
                f"RADAR_STATS: Total={total_targets} | "
                f"Confirmed={confirmed_targets} Pending={pending_targets} | "
                f"Groups={group_targets} (total_members={total_group_members}) | "
                f"MultiDroneSeen={seen_by_multi_drone}",
                "BATTLESPACE",
            )

            # Drone state summary line
            drone_parts = []
            for port, vehicle in self._drone_manager.drones.items():
                if vehicle == "connecting":
                    continue
                try:
                    loc = vehicle.location.global_relative_frame
                    if not loc:
                        continue
                    alt = loc.alt
                except Exception:
                    continue

                controller = self._drone_manager.drone_controllers.get(port)
                action = "IDLE"
                engaged = "-"
                current = "-"
                if controller:
                    raw_tid = getattr(controller, "current_target_id", None)
                    if raw_tid:
                        current = self._registry.resolve_alias(raw_tid) or raw_tid
                    if getattr(controller, "attack_approved", False):
                        action = "ENGAGING"
                        engaged = current
                    elif controller.tracking_mode:
                        action = "TRACKING"
                    elif controller.is_mission_active:
                        action = "MISSION"

                shown_target = engaged if engaged != "-" else current
                drone_parts.append(f"D{port}[{action}] alt={alt:.1f}m tgt={shown_target}")

            if drone_parts:
                SwarmLogger.log(
                    "RADAR", "LEADER",
                    f"DRONES({len(drone_parts)}): " + " | ".join(drone_parts),
                    "BATTLESPACE",
                )

    def _get_display_position(self, target_id: str, target) -> Tuple[Optional[float], Optional[float]]:
        """Return a UI/radar-safe position.

        Uses EKF-filtered coordinates if available, then applies a light
        presentation-only low-pass filter so radar does not jump between raw
        and filtered states inside the same update cycle.
        """
        lat = target.lat
        lon = target.lon

        if self._fusion_engine is not None:
            try:
                filtered = self._fusion_engine.ekf_manager.get_target_position(target_id)
                if filtered is not None:
                    gps_pos, _cov = filtered
                    lat = float(gps_pos[0])
                    lon = float(gps_pos[1])
            except Exception:
                pass

        if lat is None or lon is None:
            return lat, lon

        prev = self._display_positions.get(target_id)
        if prev is None:
            self._display_positions[target_id] = (lat, lon)
            return lat, lon

        alpha = self._display_alpha
        smooth_lat = prev[0] + alpha * (lat - prev[0])
        smooth_lon = prev[1] + alpha * (lon - prev[1])
        self._display_positions[target_id] = (smooth_lat, smooth_lon)
        return smooth_lat, smooth_lon

    def _iter_display_targets(self):
        """Yield radar-visible targets after conservative duplicate suppression."""
        candidates = []
        for t_id, t in self._registry.items():
            if t.track_state == "DELETED" or t.status == "LOST":
                continue
            if self._registry.is_alias(t_id):
                continue
            candidates.append((t_id, t))

        selected = []
        for t_id, target in sorted(
            candidates,
            key=lambda item: self._display_priority(item[1]),
            reverse=True,
        ):
            if any(self._suppress_for_display(target, chosen) for _cid, chosen in selected):
                continue
            selected.append((t_id, target))
        return selected

    @staticmethod
    def _display_priority(target: TrackedTarget) -> float:
        immutable_bonus = 300.0 if target.is_immutable() or target.is_in_attack_pipeline() else 0.0
        assigned_bonus = 200.0 if target.assigned_drone_port is not None else 0.0
        multi_drone_bonus = 100.0 if len(target.drone_local_ids) > 1 else 0.0
        group_bonus = 40.0 if target.is_group else 0.0
        sigma_penalty = 0.0
        cov = getattr(target, "covariance", None)
        try:
            if cov is not None:
                sigma_penalty = float(cov[0, 0] + cov[1, 1]) * 0.01
        except Exception:
            sigma_penalty = 0.0
        return (
            immutable_bonus + assigned_bonus + multi_drone_bonus + group_bonus
            + float(target.observation_count) - sigma_penalty
        )

    @staticmethod
    def _display_suppression_radius(t1: TrackedTarget, t2: TrackedTarget) -> float:
        if t1.is_group and t2.is_group:
            avg_members = (int(t1.group_member_count or 1) + int(t2.group_member_count or 1)) / 2.0
            return 12.0 + min(4.0, avg_members * 2.0)
        return 10.0

    def _suppress_for_display(self, candidate: TrackedTarget, chosen: TrackedTarget) -> bool:
        """Hide only weaker radar duplicates; never mutate control state."""
        if candidate is chosen:
            return False
        if candidate.lat is None or candidate.lon is None or chosen.lat is None or chosen.lon is None:
            return False
        if candidate.is_immutable() or chosen.is_immutable():
            return False
        if candidate.is_group != chosen.is_group:
            return False
        if candidate.is_group:
            if abs(int(candidate.group_member_count or 0) - int(chosen.group_member_count or 0)) != 0:
                return False
        if (
            candidate.assigned_drone_port is not None
            and chosen.assigned_drone_port is not None
            and candidate.assigned_drone_port != chosen.assigned_drone_port
        ):
            return False

        chosen_stronger = (
            len(chosen.drone_local_ids) > len(candidate.drone_local_ids)
            or (
                chosen.assigned_drone_port is not None
                and candidate.assigned_drone_port is None
            )
        )
        if not chosen_stronger:
            return False

        dist = GeoMath.haversine_distance(candidate.lat, candidate.lon, chosen.lat, chosen.lon)
        return dist <= self._display_suppression_radius(candidate, chosen)

    # ------------------------------------------------------------------
    # Drone target buffer
    # ------------------------------------------------------------------

    @staticmethod
    def get_drone_targets_buffer(drone_targets: Dict) -> Dict:
        """Visual buffer of targets detected by each drone."""
        return drone_targets
