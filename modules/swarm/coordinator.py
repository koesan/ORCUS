"""Swarm coordinator."""

import time
import threading
import weakref
import itertools
import numpy as np
from typing import Dict, Optional, Tuple

from modules.core.logger import SwarmLogger
from modules.core.comm import DroneLinkState, AssignmentSessionState, LinkSessionPhase
from modules.core.geo_math import GeoMath
from modules.swarm.target import (
    TargetRegistry, IMMUTABLE_STATES, OWNERSHIP_OWNED,
    TargetLifecycle, LocalIdentityIndex,
    targets_share_identity_evidence, targets_attack_compatible,
    target_family_signature, attack_family_exclusion_radius,
    TargetProcessor,
)
from modules.swarm.assignment import AssignmentEngine, OwnershipManager
from modules.swarm.target_fusion import FusionEngine
from modules.swarm.battlespace import BattlespaceView
from modules.swarm.leader_link import LeaderLink

from config import (
    ASSIGNMENT_LOOP_INTERVAL_S,
    LEADER_TIMEOUT_S, ENABLE_ATTACK_IMMUTABILITY,
    SWARM_ACTION_LOG_INTERVAL_S, SWARM_HEARTBEAT_LOG_INTERVAL_S,
    ATTACK_MIN_OWNER_QUALITY, ATTACK_LOCAL_ID_STABLE_WINDOW_S,
    SWARM_TARGET_ACTIONABLE_GRACE_S, SWARM_TARGET_STICKY_OWNER_GRACE_S,
    SWARM_ATTACK_FAMILY_FREEZE_S,
    SWARM_LOCAL_ID_ACTIONABLE_CACHE_S,
)

_MASS_APPROVAL_OBSERVATION_MAX_AGE_S = 1.5
_MASS_APPROVAL_IDENTITY_MAX_AGE_S = 1.5


class SwarmManager:
    """Swarm intelligence coordinator."""

    _instances = weakref.WeakSet()

    def __init__(self, drone_manager):
        self.drone_manager = drone_manager

        # Sub-modules
        self.registry = TargetRegistry()
        self.lifecycle = TargetLifecycle(self.registry)
        self.identity = LocalIdentityIndex()
        self.target_processor = TargetProcessor(self)
        self.ownership = OwnershipManager(self.registry)
        self.assignment = AssignmentEngine(self.registry, drone_manager)
        self.fusion = FusionEngine(self.registry)
        self.battlespace = BattlespaceView(self.registry, drone_manager, self.fusion)
        self.link = LeaderLink(self)

        # Shared state used by collaborating swarm modules.
        self.targets = self.registry._targets
        self.drone_targets: Dict[int, list] = {}
        self.lock = self.registry.lock
        self.attack_mode_active = False

        # Heartbeat izleme
        self.drone_heartbeat_ts: Dict[int, float] = {}
        self.drone_link_state: Dict[int, DroneLinkState] = {}

        # Geo-mismatch cooldown
        self.drone_mismatch_cooldowns: Dict[int, Dict[str, float]] = {}
        self.drone_recovery_until: Dict[int, float] = {}

        # Log rate-limit
        self._immutable_log_ts: Dict = {}
        self._immutable_log_target: Dict = {}
        self._action_log_ts: Dict = {}
        self._action_log_signature: Dict = {}
        self._active_target_grace_until: Dict[int, float] = {}
        self._attack_family_freeze: Dict[int, Tuple[Tuple, float]] = {}
        self._last_actionable_local_id_by_drone: Dict[int, Tuple[str, int, float]] = {}
        self._command_epoch_by_drone: Dict[int, int] = {}
        self._command_counter = itertools.count(1)
        self._session_counter = itertools.count(1)
        
        # UI mapping for success markers
        self.completed_missions = []

        # Periyodik döngü
        self._loop_running = True
        self._work_event = threading.Event()
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

    def report_target(self, drone_port, lat, lon, confidence,
                      tracker_id=None, raw_data=None, covariance=None):
        """Process inbound target detection and return action."""
        response = self.target_processor.process_report(
            drone_port, lat, lon, confidence,
            tracker_id=tracker_id, raw_data=raw_data, covariance=covariance,
        )
        self._work_event.set()
        return response

    def check_mission_updates(self, drone_port, raw_data=None):
        """Process inbound telemetry/detections and return action."""
        needs_prune = True
        realtime_refresh_needed = False

        # Route inbound data
        if raw_data is not None:
            if isinstance(raw_data, dict) and "detected_targets" in raw_data:
                det = raw_data.get("detected_targets", [])
                needs_prune = not bool(det)
                realtime_refresh_needed = bool(det)
                self._record_drone_telemetry(drone_port, len(det), raw_data.get("state"))
                self._log_detection_ingest(drone_port, det, raw_data.get("state"))
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

        if realtime_refresh_needed:
            self._refresh_runtime_state(trigger="detections")

        current_time = time.time()
        with self.lock:
            if needs_prune:
                self.lifecycle.prune(
                    current_time, self.attack_mode_active,
                    drone_connected_check=self._is_drone_connected,
                    release_drone=self.ownership.release,
                    remove_fusion_target=self.fusion.ekf_manager.remove_target,
                )
            action = self._build_mission_action(drone_port)
            return self._command_payload(drone_port, action)

    def _log_detection_ingest(self, drone_port: int, detections: list, state_label: Optional[str]) -> None:
        count = len(detections or [])
        groups = sum(1 for item in (detections or []) if item.get("is_group"))
        ids = []
        qualities = []
        for item in detections or []:
            tid = item.get("track_id")
            if tid is not None:
                ids.append(str(tid))
            q = item.get("observation_quality")
            if q is not None:
                try:
                    qualities.append(float(q))
                except (TypeError, ValueError):
                    pass
        quality_str = "-"
        if qualities:
            quality_str = f"{sum(qualities)/len(qualities):.1f}/{max(qualities):.1f}"
        preview = ",".join(ids[:4]) if ids else "-"
        if len(ids) > 4:
            preview += ",..."
        SwarmLogger.log_throttled(
            "INGEST",
            "LEADER",
            f"Drone_{drone_port} ingest | count={count} groups={groups} state={state_label or '-'} "
            f"quality(avg/max)={quality_str} ids={preview}",
            "LEADER",
            interval_s=3.0,
        )

    def get_battlespace_state(self):
        """Web UI için battlespace durumu."""
        state = self.battlespace.get_state(self.ownership)
        now = time.time()
        for port, drone in (state.get("drones") or {}).items():
            link = self.drone_link_state.get(int(port))
            if link is None:
                continue
            drone["vehicle_connected"] = bool(link.vehicle_connected)
            drone["link_online"] = bool(link.link_online)
            drone["last_telemetry_age_s"] = (now - link.last_telemetry_at) if link.last_telemetry_at > 0 else None
            drone["last_heartbeat_age_s"] = (now - link.last_heartbeat_at) if link.last_heartbeat_at > 0 else None
            drone["last_command"] = link.last_command
            drone["last_command_id"] = link.last_command_id
            drone["assigned_target_id"] = link.assigned_target_id
            drone["session_id"] = link.session.session_id
            drone["session_phase"] = link.session.phase.value if link.session.phase else LinkSessionPhase.IDLE.value
            drone["session_target_id"] = link.session.target_id
            drone["session_local_id"] = link.session.local_id
            
        state["completed_missions"] = list(self.completed_missions)
        return state

    def approve_attack(self, target_id):
        """Tek hedef saldırı onayı."""
        return self.link.approve_attack(target_id)

    def approve_all_targets(self):
        """Tüm CONFIRMED hedefleri onayla."""
        with self.lock:
            self.attack_mode_active = True
            SwarmLogger.log("USER_CMD", "LEADER", "MISSION APPROVAL MODE ACTIVATED", "USER")

            try:
                assignments = self.assign_attack_targets()
                if assignments:
                    SwarmLogger.log(
                        "USER_CMD", "LEADER",
                        f"MISSION PRE-ASSIGN: {assignments}",
                        "USER",
                    )
            except Exception as e:
                SwarmLogger.log("WARNING", "LEADER",
                                f"MISSION PRE-ASSIGN failed: {e}", "USER")

            eligible = []
            now = time.time()
            skipped = {
                "deleted_or_lost": 0,
                "already_approved": 0,
                "no_position": 0,
                "not_confirmed": 0,
                "unassigned": 0,
                "stale_observation": 0,
                "missing_local_id": 0,
                "stale_identity": 0,
                "low_quality": 0,
                "unstable_identity": 0,
            }
            for t_id, t in self.targets.items():
                if t.track_state == "DELETED" or t.status == "LOST":
                    skipped["deleted_or_lost"] += 1
                    continue
                if t.approved_for_attack:
                    skipped["already_approved"] += 1
                    continue
                if t.lat is None:
                    skipped["no_position"] += 1
                    continue
                if t.track_state != "CONFIRMED":
                    skipped["not_confirmed"] += 1
                    continue
                if t.assigned_drone_port is None:
                    skipped["unassigned"] += 1
                    continue
                if (now - float(t.last_observation_time or 0.0)) > _MASS_APPROVAL_OBSERVATION_MAX_AGE_S:
                    skipped["stale_observation"] += 1
                    continue
                local_id = self._resolve_local_target_id(t, t.assigned_drone_port)
                if local_id is None:
                    skipped["missing_local_id"] += 1
                    continue
                last_seen = float(t.drone_last_seen_time.get(t.assigned_drone_port, 0.0) or 0.0)
                if (now - last_seen) > _MASS_APPROVAL_IDENTITY_MAX_AGE_S:
                    skipped["stale_identity"] += 1
                    continue
                owner_quality = float(t.observation_quality.get(t.assigned_drone_port, 0.0) or 0.0)
                if owner_quality <= 0.0 and local_id is not None:
                    owner_quality = ATTACK_MIN_OWNER_QUALITY
                phase = (self._get_controller_phase(t.assigned_drone_port) or "").upper()
                active_tracking_ready = (
                    phase in {"TRACK", "ATTACK"}
                    and local_id is not None
                    and (now - last_seen) <= _MASS_APPROVAL_IDENTITY_MAX_AGE_S
                )
                if owner_quality < ATTACK_MIN_OWNER_QUALITY and not active_tracking_ready:
                    skipped["low_quality"] += 1
                    continue
                identity_change_ts = float(
                    getattr(t, "identity_change_time", {}).get(
                        t.assigned_drone_port,
                        t.identity_last_update_time.get(t.assigned_drone_port, 0.0),
                    ) or 0.0
                )
                stable_duration = (now - identity_change_ts) if identity_change_ts > 0.0 else float("inf")
                if (
                    stable_duration < ATTACK_LOCAL_ID_STABLE_WINDOW_S
                    and int(getattr(t, "consecutive_obs", 0) or 0) <= 1
                    and t.assigned_drone_port in getattr(t, "drone_local_ids", {})
                ):
                    stable_duration = float("inf")
                if stable_duration < ATTACK_LOCAL_ID_STABLE_WINDOW_S and not active_tracking_ready:
                    skipped["unstable_identity"] += 1
                    continue
                if self._shadowed_by_stronger_family(t):
                    skipped.setdefault("shadowed_family", 0)
                    skipped["shadowed_family"] += 1
                    continue
                stab = t.consecutive_obs / (1.0 + t.id_switch_count + t.drop_count)
                eligible.append((t_id, t, stab, local_id))

            eligible.sort(key=lambda x: x[2], reverse=True)

            approved_targets = []
            active_families = set()
            for a_id, a_t in self.targets.items():
                if a_t.track_state == "DELETED" or a_t.status == "LOST":
                    continue
                if a_t.assigned_drone_port is None:
                    continue
                if a_t.approved_for_attack or a_t.status in IMMUTABLE_STATES:
                    active_families.add(target_family_signature(a_t))
            count = 0
            for t_id, t, stab, local_id in eligible:
                family_sig = target_family_signature(t)
                frozen = self._frozen_family_for_drone(t.assigned_drone_port, now)
                if frozen is not None and frozen != family_sig and not getattr(t, "is_group", False):
                    continue
                if family_sig in active_families and family_sig[0] != "group_fallback" and not getattr(t, "is_group", False):
                    SwarmLogger.log(
                        "USER_CMD", "LEADER",
                        f"MASS APPROVE SUPPRESS: {t_id} skipped (family already active)",
                        "USER",
                    )
                    continue
                dup = False
                for approved in approved_targets:
                    a_t = approved["target"]
                    if not targets_attack_compatible(t, a_t):
                        continue
                    exclusion = attack_family_exclusion_radius(t, a_t)
                    if GeoMath.haversine_distance(t.lat, t.lon, a_t.lat, a_t.lon) < exclusion:
                        dup = True
                        SwarmLogger.log(
                            "USER_CMD", "LEADER",
                            f"MASS APPROVE SUPPRESS: {t_id} skipped (same physical family as {approved['id']})",
                            "USER",
                        )
                        break
                if dup:
                    continue

                if t.status == "PENDING" or (t.track_state == "CONFIRMED" and not t.approved_for_attack):
                    t.approved_for_attack = True
                    t.status = "ASSIGNED"
                    SwarmLogger.log(
                        "USER_CMD", "LEADER",
                        f"MASS APPROVE: {t_id} → ASSIGNED | drone={t.assigned_drone_port} local_id={local_id}",
                        "USER",
                    )
                    self._attack_family_freeze[t.assigned_drone_port] = (
                        family_sig,
                        time.time() + SWARM_ATTACK_FAMILY_FREEZE_S,
                    )
                    count += 1
                    approved_targets.append({"target": t, "id": t_id, "stab": stab})
                    active_families.add(family_sig)

            if count == 0:
                summary = ", ".join(f"{k}={v}" for k, v in skipped.items() if v)
                SwarmLogger.log(
                    "USER_CMD", "LEADER",
                    f"MASS APPROVE: no eligible targets | {summary or 'no targets present'}",
                    "USER",
                )
            return count

    def _shadowed_by_stronger_family(self, candidate) -> bool:
        if candidate is None or candidate.lat is None or candidate.lon is None:
            return False
        if candidate.is_in_attack_pipeline():
            return False
        if len(getattr(candidate, "drone_local_ids", {})) > 1:
            return False

        for other in self.targets.values():
            if other is candidate:
                continue
            if other.track_state == "DELETED" or other.status == "LOST":
                continue
            if other.lat is None or other.lon is None:
                continue
            if len(getattr(other, "drone_local_ids", {})) < 2:
                continue
            if other.assigned_drone_port is None and other.owner_port is None:
                continue
            shared_ports = set(getattr(candidate, "drone_local_ids", {})).intersection(
                set(getattr(other, "drone_local_ids", {}))
            )
            if not shared_ports:
                continue
            dist = GeoMath.haversine_distance(candidate.lat, candidate.lon, other.lat, other.lon)
            if dist > max(attack_family_exclusion_radius(candidate, other), 18.0):
                continue
            if targets_share_identity_evidence(candidate, other):
                return True
            if other.observation_count >= max(100, int(candidate.observation_count or 0) * 4):
                return True
        return False

    def _frozen_family_for_drone(self, drone_port: int, now: float) -> Optional[Tuple]:
        frozen = self._attack_family_freeze.get(drone_port)
        if not frozen:
            return None
        signature, expires_at = frozen
        if now > expires_at:
            self._attack_family_freeze.pop(drone_port, None)
            return None
        return signature

    def _has_actionable_grace(self, target, drone_port: int, now: float) -> bool:
        if now <= self._active_target_grace_until.get(drone_port, 0.0):
            return True
        has_group_context = getattr(target, "group_id_by_port", {}).get(drone_port) is not None
        has_local_context = getattr(target, "drone_local_ids", {}).get(drone_port) is not None
        if not has_group_context and not has_local_context:
            return False
        if target.has_recent_identity(drone_port, now, SWARM_TARGET_ACTIONABLE_GRACE_S):
            self._active_target_grace_until[drone_port] = now + SWARM_TARGET_ACTIONABLE_GRACE_S
            return True
        last_seen = float(target.drone_last_seen_time.get(drone_port, 0.0) or 0.0)
        if (now - last_seen) <= SWARM_TARGET_STICKY_OWNER_GRACE_S:
            self._active_target_grace_until[drone_port] = now + SWARM_TARGET_ACTIONABLE_GRACE_S
            return True
        return False

    def release_drone_assignment(self, drone_port):
        """Drone atamasını bırak (R1/R32)."""
        with self.lock:
            released = [t_id for t_id, t in self.targets.items()
                        if t.assigned_drone_port == drone_port]
            self.ownership.release(drone_port)
            self.drone_heartbeat_ts.pop(drone_port, None)
            self.drone_link_state.pop(drone_port, None)
            self.drone_recovery_until.pop(drone_port, None)
            self._active_target_grace_until.pop(drone_port, None)
            self._attack_family_freeze.pop(drone_port, None)
            clear_fn = getattr(self.drone_manager, "clear_airspace_reservation", None)
            if callable(clear_fn):
                clear_fn(drone_port)
            if released:
                SwarmLogger.log("LIFECYCLE", "LEADER",
                                f"Drone_{drone_port} RELEASED: targets={released}", "TRACK")

    def handle_drone_disconnect(self, port):
        """Drone bağlantı kopması — ownership release, heartbeat temizle.

        fleet_manager.connect_drone_async() tarafından disconnect'te çağrılır.
        """
        with self.lock:
            # Ownership release
            released = [t_id for t_id, t in self.targets.items()
                        if t.assigned_drone_port == port]
            self.ownership.release(port)

            # Heartbeat temizle
            self.drone_heartbeat_ts.pop(port, None)
            self.drone_link_state.pop(port, None)
            self.drone_recovery_until.pop(port, None)
            self._active_target_grace_until.pop(port, None)
            self._attack_family_freeze.pop(port, None)
            clear_fn = getattr(self.drone_manager, "clear_airspace_reservation", None)
            if callable(clear_fn):
                clear_fn(port)

            # Drone hedeflerini temizle
            self.drone_targets.pop(port, None)

            # Mismatch cooldown temizle
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
        """Drone meşgul mü?"""
        return self.ownership.is_drone_busy(drone_port)

    def assign_attack_targets(self):
        """Optimal atama."""
        return self.assignment.assign_attack_targets(self.ownership)

    def fuse_cross_drone_targets(self):
        """Çapraz füzyon (identity index ile)."""
        return self.fusion.fuse_cross_drone(self.ownership, self.identity)

    def get_assigned_local_id(self, drone_port):
        """Drone'un hedefinin lokal ID'si."""
        with self.lock:
            for t_id, t in self.targets.items():
                if t.assigned_drone_port == drone_port and (t.approved_for_attack or t.status in IMMUTABLE_STATES):
                    return t.drone_local_ids.get(drone_port)
        return None

    def remember_actionable_local_id(self, drone_port: int, target_id: str, local_id: Optional[int]) -> None:
        if local_id is None or target_id is None:
            return
        self._last_actionable_local_id_by_drone[drone_port] = (str(target_id), int(local_id), time.time())

    def get_cached_actionable_local_id(self, drone_port: int, target_id: str) -> Optional[int]:
        cached = self._last_actionable_local_id_by_drone.get(drone_port)
        if not cached:
            return None
        cached_target_id, cached_local_id, ts = cached
        if cached_target_id != str(target_id):
            return None
        if (time.time() - float(ts)) > SWARM_LOCAL_ID_ACTIONABLE_CACHE_S:
            return None
        return int(cached_local_id)

    def get_drone_targets_buffer(self):
        """Drone tespit tamponu."""
        return self.drone_targets

    def reset_runtime_state(self):
        """Soft reset volatile swarm state while preserving drone links."""
        with self.lock:
            self.registry.reset_runtime_state()
            self.drone_targets.clear()
            self.drone_heartbeat_ts.clear()
            self.drone_link_state.clear()
            self.drone_recovery_until.clear()
            self.ownership.drone_active_target.clear()
            self.ownership.target_owner.clear()
            self.attack_mode_active = False
            self._active_target_grace_until.clear()
            self._attack_family_freeze.clear()
            self._command_epoch_by_drone.clear()
            self.identity.reset_runtime_state()
            self.fusion._merge_backoff.clear()
            self.fusion.ekf_manager.reset_runtime_state()
            self.battlespace.reset_runtime_state()
        SwarmLogger.log("RESET", "LEADER",
                        "Soft reset completed: radar, targets, fusion and ownership cleared.",
                        "SYSTEM")

    # ==================================================================
    # İÇ YARDIMCILAR
    # ==================================================================

    def _is_drone_connected(self, port: int) -> bool:
        """Drone hâlâ bağlı mı?"""
        return (port in self.drone_manager.drones
                and self.drone_manager.drones[port] != "connecting")

    def _ensure_drone_link_state(self, port: int) -> DroneLinkState:
        state = self.drone_link_state.get(port)
        if state is None:
            state = DroneLinkState(drone_id=int(port))
            self.drone_link_state[port] = state
        state.vehicle_connected = self._is_drone_connected(port)
        return state

    def _record_drone_telemetry(self, port: int, detection_count: int, state_label: Optional[str]) -> None:
        state = self._ensure_drone_link_state(port)
        state.link_online = True
        state.vehicle_connected = self._is_drone_connected(port)
        state.last_telemetry_at = time.time()
        state.last_detection_count = int(detection_count or 0)
        state.last_state_label = state_label

    def _record_drone_heartbeat(self, port: int, target_id: Optional[str] = None, session_id: Optional[str] = None) -> None:
        state = self._ensure_drone_link_state(port)
        now = time.time()
        state.link_online = True
        state.vehicle_connected = self._is_drone_connected(port)
        state.last_heartbeat_at = now
        if target_id is not None:
            state.assigned_target_id = str(target_id)
        if session_id is not None:
            state.session.session_id = str(session_id)
            state.session.last_update_at = now

    def _record_command_publish(
        self,
        port: int,
        action: str,
        command_id: Optional[str],
        target_id: Optional[str] = None,
        local_id: Optional[int] = None,
        session_id: Optional[str] = None,
    ) -> None:
        state = self._ensure_drone_link_state(port)
        now = time.time()
        state.last_command_at = now
        state.last_command = str(action)
        state.last_command_id = command_id
        state.vehicle_connected = self._is_drone_connected(port)
        if target_id is not None:
            state.assigned_target_id = str(target_id)
        if action == "ASSIGN":
            sid = str(session_id or f"{port}:{next(self._session_counter)}")
            state.session = AssignmentSessionState(
                session_id=sid,
                phase=LinkSessionPhase.OPEN,
                target_id=str(target_id) if target_id is not None else None,
                local_id=local_id,
                opened_at=now,
                last_update_at=now,
            )
        elif action == "VERIFY":
            state.session.phase = LinkSessionPhase.VERIFYING
            state.session.last_update_at = now
        elif action == "EXECUTE":
            state.session.phase = LinkSessionPhase.CLOSED
            state.session.last_update_at = now
            if state.session.confirm_sent_at <= 0:
                state.session.confirm_sent_at = now
        elif action in {"STOP", "ABORT"}:
            state.session.phase = LinkSessionPhase.ABORTED
            state.session.last_update_at = now

    def _record_session_event(
        self,
        port: int,
        phase: LinkSessionPhase,
        target_id: Optional[str] = None,
        local_id: Optional[int] = None,
        session_id: Optional[str] = None,
    ) -> None:
        state = self._ensure_drone_link_state(port)
        now = time.time()
        state.link_online = True
        state.vehicle_connected = self._is_drone_connected(port)
        if session_id is not None:
            state.session.session_id = str(session_id)
        elif not state.session.session_id:
            state.session.session_id = f"{port}:{next(self._session_counter)}"
        state.session.phase = phase
        state.session.last_update_at = now
        if state.session.opened_at <= 0:
            state.session.opened_at = now
        if phase == LinkSessionPhase.CONFIRMED:
            state.session.confirm_sent_at = now
        if target_id is not None:
            state.session.target_id = str(target_id)
            state.assigned_target_id = str(target_id)
        if local_id is not None:
            state.session.local_id = int(local_id)

    def _is_drone_stopping(self, port: int) -> bool:
        """Stop/RTL geçişindeki drone'a yeni attack publish etme."""
        controllers = getattr(self.drone_manager, "drone_controllers", {}) or {}
        controller = controllers.get(port)
        return bool(getattr(controller, "is_stopping", False))

    def _get_controller_phase(self, port: int) -> Optional[str]:
        controllers = getattr(self.drone_manager, "drone_controllers", {}) or {}
        controller = controllers.get(port)
        if controller is None:
            return None
        return getattr(controller, "mission_phase", None)

    def _drone_in_recovery_window(self, port: int) -> bool:
        until = self.drone_recovery_until.get(port, 0.0)
        return time.time() < until

    def _mark_drone_recovering(self, port: int, reason: str):
        self.drone_recovery_until[port] = time.time() + LEADER_TIMEOUT_S
        clear_fn = getattr(self.drone_manager, "clear_airspace_reservation", None)
        if callable(clear_fn):
            clear_fn(port)
        SwarmLogger.log(
            "FAILSAFE", "LEADER",
            f"Drone_{port} marked recovering ({reason})",
            "FAILSAFE",
        )

    def _log_action_decision(self, drone_port: int, action: str, reason: str,
                             target_id=None, local_id=None,
                             interval_s: float = SWARM_ACTION_LOG_INTERVAL_S):
        signature = (action, reason, target_id, local_id)
        last_sig = self._action_log_signature.get(drone_port)
        now = time.time()
        last_ts = self._action_log_ts.get(drone_port, 0.0)
        if signature == last_sig and (now - last_ts) < interval_s:
            return
        SwarmLogger.log(
            "STATE", "PUBLISH",
            f"Drone_{drone_port}: action={action} reason={reason} "
            f"target={target_id} local_id={local_id}",
            "ACTION",
        )
        SwarmLogger.log_structured(
            "leader_command_decision",
            "ACTION",
            level="STATE",
            drone_id=drone_port,
            target_id=target_id,
            local_track_id=local_id,
            command=action,
            decision_reason=reason,
            metadata={"signature": signature},
        )
        self._action_log_signature[drone_port] = signature
        self._action_log_ts[drone_port] = now

    def _next_command_metadata(self, drone_port: int, action: str, target_id=None) -> Dict[str, object]:
        return self.link.next_command_metadata(drone_port, action, target_id=target_id)

    def _command_payload(self, drone_port: int, action_tuple):
        return self.link.build_command_payload(drone_port, action_tuple)

    def _reserve_attack_corridor(self, port: int, target_id: str, target) -> None:
        if target.lat is None or target.lon is None:
            return
        refresh_fn = getattr(self.drone_manager, "refresh_airspace_reservation", None)
        vehicle = getattr(self.drone_manager, "drones", {}).get(port)
        if callable(refresh_fn) and vehicle not in (None, "connecting"):
            try:
                loc = vehicle.location.global_relative_frame
                if loc and loc.lat is not None and loc.lon is not None:
                    refresh_fn(
                        port,
                        "ATTACK",
                        float(loc.lat),
                        float(loc.lon),
                        float(target.lat),
                        float(target.lon),
                        altitude=float(getattr(loc, "alt", 0.0) or 0.0),
                    )
            except Exception:
                pass

    def _handle_heartbeat(self, drone_port):
        """Heartbeat mesajını işle."""
        self._record_drone_heartbeat(drone_port)
        for t_id, t in self.targets.items():
            is_protected = (t.approved_for_attack
                            or (ENABLE_ATTACK_IMMUTABILITY and t.status in IMMUTABLE_STATES))
            if t.assigned_drone_port == drone_port and is_protected:
                t.last_seen_time = time.time()
                self.drone_heartbeat_ts[drone_port] = time.time()
                link_state = self._ensure_drone_link_state(drone_port)
                self._record_drone_heartbeat(drone_port, t_id, session_id=link_state.session.session_id)
                local_id = self._resolve_local_target_id(t, drone_port)
                self._reserve_attack_corridor(drone_port, t_id, t)
                lat_f, lon_f = self._get_filtered_position(t_id, t)
                SwarmLogger.log("HEARTBEAT", f"DRONE_{drone_port}",
                                f"Target {t_id} still {t.status}", "FAILSAFE")
                return {
                    "action": "EXECUTE",
                    "target_id": t_id,
                    "tracker_id": local_id,
                    "local_id": local_id,
                    "lat": lat_f,
                    "lon": lon_f,
                }
        SwarmLogger.log_throttled(
            "HEARTBEAT", f"DRONE_{drone_port}",
            "Heartbeat received but no protected active target exists",
            "FAILSAFE",
            interval_s=SWARM_HEARTBEAT_LOG_INTERVAL_S,
        )
        return {"action": "SEARCH", "message": "No active target for heartbeat"}

    def _resolve_local_target_id(self, target, drone_port) -> Optional[int]:
        return self.link.resolve_local_target_id(target, drone_port)

    def _dispatch_action(self, drone_port, raw_data):
        """check_mission_updates içinden gelen action mesajlarını yönlendir."""
        return self.link.dispatch_inbound(drone_port, raw_data)

    def _build_mission_action(self, drone_port):
        return self.link.build_mission_action(drone_port)

    def _get_filtered_position(self, target_id: str, target) -> Tuple[float, float]:
        return self.link.get_filtered_position(target_id, target)

    def _immutable_action(self, target, t_id, local_id):
        return self.link.immutable_action(target, t_id, local_id)

    def _target_action(self, target, t_id, local_id, drone_port):
        return self.link.target_action(target, t_id, local_id, drone_port)

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
        return self.link.find_direct_reassignment(drone_port)

    def _conflicts_with_active_family(self, candidate_id: str, candidate_target) -> bool:
        return self.link.conflicts_with_active_family(candidate_id, candidate_target)

    # ==================================================================
    # PERİYODİK OPTİMİZASYON DÖNGÜSÜ
    # ==================================================================

    def _periodic_loop(self):
        """Periyodik: radar log, EKF batch, owner loss, merge, Hungarian, fusion."""
        SwarmLogger.log("INFO", "LEADER", "Proactive Assignment Loop Started.", "OPT_LOOP")

        while self._loop_running:
            try:
                self._work_event.wait(timeout=min(float(ASSIGNMENT_LOOP_INTERVAL_S), 0.25))
                self._work_event.clear()
                self._refresh_runtime_state(trigger="periodic")

            except Exception as e:
                SwarmLogger.log("ERROR", "OPT_LOOP",
                                f"Optimization Error: {e}", "OPT_LOOP")

            try:
                self._prune_stale_drones()
            except Exception as e:
                SwarmLogger.log("ERROR", "OPT_LOOP",
                                f"Heartbeat prune error: {e}", "OPT_LOOP")

    def _refresh_runtime_state(self, trigger: str = "runtime") -> None:
        """Run leader-side realtime maintenance independent of fixed loop cadence."""
        now = time.time()
        self.lifecycle.prune(
            now, self.attack_mode_active,
            drone_connected_check=self._is_drone_connected,
            release_drone=self.ownership.release,
            remove_fusion_target=self.fusion.ekf_manager.remove_target,
        )

        self._update_leader_reference()
        try:
            self.battlespace.log_radar_snapshot()
        except Exception as exc:
            SwarmLogger.log("WARNING", "OPT_LOOP", f"Radar snapshot skipped: {exc}", "OPT_LOOP")

        with self.lock:
            coasting_status = {t_id: t.is_coasting for t_id, t in self.targets.items()}
        self.fusion.process_ekf_batch(coasting_status)

        with self.lock:
            for _t_id, target in list(self.targets.items()):
                if target.ownership_state != OWNERSHIP_OWNED or target.owner_port is None:
                    continue
                since = now - target.last_observation_time
                quality = target.observation_quality.get(target.owner_port, 0)
                if since > 3.0 and quality > 0:
                    self.ownership.process_owner_loss(target)

            pending = [
                t_id for t_id, t in self.targets.items()
                if t.status == "PENDING" and t.assigned_drone_port is None
            ]
            idle = set(p for p, v in self.drone_manager.drones.items() if v != "connecting") - {
                t.assigned_drone_port for t in self.targets.values()
                if t.assigned_drone_port is not None and (t.status == "PENDING" or t.approved_for_attack)
            }

        self.fusion.merge_duplicates(self.ownership, self.identity)

        if pending and idle:
            assignments = self.assign_attack_targets()
            if assignments:
                SwarmLogger.log(
                    "ASSIGN", "OPT_LOOP",
                    f"Realtime Assignment[{trigger}]: {assignments}",
                    "ASSIGN",
                )

        fused = self.fuse_cross_drone_targets()
        if fused > 0:
            SwarmLogger.log(
                "FUSION", "OPT_LOOP",
                f"Realtime Fusion[{trigger}]: {fused} targets merged.",
                "FUSION",
            )

    def _prune_stale_drones(self):
        """Heartbeat zaman aşımına uğramış drone atamalarını bırak."""
        now = time.time()
        with self.lock:
            for t_id, t in self.targets.items():
                port = t.assigned_drone_port
                if port is None:
                    continue
                last_hb = self.drone_heartbeat_ts.get(port)
                if last_hb is not None and (now - last_hb) > LEADER_TIMEOUT_S:
                    terminal_protected = t.status in {"VERIFYING", "READY", "STAGING", "CENTERING", "EXECUTING", "DIVING"}
                    if terminal_protected:
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
                    self._mark_drone_recovering(port, f"stale heartbeat on {t_id}")
                    self.ownership.release(port)

    def _update_leader_reference(self):
        """Update the NED reference from the most active connected drone."""
        leader_port = None
        leader_lat = None
        leader_lon = None
        max_targets = 0
        
        with self.lock:
            drone_target_counts: Dict[int, int] = {}
            for t_id, t in self.targets.items():
                if t.assigned_drone_port is not None and (t.status == "PENDING" or t.approved_for_attack):
                    port = t.assigned_drone_port
                    drone_target_counts[port] = drone_target_counts.get(port, 0) + 1
            
            # En çok hedefi olan drone'u lider olarak seç
            for port, count in drone_target_counts.items():
                if count > max_targets:
                    max_targets = count
                    leader_port = port
            
            # Eğer aktif hedef yoksa, ilk bağlı drone'u lider olarak seç
            if leader_port is None:
                for port, vehicle in self.drone_manager.drones.items():
                    if vehicle != "connecting":
                        leader_port = port
                        break
            
            # Lider drone'un konumunu al
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
        
        # EKF dünya referansı sabittir; lider konumu dinamik referans olarak kullanılmaz.

    def stop(self):
        """Periyodik döngüyü durdur."""
        self._loop_running = False
        self._work_event.set()
        thread = getattr(self, "_loop_thread", None)
        if thread and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=ASSIGNMENT_LOOP_INTERVAL_S * 2.0)
