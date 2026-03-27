"""Shared swarm communication primitives."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, Optional, Tuple


class LinkSessionPhase(str, Enum):
    IDLE = "idle"
    OPEN = "open"
    ACKED = "acked"
    VERIFYING = "verifying"
    CONFIRMED = "confirmed"
    CLOSED = "closed"
    ABORTED = "aborted"


class DroneExecutionState(str, Enum):
    IDLE = "idle"
    SCAN = "scan"
    TRACK = "track"
    SESSION_OPEN = "session_open"
    VERIFYING = "verifying"
    READY = "ready"
    AUTONOMOUS_ENGAGE = "autonomous_engage"
    RECOVERY = "recovery"
    RTL = "rtl"


class TargetPresentationState(str, Enum):
    CANDIDATE = "candidate"
    ASSIGNED = "assigned"
    VERIFYING = "verifying"
    CONFIRMED_FOR_ATTACK = "confirmed_for_attack"
    ENGAGED_MONITOR = "engaged_monitor"
    LOST = "lost"
    COMPLETE = "complete"


class TargetTrackState(str, Enum):
    TENTATIVE = "TENTATIVE"
    CONFIRMED = "CONFIRMED"
    COASTING = "COASTING"
    LOST = "LOST"
    DELETED = "DELETED"
    UNKNOWN = "UNKNOWN"


class EngagementState(str, Enum):
    NONE = "NONE"
    OBSERVING = "OBSERVING"
    ASSIGNED = "ASSIGNED"
    LOCK_PENDING = "LOCK_PENDING"
    VERIFYING = "VERIFYING"
    READY = "READY"
    EXECUTING = "EXECUTING"
    REACQUIRE = "REACQUIRE"
    HOLD = "HOLD"
    COMPLETE = "COMPLETE"
    ABORTED = "ABORTED"
    UNKNOWN = "UNKNOWN"


class EngagementSubphase(str, Enum):
    NONE = "NONE"
    ECHO = "ECHO"
    VERIFY_HOLD = "VERIFY_HOLD"
    STAGING = "STAGING"
    CENTERING = "CENTERING"
    TERMINAL_RUN = "TERMINAL_RUN"
    TERMINAL_HOLD = "TERMINAL_HOLD"
    UNKNOWN = "UNKNOWN"


class MissionExecutionState(str, Enum):
    IDLE = "IDLE"
    TAKEOFF = "TAKEOFF"
    TRANSIT = "TRANSIT"
    SCAN = "SCAN"
    TRACK = "TRACK"
    RECOVERY = "RECOVERY"
    TERMINAL_HOLD = "TERMINAL_HOLD"
    RTL = "RTL"
    PAUSED = "PAUSED"
    COMPLETE = "COMPLETE"
    ERROR = "ERROR"
    UNKNOWN = "UNKNOWN"


@dataclass(frozen=True)
class CanonicalTargetState:
    track: TargetTrackState
    engagement: EngagementState
    subphase: EngagementSubphase


@dataclass(frozen=True)
class CanonicalDroneState:
    mission: MissionExecutionState
    engagement: EngagementState
    subphase: EngagementSubphase


@dataclass
class AssignmentSessionState:
    session_id: Optional[str] = None
    phase: LinkSessionPhase = LinkSessionPhase.IDLE
    target_id: Optional[str] = None
    local_id: Optional[int] = None
    opened_at: float = 0.0
    last_update_at: float = 0.0
    confirm_sent_at: float = 0.0


@dataclass
class DroneLinkState:
    drone_id: int
    vehicle_connected: bool = False
    link_online: bool = False
    assigned_target_id: Optional[str] = None
    last_telemetry_at: float = 0.0
    last_heartbeat_at: float = 0.0
    last_command_at: float = 0.0
    last_command_id: Optional[str] = None
    last_command: Optional[str] = None
    last_detection_count: int = 0
    last_state_label: Optional[str] = None
    session: AssignmentSessionState = field(default_factory=AssignmentSessionState)


def canonical_target_track_state(track_state: Optional[str], is_coasting: bool = False) -> TargetTrackState:
    value = str(track_state or "").upper()
    if is_coasting and value in {"CONFIRMED", "ACTIVE"}:
        return TargetTrackState.COASTING
    if value in {"TENTATIVE", "CONFIRMED", "LOST", "DELETED"}:
        return TargetTrackState(value)
    return TargetTrackState.UNKNOWN


def canonical_target_engagement_state(status: Optional[str], approved_for_attack: bool = False) -> EngagementState:
    value = str(status or "").upper()
    if value in {"", "ACTIVE"}:
        return EngagementState.OBSERVING if approved_for_attack else EngagementState.NONE
    if value == "PENDING":
        return EngagementState.OBSERVING
    if value == "ASSIGNED":
        return EngagementState.ASSIGNED
    if value == "VERIFYING":
        return EngagementState.VERIFYING
    if value == "READY":
        return EngagementState.READY
    if value in {"STAGING", "CENTERING", "EXECUTING", "DIVING"}:
        return EngagementState.EXECUTING
    if value == "DESTROYED":
        return EngagementState.COMPLETE
    if value == "LOST":
        return EngagementState.REACQUIRE
    return EngagementState.UNKNOWN


def canonical_target_subphase(status: Optional[str]) -> EngagementSubphase:
    value = str(status or "").upper()
    if value == "ASSIGNED":
        return EngagementSubphase.ECHO
    if value == "STAGING":
        return EngagementSubphase.STAGING
    if value == "CENTERING":
        return EngagementSubphase.CENTERING
    if value in {"EXECUTING", "DIVING"}:
        return EngagementSubphase.TERMINAL_RUN
    return EngagementSubphase.NONE


def canonical_fsm_engagement_state(
    phase: Optional[str],
    attack_approved: bool = False,
    impact_detected: bool = False,
) -> EngagementState:
    value = str(phase or "").upper()
    if impact_detected:
        return EngagementState.COMPLETE
    if value in {"", "NONE"}:
        return EngagementState.OBSERVING if attack_approved else EngagementState.NONE
    if value == "LOCKING":
        return EngagementState.LOCK_PENDING
    if value == "VERIFY":
        return EngagementState.VERIFYING
    if value == "AUTONOMOUS":
        return EngagementState.READY
    if value == "APPROACH":
        return EngagementState.EXECUTING
    if value == "SCAN_RESUME":
        return EngagementState.REACQUIRE
    return EngagementState.UNKNOWN


def canonical_fsm_subphase(phase: Optional[str]) -> EngagementSubphase:
    value = str(phase or "").upper()
    if value == "LOCKING":
        return EngagementSubphase.ECHO
    if value == "VERIFY":
        return EngagementSubphase.VERIFY_HOLD
    if value == "AUTONOMOUS":
        return EngagementSubphase.CENTERING
    if value == "APPROACH":
        return EngagementSubphase.TERMINAL_RUN
    return EngagementSubphase.NONE


def canonical_mission_execution_state(phase: Optional[str]) -> MissionExecutionState:
    value = str(phase or "").upper()
    if value in {"IDLE", "TAKEOFF", "TRANSIT", "SCAN", "TRACK", "TERMINAL_HOLD", "RTL", "PAUSED", "COMPLETE", "ERROR"}:
        if value == "TERMINAL_HOLD":
            return MissionExecutionState.TERMINAL_HOLD
        return MissionExecutionState(value)
    if value in {"ATTACK_ABORT", "RETURN_TO_SCAN"}:
        return MissionExecutionState.RECOVERY
    if value == "ATTACK":
        return MissionExecutionState.TRACK
    return MissionExecutionState.UNKNOWN


@dataclass(frozen=True)
class LinkEnvelope:
    message_type: str
    tx_id: Optional[str] = None
    seq: int = 0
    sent_at: float = 0.0
    ttl_s: float = 0.0
    metadata: Dict[str, object] = field(default_factory=dict)

    def is_stale(self, now: Optional[float] = None) -> bool:
        if self.sent_at <= 0 or self.ttl_s <= 0:
            return False
        now = time.time() if now is None else float(now)
        return (now - float(self.sent_at)) > float(self.ttl_s)


@dataclass
class LinkWindow:
    min_interval_ms: int
    max_silence_ms: int
    last_emit_at: float = 0.0
    last_value_signature: Optional[Tuple[object, ...]] = None

    def should_emit(self, now: Optional[float] = None, signature: Optional[Tuple[object, ...]] = None) -> bool:
        now = time.time() if now is None else float(now)
        elapsed_ms = (now - float(self.last_emit_at or 0.0)) * 1000.0
        if self.last_emit_at <= 0:
            return True
        if elapsed_ms >= int(self.max_silence_ms):
            return True
        if elapsed_ms < int(self.min_interval_ms):
            return False
        if signature is None or self.last_value_signature is None:
            return True
        return signature != self.last_value_signature

    def mark_emitted(self, now: Optional[float] = None, signature: Optional[Tuple[object, ...]] = None) -> None:
        now = time.time() if now is None else float(now)
        self.last_emit_at = now
        self.last_value_signature = signature


class LinkDiscipline:
    """Dedup, ordering and pacing for noisy links."""

    def __init__(self, node_id: str):
        self.node_id = str(node_id)
        self._next_seq = 1
        self._inbound_seq_by_tx: Dict[str, int] = {}
        self._recent_message_keys: Dict[Tuple[str, int], float] = {}
        self._windows: Dict[str, LinkWindow] = {}

    def envelope(self, message_type: str, tx_id: Optional[str] = None, ttl_s: float = 1.5) -> LinkEnvelope:
        env = LinkEnvelope(
            message_type=str(message_type),
            tx_id=tx_id,
            seq=self._next_seq,
            sent_at=time.time(),
            ttl_s=float(ttl_s),
        )
        self._next_seq += 1
        return env

    def accept_inbound(self, envelope: LinkEnvelope, now: Optional[float] = None) -> Tuple[bool, Optional[str]]:
        now = time.time() if now is None else float(now)
        if envelope.is_stale(now):
            return False, "stale"

        tx_id = envelope.tx_id or envelope.message_type
        previous_seq = self._inbound_seq_by_tx.get(tx_id, 0)
        if envelope.seq < previous_seq:
            return False, "out_of_order"

        message_key = (tx_id, int(envelope.seq))
        last_seen = self._recent_message_keys.get(message_key)
        if last_seen is not None and (now - last_seen) <= max(float(envelope.ttl_s or 0.0), 1.0):
            return False, "duplicate"

        self._inbound_seq_by_tx[tx_id] = max(previous_seq, int(envelope.seq))
        self._recent_message_keys[message_key] = now
        self._gc(now)
        return True, None

    def window(self, name: str, min_interval_ms: int, max_silence_ms: int) -> LinkWindow:
        key = str(name)
        window = self._windows.get(key)
        if window is None:
            window = LinkWindow(
                min_interval_ms=int(min_interval_ms),
                max_silence_ms=int(max_silence_ms),
            )
            self._windows[key] = window
        return window

    def _gc(self, now: float) -> None:
        stale_before = now - 5.0
        self._recent_message_keys = {
            key: seen_at for key, seen_at in self._recent_message_keys.items()
            if seen_at >= stale_before
        }


def map_target_presentation_state(status: Optional[str], approved_for_attack: bool = False) -> TargetPresentationState:
    value = str(status or "").upper()
    if value in {"", "PENDING", "ACTIVE"}:
        return TargetPresentationState.ASSIGNED if approved_for_attack else TargetPresentationState.CANDIDATE
    if value == "ASSIGNED":
        return TargetPresentationState.ASSIGNED
    if value in {"VERIFYING", "VERIFY_HOLD"}:
        return TargetPresentationState.VERIFYING
    if value in {"READY", "STAGING"}:
        return TargetPresentationState.CONFIRMED_FOR_ATTACK
    if value in {"CENTERING", "EXECUTING", "DIVING"}:
        return TargetPresentationState.ENGAGED_MONITOR
    if value in {"LOST"}:
        return TargetPresentationState.LOST
    if value in {"DESTROYED", "COMPLETE"}:
        return TargetPresentationState.COMPLETE
    return TargetPresentationState.CANDIDATE


def map_drone_execution_state(
    mission_phase: Optional[str],
    tracking_mode: bool = False,
    attack_approved: bool = False,
    lock_status: Optional[str] = None,
) -> DroneExecutionState:
    phase = str(mission_phase or "").upper()
    lock = str(lock_status or "").upper()
    if phase in {"RTL", "TERMINAL_HOLD", "COMPLETE"}:
        return DroneExecutionState.RTL if phase == "RTL" else DroneExecutionState.RECOVERY
    if phase in {"ATTACK_ABORT", "RETURN_TO_SCAN", "PAUSED", "ERROR"}:
        return DroneExecutionState.RECOVERY
    if attack_approved:
        if lock in {"SESSION_OPEN", "OPEN"}:
            return DroneExecutionState.SESSION_OPEN
        if lock in {"VERIFYING", "READY"}:
            return DroneExecutionState.VERIFYING
        return DroneExecutionState.AUTONOMOUS_ENGAGE
    if tracking_mode or phase in {"TRACK", "ATTACK"}:
        return DroneExecutionState.TRACK
    if phase in {"SCAN", "TRANSIT", "TAKEOFF"}:
        return DroneExecutionState.SCAN
    return DroneExecutionState.IDLE
