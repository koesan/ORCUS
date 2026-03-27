"""MIL-STD-498 Structured Logger with rate-limiting and file rotation."""

from __future__ import annotations

import json
import os
import sys
import time
import uuid
import datetime
import logging
import threading
from dataclasses import asdict, dataclass, field, is_dataclass
from typing import Any, Dict, Optional
from config import SWARM_LOG_PATH
from config import PROJECT_ROOT
from modules.core.comm import canonical_mission_execution_state


EVENT_LOG_PATH = os.path.join(PROJECT_ROOT, "logs", "swarm_events.jsonl")


def _coerce_event_value(value: Any) -> Any:
    if is_dataclass(value):
        return asdict(value)
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    if isinstance(value, dict):
        return {str(k): _coerce_event_value(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [_coerce_event_value(v) for v in value]
    return str(value)


@dataclass
class StructuredEvent:
    ts: float
    event_type: str
    source: str
    level: str = "INFO"
    drone_id: Optional[Any] = None
    target_id: Optional[str] = None
    local_track_id: Optional[int] = None
    state_before: Optional[str] = None
    state_after: Optional[str] = None
    command: Optional[str] = None
    bbox: Optional[Any] = None
    world_estimate: Optional[Any] = None
    confidence: Optional[float] = None
    quality: Optional[float] = None
    decision_reason: Optional[str] = None
    timeout_reason: Optional[str] = None
    retry_reason: Optional[str] = None
    stale_flag: bool = False
    metadata: Dict[str, Any] = field(default_factory=dict)

    def to_record(self) -> Dict[str, Any]:
        return {key: _coerce_event_value(value) for key, value in asdict(self).items()}


class MissionPhase:
    """Canonical mission phases for lifecycle orchestration."""

    IDLE = "IDLE"
    TAKEOFF = "TAKEOFF"
    TRANSIT = "TRANSIT"
    SCAN = "SCAN"
    TRACK = "TRACK"
    ATTACK = "ATTACK"
    TERMINAL_HOLD = "TERMINAL_HOLD"
    RETURN_TO_SCAN = "RETURN_TO_SCAN"
    ATTACK_ABORT = "ATTACK_ABORT"
    RTL = "RTL"
    PAUSED = "PAUSED"
    COMPLETE = "COMPLETE"
    ERROR = "ERROR"


@dataclass
class MissionState:
    """Mutable mission state snapshot with structured transition logging."""

    phase: str = MissionPhase.IDLE
    previous_phase: Optional[str] = None
    reason: Optional[str] = None

    def transition(self, phase: str, reason: Optional[str] = None):
        before = self.canonical_phase()
        self.previous_phase = self.phase
        self.phase = phase
        self.reason = reason
        after = self.canonical_phase()
        if before != after:
            SwarmLogger.log_structured(
                "mission_phase_transition",
                "MISSION_CTRL",
                level="STATE",
                state_before=str(before),
                state_after=str(after),
                decision_reason=reason,
                metadata={"previous_phase": self.previous_phase, "phase": self.phase},
            )

    def is_terminal(self) -> bool:
        return self.phase in {MissionPhase.RTL, MissionPhase.COMPLETE, MissionPhase.ERROR}

    def canonical_phase(self):
        return canonical_mission_execution_state(self.phase)


class EventLogger:
    _path = EVENT_LOG_PATH
    _lock = threading.Lock()
    _initialized = False

    @classmethod
    def init_log(cls):
        with cls._lock:
            if cls._initialized:
                return
            log_dir = os.path.dirname(cls._path)
            if log_dir and not os.path.exists(log_dir):
                os.makedirs(log_dir, exist_ok=True)
            with open(cls._path, "w", encoding="utf-8") as handle:
                handle.write("")
            cls._initialized = True

    @classmethod
    def append(cls, event: StructuredEvent):
        cls.init_log()
        line = json.dumps(event.to_record(), ensure_ascii=True, sort_keys=True)
        with cls._lock:
            with open(cls._path, "a", encoding="utf-8") as handle:
                handle.write(line + "\n")

    @classmethod
    def emit(cls, event_type: str, source: str, **kwargs):
        event = StructuredEvent(ts=time.time(), event_type=event_type, source=source, **kwargs)
        cls.append(event)
        return event


class StreamTee:
    """Thread-safe dual-write to terminal and log file."""

    __slots__ = ('_stream', '_file', '_lock')

    def __init__(self, original_stream, log_file):
        self._stream = original_stream
        self._file = log_file
        self._lock = threading.Lock()

    def write(self, message):
        self._stream.write(message)
        self._stream.flush()
        with self._lock:
            try:
                self._file.write(message)
                self._file.flush()
            except (IOError, OSError):
                pass

    def flush(self):
        self._stream.flush()
        with self._lock:
            try:
                self._file.flush()
            except (IOError, OSError):
                pass


class SwarmLogger:
    """MIL-STD-498 compliant structured logger.

    Severity Levels (descending priority):
      CRITICAL — System failure, immediate action required
      ERROR    — Operation failed, degraded capability
      WARNING  — Unexpected condition, system compensating
      PROTOCOL — Attack handshake and state transitions
      INFO     — Significant operational events (lock, assign, fusion)
      DEBUG    — Diagnostic data (GPS_CALC, sensor readings)

    Rate-Limiting:
      Levels in _EXEMPT_LEVELS are NEVER suppressed.
      All other levels use per-(level,module) throttle windows.
    """

    _log_file = None
    _session_id = None
    _cache = {}
    _lock = threading.Lock()
    _max_file_bytes = 10 * 1024 * 1024  # 10 MB
    _max_cache_entries = 4096

    # Throttle windows (seconds) — lower = more frequent
    _THROTTLE = {
        "DEBUG":     3.0,
        "GPS_CALC":  10.0,
        "IBVS_CMD":  2.0,
        "HOVER":     5.0,
        "EKF_INPUT": 3.0,   # Her 3 saniyede bir (observation ekleme)
        "EKF_OUTPUT": 3.0,  # Her 3 saniyede bir (EKF sonucu)
        "FILTER": 8.0,
        "CENTER":    2.0,
        "GUARD":     5.0,
        "HEARTBEAT": 5.0,
        "REJECT":    2.0,
        "SECONDARY": 5.0,
        "PUBLISH":  3.0,
        "TRACK_CMD": 2.0,
        "CONTROL":  2.0,
        "FUSION_DETAIL": 15.0,
        "OBS_QUALITY": 10.0,
        "PUANLAMA": 10.0,
        "CAMERA_STATS": 8.0,
        "RADAR_DETAY": 12.0,
        "GRACE": 5.0,
        "BBOX_SMOOTH": 10.0,
        "GROUP": 8.0,
        "STATUS": 5.0,
        "SECURITY": 8.0,
        "RESPONSE": 8.0,
        "BRIDGE": 3.0,
        "INFO": 5.0,  # Info logları artık varsayılan olarak 5 sn sınırında
        "FRAME": 2.0, # Vision/Frame seviyesi saniyede 1-2
    }

    # Levels that bypass ALL rate-limiting
    _EXEMPT = frozenset({
        "CRITICAL", "ERROR", "WARNING", "PROTOCOL",
        "STATE", "STOP", "ATTACK", "FUSION",
        "ASSIGN", "LOCK", "COLLISION", "LIFECYCLE",
        "USER_CMD", "REASSIGN", "TIMEOUT", "TERMINAL",
        "SUCCESS", "TARGET", "OWNERSHIP",
        "CAPACITY", "COLLAPSE",
        # "INFO" removed from exemptions — it is now rate-limited to stop spam
    })

    @staticmethod
    def _timestamp() -> str:
        return datetime.datetime.now(
            datetime.timezone(datetime.timedelta(hours=3))
        ).strftime("%H:%M:%S.%f")[:-3]

    @staticmethod
    def _emit_line(level: str, module: str, source: str, message: str) -> None:
        print(f"[{SwarmLogger._timestamp()}] | {level:<10} | {module:<14} | {source:<12} | {message}")

    @classmethod
    def init_log(cls):
        """Initialize log session: create file, redirect streams, write header."""
        with cls._lock:
            logging.getLogger('werkzeug').setLevel(logging.ERROR)

            log_dir = os.path.dirname(SWARM_LOG_PATH)
            if log_dir and not os.path.exists(log_dir):
                os.makedirs(log_dir, exist_ok=True)

            # Restore real stdio before re-wrapping to avoid tee-chaining file handles.
            sys.stdout = sys.__stdout__
            sys.stderr = sys.__stderr__

            if cls._log_file is not None:
                try:
                    cls._log_file.flush()
                    cls._log_file.close()
                except (IOError, OSError):
                    pass
                cls._log_file = None

            # Rotate if file exceeds max size
            cls._rotate_if_needed()

            cls._session_id = uuid.uuid4().hex[:8]
            cls._cache.clear()
            EventLogger.init_log()

            f = open(SWARM_LOG_PATH, "w", encoding="utf-8")
            cls._log_file = f

            now = datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=3)))
            f.write(f"\n{'='*100}\n")
            f.write(f" ORCUS SWARM LOG SESSION STARTED: {now}\n")
            f.write(f" SESSION ID: {cls._session_id}\n")
            f.write(f"{'='*100}\n")
            f.write(f"{'TIMESTAMP':<14} | {'LEVEL':<10} | {'MODULE':<14} | {'SOURCE':<12} | EVENT\n")
            f.write(f"{'-'*100}\n")
            f.flush()

            sys.stdout = StreamTee(sys.__stdout__, f)
            sys.stderr = StreamTee(sys.__stderr__, f)

    @classmethod
    def _rotate_if_needed(cls):
        """Rotate log file if it exceeds max size. Keeps last 5 backups."""
        if not os.path.exists(SWARM_LOG_PATH):
            return
        try:
            if os.path.getsize(SWARM_LOG_PATH) > cls._max_file_bytes:
                base = SWARM_LOG_PATH
                for i in range(4, 0, -1):
                    src = f"{base}.{i}"
                    dst = f"{base}.{i+1}"
                    if os.path.exists(src):
                        os.rename(src, dst)
                os.rename(base, f"{base}.1")
        except (IOError, OSError):
            pass

    @staticmethod
    def log(level, module, message, source="SYSTEM"):
        """Write a structured log entry.

        Args:
            level:   Severity level (CRITICAL, ERROR, WARNING, INFO, DEBUG, etc.)
            module:  Component name (GeoMath, TRACKER, SwarmCoord, etc.)
            message: Human-readable event description
            source:  Source context (SYSTEM, ATTACK_FLOW, FAILSAFE, etc.)
        """
        # Rate-limit check for non-exempt levels
        if level not in SwarmLogger._EXEMPT:
            window = SwarmLogger._THROTTLE.get(level, 0)
            if window > 0:
                key = f"{level}:{module}:{source}"
                now = time.time()
                with SwarmLogger._lock:
                    SwarmLogger._prune_cache_locked(now)
                    if key in SwarmLogger._cache:
                        last_t, count = SwarmLogger._cache[key]
                        if now - last_t < window:
                            SwarmLogger._cache[key] = (last_t, count + 1)
                            return
                        if count > 0:
                            message = f"{message} [+{count} suppressed in {window}s]"
                        SwarmLogger._cache[key] = (now, 0)
                    else:
                        SwarmLogger._cache[key] = (now, 0)

        SwarmLogger._emit_line(level, module, source, message)

    @staticmethod
    def log_structured(event_type, source, level="INFO", **kwargs):
        """Append replay-friendly JSONL event in parallel with normal logs."""
        try:
            EventLogger.emit(event_type=event_type, source=source, level=level, **kwargs)
        except Exception:
            pass

    @staticmethod
    def log_operational(
        *,
        key: str,
        level: str,
        module: str,
        message: str,
        source: str = "SYSTEM",
        interval_s: float = 5.0,
        event_type: Optional[str] = None,
        structured_source: Optional[str] = None,
        structured_level: Optional[str] = None,
        **event_fields,
    ) -> bool:
        """Debounced human log with optional paired structured event."""
        emitted = SwarmLogger.log_keyed_debounce(
            key=key,
            level=level,
            module=module,
            message=message,
            source=source,
            interval_s=interval_s,
        )
        if emitted and event_type:
            SwarmLogger.log_structured(
                event_type,
                structured_source or source,
                level=structured_level or level,
                **event_fields,
            )
        return emitted

    @staticmethod
    def log_phase_change(
        *,
        module: str,
        entity: str,
        old_phase: Any,
        new_phase: Any,
        source: str,
        reason: Optional[str] = None,
        level: str = "STATE",
        event_type: Optional[str] = None,
        structured_source: Optional[str] = None,
        structured_level: Optional[str] = None,
        interval_s: float = 0.0,
        **event_fields,
    ) -> bool:
        old_label = old_phase if old_phase not in (None, "") else "IDLE"
        new_label = new_phase if new_phase not in (None, "") else "IDLE"
        message = f"{entity}: {old_label} -> {new_label}"
        if reason:
            message = f"{message} | {reason}"
        if interval_s > 0.0:
            return SwarmLogger.log_operational(
                key=f"phase:{module}:{entity}:{old_label}:{new_label}:{reason or '-'}",
                level=level,
                module=module,
                message=message,
                source=source,
                interval_s=interval_s,
                event_type=event_type,
                structured_source=structured_source,
                structured_level=structured_level,
                state_before=str(old_label),
                state_after=str(new_label),
                decision_reason=reason,
                **event_fields,
            )
        SwarmLogger.log(level, module, message, source)
        if event_type:
            SwarmLogger.log_structured(
                event_type,
                structured_source or source,
                level=structured_level or level,
                state_before=str(old_label),
                state_after=str(new_label),
                decision_reason=reason,
                **event_fields,
            )
        return True

    @staticmethod
    def log_link_activity(
        *,
        module: str,
        direction: str,
        action: str,
        source: str = "SWARM_LINK",
        detail: str = "",
        level: str = "BRIDGE",
        interval_s: float = 5.0,
        key: Optional[str] = None,
        event_type: Optional[str] = None,
        structured_source: Optional[str] = None,
        structured_level: Optional[str] = None,
        **event_fields,
    ) -> bool:
        message = f"{direction} {action}"
        if detail:
            message = f"{message} | {detail}"
        return SwarmLogger.log_operational(
            key=key or f"link:{module}:{direction}:{action}:{detail}",
            level=level,
            module=module,
            message=message,
            source=source,
            interval_s=interval_s,
            event_type=event_type,
            structured_source=structured_source,
            structured_level=structured_level,
            metadata=event_fields,
        )

    @staticmethod
    def log_throttled(level, module, message, source="SYSTEM", interval_s=5.0):
        """Explicit custom-interval throttled log."""
        key = f"T:{level}:{module}:{source}"
        now = time.time()
        with SwarmLogger._lock:
            SwarmLogger._prune_cache_locked(now)
            if key in SwarmLogger._cache:
                last_t, _ = SwarmLogger._cache[key]
                if now - last_t < interval_s:
                    return
            SwarmLogger._cache[key] = (now, 0)

        SwarmLogger._emit_line(level, module, source, message)

    @staticmethod
    def log_keyed_debounce(key: str, level: str, module: str, message: str,
                           source: str = "SYSTEM", interval_s: float = 15.0) -> bool:
        """Debounced log keyed by an arbitrary string.

        Unlike log_throttled (which keys on level+module+source), this uses
        a caller-provided key so you can debounce per (drone, target), etc.

        Returns True if the log was emitted, False if suppressed.
        """
        now = time.time()
        cache_key = f"D:{key}"
        with SwarmLogger._lock:
            SwarmLogger._prune_cache_locked(now)
            if cache_key in SwarmLogger._cache:
                last_t, _ = SwarmLogger._cache[cache_key]
                if now - last_t < interval_s:
                    return False
            SwarmLogger._cache[cache_key] = (now, 0)

        SwarmLogger._emit_line(level, module, source, message)
        return True

    @classmethod
    def _prune_cache_locked(cls, now: float):
        """Bound throttling cache growth during long-running missions."""
        if len(cls._cache) <= cls._max_cache_entries:
            return
        stale_before = now - 300.0
        stale_keys = [key for key, (ts, _) in cls._cache.items() if ts < stale_before]
        for key in stale_keys:
            cls._cache.pop(key, None)
        if len(cls._cache) <= cls._max_cache_entries:
            return
        # Fall back to dropping the oldest entries.
        for key, _ in sorted(cls._cache.items(), key=lambda item: item[1][0])[: len(cls._cache) - cls._max_cache_entries]:
            cls._cache.pop(key, None)
