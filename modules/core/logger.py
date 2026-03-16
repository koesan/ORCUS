"""MIL-STD-498 Structured Logger with rate-limiting and file rotation."""

import os
import sys
import time
import uuid
import datetime
import logging
import threading
from config import SWARM_LOG_PATH


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
        "FUSION_DETAY": 15.0,
        "PUANLAMA": 10.0,
        "CAMERA_STATS": 8.0,
        "RADAR_DETAY": 12.0,
        "GRACE": 5.0,
        "BBOX_SMOOTH": 10.0,
        "GROUP": 8.0,
        "RESPONSE": 8.0,
    }

    # Levels that bypass ALL rate-limiting
    _EXEMPT = frozenset({
        "CRITICAL", "ERROR", "WARNING", "PROTOCOL",
        "STATE", "STOP", "ATTACK", "FUSION",
        "ASSIGN", "LOCK", "COLLISION", "LIFECYCLE",
        "USER_CMD", "REASSIGN", "TIMEOUT", "TERMINAL",
        "SUCCESS", "TARGET", "OWNERSHIP",
        "CAPACITY", "COLLAPSE", "INFO",
        # --- Kaldırıldı: Artık rate-limited ---
        # "CAMERA_STATS", "FUSION_DETAY", "PUANLAMA"
    })

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
                key = f"{level}:{module}"
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

        ts = datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=3))).strftime("%H:%M:%S.%f")[:-3]
        line = f"[{ts}] | {level:<10} | {module:<14} | {source:<12} | {message}"
        print(line)

    @staticmethod
    def log_event(level, module, drone_id, event, details=""):
        """MIL-STD structured event log with explicit drone_id field.

        Use for critical operational events that need forensic traceability.

        Args:
            level:    Severity (CRITICAL, ERROR, WARNING, INFO)
            module:   Component (SwarmCoord, MissionCtrl, GeoMath, etc.)
            drone_id: Drone port or identifier (5760, 5773, "LEADER", etc.)
            event:    Short event name (TARGET_LOCKED, FUSION_MERGE, ATTACK_START)
            details:  Additional context string
        """
        ts = datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=3))).strftime("%H:%M:%S.%f")[:-3]
        line = f"[{ts}] | {level:<10} | {module:<14} | DRONE_{drone_id:<8} | {event}: {details}"
        print(line)

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

        ts = datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=3))).strftime("%H:%M:%S.%f")[:-3]
        line = f"[{ts}] | {level:<10} | {module:<14} | {source:<12} | {message}"
        print(line)

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
