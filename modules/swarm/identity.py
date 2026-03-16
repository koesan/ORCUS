"""Local ID index management and hysteresis mechanism."""

import time
from typing import Dict, Optional, Tuple

from modules.core.logger import SwarmLogger
from config import SWARM_LOCAL_ID_INDEX_TTL_S, SWARM_LOCAL_ID_INDEX_MAX_JUMP_M


class LocalIdentityIndex:
    """Drone-local tracker ID → canonical target ID mapping.

    Each drone generates its own local tracker IDs. This index
    maps (drone_port, local_tracker_id) pairs to global target IDs.
    """

    def __init__(self):
        # (drone_port, local_tracker_id) → {"tid": str, "ts": float}
        self._index: Dict[Tuple[int, int], Dict] = {}
        # ID update debounce: (drone_port, target_id) → last_log_ts
        self._log_debounce: Dict[Tuple[int, str], float] = {}
        self._debounce_interval = 15.0
        # Oscillation detector: (drone_port, target_id) → {"prev": old_id, "cur": new_id}
        self._id_history: Dict[Tuple[int, str], Dict] = {}

    # ------------------------------------------------------------------
    # Update
    # ------------------------------------------------------------------

    def update(self, drone_port: int, tracker_id: int, target_id: str,
               now: Optional[float] = None):
        """Bind a (drone, localID) pair to target ID."""
        if drone_port is None or tracker_id is None or not target_id:
            return
        now = now if now is not None else time.time()
        try:
            key = (int(drone_port), int(tracker_id))
        except (ValueError, TypeError):
            return
        self._index[key] = {"tid": target_id, "ts": now}

    # ------------------------------------------------------------------
    # Lookup
    # ------------------------------------------------------------------

    def lookup(self, drone_port: int, tracker_id: int) -> Optional[str]:
        """Find global target ID from (drone, localID) pair."""
        try:
            key = (int(drone_port), int(tracker_id))
        except (ValueError, TypeError):
            return None
        entry = self._index.get(key)
        return entry["tid"] if entry else None

    def has_key(self, drone_port: int, tracker_id: int) -> bool:
        try:
            return (int(drone_port), int(tracker_id)) in self._index
        except (ValueError, TypeError):
            return False

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def cleanup(self, now: Optional[float] = None, active_targets=None):
        """Clean up expired or non-existent target mappings."""
        now = now if now is not None else time.time()
        ttl = SWARM_LOCAL_ID_INDEX_TTL_S
        if ttl <= 0:
            return

        to_del = []
        for k, v in self._index.items():
            try:
                ts = float(v.get("ts", 0.0))
                tid = v.get("tid")
                if (now - ts) > ttl:
                    to_del.append(k)
                elif active_targets is not None and tid and tid not in active_targets:
                    to_del.append(k)
            except (ValueError, TypeError):
                to_del.append(k)

        for k in to_del:
            self._index.pop(k, None)

        # Also clean debounce cache
        expired_db = [k for k, ts in self._log_debounce.items() if (now - ts) > 30.0]
        for k in expired_db:
            self._log_debounce.pop(k, None)

    # ------------------------------------------------------------------
    # ID update logging (debounce)
    # ------------------------------------------------------------------

    def log_id_update(self, drone_port: int, target_id: str, old_id: int, new_id: int):
        """Log ID change — with oscillation + debounce spam protection."""
        now = time.time()
        key = (drone_port, target_id)

        # Oscillation check: A→B→A→B pattern suppress
        hist = self._id_history.get(key)
        if hist is not None:
            # Same swap pattern repeating (e.g., 9→7 after 7→9): suppress entirely
            if hist.get("prev") == new_id and hist.get("cur") == old_id:
                # Update history but don't log — it's just oscillating back
                self._id_history[key] = {"prev": old_id, "cur": new_id}
                return
        self._id_history[key] = {"prev": old_id, "cur": new_id}

        # Time-based debounce
        last = self._log_debounce.get(key, 0.0)
        if now - last < self._debounce_interval:
            return
        self._log_debounce[key] = now
        SwarmLogger.log(
            "ID_UPDATE", "LEADER",
            f"{target_id}: Drone_{drone_port} LocalID {old_id}→{new_id}", "ID",
        )

    # ------------------------------------------------------------------
    # Return entire index (for debugging)
    # ------------------------------------------------------------------

    def snapshot(self) -> Dict:
        return dict(self._index)
