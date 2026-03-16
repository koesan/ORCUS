"""Target data model and thread-safe target registry."""

import time
import threading
import numpy as np
from typing import Dict, Optional, Tuple

import config
from modules.core.logger import SwarmLogger

# Immutable attack states — all modules read from this set
IMMUTABLE_STATES = frozenset({
    "LOCKED", "ECHO_WAIT", "CONFIRMED_ATTACK",
    "CENTERING", "ATTACKING", "DIVING"
})

# Ownership constants
OWNERSHIP_FREE = "FREE"
OWNERSHIP_OWNED = "OWNED"
OWNERSHIP_RESERVED = "RESERVED"
OWNERSHIP_HANDOFF = "HANDOFF_PENDING"


class TrackedTarget:
    """Data object holding all state information for a single target."""

    __slots__ = (
        "id", "lat", "lon", "first_seen_time", "last_seen_time",
        "assigned_drone_port", "status", "approved_for_attack", "tracker_id",
        # Lifecycle
        "track_state", "observation_count", "last_observation_time",
        # Ownership
        "ownership_state", "owner_port", "owner_lost_time",
        "last_handoff_time", "handoff_candidate_port", "handoff_score_accumulator",
        # Observation quality
        "secondary_observations", "observation_quality",
        # Stability metrics
        "id_switch_count", "drop_count", "consecutive_obs", "_edge_accumulator",
        # Drone-local ID mapping
        "drone_local_ids", "source_drone_port",
        "drone_last_seen_time", "identity_last_update_time",
        # Covariance and sources
        "covariance", "observation_sources", "local_id_hysteresis",
        # Group info
        "is_group", "group_member_count", "group_id_local", "group_id_by_port",
        "group_member_local_ids", "group_count_by_port",
        # Assignment time
        "assignment_time",
        # Grace Period & Coast Mode (Phase 1)
        "is_coasting", "coast_start_time", "last_velocity", "coast_distance_traveled",
    )

    def __init__(
        self,
        t_id: str,
        lat: float,
        lon: float,
        first_seen_time: float,
        tracker_id: Optional[int] = None,
        covariance=None,
        source_drone_port: Optional[int] = None,
    ):
        # Legacy compatibility:
        #   TrackedTarget(id, lat, lon, confidence, drone_port, tracker_id)
        # New signature:
        #   TrackedTarget(id, lat, lon, first_seen_time, tracker_id=None, covariance=None, source_drone_port=None)
        if source_drone_port is None and covariance is not None and tracker_id is not None:
            if np.isscalar(first_seen_time) and 0.0 <= float(first_seen_time) <= 1.0:
                if np.isscalar(covariance):
                    source_drone_port = int(tracker_id)
                    tracker_id = int(covariance)
                    covariance = None
                    first_seen_time = time.time()

        self.id = t_id
        self.lat = lat
        self.lon = lon
        self.first_seen_time = first_seen_time
        self.last_seen_time = first_seen_time
        self.assigned_drone_port: Optional[int] = None
        self.status = "PENDING"
        self.approved_for_attack = False
        self.tracker_id = tracker_id

        # Lifecycle
        self.track_state = "TENTATIVE"
        self.observation_count = 1
        self.last_observation_time = first_seen_time

        # Ownership
        self.ownership_state = OWNERSHIP_FREE
        self.owner_port: Optional[int] = None
        self.owner_lost_time: Optional[float] = None
        self.last_handoff_time: Optional[float] = None
        self.handoff_candidate_port: Optional[int] = None
        self.handoff_score_accumulator: Dict[int, int] = {}

        # Observation quality
        self.secondary_observations: Dict[int, Dict] = {}
        self.observation_quality: Dict[int, float] = {}

        # Stability
        self.id_switch_count = 0
        self.drop_count = 0
        self.consecutive_obs = 1
        self._edge_accumulator = 0.0

        # Drone-local ID
        self.drone_local_ids: Dict[int, int] = {}
        self.drone_last_seen_time: Dict[int, float] = {}
        self.identity_last_update_time: Dict[int, float] = {}
        if source_drone_port is not None and tracker_id is not None:
            self.drone_local_ids[source_drone_port] = tracker_id
            self.drone_last_seen_time[source_drone_port] = first_seen_time
            self.identity_last_update_time[source_drone_port] = first_seen_time
        self.source_drone_port = source_drone_port

        # Covariance
        self.covariance = covariance if covariance is not None else np.eye(3) * 25.0
        self.observation_sources: Dict[int, Tuple[float, float, float]] = {}
        
        # Grace Period & Coast Mode (Phase 1)
        self.is_coasting = False
        self.coast_start_time: Optional[float] = None
        self.last_velocity: Optional[np.ndarray] = None  # [vx, vy, vz] m/s
        self.coast_distance_traveled = 0.0  # Total coasting distance (meters)
        self.local_id_hysteresis: Dict[int, Dict] = {}

        # Group
        self.is_group = False
        self.group_member_count = 0
        self.group_id_local = None
        self.group_id_by_port: Dict[int, str] = {}
        self.group_member_local_ids: Dict[int, Tuple[int, ...]] = {}
        self.group_count_by_port: Dict[int, int] = {}

        # Assignment time
        self.assignment_time: Optional[float] = None

    # ------------------------------------------------------------------
    # Queries
    # ------------------------------------------------------------------

    def is_immutable(self) -> bool:
        """Is the target protected in the attack pipeline?"""
        return self.status in IMMUTABLE_STATES

    def is_in_attack_pipeline(self) -> bool:
        """Is the target in an active attack phase?"""
        return self.status in {
            "ECHO_WAIT", "CONFIRMED_ATTACK", "CENTERING",
            "ATTACKING", "LOCKED", "ENGAGED"
        }

    def check_confirmation(self, current_time: float, is_drone_searching: bool = True) -> bool:
        """TENTATIVE → CONFIRMED promotion (backward compat).

        Delegates to TargetLifecycle.check_confirmation but satisfies
        legacy test calls to t.check_confirmation(time.time()).
        """
        if self.track_state != "TENTATIVE":
            return False

        time_alive = current_time - self.first_seen_time
        required = (config.TRACK_MIN_OBSERVATIONS_SEARCH
                    if is_drone_searching else config.TRACK_MIN_OBSERVATIONS)

        if time_alive >= config.TRACK_CONFIRMATION_TIME_SEC and self.observation_count >= required:
            self.track_state = "CONFIRMED"
            SwarmLogger.log(
                "LIFECYCLE", "LEADER",
                f"{self.id}: TENTATIVE → CONFIRMED (t={time_alive:.2f}s, obs={self.observation_count})",
                "TRACK",
            )
            return True
        return False

    # ------------------------------------------------------------------
    # Ownership helpers
    # ------------------------------------------------------------------

    def can_handoff(self, cooldown_s: float = 5.0) -> bool:
        """Can the target be handed off?"""
        if self.is_immutable():
            return False
        if self.ownership_state == OWNERSHIP_FREE:
            return True
        if self.last_handoff_time is not None:
            if time.time() - self.last_handoff_time < cooldown_s:
                return False
        return True

    def get_owner_quality(self) -> float:
        """Owner's observation quality score."""
        if self.owner_port is None:
            return 0.0
        return self.observation_quality.get(self.owner_port, 0.0)

    def get_best_secondary(self) -> Tuple[Optional[int], float]:
        """Best secondary observer (port, quality)."""
        if not self.secondary_observations:
            return None, 0.0
        best_port = max(
            self.secondary_observations,
            key=lambda p: self.secondary_observations[p].get("quality", 0.0),
        )
        return best_port, self.secondary_observations[best_port].get("quality", 0.0)

    def record_observation(self, drone_port: int, quality_score: float, is_owner: bool = False):
        """Record observation quality."""
        self.observation_quality[drone_port] = quality_score
        now = time.time()
        self.last_observation_time = now
        self.drone_last_seen_time[drone_port] = now
        if not is_owner:
            self.secondary_observations[drone_port] = {"time": now, "quality": quality_score}
        
        # --- Scoring log ---
        from modules.core.logger import SwarmLogger
        role = "OWNER" if is_owner else "SECONDARY"
        SwarmLogger.log("SCORING", "Target",
            f"{self.id}: D{drone_port}[{role}] quality={quality_score:.2f} | "
            f"obs={self.observation_count} consec={self.consecutive_obs}",
            "QUALITY")

    # ------------------------------------------------------------------
    # Position update
    # ------------------------------------------------------------------

    def update_position(self, lat: float, lon: float, covariance=None, is_edge: bool = False):
        """Update target position and increment observation count."""
        previous_observation_time = self.last_observation_time
        self.lat = lat
        self.lon = lon
        now = time.time()
        self.last_seen_time = now
        self.last_observation_time = now

        if is_edge and self.track_state == "TENTATIVE":
            # Edge detection: 3 frames = 1 point
            self._edge_accumulator += config.SWARM_EDGE_FILTER_FRACTIONAL_CREDIT
            if self._edge_accumulator >= 1.0:
                earned = int(self._edge_accumulator)
                self.observation_count += earned
                self.consecutive_obs += earned
                self._edge_accumulator -= earned
        else:
            self.observation_count += 1
            self.consecutive_obs += 1

        if covariance is not None:
            self.covariance = covariance

        # LOST → CONFIRMED re-acquisition
        if self.track_state == "LOST":
            lost_duration = now - previous_observation_time if previous_observation_time else 0
            self.track_state = "CONFIRMED"
            self.drop_count += 1
            self.consecutive_obs = 1
            SwarmLogger.log("LIFECYCLE", "LEADER",
                f"{self.id}: LOST → CONFIRMED (re-acquired) | "
                f"drops={self.drop_count} lost_for={lost_duration:.1f}s",
                "TRACK")
        if self.status == "LOST":
            self.status = "ACTIVE"

    # ------------------------------------------------------------------
    # Ownership reset
    # ------------------------------------------------------------------

    def reset_ownership(self):
        """Completely reset ownership."""
        self.ownership_state = OWNERSHIP_FREE
        self.owner_port = None
        self.owner_lost_time = None
        self.handoff_candidate_port = None
        self.handoff_score_accumulator.clear()
        self.secondary_observations.clear()

    def has_recent_observer(self, current_time: float, max_age_s: float) -> bool:
        """At least one drone has observed the target recently."""
        return any((current_time - ts) <= max_age_s for ts in self.drone_last_seen_time.values())

    def has_recent_identity(self, drone_port: int, current_time: float, max_age_s: float) -> bool:
        """Whether per-drone local/group identity evidence is still concurrent."""
        ts = self.identity_last_update_time.get(drone_port)
        return ts is not None and (current_time - ts) <= max_age_s

    def prune_stale_observers(self, current_time: float, max_age_s: float):
        """Remove stale per-drone visibility state so local IDs reflect current visibility."""
        stale_ports = [
            port for port, ts in self.drone_last_seen_time.items()
            if (current_time - ts) > max_age_s
        ]
        for port in stale_ports:
            self.drone_last_seen_time.pop(port, None)
            self.identity_last_update_time.pop(port, None)
            self.drone_local_ids.pop(port, None)
            self.group_id_by_port.pop(port, None)
            self.group_member_local_ids.pop(port, None)
            self.group_count_by_port.pop(port, None)
            self.secondary_observations.pop(port, None)
            self.observation_quality.pop(port, None)
            self.local_id_hysteresis.pop(port, None)
        self.refresh_group_summary()

    def refresh_group_summary(self):
        """Recompute group summary from current per-drone evidence."""
        if not self.is_group:
            return

        if self.group_count_by_port:
            self.group_member_count = max(
                int(count or 0) for count in self.group_count_by_port.values()
            )
        elif self.group_member_local_ids:
            self.group_member_count = max(
                len(member_ids or ()) for member_ids in self.group_member_local_ids.values()
            )

        if self.group_member_count <= 0 and not self.group_member_local_ids:
            self.group_member_count = 0


# ======================================================================
# Target Registry — thread-safe dict wrapper
# ======================================================================

class TargetRegistry:
    """Thread-safe target store. All target CRUD operations go through here."""

    def __init__(self):
        self._targets: Dict[str, TrackedTarget] = {}
        self._counter = 0
        self._lock = threading.RLock()
        # Alias mapping: merged_id → canonical_id
        self._aliases: Dict[str, str] = {}
        # Alias TTL: merged_id → {"ts": float, "canonical": str}
        self._alias_ttl: Dict[str, Dict] = {}

    @property
    def lock(self) -> threading.RLock:
        """Lock access for external modules."""
        return self._lock

    # ------------------------------------------------------------------
    # CRUD
    # ------------------------------------------------------------------

    def generate_id(self) -> str:
        """Generate a new unique target ID."""
        with self._lock:
            self._counter += 1
            return f"T{self._counter}"

    def add(self, target: TrackedTarget):
        """Add target to registry."""
        with self._lock:
            self._targets[target.id] = target

    def get(self, t_id: str) -> Optional[TrackedTarget]:
        """Get target by ID. Redirect to canonical if alias exists."""
        with self._lock:
            canonical = self._aliases.get(t_id, t_id)
            return self._targets.get(canonical)

    def remove(self, t_id: str) -> Optional[TrackedTarget]:
        """Remove target from registry."""
        with self._lock:
            return self._targets.pop(t_id, None)

    def contains(self, t_id: str) -> bool:
        with self._lock:
            canonical = self._aliases.get(t_id, t_id)
            return canonical in self._targets

    def all_targets(self) -> Dict[str, TrackedTarget]:
        """Copy of all targets (for iteration safety)."""
        with self._lock:
            return dict(self._targets)

    def items(self):
        """Direct dict items access (do not use outside lock!)."""
        return self._targets.items()

    def values(self):
        return self._targets.values()

    def __len__(self) -> int:
        return len(self._targets)

    def __contains__(self, t_id: str) -> bool:
        return self.contains(t_id)

    # ------------------------------------------------------------------
    # Alias management
    # ------------------------------------------------------------------

    def add_alias(self, alias_id: str, canonical_id: str):
        """Add alias for a merged target."""
        with self._lock:
            self._aliases[alias_id] = canonical_id
            self._alias_ttl[alias_id] = {"ts": time.time(), "canonical": canonical_id}

    def resolve_alias(self, t_id: str) -> str:
        """Convert alias to canonical ID if exists."""
        with self._lock:
            return self._aliases.get(t_id, t_id)

    def is_alias(self, t_id: str) -> bool:
        with self._lock:
            return t_id in self._alias_ttl

    def cleanup_aliases(self, ttl_s: float = 1.0):
        """Clean up expired aliases."""
        now = time.time()
        with self._lock:
            expired = [k for k, v in self._alias_ttl.items() if now - v["ts"] > ttl_s]
            for k in expired:
                self._alias_ttl.pop(k, None)
                self._aliases.pop(k, None)
