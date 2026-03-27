"""Target model, identity, lifecycle, and registry services."""

import re
import time
import threading
import numpy as np
from typing import Callable, Dict, List, Optional, Tuple

import config
from modules.core.geo_math import GeoMath
from modules.core.logger import SwarmLogger
from modules.core.comm import (
    CanonicalTargetState,
    canonical_target_engagement_state,
    canonical_target_subphase,
    canonical_target_track_state,
)

# Shared immutable attack states.
IMMUTABLE_STATES = frozenset({
    "VERIFYING", "ASSIGNED", "READY",
    "STAGING", "CENTERING", "EXECUTING", "DIVING"
})

# Ownership states.
OWNERSHIP_FREE = "FREE"
OWNERSHIP_OWNED = "OWNED"
OWNERSHIP_RESERVED = "RESERVED"
OWNERSHIP_HANDOFF = "HANDOFF_PENDING"


class TrackedTarget:
    """State container for a tracked target."""

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
        # Stability
        "id_switch_count", "drop_count", "consecutive_obs", "_edge_accumulator",
        # Per-drone identity
        "drone_local_ids", "source_drone_port",
        "drone_last_seen_time", "identity_last_update_time", "identity_change_time",
        "last_bbox_by_port", "bbox_last_update_time",
        # Covariance and sources
        "covariance", "observation_sources", "local_id_hysteresis",
        # Group state
        "is_group", "group_member_count", "group_id_local", "group_id_by_port",
        "group_count_by_port", "group_member_local_ids",
        # Assignment timing
        "assignment_time",
        # Coasting
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

        # Per-drone identity
        self.drone_local_ids: Dict[int, int] = {}
        self.drone_last_seen_time: Dict[int, float] = {}
        self.identity_last_update_time: Dict[int, float] = {}
        self.identity_change_time: Dict[int, float] = {}
        self.last_bbox_by_port: Dict[int, Tuple[float, float, float, float]] = {}
        self.bbox_last_update_time: Dict[int, float] = {}
        if source_drone_port is not None and tracker_id is not None:
            self.drone_local_ids[source_drone_port] = tracker_id
            self.drone_last_seen_time[source_drone_port] = first_seen_time
            self.identity_last_update_time[source_drone_port] = first_seen_time
            self.identity_change_time[source_drone_port] = first_seen_time
        self.source_drone_port = source_drone_port

        # Covariance
        self.covariance = covariance if covariance is not None else np.eye(3) * 25.0
        self.observation_sources: Dict[int, Tuple[float, float, float]] = {}

        # Coasting
        self.is_coasting = False
        self.coast_start_time: Optional[float] = None
        self.last_velocity: Optional[np.ndarray] = None  # [vx, vy, vz] m/s
        self.coast_distance_traveled = 0.0  # Total predicted travel in meters.
        self.local_id_hysteresis: Dict[int, Dict] = {}

        # Group state
        self.is_group = False
        self.group_member_count = 0
        self.group_id_local = None
        self.group_id_by_port: Dict[int, str] = {}
        self.group_count_by_port: Dict[int, int] = {}
        self.group_member_local_ids: Dict[int, Tuple[int, ...]] = {}

        # Assignment timing
        self.assignment_time: Optional[float] = None

    # Query helpers

    def is_immutable(self) -> bool:
        """Whether attack-stage state blocks mutation."""
        return self.status in IMMUTABLE_STATES

    def is_in_attack_pipeline(self) -> bool:
        """Whether the target is in an active attack stage."""
        return self.status in {
            "ASSIGNED", "READY", "STAGING", "CENTERING",
            "EXECUTING", "VERIFYING"
        }

    def canonical_state(self) -> CanonicalTargetState:
        return CanonicalTargetState(
            track=canonical_target_track_state(self.track_state, is_coasting=self.is_coasting),
            engagement=canonical_target_engagement_state(self.status, approved_for_attack=self.approved_for_attack),
            subphase=canonical_target_subphase(self.status),
        )

    def check_confirmation(self, current_time: float, is_drone_searching: bool = True) -> bool:
        """Promote a tentative target when confirmation rules are met."""
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

    # Ownership helpers

    def can_handoff(self, cooldown_s: float = 5.0) -> bool:
        """Whether ownership may be handed off."""
        if self.is_immutable():
            return False
        if self.ownership_state == OWNERSHIP_FREE:
            return True
        if self.last_handoff_time is not None:
            if time.time() - self.last_handoff_time < cooldown_s:
                return False
        return True

    def get_owner_quality(self) -> float:
        """Return the owner observation quality."""
        if self.owner_port is None:
            return 0.0
        return self.observation_quality.get(self.owner_port, 0.0)

    def get_best_secondary(self) -> Tuple[Optional[int], float]:
        """Return the best secondary observer as ``(port, quality)``."""
        if not self.secondary_observations:
            return None, 0.0
        best_port = max(
            self.secondary_observations,
            key=lambda p: self.secondary_observations[p].get("quality", 0.0),
        )
        return best_port, self.secondary_observations[best_port].get("quality", 0.0)

    def record_observation(self, drone_port: int, quality_score: float, is_owner: bool = False):
        """Record per-drone observation quality."""
        self.observation_quality[drone_port] = quality_score
        now = time.time()
        self.last_observation_time = now
        self.drone_last_seen_time[drone_port] = now
        if not is_owner:
            self.secondary_observations[drone_port] = {"time": now, "quality": quality_score}
        role = "OWNER" if is_owner else "SECONDARY"
        SwarmLogger.log("OBS_QUALITY", "Target",
            f"{self.id}: D{drone_port}[{role}] quality={quality_score:.2f} | "
            f"obs={self.observation_count} consec={self.consecutive_obs}",
            "QUALITY")

    # Position updates

    def update_position(self, lat: float, lon: float, covariance=None, is_edge: bool = False):
        """Update position and observation counters."""
        previous_observation_time = self.last_observation_time
        self.lat = lat
        self.lon = lon
        now = time.time()
        self.last_seen_time = now
        self.last_observation_time = now

        if is_edge and self.track_state == "TENTATIVE":
            # Edge sightings count fractionally until they sum to one full hit.
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

        # Reacquire a lost track.
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

    # Ownership reset

    def reset_ownership(self):
        """Clear all ownership state."""
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
            self.identity_change_time.pop(port, None)
            self.drone_local_ids.pop(port, None)
            self.group_id_by_port.pop(port, None)
            self.group_count_by_port.pop(port, None)
            self.group_member_local_ids.pop(port, None)
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
        if self.group_member_count <= 0:
            self.group_member_count = 0


# Registry

class TargetRegistry:
    """Thread-safe target store."""

    def __init__(self):
        self._targets: Dict[str, TrackedTarget] = {}
        self._counter = 0
        self._lock = threading.RLock()
        # merged_id -> canonical_id
        self._aliases: Dict[str, str] = {}
        # merged_id -> {"ts": float, "canonical": str}
        self._alias_ttl: Dict[str, Dict] = {}

    @property
    def lock(self) -> threading.RLock:
        """Expose the registry lock to other swarm services."""
        return self._lock

    # CRUD

    def generate_id(self) -> str:
        """Return a new target ID."""
        with self._lock:
            self._counter += 1
            return f"T{self._counter}"

    def add(self, target: TrackedTarget):
        """Store a target."""
        with self._lock:
            self._targets[target.id] = target

    def get(self, t_id: str) -> Optional[TrackedTarget]:
        """Fetch a target and resolve aliases."""
        with self._lock:
            canonical = self._aliases.get(t_id, t_id)
            return self._targets.get(canonical)

    def remove(self, t_id: str) -> Optional[TrackedTarget]:
        """Remove a target from the registry."""
        with self._lock:
            return self._targets.pop(t_id, None)

    def contains(self, t_id: str) -> bool:
        with self._lock:
            canonical = self._aliases.get(t_id, t_id)
            return canonical in self._targets

    def all_targets(self) -> Dict[str, TrackedTarget]:
        """Return a copy for safe iteration."""
        with self._lock:
            return dict(self._targets)

    def items(self):
        """Return raw dict items. Use under lock."""
        return self._targets.items()

    def values(self):
        return self._targets.values()

    def __len__(self) -> int:
        return len(self._targets)

    def __contains__(self, t_id: str) -> bool:
        return self.contains(t_id)

    # Alias handling

    def add_alias(self, alias_id: str, canonical_id: str):
        """Register an alias for a merged target."""
        with self._lock:
            self._aliases[alias_id] = canonical_id
            self._alias_ttl[alias_id] = {"ts": time.time(), "canonical": canonical_id}

    def resolve_alias(self, t_id: str) -> str:
        """Resolve an alias to its canonical ID."""
        with self._lock:
            return self._aliases.get(t_id, t_id)

    def is_alias(self, t_id: str) -> bool:
        with self._lock:
            return t_id in self._alias_ttl

    def cleanup_aliases(self, ttl_s: float = 1.0):
        """Drop expired aliases."""
        now = time.time()
        with self._lock:
            expired = [k for k, v in self._alias_ttl.items() if now - v["ts"] > ttl_s]
            for k in expired:
                self._alias_ttl.pop(k, None)
                self._aliases.pop(k, None)

    def reset_runtime_state(self) -> None:
        with self._lock:
            self._targets.clear()
            self._aliases.clear()
            self._alias_ttl.clear()
            self._counter = 0


class LocalIdentityIndex:
    """Maps per-drone local tracker IDs to canonical target IDs."""

    def __init__(self):
        self._index: Dict[Tuple[int, int], Dict] = {}
        self._id_history: Dict[Tuple[int, str], Dict] = {}

    def update(self, drone_port: int, tracker_id: int, target_id: str,
               now: Optional[float] = None):
        if drone_port is None or tracker_id is None or not target_id:
            return
        now = now if now is not None else time.time()
        try:
            key = (int(drone_port), int(tracker_id))
        except (ValueError, TypeError):
            return
        self._index[key] = {"tid": target_id, "ts": now}

    def lookup(self, drone_port: int, tracker_id: int) -> Optional[str]:
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

    def cleanup(self, now: Optional[float] = None, active_targets=None):
        now = now if now is not None else time.time()
        ttl = config.SWARM_LOCAL_ID_INDEX_TTL_S
        if ttl <= 0:
            return

        to_del = []
        for key, value in self._index.items():
            try:
                ts = float(value.get("ts", 0.0))
                tid = value.get("tid")
                if (now - ts) > ttl:
                    to_del.append(key)
                elif active_targets is not None and tid and tid not in active_targets:
                    to_del.append(key)
            except (ValueError, TypeError):
                to_del.append(key)

        for key in to_del:
            self._index.pop(key, None)

    def log_id_update(self, drone_port: int, target_id: str, old_id: int, new_id: int):
        key = (drone_port, target_id)
        hist = self._id_history.get(key)
        if hist is not None and hist.get("prev") == new_id and hist.get("cur") == old_id:
            self._id_history[key] = {"prev": old_id, "cur": new_id}
            return

        self._id_history[key] = {"prev": old_id, "cur": new_id}
        SwarmLogger.log_keyed_debounce(
            f"id_update:{drone_port}:{target_id}",
            "ID_UPDATE", "LEADER",
            f"{target_id}: Drone_{drone_port} LocalID {old_id}→{new_id}",
            "ID", interval_s=15.0,
        )

    def snapshot(self) -> Dict:
        return dict(self._index)

    def reset_runtime_state(self) -> None:
        self._index.clear()
        self._id_history.clear()


def targets_share_identity_evidence(
    t1: TrackedTarget,
    t2: TrackedTarget,
    current_time: Optional[float] = None,
) -> bool:
    """Return True when two targets share direct same-family identity evidence."""
    current_time = current_time if current_time is not None else time.time()
    common_ports = set(getattr(t1, "drone_local_ids", {})).intersection(
        set(getattr(t2, "drone_local_ids", {}))
    )
    for port in common_ports:
        if not (
            t1.has_recent_identity(port, current_time, config.TRACK_IDENTITY_CONFLICT_WINDOW_SEC)
            and t2.has_recent_identity(port, current_time, config.TRACK_IDENTITY_CONFLICT_WINDOW_SEC)
        ):
            continue

        members1 = set(int(mid) for mid in (getattr(t1, "group_member_local_ids", {}).get(port, ()) or ()))
        members2 = set(int(mid) for mid in (getattr(t2, "group_member_local_ids", {}).get(port, ()) or ()))
        if members1 and members2 and (members1 & members2):
            return True

        gid1 = getattr(t1, "group_id_by_port", {}).get(port)
        gid2 = getattr(t2, "group_id_by_port", {}).get(port)
        if gid1 is not None and gid2 is not None and str(gid1) == str(gid2):
            return True

        if getattr(t1, "is_group", False) and getattr(t2, "is_group", False):
            lid1 = getattr(t1, "drone_local_ids", {}).get(port)
            lid2 = getattr(t2, "drone_local_ids", {}).get(port)
            if lid1 is not None and members2 and int(lid1) in members2:
                return True
            if lid2 is not None and members1 and int(lid2) in members1:
                return True
            continue

        lid1 = getattr(t1, "drone_local_ids", {}).get(port)
        lid2 = getattr(t2, "drone_local_ids", {}).get(port)
        if lid1 is not None and lid2 is not None and int(lid1) == int(lid2):
            return True

    return False


def targets_have_conflicting_identity(
    t1: TrackedTarget,
    t2: TrackedTarget,
    current_time: Optional[float] = None,
) -> bool:
    """Return True when identity evidence shows two targets are distinct."""
    current_time = current_time if current_time is not None else time.time()
    common_ports = set(getattr(t1, "drone_local_ids", {})).intersection(
        set(getattr(t2, "drone_local_ids", {}))
    )
    if not common_ports:
        return False

    for port in common_ports:
        if not (
            t1.has_recent_identity(port, current_time, config.TRACK_IDENTITY_CONFLICT_WINDOW_SEC)
            and t2.has_recent_identity(port, current_time, config.TRACK_IDENTITY_CONFLICT_WINDOW_SEC)
        ):
            continue

        members1 = set(int(mid) for mid in (getattr(t1, "group_member_local_ids", {}).get(port, ()) or ()))
        members2 = set(int(mid) for mid in (getattr(t2, "group_member_local_ids", {}).get(port, ()) or ()))
        if members1 and members2 and (members1 & members2):
            continue

        gid1 = getattr(t1, "group_id_by_port", {}).get(port)
        gid2 = getattr(t2, "group_id_by_port", {}).get(port)
        if gid1 is not None and gid2 is not None and str(gid1) == str(gid2):
            continue

        count1 = int(getattr(t1, "group_count_by_port", {}).get(port, 0) or 0)
        count2 = int(getattr(t2, "group_count_by_port", {}).get(port, 0) or 0)
        if (
            getattr(t1, "is_group", False)
            and getattr(t2, "is_group", False)
            and not group_counts_compatible(count1, count2)
        ):
            return True

        if getattr(t1, "is_group", False) and getattr(t2, "is_group", False):
            continue

        lid1 = getattr(t1, "drone_local_ids", {}).get(port)
        lid2 = getattr(t2, "drone_local_ids", {}).get(port)
        if lid1 is not None and lid2 is not None and int(lid1) != int(lid2):
            return True

    return False


def groups_compatible(
    t1: TrackedTarget,
    t2: TrackedTarget,
    current_time: Optional[float] = None,
) -> bool:
    """Return True when two targets may be merged as the same group."""
    current_time = current_time if current_time is not None else time.time()
    if targets_have_conflicting_identity(t1, t2, current_time):
        return False
    if getattr(t1, "is_group", False) or getattr(t2, "is_group", False):
        if getattr(t1, "is_group", False) != getattr(t2, "is_group", False):
            return False
        count1 = int(getattr(t1, "group_member_count", 0) or 0)
        count2 = int(getattr(t2, "group_member_count", 0) or 0)
        if not group_counts_compatible(count1, count2):
            return False
    return True


def target_family_signature(target: TrackedTarget) -> Tuple:
    """Return a stable family signature for dedupe and attack guards."""
    if not getattr(target, "is_group", False):
        return ("single", getattr(target, "id", None))

    member_tokens = []
    for port, members in sorted(getattr(target, "group_member_local_ids", {}).items()):
        if members:
            member_tokens.append((int(port), tuple(sorted(int(mid) for mid in members))))
    if member_tokens:
        return ("group_members", tuple(member_tokens))

    group_tokens = []
    for port, gid in sorted(getattr(target, "group_id_by_port", {}).items()):
        if gid is not None:
            group_tokens.append((int(port), str(gid)))
    if group_tokens:
        return ("group", tuple(group_tokens))

    local_gid = getattr(target, "group_id_local", None)
    if local_gid is not None:
        return ("group_local", (str(local_gid),))

    return (
        "group_fallback",
        int(getattr(target, "group_member_count", 0) or 0),
        round(float(getattr(target, "lat", 0.0) or 0.0), 6),
        round(float(getattr(target, "lon", 0.0) or 0.0), 6),
    )


def extract_group_member_ids(raw_data) -> set:
    """Extract stable local member ids from a group payload."""
    member_ids = set()
    if not isinstance(raw_data, dict):
        return member_ids
    for mid in raw_data.get("member_ids", []) or []:
        try:
            member_ids.add(int(mid))
        except (TypeError, ValueError):
            continue
    return member_ids


def group_counts_compatible(count1: int, count2: int, max_diff: int = 1) -> bool:
    """Return True when two group sizes are close enough to describe one physical group."""
    count1 = int(count1 or 0)
    count2 = int(count2 or 0)
    if count1 <= 0 or count2 <= 0:
        return True
    return abs(count1 - count2) <= max_diff


def same_drone_group_continuity_ok(
    target: TrackedTarget,
    drone_port: int,
    lat: float,
    lon: float,
    incoming_count: int,
    current_time: float,
    max_dist_m: float,
    max_age_s: float = 3.5,
) -> bool:
    """Return True when a same-drone group can be continued by spatial continuity alone."""
    if not getattr(target, "is_group", False):
        return False
    if target.track_state == "DELETED" or target.lat is None or target.lon is None:
        return False
    seen_ts = getattr(target, "drone_last_seen_time", {}).get(drone_port)
    if seen_ts is None or (current_time - float(seen_ts)) > max_age_s:
        return False
    target_count = int(getattr(target, "group_count_by_port", {}).get(drone_port, target.group_member_count) or 0)
    if not group_counts_compatible(incoming_count, target_count):
        return False
    try:
        dist = GeoMath.haversine_distance(lat, lon, target.lat, target.lon)
    except Exception:
        return False
    allowed_dist = max(float(max_dist_m), 12.0 if target_count >= 3 or incoming_count >= 3 else 9.0)
    return dist <= allowed_dist


def attack_family_exclusion_radius(
    t1: TrackedTarget,
    t2: TrackedTarget,
) -> float:
    """Return minimum separation for attack-family deconfliction."""
    if getattr(t1, "is_group", False) and getattr(t2, "is_group", False):
        if targets_share_identity_evidence(t1, t2):
            return max(config.DUPLICATE_MERGE_DISTANCE_M, 14.0)
        return max(10.0, config.DUPLICATE_MERGE_DISTANCE_M * 0.75)
    if getattr(t1, "is_group", False) or getattr(t2, "is_group", False):
        return max(config.DUPLICATE_MERGE_DISTANCE_M, 14.0)
    if len(getattr(t1, "drone_local_ids", {})) > 1 or len(getattr(t2, "drone_local_ids", {})) > 1:
        return max(config.DUPLICATE_MERGE_DISTANCE_M, 14.0)
    return config.DUPLICATE_MERGE_DISTANCE_M


def targets_attack_compatible(
    t1: TrackedTarget,
    t2: TrackedTarget,
) -> bool:
    """Return True when two targets may belong to the same attack family."""
    if targets_have_conflicting_identity(t1, t2):
        return False
    if targets_share_identity_evidence(t1, t2):
        return True
    if getattr(t1, "is_group", False) and getattr(t2, "is_group", False):
        return True
    if getattr(t1, "is_group", False) or getattr(t2, "is_group", False):
        return True
    return len(getattr(t1, "drone_local_ids", {})) > 1 or len(getattr(t2, "drone_local_ids", {})) > 1


def merge_target_port_identity(keep_t: TrackedTarget, drop_t: TrackedTarget,
                               port: int, drop_lid: Optional[int]) -> Optional[int]:
    """Merge per-port identity state without clobbering a stronger lock."""
    keep_seen = float(keep_t.drone_last_seen_time.get(port, 0.0) or 0.0)
    drop_seen = float(drop_t.drone_last_seen_time.get(port, 0.0) or 0.0)
    keep_ident = float(keep_t.identity_last_update_time.get(port, 0.0) or 0.0)
    drop_ident = float(drop_t.identity_last_update_time.get(port, 0.0) or 0.0)
    keep_lid = keep_t.drone_local_ids.get(port)

    preserve_keep = (
        keep_lid is not None and keep_lid != drop_lid
        and (keep_t.assigned_drone_port == port or keep_t.owner_port == port
             or keep_ident >= drop_ident or keep_seen >= drop_seen)
    )
    if not preserve_keep and drop_lid is not None:
        keep_t.drone_local_ids[port] = drop_lid

    keep_t.drone_last_seen_time[port] = max(keep_seen, drop_seen)
    keep_t.identity_last_update_time[port] = max(keep_ident, drop_ident)

    if keep_t.assigned_drone_port == port and keep_t.drone_local_ids.get(port) is not None:
        keep_t.tracker_id = keep_t.drone_local_ids.get(port)

    return keep_t.drone_local_ids.get(port)


def absorb_merged_target_state(keep_t: TrackedTarget, drop_t: TrackedTarget) -> None:
    """Move target identity and group state from merged target into canonical target."""
    for port, count in getattr(drop_t, "group_count_by_port", {}).items():
        keep_t.group_count_by_port[port] = max(
            int(keep_t.group_count_by_port.get(port, 0) or 0),
            int(count or 0),
        )

    for port, gid in getattr(drop_t, "group_id_by_port", {}).items():
        if gid is not None and (
            port not in keep_t.group_id_by_port
            or drop_t.identity_last_update_time.get(port, 0.0)
            >= keep_t.identity_last_update_time.get(port, 0.0)
        ):
            keep_t.group_id_by_port[port] = gid

    if drop_t.is_group:
        keep_t.is_group = True
        keep_t.group_member_count = max(
            int(keep_t.group_member_count or 0),
            int(drop_t.group_member_count or 0),
        )
        preferred = keep_t.assigned_drone_port or keep_t.owner_port or drop_t.assigned_drone_port
        if preferred in keep_t.group_id_by_port:
            keep_t.group_id_local = keep_t.group_id_by_port[preferred]
        elif keep_t.group_id_local is None:
            keep_t.group_id_local = drop_t.group_id_local

    keep_t.refresh_group_summary()


class TargetLifecycle:
    """Runs target confirmation and prune transitions."""

    def __init__(self, registry: TargetRegistry):
        self._registry = registry

    def check_confirmation(self, target: TrackedTarget, current_time: float,
                           is_searching: bool = True) -> bool:
        if target.track_state != "TENTATIVE":
            return False

        time_alive = current_time - target.first_seen_time
        required = (
            config.TRACK_MIN_OBSERVATIONS_SEARCH
            if is_searching else config.TRACK_MIN_OBSERVATIONS
        )

        if time_alive >= config.TRACK_CONFIRMATION_TIME_SEC and target.observation_count >= required:
            target.track_state = "CONFIRMED"
            SwarmLogger.log(
                "LIFECYCLE", "LEADER",
                f"{target.id}: TENTATIVE → CONFIRMED (t={time_alive:.2f}s, obs={target.observation_count})",
                "TRACK",
            )
            return True
        return False

    def prune(
        self,
        current_time: float,
        attack_mode_active: bool,
        drone_connected_check: Callable[[int], bool],
        release_drone: Callable[[int], None],
        remove_fusion_target: Callable[[str], None],
    ) -> List[str]:
        to_delete: List[str] = []

        with self._registry.lock:
            for t_id, target in list(self._registry.items()):
                target.prune_stale_observers(current_time, config.TRACK_OBSERVER_STALE_TIMEOUT_SEC)
                time_since = current_time - target.last_observation_time

                if target.track_state == "TENTATIVE":
                    self._prune_tentative(
                        target, t_id, time_since, current_time, attack_mode_active, to_delete
                    )
                    continue

                if target.track_state == "CONFIRMED":
                    self._prune_confirmed(
                        target, t_id, time_since, drone_connected_check, release_drone, to_delete
                    )
                    continue

                if target.track_state == "LOST" and time_since > config.TRACK_DELETE_TIMEOUT_SEC:
                    SwarmLogger.log(
                        "LIFECYCLE", "LEADER",
                        f"{t_id}: LOST → DELETED (NoObs={time_since:.1f}s)", "TRACK",
                    )
                    if target.assigned_drone_port:
                        release_drone(target.assigned_drone_port)
                    to_delete.append(t_id)

            for t_id in to_delete:
                remove_fusion_target(t_id)
                self._registry.remove(t_id)

        return to_delete

    def _prune_tentative(self, target: TrackedTarget, t_id: str,
                         time_since: float, current_time: float,
                         attack_mode_active: bool, to_delete: List[str]):
        self.check_confirmation(
            target, current_time, is_searching=not attack_mode_active
        )
        if target.track_state == "TENTATIVE" and time_since > config.TRACK_TENTATIVE_TIMEOUT_SEC:
            if target.has_recent_observer(current_time, config.TRACK_OBSERVER_STALE_TIMEOUT_SEC):
                return
            SwarmLogger.log(
                "LIFECYCLE", "LEADER",
                f"TENTATIVE DELETED: {t_id} | NoObs={time_since:.2f}s", "LIFECYCLE",
            )
            to_delete.append(t_id)

    def _prune_confirmed(self, target: TrackedTarget, t_id: str,
                         time_since: float,
                         drone_connected_check: Callable[[int], bool],
                         release_drone: Callable[[int], None],
                         to_delete: List[str]):
        is_protected = (
            target.approved_for_attack
            or (config.ENABLE_ATTACK_IMMUTABILITY and target.status in IMMUTABLE_STATES)
        )
        terminal_attack_bypass = (
            target.assigned_drone_port is not None
            and (target.status in IMMUTABLE_STATES or target.approved_for_attack)
        )

        if is_protected and target.assigned_drone_port is not None:
            if not drone_connected_check(target.assigned_drone_port):
                SwarmLogger.log(
                    "PRUNE", "LEADER",
                    f"GHOST RELEASE: {t_id} | Drone_{target.assigned_drone_port} disconnected",
                    "PRUNE",
                )
                release_drone(target.assigned_drone_port)
                return

            if time_since > config.TARGET_PRUNE_TIME_S:
                if terminal_attack_bypass:
                    SwarmLogger.log(
                        "GUARD", "LEADER",
                        f"TERMINAL ATTACK BYPASS: {t_id} | NoObs={time_since:.1f}s | "
                        f"Status={target.status} | Drone={target.assigned_drone_port}",
                        "GUARD",
                    )
                    return
                SwarmLogger.log(
                    "PRUNE", "LEADER",
                    f"TARGET DELETED (Stuck Ghost): {t_id} | NoObs={time_since:.1f}s | "
                    f"Status={target.status} | Drone={target.assigned_drone_port}",
                    "PRUNE",
                )
                to_delete.append(t_id)
            elif time_since > config.ATTACK_STALE_PIPELINE_TIMEOUT_S:
                if terminal_attack_bypass:
                    SwarmLogger.log(
                        "GUARD", "LEADER",
                        f"TERMINAL ATTACK BYPASS: {t_id} | NoObs={time_since:.1f}s | "
                        f"Status={target.status} | Drone={target.assigned_drone_port}",
                        "GUARD",
                    )
                    return
                SwarmLogger.log(
                    "PRUNE", "LEADER",
                    f"STALE {target.status}: {t_id} | NoObs={time_since:.1f}s → LOST",
                    "PRUNE",
                )
                target.track_state = "LOST"
                target.status = "LOST"
                release_drone(target.assigned_drone_port)
            elif time_since > config.TARGET_COAST_TIME_S:
                SwarmLogger.log(
                    "GUARD", "LEADER",
                    f"PRUNE GUARD: {t_id} is {target.status} — kept (NoObs={time_since:.1f}s)",
                    "GUARD",
                )
            return

        if time_since < 0.1 and target.is_coasting:
            target.is_coasting = False
            target.coast_distance_traveled = 0.0
            target.coast_start_time = None

        observation_gap_started = time_since >= 0.25
        in_grace_period = observation_gap_started and time_since < config.TARGET_GRACE_PERIOD_S
        if in_grace_period:
            if not target.is_coasting:
                target.is_coasting = True
                target.coast_start_time = time.time()
                SwarmLogger.log(
                    "GRACE", "LEADER",
                    f"{t_id}: GRACE PERIOD STARTED - Coasting (NoObs={time_since:.1f}s)",
                    "GRACE",
                )

            if target.last_velocity is not None:
                vel_mag = np.linalg.norm(target.last_velocity)
                if vel_mag > config.TARGET_MAX_COAST_VELOCITY:
                    target.last_velocity = target.last_velocity * (config.TARGET_MAX_COAST_VELOCITY / vel_mag)

            if target.coast_distance_traveled > config.TARGET_MAX_COAST_DISPLACEMENT_M:
                SwarmLogger.log(
                    "STALE", "LEADER",
                    f"{t_id}: COAST DISPLACEMENT EXCEEDED ({target.coast_distance_traveled:.1f}m) - Forcing LOST",
                    "STALE",
                )
                target.track_state = "LOST"
                target.status = "LOST"
                target.is_coasting = False
                if target.assigned_drone_port:
                    release_drone(target.assigned_drone_port)
                return
            return

        if target.is_coasting:
            target.is_coasting = False
            SwarmLogger.log(
                "GRACE", "LEADER",
                f"{t_id}: GRACE PERIOD ENDED (NoObs={time_since:.1f}s)",
                "GRACE",
            )

        if time_since > config.TARGET_STALE_COAST_S:
            SwarmLogger.log(
                "STALE", "LEADER",
                f"{t_id}: STALE TARGET - Forcing LOST (NoObs={time_since:.1f}s)",
                "STALE",
            )
            target.track_state = "LOST"
            target.status = "LOST"
            if target.assigned_drone_port:
                release_drone(target.assigned_drone_port)
            return

        if time_since >= config.TARGET_GRACE_PERIOD_S and target.last_velocity is not None:
            vel_mag = np.linalg.norm(target.last_velocity)
            if vel_mag > config.TARGET_STALE_VELOCITY_THRESHOLD:
                SwarmLogger.log(
                    "STALE", "LEADER",
                    f"{t_id}: STALE VELOCITY ({vel_mag:.1f} m/s) - Forcing LOST",
                    "STALE",
                )
                target.track_state = "LOST"
                target.status = "LOST"
                if target.assigned_drone_port:
                    release_drone(target.assigned_drone_port)
                return

        if time_since > config.TRACK_LOST_TIMEOUT_SEC:
            target.track_state = "LOST"
            target.status = "LOST"
            SwarmLogger.log(
                "LIFECYCLE", "LEADER",
                f"{t_id}: CONFIRMED → LOST (NoObs={time_since:.1f}s)", "TRACK",
            )


class TargetProcessor:
    """Handles inbound target association and state mutation."""

    def __init__(self, owner):
        self.owner = owner

    def process_report(
        self,
        drone_port,
        lat,
        lon,
        confidence,
        tracker_id=None,
        raw_data=None,
        covariance=None,
    ):
        from modules.swarm.target_fusion import should_fuse_targets, sigma_from_cov

        raw_data = self.normalize_group_report(drone_port, tracker_id, raw_data)
        if isinstance(raw_data, dict) and tracker_id is None:
            tracker_id = raw_data.get("track_id")
        if raw_data != "HEARTBEAT" and confidence < config.SWARM_ATTACK_MIN_CONFIDENCE:
            return {"action": "SEARCH", "message": "Low Confidence"}

        with self.owner.lock:
            self.owner.identity.cleanup(active_targets=self.owner.targets)

            if isinstance(raw_data, dict) and "action" in raw_data:
                return self.owner.link.dispatch_inbound(drone_port, raw_data)

            if raw_data == "HEARTBEAT":
                return self.owner._handle_heartbeat(drone_port)

            final_lat, final_lon, is_edge = self.leader_verify(lat, lon, raw_data)
            current_time = time.time()

            self.owner.lifecycle.prune(
                current_time,
                self.owner.attack_mode_active,
                drone_connected_check=self.owner._is_drone_connected,
                release_drone=self.owner.ownership.release,
                remove_fusion_target=self.owner.fusion.ekf_manager.remove_target,
            )

            best_id, match_reason = self.match_target(
                drone_port, final_lat, final_lon, tracker_id, covariance, raw_data, should_fuse_targets, sigma_from_cov
            )

            if best_id:
                target_id = self.update_existing_target(
                    best_id, drone_port, final_lat, final_lon, tracker_id,
                    covariance, raw_data, is_edge, current_time,
                )
            else:
                target_id = self.create_new_target(
                    drone_port, final_lat, final_lon, tracker_id,
                    covariance, raw_data, is_edge, current_time, sigma_from_cov,
                )

            if not target_id or target_id not in self.owner.targets:
                return {"action": "SEARCH", "message": "Target lost"}

            self.record_fusion(target_id, final_lat, final_lon, covariance, drone_port, confidence)

            target = self.owner.targets[target_id]
            if target.track_state == "TENTATIVE":
                redirect = self.tentative_redirect(
                    target_id, drone_port, final_lat, final_lon, tracker_id, covariance, raw_data
                )
                if redirect:
                    return redirect

            return self.build_report_response(target, target_id, drone_port)

    @staticmethod
    def normalize_group_report(drone_port: int, tracker_id, raw_data):
        if not isinstance(raw_data, dict) or "action" in raw_data:
            return raw_data
        normalized = dict(raw_data)
        normalized["is_group"] = True
        if normalized.get("track_id") is None and tracker_id is not None:
            normalized["track_id"] = tracker_id
        normalized["group_member_count"] = max(1, int(normalized.get("group_member_count", 0) or 0))
        member_ids = []
        for mid in normalized.get("member_ids", []) or []:
            try:
                member_ids.append(int(mid))
            except (TypeError, ValueError):
                continue
        normalized["member_ids"] = sorted(set(member_ids))
        return normalized

    @staticmethod
    def leader_verify(lat, lon, raw_data):
        final_lat, final_lon = lat, lon
        is_edge = False
        try:
            is_edge = bool(raw_data.get("is_edge", False))
        except Exception as exc:
            SwarmLogger.log("ERROR", "SwarmCoordinator", f"Leader Verification error: {exc}", "LEADER")
        return final_lat, final_lon, is_edge

    @staticmethod
    def association_threshold(target, incoming_covariance, same_drone: bool = False,
                              sigma_from_cov_fn=None) -> float:
        sigma_from_cov_fn = sigma_from_cov_fn or (lambda cov: None)
        s_existing = sigma_from_cov_fn(target.covariance)
        s_incoming = sigma_from_cov_fn(incoming_covariance)
        sigmas = [s for s in (s_existing, s_incoming) if s is not None]
        avg_sigma = sum(sigmas) / len(sigmas) if sigmas else 0.0
        threshold = config.MERGE_DISTANCE_BASE_M + avg_sigma * config.MERGE_DISTANCE_SIGMA_SCALE
        if same_drone:
            threshold += 2.0
        return min(threshold, max(config.MERGE_DISTANCE_MAX_M, config.DUPLICATE_MERGE_DISTANCE_M))

    @staticmethod
    def canonical_group_id_token(group_id) -> Optional[str]:
        """Normalize equivalent group ids like `1` and `G1`."""
        if group_id is None:
            return None
        token = str(group_id).strip()
        if not token:
            return None
        match = re.fullmatch(r"[Gg](\d+)", token)
        if match:
            return match.group(1)
        return token.upper()

    @classmethod
    def canonical_group_id_value(cls, group_id) -> Optional[str]:
        token = cls.canonical_group_id_token(group_id)
        if token is None:
            return None
        if token.isdigit():
            return f"G{int(token)}"
        return token

    @classmethod
    def same_drone_group_id_match(cls, target, drone_port: int, incoming_group_id, current_time: float) -> bool:
        if incoming_group_id is None:
            return False
        if not target.has_recent_identity(drone_port, current_time, config.TRACK_IDENTITY_CONFLICT_WINDOW_SEC):
            return False
        existing_group_id = getattr(target, "group_id_by_port", {}).get(drone_port)
        existing_token = cls.canonical_group_id_token(existing_group_id)
        incoming_token = cls.canonical_group_id_token(incoming_group_id)
        return existing_token is not None and incoming_token is not None and existing_token == incoming_token

    @classmethod
    def conflicts_with_same_drone_identity(cls, target, drone_port: int,
                                           tracker_id, incoming_group_id,
                                           current_time: float) -> bool:
        if not target.has_recent_identity(drone_port, current_time, config.TRACK_IDENTITY_CONFLICT_WINDOW_SEC):
            return False
        if getattr(target, "is_group", False):
            return False
        existing_local_id = target.drone_local_ids.get(drone_port)
        existing_group_id = getattr(target, "group_id_by_port", {}).get(drone_port)
        if incoming_group_id is not None and existing_group_id is not None:
            incoming_token = cls.canonical_group_id_token(incoming_group_id)
            existing_token = cls.canonical_group_id_token(existing_group_id)
            if incoming_token is not None and incoming_token == existing_token:
                return False
            return True
        if tracker_id is None or existing_local_id is None:
            return False
        try:
            return int(tracker_id) != int(existing_local_id)
        except (TypeError, ValueError):
            return False

    def match_target(self, drone_port, lat, lon, tracker_id, covariance, raw_data,
                     should_fuse_targets_fn, sigma_from_cov_fn):
        current_time = time.time()
        best_id = None
        best_score = float("inf")
        reason = ""
        incoming_is_group = bool(isinstance(raw_data, dict) and raw_data.get("is_group"))
        incoming_group_id = raw_data.get("group_id") if isinstance(raw_data, dict) else None

        if tracker_id is not None and not incoming_is_group:
            idx_tid = self.owner.identity.lookup(drone_port, tracker_id)
            if idx_tid and idx_tid in self.owner.targets:
                cand = self.owner.targets[idx_tid]
                if cand.track_state != "DELETED" and cand.status != "LOST":
                    try:
                        dist = GeoMath.haversine_distance(lat, lon, cand.lat, cand.lon)
                    except Exception:
                        dist = 0.0
                    if dist <= config.SWARM_LOCAL_ID_INDEX_MAX_JUMP_M:
                        return idx_tid, "LOCAL_ID_INDEX"
        elif tracker_id is not None and incoming_is_group:
            idx_tid = self.owner.identity.lookup(drone_port, tracker_id)
            if idx_tid and idx_tid in self.owner.targets:
                cand = self.owner.targets[idx_tid]
                if cand.track_state != "DELETED" and cand.status != "LOST" and cand.is_group:
                    if not self.conflicts_with_same_drone_identity(
                        cand, drone_port, tracker_id, incoming_group_id, current_time
                    ):
                        try:
                            dist = GeoMath.haversine_distance(lat, lon, cand.lat, cand.lon)
                        except Exception:
                            dist = 0.0
                        threshold = self.association_threshold(cand, covariance, same_drone=True,
                                                               sigma_from_cov_fn=sigma_from_cov_fn)
                        if dist <= min(config.SWARM_LOCAL_ID_INDEX_MAX_JUMP_M, threshold):
                            return idx_tid, "LOCAL_GROUP_ID_INDEX"

        if config.ENABLE_SPATIAL_FALLBACK:
            for target_id, target in self.owner.targets.items():
                if target.track_state == "DELETED" or incoming_is_group != bool(target.is_group):
                    continue
                if incoming_is_group and target.is_group:
                    incoming_count = int(raw_data.get("group_member_count", 0) or 0) if isinstance(raw_data, dict) else 0
                    target_count = int(target.group_member_count or 0)
                    if not group_counts_compatible(incoming_count, target_count):
                        continue
                    if drone_port in target.drone_local_ids and self.conflicts_with_same_drone_identity(
                        target, drone_port, tracker_id, incoming_group_id, current_time
                    ):
                        continue
                    same_family = self.same_group_family_evidence(target, drone_port, raw_data)
                    same_group_id = self.same_drone_group_id_match(target, drone_port, incoming_group_id, current_time)
                    if same_family:
                        return target_id, "GROUP_MEMBER_MATCH"
                    if same_group_id:
                        return target_id, "GROUP_ID_MATCH"
                    if drone_port in target.drone_local_ids:
                        continuity_limit = min(
                            self.association_threshold(
                                target,
                                covariance,
                                same_drone=True,
                                sigma_from_cov_fn=sigma_from_cov_fn,
                            ),
                            12.0,
                        )
                        if same_drone_group_continuity_ok(
                            target,
                            drone_port,
                            lat,
                            lon,
                            incoming_count,
                            current_time,
                            continuity_limit,
                        ):
                            return target_id, "GROUP_SAME_DRONE_CONTINUITY"
                        continue

                threshold = self.association_threshold(
                    target, covariance, same_drone=(drone_port in target.drone_local_ids),
                    sigma_from_cov_fn=sigma_from_cov_fn,
                )
                ok, mahal, _ = should_fuse_targets_fn(
                    (lat, lon), (target.lat, target.lon),
                    cov1=covariance, cov2=target.covariance, threshold_m=threshold,
                )
                dist_m = GeoMath.haversine_distance(lat, lon, target.lat, target.lon)
                if not ok and dist_m < threshold:
                    ok = True
                    mahal = dist_m / 10.0
                if not ok:
                    continue

                score = mahal
                if tracker_id is not None and target.drone_local_ids.get(drone_port) == tracker_id:
                    score *= (1.0 - config.TRACKER_ID_ALPHA)
                if score < best_score:
                    best_score = score
                    best_id = target_id
                    reason = "SPATIAL_MAHALANOBIS"
        elif tracker_id is not None:
            for target_id, target in self.owner.targets.items():
                if target.assigned_drone_port == drone_port and target.tracker_id == tracker_id:
                    return target_id, "ID_PERSIST"

        return best_id, reason

    def update_existing_target(self, target_id, drone_port, lat, lon, tracker_id,
                               covariance, raw_data, is_edge, current_time):
        target = self.owner.targets[target_id]
        is_owner = target.owner_port == drone_port
        has_owner = target.owner_port is not None
        quality = self.owner.ownership.calculate_quality(raw_data or {})

        if has_owner and not is_owner:
            self.owner.ownership.process_secondary_observation(target, drone_port, raw_data or {})
            target.update_position(lat, lon, covariance, is_edge=is_edge)
            immutable_identity = (
                target.assigned_drone_port is not None
                and target.assigned_drone_port != drone_port
                and (target.is_in_attack_pipeline() or target.status in IMMUTABLE_STATES)
            )
            if not immutable_identity:
                self.update_visual_context(target, drone_port, raw_data, current_time)
                self.update_group_metadata(target, target_id, drone_port, raw_data)
                if tracker_id is not None:
                    self.update_local_id(target, target_id, drone_port, tracker_id, current_time)
            return target_id

        target.update_position(lat, lon, covariance, is_edge=is_edge)
        target.record_observation(drone_port, quality, is_owner=True)
        self.update_visual_context(target, drone_port, raw_data, current_time)

        if target.ownership_state == OWNERSHIP_RESERVED:
            target.ownership_state = OWNERSHIP_OWNED
            target.owner_lost_time = None
            if target.assigned_drone_port != drone_port:
                old_assignee = target.assigned_drone_port
                target.assigned_drone_port = drone_port
                SwarmLogger.log(
                    "OWNERSHIP", "LEADER",
                    f"{target_id}: RE-ASSIGNED {old_assignee} → {drone_port} (reclaimed)",
                    "OWNERSHIP",
                )

        self.update_group_metadata(target, target_id, drone_port, raw_data)
        self.owner.lifecycle.check_confirmation(
            target, current_time, is_searching=not self.owner.attack_mode_active
        )
        if tracker_id is not None:
            self.update_local_id(target, target_id, drone_port, tracker_id, current_time)
        return target_id

    def create_new_target(self, drone_port, lat, lon, tracker_id,
                          covariance, raw_data, is_edge, current_time, sigma_from_cov_fn):
        incoming_is_group = bool(isinstance(raw_data, dict) and raw_data.get("is_group"))
        incoming_group_id = raw_data.get("group_id") if isinstance(raw_data, dict) else None

        for existing_id, existing_target in self.owner.targets.items():
            if existing_target.track_state == "DELETED" or existing_target.lat is None:
                continue
            if incoming_is_group != bool(existing_target.is_group):
                continue
            if incoming_is_group and existing_target.is_group:
                incoming_count = int(raw_data.get("group_member_count", 0) or 0) if isinstance(raw_data, dict) else 0
                existing_count = int(existing_target.group_member_count or 0)
                if not group_counts_compatible(incoming_count, existing_count):
                    continue
                if drone_port in existing_target.drone_local_ids and self.conflicts_with_same_drone_identity(
                    existing_target, drone_port, tracker_id, incoming_group_id, current_time
                ):
                    continue
                same_family_evidence = self.same_group_family_evidence(
                    existing_target, drone_port, raw_data
                )
                same_group_id = self.same_drone_group_id_match(
                    existing_target, drone_port, incoming_group_id, current_time
                )
                if same_family_evidence:
                    existing_target.update_position(lat, lon, covariance, is_edge=is_edge)
                    self.update_visual_context(existing_target, drone_port, raw_data, current_time)
                    self.update_group_metadata(existing_target, existing_id, drone_port, raw_data)
                    if tracker_id is not None:
                        self.update_local_id(existing_target, existing_id, drone_port, tracker_id, current_time)
                    quality = self.owner.ownership.calculate_quality(raw_data or {})
                    existing_target.record_observation(drone_port, quality, is_owner=(existing_target.owner_port == drone_port))
                    return existing_id
                if same_group_id:
                    existing_target.update_position(lat, lon, covariance, is_edge=is_edge)
                    self.update_visual_context(existing_target, drone_port, raw_data, current_time)
                    self.update_group_metadata(existing_target, existing_id, drone_port, raw_data)
                    if tracker_id is not None:
                        self.update_local_id(existing_target, existing_id, drone_port, tracker_id, current_time)
                    quality = self.owner.ownership.calculate_quality(raw_data or {})
                    existing_target.record_observation(drone_port, quality, is_owner=(existing_target.owner_port == drone_port))
                    return existing_id
                if drone_port in existing_target.drone_local_ids:
                    continuity_limit = min(
                        self.association_threshold(
                            existing_target,
                            covariance,
                            same_drone=True,
                            sigma_from_cov_fn=sigma_from_cov_fn,
                        ),
                        12.0,
                    )
                    if same_drone_group_continuity_ok(
                        existing_target,
                        drone_port,
                        lat,
                        lon,
                        incoming_count,
                        current_time,
                        continuity_limit,
                    ):
                        existing_target.update_position(lat, lon, covariance, is_edge=is_edge)
                        self.update_visual_context(existing_target, drone_port, raw_data, current_time)
                        self.update_group_metadata(existing_target, existing_id, drone_port, raw_data)
                        if tracker_id is not None:
                            self.update_local_id(existing_target, existing_id, drone_port, tracker_id, current_time)
                        quality = self.owner.ownership.calculate_quality(raw_data or {})
                        existing_target.record_observation(drone_port, quality, is_owner=(existing_target.owner_port == drone_port))
                        return existing_id
                    continue
                duplicate_threshold = self.association_threshold(
                    existing_target, covariance, same_drone=(drone_port in existing_target.drone_local_ids),
                    sigma_from_cov_fn=sigma_from_cov_fn,
                )
            else:
                duplicate_threshold = self.association_threshold(
                    existing_target, covariance, sigma_from_cov_fn=sigma_from_cov_fn
                )
            dist = GeoMath.haversine_distance(lat, lon, existing_target.lat, existing_target.lon)
            if dist < duplicate_threshold:
                existing_target.update_position(lat, lon, covariance, is_edge=is_edge)
                self.update_visual_context(existing_target, drone_port, raw_data, current_time)
                self.update_group_metadata(existing_target, existing_id, drone_port, raw_data)
                if tracker_id is not None:
                    self.update_local_id(existing_target, existing_id, drone_port, tracker_id, current_time)
                quality = self.owner.ownership.calculate_quality(raw_data or {})
                existing_target.record_observation(drone_port, quality, is_owner=False)
                return existing_id

        target_id = self.owner.registry.generate_id()
        new_target = TrackedTarget(
            target_id, lat, lon, current_time,
            tracker_id=tracker_id, covariance=covariance,
            source_drone_port=drone_port,
        )
        has_existing = any(
            target.assigned_drone_port == drone_port and target.track_state != "DELETED"
            for target in self.owner.targets.values()
        )
        if config.ENABLE_CAPACITY_GUARD and has_existing:
            new_target.ownership_state = OWNERSHIP_FREE
            new_target.owner_port = drone_port
            new_target.assigned_drone_port = None
        else:
            new_target.ownership_state = OWNERSHIP_OWNED
            new_target.owner_port = drone_port
            new_target.assigned_drone_port = drone_port

        quality = self.owner.ownership.calculate_quality(raw_data or {})
        new_target.record_observation(drone_port, quality, is_owner=True)
        if is_edge:
            new_target.observation_count = 0
            new_target.consecutive_obs = 0
            new_target._edge_accumulator = config.SWARM_EDGE_FILTER_FRACTIONAL_CREDIT

        self.owner.registry.add(new_target)
        self.owner.ownership.target_owner[target_id] = drone_port
        if new_target.assigned_drone_port == drone_port:
            self.owner.ownership.drone_active_target[drone_port] = target_id
        if tracker_id is not None:
            self.owner.identity.update(drone_port, tracker_id, target_id, now=current_time)

        self.update_visual_context(new_target, drone_port, raw_data, current_time)
        self.update_group_metadata(new_target, target_id, drone_port, raw_data)
        SwarmLogger.log(
            "TARGET", "LEADER",
            f"NEW TARGET (TENTATIVE): {target_id} (TID:{tracker_id}) @ ({lat:.6f}, {lon:.6f}) | Owner: Drone_{drone_port}",
            "TARGET",
        )
        return target_id

    @staticmethod
    def update_visual_context(target, drone_port, raw_data, current_time):
        if not isinstance(raw_data, dict):
            return
        bbox = raw_data.get("bbox")
        if not bbox or len(bbox) < 4:
            return
        try:
            target.last_bbox_by_port[drone_port] = tuple(float(v) for v in bbox[:4])
            target.bbox_last_update_time[drone_port] = current_time
        except (TypeError, ValueError):
            return

    def update_local_id(self, target, target_id, drone_port, tracker_id, current_time):
        old = target.drone_local_ids.get(drone_port)
        if (
            old is not None and old != tracker_id
            and target.is_in_attack_pipeline()
            and target.assigned_drone_port == drone_port
        ):
            hyst = target.local_id_hysteresis.setdefault(drone_port, {"id": None, "count": 0})
            if hyst["id"] == tracker_id:
                hyst["count"] += 1
            else:
                hyst["id"] = tracker_id
                hyst["count"] = 1
            if hyst["count"] >= config.SWARM_LOCAL_ID_ATTACK_SWITCH_FRAMES:
                target.drone_local_ids[drone_port] = tracker_id
                target.identity_last_update_time[drone_port] = current_time
                target.identity_change_time[drone_port] = current_time
                target.tracker_id = tracker_id
                self.owner.identity.log_id_update(drone_port, target_id, old, tracker_id)
                hyst["count"] = 0
            else:
                target.identity_last_update_time[drone_port] = current_time
            return

        target.drone_local_ids[drone_port] = tracker_id
        target.identity_last_update_time[drone_port] = current_time
        if old != tracker_id:
            target.identity_change_time[drone_port] = current_time
        if target.assigned_drone_port == drone_port:
            target.tracker_id = tracker_id
        self.owner.identity.update(drone_port, tracker_id, target_id, now=current_time)
        if old is not None and old != tracker_id:
            self.owner.identity.log_id_update(drone_port, target_id, old, tracker_id)

    def record_fusion(self, target_id, lat, lon, covariance, drone_port, confidence: float = 1.0):
        try:
            obs_cov = np.array(covariance, dtype=np.float64) if covariance is not None and hasattr(covariance, "shape") else np.eye(3) * 4.0
            target = self.owner.targets.get(target_id)
            if target:
                self.owner.fusion.ekf_manager.add_observation(
                    target_id, (lat, lon, 0.0), obs_cov, drone_port,
                    confidence=confidence,
                    is_group=target.is_group,
                    group_member_count=target.group_member_count,
                    attack_pipeline=bool(target.is_in_attack_pipeline() or target.approved_for_attack),
                )
        except Exception as exc:
            SwarmLogger.log("ERROR", "SwarmCoordinator", f"add_observation error: {exc}", "LEADER")

    def tentative_redirect(self, target_id, drone_port, lat, lon, tracker_id, covariance, raw_data=None):
        source_target = self.owner.targets.get(target_id)
        if source_target is None:
            return {"action": "SEARCH", "target_id": target_id, "tracker_id": tracker_id, "message": "Awaiting confirmation"}

        current_time = time.time()
        incoming_group_id = raw_data.get("group_id") if isinstance(raw_data, dict) else None
        incoming_count = int(getattr(source_target, "group_member_count", 0) or 0)
        best_candidate = None
        best_dist = float("inf")

        for check_id, check_target in self.owner.targets.items():
            if check_id == target_id or check_target.track_state != "CONFIRMED":
                continue
            if source_target.is_group != check_target.is_group:
                continue

            seen_ts = check_target.drone_last_seen_time.get(drone_port)
            if seen_ts is None or (current_time - float(seen_ts)) > 2.5:
                continue

            dist = GeoMath.haversine_distance(
                source_target.lat,
                source_target.lon,
                check_target.lat,
                check_target.lon,
            )
            if dist >= 20.0 or dist >= best_dist:
                continue

            if source_target.is_group and check_target.is_group:
                if not group_counts_compatible(incoming_count, check_target.group_member_count):
                    continue
                same_family = self.same_group_family_evidence(check_target, drone_port, raw_data)
                same_group_id = self.same_drone_group_id_match(
                    check_target,
                    drone_port,
                    incoming_group_id,
                    current_time,
                )
                continuity_ok = same_drone_group_continuity_ok(
                    check_target,
                    drone_port,
                    lat,
                    lon,
                    incoming_count,
                    current_time,
                    max_dist_m=8.0,
                )
                if not same_family and not same_group_id and not continuity_ok:
                    SwarmLogger.log_throttled(
                        "REJECT",
                        "LEADER",
                        f"TENTATIVE redirect rejected: {target_id} !-> {check_id} "
                        f"(distinct same-drone group family, dist={dist:.1f}m)",
                        "LIFECYCLE",
                        interval_s=2.0,
                    )
                    continue

            best_candidate = (check_id, check_target, dist)
            best_dist = dist

        if best_candidate is not None:
            check_id, check_target, dist = best_candidate
            check_target.update_position(lat, lon, covariance)
            if tracker_id is not None:
                self.update_local_id(check_target, check_id, drone_port, tracker_id, current_time)
            if raw_data:
                self.update_group_metadata(check_target, check_id, drone_port, raw_data)
            self.owner.registry.remove(target_id)
            SwarmLogger.log(
                "REDIRECT", "LEADER",
                f"TENTATIVE {target_id} → CONFIRMED {check_id} (dist={dist:.1f}m)",
                "LIFECYCLE",
            )
            return self.build_report_response(check_target, check_id, drone_port)

        return {"action": "SEARCH", "target_id": target_id, "tracker_id": tracker_id, "message": "Awaiting confirmation"}

    def build_report_response(self, target, target_id, drone_port):
        filtered_pos = self.owner.fusion.ekf_manager.get_target_position(target_id)
        if filtered_pos is not None:
            gps_pos, covariance_filtered = filtered_pos
            lat_filtered = float(gps_pos[0])
            lon_filtered = float(gps_pos[1])
        else:
            lat_filtered = target.lat
            lon_filtered = target.lon
            covariance_filtered = target.covariance

        drone_local_id = self.owner.link.resolve_local_target_id(target, drone_port)
        response = {
            "target_id": target_id,
            "tracker_id": drone_local_id,
            "local_id": drone_local_id,
            "lat": lat_filtered,
            "lon": lon_filtered,
            "covariance": covariance_filtered.tolist() if covariance_filtered is not None else None,
            "is_group": bool(target.is_group),
            "group_id": target.group_id_by_port.get(drone_port, target.group_id_local),
            "group_member_count": int(target.group_member_count or 0),
        }

        if target.assigned_drone_port == drone_port:
            if target.status == "ASSIGNED" and drone_local_id is not None:
                response["action"] = "ASSIGN"
            elif target.status == "READY":
                response["action"] = "VERIFY"
            elif target.status in ("VERIFYING", "STAGING", "CENTERING", "EXECUTING"):
                response["action"] = "EXECUTE"
            elif target.approved_for_attack and drone_local_id is not None:
                target.status = "ASSIGNED"
                response["action"] = "ASSIGN"
            elif target.approved_for_attack and drone_local_id is None:
                response["action"] = "HOVER"
                response["message"] = "Awaiting local representative"
            else:
                target.status = "PENDING"
                response["action"] = "HOVER"
        elif target.assigned_drone_port is None:
            response["action"] = "TRACK"
            response["message"] = "Unassigned"
        else:
            if target.ownership_state == OWNERSHIP_RESERVED:
                response["action"] = "HOVER"
                response["message"] = "Reserved target - reclaiming"
            elif drone_port in target.drone_local_ids:
                response["action"] = "HOVER"
                response["message"] = f"Merged target - tracking {target.assigned_drone_port}'s target"
            else:
                response["action"] = "TRACK"
                response["owner"] = target.assigned_drone_port

        sigma = 0.0
        if covariance_filtered is not None and hasattr(covariance_filtered, "shape"):
            sigma = (covariance_filtered[0, 0] + covariance_filtered[1, 1]) ** 0.5
        SwarmLogger.log(
            "RESPONSE", "Coordinator",
            f"→DRONE_{drone_port}: {response['action']} | "
            f"target={target_id} | local_id={drone_local_id} | "
            f"lat={lat_filtered:.6f} lon={lon_filtered:.6f} | σ={sigma:.1f}m",
            "FILTER",
        )
        return response

    def update_group_metadata(self, target, target_id, drone_port, raw_data):
        if not isinstance(raw_data, dict) or not raw_data.get("is_group"):
            return

        old_count = target.group_member_count
        was_group = target.is_group
        target.is_group = True
        incoming_count = int(raw_data.get("group_member_count", old_count or 0) or 0)
        incoming_group_id = self.canonical_group_id_value(raw_data.get("group_id", target.group_id_local))
        incoming_members = tuple(sorted(extract_group_member_ids(raw_data)))
        if incoming_group_id is not None:
            prev_group_id = target.group_id_by_port.get(drone_port)
            target.group_id_by_port[drone_port] = incoming_group_id
            target.identity_last_update_time[drone_port] = time.time()
            if prev_group_id != incoming_group_id:
                target.identity_change_time[drone_port] = time.time()
            preferred_port = target.assigned_drone_port or target.owner_port or drone_port
            target.group_id_local = target.group_id_by_port.get(preferred_port, incoming_group_id)

        target.group_count_by_port[drone_port] = incoming_count
        if incoming_members:
            target.group_member_local_ids[drone_port] = incoming_members
        target.refresh_group_summary()
        if not was_group:
            SwarmLogger.log(
                "GROUP", "LEADER",
                f"{target_id}: Became GROUP x{target.group_member_count} (GID:{target.group_id_local})",
                "CLUSTER",
            )
        elif old_count != target.group_member_count:
            SwarmLogger.log(
                "GROUP", "LEADER",
                f"{target_id}: Group size changed x{old_count}→x{target.group_member_count}",
                "CLUSTER",
            )

    def same_group_family_evidence(self, target, drone_port: int, raw_data) -> bool:
        if not getattr(target, "is_group", False):
            return False
        incoming_members = extract_group_member_ids(raw_data)
        existing_members = set(
            int(mid) for mid in (getattr(target, "group_member_local_ids", {}).get(drone_port, ()) or ())
        )
        if incoming_members and existing_members and (incoming_members & existing_members):
            return True

        incoming_group_id = raw_data.get("group_id") if isinstance(raw_data, dict) else None
        return self.same_drone_group_id_match(target, drone_port, incoming_group_id, time.time())
