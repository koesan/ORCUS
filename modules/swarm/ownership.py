"""Target ownership model and handoff management."""

import math
import time
from typing import Dict, Optional

from modules.core.logger import SwarmLogger
from modules.swarm.target import (
    TrackedTarget, TargetRegistry,
    OWNERSHIP_FREE, OWNERSHIP_OWNED, OWNERSHIP_RESERVED, OWNERSHIP_HANDOFF,
)
from config import (
    HANDOFF_CONSECUTIVE_FRAMES, HANDOFF_QUALITY_RATIO, HANDOFF_COOLDOWN_S,
    CAMERA_WIDTH, CAMERA_HEIGHT,
)


class OwnershipManager:
    """Manages target ownership and handoff operations.

    Ownership transitions:
        FREE → OWNED      (by assignment)
        OWNED → RESERVED   (owner loses observation)
        RESERVED → OWNED   (owner regains)
        RESERVED → FREE    (grace period expires)
        * → HANDOFF_PENDING → OWNED  (transfer)
    """

    def __init__(self, registry: TargetRegistry):
        self._registry = registry
        # port → canonical_tid
        self.drone_active_target: Dict[int, Optional[str]] = {}
        # canonical_tid → port
        self.target_owner: Dict[str, Optional[int]] = {}

    # ------------------------------------------------------------------
    # Assignment / Release (Atomic)
    # ------------------------------------------------------------------

    def assign(self, port: int, canonical_id: str):
        """Assign drone to target. Atomically clear old assignments."""
        with self._registry.lock:
            # Release this drone's old target
            for t_id, t in self._registry.items():
                if t.assigned_drone_port == port and t_id != canonical_id:
                    t.assigned_drone_port = None
                    t.ownership_state = OWNERSHIP_FREE
                    t.owner_port = None
                    self.target_owner.pop(t_id, None)
                    SwarmLogger.log(
                        "ASSIGN", "LEADER",
                        f"Released {t_id} from Drone_{port} (one-drone-one-target)", "ASSIGN",
                    )

            self.drone_active_target[port] = canonical_id
            self.target_owner[canonical_id] = port
            t = self._registry.get(canonical_id)
            if t:
                t.assigned_drone_port = port
                t.assignment_time = time.time()
                if t.status not in {"LOCKED", "ECHO_WAIT", "CONFIRMED_ATTACK",
                                     "CENTERING", "ATTACKING", "DIVING"}:
                    t.status = "PENDING"
                if t.ownership_state == OWNERSHIP_FREE:
                    t.ownership_state = OWNERSHIP_OWNED
                    t.owner_port = port
                    quality = t.observation_quality.get(port, 0.0)
                    SwarmLogger.log(
                        "OWNERSHIP", "LEADER",
                        f"{canonical_id}: FREE → OWNED (assigned to Drone_{port}) | "
                        f"quality={quality:.2f} obs={t.observation_count}",
                        "OWNERSHIP",
                    )

    def release(self, port: int):
        """Atomically remove drone's target binding."""
        with self._registry.lock:
            # Release all targets belonging to this drone
            for t_id, t in self._registry.items():
                if t.assigned_drone_port == port:
                    t.assigned_drone_port = None
                    t.status = "PENDING"
                    t.approved_for_attack = False
                    t.ownership_state = OWNERSHIP_FREE
                    t.owner_port = None
                    self.target_owner.pop(t_id, None)
            self.drone_active_target.pop(port, None)

    def transfer(self, old_port: int, new_port: int, canonical_id: str):
        """Transfer ownership from old_port to new_port."""
        with self._registry.lock:
            self.release(old_port)
            self.assign(new_port, canonical_id)

    # ------------------------------------------------------------------
    # Query
    # ------------------------------------------------------------------

    def is_drone_busy(self, port: int) -> bool:
        """Does drone have an active target?"""
        with self._registry.lock:
            tid = self.drone_active_target.get(port)
            if not tid:
                return False
            t = self._registry.get(tid)
            if t and t.track_state != "DELETED":
                if t.status in {"PENDING", "ACTIVE"} or t.is_in_attack_pipeline():
                    return True
            return False

    def get_active_target(self, port: int) -> Optional[str]:
        """Drone's active target ID."""
        return self.drone_active_target.get(port)

    # ------------------------------------------------------------------
    # Observation quality calculation
    # ------------------------------------------------------------------

    @staticmethod
    def calculate_quality(observation_data: Dict) -> float:
        """Calculate observation quality score. Higher = better."""
        score = 0.0

        # Covariance factor
        cov = observation_data.get("covariance")
        if cov is not None:
            try:
                if hasattr(cov, "shape") and cov.shape[0] >= 2:
                    trace = float(cov[0, 0] + cov[1, 1])
                    if trace > 0:
                        score += min(10.0, 100.0 / trace)
            except (IndexError, TypeError):
                pass
        else:
            score += 5.0

        # Confidence score
        conf = observation_data.get("confidence", 0.5)
        score += conf * 10.0

        # Target proximity to center
        bbox = observation_data.get("bbox")
        img_dims = observation_data.get("img_dims", (CAMERA_WIDTH, CAMERA_HEIGHT))
        if bbox and len(bbox) >= 2:
            cx, cy = bbox[0], bbox[1]
            img_cx = img_dims[0] / 2
            img_cy = img_dims[1] / 2
            dist_norm = math.sqrt(((cx - img_cx) / max(img_cx, 1)) ** 2 +
                                  ((cy - img_cy) / max(img_cy, 1)) ** 2)
            score += max(0.0, 5.0 - dist_norm * 5.0)

        # Altitude factor
        drone_gps = observation_data.get("drone_gps")
        if drone_gps and len(drone_gps) > 2:
            alt = drone_gps[2]
            if alt > 0:
                score += min(5.0, alt / 10.0)

        return max(0.0, score)

    # ------------------------------------------------------------------
    # Handoff
    # ------------------------------------------------------------------

    def process_secondary_observation(self, target: TrackedTarget,
                                      drone_port: int, observation_data: Dict):
        """Process observation from non-owner drone, check handoff."""
        quality = self.calculate_quality(observation_data)
        target.record_observation(drone_port, quality, is_owner=False)

        # Update handoff accumulator
        if quality > target.get_owner_quality() * HANDOFF_QUALITY_RATIO:
            target.handoff_score_accumulator[drone_port] = (
                target.handoff_score_accumulator.get(drone_port, 0) + 1
            )
        else:
            target.handoff_score_accumulator[drone_port] = 0

        if self._check_handoff(target, drone_port, quality):
            self._initiate_handoff(target, drone_port)

    def _check_handoff(self, target: TrackedTarget, candidate_port: int,
                       candidate_quality: float) -> bool:
        """Are handoff conditions satisfied?"""
        if target.is_immutable():
            return False
        consec = target.handoff_score_accumulator.get(candidate_port, 0)
        if consec < HANDOFF_CONSECUTIVE_FRAMES:
            return False
        if candidate_quality <= target.get_owner_quality() * HANDOFF_QUALITY_RATIO:
            return False
        if target.last_handoff_time is not None:
            if time.time() - target.last_handoff_time < HANDOFF_COOLDOWN_S:
                return False
        return True

    def _initiate_handoff(self, target: TrackedTarget, new_port: int):
        """Transfer ownership to new drone."""
        # Does candidate already own another target?
        if self.drone_active_target.get(new_port) not in (None, target.id):
            # Throttle: log at most once per 5s per drone
            now = time.time()
            throttle_key = f"handoff_block_{new_port}"
            last = getattr(self, '_handoff_block_log_time', {})
            if now - last.get(throttle_key, 0) > 5.0:
                if not hasattr(self, '_handoff_block_log_time'):
                    self._handoff_block_log_time = {}
                self._handoff_block_log_time[throttle_key] = now
                SwarmLogger.log(
                    "HANDOFF_BLOCKED", "LEADER",
                    f"Handoff blocked: Drone_{new_port} already has target "
                    f"{self.drone_active_target[new_port]}",
                    "OWNERSHIP",
                )
            return

        # Cooldown check
        if target.last_handoff_time is not None:
            elapsed = time.time() - target.last_handoff_time
            if elapsed < HANDOFF_COOLDOWN_S:
                return

        old_owner = target.owner_port
        SwarmLogger.log(
            "HANDOFF", "LEADER",
            f"Ownership handoff: {target.id} from {old_owner} to {new_port}",
            "OWNERSHIP",
        )

        # Atomic transfer
        target.ownership_state = OWNERSHIP_OWNED
        target.owner_port = new_port
        target.assigned_drone_port = new_port
        target.last_handoff_time = time.time()
        target.handoff_candidate_port = None
        target.handoff_score_accumulator.clear()
        target.secondary_observations.clear()

        self.target_owner[target.id] = new_port
        if old_owner is not None:
            self.drone_active_target[old_owner] = None
        self.drone_active_target[new_port] = target.id

    # ------------------------------------------------------------------
    # Owner loss handling
    # ------------------------------------------------------------------

    def process_owner_loss(self, target: TrackedTarget, grace_period: float = 5.0):
        """When owner loses observation: OWNED → RESERVED → FREE."""
        if target.is_in_attack_pipeline() or (target.status == "ENGAGED" and target.approved_for_attack):
            SwarmLogger.log(
                "GUARD", "LEADER",
                f"OWNER LOSS BYPASS: {target.id} stays {target.status} during terminal attack",
                "OWNERSHIP",
            )
            return

        if target.ownership_state == OWNERSHIP_OWNED:
            target.ownership_state = OWNERSHIP_RESERVED
            target.owner_lost_time = time.time()
            SwarmLogger.log(
                "OWNERSHIP", "LEADER",
                f"{target.id}: OWNED → RESERVED (owner {target.owner_port} lost sight) | "
                f"obs={target.observation_count} consec={target.consecutive_obs}",
                "OWNERSHIP",
            )

        elif target.ownership_state == OWNERSHIP_RESERVED:
            if target.owner_lost_time is not None:
                if time.time() - target.owner_lost_time > grace_period:
                    best_port, quality = target.get_best_secondary()
                    if best_port is not None and quality > 0:
                        self._initiate_handoff(target, best_port)
                    else:
                        lost_duration = time.time() - target.owner_lost_time if target.owner_lost_time else 0
                        target.reset_ownership()
                        target.assigned_drone_port = None
                        SwarmLogger.log(
                            "OWNERSHIP", "LEADER",
                            f"{target.id}: RESERVED → FREE (no secondary observer, "
                            f"lost_for={lost_duration:.1f}s)",
                            "OWNERSHIP",
                        )

    # ------------------------------------------------------------------
    # Drone disconnect
    # ------------------------------------------------------------------

    def handle_disconnect(self, port: int):
        """Release drone's targets when connection is lost."""
        self.release(port)
