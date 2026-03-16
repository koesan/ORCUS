"""Target lifecycle state machine and pruning engine."""

import time
import numpy as np
from typing import List, Callable, Optional

from modules.core.logger import SwarmLogger
from modules.swarm.target import (
    TrackedTarget, TargetRegistry, IMMUTABLE_STATES, OWNERSHIP_FREE
)
from config import (
    TRACK_CONFIRMATION_TIME_SEC, TRACK_MIN_OBSERVATIONS,
    TRACK_MIN_OBSERVATIONS_SEARCH, TRACK_TENTATIVE_TIMEOUT_SEC,
    TRACK_LOST_TIMEOUT_SEC, TRACK_DELETE_TIMEOUT_SEC,
    TRACK_OBSERVER_STALE_TIMEOUT_SEC,
    TARGET_COAST_TIME_S, TARGET_PRUNE_TIME_S,
    ATTACK_STALE_ENGAGED_TIMEOUT_S,
    ENABLE_ATTACK_IMMUTABILITY,
    # Grace Period & Coast Mode (Phase 1)
    TARGET_GRACE_PERIOD_S, TARGET_STALE_COAST_S,
    TARGET_MAX_COAST_VELOCITY, TARGET_MAX_COAST_DISPLACEMENT_M,
    TARGET_STALE_VELOCITY_THRESHOLD,
)


class TargetLifecycle:
    """Centrally manages target lifecycle transitions.

    Transition table:
        TENTATIVE  →  CONFIRMED  (sufficient observation + time)
        TENTATIVE  →  DELETED    (timeout, no observation)
        CONFIRMED  →  LOST       (observation loss)
        LOST       →  CONFIRMED  (re-observation — in update_position)
        LOST       →  DELETED    (long time without observation)
    """

    def __init__(self, registry: TargetRegistry):
        self._registry = registry

    # ------------------------------------------------------------------
    # TENTATIVE → CONFIRMED promotion
    # ------------------------------------------------------------------

    def check_confirmation(self, target: TrackedTarget, current_time: float,
                           is_searching: bool = True) -> bool:
        """Promote TENTATIVE target to CONFIRMED."""
        if target.track_state != "TENTATIVE":
            return False

        time_alive = current_time - target.first_seen_time
        required = TRACK_MIN_OBSERVATIONS_SEARCH if is_searching else TRACK_MIN_OBSERVATIONS

        if time_alive >= TRACK_CONFIRMATION_TIME_SEC and target.observation_count >= required:
            target.track_state = "CONFIRMED"
            SwarmLogger.log(
                "LIFECYCLE", "LEADER",
                f"{target.id}: TENTATIVE → CONFIRMED (t={time_alive:.2f}s, obs={target.observation_count})",
                "TRACK",
            )
            return True
        return False

    # ------------------------------------------------------------------
    # Full pruning cycle
    # ------------------------------------------------------------------

    def prune(
        self,
        current_time: float,
        attack_mode_active: bool,
        drone_connected_check: Callable[[int], bool],
        release_drone: Callable[[int], None],
        remove_fusion_target: Callable[[str], None],
    ) -> List[str]:
        """Check TENTATIVE/CONFIRMED/LOST states, return deleted IDs."""
        to_delete: List[str] = []

        with self._registry.lock:
            for t_id, target in list(self._registry.items()):
                target.prune_stale_observers(current_time, TRACK_OBSERVER_STALE_TIMEOUT_SEC)
                time_since = current_time - target.last_observation_time

                # ----- TENTATIVE -----
                if target.track_state == "TENTATIVE":
                    self._prune_tentative(target, t_id, time_since, current_time,
                                          attack_mode_active, to_delete)
                    continue

                # ----- CONFIRMED -----
                if target.track_state == "CONFIRMED":
                    self._prune_confirmed(target, t_id, time_since,
                                          drone_connected_check, release_drone, to_delete)
                    continue

                # ----- LOST -----
                if target.track_state == "LOST":
                    if time_since > TRACK_DELETE_TIMEOUT_SEC:
                        SwarmLogger.log(
                            "LIFECYCLE", "LEADER",
                            f"{t_id}: LOST → DELETED (NoObs={time_since:.1f}s)", "TRACK",
                        )
                        if target.assigned_drone_port:
                            release_drone(target.assigned_drone_port)
                        to_delete.append(t_id)

            # Remove from registry
            for t_id in to_delete:
                remove_fusion_target(t_id)
                self._registry.remove(t_id)

        return to_delete

    # ------------------------------------------------------------------
    # Sub-pruning routines
    # ------------------------------------------------------------------

    def _prune_tentative(self, target: TrackedTarget, t_id: str,
                         time_since: float, current_time: float,
                         attack_mode_active: bool, to_delete: List[str]):
        """TENTATIVE target pruning logic."""
        # Promotion check
        just_confirmed = self.check_confirmation(
            target, current_time, is_searching=not attack_mode_active
        )
        if just_confirmed and attack_mode_active and target.assigned_drone_port is not None:
            target.approved_for_attack = True
            target.status = "ENGAGED"
            SwarmLogger.log("ASSIGN", "LEADER", f"{t_id} Auto-Approved (Attack Mode Active)", "AUTO")

        # Timeout check
        if target.track_state == "TENTATIVE" and time_since > TRACK_TENTATIVE_TIMEOUT_SEC:
            # Is at least one drone still seeing it recently?
            if target.has_recent_observer(current_time, TRACK_OBSERVER_STALE_TIMEOUT_SEC):
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
        """CONFIRMED target pruning logic."""
        is_protected = (
            target.status == "ENGAGED"
            or (ENABLE_ATTACK_IMMUTABILITY and target.status in IMMUTABLE_STATES)
        )
        terminal_attack_bypass = (
            target.assigned_drone_port is not None
            and (
                target.status in IMMUTABLE_STATES
                or (target.status == "ENGAGED" and target.approved_for_attack)
            )
        )

        if is_protected and target.assigned_drone_port is not None:
            # Is drone connected?
            if not drone_connected_check(target.assigned_drone_port):
                SwarmLogger.log(
                    "PRUNE", "LEADER",
                    f"GHOST RELEASE: {t_id} | Drone_{target.assigned_drone_port} disconnected",
                    "PRUNE",
                )
                release_drone(target.assigned_drone_port)
                return

            # Long time without observation
            if time_since > TARGET_PRUNE_TIME_S:
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
            elif time_since > ATTACK_STALE_ENGAGED_TIMEOUT_S:
                if terminal_attack_bypass:
                    SwarmLogger.log(
                        "GUARD", "LEADER",
                        f"TERMINAL ATTACK BYPASS: {t_id} | NoObs={time_since:.1f}s | "
                        f"Status={target.status} | Drone={target.assigned_drone_port}",
                        "GUARD",
                    )
                    return
                # Target stuck in attack pipeline
                SwarmLogger.log(
                    "PRUNE", "LEADER",
                    f"STALE {target.status}: {t_id} | NoObs={time_since:.1f}s → LOST",
                    "PRUNE",
                )
                target.track_state = "LOST"
                target.status = "LOST"
                release_drone(target.assigned_drone_port)
            elif time_since > TARGET_COAST_TIME_S:
                SwarmLogger.log(
                    "GUARD", "LEADER",
                    f"PRUNE GUARD: {t_id} is {target.status} — kept (NoObs={time_since:.1f}s)",
                    "GUARD",
                )
            return

        # --- GRACE PERIOD & COAST MODE (Phase 1) ---
        # Delay transition to LOST for short-term detection loss
        
        # If observation just arrived, exit coast mode and reset distance
        # (Oscillation bug fix: re-acquired targets should not keep old coast distance)
        if time_since < 0.1 and target.is_coasting:
            target.is_coasting = False
            target.coast_distance_traveled = 0.0
            target.coast_start_time = None
        
        observation_gap_started = time_since >= 0.25
        in_grace_period = observation_gap_started and time_since < TARGET_GRACE_PERIOD_S
        
        if in_grace_period:
            # Coast mode: continue with EKF prediction, don't go LOST
            if not target.is_coasting:
                target.is_coasting = True
                target.coast_start_time = time.time()
                SwarmLogger.log(
                    "GRACE", "LEADER",
                    f"{t_id}: GRACE PERIOD STARTED - Coasting (NoObs={time_since:.1f}s)",
                    "GRACE",
                )
            
            # Limit velocity growth
            if target.last_velocity is not None:
                vel_mag = np.linalg.norm(target.last_velocity)
                if vel_mag > TARGET_MAX_COAST_VELOCITY:
                    target.last_velocity = target.last_velocity * (TARGET_MAX_COAST_VELOCITY / vel_mag)
            
            # Coast displacement limit check
            if target.coast_distance_traveled > TARGET_MAX_COAST_DISPLACEMENT_M:
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
        
        
        # Grace period ended
        if target.is_coasting:
            target.is_coasting = False
            SwarmLogger.log(
                "GRACE", "LEADER",
                f"{t_id}: GRACE PERIOD ENDED (NoObs={time_since:.1f}s)",
                "GRACE",
            )
        
        
        # Stale check: lost for too long?
        if time_since > TARGET_STALE_COAST_S:
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
        
        
        # Velocity-based stale check
        if time_since >= TARGET_GRACE_PERIOD_S and target.last_velocity is not None:
            vel_mag = np.linalg.norm(target.last_velocity)
            if vel_mag > TARGET_STALE_VELOCITY_THRESHOLD:
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

        # Unprotected CONFIRMED → LOST
        if time_since > TRACK_LOST_TIMEOUT_SEC:
            target.track_state = "LOST"
            target.status = "LOST"
            SwarmLogger.log(
                "LIFECYCLE", "LEADER",
                f"{t_id}: CONFIRMED → LOST (NoObs={time_since:.1f}s)", "TRACK",
            )
