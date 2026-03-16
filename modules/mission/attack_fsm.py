"""Attack State Machine (FSM) — Manages the 7-step attack protocol.

States:
    IDLE → ECHO → WAIT_CONFIRM → CENTERING → DIVING → IMPACT

All transitions are managed from a single point; zombie protection,
mid-attack guard, and timeout management are encapsulated within the FSM.
"""

import time
from typing import Optional

from modules.core.logger import SwarmLogger
from config import (
    ATTACK_ECHO_TIMEOUT_S, ATTACK_CENTER_TIMEOUT_S,
    ATTACK_TIMEOUT_S, LEADER_TIMEOUT_S,
)

# Protected phases — transition guard
_ACTIVE_ATTACK_PHASES = frozenset({"CENTERING", "DIVING", "CONFIRMED_ATTACK"})


class AttackFSM:
    """Attack state machine.

    All phase transitions, timeouts, and mid-attack guards are in this class.
    Provides clean attribute access instead of walrus `:=` state dict.
    """

    def __init__(self, port: int):
        self.port = port
        self.reset()

    # ------------------------------------------------------------------
    # State
    # ------------------------------------------------------------------

    def reset(self):
        """Reset all state."""
        self.phase: Optional[str] = None          # None, ECHO, WAIT_CONFIRM, CENTERING, DIVING
        self.phase_start: Optional[float] = None
        self.attack_start: Optional[float] = None
        self.attack_approved: bool = False
        self.current_target_id: Optional[str] = None
        self.assigned_local_id: Optional[int] = None
        self.search_consecutive: int = 0
        self.last_leader_response: float = time.time()
        self.center_confirm_count: int = 0
        self.impact_detected: bool = False
        # Filtered position from leader
        self.filtered_target_lat: Optional[float] = None
        self.filtered_target_lon: Optional[float] = None

    # ------------------------------------------------------------------
    # Queries
    # ------------------------------------------------------------------

    @property
    def is_active_attack(self) -> bool:
        return self.attack_approved and self.phase in _ACTIVE_ATTACK_PHASES

    @property
    def is_in_protocol(self) -> bool:
        return self.phase is not None

    def phase_elapsed(self) -> float:
        if self.phase_start is None:
            return 0.0
        return time.time() - self.phase_start

    def attack_elapsed(self) -> float:
        if self.attack_start is None:
            return 0.0
        return time.time() - self.attack_start

    # ------------------------------------------------------------------
    # Mid-Attack Guard
    # ------------------------------------------------------------------

    def guard_target_swap(self, new_target_id: Optional[str], action_label: str) -> str:
        """Prevent target change during active attack.

        Returns: target_id to use
        """
        if not self.is_active_attack:
            return new_target_id

        if (new_target_id is not None
                and self.current_target_id is not None
                and new_target_id != self.current_target_id):
            SwarmLogger.log("GUARD", f"DRONE_{self.port}",
                            f"{action_label} Ignored: Mid-attack ID swap blocked "
                            f"({new_target_id} → keeping {self.current_target_id})",
                            "ATTACK_FLOW")
            return self.current_target_id

        return new_target_id

    # ------------------------------------------------------------------
    # Phase transitions
    # ------------------------------------------------------------------

    def enter_echo(self, target_id: str, local_id: Optional[int] = None):
        """IDLE/None → ECHO."""
        if self.phase is not None:
            return  # Already in a phase
        self.current_target_id = target_id
        self.assigned_local_id = local_id
        self.phase = "ECHO"
        self.phase_start = time.time()
        self.search_consecutive = 0
        SwarmLogger.log("PROTOCOL", f"DRONE_{self.port}",
                        f"Step 2: ECHO — Target={target_id} LocalID={local_id}",
                        "ATTACK_FLOW")

    def enter_wait_confirm(self):
        """ECHO → WAIT_CONFIRM."""
        self.phase = "WAIT_CONFIRM"
        self.phase_start = time.time()

    def enter_centering(self, target_id: Optional[str] = None,
                        local_id: Optional[int] = None):
        """→ CENTERING."""
        self.phase = "CENTERING"
        self.phase_start = time.time()
        self.center_confirm_count = 0
        self.attack_approved = True
        self.attack_start = self.attack_start or time.time()
        if target_id is not None:
            self.current_target_id = target_id
        if local_id is not None:
            self.assigned_local_id = local_id
        SwarmLogger.log("PROTOCOL", f"DRONE_{self.port}",
                        f"→ CENTERING | Target={self.current_target_id}",
                        "ATTACK_FLOW")

    def enter_diving(self):
        """CENTERING → DIVING."""
        self.phase = "DIVING"
        self.phase_start = time.time()
        SwarmLogger.log("PROTOCOL", f"DRONE_{self.port}",
                        f"→ DIVING | Target={self.current_target_id}",
                        "ATTACK_FLOW")

    def confirm_impact(self, coverage: float, altitude: float):
        """Confirm impact."""
        if self.impact_detected:
            return
        self.impact_detected = True
        SwarmLogger.log("SUCCESS", f"DRONE_{self.port}",
                        f"IMPACT CONFIRMED! Coverage={coverage:.1f}% Alt={altitude:.1f}m",
                        "ATTACK_FLOW")

    def abort(self, reason: str = ""):
        """Abort the attack."""
        if reason:
            SwarmLogger.log("ABORT", f"DRONE_{self.port}",
                            f"Attack aborted: {reason}", "ATTACK_FLOW")
        self.attack_approved = False
        self.phase = None
        self.current_target_id = None
        self.assigned_local_id = None
        self.center_confirm_count = 0
        self.search_consecutive = 0

    # ------------------------------------------------------------------
    # Protocol actions (swarm response)
    # ------------------------------------------------------------------

    def on_attack_assign(self, target_id: str, local_id: Optional[int]):
        """ATTACK_ASSIGN response."""
        self.last_leader_response = time.time()
        self.search_consecutive = 0

        if self.phase is None:
            self.enter_echo(target_id, local_id)

    def on_echo_response(self, resp_action: str, target_id: Optional[str] = None,
                         local_id: Optional[int] = None):
        """Process ECHO response."""
        if resp_action == "ATTACK_CONFIRMED":
            self.enter_centering()
        elif resp_action == "REASSIGN":
            self.current_target_id = target_id
            if local_id is not None:
                self.assigned_local_id = local_id
            self.phase = None
        elif resp_action == "SEARCH":
            self.abort("ECHO → SEARCH")
        elif resp_action == "MISMATCH":
            self.abort("ECHO → MISMATCH")
        else:
            self.enter_wait_confirm()

    def on_center(self, target_id: Optional[str], local_id: Optional[int]):
        """CENTER response."""
        self.last_leader_response = time.time()
        self.search_consecutive = 0

        target_id = self.guard_target_swap(target_id, "Center")

        if self.phase in ("WAIT_CONFIRM", "ECHO", None):
            self.enter_centering(target_id, local_id)

    def on_attack(self, target_id: Optional[str], local_id: Optional[int]):
        """ATTACK response."""
        self.last_leader_response = time.time()
        self.search_consecutive = 0

        target_id = self.guard_target_swap(target_id, "Attack")

        if target_id is not None:
            self.current_target_id = target_id

        if not self.attack_approved or self.phase in (None, "ECHO", "WAIT_CONFIRM"):
            self.enter_centering(target_id, local_id)
        elif local_id is not None and local_id != self.assigned_local_id:
            self.assigned_local_id = local_id

    def on_hover(self, target_id: Optional[str], local_id: Optional[int]):
        """HOVER response."""
        self.last_leader_response = time.time()
        self.search_consecutive = 0
        if target_id is not None:
            self.current_target_id = target_id
        if local_id is not None:
            self.assigned_local_id = local_id

    def on_track(self, target_id: Optional[str], local_id: Optional[int]):
        """TRACK response."""
        self.last_leader_response = time.time()
        self.search_consecutive = 0

        if self.is_active_attack:
            SwarmLogger.log("GUARD", f"DRONE_{self.port}",
                            "Track Ignored: Cannot switch to passive track while engaging.",
                            "ATTACK_FLOW")
            return

        self.attack_approved = False
        self.phase = None
        if target_id is not None:
            self.current_target_id = target_id
        if local_id is not None:
            self.assigned_local_id = local_id

    def on_search(self, loop_count: int = 0) -> str:
        """SEARCH response.

        Returns: "CONTINUE" | "BREAK" | "OK"
        """
        from config import SWARM_IMMUTABLE_TIMEOUT_S
        self.last_leader_response = time.time()

        # Ignore SEARCH during active attack
        if self.is_active_attack:
            self.search_consecutive += 1
            if loop_count % 30 == 0:
                SwarmLogger.log("GUARD", f"DRONE_{self.port}",
                                f"SEARCH ignored — phase={self.phase}. "
                                f"Frames: {self.search_consecutive}",
                                "ATTACK_FLOW")

            # Zombie lock failsafe
            if self.search_consecutive >= (30 * SWARM_IMMUTABLE_TIMEOUT_S):
                SwarmLogger.log("TIMEOUT", f"DRONE_{self.port}",
                                f"Immutable Lock Failsafe: Target lost for "
                                f"{SWARM_IMMUTABLE_TIMEOUT_S}s",
                                "ATTACK_FLOW")
                self.abort("Immutable lock timeout")
                return "BREAK"

            return "CONTINUE"

        # Attack approved but no active phase
        if self.attack_approved:
            return "CONTINUE"

        # Passive target cleanup
        self.current_target_id = None
        self.assigned_local_id = None
        self.search_consecutive += 1

        if self.search_consecutive >= 45:
            SwarmLogger.log("STATE", f"DRONE_{self.port}",
                            "45 consecutive SEARCH — Ending Tracking.",
                            "ATTACK_FLOW")
            self.abort("45 consecutive SEARCH")
            return "BREAK"

        return "OK"

    def on_stop(self):
        """STOP command."""
        self.abort("STOP received")

    def on_abort(self, reason: str = ""):
        """ABORT / MISMATCH command."""
        self.abort(reason or "ABORT received")

    def on_reassign(self, target_id: Optional[str], local_id: Optional[int]):
        """REASSIGN response."""
        if self.is_active_attack:
            SwarmLogger.log("GUARD", f"DRONE_{self.port}",
                            f"REASSIGN to {target_id} ignored — Immutable Lock Active "
                            f"on phase {self.phase}",
                            "ATTACK_FLOW")
            return
        self.current_target_id = target_id
        self.assigned_local_id = local_id
        self.phase = None

    # ------------------------------------------------------------------
    # Timeout checks
    # ------------------------------------------------------------------

    def check_timeouts(self) -> Optional[str]:
        """Check phase timeouts.

        Returns: None | "FORCE_DIVE" | "ABORT" | "LEADER_TIMEOUT"
        """
        # ECHO timeout
        if self.phase == "ECHO" and self.phase_start:
            if self.phase_elapsed() > ATTACK_ECHO_TIMEOUT_S:
                SwarmLogger.log("TIMEOUT", f"DRONE_{self.port}",
                                f"ECHO phase timeout ({ATTACK_ECHO_TIMEOUT_S}s)",
                                "ATTACK_FLOW")
                self.abort("ECHO timeout")
                return "ABORT"

        # WAIT_CONFIRM timeout
        if self.phase == "WAIT_CONFIRM" and self.phase_start:
            if self.phase_elapsed() > ATTACK_ECHO_TIMEOUT_S:
                SwarmLogger.log("TIMEOUT", f"DRONE_{self.port}",
                                f"WAIT_CONFIRM timeout ({ATTACK_ECHO_TIMEOUT_S}s)",
                                "ATTACK_FLOW")
                self.abort("WAIT_CONFIRM timeout")
                return "ABORT"

        # CENTERING timeout → force dive
        if self.phase == "CENTERING" and self.phase_start:
            if self.phase_elapsed() > ATTACK_CENTER_TIMEOUT_S:
                SwarmLogger.log("TIMEOUT", f"DRONE_{self.port}",
                                "CENTERING timeout — forcing dive", "ATTACK_FLOW")
                self.enter_diving()
                return "FORCE_DIVE"

        # Global attack timeout
        if self.attack_approved and self.attack_start:
            if self.attack_elapsed() > ATTACK_TIMEOUT_S:
                SwarmLogger.log("TIMEOUT", f"DRONE_{self.port}",
                                f"ATTACK TIMEOUT ({ATTACK_TIMEOUT_S}s) — Aborting",
                                "ATTACK_FLOW")
                self.abort("Global attack timeout")
                return "ABORT"

        # Leader timeout (only outside active attack)
        if self.phase not in ("CENTERING", "DIVING"):
            if time.time() - self.last_leader_response > LEADER_TIMEOUT_S:
                self.last_leader_response = time.time()
                return "LEADER_TIMEOUT"

        return None
