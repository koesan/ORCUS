"""360° rotation scanner for target acquisition."""

import time
from pymavlink import mavutil

from modules.core.logger import SwarmLogger
from config import (
    MISSION_STATUS_MESSAGES,
    ROTATION_YAW_INCREMENT, ROTATION_YAW_SPEED, ROTATION_DIRECTION,
    SCAN_DETECTION_POLL_INTERVAL_S, SCAN_YAW_SETTLE_MARGIN_S,
)


class Scanner:
    """360 rotation scanner for target acquisition."""

    def __init__(self, port=None):
        self.port = port

    def perform_360_rotation_scan(self, vehicle, is_active_check=None,
                                    detection_check=None, status_callback=None,
                                    pause_check=None, pause_waiter=None):
        """Execute 360 rotation scan. Returns True if target detected."""
        if status_callback:
            status_callback(MISSION_STATUS_MESSAGES["ROTATING_360"])
        SwarmLogger.log("INFO", f"DRONE_{self.port}", MISSION_STATUS_MESSAGES["ROTATING_360"], "SCANNER")

        rotated_total = 0

        if self._poll_detection(detection_check, is_active_check, status_callback,
                                pause_check=pause_check, pause_waiter=pause_waiter):
            return True

        while rotated_total < 360:
            if pause_check and pause_check():
                if pause_waiter:
                    pause_waiter()
                continue
            if is_active_check and not is_active_check():
                return False

            rotated_total += ROTATION_YAW_INCREMENT

            # MAV_CMD_CONDITION_YAW - incremental rotation
            msg = vehicle.message_factory.command_long_encode(
                0, 0,
                mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                0,
                ROTATION_YAW_INCREMENT,
                ROTATION_YAW_SPEED,
                ROTATION_DIRECTION,
                1,  # Relative mode
                0, 0, 0
            )

            try:
                vehicle.send_mavlink(msg)
                vehicle.flush()
            except Exception as e:
                SwarmLogger.log("ERROR", f"DRONE_{self.port}", f"Rotation command error: {e}", "SCANNER")

            # Poll continuously during the yaw instead of checking only once
            # after the step ends. This avoids late acquisition while a target
            # crosses the FOV during rotation.
            wait_time = (ROTATION_YAW_INCREMENT / max(ROTATION_YAW_SPEED, 1e-3)) + SCAN_YAW_SETTLE_MARGIN_S
            deadline = time.time() + wait_time
            while time.time() < deadline:
                if pause_check and pause_check():
                    if pause_waiter:
                        pause_waiter()
                    deadline = time.time() + SCAN_YAW_SETTLE_MARGIN_S
                    continue
                if is_active_check and not is_active_check():
                    return False
                if self._poll_detection(detection_check, is_active_check, status_callback,
                                        pause_check=pause_check, pause_waiter=pause_waiter):
                    return True
                remaining = deadline - time.time()
                if remaining <= 0:
                    break
                time.sleep(min(SCAN_DETECTION_POLL_INTERVAL_S, remaining))

        return False

    def _poll_detection(self, detection_check=None, is_active_check=None, status_callback=None,
                        pause_check=None, pause_waiter=None):
        """Run a single scan-mode detection probe with mission safety checks."""
        if pause_check and pause_check():
            if pause_waiter:
                pause_waiter()
            return False
        if is_active_check and not is_active_check():
            return False
        if detection_check and detection_check():
            SwarmLogger.log("SUCCESS", f"DRONE_{self.port}", MISSION_STATUS_MESSAGES["HUMAN_DETECTED"], "SCANNER")
            if status_callback:
                status_callback(MISSION_STATUS_MESSAGES["HUMAN_DETECTED"])
            return True
        return False
