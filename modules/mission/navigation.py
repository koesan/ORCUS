"""Grid-based navigation with boustrophedon path planning and PID control."""

import math
import time
from collections import deque
from statistics import median

from modules.mission.flight_controller import FlightController
from modules.core.pid_controller import PID
from modules.core.logger import SwarmLogger
from config import (
    MISSION_STATUS_MESSAGES,
    PID_KP, PID_KI, PID_KD,
    MAX_SPEED_M_S, CONTROL_INTERVAL_S, CELL_REACHED_THRESHOLD_M,
    FINE_APPROACH_THRESHOLD_M, FINE_APPROACH_SCALE,
    CENTER_CONFIRM_TOLERANCE_M, CENTER_CONFIRM_HOLD_S,
    FINE_APPROACH_HOLD_S, PER_CELL_TIMEOUT_S,
    RETRY_LIMIT, GPS_MEDIAN_WINDOW,
    STUCK_MOVED_THRESHOLD_M, STUCK_TIMEOUT_S,
    STUCK_MANEUVER_SCALE_BACK, STUCK_MANEUVER_SCALE_FWD,
    STUCK_MANEUVER_DURATION, TIMEOUT_PENALTY_SCALE,
)


class Navigation:
    """Grid-based navigation with boustrophedon path planning and PID control."""

    def __init__(self, flight_ctrl: FlightController, status_callback=None):
        self.flight_ctrl = flight_ctrl
        self._status_cb = status_callback
        self.centers_2d = []
        self.grid_status = {}
        self.num_rows = 0
        self.num_cols = 0
        self.mission_coordinates = None

    # ------------------------------------------------------------------
    # Status
    # ------------------------------------------------------------------

    def _status(self, msg):
        if self._status_cb:
            self._status_cb(msg)

    # ------------------------------------------------------------------
    # Grid creation
    # ------------------------------------------------------------------

    def create_grid(self, cell_centers):
        """Create traversal grid from list of (lat, lon) cell centers."""
        self.mission_coordinates = cell_centers
        self.grid_status = {}
        self.centers_2d = []

        if not cell_centers:
            self.num_rows = self.num_cols = 0
            return

        total = len(cell_centers)
        num_cols = int(math.sqrt(total)) or 1
        num_rows = math.ceil(total / num_cols)
        self.num_rows, self.num_cols = num_rows, num_cols

        self.centers_2d = [[None for _ in range(num_cols)] for _ in range(num_rows)]
        idx = 0
        for r in range(num_rows):
            for c in range(num_cols):
                if idx < total:
                    lat, lon = cell_centers[idx]
                    self.centers_2d[r][c] = (lat, lon)
                    self.grid_status[(r, c)] = 'unvisited'
                    idx += 1
                else:
                    self.centers_2d[r][c] = None

        self._status(MISSION_STATUS_MESSAGES["AREA_SET"].format(rows=num_rows, cols=num_cols))
        SwarmLogger.log("INFO", "Navigation", f"Grid created: {num_rows}x{num_cols}", "GRID")

    def reset_runtime_state(self):
        """Clear mission-local grid state without touching live vehicle links."""
        self.centers_2d = []
        self.grid_status = {}
        self.num_rows = 0
        self.num_cols = 0
        self.mission_coordinates = None

    # ------------------------------------------------------------------
    # Boustrophedon traversal
    # ------------------------------------------------------------------

    def next_indices(self, row, col, horiz_dir, vert_dir):
        """Compute next cell indices in boustrophedon pattern."""
        if self.num_rows <= 0 or self.num_cols <= 0:
            return -1, -1, horiz_dir, vert_dir

        next_col = col + horiz_dir

        if 0 <= next_col < self.num_cols and self.centers_2d[row][next_col] is not None:
            return row, next_col, horiz_dir, vert_dir

        next_row = row + vert_dir
        if 0 <= next_row < self.num_rows:
            horiz_dir *= -1
            new_col = 0 if horiz_dir == 1 else self.num_cols - 1
            return next_row, new_col, horiz_dir, vert_dir

        vert_dir *= -1
        horiz_dir *= -1
        new_row = row + vert_dir
        new_col = 0 if horiz_dir == 1 else self.num_cols - 1

        if not (0 <= new_row < self.num_rows):
            return -1, -1, horiz_dir, vert_dir

        return new_row, new_col, horiz_dir, vert_dir

    # ------------------------------------------------------------------
    # GPS filtering
    # ------------------------------------------------------------------

    @staticmethod
    def get_median_location(vehicle, samples_deque):
        """Get median-filtered GPS position from sample buffer."""
        if not samples_deque:
            loc = vehicle.location.global_relative_frame
            return (loc.lat, loc.lon) if loc else (None, None)
        lats = [s[0] for s in samples_deque]
        lons = [s[1] for s in samples_deque]
        return median(lats), median(lons)

    # ------------------------------------------------------------------
    # Position stability
    # ------------------------------------------------------------------

    def confirm_center_stability(self, vehicle, target_lat, target_lon,
                                  hold_s=None, tol_m=None, is_active_check=None,
                                  pause_check=None, pause_waiter=None):
        """Hold position and verify drone stays within tolerance."""
        hold_s = CENTER_CONFIRM_HOLD_S if hold_s is None else hold_s
        tol_m = CENTER_CONFIRM_TOLERANCE_M if tol_m is None else tol_m
        start = time.time()

        while time.time() - start < hold_s:
            if pause_check and pause_check():
                if pause_waiter:
                    pause_waiter()
                start = time.time()
                continue
            if is_active_check and not is_active_check():
                return False
            loc = vehicle.location.global_relative_frame
            if not loc:
                return False
            d = FlightController.haversine_distance_m(loc.lat, loc.lon, target_lat, target_lon)
            self.flight_ctrl.send_ned_velocity(vehicle, 0, 0, 0)
            if d > tol_m:
                return False
            time.sleep(0.05)
        return True

    # ------------------------------------------------------------------
    # Approach with PID + stuck detection
    # ------------------------------------------------------------------

    def approach_target(self, vehicle, target_lat, target_lon,
                        from_row, from_col, to_row, to_col,
                        is_active_check=None, scanning_check=None,
                        pause_check=None, pause_waiter=None):
        """Navigate to target GPS with PID control and stuck detection."""
        port = self.flight_ctrl.port

        for attempt in range(RETRY_LIMIT + 1):
            pid_n = PID(kp=PID_KP, ki=PID_KI, kd=PID_KD)
            pid_e = PID(kp=PID_KP, ki=PID_KI, kd=PID_KD)

            samples = deque(maxlen=GPS_MEDIAN_WINDOW)
            start_ts = time.time()
            last_move_ts = time.time()
            last_pos = None

            while True:
                if pause_check and pause_check():
                    if pause_waiter:
                        pause_waiter()
                    continue
                if is_active_check and not is_active_check():
                    return False

                loc = vehicle.location.global_relative_frame
                if loc and loc.lat is not None:
                    samples.append((loc.lat, loc.lon))

                # Check for human detection during cruise
                if scanning_check and scanning_check():
                    SwarmLogger.log("SUCCESS", f"DRONE_{port}", "CRUISE: Human detected!", "DETECTION")
                    return False

                if not samples:
                    time.sleep(CONTROL_INTERVAL_S)
                    continue

                filt_lat, filt_lon = self.get_median_location(vehicle, samples)
                d_north, d_east = FlightController.latlon_to_north_east_m(
                    filt_lat, filt_lon, target_lat, target_lon)
                dist = math.hypot(d_north, d_east)
                speed = math.hypot(vehicle.velocity[0], vehicle.velocity[1])

                self._status(MISSION_STATUS_MESSAGES["APPROACHING"].format(
                    from_row=from_row, from_col=from_col,
                    to_col=to_col, to_row=to_row, distance=dist, speed=speed))

                # Cell reached check
                if dist <= CELL_REACHED_THRESHOLD_M:
                    self.flight_ctrl.send_ned_velocity(vehicle, 0, 0, 0)
                    hold_ok = self.confirm_center_stability(
                        vehicle, target_lat, target_lon,
                        hold_s=FINE_APPROACH_HOLD_S, tol_m=CELL_REACHED_THRESHOLD_M,
                        is_active_check=is_active_check,
                        pause_check=pause_check,
                        pause_waiter=pause_waiter)
                    if hold_ok:
                        return True
                    else:
                        pid_n.reset()
                        pid_e.reset()
                        time.sleep(0.12)
                        continue

                # PID velocity computation
                vx = pid_n.step(d_north, CONTROL_INTERVAL_S)
                vy = pid_e.step(d_east, CONTROL_INTERVAL_S)

                if dist < FINE_APPROACH_THRESHOLD_M:
                    vx *= FINE_APPROACH_SCALE
                    vy *= FINE_APPROACH_SCALE

                speed = math.hypot(vx, vy)
                if speed > MAX_SPEED_M_S:
                    scale = MAX_SPEED_M_S / speed
                    vx *= scale
                    vy *= scale

                try:
                    self.flight_ctrl.send_ned_velocity(vehicle, vx, vy, 0)
                except Exception as e:
                    SwarmLogger.log("ERROR", f"DRONE_{port}", f"Velocity send error: {e}", "NAVIGATION")

                # Stuck detection
                if last_pos is None and loc:
                    last_pos = (loc.lat, loc.lon)
                    last_move_ts = time.time()
                elif loc:
                    moved = FlightController.haversine_distance_m(
                        last_pos[0], last_pos[1], loc.lat, loc.lon)
                    if moved > STUCK_MOVED_THRESHOLD_M:
                        last_pos = (loc.lat, loc.lon)
                        last_move_ts = time.time()
                    elif time.time() - last_move_ts > STUCK_TIMEOUT_S:
                        self._status(MISSION_STATUS_MESSAGES["STUCK"])
                        SwarmLogger.log("WARNING", f"DRONE_{port}", MISSION_STATUS_MESSAGES["STUCK"], "NAVIGATION")
                        try:
                            self.flight_ctrl.send_ned_velocity(
                                vehicle, -vx * STUCK_MANEUVER_SCALE_BACK,
                                -vy * STUCK_MANEUVER_SCALE_BACK, 0)
                            time.sleep(STUCK_MANEUVER_DURATION)
                            self.flight_ctrl.send_ned_velocity(
                                vehicle, vx * STUCK_MANEUVER_SCALE_FWD,
                                vy * STUCK_MANEUVER_SCALE_FWD, 0)
                        except Exception as e:
                            SwarmLogger.log("ERROR", f"DRONE_{port}", f"Maneuver error: {e}", "NAVIGATION")
                        last_move_ts = time.time()

                # Timeout check
                elapsed = time.time() - start_ts
                timeout = PER_CELL_TIMEOUT_S * (1 + attempt * TIMEOUT_PENALTY_SCALE)
                if elapsed > timeout:
                    SwarmLogger.log("WARNING", f"DRONE_{port}", MISSION_STATUS_MESSAGES["TIMEOUT"], "NAVIGATION")
                    self._status(MISSION_STATUS_MESSAGES["TIMEOUT"])
                    break

                time.sleep(CONTROL_INTERVAL_S)

        return False
