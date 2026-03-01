"""Mission Controller - Orchestrates flight, navigation, scanning, and tracking."""

import math
import threading
import time
from collections import deque
from statistics import median
from dronekit import VehicleMode
from pymavlink import mavutil
from modules.mission.flight_controller import FlightController
from modules.mission.tracking_controller import TrackingController
from modules.core.pid_controller import PID
from modules.vision.detector import HumanTracker
from modules.core.logger import SwarmLogger
from config import (
    TAKEOFF_ALTITUDE, MISSION_STATUS_MESSAGES,
    CENTER_CONFIRM_HOLD_S, CENTER_CONFIRM_TOLERANCE_M,
    RESUME_SCAN_AFTER_LOST,
    PID_KP, PID_KI, PID_KD,
    MAX_SPEED_M_S, CONTROL_INTERVAL_S, CELL_REACHED_THRESHOLD_M,
    FINE_APPROACH_THRESHOLD_M, FINE_APPROACH_SCALE,
    CENTER_CONFIRM_TOLERANCE_M, CENTER_CONFIRM_HOLD_S,
    FINE_APPROACH_HOLD_S, PER_CELL_TIMEOUT_S,
    RETRY_LIMIT, GPS_MEDIAN_WINDOW,
    STUCK_MOVED_THRESHOLD_M, STUCK_TIMEOUT_S,
    STUCK_MANEUVER_SCALE_BACK, STUCK_MANEUVER_SCALE_FWD,
    STUCK_MANEUVER_DURATION, TIMEOUT_PENALTY_SCALE,
    ROTATION_YAW_INCREMENT, ROTATION_YAW_SPEED, ROTATION_DIRECTION
)

class Navigation:
    """Grid-based navigation with boustrophedon path planning and PID control."""
    
    def __init__(self, flight_ctrl, status_callback=None):
        self.flight_ctrl = flight_ctrl
        self._status_cb = status_callback
        self.centers_2d = []
        self.grid_status = {}
        self.num_rows = 0
        self.num_cols = 0
        self.mission_coordinates = None

    def _status(self, msg):
        if self._status_cb:
            self._status_cb(msg)

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

    def next_indices(self, row, col, horiz_dir, vert_dir):
        """Compute next cell indices in boustrophedon pattern."""
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

        return new_row, new_col, horiz_dir, vert_dir

    def get_median_location(self, vehicle, samples_deque):
        """Get median-filtered GPS position from sample buffer."""
        if not samples_deque:
            loc = vehicle.location.global_relative_frame
            return (loc.lat, loc.lon) if loc else (None, None)
        lats = [s[0] for s in samples_deque]
        lons = [s[1] for s in samples_deque]
        return median(lats), median(lons)

    def confirm_center_stability(self, vehicle, target_lat, target_lon,
                                  hold_s=None, tol_m=None, is_active_check=None):
        """Hold position and verify drone stays within tolerance."""
        hold_s = CENTER_CONFIRM_HOLD_S if hold_s is None else hold_s
        tol_m = CENTER_CONFIRM_TOLERANCE_M if tol_m is None else tol_m
        start = time.time()

        while time.time() - start < hold_s:
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

    def approach_target(self, vehicle, target_lat, target_lon,
                        from_row, from_col, to_row, to_col,
                        is_active_check=None, scanning_check=None):
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
                        is_active_check=is_active_check)
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


class Scanner:
    """360 rotation scanner for target acquisition."""
    
    def __init__(self, port=None):
        self.port = port

    def perform_360_rotation_scan(self, vehicle, is_active_check=None,
                                    detection_check=None, status_callback=None):
        """Execute 360 rotation scan. Returns True if target detected."""
        if status_callback:
            status_callback(MISSION_STATUS_MESSAGES["ROTATING_360"])
        SwarmLogger.log("INFO", f"DRONE_{self.port}", MISSION_STATUS_MESSAGES["ROTATING_360"], "SCANNER")

        target_yaw = 0

        while True:
            if is_active_check and not is_active_check():
                return False

            target_yaw = (target_yaw + ROTATION_YAW_INCREMENT) % 360

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

            # Wait for rotation
            wait_time = (ROTATION_YAW_INCREMENT / ROTATION_YAW_SPEED) + 0.25
            time.sleep(wait_time)

            # Check for target
            if detection_check and detection_check():
                SwarmLogger.log("SUCCESS", f"DRONE_{self.port}", MISSION_STATUS_MESSAGES["HUMAN_DETECTED"], "SCANNER")
                if status_callback:
                    status_callback(MISSION_STATUS_MESSAGES["HUMAN_DETECTED"])
                return True

        return False


class MissionController:
    """Mission lifecycle coordinator for single/multi-cell scan and tracking."""
    
    def __init__(self, drone_manager, assigned_port=None, swarm_manager=None):
        self.drone_manager = drone_manager
        self.assigned_port = assigned_port
        self.swarm_manager = swarm_manager
        self.camera_handler = self.drone_manager.camera_handler

        # Sub-modules
        self.flight_ctrl = FlightController(port=assigned_port)
        self.navigation = Navigation(
            flight_ctrl=self.flight_ctrl,
            status_callback=self._set_status
        )
        self.scanner = Scanner(port=assigned_port)
        self.human_tracker = HumanTracker()
        self.tracking_ctrl = TrackingController(
            flight_ctrl=self.flight_ctrl,
            camera_handler=self.camera_handler,
            human_tracker=self.human_tracker,
            swarm_manager=swarm_manager,
            drone_manager=drone_manager,
            port=assigned_port
        )

        # Mission state
        self.is_mission_active = False
        self.current_mission_type = None
        self.current_mission_point = None
        self.mission_path_points = []
        self._mission_thread = None

    def _set_status(self, msg):
        """Update drone manager status message."""
        self.drone_manager.mission_status_message = msg

    def _is_active(self):
        return self.is_mission_active

    def _is_active_and_not_tracking(self):
        return self.is_mission_active and not self.tracking_ctrl.tracking_mode

    # Backward-compatible properties
    @property
    def tracking_mode(self):
        return self.tracking_ctrl.tracking_mode

    @tracking_mode.setter
    def tracking_mode(self, value):
        self.tracking_ctrl.tracking_mode = value

    @property
    def attack_approved(self):
        return self.tracking_ctrl.attack_approved

    @attack_approved.setter
    def attack_approved(self, value):
        self.tracking_ctrl.attack_approved = value

    @property
    def collision_detected(self):
        return self.tracking_ctrl.collision_detected

    @collision_detected.setter
    def collision_detected(self, value):
        self.tracking_ctrl.collision_detected = value

    @property
    def current_target_id(self):
        return self.tracking_ctrl.current_target_id

    @property
    def assigned_local_tracker_id(self):
        return self.tracking_ctrl.assigned_local_tracker_id

    @property
    def scanning_active(self):
        return self.tracking_ctrl.scanning_active

    @scanning_active.setter
    def scanning_active(self, value):
        self.tracking_ctrl.scanning_active = value

    @property
    def grid_status(self):
        return self.navigation.grid_status

    @property
    def centers_2d(self):
        return self.navigation.centers_2d

    @property
    def num_rows(self):
        return self.navigation.num_rows

    @property
    def num_cols(self):
        return self.navigation.num_cols

    @property
    def mission_coordinates(self):
        return self.navigation.mission_coordinates

    # Public API

    def create_grid(self, cell_centers):
        """Create traversal grid from cell center coordinates."""
        self.navigation.create_grid(cell_centers)

    def set_area(self, coordinates):
        """Set observation area and create grid."""
        self.create_grid(coordinates)
        return {'status': 'ok', 'message': MISSION_STATUS_MESSAGES["AREA_SET"].format(
            rows=self.num_rows, cols=self.num_cols)}

    def start_mission(self, mission_type, resume_point=None):
        """Start mission in a background thread."""
        target_vehicle = self._get_vehicle()

        if not target_vehicle or target_vehicle == "connecting":
            self._set_status(MISSION_STATUS_MESSAGES["NO_ACTIVE"])
            return {'status': 'error', 'message': MISSION_STATUS_MESSAGES["NO_ACTIVE"]}

        self.current_mission_type = mission_type

        if self.is_mission_active:
            self._set_status(MISSION_STATUS_MESSAGES["ALREADY_ACTIVE"])
            return {"status": "error", "message": MISSION_STATUS_MESSAGES["ALREADY_ACTIVE"]}

        if not self.mission_coordinates:
            self._set_status(MISSION_STATUS_MESSAGES["NO_AREA"])
            return {'status': 'error', 'message': MISSION_STATUS_MESSAGES["NO_AREA"]}

        if resume_point:
            self._set_status(MISSION_STATUS_MESSAGES["RESUMING"])
            self._mission_thread = threading.Thread(
                target=self._run_mission_loop,
                args=(mission_type, resume_point),
                daemon=True
            )
        else:
            self._set_status(MISSION_STATUS_MESSAGES["STARTING"])
            self._mission_thread = threading.Thread(
                target=self._run_mission_loop,
                args=(mission_type,),
                daemon=True
            )

        self._mission_thread.start()
        return {'status': 'ok', 'message': self.drone_manager.mission_status_message}

    def stop_mission(self):
        """Stop active mission, RTL, and clear engagement state."""
        target_vehicle = self._get_vehicle()

        if target_vehicle and target_vehicle != "connecting" and self.is_mission_active:
            # Zero velocity before RTL
            try:
                self.flight_ctrl.send_ned_velocity(target_vehicle, 0, 0, 0)
                SwarmLogger.log("STOP", f"DRONE_{self.assigned_port}",
                    "Zero velocity sent before RTL", "MISSION_CTRL")
            except Exception as e:
                SwarmLogger.log("WARNING", f"DRONE_{self.assigned_port}",
                    f"Zero velocity send failed: {e}", "MISSION_CTRL")

            try:
                target_vehicle.mode = VehicleMode('RTL')
            except Exception:
                pass

            # Full engagement state clear
            self.is_mission_active = False
            self.tracking_ctrl.reset_engagement_state()

            SwarmLogger.log("STOP", f"DRONE_{self.assigned_port}",
                "Engagement state cleared", "MISSION_CTRL")

            # Release swarm assignment
            if self.swarm_manager and self.assigned_port:
                try:
                    self.swarm_manager.release_drone_assignment(self.assigned_port)
                    SwarmLogger.log("STOP", f"DRONE_{self.assigned_port}",
                        "Swarm assignment released", "MISSION_CTRL")
                except Exception as e:
                    SwarmLogger.log("WARNING", f"DRONE_{self.assigned_port}",
                        f"Swarm release failed: {e}", "MISSION_CTRL")

            self._set_status(MISSION_STATUS_MESSAGES["STOPPED"])
            return {'status': 'ok', 'message': MISSION_STATUS_MESSAGES["STOPPED"]}

        self._set_status(MISSION_STATUS_MESSAGES["NO_ACTIVE"])
        return {'status': 'error', 'message': MISSION_STATUS_MESSAGES["NO_ACTIVE"]}

    def get_status(self):
        """Return current mission status as dict."""
        elapsed = 0
        if self.is_mission_active and hasattr(self, 'mission_start_time'):
            try:
                elapsed = int(time.time() - self.mission_start_time)
            except Exception:
                elapsed = 0

        current_location = None
        battery_level = None
        status_message = getattr(self.drone_manager, "mission_status_message", None)

        try:
            vehicle = getattr(self.drone_manager, "active_drone", None)
            if vehicle:
                loc = getattr(vehicle, "location", None)
                if loc:
                    g = getattr(loc, "global_relative_frame", None)
                    if g and g.lat is not None:
                        current_location = {
                            "lat": float(g.lat),
                            "lon": float(g.lon),
                            "alt": float(getattr(g, "alt", 0.0))
                        }
                batt = getattr(vehicle, "battery", None)
                if batt is not None and hasattr(batt, "level"):
                    battery_level = batt.level
        except Exception as e:
            SwarmLogger.log("ERROR", f"DRONE_{self.assigned_port}", f"get_status error: {e}", "MISSION")

        json_friendly_grid_status = {
            f"{row},{col}": status
            for (row, col), status in self.grid_status.items()
        }

        return {
            "is_mission_active": self.is_mission_active,
            "tracking_mode": self.tracking_mode,
            "collision_detected": self.collision_detected,
            "elapsed_time": elapsed,
            "grid_status": json_friendly_grid_status,
            "mission_path_points": self.mission_path_points,
            "current_location": current_location,
            "battery_level": battery_level,
            "status_message": status_message
        }

    # Private methods
    def _get_vehicle(self):
        """Get the vehicle instance for this controller."""
        if self.assigned_port and self.assigned_port in self.drone_manager.drones:
            return self.drone_manager.drones[self.assigned_port]
        return self.drone_manager.active_drone

    def _run_mission_loop(self, mission_type, start_point=None):
        """Main mission execution loop."""
        vehicle = self._get_vehicle()

        if not vehicle or vehicle == "connecting" or not self.centers_2d:
            self._set_status(MISSION_STATUS_MESSAGES["NO_AREA"])
            return

        try:
            self.mission_path_points = []
            self.mission_start_time = time.time()

            self.flight_ctrl.arm_and_takeoff(
                vehicle, TAKEOFF_ALTITUDE,
                status_callback=self._set_status
            )
            self.is_mission_active = True

            self._set_status(MISSION_STATUS_MESSAGES["BEGIN_MISSION_FLIGHT"])
            SwarmLogger.log("INFO", f"DRONE_{self.assigned_port}", self.drone_manager.mission_status_message, "MISSION")

            total_cells = sum(
                1 for r in range(self.num_rows) for c in range(self.num_cols)
                if self.centers_2d[r][c] is not None
            )

            if total_cells == 1:
                self._run_single_cell(vehicle)
            else:
                self._run_multi_cell(vehicle, start_point)

        except Exception as e:
            self._set_status(MISSION_STATUS_MESSAGES["MISSION_ERROR"].format(error=str(e)))
            SwarmLogger.log("ERROR", f"DRONE_{self.assigned_port}", self.drone_manager.mission_status_message, "MISSION")
            try:
                vehicle.mode = VehicleMode('RTL')
            except Exception:
                pass
        finally:
            if vehicle and self.is_mission_active:
                try:
                    if self.tracking_mode or self.attack_approved:
                        SwarmLogger.log("WARNING",
                            f"DRONE_{self.assigned_port or '?'}",
                            "RTL BLOCKED: Tracking/Attack active", "MISSION_CTRL")
                    else:
                        self._set_status(MISSION_STATUS_MESSAGES["MISSION_COMPLETE"])
                        vehicle.mode = VehicleMode('RTL')
                except Exception:
                    pass
            self.is_mission_active = False
            self.tracking_ctrl.tracking_mode = False
            self._set_status(MISSION_STATUS_MESSAGES["STOPPED"])

    def _run_single_cell(self, vehicle):
        """Single-cell mode: approach center, 360° scan, track."""
        target_lat, target_lon = None, None
        for r in range(self.num_rows):
            for c in range(self.num_cols):
                if self.centers_2d[r][c] is not None:
                    target_lat, target_lon = self.centers_2d[r][c]
                    break
            if target_lat is not None:
                break

        if target_lat is None:
            return

        while self.is_mission_active:
            ok = self.navigation.approach_target(
                vehicle, target_lat, target_lon, 0, 0, 0, 0,
                is_active_check=self._is_active_and_not_tracking,
                scanning_check=lambda: self.tracking_ctrl.check_human_detection(self.is_mission_active)
            )

            if ok and not self.tracking_mode:
                self.scanning_active = True
                human_detected = self.scanner.perform_360_rotation_scan(
                    vehicle,
                    is_active_check=self._is_active,
                    detection_check=lambda: self.tracking_ctrl.check_human_detection(self.is_mission_active),
                    status_callback=self._set_status
                )
                if human_detected:
                    self.tracking_ctrl.execute_tracking_mode(vehicle, self._is_active)
            elif self.tracking_mode:
                self.tracking_ctrl.execute_tracking_mode(vehicle, self._is_active)

            time.sleep(1.0)

    def _run_multi_cell(self, vehicle, start_point=None):
        """Multi-cell boustrophedon scan mode."""
        if start_point:
            cur_row, cur_col, horiz_dir, vert_dir = start_point
        else:
            cur_row, cur_col = self.num_rows - 1, 0
            horiz_dir, vert_dir = 1, -1

        prev_row, prev_col = None, None

        while self.is_mission_active and not self.tracking_mode:
            self.current_mission_point = (cur_row, cur_col, horiz_dir, vert_dir)
            target = self.centers_2d[cur_row][cur_col]

            if target is None:
                prev_row, prev_col = cur_row, cur_col
                cur_row, cur_col, horiz_dir, vert_dir = \
                    self.navigation.next_indices(cur_row, cur_col, horiz_dir, vert_dir)
                if cur_row == -1 and cur_col == -1:
                    SwarmLogger.log("SUCCESS", f"DRONE_{self.assigned_port}", "All cells visited", "MISSION")
                    self.is_mission_active = False
                continue

            lat, lon = target
            self._set_status(MISSION_STATUS_MESSAGES["SCANNING_AREA"].format(
                row=cur_row, col=cur_col))

            if prev_row is not None:
                self.scanning_active = True

            ok = self.navigation.approach_target(
                vehicle, lat, lon, prev_row, prev_col, cur_row, cur_col,
                is_active_check=self._is_active_and_not_tracking,
                scanning_check=lambda: self.tracking_ctrl.check_human_detection(self.is_mission_active)
            )

            if self.tracking_mode:
                self.tracking_ctrl.execute_tracking_mode(vehicle, self._is_active)
                self.scanning_active = True
                continue

            if ok:
                self.scanning_active = True
                self.navigation.grid_status[(cur_row, cur_col)] = 'visited'
                self._set_status(MISSION_STATUS_MESSAGES["CELL_CONFIRMED"].format(
                    row=cur_row, col=cur_col))
                self.navigation.confirm_center_stability(
                    vehicle, lat, lon,
                    hold_s=CENTER_CONFIRM_HOLD_S,
                    tol_m=CENTER_CONFIRM_TOLERANCE_M,
                    is_active_check=self._is_active
                )
            else:
                self.navigation.grid_status[(cur_row, cur_col)] = 'old'
                self._set_status(MISSION_STATUS_MESSAGES["CELL_SKIPPED"].format(
                    row=cur_row, col=cur_col))

            prev_row, prev_col = cur_row, cur_col
            cur_row, cur_col, horiz_dir, vert_dir = \
                self.navigation.next_indices(cur_row, cur_col, horiz_dir, vert_dir)

            if cur_row == -1 and cur_col == -1:
                self.is_mission_active = False

            if cur_row != prev_row:
                time.sleep(0.12)
            time.sleep(0.08)

# Backward-compatible alias
CollisionMissionController = MissionController
