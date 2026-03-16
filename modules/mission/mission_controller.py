"""Mission Controller — Mission lifecycle coordinator.

Navigation and Scanner are now separate modules, imported here.
Backward-compat properties are preserved.
"""

import threading
import time
from dronekit import VehicleMode

from modules.mission.flight_controller import FlightController
from modules.mission.navigation import Navigation
from modules.mission.scanner import Scanner
from modules.mission.tracking_controller import TrackingController
from modules.vision.detector import HumanTracker
from modules.core.logger import SwarmLogger
from config import (
    TAKEOFF_ALTITUDE, MISSION_STATUS_MESSAGES,
    CENTER_CONFIRM_HOLD_S, CENTER_CONFIRM_TOLERANCE_M,
    RESUME_SCAN_AFTER_LOST,
    IBVS_ALTITUDE_FLOOR_M,
    RTL_LANDING_TIMEOUT_S,
)


class MissionController:
    """Mission lifecycle coordinator for single/multi-cell scan and tracking."""

    def __init__(self, drone_manager, assigned_port=None, swarm_manager=None):
        self.drone_manager = drone_manager
        self.assigned_port = assigned_port or getattr(drone_manager, "active_drone_port", None)
        self.swarm_manager = swarm_manager or getattr(drone_manager, "swarm_manager", None)
        self._camera_handler = self.drone_manager.camera_handler

        # Sub-modules
        self.flight_ctrl = FlightController(port=self.assigned_port)
        self.navigation = Navigation(
            flight_ctrl=self.flight_ctrl,
            status_callback=self._set_status,
        )
        self.scanner = Scanner(port=self.assigned_port)
        self._human_tracker = HumanTracker()
        self.tracking_ctrl = TrackingController(
            flight_ctrl=self.flight_ctrl,
            camera_handler=self._camera_handler,
            human_tracker=self._human_tracker,
            swarm_manager=self.swarm_manager,
            drone_manager=drone_manager,
            port=self.assigned_port,
        )

        # Mission state
        self.is_mission_active = False
        self.current_mission_type = None
        self.current_mission_point = None
        self.mission_path_points = []
        self._mission_thread = None
        self._mission_lock = threading.RLock()
        self._paused = False
        self._pause_event = threading.Event()
        self._pause_event.set()
        self._stop_in_progress = False
        self.ibvs_altitude_floor_m = IBVS_ALTITUDE_FLOOR_M

    # ------------------------------------------------------------------
    # Status helper
    # ------------------------------------------------------------------

    def _set_status(self, msg):
        self.drone_manager.mission_status_message = msg

    def _is_active(self):
        return self.is_mission_active

    def _is_active_and_not_tracking(self):
        return self.is_mission_active and not self.tracking_ctrl.tracking_mode

    def _is_paused(self):
        return self._paused

    @property
    def is_paused(self):
        return self._paused

    @property
    def is_stopping(self):
        return self._stop_in_progress

    def is_running(self):
        return self.is_mission_active or (self._mission_thread is not None and self._mission_thread.is_alive())

    def _wait_if_paused(self):
        while self._paused and self.is_mission_active:
            self._pause_event.wait(timeout=0.2)

    # ------------------------------------------------------------------
    # Backward-compatible properties
    # ------------------------------------------------------------------

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

    @current_target_id.setter
    def current_target_id(self, value):
        self.tracking_ctrl.current_target_id = value

    @property
    def assigned_local_tracker_id(self):
        return self.tracking_ctrl.assigned_local_tracker_id

    @assigned_local_tracker_id.setter
    def assigned_local_tracker_id(self, value):
        self.tracking_ctrl.assigned_local_tracker_id = value

    @property
    def lock_status(self):
        return self.tracking_ctrl.lock_status

    @lock_status.setter
    def lock_status(self, value):
        self.tracking_ctrl.lock_status = value

    @property
    def target_gps_lat(self):
        return self.tracking_ctrl.target_gps_lat

    @target_gps_lat.setter
    def target_gps_lat(self, value):
        self.tracking_ctrl.target_gps_lat = value

    @property
    def target_gps_lon(self):
        return self.tracking_ctrl.target_gps_lon

    @target_gps_lon.setter
    def target_gps_lon(self, value):
        self.tracking_ctrl.target_gps_lon = value

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

    @property
    def camera_handler(self):
        return self._camera_handler

    @camera_handler.setter
    def camera_handler(self, value):
        self._camera_handler = value
        if hasattr(self, "tracking_ctrl"):
            self.tracking_ctrl.camera_handler = value
            self.tracking_ctrl.detector.camera_handler = value

    @property
    def human_tracker(self):
        return self._human_tracker

    @human_tracker.setter
    def human_tracker(self, value):
        self._human_tracker = value
        if hasattr(self, "tracking_ctrl"):
            self.tracking_ctrl.human_tracker = value
            self.tracking_ctrl.detector.human_tracker = value

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def create_grid(self, cell_centers):
        """Create traversal grid from cell center coordinates."""
        self.navigation.create_grid(cell_centers)

    def set_area(self, coordinates):
        """Set observation area and create grid."""
        self.create_grid(coordinates)
        return {
            "status": "ok",
            "message": MISSION_STATUS_MESSAGES["AREA_SET"].format(
                rows=self.num_rows, cols=self.num_cols),
        }

    def start_mission(self, mission_type, resume_point=None):
        """Start mission in a background thread."""
        with self._mission_lock:
            target_vehicle = self._get_vehicle()

            if not target_vehicle or target_vehicle == "connecting":
                self._set_status(MISSION_STATUS_MESSAGES["NO_ACTIVE"])
                return {"status": "error", "message": MISSION_STATUS_MESSAGES["NO_ACTIVE"]}

            self.current_mission_type = mission_type

            if self.is_mission_active or (self._mission_thread is not None and self._mission_thread.is_alive()):
                self._set_status(MISSION_STATUS_MESSAGES["ALREADY_ACTIVE"])
                return {"status": "error", "message": MISSION_STATUS_MESSAGES["ALREADY_ACTIVE"]}

            if not self.mission_coordinates:
                self._set_status(MISSION_STATUS_MESSAGES["NO_AREA"])
                return {"status": "error", "message": MISSION_STATUS_MESSAGES["NO_AREA"]}

            self._paused = False
            self._stop_in_progress = False
            self._pause_event.set()
            if resume_point:
                self._set_status(MISSION_STATUS_MESSAGES["RESUMING"])
            else:
                self._set_status(MISSION_STATUS_MESSAGES["STARTING"])

            self.is_mission_active = True
            self._sync_runtime_dependencies()
            self._mission_thread = threading.Thread(
                target=self._run_mission_loop,
                args=(mission_type, resume_point),
                daemon=True,
            )
            self._mission_thread.start()
            return {"status": "ok", "message": self.drone_manager.mission_status_message}

    def send_ned_velocity(self, vehicle, vx, vy, vz):
        """Backward-compatible proxy used by tests and stop/pause flows."""
        return self.flight_ctrl.send_ned_velocity(vehicle, vx, vy, vz)

    def execute_tracking_mode(self, vehicle, is_mission_active_check=None):
        """Backward-compatible proxy for legacy callers."""
        self._sync_runtime_dependencies()
        if is_mission_active_check is None:
            is_mission_active_check = lambda: self.is_mission_active
        return self.tracking_ctrl.execute_tracking_mode(
            vehicle,
            is_mission_active_check,
            pause_check=self._is_paused,
            pause_waiter=self._wait_if_paused,
        )

    def pause_mission(self):
        """Pause mission in-place and hold current position."""
        with self._mission_lock:
            active = self.is_running()
            if not active:
                return {"status": "error", "message": "Mission is not running."}
            if self._paused:
                return {"status": "error", "message": "Mission is already paused."}
            self._paused = True
            self._pause_event.clear()

        vehicle = self._get_vehicle()
        if vehicle and vehicle != "connecting":
            try:
                self.send_ned_velocity(vehicle, 0, 0, 0)
            except Exception:
                pass
            try:
                vehicle.mode = VehicleMode("GUIDED")
            except Exception:
                pass

        self._set_status("Mission paused. Holding position.")
        return {"status": "ok", "message": "Mission paused. Holding position."}

    def resume_mission(self):
        """Resume a paused mission without clearing mission state."""
        with self._mission_lock:
            if not self._paused:
                return {"status": "error", "message": "Mission is not paused."}
            self._paused = False
            self._stop_in_progress = False
            self._pause_event.set()

        vehicle = self._get_vehicle()
        if vehicle and vehicle != "connecting":
            try:
                vehicle.mode = VehicleMode("GUIDED")
            except Exception:
                pass

        self._set_status(MISSION_STATUS_MESSAGES["RESUMING"])
        return {"status": "ok", "message": MISSION_STATUS_MESSAGES["RESUMING"]}

    def soft_reset_state(self):
        """Clear mission-local state while preserving live drone connections."""
        with self._mission_lock:
            self.is_mission_active = False
            self._paused = False
            self._stop_in_progress = False
            self._pause_event.set()
            self.current_mission_type = None
            self.current_mission_point = None
            self.mission_path_points = []
            self._mission_thread = None
        self.scanning_active = False
        self.tracking_ctrl.reset_engagement_state()
        self.navigation.reset_runtime_state()
        try:
            self._camera_handler.reset_runtime_state(ports=[self.assigned_port])
        except Exception:
            pass
        self._set_status(MISSION_STATUS_MESSAGES["CONNECTED"])

    def stop_mission(self):
        """Stop active mission, RTL, and clear engagement state."""
        with self._mission_lock:
            target_vehicle = self._get_vehicle()
            active = self.is_running()
            mission_thread = self._mission_thread
            if active:
                self._stop_in_progress = True

        if target_vehicle and target_vehicle != "connecting" and active:
            # Zero velocity before RTL
            try:
                self.send_ned_velocity(target_vehicle, 0, 0, 0)
                SwarmLogger.log("STOP", f"DRONE_{self.assigned_port}",
                                "Zero velocity sent before RTL", "MISSION_CTRL")
            except Exception as e:
                SwarmLogger.log("WARNING", f"DRONE_{self.assigned_port}",
                                f"Zero velocity send failed: {e}", "MISSION_CTRL")

            try:
                target_vehicle.mode = VehicleMode("RTL")
            except Exception:
                pass

            with self._mission_lock:
                self.is_mission_active = False
                self._paused = False
                self._pause_event.set()
            self.tracking_ctrl.reset_engagement_state()
            if mission_thread and mission_thread.is_alive() and mission_thread is not threading.current_thread():
                mission_thread.join(timeout=2.0)
            with self._mission_lock:
                self._mission_thread = None

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
            return {"status": "ok", "message": MISSION_STATUS_MESSAGES["STOPPED"]}

        self._set_status(MISSION_STATUS_MESSAGES["NO_ACTIVE"])
        return {"status": "error", "message": MISSION_STATUS_MESSAGES["NO_ACTIVE"]}

    def await_rtl_completion(self, timeout_s: float = RTL_LANDING_TIMEOUT_S) -> bool:
        """Wait until the vehicle finishes RTL and is effectively landed/disarmed."""
        vehicle = self._get_vehicle()
        if not vehicle or vehicle == "connecting":
            return True

        deadline = time.time() + max(1.0, float(timeout_s))
        while time.time() < deadline:
            try:
                armed = bool(getattr(vehicle, "armed", False))
            except Exception:
                armed = False

            try:
                loc = getattr(vehicle, "location", None)
                rel = getattr(loc, "global_relative_frame", None) if loc else None
                altitude = float(getattr(rel, "alt", 0.0) or 0.0)
            except Exception:
                altitude = 0.0

            try:
                mode = getattr(getattr(vehicle, "mode", None), "name", "") or ""
            except Exception:
                mode = ""

            if (not armed) or altitude <= 0.25 or mode == "LAND":
                return True
            time.sleep(0.5)

        return False

    def get_status(self):
        """Return current mission status as dict."""
        elapsed = 0
        if self.is_mission_active and hasattr(self, "mission_start_time"):
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
                            "alt": float(getattr(g, "alt", 0.0)),
                        }
                batt = getattr(vehicle, "battery", None)
                if batt is not None and hasattr(batt, "level"):
                    battery_level = batt.level
        except Exception as e:
            SwarmLogger.log("ERROR", f"DRONE_{self.assigned_port}",
                            f"get_status error: {e}", "MISSION")

        json_grid = {
            f"{row},{col}": status
            for (row, col), status in self.grid_status.items()
        }

        return {
            "is_mission_active": self.is_mission_active,
            "is_paused": self._paused,
            "tracking_mode": self.tracking_mode,
            "collision_detected": self.collision_detected,
            "elapsed_time": elapsed,
            "grid_status": json_grid,
            "mission_path_points": self.mission_path_points,
            "current_location": current_location,
            "battery_level": battery_level,
            "status_message": status_message,
        }

    # ------------------------------------------------------------------
    # Private
    # ------------------------------------------------------------------

    def _get_vehicle(self):
        if self.assigned_port and self.assigned_port in self.drone_manager.drones:
            return self.drone_manager.drones[self.assigned_port]
        return self.drone_manager.active_drone

    def _sync_runtime_dependencies(self):
        """Propagate late-bound compatibility fields into refactored submodules."""
        self.tracking_ctrl.port = self.assigned_port
        self.tracking_ctrl.swarm_manager = self.swarm_manager or getattr(self.drone_manager, "swarm_manager", None)
        self.tracking_ctrl.bridge.port = self.assigned_port
        self.tracking_ctrl.bridge._sm = self.tracking_ctrl.swarm_manager
        self.tracking_ctrl.detector.port = self.assigned_port
        self.scanner.port = self.assigned_port
        self.flight_ctrl.port = self.assigned_port

    def _run_mission_loop(self, mission_type, start_point=None):
        """Main mission execution loop."""
        self._sync_runtime_dependencies()
        vehicle = self._get_vehicle()
        if not vehicle or vehicle == "connecting" or not self.centers_2d:
            self._set_status(MISSION_STATUS_MESSAGES["NO_AREA"])
            with self._mission_lock:
                self.is_mission_active = False
                self._mission_thread = None
            return

        try:
            self.mission_path_points = []
            self.mission_start_time = time.time()

            self.flight_ctrl.arm_and_takeoff(
                vehicle, TAKEOFF_ALTITUDE, status_callback=self._set_status)

            self._set_status(MISSION_STATUS_MESSAGES["BEGIN_MISSION_FLIGHT"])
            SwarmLogger.log("INFO", f"DRONE_{self.assigned_port}",
                            self.drone_manager.mission_status_message, "MISSION")

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
            SwarmLogger.log("ERROR", f"DRONE_{self.assigned_port}",
                            self.drone_manager.mission_status_message, "MISSION")
            try:
                vehicle.mode = VehicleMode("RTL")
            except Exception:
                pass
        finally:
            if vehicle and self.is_mission_active:
                try:
                    if self.tracking_mode or self.attack_approved:
                        SwarmLogger.log("WARNING", f"DRONE_{self.assigned_port or '?'}",
                                        "RTL BLOCKED: Tracking/Attack active", "MISSION_CTRL")
                    else:
                        self._set_status(MISSION_STATUS_MESSAGES["MISSION_COMPLETE"])
                        vehicle.mode = VehicleMode("RTL")
                except Exception:
                    pass
            with self._mission_lock:
                self.is_mission_active = False
                self._mission_thread = None
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
            self._wait_if_paused()
            if not self.is_mission_active:
                break
            self.scanning_active = False
            ok = self.navigation.approach_target(
                vehicle, target_lat, target_lon, 0, 0, 0, 0,
                is_active_check=self._is_active_and_not_tracking,
                pause_check=self._is_paused,
                pause_waiter=self._wait_if_paused,
            )

            if ok and not self.tracking_mode:
                self.scanning_active = True
                human_detected = self.scanner.perform_360_rotation_scan(
                    vehicle,
                    is_active_check=self._is_active,
                    detection_check=lambda: self.tracking_ctrl.check_human_detection(self.is_mission_active),
                    status_callback=self._set_status,
                    pause_check=self._is_paused,
                    pause_waiter=self._wait_if_paused,
                )
                if human_detected:
                    self.execute_tracking_mode(vehicle, self._is_active)
                self.scanning_active = False
            elif self.tracking_mode:
                self.execute_tracking_mode(vehicle, self._is_active)
                self.scanning_active = False

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
            self._wait_if_paused()
            if not self.is_mission_active:
                break
            self.scanning_active = False
            self.current_mission_point = (cur_row, cur_col, horiz_dir, vert_dir)
            target = self.centers_2d[cur_row][cur_col]

            if target is None:
                prev_row, prev_col = cur_row, cur_col
                cur_row, cur_col, horiz_dir, vert_dir = \
                    self.navigation.next_indices(cur_row, cur_col, horiz_dir, vert_dir)
                if cur_row == -1 and cur_col == -1:
                    SwarmLogger.log("SUCCESS", f"DRONE_{self.assigned_port}",
                                    "All cells visited", "MISSION")
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
                pause_check=self._is_paused,
                pause_waiter=self._wait_if_paused,
            )

            if self.tracking_mode:
                self.execute_tracking_mode(vehicle, self._is_active)
                self.scanning_active = False
                continue

            if ok:
                self.scanning_active = True
                hold_ok = self.navigation.confirm_center_stability(
                    vehicle, lat, lon,
                    hold_s=CENTER_CONFIRM_HOLD_S,
                    tol_m=CENTER_CONFIRM_TOLERANCE_M,
                    is_active_check=self._is_active,
                    pause_check=self._is_paused,
                    pause_waiter=self._wait_if_paused,
                )
                if hold_ok and self.is_mission_active and not self.tracking_mode:
                    human_detected = self.scanner.perform_360_rotation_scan(
                        vehicle,
                        is_active_check=self._is_active,
                        detection_check=lambda: self.tracking_ctrl.check_human_detection(self.is_mission_active),
                        status_callback=self._set_status,
                        pause_check=self._is_paused,
                        pause_waiter=self._wait_if_paused,
                    )
                    if human_detected:
                        self.execute_tracking_mode(vehicle, self._is_active)
                        self.scanning_active = False
                        continue
                self.navigation.grid_status[(cur_row, cur_col)] = "visited"
                self._set_status(MISSION_STATUS_MESSAGES["CELL_CONFIRMED"].format(
                    row=cur_row, col=cur_col))
                self.scanning_active = False
            else:
                self.navigation.grid_status[(cur_row, cur_col)] = "old"
                self._set_status(MISSION_STATUS_MESSAGES["CELL_SKIPPED"].format(row=cur_row, col=cur_col))

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
