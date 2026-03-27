"""Mission lifecycle orchestrator."""

import threading
import time
from dronekit import VehicleMode

from modules.mission.flight_controller import FlightController
from modules.mission.navigation import Navigation
from modules.mission.attack_controller import AttackController
from modules.vision.detector import HumanTracker
from modules.core.logger import SwarmLogger, MissionState, MissionPhase
from config import (
    TAKEOFF_ALTITUDE, MISSION_STATUS_MESSAGES,
    RTL_LANDING_TIMEOUT_S,
    MISSION_ROW_TRANSITION_DELAY_S,
    MISSION_CELL_LOOP_DELAY_S,
    MISSION_PROGRESS_LOG_INTERVAL_S,
    STATUS_SNAPSHOT_LOG_INTERVAL_S,
    TERMINAL_COMPLETE_HOVER_S,
    TERMINAL_COMPLETE_LOG_INTERVAL_S,
)


class MissionController:
    """Mission lifecycle coordinator for single/multi-cell scan and tracking."""

    def __init__(self, drone_manager, assigned_port=None, swarm_manager=None):
        self.drone_manager = drone_manager
        self.assigned_port = assigned_port or getattr(drone_manager, "active_drone_port", None)
        self.swarm_manager = swarm_manager or getattr(drone_manager, "swarm_manager", None)
        self._camera_handler = self.drone_manager.camera_handler

        self.flight_ctrl = FlightController(port=self.assigned_port)
        self.navigation = Navigation(
            flight_ctrl=self.flight_ctrl,
            status_callback=self._set_status,
            drone_manager=self.drone_manager,
        )
        self._human_tracker = HumanTracker()
        self.attack_ctrl = AttackController(
            flight_ctrl=self.flight_ctrl,
            camera_handler=self._camera_handler,
            human_tracker=self._human_tracker,
            swarm_manager=self.swarm_manager,
            drone_manager=drone_manager,
            port=self.assigned_port,
        )

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
        self.state = MissionState()
        self.current_nav_target = None
        self._last_status_signature = None
        self._status_message = getattr(self.drone_manager, "mission_status_message", "")

    def _set_status(self, msg):
        self._status_message = msg
        self.drone_manager.mission_status_message = msg

    def _set_phase(self, phase, reason=None):
        """Update mission phase and keep status transitions observable."""
        if not hasattr(self, "state") or self.state is None:
            self.state = MissionState()
        prev = getattr(self.state, "phase", MissionPhase.IDLE)
        self.state.transition(phase, reason=reason)
        if prev != phase:
            SwarmLogger.log_phase_change(
                module=f"DRONE_{self.assigned_port}",
                entity="MISSION",
                old_phase=prev,
                new_phase=phase,
                source="MISSION_CTRL",
                reason=reason,
                level="STATE",
                event_type="mission_phase_transition",
                structured_source="MISSION_CTRL",
                structured_level="STATE",
                drone_id=self.assigned_port,
            )

    def _log_mission_progress(self, event: str, message: str,
                              interval_s: float = MISSION_PROGRESS_LOG_INTERVAL_S,
                              level: str = "INFO"):
        SwarmLogger.log_throttled(
            level,
            f"DRONE_{self.assigned_port}",
            message,
            f"MISSION_{event}",
            interval_s=interval_s,
        )

    def _log_status_snapshot(self, status: dict):
        route = status.get("route_visual") or {}
        current = status.get("current_location") or {}
        planned = status.get("planned_terminal_point")
        event = status.get("terminal_event")
        signature = (
            status.get("mission_phase"),
            bool(status.get("tracking_mode")),
            bool(status.get("is_mission_active")),
            bool(status.get("collision_detected")),
            status.get("status_message"),
            len(route.get("transit_path") or []),
            len(route.get("scan_path") or []),
            bool(planned),
            event.get("id") if isinstance(event, dict) else None,
        )
        if signature == self._last_status_signature:
            return
        self._last_status_signature = signature
        lat = current.get("lat")
        lon = current.get("lon")
        alt = current.get("alt")
        loc_str = (
            f"({float(lat):.6f},{float(lon):.6f},{float(alt or 0.0):.1f}m)"
            if lat is not None and lon is not None else "(no-fix)"
        )
        self._log_mission_progress(
            "STATUS",
            f"Status snapshot | phase={status.get('mission_phase')} mission={status.get('is_mission_active')} "
            f"tracking={status.get('tracking_mode')} collision={status.get('collision_detected')} "
            f"route_transit={len(route.get('transit_path') or [])} route_scan={len(route.get('scan_path') or [])} "
            f"planned_terminal={bool(planned)} terminal_event={(event.get('kind') or event.get('type')) if isinstance(event, dict) else '-'} "
            f"loc={loc_str} status=\"{status.get('status_message')}\"",
            interval_s=STATUS_SNAPSHOT_LOG_INTERVAL_S,
            level="STATUS",
        )

    @property
    def mission_phase(self):
        state = getattr(self, "state", None)
        return getattr(state, "phase", MissionPhase.IDLE)

    def _is_active(self):
        return self.is_mission_active

    def _is_active_and_not_tracking(self):
        return self.is_mission_active and not self.attack_ctrl.tracking_mode

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

    def _handle_terminal_completion(self, vehicle):
        """After a confirmed terminal event, fly through the target briefly before holding for RTL."""
        self._set_phase(MissionPhase.TERMINAL_HOLD, reason="terminal event confirmed")
        self._set_status("Terminal event confirmed. Executing fly-through coasting, then holding for RTL.")
        
        coast_deadline = time.time() + 1.5
        SwarmLogger.log(
            "STATUS",
            f"DRONE_{self.assigned_port}",
            "Terminal impact confirmed — executing 1.5s fly-through coasting",
            "MISSION_CTRL"
        )
        while time.time() < coast_deadline:
            if vehicle and vehicle != "connecting":
                try:
                    cmd = self.attack_ctrl._coast_visual_dive_command()
                    if cmd:
                        vx, vy, vz, yaw = cmd
                        self.flight_ctrl.command_body_velocity(
                            vehicle, "ATTACK_FLY_THROUGH", vx, vy, vz, yaw,
                            attack_approved=True, reason="post-impact fly-through coasting"
                        )
                except Exception:
                    pass
            time.sleep(0.05)

        deadline = time.time() + max(0.0, float(TERMINAL_COMPLETE_HOVER_S))
        event = self.attack_ctrl.get_terminal_event() or {}
        event_kind = event.get("kind") or event.get("type") or "terminal_event"
        while time.time() < deadline:
            if vehicle and vehicle != "connecting":
                try:
                    self.flight_ctrl.command_hold(vehicle, reason="terminal completion hold", failsafe=True)
                except Exception:
                    pass
            remaining = max(0.0, deadline - time.time())
            SwarmLogger.log_throttled(
                "STATUS",
                f"DRONE_{self.assigned_port}",
                f"Terminal hold active | event={event_kind} remaining={remaining:.1f}s",
                "MISSION_CTRL",
                interval_s=TERMINAL_COMPLETE_LOG_INTERVAL_S,
            )
            time.sleep(0.2)

        SwarmLogger.log(
            "STOP",
            f"DRONE_{self.assigned_port}",
            "Terminal hold complete — initiating drone-local RTL",
            "MISSION_CTRL",
        )
        self.stop_mission()
        self.await_rtl_completion()
        self.soft_reset_state()

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
        if hasattr(self, "attack_ctrl"):
            self.attack_ctrl.camera_handler = value
            self.attack_ctrl.detector.camera_handler = value

    @property
    def human_tracker(self):
        return self._human_tracker

    @human_tracker.setter
    def human_tracker(self, value):
        self._human_tracker = value
        if hasattr(self, "attack_ctrl"):
            self.attack_ctrl.human_tracker = value
            self.attack_ctrl.detector.human_tracker = value

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def create_grid(self, cell_centers):
        """Create traversal grid from cell center coordinates."""
        self.navigation.create_grid(cell_centers)

    def set_area(self, coordinates):
        """Set observation area and create grid."""
        self.create_grid(coordinates)
        scan_points = self.navigation.compute_scan_route_points()
        if scan_points:
            first = scan_points[0]
            last = scan_points[-1]
            SwarmLogger.log(
                "INFO",
                f"DRONE_{self.assigned_port}",
                f"Area assigned | scan_points={len(scan_points)} "
                f"entry=({first['row']},{first['col']}) exit=({last['row']},{last['col']})",
                "ROUTE",
            )
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
            self._set_phase(MissionPhase.TRANSIT, reason="mission start requested")
            if resume_point:
                self._set_status(MISSION_STATUS_MESSAGES["RESUMING"])
            else:
                self._set_status(MISSION_STATUS_MESSAGES["STARTING"])
            self._log_mission_progress(
                "START",
                f"Mission start requested | type={mission_type} resume_point={resume_point}",
                interval_s=0.0,
            )

            self.is_mission_active = True
            self._sync_runtime_dependencies()
            self._mission_thread = threading.Thread(
                target=self._run_mission_loop,
                args=(mission_type, resume_point),
                daemon=True,
            )
            self._mission_thread.start()
            return {"status": "ok", "message": self.drone_manager.mission_status_message}

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
                self.flight_ctrl.command_hold(vehicle, reason="mission pause")
            except Exception:
                pass
            try:
                vehicle.mode = VehicleMode("GUIDED")
            except Exception:
                pass

        self._set_phase(MissionPhase.PAUSED, reason="pause requested")
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

        next_phase = MissionPhase.TRANSIT if getattr(self, "current_mission_type", None) else MissionPhase.IDLE
        self._set_phase(next_phase, reason="resume requested")
        self._set_status(MISSION_STATUS_MESSAGES["RESUMING"])
        return {"status": "ok", "message": MISSION_STATUS_MESSAGES["RESUMING"]}

    def soft_reset_state(self):
        """Clear mission-local state while preserving live drone connections."""
        prev_phase = self.mission_phase
        with self._mission_lock:
            self.is_mission_active = False
            self._paused = False
            self._stop_in_progress = False
            self._pause_event.set()
            self.current_mission_type = None
            self.current_mission_point = None
            self.current_nav_target = None
            self.mission_path_points = []
            self._mission_thread = None
            self._set_phase(MissionPhase.IDLE, reason="soft reset")
            if prev_phase != MissionPhase.RTL:
                self.navigation.clear_transit_reservation()
        self.attack_ctrl.scanning_active = False
        self.attack_ctrl.reset_engagement_state()
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
                self.flight_ctrl.send_ned_velocity(target_vehicle, 0, 0, 0)
                self.flight_ctrl.command_world_velocity(
                    target_vehicle, "RTL_PREP", 0.0, 0.0, 0.0, reason="mission stop"
                )
                SwarmLogger.log("STOP", f"DRONE_{self.assigned_port}",
                                "Zero velocity sent before RTL", "MISSION_CTRL")
            except Exception as e:
                SwarmLogger.log("WARNING", f"DRONE_{self.assigned_port}",
                                f"Zero velocity send failed: {e}", "MISSION_CTRL")

            self.navigation.wait_for_rtl_slot(target_vehicle)

            try:
                target_vehicle.mode = VehicleMode("RTL")
            except Exception:
                pass

            with self._mission_lock:
                self.is_mission_active = False
                self._paused = False
                self._pause_event.set()
                self._set_phase(MissionPhase.RTL, reason="stop mission")
                self.current_nav_target = None
            self.attack_ctrl.reset_engagement_state()
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
        status_message = getattr(self, "_status_message", None) or getattr(self.drone_manager, "mission_status_message", None)

        try:
            vehicle = self._get_vehicle()
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

        status = {
            "is_mission_active": self.is_mission_active,
            "is_paused": self._paused,
            "mission_phase": self.mission_phase,
            "tracking_mode": self.attack_ctrl.tracking_mode,
            "collision_detected": self.attack_ctrl.collision_detected,
            "elapsed_time": elapsed,
            "grid_status": json_grid,
            "mission_path_points": self.mission_path_points,
            "current_location": current_location,
            "battery_level": battery_level,
            "status_message": status_message,
            "route_visual": self.navigation.build_route_visual(
                current_location=current_location,
                mission_phase=self.mission_phase,
                current_nav_target=self.current_nav_target,
                vehicle=self._get_vehicle(),
            ),
            "planned_terminal_point": self.attack_ctrl.get_planned_terminal_point(),
            "terminal_event": self.attack_ctrl.get_terminal_event(),
            "attack_phase": self.attack_ctrl.fsm.phase,
            "attack_target_id": self.attack_ctrl.fsm.assigned_target_id,
            "attack_autonomous": self.attack_ctrl.fsm.is_autonomous,
        }
        self._log_status_snapshot(status)
        return status

    # ------------------------------------------------------------------
    # Private
    # ------------------------------------------------------------------

    def _get_vehicle(self):
        if self.assigned_port and self.assigned_port in self.drone_manager.drones:
            return self.drone_manager.drones[self.assigned_port]
        return self.drone_manager.active_drone

    def _sync_runtime_dependencies(self):
        """Propagate late-bound runtime dependencies into submodules."""
        self.attack_ctrl.port = self.assigned_port
        self.attack_ctrl.swarm_manager = self.swarm_manager or getattr(self.drone_manager, "swarm_manager", None)
        self.attack_ctrl.bridge.port = self.assigned_port
        self.attack_ctrl.bridge._sm = self.attack_ctrl.swarm_manager
        self.attack_ctrl.detector.port = self.assigned_port
        self.flight_ctrl.port = self.assigned_port
        self.flight_ctrl.motion.port = self.assigned_port
        self.navigation.flight_ctrl = self.flight_ctrl
        self.navigation.scanner.port = self.assigned_port
        self.navigation.scanner.flight_ctrl = self.flight_ctrl

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
            self._set_phase(MissionPhase.TAKEOFF, reason="arm and takeoff")

            self.flight_ctrl.arm_and_takeoff(
                vehicle, TAKEOFF_ALTITUDE, status_callback=self._set_status)

            self._set_status(MISSION_STATUS_MESSAGES["BEGIN_MISSION_FLIGHT"])
            self._set_phase(MissionPhase.TRANSIT, reason="takeoff complete")
            self.flight_ctrl.set_motion_intent("TRANSIT", reason="post takeoff mission transit")
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
            self._set_phase(MissionPhase.ERROR, reason=str(e))
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
                    if self.attack_ctrl.tracking_mode or self.attack_ctrl.attack_approved:
                        SwarmLogger.log("WARNING", f"DRONE_{self.assigned_port or '?'}",
                                        "RTL BLOCKED: Tracking/Attack active", "MISSION_CTRL")
                    else:
                        self._set_phase(MissionPhase.COMPLETE, reason="mission finished")
                        self._set_status(MISSION_STATUS_MESSAGES["MISSION_COMPLETE"])
                        vehicle.mode = VehicleMode("RTL")
                except Exception:
                    pass
            with self._mission_lock:
                self.is_mission_active = False
                self._mission_thread = None
                self.current_nav_target = None
                self.navigation.clear_transit_reservation()
            self.attack_ctrl.tracking_mode = False
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
            self.attack_ctrl.scanning_active = False
            self._set_phase(MissionPhase.TRANSIT, reason="single-cell approach")
            self.current_nav_target = (target_lat, target_lon)
            self._log_mission_progress(
                "SINGLE_TRANSIT",
                f"Single-cell approach | lat={target_lat:.6f} lon={target_lon:.6f}",
            )
            ok = self.navigation.approach_target(
                vehicle, target_lat, target_lon, 0, 0, 0, 0,
                is_active_check=self._is_active_and_not_tracking,
                pause_check=self._is_paused,
                pause_waiter=self._wait_if_paused,
            )

            if ok and not self.attack_ctrl.tracking_mode:
                self.current_nav_target = None
                self.attack_ctrl.scanning_active = True
                self._set_phase(MissionPhase.SCAN, reason="single-cell sweep")
                human_detected = self.navigation.perform_scan(
                    vehicle,
                    detection_check=lambda: self.attack_ctrl.check_human_detection(),
                    is_active_check=self._is_active,
                    pause_check=self._is_paused,
                    pause_waiter=self._wait_if_paused,
                )
                if human_detected:
                    self.execute_tracking_mode(vehicle, self._is_active)
                self.attack_ctrl.scanning_active = False
            elif self.attack_ctrl.tracking_mode:
                self.current_nav_target = None
                self.execute_tracking_mode(vehicle, self._is_active)
                self.attack_ctrl.scanning_active = False
                if self.is_mission_active and not self._stop_in_progress:
                    continue

            if not ok and self.is_mission_active and not self._stop_in_progress and not self.attack_ctrl.tracking_mode:
                self._log_mission_progress(
                    "SINGLE_RETRY",
                    "Single-cell approach incomplete; retrying approach/scan loop",
                    level="WARNING",
                )

            time.sleep(1.0)

    def _run_multi_cell(self, vehicle, start_point=None):
        """Multi-cell boustrophedon scan mode."""
        if start_point:
            cur_row, cur_col, horiz_dir, vert_dir = start_point
        else:
            cur_row, cur_col = self.num_rows - 1, 0
            horiz_dir, vert_dir = 1, -1

        prev_row, prev_col = None, None

        while self.is_mission_active and not self.attack_ctrl.tracking_mode:
            self._wait_if_paused()
            if not self.is_mission_active:
                break
            self.attack_ctrl.scanning_active = False
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
            self.current_nav_target = (lat, lon)
            self._set_status(MISSION_STATUS_MESSAGES["SCANNING_AREA"].format(
                row=cur_row, col=cur_col))
            self._set_phase(MissionPhase.TRANSIT, reason=f"cell {cur_row},{cur_col} approach")
            self._log_mission_progress(
                "CELL_TRANSIT",
                f"Approaching cell=({cur_row},{cur_col}) from=({prev_row},{prev_col}) "
                f"lat={lat:.6f} lon={lon:.6f} dir=({horiz_dir},{vert_dir})",
            )

            if prev_row is not None:
                self.attack_ctrl.scanning_active = True

            ok = self.navigation.approach_target(
                vehicle, lat, lon, prev_row, prev_col, cur_row, cur_col,
                is_active_check=self._is_active_and_not_tracking,
                pause_check=self._is_paused,
                pause_waiter=self._wait_if_paused,
            )

            if self.attack_ctrl.tracking_mode:
                self.current_nav_target = None
                self.execute_tracking_mode(vehicle, self._is_active)
                self.attack_ctrl.scanning_active = False
                continue

            if ok:
                self.current_nav_target = None
                self.attack_ctrl.scanning_active = True
                self._set_phase(MissionPhase.SCAN, reason=f"cell {cur_row},{cur_col} scan")
                self._log_mission_progress(
                    "CELL_HOLD",
                    f"Cell hold/confirm | cell=({cur_row},{cur_col}) lat={lat:.6f} lon={lon:.6f}",
                )
                hold_ok, human_detected = self.navigation.scan_confirmed_cell(
                    vehicle, lat, lon, cur_row, cur_col,
                    detection_check=lambda: self.attack_ctrl.check_human_detection(),
                    is_active_check=self._is_active,
                    pause_check=self._is_paused,
                    pause_waiter=self._wait_if_paused,
                )
                if human_detected:
                    self.execute_tracking_mode(vehicle, self._is_active)
                    self.attack_ctrl.scanning_active = False
                    continue
                self._set_status(MISSION_STATUS_MESSAGES["CELL_CONFIRMED"].format(
                    row=cur_row, col=cur_col))
                self._log_mission_progress(
                    "CELL_DONE",
                    f"Cell complete | cell=({cur_row},{cur_col}) hold_ok={hold_ok}",
                )
                self.attack_ctrl.scanning_active = False
            else:
                if self._stop_in_progress or not self.is_mission_active:
                    self.current_nav_target = None
                    break
                self.navigation.grid_status[(cur_row, cur_col)] = "old"
                self._set_status(MISSION_STATUS_MESSAGES["CELL_SKIPPED"].format(row=cur_row, col=cur_col))
                self._log_mission_progress(
                    "CELL_SKIP",
                    f"Cell skipped | cell=({cur_row},{cur_col})",
                    level="WARNING",
                )

            prev_row, prev_col = cur_row, cur_col
            cur_row, cur_col, horiz_dir, vert_dir = \
                self.navigation.next_indices(cur_row, cur_col, horiz_dir, vert_dir)

            if cur_row == -1 and cur_col == -1:
                self.is_mission_active = False

            if cur_row != prev_row:
                time.sleep(MISSION_ROW_TRANSITION_DELAY_S)
            time.sleep(MISSION_CELL_LOOP_DELAY_S)

    def execute_tracking_mode(self, vehicle, is_mission_active_check=None):
        """Mission-aware tracking wrapper with explicit recovery phases."""
        self._sync_runtime_dependencies()
        self.navigation.clear_transit_reservation()
        if is_mission_active_check is None:
            is_mission_active_check = lambda: self.is_mission_active

        phase = MissionPhase.ATTACK if self.attack_ctrl.attack_approved else MissionPhase.TRACK
        self._set_phase(phase, reason="tracking entered")
        result = self.attack_ctrl.execute_tracking_mode(
            vehicle,
            is_mission_active_check,
            pause_check=self._is_paused,
            pause_waiter=self._wait_if_paused,
        )

        if not self.is_mission_active:
            return result

        # SCAN_RESUME: graceful return to scan loop (not a terminal event)
        if self.attack_ctrl.fsm.phase == "SCAN_RESUME":
            self.attack_ctrl.reset_engagement_state()
            self._set_phase(MissionPhase.RETURN_TO_SCAN, reason="scan resume after verify timeout")
            return result

        if self.attack_ctrl.has_terminal_completion_pending() or self.attack_ctrl.collision_detected:
            self._handle_terminal_completion(vehicle)
            return result

        if self.attack_ctrl.fsm.phase is None and not self.attack_ctrl.attack_approved:
            self._set_phase(MissionPhase.RETURN_TO_SCAN, reason="tracking ended; resume scan")
        else:
            self._set_phase(MissionPhase.ATTACK_ABORT, reason="tracking disengaged")
        return result
