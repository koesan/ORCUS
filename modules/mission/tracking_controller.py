"""ORCUS Tracking Controller — Detection, tracking, and attack orchestrator.

Responsibilities:
  - Frame processing with DetectionProcessor
  - Attack state management with AttackFSM
  - Communication with SwarmBridge
  - Velocity command generation with IBVSGuidance
  - Annotated frame streaming

"""

import time
import math
import logging
import cv2
from typing import Optional

from modules.mission.flight_controller import FlightController
from modules.mission.detection_processor import DetectionProcessor, DetectionResult
from modules.mission.attack_fsm import AttackFSM
from modules.mission.swarm_bridge import SwarmBridge, SwarmResponse
from modules.mission.ibvs_guidance import IBVSGuidance, VelocitySmoother, LowPassFilter
from modules.core.logger import SwarmLogger
from modules.core.geo_math import GeoMath
from config import (
    CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_RESOLUTION_WIDTH, CENTERING_TO_DIVING_HOLD_S,
    IBVS_VX_MAX, IBVS_VX_MIN, IBVS_VZ_MAX, IBVS_ALTITUDE_FLOOR_M, IBVS_SHALLOW_DIVE_VZ_MAX,
    JPEG_QUALITY, MISSION_STATUS_MESSAGES, ATTACK_CENTER_ERROR_THRESHOLD, ATTACK_CENTER_CONFIRM_FRAMES,
    CENTERING_YAW_KP, CENTERING_YAW_RATE_MAX, CENTERING_YAW_DEADZONE, IMPACT_CONFIRM_FRAMES,
    CENTERING_SETTLE_FRAMES, CENTERING_YAW_FILTER_ALPHA, IMPACT_COVERAGE_THRESHOLD, IMPACT_ALTITUDE_MIN_M,
    IMPACT_ALTITUDE_PROGRESSIVE, IMPACT_ALTITUDE_HIGH_M, IMPACT_ALTITUDE_LOW_M,
    TRACKING_PIXEL_DEADZONE_PX,
    OBSERVER_TARGET_LOSS_GRACE_S,
)


class TrackingController:
    """Human tracking and attack state machine."""

    def __init__(self, flight_ctrl, camera_handler, human_tracker,
                 swarm_manager, drone_manager, port=None):
        """Initialize tracking controller with required components."""
        self.flight_ctrl: FlightController = flight_ctrl
        self.camera_handler = camera_handler
        self.human_tracker = human_tracker
        self.swarm_manager = swarm_manager
        self.drone_manager = drone_manager
        self.port = port

        # Sub-modules
        self.detector = DetectionProcessor(port, camera_handler, human_tracker, drone_manager)
        self.fsm = AttackFSM(port)
        self.bridge = SwarmBridge(swarm_manager, drone_manager, port)
        self.ibvs = IBVSGuidance()
        self.smoother = VelocitySmoother(max_accel=1.0, max_decel=2.0)

        # Tracking state
        self.tracking_mode = False
        self.scanning_active = False
        self.collision_detected = False
        self.target_gps_lat: Optional[float] = None
        self.target_gps_lon: Optional[float] = None
        self._legacy_lock_status: Optional[str] = None

        # Centering stabilization
        self._centering_yaw_filter: Optional[LowPassFilter] = None
        self._centering_settle_count = 0

        # Impact detection
        self._impact_confirm_count = 0

        # Phase transition buffer
        self._phase_transition_start: Optional[float] = None

    # ------------------------------------------------------------------
    # Backward compat accessors
    # ------------------------------------------------------------------

    @property
    def attack_approved(self):
        return self.fsm.attack_approved

    @attack_approved.setter
    def attack_approved(self, value):
        self.fsm.attack_approved = value

    @property
    def current_target_id(self):
        return self.fsm.current_target_id

    @current_target_id.setter
    def current_target_id(self, value):
        self.fsm.current_target_id = value

    @property
    def assigned_local_tracker_id(self):
        return self.fsm.assigned_local_id

    @assigned_local_tracker_id.setter
    def assigned_local_tracker_id(self, value):
        self.fsm.assigned_local_id = value

    @property
    def lock_status(self):
        if self._legacy_lock_status is not None and not self.fsm.is_in_protocol:
            return self._legacy_lock_status
        if self.fsm.phase == "ECHO":
            return "REQUESTING"
        if self.fsm.is_active_attack:
            return "LOCKED"
        if self.fsm.attack_approved:
            return "APPROVED"
        return None

    @lock_status.setter
    def lock_status(self, value):
        self._legacy_lock_status = value

    # ------------------------------------------------------------------
    # Reset
    # ------------------------------------------------------------------

    def reset_engagement_state(self):
        """Clear all engagement state — called on stop mission."""
        self.fsm.reset()
        try:
            self.human_tracker.set_primary_target(None)
        except Exception:
            pass
        try:
            self.human_tracker.reset()
        except Exception:
            pass
        try:
            self.detector.reset_runtime_state()
        except Exception:
            pass
        try:
            self.camera_handler.reset_runtime_state(ports=[self.port])
        except Exception:
            pass
        self.ibvs.reset()
        self.smoother.reset()
        self.collision_detected = False
        self.tracking_mode = False
        self.target_gps_lat = None
        self.target_gps_lon = None
        self._legacy_lock_status = None
        self._centering_yaw_filter = None
        self._centering_settle_count = 0
        self._impact_confirm_count = 0
        self._phase_transition_start = None

    # ------------------------------------------------------------------
    # Scan-mode detection (check_human_detection)
    # ------------------------------------------------------------------

    def check_human_detection(self, is_mission_active) -> bool:
        """Check camera for human detections and report to swarm manager."""
        if not self.scanning_active:
            return False

        try:
            detections = self.detector.process_scan_frame(
                attack_approved=self.fsm.attack_approved,
                locked_id=self.fsm.assigned_local_id)

            if not detections:
                return False

            found = False
            vehicle = self.drone_manager.drones.get(self.port)
            if vehicle is None or vehicle == "connecting":
                return False
            loc = vehicle.location.global_relative_frame
            if not loc or loc.lat == 0:
                return False

            d_lat, d_lon, d_alt = loc.lat, loc.lon, loc.alt
            heading = vehicle.heading

            for det in detections:
                if det.lat is None:
                    continue

                raw_data = det.to_swarm_dict(
                    (d_lat, d_lon, d_alt), heading,
                    (vehicle.attitude.roll, vehicle.attitude.pitch, vehicle.attitude.yaw))

                # LOCKED state: heartbeat only
                if self.lock_status == "LOCKED":
                    self.bridge.send_heartbeat(
                        self.fsm.current_target_id,
                        self.fsm.assigned_local_id,
                        (d_lat, d_lon, d_alt))
                    found = True
                    continue

                # REQUESTING: heartbeat
                if self.lock_status == "REQUESTING":
                    self.bridge.send_heartbeat(
                        self.fsm.current_target_id,
                        self.fsm.assigned_local_id,
                        (d_lat, d_lon, d_alt))
                    continue

                # Attack mode: freeze new target reporting (prevent ghost targets)
                if self.fsm.attack_approved and self.fsm.current_target_id:
                    if self._matches_locked_detection(det):
                        self.bridge.send_locked_data(
                            self.fsm.current_target_id,
                            (d_lat, d_lon, d_alt),
                            target_gps=(det.lat, det.lon),
                            bbox=raw_data.get("bbox"),
                        )
                    else:
                        self.bridge.send_heartbeat(
                            self.fsm.current_target_id,
                            self.fsm.assigned_local_id,
                            (d_lat, d_lon, d_alt))
                    found = True
                    continue

                # Normal report
                resp = self.bridge.report_target(
                    det.lat, det.lon, det.conf,
                    tracker_id=det.track_id,
                    raw_data=raw_data,
                    covariance=det.covariance)

                found = self._handle_handshake(
                    resp, det, d_lat, d_lon, d_alt, found)

            return found

        except Exception as e:
            # Throttle error log: max once per 10 seconds
            now = time.time()
            if now - getattr(self, '_last_det_error_log', 0) > 10.0:
                self._last_det_error_log = now
                SwarmLogger.log("ERROR", f"DRONE_{self.port}",
                                f"Human detection check error: {e}", "TRACKING")
            return False

    def _handle_handshake(self, resp: SwarmResponse, det: DetectionResult,
                          d_lat, d_lon, d_alt, found) -> bool:
        """Process swarm action response and execute handshake protocol."""
        if resp.action == "ATTACK":
            req_resp = self.bridge.report_target(
                det.lat, det.lon, det.conf,
                tracker_id=det.track_id,
                raw_data={"action": "REQUEST_LOCK", "target_id": resp.target_id,
                          "drone_gps": (d_lat, d_lon, d_alt)})
            if req_resp.action == "ATTACK_APPROVED":
                resp = req_resp

        if resp.action == "ATTACK_APPROVED":
            self.bridge.report_target(
                det.lat, det.lon, det.conf,
                tracker_id=det.track_id,
                raw_data={"action": "LOCKED", "target_id": resp.target_id})

            self.fsm.enter_centering(resp.target_id, resp.local_id)
            if resp.local_id is not None:
                self.human_tracker.set_primary_target(resp.local_id)
                self.ibvs.set_target_id(resp.local_id)

            SwarmLogger.log("SUCCESS", "ATTACK",
                            f"LOCKED on {resp.target_id}. Engaging...",
                            f"DRONE_{self.port}")
            return True

        if resp.action == "HOVER":
            self.fsm.attack_approved = False
            return True

        return found

    def _matches_locked_detection(self, det: DetectionResult) -> bool:
        """Does this detection correspond to the locally assigned attack target?"""
        assigned = self.fsm.assigned_local_id
        if assigned is None:
            return False
        if det.track_id is not None and int(det.track_id) == int(assigned):
            return True
        if det.is_group and det.member_track_ids:
            return int(assigned) in set(int(mid) for mid in det.member_track_ids)
        return False

    # ------------------------------------------------------------------
    # Tracking mode (main loop)
    # ------------------------------------------------------------------

    def execute_tracking_mode(self, vehicle, is_mission_active_check,
                              pause_check=None, pause_waiter=None):
        """Main tracking loop with 7-step attack protocol and IBVS guidance."""
        self.tracking_mode = True
        target_port = self.port or self.drone_manager.active_drone_port

        SwarmLogger.log("STATE", f"DRONE_{target_port}",
                        f"TRACKING MODE STARTED | Mode={vehicle.mode.name} | "
                        f"Alt={vehicle.location.global_relative_frame.alt:.2f}m | "
                        f"Attack={self.fsm.attack_approved} | Target={self.fsm.current_target_id}")

        loop_count = 0
        last_reported = {}

        try:
            while is_mission_active_check() and self.tracking_mode:
                if pause_check and pause_check():
                    if pause_waiter:
                        pause_waiter()
                    continue
                loop_count += 1
                target_port = self.port or self.drone_manager.active_drone_port

                # Update tracker attitude
                self._update_tracker_pose(vehicle, target_port, loop_count)

                # Process frame
                result, detections = self.detector.process_frame(
                    attack_approved=self.fsm.attack_approved,
                    locked_id=self.fsm.assigned_local_id,
                    debug=True,
                    attack_mode=self.fsm.attack_approved)

                if result is None:
                    if loop_count % 20 == 0:
                        SwarmLogger.log("WARNING", f"DRONE_{target_port}", f"Cannot get frame — loop #{loop_count}", "TRACKING")
                    time.sleep(0.05)
                    continue

                # Stream annotated frame
                self._stream_annotated_frame(result, target_port)

                # Process swarm commands
                if self.drone_manager.swarm_manager:
                    flow = self._process_swarm(vehicle, result, detections, target_port, loop_count, last_reported)
                    if flow == "BREAK":
                        break
                    if flow == "CONTINUE":
                        continue

                # Generate velocity commands
                if self._generate_velocity(vehicle, result, detections, target_port, loop_count) == "BREAK":
                    break

                time.sleep(0.01)

        except Exception as e:
            SwarmLogger.log("ERROR", f"DRONE_{target_port}", f"TRACKING MODE ERROR: {e}", "TRACKING")
            logging.error("TRACKING MODE ERROR", exc_info=True)
        finally:
            self._tracking_cleanup(vehicle, target_port, loop_count, is_mission_active_check)

    # ------------------------------------------------------------------
    # Tracking loop helpers
    # ------------------------------------------------------------------

    def _update_tracker_pose(self, vehicle, target_port, loop_count):
        """Update tracker with vehicle attitude and pose."""
        try:
            att = vehicle.attitude
            self.human_tracker.set_attitude(att.pitch, att.roll)
            loc = vehicle.location.global_relative_frame
            if loc and loc.lat != 0:
                self.human_tracker.set_drone_pose(
                    loc.lat, loc.lon, loc.alt, att.roll, att.pitch, att.yaw)
        except Exception as e:
            if loop_count == 1:
                SwarmLogger.log("WARNING", f"DRONE_{target_port}",
                                f"Attitude/pose read error: {e}", "TRACKING")

    def _stream_annotated_frame(self, result, target_port):
        """Encode and push annotated frame."""
        if "annotated_frame" not in result or result["annotated_frame"] is None:
            return
        annotated = result["annotated_frame"].copy()

        if result.get("track_id") is not None:
            cv2.putText(annotated, f"Track ID: {result['track_id']}", (10, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
        if self.fsm.phase:
            cv2.putText(annotated, f"PHASE: {self.fsm.phase}", (10, 140),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        ret, buf = cv2.imencode(".jpg", annotated, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
        if ret:
            jpeg = buf.tobytes()
            with self.camera_handler.lock:
                if target_port in self.camera_handler.camera_feeds:
                    entry = self.camera_handler.camera_feeds[target_port]
                    if isinstance(entry, dict):
                        entry["jpeg"] = jpeg
                else:
                    self.camera_handler.camera_feeds[target_port] = {"raw": None, "jpeg": jpeg}

    def _tracking_cleanup(self, vehicle, target_port, loop_count, is_check):
        """Tracking loop cleanup."""
        from config import RESUME_SCAN_AFTER_LOST

        should_resume_scan = (not self.collision_detected and RESUME_SCAN_AFTER_LOST and is_check())
        SwarmLogger.log("STATE", f"DRONE_{target_port}",
                        f"TRACKING ENDED | Frames={loop_count} | Attack={self.fsm.attack_approved}")
        try:
            self.flight_ctrl.send_ned_velocity(vehicle, 0, 0, 0)
        except Exception:
            pass

        self.reset_engagement_state()
        if should_resume_scan:
            SwarmLogger.log("INFO", f"DRONE_{target_port}", "Resume scanning...")

    # ------------------------------------------------------------------
    # Swarm command processing
    # ------------------------------------------------------------------

    def _process_swarm(self, vehicle, result, detections, target_port,
                       loop_count, last_reported) -> Optional[str]:
        """Process swarm commands. Returns "BREAK" / "CONTINUE" / None."""
        loc = vehicle.location.global_relative_frame
        if not loc:
            return None
        d_lat, d_lon, d_alt = loc.lat, loc.lon, loc.alt
        heading = vehicle.heading
        att = (vehicle.attitude.roll, vehicle.attitude.pitch, vehicle.attitude.yaw)

        # Build reports
        reports = []
        for det in detections:
            if det.lat is not None:
                reports.append(det.to_swarm_dict((d_lat, d_lon, d_alt), heading, att))
                if det.track_id is not None:
                    last_reported[det.track_id] = (det.lat, det.lon)

        # Phase-specific handling
        if self.fsm.phase == "CENTERING" and self.fsm.current_target_id:
            self._send_centering_data(reports, d_lat, d_lon, d_alt)
        elif self.fsm.phase == "DIVING":
            locked_reports = self._filter_locked_reports(reports)
            if locked_reports:
                self.bridge.send_detection_packet(
                    locked_reports, self.fsm.current_target_id, "DIVING")
        else:
            # Normal packet
            resp = self.bridge.send_detection_packet(
                reports, self.fsm.current_target_id,
                self.drone_manager.mission_status_message)

            # Geo-validation
            resp = self.bridge.geo_validate(resp, reports)

            # Handle protocol action
            flow = self._dispatch_action(resp, vehicle, target_port,
                                         loop_count, result, reports,
                                         d_lat, d_lon, d_alt)
            if flow:
                return flow

        # Check timeouts
        timeout = self.fsm.check_timeouts()
        if timeout == "ABORT":
            self.drone_manager.mission_status_message = "TARGET BURNT - TIMEOUT"
            return "BREAK"
        if timeout == "LEADER_TIMEOUT":
            if not self.fsm.attack_approved:
                self.drone_manager.mission_status_message = "FAILSAFE: HOVER — No Leader Connection"

        return None

    def _filter_locked_reports(self, reports):
        """Keep only the currently assigned local target during attack phases."""
        if not reports:
            return []
        locked_id = self.fsm.assigned_local_id
        if locked_id is None:
            return []
        exact = [r for r in reports if r.get("track_id") == locked_id]
        if exact:
            return exact
        member_match = [
            r for r in reports
            if locked_id in set(r.get("member_ids", []) or [])
        ]
        return member_match

    def _send_centering_data(self, reports, d_lat, d_lon, d_alt):
        """Send locked target GPS during CENTERING phase."""
        locked = next((r for r in reports
                       if r.get("track_id") == self.fsm.assigned_local_id), None)
        if locked:
            world = locked.get("world_xyz", (None, None, None))
            target_gps = (world[0], world[1]) if world[0] is not None else None
            self.bridge.send_locked_data(
                self.fsm.current_target_id,
                (d_lat, d_lon, d_alt),
                target_gps=target_gps,
                bbox=locked.get("bbox"))

    def _dispatch_action(self, resp: SwarmResponse, vehicle, target_port,
                         loop_count, result, reports,
                         d_lat, d_lon, d_alt) -> Optional[str]:
        """FSM action dispatch. Returns flow control or None."""
        action = resp.action
        if action in ("ATTACK_ASSIGN", "CENTER", "ATTACK", "REASSIGN") and self._stop_in_progress():
            SwarmLogger.log(
                "GUARD", f"DRONE_{target_port}",
                f"{action} ignored — stop/RTL in progress",
                "ATTACK_FLOW",
            )
            self.fsm.on_stop()
            self.human_tracker.set_primary_target(None)
            return "BREAK"

        if action == "ATTACK_ASSIGN":
            self.fsm.on_attack_assign(resp.target_id, resp.local_id)
            # Tracker target set - local_id is the drone's own ID
            if resp.local_id is not None:
                self.human_tracker.set_primary_target(resp.local_id)
                self.ibvs.set_target_id(resp.local_id)
            # Send echo
            echo_resp = self.bridge.send_echo(
                resp.target_id, (d_lat, d_lon, d_alt),
                resp.local_id, result.get("bbox"))
            echo_resp = self.bridge.geo_validate(echo_resp, reports)
            self.fsm.on_echo_response(
                echo_resp.action, echo_resp.target_id, echo_resp.local_id)
            if echo_resp.local_id is not None:
                self.human_tracker.set_primary_target(echo_resp.local_id)
                self.ibvs.set_target_id(echo_resp.local_id)

        elif action == "CENTER":
            self.fsm.on_center(resp.target_id, resp.local_id)
            self._set_tracker_target(resp.local_id)

        elif action == "ATTACK":
            self.fsm.on_attack(resp.target_id, resp.local_id)
            self._set_tracker_target(resp.local_id)
            # Use filtered position from leader
            if resp.lat is not None and resp.lon is not None:
                self.fsm.filtered_target_lat = resp.lat
                self.fsm.filtered_target_lon = resp.lon
                SwarmLogger.log("DEBUG", f"DRONE_{target_port}",
                    f"Filtered position from leader: lat={resp.lat:.6f} lon={resp.lon:.6f}",
                    "FILTER")

        elif action == "HOVER":
            self.fsm.on_hover(resp.target_id, resp.local_id)
            # Use filtered position from leader
            if resp.lat is not None and resp.lon is not None:
                self.fsm.filtered_target_lat = resp.lat
                self.fsm.filtered_target_lon = resp.lon

        elif action == "TRACK":
            self.fsm.on_track(resp.target_id, resp.local_id)
            self._set_tracker_target(resp.local_id)

        elif action == "SEARCH":
            result_search = self.fsm.on_search(loop_count)
            if result_search == "BREAK":
                self.human_tracker.set_primary_target(None)
                self.drone_manager.mission_status_message = "TARGET BURNT - RETURNING TO SCAN"
                return "BREAK"
            if result_search == "CONTINUE":
                return "CONTINUE"

        elif action == "STOP":
            self.fsm.on_stop()
            self.human_tracker.set_primary_target(None)

        elif action in ("ABORT", "MISMATCH"):
            self.fsm.on_abort(action)
            self.human_tracker.set_primary_target(None)
            return "BREAK"

        elif action == "REASSIGN":
            self.fsm.on_reassign(resp.target_id, resp.local_id)
            self._set_tracker_target(resp.local_id)

        return None

    def _stop_in_progress(self) -> bool:
        """If mission controller stop flow has started, reject attack commands."""
        controllers = getattr(self.drone_manager, "drone_controllers", {}) or {}
        controller = controllers.get(self.port)
        return bool(getattr(controller, "is_stopping", False))

    def _set_tracker_target(self, tracker_id):
        """Helper: assign target ID to tracker and IBVS."""
        if tracker_id is not None:
            self.human_tracker.set_primary_target(tracker_id)
            self.ibvs.set_target_id(tracker_id)

    # ------------------------------------------------------------------
    # Velocity command generation
    # ------------------------------------------------------------------

    def _generate_velocity(self, vehicle, result, detections,
                           target_port, loop_count) -> Optional[str]:
        """Generate velocity commands. Returns "BREAK" or None."""
        if not result.get("detected", False):
            self._handle_no_detection(vehicle)
            return None

        # Impact already confirmed — stop
        if self.fsm.impact_detected:
            try:
                self.flight_ctrl.send_ned_velocity(vehicle, 0, 0, 0)
            except Exception:
                pass
            self.tracking_mode = False
            return "BREAK"

        coverage = result.get("screen_coverage", 0) * 100

        # Observer mode (pre-attack)
        if not self.fsm.attack_approved:
            self._observer_velocity(vehicle, result, coverage)
            return None

        # Attack mode status
        self.drone_manager.mission_status_message = \
            MISSION_STATUS_MESSAGES["TRACKING_HUMAN"].format(coverage=coverage)


        # Get bbox
        bbox = result.get("bbox")
        track_id = result.get("track_id")
        vx, vy, vz, yaw_rate = 0.0, 0.0, 0.0, 0.0

        # Lock guard
        if self.fsm.attack_approved and self.fsm.assigned_local_id is not None:
            if track_id is not None and int(track_id) != int(self.fsm.assigned_local_id):
                bbox = None
                track_id = self.fsm.assigned_local_id

        if bbox and len(bbox) == 4:
            x1, y1, x2, y2 = bbox
            bbox_center = ((x1 + x2) / 2, (y1 + y2) / 2)
            bbox_size = (x2 - x1, y2 - y1)
            frame_size = (CAMERA_RESOLUTION_WIDTH, CAMERA_HEIGHT)

            if self.fsm.phase == "CENTERING":
                vx, vy, vz, yaw_rate = self._centering_velocity(
                    bbox_center, frame_size, target_port)
            elif self.fsm.phase in ("DIVING", None):
                vx, vy, vz, yaw_rate = self._diving_velocity(
                    vehicle, bbox_center, bbox_size, frame_size,
                    track_id, coverage, target_port, loop_count)
            else:
                vx, vy, vz, yaw_rate = 0.0, 0.0, 0.0, 0.0
        else:
            vx, vy, vz, yaw_rate = self.ibvs.compute_velocity(
                None, None, (CAMERA_RESOLUTION_WIDTH, CAMERA_HEIGHT), None)

        # Send
        try:
            vz = max(0.0, min(vz, IBVS_VZ_MAX))
            if self.fsm.attack_approved:
                self.flight_ctrl.send_ned_velocity_with_yaw_rate(vehicle, vx, vy, vz, yaw_rate)
            else:
                self.flight_ctrl.safe_track_velocity(vehicle, vx, vy, vz, yaw_rate, False)
        except Exception as e:
            SwarmLogger.log("ERROR", f"DRONE_{target_port}", f"Velocity command failed: {e}")

        return None

    def _handle_no_detection(self, vehicle):
        """No detection — ghost mode or hover."""
        self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["HUMAN_LOST"]
        if self.fsm.attack_approved:
            # Attack mode ghost velocity — continue with IBVS decay
            vx, vy, vz, yaw_rate = self.ibvs.compute_velocity(
                None, None, (CAMERA_RESOLUTION_WIDTH, CAMERA_HEIGHT), None)
            try:
                vz = max(0.0, min(vz, IBVS_VZ_MAX))
                self.flight_ctrl.send_ned_velocity_with_yaw_rate(vehicle, vx, vy, vz, yaw_rate)
            except Exception:
                pass
        else:
            try:
                time_since_seen = time.time() - float(getattr(self.human_tracker, "last_detection_time", 0.0) or 0.0)
                if 0.0 < time_since_seen <= OBSERVER_TARGET_LOSS_GRACE_S:
                    last_cmd = getattr(self.human_tracker, "last_velocity_command", {}) or {}
                    yaw_rate = float(last_cmd.get("yaw_rate", 0.0))
                    decay = max(0.0, 1.0 - (time_since_seen / OBSERVER_TARGET_LOSS_GRACE_S))
                    self.flight_ctrl.send_ned_velocity_with_yaw_rate(
                        vehicle, 0.0, 0.0, 0.0, yaw_rate * decay
                    )
                else:
                    self.flight_ctrl.send_ned_velocity(vehicle, 0, 0, 0)
            except Exception:
                pass

    def _observer_velocity(self, vehicle, result, coverage):
        """Observer mode — rotate in place, no translation."""
        self.drone_manager.mission_status_message = \
            f"Tracking - Waiting Approval ({coverage:.1f}%)"
        vel_cmd = result.get("velocity_cmd", {})
        yaw_rate = vel_cmd.get("yaw_rate", 0.0)
        if abs(yaw_rate) < 0.05:
            yaw_rate = 0.0
        try:
            self.flight_ctrl.send_ned_velocity_with_yaw_rate(vehicle, 0.0, 0.0, 0.0, yaw_rate)
        except Exception:
            pass

    def _centering_velocity(self, bbox_center, frame_size, target_port):
        """CENTERING phase — yaw to center target."""
        center_x = frame_size[0] / 2
        error_px = bbox_center[0] - center_x
        error_x_norm = error_px / center_x

        if self._centering_yaw_filter is None:
            self._centering_yaw_filter = LowPassFilter(alpha=CENTERING_YAW_FILTER_ALPHA)

        if abs(error_px) < TRACKING_PIXEL_DEADZONE_PX:
            error_x_norm = 0.0
        elif abs(error_x_norm) < CENTERING_YAW_DEADZONE:
            error_x_norm = 0.0

        if abs(error_x_norm) < ATTACK_CENTER_ERROR_THRESHOLD:
            self.fsm.center_confirm_count += 1
            yaw_rate = 0.0
            self._centering_yaw_filter.reset()
        else:
            self.fsm.center_confirm_count = 0
            yaw_raw = error_x_norm * CENTERING_YAW_KP
            yaw_rate = self._centering_yaw_filter.update(yaw_raw)
            yaw_rate = max(min(yaw_rate, CENTERING_YAW_RATE_MAX), -CENTERING_YAW_RATE_MAX)

        # Settle check
        if self.fsm.center_confirm_count >= ATTACK_CENTER_CONFIRM_FRAMES:
            self._centering_settle_count += 1
            if self._centering_settle_count >= CENTERING_SETTLE_FRAMES:
                SwarmLogger.log("PROTOCOL", f"DRONE_{target_port}",
                                f"Step 7: TARGET CENTERED → DIVING! "
                                f"(settled {self._centering_settle_count} frames)",
                                "ATTACK_FLOW")
                if self.swarm_manager and self.fsm.current_target_id:
                    self.bridge.send_attacking(self.fsm.current_target_id)

                self.fsm.enter_diving()
                self._phase_transition_start = time.time()
                self._centering_settle_count = 0
            else:
                self.drone_manager.mission_status_message = \
                    f"CENTERING - SETTLING ({self._centering_settle_count}/{CENTERING_SETTLE_FRAMES})"
        else:
            self._centering_settle_count = 0
            self.drone_manager.mission_status_message = \
                f"CENTERING ({self.fsm.center_confirm_count}/{ATTACK_CENTER_CONFIRM_FRAMES})"

        return 0.0, 0.0, 0.0, yaw_rate

    def _diving_velocity(self, vehicle, bbox_center, bbox_size, frame_size,
                         track_id, coverage, target_port, loop_count):
        """DIVING phase — IBVS guidance."""
        # Transition buffer
        if self._phase_transition_start is not None:
            elapsed = time.time() - self._phase_transition_start
            if elapsed < CENTERING_TO_DIVING_HOLD_S:
                self.drone_manager.mission_status_message = "TRANSITION → DIVING"
                return 0.0, 0.0, 0.0, 0.0
            self._phase_transition_start = None

        try:
            drone_pitch = vehicle.attitude.pitch
        except Exception:
            drone_pitch = 0.0

        vx, vy, vz, yaw_rate = self.ibvs.compute_velocity(
            bbox_center=bbox_center, bbox_size=bbox_size,
            frame_size=frame_size, current_tracker_id=track_id,
            drone_pitch=drone_pitch)
        vx, vy, vz = self.smoother.smooth(vx, vy, vz)

        current_alt = vehicle.location.global_relative_frame.alt or 0

        # Deadlock guard
        if abs(vx) < 0.3 and coverage < 5.0:
            if current_alt > IBVS_ALTITUDE_FLOOR_M:
                vz = min(0.5, IBVS_SHALLOW_DIVE_VZ_MAX)
            else:
                vx = IBVS_VX_MIN
                vz = 0.0

        # Altitude floor
        if current_alt <= IBVS_ALTITUDE_FLOOR_M and vz > 0:
            vz = 0.0

        # Impact detection
        self._check_impact(coverage, current_alt, target_port)

        if loop_count % 10 == 0:
            impact_str = " [IMPACT]" if self.fsm.impact_detected else ""
            SwarmLogger.log("IBVS_CMD", f"DRONE_{target_port}",
                            f"vx={vx:.2f} vz={vz:.2f} yaw={yaw_rate:.2f} "
                            f"pitch={math.degrees(drone_pitch):.1f}° | "
                            f"alt={current_alt:.1f}m cov={coverage:.1f}%{impact_str}",
                            "ATTACK_FLOW")

        return vx, vy, vz, yaw_rate

    def _check_impact(self, coverage, altitude, target_port):
        """Impact detection with progressive altitude threshold."""
        if IMPACT_ALTITUDE_PROGRESSIVE and altitude <= IMPACT_ALTITUDE_LOW_M:
            threshold = IMPACT_COVERAGE_THRESHOLD * 0.5
        elif IMPACT_ALTITUDE_PROGRESSIVE and altitude <= IMPACT_ALTITUDE_HIGH_M:
            ratio = (altitude - IMPACT_ALTITUDE_LOW_M) / (IMPACT_ALTITUDE_HIGH_M - IMPACT_ALTITUDE_LOW_M)
            threshold = IMPACT_COVERAGE_THRESHOLD * (0.5 + 0.5 * ratio)
        else:
            threshold = IMPACT_COVERAGE_THRESHOLD

        if coverage >= threshold:
            self._impact_confirm_count += 1
            if self._impact_confirm_count >= IMPACT_CONFIRM_FRAMES:
                self.fsm.confirm_impact(coverage, altitude)
                self.drone_manager.mission_status_message = "MISSION COMPLETE - TARGET ENGAGED"
                if self.swarm_manager and self.fsm.current_target_id:
                    self.bridge.send_impact(
                        self.fsm.current_target_id, coverage, altitude)
                self.tracking_mode = False  # Stop tracking loop
        elif altitude <= IMPACT_ALTITUDE_MIN_M:
            self._impact_confirm_count += 1
            if self._impact_confirm_count >= IMPACT_CONFIRM_FRAMES:
                self.fsm.confirm_impact(coverage, altitude)
                self.drone_manager.mission_status_message = "MISSION COMPLETE - GROUND IMPACT"
                self.tracking_mode = False  # Stop tracking loop
        else:
            self._impact_confirm_count = 0  # Hard reset instead of decrement
