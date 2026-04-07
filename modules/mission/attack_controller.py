"""Attack flow, state, and terminal guidance."""

import math
import time
import cv2
from typing import Optional, Tuple

from modules.mission.flight_controller import FlightController
from modules.vision.detection_processor import DetectionProcessor, DetectionResult
from modules.mission.follower_link import FollowerLink, SwarmResponse
from modules.core.pid_controller import LowPassFilter, VelocitySmoother
from modules.core.logger import SwarmLogger
from modules.core.geo_math import GeoMath
from config import (
    CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FY, CAMERA_FX, CAMERA_PITCH_OFFSET,
    JPEG_QUALITY,
    ATTACK_CENTER_ACCEPT_RADIUS_PX,
    ATTACK_CENTER_REQUIRED_FRAMES,
    CENTERING_YAW_KP, CENTERING_YAW_RATE_MAX, CENTERING_YAW_DEADZONE,
    ATTACK_CENTER_BLIND_BOX_PX,
    FRAMING_TARGET_LOSS_GRACE_S, ATTACK_CENTER_LOSS_GRACE_S,
    ATTACK_TERMINAL_AREA_RADIUS_M,
    ATTACK_DIVE_TERMINAL_FORWARD_SPEED, ATTACK_DIVE_MAX_DESCENT_MPS,
    ATTACK_DIVE_MAX_CLIMB_MPS, ATTACK_DIVE_YAW_KP,
    ATTACK_DIVE_VERTICAL_KP, ATTACK_DIVE_STARTUP_SPEED_MPS,
    ATTACK_DIVE_STARTUP_RAMP_S, ATTACK_DIVE_PITCH_COMPENSATION_GAIN,
    ATTACK_DIVE_DESCENT_SOFTENING, IBVS_STARTUP_VX_MIN, IBVS_RAMP_UP_TIME_S,
    ATTACK_DIVE_STARTUP_YAW_SCALE, ATTACK_DIVE_STARTUP_VZ_SCALE,
    ATTACK_LOCK_MAX_JUMP_PX, ATTACK_LOCK_SIZE_RATIO_MIN,
    CENTERING_YAW_FILTER_ALPHA,
    IMPACT_COVERAGE_THRESHOLD,
    TERMINAL_EVENT_CONFIRM_FRAMES,
    TERMINAL_EVENT_LOCK_WINDOW_S,
    TERMINAL_EVENT_PROBABLE_COVERAGE,
    TERMINAL_EVENT_PROXIMITY_M,
    TERMINAL_EVENT_PROXIMITY_ALT_SCALE,
    MAX_VISUAL_REACQUIRE_DIST_PX,
    HUMAN_LOST_TIMEOUT,
    ATTACK_TIMEOUT_S,
    ATTACK_CENTER_TIMEOUT_S,
    ATTACK_LOCK_TIMEOUT_S,
)

_MISSION_AREA_RADIUS_M = ATTACK_TERMINAL_AREA_RADIUS_M
_LOCK_CONFIRM_FRAMES = 5
_LOCK_MAX_LOSS_FRAMES = 50
_VERIFY_MAX_ATTEMPTS = 3
_AUTONOMOUS_PHASES = frozenset({"AUTONOMOUS", "APPROACH"})
_ACTIVE_PHASES = frozenset({"LOCKING", "VERIFY", "AUTONOMOUS", "APPROACH"})
_ASSIGNED_MATCH_GEO_MAX_M = 60.0
_ASSIGNED_MATCH_VISUAL_MAX_PX = 220.0
_ASSIGNED_MATCH_SIZE_RATIO_MIN = 0.22
_TERMINAL_ROUTE_FALLBACK_DELAY_S = 3.0


class AttackFSM:
    """Attack phase state with logged transitions."""

    def __init__(self, port: int):
        self.port = port
        self.reset()

    def reset(self):
        self.phase: Optional[str] = None
        self.phase_start: Optional[float] = None
        self.attack_start: Optional[float] = None
        self.assigned_target_id: Optional[str] = None
        self.assigned_target_lat: Optional[float] = None
        self.assigned_target_lon: Optional[float] = None
        self.assigned_local_id: Optional[int] = None
        self.locked_track_id: Optional[int] = None
        self.impact_detected = False
        self.verify_attempts = 0

    @property
    def is_autonomous(self) -> bool:
        return self.phase in _AUTONOMOUS_PHASES

    @property
    def is_active(self) -> bool:
        return self.phase in _ACTIVE_PHASES

    def phase_elapsed(self) -> float:
        if self.phase_start is None:
            return 0.0
        return time.time() - self.phase_start

    def attack_elapsed(self) -> float:
        if self.attack_start is None:
            return 0.0
        return time.time() - self.attack_start

    def enter_locking(self, target_id: str, target_lat: float, target_lon: float,
                      local_id: Optional[int] = None):
        if self.phase is not None:
            self._log("GUARD", f"enter_locking blocked: already in {self.phase}")
            return
        self.assigned_target_id = target_id
        self.assigned_target_lat = target_lat
        self.assigned_target_lon = target_lon
        self.assigned_local_id = local_id
        self._transition("LOCKING", f"Target={target_id} pos=({target_lat:.6f},{target_lon:.6f})")

    def enter_verify(self, locked_bbox: Tuple[int, int, int, int],
                     locked_track_id: Optional[int] = None):
        if self.phase != "LOCKING":
            self._log("GUARD", f"enter_verify blocked: phase={self.phase}")
            return
        self.locked_track_id = locked_track_id
        self.verify_attempts += 1
        self._transition("VERIFY", f"bbox={locked_bbox} track_id={locked_track_id}")

    def enter_locking_retry(self, reason: str = "leader_reject"):
        if self.phase != "VERIFY":
            return
        self.locked_track_id = None
        self._transition("LOCKING", f"Retry: {reason}")

    def enter_autonomous(self):
        if self.phase != "VERIFY":
            self._log("GUARD", f"enter_autonomous blocked: phase={self.phase}")
            return
        self.attack_start = time.time()
        self._transition("AUTONOMOUS", "Leader disconnected, centering target")

    def enter_approach(self, route_lat: Optional[float], route_lon: Optional[float], route_alt: float):
        if self.phase != "AUTONOMOUS":
            self._log("GUARD", f"enter_approach blocked: phase={self.phase}")
            return
        if route_lat is not None and route_lon is not None:
            detail = f"Visual terminal armed @ ({route_lat:.6f},{route_lon:.6f}) alt={route_alt:.1f}m"
        else:
            detail = f"Visual terminal armed alt={route_alt:.1f}m"
        self._transition("APPROACH", detail)

    def enter_impact(self):
        if self.phase != "APPROACH":
            return
        self.impact_detected = True
        self._transition("IMPACT", "TARGET ELIMINATED")

    def enter_scan_resume(self, reason: str = ""):
        prev = self.phase
        self._transition("SCAN_RESUME", f"Resuming scan: {reason} (was {prev})")

    def abort(self, reason: str = ""):
        if self.phase is None:
            return
        self._log("ABORT", f"Attack aborted: {reason}")
        prev_phase = self.phase
        self.reset()
        SwarmLogger.log_structured(
            "fsm_transition", "ATTACK_FLOW",
            level="ABORT", drone_id=self.port,
            state_before=prev_phase, state_after="IDLE",
            decision_reason=reason,
        )

    def check_timeouts(self) -> Optional[str]:
        if self.phase is None:
            return None
        elapsed = self.phase_elapsed()
        if self.phase == "LOCKING" and elapsed > ATTACK_LOCK_TIMEOUT_S:
            return "locking_timeout"
        if self.phase == "VERIFY" and elapsed > ATTACK_CENTER_TIMEOUT_S:
            return "verify_timeout"
        if self.phase == "AUTONOMOUS" and elapsed > ATTACK_CENTER_TIMEOUT_S:
            return "autonomous_timeout"
        if self.phase == "APPROACH" and self.attack_elapsed() > ATTACK_TIMEOUT_S:
            return "attack_total_timeout"
        return None

    def _transition(self, new_phase: str, detail: str = ""):
        old_phase = self.phase
        self.phase = new_phase
        self.phase_start = time.time()
        SwarmLogger.log_phase_change(
            module=f"DRONE_{self.port}",
            entity="ATTACK_FSM",
            old_phase=old_phase or "IDLE",
            new_phase=new_phase,
            source="ATTACK_FLOW",
            reason=detail or None,
            level="PROTOCOL",
            event_type="fsm_transition",
            structured_source="ATTACK_FLOW",
            structured_level="PROTOCOL",
            drone_id=self.port,
            target_id=self.assigned_target_id,
        )

    def _log(self, level: str, msg: str):
        SwarmLogger.log(level, f"DRONE_{self.port}", msg, "ATTACK_FLOW")


class AttackController:
    """Unified scan + attack controller for drone mission."""

    def __init__(self, flight_ctrl, camera_handler, human_tracker,
                 swarm_manager, drone_manager, port=None):
        self.flight_ctrl: FlightController = flight_ctrl
        self.camera_handler = camera_handler
        self.human_tracker = human_tracker
        self.swarm_manager = swarm_manager
        self.drone_manager = drone_manager
        self.port = port

        self.detector = DetectionProcessor(port, camera_handler, human_tracker, drone_manager)
        self.fsm = AttackFSM(port)
        self.bridge = FollowerLink(swarm_manager, drone_manager, port)
        self.tracking_mode = False
        self.scanning_active = False
        self.collision_detected = False

        self._lock_confirm_count = 0
        self._lock_loss_count = 0
        self._locked_bbox_center: Optional[Tuple[float, float]] = None
        self._locked_bbox_size: Optional[Tuple[float, float]] = None
        self._locked_target_gps: Optional[Tuple[float, float]] = None
        self._locked_group_id: Optional[object] = None
        self._locked_group_count: int = 0

        self._centering_settle_count = 0
        self._prev_attack_center: Optional[Tuple[float, float]] = None
        self._prev_attack_center_ts: float = 0.0
        self._framing_center: Optional[Tuple[float, float]] = None
        self._framing_center_ts: float = 0.0
        self._framing_centered_count = 0
        self._framing_locked = False
        self._last_locked_bbox_ts: float = 0.0

        self._last_visual_cmd = (0.0, 0.0, 0.0, 0.0)
        self._last_visual_cmd_ts = 0.0
        self._latest_frame_shape: Optional[Tuple[int, int]] = None
        self._last_processed_frame_seq: Optional[int] = None
        self._observer_visual_anchor_ts: float = 0.0
        self._observer_visual_anchor_yaw_rate: float = 0.0

        self._terminal_event = None
        self._terminal_completion_pending = False
        self._planned_terminal_point = None
        self._last_reliable_visual_ts: float = 0.0
        self._terminal_guidance_mode: str = "visual"
        self._terminal_reacquire_streak: int = 0
        self._terminal_probable_confirm_count: int = 0
        self._terminal_max_coverage: float = 0.0
        self._route_fallback_started_ts: float = 0.0
        self._terminal_planned_point_frozen: bool = False
        self._attack_visual_token: Optional[str] = None
        self._attack_visual_streak: int = 0

        self._yaw_filter = LowPassFilter(alpha=CENTERING_YAW_FILTER_ALPHA)
        self._velocity_smoother = VelocitySmoother(max_accel=8.0, max_decel=12.0)

    def _log_attack_health(
        self,
        key_suffix: str,
        level: str,
        message: str,
        *,
        event_type: str,
        decision_reason: Optional[str] = None,
        interval_s: float = 3.0,
        target_id: Optional[str] = None,
        metadata: Optional[dict] = None,
    ) -> None:
        SwarmLogger.log_operational(
            key=f"attack:{self.port}:{key_suffix}",
            level=level,
            module=f"DRONE_{self.port}",
            message=message,
            source="ATTACK_FLOW",
            interval_s=interval_s,
            event_type=event_type,
            structured_source="ATTACK_FLOW",
            structured_level=level,
            drone_id=self.port,
            target_id=target_id or self.fsm.assigned_target_id,
            state_before=self.fsm.phase,
            decision_reason=decision_reason or message,
            metadata=metadata or {},
        )

    def _log_attack_protocol(
        self,
        action: str,
        detail: str,
        *,
        event_type: Optional[str] = None,
        interval_s: float = 0.0,
        **event_fields,
    ) -> bool:
        return SwarmLogger.log_operational(
            key=f"attack:protocol:{self.port}:{action}:{detail}",
            level="PROTOCOL",
            module=f"DRONE_{self.port}",
            message=f"{action} | {detail}",
            source="ATTACK_FLOW",
            interval_s=interval_s,
            event_type=event_type,
            structured_source="ATTACK_FLOW",
            structured_level="PROTOCOL",
            drone_id=self.port,
            target_id=self.fsm.assigned_target_id,
            state_before=self.fsm.phase,
            metadata=event_fields,
        )

    @property
    def attack_approved(self):
        return self.fsm.is_autonomous

    @attack_approved.setter
    def attack_approved(self, value):
        _ = value

    @property
    def current_target_id(self):
        return self.fsm.assigned_target_id

    @current_target_id.setter
    def current_target_id(self, value):
        self.fsm.assigned_target_id = value

    @property
    def assigned_local_tracker_id(self):
        return self.fsm.assigned_local_id

    @assigned_local_tracker_id.setter
    def assigned_local_tracker_id(self, value):
        self.fsm.assigned_local_id = value

    @property
    def lock_status(self):
        phase = self.fsm.phase
        if phase in ("LOCKING", "VERIFY"):
            return "SESSION_OPEN"
        if phase in ("AUTONOMOUS", "APPROACH"):
            return "EXECUTING"
        if phase == "IMPACT":
            return "COMPLETED"
        return None

    def has_terminal_completion_pending(self) -> bool:
        return bool(self._terminal_completion_pending)

    def get_planned_terminal_point(self):
        return self._planned_terminal_point

    def get_terminal_event(self):
        return self._terminal_event

    def reset_engagement_state(self):
        """Clear all engagement state."""
        self.fsm.reset()
        try:
            self.human_tracker.set_primary_target(None)
            self.human_tracker.set_primary_group(None)
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
        self.collision_detected = False
        self.tracking_mode = False
        self._lock_confirm_count = 0
        self._lock_loss_count = 0
        self._locked_bbox_center = None
        self._locked_bbox_size = None
        self._locked_target_gps = None
        self._centering_settle_count = 0
        self._prev_attack_center = None
        self._prev_attack_center_ts = 0.0
        self._framing_center = None
        self._framing_center_ts = 0.0
        self._framing_centered_count = 0
        self._framing_locked = False
        self._last_locked_bbox_ts = 0.0
        self._reset_attack_state()
        self._latest_frame_shape = None
        self._last_processed_frame_seq = None
        self._terminal_event = None
        self._terminal_completion_pending = False
        self._planned_terminal_point = None
        self._last_reliable_visual_ts = 0.0
        self._terminal_guidance_mode = "visual"
        self._terminal_reacquire_streak = 0
        self._terminal_probable_confirm_count = 0
        self._terminal_max_coverage = 0.0
        self._route_fallback_started_ts = 0.0

    def execute_tracking_mode(self, vehicle, is_mission_active_check=None,
                              pause_check=None, pause_waiter=None):
        """Run the attack loop until completion or disengage."""
        if is_mission_active_check is None:
            is_mission_active_check = lambda: True

        self.tracking_mode = True
        try:
            while is_mission_active_check():
                if pause_check and pause_check():
                    if pause_waiter:
                        pause_waiter()
                    continue

                self.check_human_detection()
                if self._terminal_completion_pending:
                    break
                if self.fsm.phase == "SCAN_RESUME":
                    break
                if not self.tracking_mode and not self.fsm.is_active:
                    break

                time.sleep(0.03)

        except Exception as e:
            SwarmLogger.log("ERROR", f"DRONE_{self.port}",
                            f"Tracking loop error: {e}", "TRACKING")

        self.tracking_mode = False
        return self._terminal_event

    def check_human_detection(self) -> bool:
        """Process camera frame. In scan mode: report targets. In attack mode: drive FSM."""
        if not self.scanning_active and not self.fsm.is_active and not self.tracking_mode:
            return False

        try:
            vehicle = self.drone_manager.drones.get(self.port)
            if vehicle is None or vehicle == "connecting":
                return False
            loc = vehicle.location.global_relative_frame
            if not loc or loc.lat == 0:
                return False
            d_lat, d_lon, d_alt = loc.lat, loc.lon, loc.alt
            heading = vehicle.heading

            tracker_result, detections = self.detector.process_scan_frame(
                attack_approved=self.fsm.is_autonomous,
                locked_id=self.fsm.assigned_local_id,
                attack_mode=self.fsm.is_active)
            self._update_frame_shape(tracker_result)

            self._stream_annotated_frame(tracker_result)
            frame_seq = tracker_result.get("frame_seq") if isinstance(tracker_result, dict) else None
            if frame_seq is not None:
                frame_seq = int(frame_seq)
                if self._last_processed_frame_seq == frame_seq:
                    return False
                self._last_processed_frame_seq = frame_seq
            if self.fsm.is_active:
                return self._process_attack_frame(detections, vehicle, d_lat, d_lon, d_alt)

            if self.tracking_mode:
                if detections:
                    self._apply_framing_yaw(vehicle, tracker_result)
                    self._process_scan_detections(detections, vehicle, d_lat, d_lon, d_alt, heading)
                    return True
                self._apply_framing_loss_hold(vehicle)
                return True

            if not detections:
                return False
            return self._process_scan_detections(detections, vehicle, d_lat, d_lon, d_alt, heading)

        except Exception as e:
            now = time.time()
            if now - getattr(self, '_last_det_error_log', 0) > 10.0:
                self._last_det_error_log = now
                SwarmLogger.log("ERROR", f"DRONE_{self.port}",
                                f"Detection error: {e}", "TRACKING")
            return False

    def _process_scan_detections(self, detections, vehicle, d_lat, d_lon, d_alt, heading
                                 ) -> bool:
        """Report the full current detection set to the leader in one telemetry packet."""
        payload = []
        valid_detections = []
        for det in detections:
            if det.lat is None:
                continue
            payload.append(det.to_swarm_dict(
                (d_lat, d_lon, d_alt), heading,
                (vehicle.attitude.roll, vehicle.attitude.pitch, vehicle.attitude.yaw)))
            valid_detections.append(det)

        if not valid_detections:
            return False

        resp = self.bridge.publish_telemetry(payload, state_label="SCAN")
        matched = self._match_assignment_detection(resp, valid_detections)
        if matched is not None:
            self._handle_assignment(resp, matched)
        return True

    def _match_assignment_detection(self, resp: SwarmResponse, detections) -> Optional[DetectionResult]:
        """Resolve a leader assignment back to the drone-local detection that produced it."""
        local_id = getattr(resp, "local_id", None)
        if local_id is None:
            local_id = getattr(resp, "tracker_id", None)
        if local_id is not None:
            for det in detections or []:
                if getattr(det, "track_id", None) == local_id:
                    return det

        target_lat = getattr(resp, "lat", None)
        target_lon = getattr(resp, "lon", None)
        if target_lat is None:
            target_lat = getattr(resp, "target_lat", None)
        if target_lon is None:
            target_lon = getattr(resp, "target_lon", None)
        if target_lat is None or target_lon is None:
            return None

        best = None
        best_dist = float("inf")
        for det in detections or []:
            if getattr(det, "lat", None) is None or getattr(det, "lon", None) is None:
                continue
            dist = GeoMath.haversine_distance(float(det.lat), float(det.lon), float(target_lat), float(target_lon))
            if dist < best_dist:
                best_dist = dist
                best = det
        return best

    def _handle_assignment(self, resp: SwarmResponse, det: DetectionResult) -> bool:
        """Handle leader assignment response during scan mode."""
        if resp.action in ("EXECUTE", "ASSIGN"):
            accept_resp = self.bridge.report_target(
                det.lat, det.lon, det.conf,
                tracker_id=det.track_id,
                raw_data={"action": "ASSIGNMENT_ACCEPT",
                          "target_id": resp.target_id,
                          "local_tracker_id": det.track_id})
            if accept_resp.action == "ASSIGNMENT_ACCEPTED":
                resp = accept_resp

        if resp.action == "ASSIGNMENT_ACCEPTED":
            if self.fsm.is_active:
                same_target = str(getattr(resp, "target_id", "") or "") == str(self.fsm.assigned_target_id or "")
                same_local = getattr(resp, "local_id", None) == getattr(self.fsm, "assigned_local_id", None)
                if same_target and same_local:
                    return True
                return False

            target_lat = getattr(resp, 'target_lat', None) or (det.lat if det.lat else 0.0)
            target_lon = getattr(resp, 'target_lon', None) or (det.lon if det.lon else 0.0)

            self.fsm.enter_locking(
                target_id=resp.target_id,
                target_lat=target_lat,
                target_lon=target_lon,
                local_id=resp.local_id)

            self.scanning_active = False
            self.tracking_mode = True
            self._reset_framing_state()
            self._reset_attack_state(clear_visual_lock=True)

            try:
                self.human_tracker.set_primary_target(None)
                self.human_tracker.set_primary_group(None)
            except Exception:
                pass

            self._log_attack_protocol(
                "ASSIGNMENT_ACCEPTED",
                f"target={resp.target_id} -> LOCKING",
                event_type="attack_assignment_accepted",
                local_track_id=resp.local_id,
            )
            return True

        return False

    def _process_attack_frame(self, detections, vehicle, d_lat, d_lon, d_alt) -> bool:
        """Drive attack FSM based on current detections."""
        phase = self.fsm.phase
        timeout_reason = self.fsm.check_timeouts()
        if timeout_reason:
            if timeout_reason == "autonomous_timeout" and phase == "AUTONOMOUS":
                route_lat, route_lon = self._terminal_route()
                self.fsm.enter_approach(route_lat, route_lon, d_alt)
                if route_lat is not None and route_lon is not None:
                    self._update_planned_terminal_point(
                        float(route_lat),
                        float(route_lon),
                        d_alt,
                        d_lat,
                        d_lon,
                        "autonomous_timeout_fallback",
                    )
                self._log_attack_health(
                    "autonomous_timeout_fallback",
                    "WARNING",
                    "Autonomous centering timeout, committing planned terminal route",
                    event_type="attack_timeout_continue",
                    decision_reason=timeout_reason,
                    interval_s=0.0,
                    metadata={"phase": phase},
                )
                return True
            if timeout_reason == "attack_total_timeout" and phase == "APPROACH":
                self._log_attack_health(
                    "timeout_continue",
                    "WARNING",
                    f"Timeout ignored, committed terminal route active | reason={timeout_reason}",
                    event_type="attack_timeout_continue",
                    decision_reason=timeout_reason,
                    interval_s=4.0,
                    metadata={"phase": phase},
                )
            else:
                self._handle_attack_failure(timeout_reason)
                return True

        if phase == "LOCKING":
            return self._phase_locking(detections)
        elif phase == "VERIFY":
            return self._phase_verify(detections, d_lat, d_lon)
        elif phase == "AUTONOMOUS":
            return self._phase_autonomous(detections, vehicle, d_lat, d_lon, d_alt)
        elif phase == "APPROACH":
            return self._phase_terminal_run(detections, vehicle, d_lat, d_lon, d_alt)

        return False

    def _phase_locking(self, detections) -> bool:
        """Find leader-assigned target using ID + position, lock via bbox."""
        if not detections:
            self._lock_loss_count += 1
            self._log_attack_health(
                "locking_loss",
                "WARNING",
                f"Locking lost visual detections | loss_count={self._lock_loss_count}",
                event_type="attack_lock_health",
                decision_reason="locking_no_detections_partial",
                interval_s=1.5,
                metadata={"phase": "LOCKING", "loss_count": self._lock_loss_count},
            )
            if self._lock_loss_count > _LOCK_MAX_LOSS_FRAMES * 2:
                self._handle_attack_failure("locking_no_detections")
            return False

        best_det = self._find_assigned_target(detections)
        if best_det is None:
            best_det = self._find_best_bbox_match(detections)
        if best_det is None:
            self._lock_loss_count += 1
            self._log_attack_health(
                "locking_mismatch",
                "WARNING",
                f"Assigned target not centered yet | loss_count={self._lock_loss_count}",
                event_type="attack_lock_health",
                decision_reason="locking_target_not_matched",
                interval_s=1.5,
                metadata={"phase": "LOCKING", "loss_count": self._lock_loss_count},
            )
            return False

        self._locked_bbox_center = best_det.bbox_center
        self._locked_bbox_size = best_det.bbox_size
        self._last_locked_bbox_ts = time.time()
        if getattr(best_det, "group_id", None) is not None:
            self._locked_group_id = best_det.group_id
            self._locked_group_count = int(getattr(best_det, "group_member_count", 0) or 0)
            try:
                self.human_tracker.set_primary_group(best_det.group_id)
            except Exception:
                pass
        if best_det.lat is not None and best_det.lon is not None:
            self._locked_target_gps = (float(best_det.lat), float(best_det.lon))
        self._lock_loss_count = 0
        self._lock_confirm_count += 1

        if self._locked_bbox_center:
            self._yaw_toward_bbox(best_det, self._locked_bbox_center)

        if self._lock_confirm_count >= _LOCK_CONFIRM_FRAMES:
            bbox_xyxy = best_det.bbox_xyxy if hasattr(best_det, 'bbox_xyxy') else (0, 0, 0, 0)
            self.fsm.enter_verify(
                locked_bbox=bbox_xyxy,
                locked_track_id=best_det.track_id)
            self._log_attack_protocol(
                "VISUAL_LOCK",
                f"group_id={getattr(best_det, 'group_id', None)} bbox_center={best_det.bbox_center}",
                event_type="attack_visual_lock",
                local_track_id=best_det.track_id,
                group_id=getattr(best_det, "group_id", None),
                bbox=best_det.bbox_center,
            )
        return True

    def _find_assigned_target(self, detections) -> Optional[DetectionResult]:
        """Find the best detection for the assigned target."""
        assigned_id = self.fsm.assigned_local_id
        assigned_lat = self.fsm.assigned_target_lat
        assigned_lon = self.fsm.assigned_target_lon

        best_det = None
        best_score = float("-inf")

        for det in self._collapse_group_candidates(
            detections,
            lambda candidate: self._score_assigned_target_candidate(
                candidate, assigned_id, assigned_lat, assigned_lon
            ),
        ):
            score = self._score_assigned_target_candidate(det, assigned_id, assigned_lat, assigned_lon)

            if score > best_score:
                best_score = score
                best_det = det

        if best_score < 0.0:
            return None
        return best_det

    def _score_assigned_target_candidate(
        self,
        det: DetectionResult,
        assigned_id: Optional[int],
        assigned_lat: Optional[float],
        assigned_lon: Optional[float],
    ) -> float:
        """Score a detection against the assigned target."""
        score = 0.0

        if det.bbox_center is None or det.bbox_size is None:
            return float("-inf")

        center_limit = min(float(MAX_VISUAL_REACQUIRE_DIST_PX), _ASSIGNED_MATCH_VISUAL_MAX_PX)
        if self._locked_bbox_center is not None:
            pred_cx, pred_cy = self._predicted_locked_center()
            center_dist = math.hypot(float(det.bbox_center[0]) - pred_cx, float(det.bbox_center[1]) - pred_cy)
            dynamic_center_limit = float(center_limit)
            det_area = max(1.0, float(det.bbox_size[0]) * float(det.bbox_size[1]))
            if det_area < 12000.0:
                dynamic_center_limit = max(dynamic_center_limit, float(MAX_VISUAL_REACQUIRE_DIST_PX) * 0.92)
            if center_dist > dynamic_center_limit:
                return float("-inf")
            score += max(0.0, 40.0 - center_dist * 0.18)

            last_area = self._locked_bbox_area()
            if last_area is not None:
                size_ratio = min(det_area, last_area) / max(det_area, last_area)
                size_ratio_limit = float(_ASSIGNED_MATCH_SIZE_RATIO_MIN)
                if min(det_area, last_area) < 12000.0:
                    size_ratio_limit = max(0.12, float(_ASSIGNED_MATCH_SIZE_RATIO_MIN) * 0.55)
                if size_ratio < size_ratio_limit:
                    return float("-inf")
                score += size_ratio * 8.0

        if assigned_lat is not None and assigned_lon is not None and det.lat is not None and det.lon is not None:
            geo_dist = GeoMath.haversine_distance(float(assigned_lat), float(assigned_lon), float(det.lat), float(det.lon))
            if geo_dist > _ASSIGNED_MATCH_GEO_MAX_M:
                return float("-inf")
            score += max(0.0, 90.0 - geo_dist * 1.4)
        elif assigned_lat is not None and assigned_lon is not None:
            score -= 15.0

        if self._locked_target_gps and det.lat is not None and det.lon is not None:
            lock_geo_dist = GeoMath.haversine_distance(
                float(self._locked_target_gps[0]),
                float(self._locked_target_gps[1]),
                float(det.lat),
                float(det.lon),
            )
            if lock_geo_dist > 20.0:
                score -= 30.0
            else:
                score += max(0.0, 20.0 - lock_geo_dist)

        if assigned_id is not None and det.track_id is not None and int(det.track_id) == int(assigned_id):
            score += 20.0

        locked_group_id = getattr(self, "_locked_group_id", None)
        if (
            locked_group_id is not None
            and getattr(det, "group_id", None) is not None
            and str(det.group_id) == str(locked_group_id)
        ):
            score += 120.0
        elif locked_group_id is not None and getattr(det, "group_id", None) is not None:
            try:
                if str(det.group_id).split(":")[-1] == str(locked_group_id).split(":")[-1]:
                    score += 55.0
            except Exception:
                pass
        elif locked_group_id is not None and getattr(det, "group_member_count", 0):
            locked_count = int(getattr(self, "_locked_group_count", 0) or 0)
            det_count = int(getattr(det, "group_member_count", 0) or 0)
            if locked_count > 0 and abs(locked_count - det_count) <= 1:
                score += 18.0

        if getattr(det, "is_group", False):
            score += 12.0
            score += min(24.0, float(getattr(det, "group_member_count", 1) or 1) * 3.0)

        if self._locked_bbox_center is not None and det.bbox_center is not None:
            direct_dist = math.hypot(
                float(det.bbox_center[0]) - float(self._locked_bbox_center[0]),
                float(det.bbox_center[1]) - float(self._locked_bbox_center[1]),
            )
            score += max(0.0, 22.0 - direct_dist * 0.08)

        score += max(0.0, float(det.conf or 0.0)) * 6.0
        return score

    def _phase_verify(self, detections, d_lat, d_lon) -> bool:
        """Send locked target info to leader for verification."""
        now = time.time()
        best_det = None
        if detections:
            best_det = self._find_assigned_target(detections)
            if best_det is None:
                best_det = self._find_best_bbox_match(detections)
            if best_det:
                self._locked_bbox_center = best_det.bbox_center
                self._locked_bbox_size = best_det.bbox_size
                self._last_locked_bbox_ts = now
                if getattr(best_det, "group_id", None) is not None:
                    self._locked_group_id = best_det.group_id
                    self._locked_group_count = int(getattr(best_det, "group_member_count", 0) or 0)
                    try:
                        self.human_tracker.set_primary_group(best_det.group_id)
                    except Exception:
                        pass
                if best_det.lat is not None and best_det.lon is not None:
                    self._locked_target_gps = (float(best_det.lat), float(best_det.lon))

        locked_lat = None
        locked_lon = None
        if best_det and best_det.lat is not None and best_det.lon is not None:
            locked_lat = float(best_det.lat)
            locked_lon = float(best_det.lon)
        elif (
            self._locked_target_gps is not None
            and self._last_locked_bbox_ts > 0.0
            and (now - float(self._last_locked_bbox_ts)) <= float(ATTACK_CENTER_LOSS_GRACE_S)
        ):
            locked_lat, locked_lon = self._locked_target_gps

        if locked_lat is None or locked_lon is None:
            self._log_attack_health(
                "verify_wait_visual",
                "WARNING",
                "Verify waiting for fresh visual lock",
                event_type="attack_verify_health",
                decision_reason="verify_wait_visual",
                interval_s=1.5,
                metadata={"phase": "VERIFY"},
            )
            return True

        resp = self.bridge.report_target(
            locked_lat, locked_lon, 1.0,
            tracker_id=(best_det.track_id if best_det is not None else self.fsm.locked_track_id),
            raw_data={
                "action": "LOCK_REPORT",
                "target_id": self.fsm.assigned_target_id,
                "locked_lat": locked_lat,
                "locked_lon": locked_lon,
                "drone_lat": d_lat, "drone_lon": d_lon,
                "group_id": getattr(best_det, "group_id", getattr(self, "_locked_group_id", None)),
                "member_ids": list(getattr(best_det, "member_ids", ()) or ()),
            },
            covariance=None)

        if resp.action == "VERIFY_APPROVED":
            self.fsm.enter_autonomous()
            self._centering_settle_count = 0
            self._reset_attack_state()
            SwarmLogger.log("SUCCESS", f"DRONE_{self.port}",
                            "Leader APPROVED — entering autonomous mode", "ATTACK_FLOW")
            return True

        elif resp.action == "VERIFY_REJECTED":
            self._log_attack_health(
                "verify_rejected",
                "WARNING",
                f"Verify rejected by leader | attempts={self.fsm.verify_attempts} lock=({locked_lat:.6f},{locked_lon:.6f})",
                event_type="attack_verify_rejected",
                decision_reason=str(getattr(resp, 'extra', {}).get('reason', 'leader_rejected')),
                interval_s=2.0,
                metadata={"phase": "VERIFY", "attempts": self.fsm.verify_attempts},
            )
            if self.fsm.verify_attempts < _VERIFY_MAX_ATTEMPTS:
                self._reset_attack_state(clear_visual_lock=True)
                self.fsm.enter_locking_retry("leader_rejected")
                self._lock_confirm_count = 0
                self._lock_loss_count = 0
            else:
                self._handle_attack_failure("verify_max_attempts")
            return True

        return True

    def _phase_autonomous(self, detections, vehicle, d_lat, d_lon, d_alt) -> bool:
        """Center target on screen, then arm committed visual attack."""
        best_det = self._find_best_bbox_match(detections) if detections else None
        now = time.time()

        if best_det and best_det.bbox_center:
            if not self._stabilize_attack_visual(best_det, phase="AUTONOMOUS"):
                self._lock_loss_count += 1
                return True
            self._locked_bbox_center = best_det.bbox_center
            self._locked_bbox_size = best_det.bbox_size
            self._last_locked_bbox_ts = now
            if getattr(best_det, "group_id", None) is not None:
                self._locked_group_id = best_det.group_id
                self._locked_group_count = int(getattr(best_det, "group_member_count", 0) or 0)
                try:
                    self.human_tracker.set_primary_group(best_det.group_id)
                except Exception:
                    pass
            if best_det.lat is not None and best_det.lon is not None:
                self._locked_target_gps = (float(best_det.lat), float(best_det.lon))
                self._update_planned_terminal_point(
                    float(best_det.lat),
                    float(best_det.lon),
                    d_alt,
                    d_lat,
                    d_lon,
                    "centering_visual",
                )
            self._lock_loss_count = 0

            filtered_error_x, _ = self._attack_steering_components(self._locked_bbox_center)
            
            img_cx, img_cy = self._frame_aimpoint()
            cx, cy = self._locked_bbox_center
            dx = abs(float(cx) - img_cx)
            blind_spot_radius = max(ATTACK_CENTER_ACCEPT_RADIUS_PX * 1.5, 40.0)
            centered = dx <= blind_spot_radius

            yaw_rate = self._compute_yaw_rate(
                filtered_error_x,
                gain=CENTERING_YAW_KP,
                limit=CENTERING_YAW_RATE_MAX,
            )
            vz = 0.0

            if centered:
                self._centering_settle_count += 1
            else:
                self._centering_settle_count = max(0, self._centering_settle_count - 1)

            try:
                self.flight_ctrl.command_body_velocity(
                    vehicle,
                    "ATTACK_CENTERING",
                    0.0,
                    0.0,
                    float(vz),
                    float(yaw_rate),
                    attack_approved=self.attack_approved,
                    reason="autonomous centering hover",
                )
            except Exception:
                pass

            if self._centering_settle_count >= ATTACK_CENTER_REQUIRED_FRAMES:
                terminal_lat = best_det.lat if best_det.lat is not None else self.fsm.assigned_target_lat
                terminal_lon = best_det.lon if best_det.lon is not None else self.fsm.assigned_target_lon
                self.fsm.enter_approach(terminal_lat, terminal_lon, d_alt)
                if terminal_lat is not None and terminal_lon is not None:
                    self._update_planned_terminal_point(float(terminal_lat), float(terminal_lon), d_alt, d_lat, d_lon, "attack_commit")
                    self._terminal_planned_point_frozen = True
                self._log_attack_protocol(
                    "VISUAL_ATTACK_COMMITTED",
                    f"center={self._locked_bbox_center} terminal={terminal_lat},{terminal_lon}",
                    event_type="attack_route_committed",
                    world_estimate={"lat": terminal_lat, "lon": terminal_lon, "alt": d_alt},
                )

            if best_det.lat and best_det.lon:
                self.bridge.publish_autonomous_update(
                    self.fsm.assigned_target_id,
                    (d_lat, d_lon, d_alt),
                    target_gps=(best_det.lat, best_det.lon))
        else:
            self._lock_loss_count += 1
            recent_lock = (
                self._locked_bbox_center is not None
                and self._prev_attack_center_ts > 0.0
                and (now - self._prev_attack_center_ts) <= float(ATTACK_CENTER_LOSS_GRACE_S)
            )
            if recent_lock:
                self._centering_settle_count = max(0, self._centering_settle_count - 1)
                try:
                    self.flight_ctrl.command_body_velocity(
                        vehicle,
                        "ATTACK_CENTERING",
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        attack_approved=self.attack_approved,
                        reason="autonomous centering grace hold",
                    )
                except Exception:
                    pass
                return True
            self._log_attack_health(
                "autonomous_visual_loss",
                "WARNING",
                f"Autonomous centering waiting for visual reacquire | loss_count={self._lock_loss_count}",
                event_type="attack_autonomous_health",
                decision_reason="autonomous_visual_loss",
                interval_s=2.0,
                metadata={"phase": "AUTONOMOUS", "loss_count": self._lock_loss_count},
            )
            if self._lock_loss_count > _LOCK_MAX_LOSS_FRAMES:
                if self.fsm.assigned_target_lat and self.fsm.assigned_target_lon:
                    self.fsm.enter_approach(self.fsm.assigned_target_lat, self.fsm.assigned_target_lon, d_alt)
                    self._update_planned_terminal_point(
                        self.fsm.assigned_target_lat,
                        self.fsm.assigned_target_lon,
                        d_alt,
                        d_lat,
                        d_lon,
                        "assigned_fallback",
                    )
                else:
                    self._handle_attack_failure("autonomous_target_lost")

        return True

    def _phase_terminal_run(self, detections, vehicle, d_lat, d_lon, d_alt) -> bool:
        """Single-profile terminal run: always forward, steer with nose, descend continuously."""
        best_det = self._find_best_bbox_match(detections) if detections else None
        coverage = 0.0
        visual_available = False
        if not hasattr(self, "_terminal_guidance_mode"):
            self._terminal_guidance_mode = "visual"
        if not hasattr(self, "_terminal_max_coverage"):
            self._terminal_max_coverage = 0.0
        if not hasattr(self, "_attack_visual_streak"):
            self._attack_visual_streak = 0

        if best_det and best_det.bbox_center and best_det.bbox_size:
            coverage = self._bbox_coverage(best_det.bbox_size)
            self._terminal_max_coverage = max(float(self._terminal_max_coverage), float(coverage))
            visual_available = self._accept_terminal_visual(best_det)
            if visual_available:
                self._stabilize_attack_visual(best_det, phase="APPROACH")
                self._locked_bbox_center = best_det.bbox_center
                self._locked_bbox_size = best_det.bbox_size
                self._last_locked_bbox_ts = time.time()
                self._last_reliable_visual_ts = self._last_locked_bbox_ts
                if getattr(best_det, "group_id", None) is not None:
                    self._locked_group_id = best_det.group_id
                    self._locked_group_count = int(getattr(best_det, "group_member_count", 0) or 0)
                    try:
                        self.human_tracker.set_primary_group(best_det.group_id)
                    except Exception:
                        pass
                if best_det.lat is not None and best_det.lon is not None:
                    filtered_target = self._filter_terminal_visual_target(
                        float(best_det.lat),
                        float(best_det.lon),
                    )
                    if filtered_target is not None:
                        filtered_lat, filtered_lon = filtered_target
                        self._locked_target_gps = (filtered_lat, filtered_lon)
                        self._update_planned_terminal_point(
                            filtered_lat,
                            filtered_lon,
                            d_alt,
                            d_lat,
                            d_lon,
                            "approach_visual",
                        )
                self._lock_loss_count = 0
                self._terminal_guidance_mode = "visual"
                self._route_fallback_started_ts = 0.0
                self._log_attack_health(
                    "bbox_only_guidance",
                    "INFO",
                    "Terminal attack using bbox-first guidance",
                    event_type="attack_bbox_only_guidance",
                    decision_reason="bbox_guidance_active",
                    interval_s=2.0,
                    metadata={"phase": self.fsm.phase, "coverage": float(coverage)},
                )
        else:
            self._attack_visual_streak = 0
            self._lock_loss_count += 1
            self._log_attack_health(
                "terminal_visual_coast",
                "WARNING",
                f"Terminal run visual lost, coasting with last attack vector | loss_count={self._lock_loss_count}",
                event_type="attack_ballistic_fallback",
                decision_reason="terminal_run_visual_loss",
                interval_s=2.0,
                metadata={"phase": self.fsm.phase, "loss_count": self._lock_loss_count},
            )

        if visual_available:
            self._fly_visual_attack(vehicle)
        else:
            last_locked_bbox_ts = float(getattr(self, "_last_reliable_visual_ts", 0.0) or 0.0)
            recent_visual = (
                last_locked_bbox_ts > 0.0
                and (time.time() - last_locked_bbox_ts) <= float(_TERMINAL_ROUTE_FALLBACK_DELAY_S)
            )
            if recent_visual and self._terminal_guidance_mode != "route":
                self._fly_visual_attack(vehicle)
            else:
                if self._terminal_guidance_mode != "route":
                    self._terminal_guidance_mode = "route"
                    self._terminal_reacquire_streak = 0
                    self._route_fallback_started_ts = time.time()
                    self._log_attack_health(
                        "terminal_route_fallback_enter",
                        "WARNING",
                        "Terminal route fallback engaged after visual loss",
                        event_type="attack_route_fallback_enter",
                        decision_reason="terminal_visual_fallback_active",
                        interval_s=0.0,
                        metadata={"phase": self.fsm.phase, "loss_count": self._lock_loss_count},
                    )
                self._fly_route_terminal_attack(vehicle, d_lat, d_lon)

        if coverage >= float(IMPACT_COVERAGE_THRESHOLD):
            self._confirm_impact(coverage, d_alt, "visual_coverage_reached")
            return True

        if self._is_terminal_area_reached(d_lat, d_lon):
            self._confirm_impact(coverage, d_alt, "terminal_airpoint_reached")
            return True

        probable_success = self._probable_terminal_success(d_lat, d_lon, d_alt, coverage)
        if probable_success:
            self._terminal_probable_confirm_count += 1
        else:
            self._terminal_probable_confirm_count = 0

        if (
            self._terminal_guidance_mode == "route"
            and self._terminal_probable_confirm_count >= int(TERMINAL_EVENT_CONFIRM_FRAMES)
        ):
            self._confirm_impact(max(coverage, float(self._terminal_max_coverage)), d_alt, "probable_impact_on_visual_loss")
            return True

        if self._lock_loss_count > _LOCK_MAX_LOSS_FRAMES:
            route_lat, route_lon = self._terminal_route()
            dist = GeoMath.haversine_distance(d_lat, d_lon, route_lat, route_lon) if route_lat else 999.0
            if dist <= 6.0 or probable_success:
                self._confirm_impact(max(coverage, float(self._terminal_max_coverage)), d_alt, "probable_impact_on_visual_loss")
                return True
            else:
                self._handle_attack_failure("terminal_target_lost")
                return False

        self._publish_terminal_update(d_lat, d_lon, d_alt)
        return True

    def _filter_terminal_visual_target(self, lat: float, lon: float) -> Optional[Tuple[float, float]]:
        """Bound visual terminal world updates without freezing normal motion."""
        planned = getattr(self, "_planned_terminal_point", None) or {}
        references = []

        planned_lat = planned.get("lat")
        planned_lon = planned.get("lon")
        if planned_lat is not None and planned_lon is not None:
            references.append(("planned", float(planned_lat), float(planned_lon)))

        locked = getattr(self, "_locked_target_gps", None)
        if locked is not None:
            references.append(("locked", float(locked[0]), float(locked[1])))

        assigned_lat = getattr(self.fsm, "assigned_target_lat", None)
        assigned_lon = getattr(self.fsm, "assigned_target_lon", None)
        if assigned_lat is not None and assigned_lon is not None:
            references.append(("assigned", float(assigned_lat), float(assigned_lon)))

        if not references:
            return float(lat), float(lon)

        ref_name, ref_lat, ref_lon = min(
            references,
            key=lambda item: GeoMath.haversine_distance(float(lat), float(lon), item[1], item[2]),
        )
        ref_dist = GeoMath.haversine_distance(float(lat), float(lon), ref_lat, ref_lon)
        if ref_dist <= 6.0:
            return float(lat), float(lon)

        in_terminal_visual = (
            getattr(self.fsm, "phase", None) == "APPROACH"
            and getattr(self, "_terminal_guidance_mode", "visual") == "visual"
        )
        if in_terminal_visual and ref_dist > 80.0:
            self._log_attack_health(
                "terminal_world_update_ignored",
                "WARNING",
                f"Ignored terminal world update while visual attack active | dist={ref_dist:.1f}m ref={ref_name}",
                event_type="attack_world_update_ignored",
                decision_reason="terminal_visual_world_quarantine",
                interval_s=0.5,
                metadata={"jump_m": ref_dist, "ref": ref_name},
            )
            return None

        now = time.time()
        ref_ts = float(planned.get("ts", 0.0) or 0.0)
        if ref_name == "locked":
            ref_ts = float(getattr(self, "_last_locked_bbox_ts", 0.0) or 0.0)
        dt = 0.35 if ref_ts <= 0.0 else max(0.05, min(1.0, now - ref_ts))

        soft_cap_m = min(18.0, max(7.0, 6.0 + dt * 12.0))
        hard_cap_m = max(140.0, soft_cap_m * 8.0)

        if ref_dist > hard_cap_m:
            self._log_attack_health(
                "terminal_visual_geo_outlier",
                "WARNING",
                f"Rejected visual terminal outlier | dist={ref_dist:.1f}m ref={ref_name}",
                event_type="attack_route_guard",
                decision_reason="terminal_visual_geo_outlier",
                interval_s=0.5,
                metadata={"jump_m": ref_dist, "ref": ref_name},
            )
            return None

        if ref_dist <= soft_cap_m:
            return float(lat), float(lon)

        bearing = GeoMath.bearing(ref_lat, ref_lon, float(lat), float(lon))
        step_north = soft_cap_m * math.cos(math.radians(bearing))
        step_east = soft_cap_m * math.sin(math.radians(bearing))
        bounded_lat, bounded_lon = GeoMath.ned_to_gps(ref_lat, ref_lon, step_north, step_east)
        self._log_attack_health(
            "terminal_visual_geo_clamped",
            "WARNING",
            f"Clamped visual terminal update | raw={ref_dist:.1f}m cap={soft_cap_m:.1f}m ref={ref_name}",
            event_type="attack_route_guard",
            decision_reason="terminal_visual_geo_clamped",
            interval_s=0.5,
            metadata={"jump_m": ref_dist, "cap_m": soft_cap_m, "ref": ref_name},
        )
        return float(bounded_lat), float(bounded_lon)

    def _find_best_bbox_match(self, detections) -> Optional[DetectionResult]:
        """Find detection closest to currently locked bbox (no ID, pure visual continuity)."""
        if not detections:
            return None

        candidates = self._collapse_group_candidates(detections, self._score_visual_match_candidate)

        if self._locked_bbox_center is None:
            return max(candidates,
                       key=lambda d: (d.bbox_size[0] * d.bbox_size[1]) if d.bbox_size else 0,
                       default=None)

        predicted_cx, predicted_cy = self._predicted_locked_center()
        last_area = self._locked_bbox_area()
        best_det = None
        best_score = float("inf")

        for det in candidates:
            if det.bbox_center is None or det.bbox_size is None:
                continue
            cx, cy = det.bbox_center
            dist = math.sqrt((cx - predicted_cx) ** 2 + (cy - predicted_cy) ** 2)
            if dist > min(float(MAX_VISUAL_REACQUIRE_DIST_PX), float(ATTACK_LOCK_MAX_JUMP_PX)):
                continue

            area = max(1.0, float(det.bbox_size[0]) * float(det.bbox_size[1]))
            size_penalty = 0.0
            if last_area is not None:
                ratio = min(area, last_area) / max(area, last_area)
                if ratio < float(ATTACK_LOCK_SIZE_RATIO_MIN):
                    continue
                size_penalty = (1.0 - ratio) * 80.0

            # Terminal visual aim is bbox-first. World position may drift during dive,
            # so it must not dominate bbox continuity selection.
            score = dist + size_penalty
            if score < best_score:
                best_score = score
                best_det = det

        if best_det is not None:
            return best_det

        recent_lock = (
            self._locked_bbox_center is not None
            and self._last_locked_bbox_ts > 0.0
            and (time.time() - float(self._last_locked_bbox_ts)) <= float(ATTACK_CENTER_LOSS_GRACE_S)
        )
        if not recent_lock:
            return None

        relaxed_det = None
        relaxed_score = float("inf")
        relaxed_dist_limit = min(float(MAX_VISUAL_REACQUIRE_DIST_PX) * 0.45, float(ATTACK_LOCK_MAX_JUMP_PX) * 1.15)
        relaxed_ratio_min = max(0.25, float(ATTACK_LOCK_SIZE_RATIO_MIN) * 0.65)

        for det in candidates:
            if det.bbox_center is None or det.bbox_size is None:
                continue
            cx, cy = det.bbox_center
            dist = math.sqrt((cx - predicted_cx) ** 2 + (cy - predicted_cy) ** 2)
            if dist > relaxed_dist_limit:
                continue

            area = max(1.0, float(det.bbox_size[0]) * float(det.bbox_size[1]))
            size_penalty = 0.0
            if last_area is not None:
                ratio = min(area, last_area) / max(area, last_area)
                if ratio < relaxed_ratio_min:
                    continue
                size_penalty = (1.0 - ratio) * 40.0

            score = dist + size_penalty
            if score < relaxed_score:
                relaxed_score = score
                relaxed_det = det

        return relaxed_det

    def _accept_terminal_visual(self, det: DetectionResult) -> bool:
        """Accept only stable terminal reacquire observations as reliable visual guidance."""
        if det is None or det.bbox_center is None or det.bbox_size is None:
            self._terminal_reacquire_streak = 0
            return False

        dist_limit = min(float(MAX_VISUAL_REACQUIRE_DIST_PX), float(ATTACK_LOCK_MAX_JUMP_PX))
        predicted_cx, predicted_cy = self._predicted_locked_center()
        dist = math.hypot(float(det.bbox_center[0]) - predicted_cx, float(det.bbox_center[1]) - predicted_cy)
        current_area = max(1.0, float(det.bbox_size[0]) * float(det.bbox_size[1]))
        tiny_target = current_area < 2200.0
        if tiny_target:
            dist_limit = max(dist_limit, float(MAX_VISUAL_REACQUIRE_DIST_PX) * 0.85)
        if dist > dist_limit:
            self._terminal_reacquire_streak = 0
            return False

        last_area = self._locked_bbox_area()
        if last_area is not None:
            ratio = min(current_area, last_area) / max(current_area, last_area)
            ratio_limit = max(0.12 if tiny_target else 0.30, float(ATTACK_LOCK_SIZE_RATIO_MIN) * (0.50 if tiny_target else 0.72))
            if ratio < ratio_limit:
                self._terminal_reacquire_streak = 0
                return False

        if self._terminal_guidance_mode == "route":
            self._terminal_reacquire_streak += 1
            if self._terminal_reacquire_streak < 2:
                return False
            self._log_attack_health(
                "terminal_visual_reacquired",
                "INFO",
                "Terminal visual reacquire confirmed, resuming visual guidance",
                event_type="attack_visual_reacquired",
                decision_reason="terminal_visual_reacquire_confirmed",
                interval_s=0.5,
                metadata={"phase": self.fsm.phase, "streak": self._terminal_reacquire_streak},
            )
        else:
            self._terminal_reacquire_streak = 0

        return True

    def _visual_family_token(self, det: DetectionResult) -> str:
        if getattr(det, "group_id", None) is not None:
            return f"group:{det.group_id}"
        if det.bbox_center is not None:
            band = int(float(det.bbox_center[0]) // 120.0)
            return f"bboxband:{band}"
        return "unknown"

    def _stabilize_attack_visual(self, det: DetectionResult, *, phase: str) -> bool:
        """Keep attack guidance on the same physical visual family despite ID churn."""
        if det is None or det.bbox_center is None or det.bbox_size is None:
            self._attack_visual_streak = 0
            return False

        token = self._visual_family_token(det)
        prev_token = getattr(self, "_attack_visual_token", None)
        predicted_cx, predicted_cy = self._predicted_locked_center()
        dist_px = math.hypot(float(det.bbox_center[0]) - predicted_cx, float(det.bbox_center[1]) - predicted_cy)
        area = max(1.0, float(det.bbox_size[0]) * float(det.bbox_size[1]))
        last_area = self._locked_bbox_area()
        ratio = 1.0
        if last_area is not None:
            ratio = min(area, last_area) / max(area, last_area)
        tiny_target = min(area, last_area or area) < 2200.0

        if prev_token is None:
            self._attack_visual_token = token
            self._attack_visual_streak = 1
            return True

        same_token = token == prev_token
        same_family_continuity = dist_px <= (240.0 if tiny_target else 150.0) and ratio >= (0.12 if tiny_target else 0.24)
        if same_token or same_family_continuity:
            if not same_token:
                self._log_attack_health(
                    "visual_family_switch",
                    "WARNING",
                    f"Attack visual family switch accepted | {prev_token} -> {token}",
                    event_type="attack_visual_family_switch",
                    decision_reason="attack_visual_family_switch",
                    interval_s=0.5,
                    metadata={"phase": phase, "dist_px": float(dist_px), "size_ratio": float(ratio)},
                )
            self._attack_visual_token = token
            self._attack_visual_streak = min(self._attack_visual_streak + 1, 1000)
            return True

        self._attack_visual_streak = 0
        self._log_attack_health(
            "visual_stabilizer_hold",
            "WARNING",
            f"Attack visual stabilizer held current family | keep={prev_token} reject={token}",
            event_type="attack_visual_stabilizer_hold",
            decision_reason="attack_visual_stabilizer_hold",
            interval_s=0.5,
            metadata={"phase": phase, "dist_px": float(dist_px), "size_ratio": float(ratio)},
        )
        return False

    def _probable_terminal_success(self, d_lat: float, d_lon: float, d_alt: float, coverage: float) -> bool:
        """Confirm likely terminal completion when route fallback remains coherent."""
        route_lat, route_lon = self._terminal_route()
        if route_lat is None or route_lon is None:
            return False

        proximity_limit = float(TERMINAL_EVENT_PROXIMITY_M) + max(
            0.0,
            min(12.0, float(d_alt) * float(TERMINAL_EVENT_PROXIMITY_ALT_SCALE)),
        )
        route_dist = GeoMath.haversine_distance(float(d_lat), float(d_lon), float(route_lat), float(route_lon))
        if route_dist > proximity_limit:
            return False

        last_visual_ts = float(getattr(self, "_last_reliable_visual_ts", 0.0) or 0.0)
        if last_visual_ts <= 0.0 or (time.time() - last_visual_ts) > float(TERMINAL_EVENT_LOCK_WINDOW_S):
            return False

        max_coverage = max(float(coverage), float(getattr(self, "_terminal_max_coverage", 0.0) or 0.0))
        return max_coverage >= float(TERMINAL_EVENT_PROBABLE_COVERAGE)

    def _collapse_group_candidates(self, detections, scorer) -> list:
        """Collapse same-group detections into one candidate for attack selection."""
        if not detections:
            return []

        grouped = {}
        for det in detections:
            group_id = getattr(det, "group_id", None)
            key = f"group:{group_id}" if group_id is not None else f"track:{getattr(det, 'track_id', None)}"
            current = grouped.get(key)
            if current is None:
                grouped[key] = det
                continue

            det_score = scorer(det)
            current_score = scorer(current)
            if det_score > current_score:
                grouped[key] = det
                continue

            if det_score == current_score:
                det_conf = float(getattr(det, "conf", 0.0) or 0.0)
                current_conf = float(getattr(current, "conf", 0.0) or 0.0)
                if det_conf > current_conf:
                    grouped[key] = det

        return list(grouped.values())

    def _score_visual_match_candidate(self, det: DetectionResult) -> float:
        """Score a detection for visual continuity."""
        if det.bbox_center is None or det.bbox_size is None:
            return float("-inf")

        if self._locked_bbox_center is None:
            return (
                float(det.bbox_size[0]) * float(det.bbox_size[1])
                + max(0.0, float(det.conf or 0.0)) * 10.0
            )

        predicted_cx, predicted_cy = self._predicted_locked_center()
        dist = math.hypot(float(det.bbox_center[0]) - predicted_cx, float(det.bbox_center[1]) - predicted_cy)
        if dist > min(float(MAX_VISUAL_REACQUIRE_DIST_PX), float(ATTACK_LOCK_MAX_JUMP_PX)):
            return float("-inf")

        score = max(0.0, 220.0 - dist)
        last_area = self._locked_bbox_area()
        if last_area is not None:
            det_area = max(1.0, float(det.bbox_size[0]) * float(det.bbox_size[1]))
            ratio = min(det_area, last_area) / max(det_area, last_area)
            if ratio < float(ATTACK_LOCK_SIZE_RATIO_MIN):
                return float("-inf")
            score += ratio * 20.0

        locked_group_id = getattr(self, "_locked_group_id", None)
        if (
            locked_group_id is not None
            and getattr(det, "group_id", None) is not None
            and str(det.group_id) == str(locked_group_id)
        ):
            score += 90.0

        if getattr(det, "is_group", False):
            score += 15.0
            score += min(25.0, float(getattr(det, "group_member_count", 1) or 1) * 4.0)

        score += max(0.0, float(det.conf or 0.0)) * 8.0
        return score

    def _locked_bbox_area(self) -> Optional[float]:
        size = getattr(self, "_locked_bbox_size", None)
        if size is None:
            return None
        return max(1.0, float(size[0]) * float(size[1]))

    def _predicted_locked_center(self) -> Tuple[float, float]:
        center = getattr(self, "_locked_bbox_center", None)
        if center is None:
            return (0.0, 0.0)
        prev_center = self._prev_attack_center
        prev_ts = float(getattr(self, "_prev_attack_center_ts", 0.0) or 0.0)
        lock_ts = float(getattr(self, "_last_locked_bbox_ts", 0.0) or 0.0)
        if prev_center is None or prev_ts <= 0.0 or lock_ts <= prev_ts:
            return (float(center[0]), float(center[1]))

        dt = lock_ts - prev_ts
        if dt <= 1e-3 or dt > 0.4:
            return (float(center[0]), float(center[1]))

        vx = (float(center[0]) - float(prev_center[0])) / dt
        vy = (float(center[1]) - float(prev_center[1])) / dt
        lookahead = min(0.18, max(0.04, time.time() - lock_ts))
        return (float(center[0]) + vx * lookahead, float(center[1]) + vy * lookahead)

    def _stream_annotated_frame(self, tracker_result):
        """Encode annotated frame (with bbox overlay) and push to camera UI."""
        if tracker_result is None:
            return
        annotated = tracker_result.get("annotated_frame")
        if annotated is None:
            return

        frame = annotated.copy()
        if self.fsm.phase:
            cv2.putText(frame, f"PHASE: {self.fsm.phase}", (10, 140),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        track_id = tracker_result.get("track_id")
        if track_id is not None:
            cv2.putText(frame, f"Track ID: {track_id}", (10, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)

        ret, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
        if ret:
            jpeg = buf.tobytes()
            target_port = self.port
            try:
                with self.camera_handler.lock:
                    if target_port in self.camera_handler.camera_feeds:
                        entry = self.camera_handler.camera_feeds[target_port]
                        if isinstance(entry, dict):
                            entry["jpeg"] = jpeg
                    else:
                        self.camera_handler.camera_feeds[target_port] = {"raw": None, "jpeg": jpeg}
            except Exception:
                pass

    def _update_frame_shape(self, tracker_result) -> None:
        if not isinstance(tracker_result, dict):
            return
        frame = tracker_result.get("annotated_frame")
        if frame is None or not hasattr(frame, "shape") or len(frame.shape) < 2:
            return
        self._latest_frame_shape = (int(frame.shape[0]), int(frame.shape[1]))

    def _frame_size(self) -> Tuple[float, float]:
        latest = getattr(self, "_latest_frame_shape", None)
        if latest is not None:
            frame_h, frame_w = latest
            return float(frame_w), float(frame_h)
        return float(CAMERA_WIDTH), float(CAMERA_HEIGHT)

    def _reset_framing_state(self) -> None:
        self._framing_center = None
        self._framing_center_ts = 0.0
        self._framing_centered_count = 0
        self._framing_locked = False
        self._observer_visual_anchor_ts = 0.0
        self._observer_visual_anchor_yaw_rate = 0.0

    def _reset_attack_state(self, *, clear_visual_lock: bool = False) -> None:
        yaw_filter = getattr(self, "_yaw_filter", None)
        if yaw_filter is not None:
            yaw_filter.reset()
        velocity_smoother = getattr(self, "_velocity_smoother", None)
        if velocity_smoother is not None:
            velocity_smoother.reset()
        self._last_visual_cmd = (0.0, 0.0, 0.0, 0.0)
        self._last_visual_cmd_ts = 0.0
        self._last_reliable_visual_ts = 0.0
        self._terminal_guidance_mode = "visual"
        self._terminal_reacquire_streak = 0
        self._terminal_probable_confirm_count = 0
        self._terminal_max_coverage = 0.0
        self._route_fallback_started_ts = 0.0
        self._terminal_planned_point_frozen = False
        self._attack_visual_token = None
        self._attack_visual_streak = 0
        if clear_visual_lock:
            self._lock_confirm_count = 0
            self._lock_loss_count = 0
            self._locked_bbox_center = None
            self._locked_bbox_size = None
            self._locked_target_gps = None
            self._locked_group_id = None
            self._locked_group_count = 0
            self._centering_settle_count = 0
            self._prev_attack_center = None
            self._prev_attack_center_ts = 0.0
            self._last_locked_bbox_ts = 0.0

    def _apply_framing_yaw(self, vehicle, tracker_result) -> None:
        """Apply pre-attack framing yaw."""
        if vehicle is None or vehicle == "connecting":
            return

        velocity_cmd = tracker_result.get("velocity_cmd", {}) if isinstance(tracker_result, dict) else {}
        yaw_rate = float(velocity_cmd.get("yaw_rate", 0.0) or 0.0)

        if abs(yaw_rate) < 0.05:
            yaw_rate = 0.0

        if yaw_rate == 0.0:
            self._yaw_filter.reset()
            self._observer_visual_anchor_ts = time.time()
            self._observer_visual_anchor_yaw_rate = 0.0
            try:
                self.flight_ctrl.command_body_velocity(
                    vehicle,
                    "TRACK_SOFT",
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    attack_approved=False,
                    reason="observer framing hold",
                )
            except Exception:
                pass
            return

        try:
            self._observer_visual_anchor_ts = time.time()
            self._observer_visual_anchor_yaw_rate = float(yaw_rate)
            self.flight_ctrl.command_body_velocity(
                vehicle,
                "TRACK_SOFT",
                0.0,
                0.0,
                0.0,
                yaw_rate,
                attack_approved=False,
                reason="observer framing",
            )
        except Exception:
            pass

    def _apply_framing_loss_hold(self, vehicle) -> None:
        if vehicle is None or vehicle == "connecting":
            return
        try:
            now = time.time()
            tracker_seen_ts = float(getattr(self.human_tracker, "last_detection_time", 0.0) or 0.0)
            anchor_ts = float(getattr(self, "_observer_visual_anchor_ts", 0.0) or 0.0)
            seen_ts = max(tracker_seen_ts, anchor_ts)
            time_since_seen = now - seen_ts if seen_ts > 0.0 else float("inf")
            grace_s = max(0.8, float(FRAMING_TARGET_LOSS_GRACE_S))
            coast_s = min(float(HUMAN_LOST_TIMEOUT), max(grace_s * 3.0, 2.5))
            last_cmd = getattr(self.human_tracker, "last_velocity_command", {}) or {}
            yaw_rate = float(last_cmd.get("yaw_rate", 0.0) or 0.0)
            anchor_yaw = float(getattr(self, "_observer_visual_anchor_yaw_rate", 0.0) or 0.0)
            if abs(yaw_rate) < abs(anchor_yaw):
                yaw_rate = anchor_yaw
            if 0.0 < time_since_seen <= grace_s:
                decay = max(0.0, 1.0 - (time_since_seen / grace_s))
                self.flight_ctrl.command_body_velocity(
                    vehicle,
                    "TRACK_SOFT",
                    0.0,
                    0.0,
                    0.0,
                    yaw_rate * decay,
                    attack_approved=False,
                    reason="observer yaw grace",
                )
                return
            if 0.0 < time_since_seen <= coast_s:
                coast_decay = max(0.0, 1.0 - ((time_since_seen - grace_s) / max(1e-3, coast_s - grace_s)))
                self.flight_ctrl.command_body_velocity(
                    vehicle,
                    "TRACK_SOFT",
                    0.0,
                    0.0,
                    0.0,
                    yaw_rate * 0.25 * coast_decay,
                    attack_approved=False,
                    reason="observer visual coast",
                )
                return
        except Exception:
            pass

        try:
            self.flight_ctrl.command_hold(vehicle, reason="observer lost hold", failsafe=True)
        except Exception:
            pass

    def _yaw_toward_bbox(self, vehicle_or_det, bbox_center=None):
        """Rotate drone toward bbox center (no lateral movement)."""
        vehicle = vehicle_or_det
        if bbox_center is None:
            bbox_center = vehicle_or_det
            vehicle = self.drone_manager.drones.get(self.port)
        cx = bbox_center[0]
        img_w, _ = self._frame_size()
        error_x = (cx - img_w / 2.0) / (img_w / 2.0)

        if abs(error_x) < CENTERING_YAW_DEADZONE:
            self._yaw_filter.reset()
            return

        yaw_rate = self._yaw_filter.update(CENTERING_YAW_KP * error_x)
        yaw_rate = max(-CENTERING_YAW_RATE_MAX, min(CENTERING_YAW_RATE_MAX, yaw_rate))
        try:
            if vehicle and vehicle != "connecting":
                self.flight_ctrl.command_body_velocity(
                    vehicle,
                    "TRACK_SOFT",
                    0.0,
                    0.0,
                    0.0,
                    yaw_rate,
                    attack_approved=False,
                    reason="scene framing yaw",
                )
        except Exception:
            pass

    def _bbox_normalized_error(self, bbox_center):
        if bbox_center is None:
            return 0.0, 0.0
        img_w, img_h = self._frame_size()
        img_cx, img_cy = self._frame_aimpoint()
        error_x = (float(bbox_center[0]) - img_cx) / max(img_cx, 1.0)
        error_y = (float(bbox_center[1]) - img_cy) / max(img_cy, 1.0)
        return error_x, error_y

    def _bbox_within_center_gate(self, bbox_center) -> bool:
        if bbox_center is None:
            return False
        img_cx, img_cy = self._frame_aimpoint()
        dx = abs(float(bbox_center[0]) - img_cx)
        dy = abs(float(bbox_center[1]) - img_cy)
        return dx <= ATTACK_CENTER_ACCEPT_RADIUS_PX and dy <= ATTACK_CENTER_ACCEPT_RADIUS_PX

    def _attack_steering_components(self, bbox_center):
        error_x, error_y = self._compensated_bbox_error(bbox_center)
        deadzone_x, deadzone_y = self._attack_axis_deadzones()
        steer_x = self._steering_axis(error_x, deadzone_x)
        steer_y = self._steering_axis(error_y, deadzone_y)
        return steer_x, steer_y

    def _compensated_bbox_error(self, bbox_center):
        error_x, error_y = self._bbox_normalized_error(bbox_center)
        if bbox_center is None:
            return error_x, error_y
        now = time.time()
        prev_center = getattr(self, "_prev_attack_center", None)
        prev_ts = float(getattr(self, "_prev_attack_center_ts", 0.0) or 0.0)
        self._prev_attack_center = (float(bbox_center[0]), float(bbox_center[1]))
        self._prev_attack_center_ts = now
        if prev_center is None or prev_ts <= 0.0:
            return error_x, error_y

        dt = now - prev_ts
        if dt <= 1e-3 or dt > 0.35:
            return error_x, error_y

        img_w, img_h = self._frame_size()
        rate_x = ((float(bbox_center[0]) - float(prev_center[0])) / max(img_w / 2.0, 1.0)) / dt
        rate_y = ((float(bbox_center[1]) - float(prev_center[1])) / max(img_h / 2.0, 1.0)) / dt
        lead_horizon_s = 0.18
        error_x += rate_x * lead_horizon_s
        error_y += rate_y * lead_horizon_s
        return max(-1.0, min(1.0, error_x)), max(-1.0, min(1.0, error_y))

    def _frame_aimpoint(self) -> Tuple[float, float]:
        img_w, img_h = self._frame_size()
        scale_y = img_h / max(float(CAMERA_HEIGHT), 1.0)
        pitch_offset_px = float(CAMERA_FY) * math.tan(float(CAMERA_PITCH_OFFSET)) * scale_y
        aim_cx = img_w / 2.0
        aim_cy = max(0.0, min(img_h, (img_h / 2.0) - pitch_offset_px))
        return aim_cx, aim_cy

    def _attack_axis_deadzones(self) -> Tuple[float, float]:
        img_w, img_h = self._frame_size()
        deadzone_x = max(CENTERING_YAW_DEADZONE, ATTACK_CENTER_BLIND_BOX_PX / max(img_w / 2.0, 1.0))
        deadzone_y = max(CENTERING_YAW_DEADZONE, ATTACK_CENTER_BLIND_BOX_PX / max(img_h / 2.0, 1.0))
        return deadzone_x, deadzone_y

    @staticmethod
    def _compute_yaw_rate(steer_x: float, *, gain: float, limit: float) -> float:
        yaw_rate = float(gain) * float(steer_x)
        return max(-float(limit), min(float(limit), yaw_rate))

    @staticmethod
    def _compute_vertical_speed(
        steer_y: float,
        *,
        max_down: float,
        max_up: float,
        authority: float = 1.0,
        bias_down: float = 0.0,
    ) -> float:
        """Compute visual vertical speed."""
        command = bias_down + (ATTACK_DIVE_VERTICAL_KP * float(authority) * float(steer_y))
        return max(-float(max_up), min(float(max_down), command))

    def _update_planned_terminal_point(
        self,
        lat: float,
        lon: float,
        alt: float,
        d_lat: float,
        d_lon: float,
        source: str,
    ):
        OFFSET_M = 4.0
        planned = getattr(self, "_planned_terminal_point", None) or {}
        prev_lat = planned.get("lat")
        prev_lon = planned.get("lon")
        prev_source = str(planned.get("source") or "")

        if (
            source == "approach_visual"
            and getattr(self.fsm, "phase", None) == "APPROACH"
            and bool(getattr(self, "_terminal_planned_point_frozen", False))
            and prev_lat is not None
            and prev_lon is not None
        ):
            frozen_jump_m = GeoMath.haversine_distance(float(prev_lat), float(prev_lon), float(lat), float(lon))
            if frozen_jump_m > 6.0:
                self._log_attack_health(
                    "terminal_planned_point_frozen",
                    "WARNING",
                    f"Frozen planned terminal point during visual dive | source={source} jump={frozen_jump_m:.1f}m",
                    event_type="attack_planned_point_frozen",
                    decision_reason="terminal_planned_point_frozen",
                    interval_s=0.5,
                    metadata={"jump_m": frozen_jump_m, "prev_source": prev_source},
                )
                return

        if source == "approach_visual" and prev_lat is not None and prev_lon is not None:
            jump_m = GeoMath.haversine_distance(float(prev_lat), float(prev_lon), float(lat), float(lon))
            if jump_m > 12.0:
                self._log_attack_health(
                    "terminal_route_jump_blocked",
                    "WARNING",
                    f"Blocked terminal route jump | source={source} jump={jump_m:.1f}m",
                    event_type="attack_route_guard",
                    decision_reason="terminal_route_jump_blocked",
                    interval_s=0.5,
                    metadata={"jump_m": jump_m, "prev_source": prev_source},
                )
                return
            if jump_m > 4.0:
                bearing = GeoMath.bearing(float(prev_lat), float(prev_lon), float(lat), float(lon))
                step_north = 4.0 * math.cos(math.radians(bearing))
                step_east = 4.0 * math.sin(math.radians(bearing))
                lat, lon = GeoMath.ned_to_gps(float(prev_lat), float(prev_lon), step_north, step_east)

        distance = GeoMath.haversine_distance(d_lat, d_lon, lat, lon)
        if distance > (OFFSET_M + 2.0):
            bearing_to_drone = GeoMath.bearing(lat, lon, d_lat, d_lon)
            d_north = OFFSET_M * math.cos(math.radians(bearing_to_drone))
            d_east = OFFSET_M * math.sin(math.radians(bearing_to_drone))
            lat, lon = GeoMath.ned_to_gps(lat, lon, d_north, d_east)

        self._planned_terminal_point = {
            "lat": float(lat),
            "lon": float(lon),
            "alt": float(alt),
            "ts": time.time(),
            "source": source,
        }
        if source == "attack_commit":
            self._terminal_planned_point_frozen = True

    def _terminal_route(self):
        planned = getattr(self, "_planned_terminal_point", None) or {}
        lat = planned.get("lat")
        lon = planned.get("lon")
        if lat is not None and lon is not None:
            return lat, lon
        fsm = getattr(self, "fsm", None)
        lat = getattr(fsm, "assigned_target_lat", None)
        lon = getattr(fsm, "assigned_target_lon", None)
        return lat, lon

    def _is_terminal_area_reached(self, d_lat: float, d_lon: float) -> bool:
        route_lat, route_lon = self._terminal_route()
        if route_lat is None or route_lon is None:
            return False
        return GeoMath.haversine_distance(d_lat, d_lon, route_lat, route_lon) <= _MISSION_AREA_RADIUS_M

    def _steering_axis(self, error: float, deadzone: float) -> float:
        """Single-profile steering shape used for both horizontal and vertical aim."""
        abs_error = abs(float(error))
        if abs_error <= deadzone:
            return 0.0
        span = max(1e-6, 1.0 - deadzone)
        scaled = min(1.0, (abs_error - deadzone) / span)
        shaped = scaled * scaled * (3.0 - 2.0 * scaled)
        return math.copysign(shaped, error)

    def _compute_visual_dive_command(self, vx_override: float = None):
        """Build the visual dive command from target line of sight."""
        center = self._locked_bbox_center
        if center is None:
            return None

        target_vx = self._dive_forward_speed(vx_override)

        img_w, img_h = self._frame_size()
        cx, cy = center

        angle_x = math.atan((float(cx) - (img_w / 2.0)) / float(CAMERA_FX))
        angle_y = math.atan((float(cy) - (img_h / 2.0)) / float(CAMERA_FY))

        yaw_rate = angle_x * float(ATTACK_DIVE_YAW_KP)
        yaw_limit = max(CENTERING_YAW_RATE_MAX, 0.75)
        startup_ratio = self._dive_startup_ratio()
        yaw_rate *= max(float(ATTACK_DIVE_STARTUP_YAW_SCALE), startup_ratio)
        yaw_rate = max(-yaw_limit, min(yaw_limit, yaw_rate))
        acceleration_pitch = (
            max(0.0, float(target_vx) - float(ATTACK_DIVE_STARTUP_SPEED_MPS))
            * 0.015
            * float(ATTACK_DIVE_PITCH_COMPENSATION_GAIN)
            * startup_ratio
        )
        aerodynamic_pitch = float(target_vx) * 0.010 + acceleration_pitch
        total_camera_pitch = float(CAMERA_PITCH_OFFSET) + aerodynamic_pitch

        target_depression_angle = total_camera_pitch + angle_y
        vz_required = float(target_vx) * math.tan(target_depression_angle) * float(ATTACK_DIVE_DESCENT_SOFTENING)

        allowable_max_descent = max(float(ATTACK_DIVE_MAX_DESCENT_MPS), float(target_vx) * 1.5)
        vz_raw = max(-float(ATTACK_DIVE_MAX_CLIMB_MPS), min(allowable_max_descent, vz_required))
        vz_raw *= max(float(ATTACK_DIVE_STARTUP_VZ_SCALE), startup_ratio)

        yaw_rate = self._yaw_filter.update(yaw_rate)
        vx, _, vz = self._velocity_smoother.smooth(float(target_vx), 0.0, vz_raw)

        return vx, 0.0, vz, yaw_rate

    def _dive_startup_ratio(self) -> float:
        if getattr(self.fsm, "phase", None) != "APPROACH":
            return 1.0
        try:
            elapsed = float(self.fsm.phase_elapsed())
        except Exception:
            elapsed = 0.0
        ramp_s = max(0.1, float(ATTACK_DIVE_STARTUP_RAMP_S or IBVS_RAMP_UP_TIME_S))
        return max(0.0, min(1.0, elapsed / ramp_s))

    def _dive_forward_speed(self, vx_override: float = None) -> float:
        if vx_override is not None:
            return float(vx_override)
        startup_vx = max(float(IBVS_STARTUP_VX_MIN), float(ATTACK_DIVE_STARTUP_SPEED_MPS))
        cruise_vx = float(ATTACK_DIVE_TERMINAL_FORWARD_SPEED)
        ratio = self._dive_startup_ratio()
        return startup_vx + (cruise_vx - startup_vx) * ratio

    def _coast_visual_dive_command(self):
        if float(getattr(self, "_last_visual_cmd_ts", 0.0) or 0.0) <= 0.0:
            return (ATTACK_DIVE_TERMINAL_FORWARD_SPEED, 0.0, 0.0, 0.0)
        return self._last_visual_cmd

    @staticmethod
    def _normalize_heading_delta(delta_deg: float) -> float:
        while delta_deg > 180.0:
            delta_deg -= 360.0
        while delta_deg < -180.0:
            delta_deg += 360.0
        return delta_deg

    def _route_fallback_command(self, vehicle, d_lat: float, d_lon: float):
        """Steer toward the last reliable terminal route when visual is gone."""
        route_lat, route_lon = self._terminal_route()
        if route_lat is None or route_lon is None:
            return self._coast_visual_dive_command()

        command = self._coast_visual_dive_command()
        last_vx, _, last_vz, _ = command
        vx = max(float(self._dive_forward_speed()), float(last_vx))
        vz = float(last_vz)

        heading_deg = float(getattr(vehicle, "heading", 0.0) or 0.0)
        desired_bearing = GeoMath.bearing(float(d_lat), float(d_lon), float(route_lat), float(route_lon))
        heading_error = self._normalize_heading_delta(float(desired_bearing) - heading_deg)
        yaw_rate = max(-CENTERING_YAW_RATE_MAX, min(CENTERING_YAW_RATE_MAX, math.radians(heading_error) * 0.9))

        smoother = getattr(self, "_velocity_smoother", None)
        if smoother is not None:
            vx, _, vz = smoother.smooth(float(vx), 0.0, float(vz))
        return vx, 0.0, vz, yaw_rate

    def _fly_visual_attack(self, vehicle) -> None:
        command = self._compute_visual_dive_command()
        if command is None:
            command = self._coast_visual_dive_command()
        vx, vy, vz, yaw_rate = command
        self._last_visual_cmd = (vx, vy, vz, yaw_rate)
        self._last_visual_cmd_ts = time.time()
        try:
            self.flight_ctrl.command_body_velocity(
                vehicle,
                "ATTACK",
                vx,
                vy,
                vz,
                yaw_rate,
                attack_approved=True,
                reason="visual kamikaze dive",
            )
        except Exception:
            pass

    def _fly_route_terminal_attack(self, vehicle, d_lat: float, d_lon: float) -> None:
        """Keep flying to the last reliable terminal airpoint after visual loss."""
        vx, vy, vz, yaw_rate = self._route_fallback_command(vehicle, d_lat, d_lon)
        self._last_visual_cmd = (vx, vy, vz, yaw_rate)
        self._last_visual_cmd_ts = time.time()
        try:
            self.flight_ctrl.command_body_velocity(
                vehicle,
                "ATTACK",
                vx,
                vy,
                vz,
                yaw_rate,
                attack_approved=True,
                reason="terminal route fallback",
            )
        except Exception:
            pass

    def _publish_terminal_update(self, d_lat: float, d_lon: float, d_alt: float) -> None:
        route_lat, route_lon = self._terminal_route()
        if route_lat is None or route_lon is None:
            return
        self.bridge.publish_autonomous_update(
            self.fsm.assigned_target_id,
            (d_lat, d_lon, d_alt),
            target_gps=(route_lat, route_lon),
        )

    def _confirm_impact(self, coverage: float, altitude: float, reason: str):
        """Confirm terminal impact and trigger RTL."""
        if self._terminal_completion_pending:
            return
        valid_reasons = {
            "visual_coverage_reached",
            "terminal_airpoint_reached",
            "probable_impact_on_visual_loss",
        }
        if reason not in valid_reasons:
            return
        self.fsm.enter_impact()
        if self.fsm.phase != "IMPACT":
            return
        event_id = f"{self.port}:{int(time.time() * 1000)}"
        self._terminal_event = {
            "id": event_id,
            "kind": "IMPACT",
            "type": "IMPACT",
            "coverage": coverage,
            "altitude": altitude,
            "reason": reason,
            "ts": time.time(),
        }
        self._terminal_completion_pending = True
        self.tracking_mode = False
        self.scanning_active = False

        SwarmLogger.log_operational(
            key=f"attack:impact:{self.port}:{event_id}",
            level="SUCCESS",
            module=f"DRONE_{self.port}",
            message=f"IMPACT_CONFIRMED | coverage={coverage:.1f}% alt={altitude:.1f}m reason={reason}",
            source="ATTACK_FLOW",
            interval_s=0.0,
            event_type="attack_impact_confirmed",
            structured_source="ATTACK_FLOW",
            structured_level="SUCCESS",
            drone_id=self.port,
            target_id=self.fsm.assigned_target_id,
            state_before=self.fsm.phase,
            decision_reason=reason,
            confidence=float(coverage),
            metadata={"altitude": altitude},
        )

        try:
            self.bridge.publish_terminal_event(
                self.fsm.assigned_target_id,
                coverage,
                altitude,
            )
        except Exception:
            pass
        self._log_attack_health(
            "success_path",
            "SUCCESS",
            f"Attack success path | reason={reason}",
            event_type="attack_success_path",
            decision_reason=reason,
            interval_s=0.0,
            metadata={
                "phase": self.fsm.phase,
                "guidance_mode": getattr(self, "_terminal_guidance_mode", "unknown"),
                "max_coverage": float(getattr(self, "_terminal_max_coverage", 0.0) or 0.0),
            },
        )

    def _handle_attack_failure(self, reason: str):
        """Handle attack failure — enter scan resume."""
        self._log_attack_health(
            f"failure:{reason}",
            "WARNING",
            f"Attack failed: {reason}",
            event_type="attack_failure",
            decision_reason=reason,
            interval_s=5.0,
            metadata={
                "phase": self.fsm.phase,
                "max_coverage": float(getattr(self, "_terminal_max_coverage", 0.0) or 0.0),
                "guidance_mode": getattr(self, "_terminal_guidance_mode", "unknown"),
            },
        )
        self.fsm.enter_scan_resume(reason)
        self._terminal_event = {
            "id": f"{self.port}:failure:{int(time.time() * 1000)}",
            "kind": "FAILURE",
            "type": "FAILURE",
            "reason": reason,
            "ts": time.time(),
        }
        self._terminal_completion_pending = True
        self.tracking_mode = False
        self.scanning_active = True

    @staticmethod
    def _bbox_coverage(bbox_size) -> float:
        if bbox_size is None:
            return 0.0
        try:
            bw, bh = bbox_size
            return max(0.0, (float(bw) * float(bh)) / (CAMERA_WIDTH * CAMERA_HEIGHT) * 100.0)
        except Exception:
            return 0.0
