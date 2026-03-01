"""ORCUS Tracking Controller - Human detection checks and tracking/attack state machine."""

import time
import math
import logging
import cv2
import numpy as np
from collections import deque
from statistics import mean

from modules.mission.flight_controller import FlightController
from modules.core.logger import SwarmLogger
from modules.core.geo_math import GeoMath
from modules.core.kalman_filter import AdaptiveGPSKalmanFilter
from modules.mission.ibvs_guidance import IBVSGuidance, VelocitySmoother
from config import (
    CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_RESOLUTION_WIDTH,

    GPS_FILTER_WINDOW_SIZE, BBOX_FILTER_WINDOW_SIZE,
    IBVS_VX_MAX, IBVS_VX_MIN, IBVS_VZ_MAX, IBVS_TERMINAL_COVERAGE,
    IBVS_ALTITUDE_FLOOR_M, IBVS_SHALLOW_DIVE_VZ_MAX,
    LEADER_TIMEOUT_S, EDGE_MARGIN_RATIO, MIN_BOX_DIM,
    JPEG_QUALITY, MISSION_STATUS_MESSAGES,
    ATTACK_ECHO_TIMEOUT_S, ATTACK_CENTER_TIMEOUT_S,
    ATTACK_CENTER_ERROR_THRESHOLD, ATTACK_CENTER_CONFIRM_FRAMES,
    SWARM_GEO_VALIDATION_THRESHOLD_M,
    ATTACK_TIMEOUT_S,
    # Centering Stabilization
    CENTERING_YAW_KP, CENTERING_YAW_RATE_MAX, CENTERING_YAW_DEADZONE,
    CENTERING_SETTLE_FRAMES, CENTERING_YAW_FILTER_ALPHA,
    # Impact Detection
    IMPACT_COVERAGE_THRESHOLD, IMPACT_ALTITUDE_MIN_M,
    IMPACT_CONFIRM_FRAMES, IMPACT_VELOCITY_THRESHOLD,
    IMPACT_ALTITUDE_PROGRESSIVE, IMPACT_ALTITUDE_HIGH_M, IMPACT_ALTITUDE_LOW_M,
    # Phase Transition Buffer
    PHASE_TRANSITION_HOLD_S, PHASE_TRANSITION_VELOCITY_ZERO,
    CENTERING_TO_DIVING_HOLD_S
)

class TrackingController:
    """Human tracking and attack state machine."""

    def __init__(self, flight_ctrl, camera_handler, human_tracker,
                 swarm_manager, drone_manager, port=None):
        """Initialize tracking controller with required components."""
        self.flight_ctrl = flight_ctrl
        self.camera_handler = camera_handler
        self.human_tracker = human_tracker
        self.swarm_manager = swarm_manager
        self.drone_manager = drone_manager
        self.port = port

        # IBVS Guidance
        self.ibvs_guidance = IBVSGuidance()
        self.velocity_smoother = VelocitySmoother(max_accel=1.0, max_decel=2.0)

        # Filters
        self.gps_lat_history = deque(maxlen=GPS_FILTER_WINDOW_SIZE)
        self.gps_lon_history = deque(maxlen=GPS_FILTER_WINDOW_SIZE)
        self.gps_kalman_filter = None
        self.bbox_width_history = deque(maxlen=BBOX_FILTER_WINDOW_SIZE)

        # Engagement state
        self.attack_approved = False
        self.current_target_id = None
        self.target_gps_lat = None
        self.target_gps_lon = None
        self.assigned_local_tracker_id = None
        self.lock_status = None  # None, "REQUESTING", "APPROVED", "LOCKED"
        self.tracking_mode = False
        self.collision_detected = False
        self.scanning_active = False

        # Centering Stabilization
        self.centering_yaw_filter = None  # LowPassFilter instance
        self.centering_settle_count = 0

        # Impact Detection
        self.impact_confirm_count = 0
        self.impact_detected = False

        # Phase Transition Buffer
        self.phase_transition_start = None
        self.last_attack_phase = None

        # Camera Detection Log (rate-limited)
        self._camera_log_last_ts = 0.0
        self._camera_log_interval = 2.0  # Log every 2 seconds

    def reset_engagement_state(self):
        """Clear all engagement state — called on stop mission."""
        self.attack_approved = False
        self.current_target_id = None
        self.assigned_local_tracker_id = None
        self.lock_status = None
        self.target_gps_lat = None
        self.target_gps_lon = None
        self.collision_detected = False
        self.tracking_mode = False
        # Reset centering state
        self.centering_yaw_filter = None
        self.centering_settle_count = 0
        # Reset impact state
        self.impact_confirm_count = 0
        self.impact_detected = False
        # Reset phase transition state
        self.phase_transition_start = None
        self.last_attack_phase = None

    def check_human_detection(self, is_mission_active):
        """Check camera for human detections and report to swarm manager."""
        if not self.scanning_active:
            return False

        try:
            target_port = self.port or self.drone_manager.active_drone_port
            frame = self.camera_handler.get_latest_frame_as_array(target_port)
            if frame is None:
                return False

            tracker_result = self.human_tracker.detect_and_track(frame, debug=False)
            if tracker_result is None:
                return False

            detected_status = tracker_result.get('detected', False)
            if not detected_status:
                return False

            detections = tracker_result.get('all_detections', [])
            if not detections:
                return False

            boxes = detections[0].boxes
            found_assigned_target = False

            # ============================================================
            # CAMERA DETECTION LOG - Kamerada görünen her şeyi kaydet
            # ============================================================
            now = time.time()
            if now - self._camera_log_last_ts >= self._camera_log_interval:
                self._camera_log_last_ts = now
                detection_summary = []
                for j, bx in enumerate(boxes):
                    try:
                        if hasattr(bx.xywh[0], 'tolist'):
                            bx_xywh = bx.xywh[0].tolist()
                        else:
                            bx_xywh = bx.xywh[0]
                        bx_cx, bx_cy, bx_w, bx_h = bx_xywh[0], bx_xywh[1], bx_xywh[2], bx_xywh[3]
                        bx_id = bx.id[0].item() if bx.id is not None else None
                        bx_conf = bx.conf[0].item() if hasattr(bx.conf[0], 'item') else float(bx.conf[0]) if bx.conf is not None else 0.0
                        detection_summary.append(f"ID:{bx_id} pos:({bx_cx:.0f},{bx_cy:.0f}) size:({bx_w:.0f}x{bx_h:.0f}) conf:{bx_conf:.2f}")
                    except Exception:
                        pass
                
                # Grup bilgisi varsa ekle
                group_info = tracker_result.get('group_info')
                group_summary = ""
                if group_info and hasattr(group_info, 'groups') and group_info.groups:
                    group_summary = " | GROUPS: "
                    for grp in group_info.groups:
                        g_lat, g_lon = grp.center_gps
                        group_summary += f"[G{grp.group_id} x{grp.member_count} @({g_lat:.6f},{g_lon:.6f})] "
                
                SwarmLogger.log("CAMERA", f"DRONE_{self.port}", 
                    f"DETECTIONS: {len(boxes)} targets | {' | '.join(detection_summary)}{group_summary}", "VISION")
            # ============================================================

            for i, box in enumerate(boxes):
                try:
                    if hasattr(box.xywh[0], 'tolist'):
                        box_xywh = box.xywh[0].tolist()
                    else:
                        box_xywh = box.xywh[0]

                    center_x, center_y, w, h = box_xywh[0], box_xywh[1], box_xywh[2], box_xywh[3]

                    # Edge filter
                    w_img, h_img = CAMERA_WIDTH, CAMERA_HEIGHT
                    effective_margin = EDGE_MARGIN_RATIO

                    # Relax margin for locked target
                    if self.attack_approved and self.assigned_local_tracker_id is not None:
                        box_id = box.id[0].item() if box.id is not None else None
                        if box_id is not None and int(box_id) == self.assigned_local_tracker_id:
                            effective_margin = 0.02
                            SwarmLogger.log("DEBUG", f"DRONE_{self.port}",
                                f"R35: Edge margin relaxed to 2% for locked target ID:{box_id}", "EDGE_FILTER")

                    margin_x = w_img * effective_margin
                    margin_y = h_img * effective_margin

                    is_edge_det = (center_x < margin_x or center_x > (w_img - margin_x) or
                        center_y < margin_y or center_y > (h_img - margin_y) or
                        w < MIN_BOX_DIM or h < MIN_BOX_DIM)
                    
                    if is_edge_det:
                        box_id = box.id[0].item() if box.id is not None else None
                        SwarmLogger.log("DEBUG", f"DRONE_{self.port}",
                            f"Edge detection (low conf): ID:{box_id} pos=({center_x:.0f},{center_y:.0f})", "EDGE_OBS")
                        is_low_confidence = True
                    else:
                        is_low_confidence = False

                    # Get vehicle telemetry
                    if self.port and self.port in self.drone_manager.drones:
                        vehicle = self.drone_manager.drones[self.port]
                        loc = vehicle.location.global_relative_frame
                        if not loc or loc.lat == 0:
                            continue

                        drone_lat, drone_lon, drone_alt = loc.lat, loc.lon, loc.alt
                        drone_heading = vehicle.heading

                        self.bbox_width_history.append(w)

                        # Ray-Ground Intersection
                        roll = vehicle.attitude.roll
                        pitch = vehicle.attitude.pitch
                        yaw = vehicle.attitude.yaw

                        # Forward drone pose to detector for group clustering
                        self.human_tracker.set_drone_pose(
                            drone_lat, drone_lon, drone_alt, roll, pitch, yaw
                        )

                        result_rgi = GeoMath.ray_ground_intersection(
                            drone_lat, drone_lon, drone_alt,
                            roll, pitch, yaw,
                            center_x, center_y
                        )

                        # Safe extraction with None guard
                        target_covariance = None
                        t_lat, t_lon = None, None
                        if result_rgi is not None and len(result_rgi) >= 2 and result_rgi[0] is not None:
                            t_lat, t_lon = result_rgi[0], result_rgi[1]
                            if len(result_rgi) > 2:
                                target_covariance = result_rgi[2]

                        if t_lat is not None:
                            # Initialize Kalman filter if needed
                            if self.gps_kalman_filter is None:
                                self.gps_kalman_filter = AdaptiveGPSKalmanFilter(t_lat, t_lon)
                                SwarmLogger.log("INFO", f"DRONE_{self.port}", "Initialized GPS Kalman Filter", "KALMAN")
                            
                            # Extract measurement covariance
                            from config import METERS_PER_DEGREE_LAT
                            
                            if target_covariance is not None:
                                sigma_nn = target_covariance[0, 0]
                                sigma_ee = target_covariance[1, 1]
                                sigma_ne = target_covariance[0, 1]
                                
                                # Scale NED covariance to Lat/Lon degrees
                                var_lat = sigma_nn / (METERS_PER_DEGREE_LAT ** 2)
                                lon_scale = METERS_PER_DEGREE_LAT * math.cos(math.radians(t_lat))
                                var_lon = sigma_ee / (lon_scale ** 2)
                                cov_lat_lon = sigma_ne / (METERS_PER_DEGREE_LAT * lon_scale)
                                
                                # Incorporate YOLO confidence
                                conf = box.conf[0].item() if hasattr(box.conf[0], 'item') else float(box.conf[0])
                                conf = max(0.1, conf) 
                                conf_scale_factor = (1.0 - conf) * 10.0 + 1.0 
                                
                                R_cov = np.array([
                                    [var_lat * conf_scale_factor, cov_lat_lon * conf_scale_factor],
                                    [cov_lat_lon * conf_scale_factor, var_lon * conf_scale_factor]
                                ])
                            else:
                                R_cov = np.eye(2) * 1e-8
                            
                            # Predict and update with Mahalanobis gating
                            self.gps_kalman_filter.predict()
                            accepted = self.gps_kalman_filter.update(t_lat, t_lon, R_cov, max_mahalanobis=9.21)
                            
                            t_lat_filtered, t_lon_filtered = self.gps_kalman_filter.get_state()
                            
                            if not accepted:
                                SwarmLogger.log("WARNING", f"DRONE_{self.port}", f"GPS Outlier Rejected: lat={t_lat:.5f} lon={t_lon:.5f}", "KALMAN")
                            
                            # Use filtered GPS for swarm reporting
                            t_lat = t_lat_filtered
                            t_lon = t_lon_filtered

                            if self.swarm_manager:
                                conf = box.conf[0].item() if hasattr(box.conf[0], 'item') else box.conf[0]
                                local_track_id = None
                                if box.id is not None:
                                    local_track_id = int(box.id[0].item())

                                raw_data = {
                                    'drone_gps': (drone_lat, drone_lon, drone_alt),
                                    'heading': drone_heading,
                                    'attitude': (roll, pitch, yaw),
                                    'bbox': (center_x, center_y, w, h),
                                    'img_dims': (CAMERA_RESOLUTION_WIDTH, CAMERA_HEIGHT),
                                    'covariance': target_covariance.tolist() if target_covariance is not None else None,
                                    'is_low_confidence': is_low_confidence
                                }

                                # LOCKED state: heartbeat only
                                if self.lock_status == "LOCKED":
                                    heartbeat_data = {
                                        "action": "HEARTBEAT",
                                        "target_id": self.current_target_id,
                                        "local_tracker_id": self.assigned_local_tracker_id,
                                        "drone_gps": (drone_lat, drone_lon, drone_alt)
                                    }
                                    self.swarm_manager.report_target(
                                        self.port, 0, 0, 0, None, raw_data=heartbeat_data)
                                    found_assigned_target = True
                                    continue

                                # REQUESTING state: wait for ACK, skip normal reporting
                                elif self.lock_status == "REQUESTING":
                                    # Send heartbeat while waiting for lock approval
                                    heartbeat_data = {
                                        "action": "HEARTBEAT",
                                        "target_id": self.current_target_id,
                                        "local_tracker_id": self.assigned_local_tracker_id,
                                        "drone_gps": (drone_lat, drone_lon, drone_alt)
                                    }
                                    self.swarm_manager.report_target(
                                        self.port, 0, 0, 0, None, raw_data=heartbeat_data)
                                    continue

                                # Normal reporting
                                result = self.swarm_manager.report_target(
                                    self.port, t_lat, t_lon, conf,
                                    tracker_id=local_track_id, raw_data=raw_data,
                                    covariance=target_covariance
                                )

                                action = result.get("action")
                                target_id = result.get("target_id")
                                assigned_tracker_id = result.get("tracker_id")

                                found_assigned_target = self._handle_handshake(
                                    action, target_id, assigned_tracker_id,
                                    t_lat, t_lon, conf, local_track_id,
                                    drone_lat, drone_lon, drone_alt,
                                    found_assigned_target
                                )

                except Exception as e:
                    SwarmLogger.log("WARNING", f"DRONE_{self.port}", f"SWARM: Target calculation error: {e}", "TRACKING")

            # Group-level metadata enrichment
            group_info = tracker_result.get('group_info')
            if group_info and self.swarm_manager and hasattr(group_info, 'groups') and group_info.groups:
                try:
                    with self.swarm_manager.lock:
                        for grp in group_info.groups:
                            g_lat, g_lon = grp.center_gps
                            if g_lat is None or g_lon is None:
                                continue

                            # Multi-criteria target matching
                            best_tid = None
                            best_dist = 0.0
                            best_score = -1.0
                            second_best_score = -1.0
                            
                            for t_id, target in self.swarm_manager.targets.items():
                                if target.track_state == "DELETED":
                                    continue
                                
                                # Adaptive max_dist based on covariance
                                # Base: 30m, range: 15-40m (increased for better group matching)
                                adaptive_max_dist = 20.0
                                if target.covariance is not None and hasattr(target.covariance, 'shape') and target.covariance.shape[0] >= 2:
                                    try:
                                        cov_trace = float(target.covariance[0, 0] + target.covariance[1, 1])
                                        if cov_trace > 0:
                                            sigma = math.sqrt(cov_trace / 2.0)
                                            adaptive_max_dist = min(40.0, max(15.0, 3.5 * sigma))
                                    except (IndexError, TypeError):
                                        pass
                                
                                d = GeoMath.haversine_distance(g_lat, g_lon, target.lat, target.lon)
                                if d > adaptive_max_dist:
                                    continue
                                
                                # Calculate match score
                                score = 0.0
                                score += (adaptive_max_dist - d) / adaptive_max_dist * 10.0
                                
                                if target.track_state == "CONFIRMED":
                                    score += 5.0
                                elif target.track_state == "TENTATIVE":
                                    score += 2.0
                                
                                time_since_obs = time.time() - getattr(target, 'last_observation_time', 0)
                                if time_since_obs < 5.0:
                                    score += 3.0
                                elif time_since_obs < 10.0:
                                    score += 1.0
                                
                                if target.covariance is not None and hasattr(target.covariance, 'shape') and target.covariance.shape[0] >= 2:
                                    try:
                                        cov_trace = float(target.covariance[0, 0] + target.covariance[1, 1])
                                        if cov_trace > 0:
                                            score += min(2.0, 10.0 / (cov_trace ** 0.5))
                                    except (IndexError, TypeError):
                                        pass
                                
                                if score > best_score:
                                    second_best_score = best_score
                                    best_score = score
                                    best_dist = d
                                    best_tid = t_id
                                elif score > second_best_score:
                                    second_best_score = score

                            # Ambiguity guard - reduced threshold for more matches
                            # Only skip if truly ambiguous (< 10% difference)
                            if best_score > 0 and second_best_score > 0:
                                score_diff_ratio = (best_score - second_best_score) / best_score
                                if score_diff_ratio < 0.10:  # 10% threshold (was 20%)
                                    SwarmLogger.log("DEBUG", f"DRONE_{self.port}",
                                        f"GROUP {grp.group_id} — ambiguous match. Skipping.", "GROUP")
                                    continue

                            if best_tid and best_score > 2.0:
                                target = self.swarm_manager.targets[best_tid]
                                target.is_group = True
                                target.group_member_count = grp.member_count
                                target.group_id_local = grp.group_id

                                SwarmLogger.log("DEBUG", f"DRONE_{self.port}",
                                    f"GROUP {grp.group_id} (x{grp.member_count}) → enriched {best_tid}", "GROUP")
                            # else: Don't create new target - group singles already have their own targets
                except Exception as ge:
                    SwarmLogger.log("WARNING", f"DRONE_{self.port}",
                        f"Group enrichment error: {ge}", "GROUP")

            return found_assigned_target

        except Exception as e:
            SwarmLogger.log("ERROR", f"DRONE_{self.port}", f"Human detection check error: {e}", "TRACKING")

        return False

    def _handle_handshake(self, action, target_id, assigned_tracker_id,
                          t_lat, t_lon, conf, local_track_id,
                          drone_lat, drone_lon, drone_alt,
                          found_assigned_target):
        """Process swarm action response and execute handshake protocol."""

        if action == "ATTACK":
            SwarmLogger.log("INFO", "ATTACK",
                f"ATTACK received for {target_id}. Requesting Lock...",
                f"DRONE_{self.port}")

            req_data = {
                "action": "REQUEST_LOCK",
                "target_id": target_id,
                "drone_gps": (drone_lat, drone_lon, drone_alt)
            }
            res_lock = self.swarm_manager.report_target(
                self.port, t_lat, t_lon, conf,
                tracker_id=local_track_id, raw_data=req_data
            )

            if res_lock.get("action") == "ATTACK_APPROVED":
                self.lock_status = "APPROVED"
                action = "ATTACK_APPROVED"
            else:
                self.lock_status = "REQUESTING"

        if action == "ATTACK_APPROVED":
            SwarmLogger.log("INFO", "ATTACK",
                f"Lock APPROVED for {target_id}. Sending LOCK confirmation.",
                f"DRONE_{self.port}")

            lock_msg = {"action": "LOCKED", "target_id": target_id}
            self.swarm_manager.report_target(
                self.port, t_lat, t_lon, conf,
                tracker_id=local_track_id, raw_data=lock_msg
            )

            self.lock_status = "LOCKED"
            self.attack_approved = True
            self.current_target_id = target_id

            if assigned_tracker_id is not None:
                self.human_tracker.set_primary_target(assigned_tracker_id)

            SwarmLogger.log("SUCCESS", "ATTACK",
                f"LOCKED on {target_id}. Engaging...", f"DRONE_{self.port}")
            return True

        elif action == "HOVER":
            self.attack_approved = False
            self.lock_status = None
            return True

        elif action == "SEARCH":
            self.lock_status = None

        return found_assigned_target

    def execute_tracking_mode(self, vehicle, is_mission_active_check):
        """Main tracking loop with 7-step attack protocol and IBVS guidance."""
        self.tracking_mode = True
        target_port = self.port or self.drone_manager.active_drone_port

        SwarmLogger.log("STATE", f"DRONE_{target_port}",
            f"TRACKING MODE STARTED | Mode={vehicle.mode.name} | "
            f"Alt={vehicle.location.global_relative_frame.alt:.2f}m | "
            f"AttackApproved={self.attack_approved} | TargetID={self.current_target_id}")

        try:
            SwarmLogger.log("INFO", f"DRONE_{target_port}", "Entering tracking loop (tracker preserved)", "TRACKING")
        except Exception as e:
            SwarmLogger.log("ERROR", f"DRONE_{target_port}", f"Human tracker init error: {e}", "TRACKING")
            return

        try:
            loop_count = 0
            last_msg_time = time.time()
            last_reported_targets = {}
            last_leader_response = time.time()
            search_consecutive_count = 0
            attack_start_time = None

            # 7-Step Attack Protocol State
            attack_phase = None  # None → "ECHO" → "WAIT_CONFIRM" → "CENTERING" → "DIVING"
            phase_start_time = None
            center_confirm_count = 0

            while is_mission_active_check() and self.tracking_mode:
                loop_count += 1

                target_port = self.port or self.drone_manager.active_drone_port
                frame = self.camera_handler.get_latest_frame_as_array(target_port)

                if frame is None:
                    if loop_count % 20 == 0:
                        SwarmLogger.log("WARNING", f"DRONE_{target_port}",
                            f"Cannot get frame — loop #{loop_count}", "TRACKING")
                    time.sleep(0.05)
                    continue

                # Update tracker attitude and drone pose
                try:
                    attitude = vehicle.attitude
                    self.human_tracker.set_attitude(attitude.pitch, attitude.roll)
                    
                    loc = vehicle.location.global_relative_frame
                    if loc and loc.lat != 0:
                        self.human_tracker.set_drone_pose(
                            loc.lat, loc.lon, loc.alt,
                            attitude.roll, attitude.pitch, attitude.yaw
                        )
                except Exception as e:
                    if loop_count == 1:
                        SwarmLogger.log("WARNING", f"DRONE_{target_port}",
                            f"Attitude/pose read error: {e}", "TRACKING")

                # Run detection and tracking
                try:
                    result = self.human_tracker.detect_and_track(
                        frame, debug=True, attack_mode=self.attack_approved)

                    self._stream_annotated_frame(result, target_port, attack_phase, loop_count)

                except Exception as e:
                    SwarmLogger.log("ERROR", f"DRONE_{target_port}",
                        f"detect_and_track error: {e}", "TRACKING")
                    logging.error("detect_and_track error", exc_info=True)
                    break

                # Swarm command check
                if self.drone_manager.swarm_manager:
                    self._process_swarm_commands(
                        vehicle, result, target_port, loop_count,
                        attack_phase, phase_start_time, center_confirm_count,
                        last_leader_response, search_consecutive_count,
                        attack_start_time, last_msg_time, last_reported_targets,
                        _state := {
                            'attack_phase': attack_phase,
                            'phase_start_time': phase_start_time,
                            'center_confirm_count': center_confirm_count,
                            'last_leader_response': last_leader_response,
                            'search_consecutive_count': search_consecutive_count,
                            'attack_start_time': attack_start_time,
                            'last_msg_time': last_msg_time,
                            'should_break': False,
                            'should_continue': False
                        }
                    )

                    attack_phase = _state['attack_phase']
                    phase_start_time = _state['phase_start_time']
                    center_confirm_count = _state['center_confirm_count']
                    last_leader_response = _state['last_leader_response']
                    search_consecutive_count = _state['search_consecutive_count']
                    attack_start_time = _state['attack_start_time']
                    last_msg_time = _state['last_msg_time']

                    if _state['should_break']:
                        break
                    if _state['should_continue']:
                        continue

                # Velocity command generation
                self._generate_velocity_commands(
                    vehicle, result, frame, target_port, loop_count,
                    attack_phase, center_confirm_count, attack_start_time,
                    _vel_state := {
                        'attack_phase': attack_phase,
                        'center_confirm_count': center_confirm_count,
                        'phase_start_time': phase_start_time,
                        'should_break': False
                    }
                )

                attack_phase = _vel_state['attack_phase']
                center_confirm_count = _vel_state['center_confirm_count']
                phase_start_time = _vel_state['phase_start_time']
                if _vel_state['should_break']:
                    break

                time.sleep(0.01)

        except Exception as e:
            SwarmLogger.log("ERROR", f"DRONE_{target_port}",
                f"TRACKING MODE ERROR: {e}", "TRACKING")
            logging.error("TRACKING MODE ERROR", exc_info=True)
        finally:
            from config import RESUME_SCAN_AFTER_LOST

            target_port = self.port or self.drone_manager.active_drone_port
            SwarmLogger.log("STATE", f"DRONE_{target_port}",
                f"TRACKING ENDED | Frames={loop_count} | AttackApproved={self.attack_approved}")

            try:
                self.flight_ctrl.send_ned_velocity(vehicle, 0, 0, 0)
            except Exception:
                pass

            self.tracking_mode = False
            if not self.collision_detected and RESUME_SCAN_AFTER_LOST and is_mission_active_check():
                SwarmLogger.log("INFO", f"DRONE_{target_port}", "Resume scanning...")

            self.attack_approved = False
            self.target_gps_lat = None
            self.target_gps_lon = None

    def _stream_annotated_frame(self, result, target_port, attack_phase, loop_count):
        """Encode and push annotated frame to camera handler for web streaming."""
        if 'annotated_frame' not in result or result['annotated_frame'] is None:
            return

        annotated = result['annotated_frame'].copy()

        if result.get('track_id') is not None:
            cv2.putText(annotated, f"Track ID: {result['track_id']}", (10, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)

        if attack_phase:
            cv2.putText(annotated, f"PHASE: {attack_phase}", (10, 140),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        ret, buf = cv2.imencode('.jpg', annotated, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
        if ret:
            jpeg_bytes = buf.tobytes()
            with self.camera_handler.lock:
                if target_port in self.camera_handler.camera_feeds:
                    entry = self.camera_handler.camera_feeds[target_port]
                    if isinstance(entry, dict):
                        entry['jpeg'] = jpeg_bytes
                else:
                    self.camera_handler.camera_feeds[target_port] = {'raw': None, 'jpeg': jpeg_bytes}

    def _process_swarm_commands(self, vehicle, result, target_port, loop_count,
                                 attack_phase_unused, phase_start_time_unused,
                                 center_confirm_count_unused, last_leader_response_unused,
                                 search_consecutive_count_unused, attack_start_time_unused,
                                 last_msg_time_unused, last_reported_targets, state):
        """Process detected targets and send to swarm manager, handle protocol responses."""
        candidates = result.get('detected_candidates', [])
        group_info = result.get('group_info')  # GroupResult with groups and singles

        d_loc = vehicle.location.global_relative_frame
        if not d_loc:
            return

        d_lat, d_lon, d_alt = d_loc.lat, d_loc.lon, d_loc.alt
        d_head = vehicle.heading

        # Build group membership lookup
        group_membership = {}
        if group_info and hasattr(group_info, 'groups') and group_info.groups:
            for grp in group_info.groups:
                for member_id in grp.member_ids:
                    group_membership[member_id] = (True, grp.member_count, grp.group_id)

        # Build multi-target report
        all_reports = []
        if candidates:
            for c in candidates:
                c_w, c_h = c['dims']
                c_cx, c_cy = c['center']

                rgi_result = GeoMath.ray_ground_intersection(
                    d_lat, d_lon, d_alt,
                    vehicle.attitude.roll, vehicle.attitude.pitch, vehicle.attitude.yaw,
                    c_cx, c_cy
                )

                t_lat, t_lon = None, None
                t_cov = None
                if rgi_result is not None and len(rgi_result) >= 2 and rgi_result[0] is not None:
                    t_lat, t_lon = rgi_result[0], rgi_result[1]
                    if len(rgi_result) > 2:
                        t_cov = rgi_result[2]

                if t_lat is not None:
                    c_id = c.get('id')
                    if c_id is not None:
                        last_reported_targets[c_id] = (t_lat, t_lon)

                    # Check if candidate is part of a group
                    is_group = False
                    group_member_count = 0
                    group_id = None
                    if c_id is not None and c_id in group_membership:
                        is_group, group_member_count, group_id = group_membership[c_id]

                    t_data = {
                        'track_id': c_id,
                        'bbox': (c_cx, c_cy, c_w, c_h),
                        'confidence': c['conf'],
                        'world_xyz': (t_lat, t_lon, 0.0),
                        'covariance': t_cov.tolist() if hasattr(t_cov, 'tolist') else t_cov,
                        'drone_gps': (d_lat, d_lon, d_alt),
                        'heading': d_head,
                        'attitude': (vehicle.attitude.roll, vehicle.attitude.pitch, vehicle.attitude.yaw),
                        'img_dims': (CAMERA_RESOLUTION_WIDTH, CAMERA_HEIGHT),
                        'is_group': is_group,
                        'group_member_count': group_member_count,
                        'group_id': group_id
                    }
                    all_reports.append(t_data)

        # Phase-specific packet handling
        if state['attack_phase'] == "CENTERING" and self.current_target_id:
            locked_t = [r for r in all_reports if r['track_id'] == self.assigned_local_tracker_id]
            if locked_t:
                world_xyz = locked_t[0].get('world_xyz', (None, None, None))
                locked_data = {
                    "action": "LOCKED_DATA",
                    "target_id": self.current_target_id,
                    "drone_gps": (d_lat, d_lon, d_alt),
                    "target_gps": (world_xyz[0], world_xyz[1]) if world_xyz[0] is not None else None,
                    "bbox": locked_t[0].get('bbox')
                }
                self.drone_manager.swarm_manager.check_mission_updates(
                    self.port, locked_data)

        elif state['attack_phase'] == "DIVING":
            if all_reports:
                packet = {
                    "drone_id": self.port,
                    "timestamp": time.time(),
                    "locked_target_id": self.current_target_id,
                    "detected_targets": all_reports,
                    "state": "DIVING"
                }
                self.drone_manager.swarm_manager.check_mission_updates(self.port, packet)

        else:
            # Normal packet flow
            packet = {
                "drone_id": self.port,
                "timestamp": time.time(),
                "locked_target_id": self.current_target_id,
                "detected_targets": all_reports,
                "state": self.drone_manager.mission_status_message
            }

            resp = self.drone_manager.swarm_manager.check_mission_updates(self.port, packet)
            
            # Robust parsing for dict, tuple, or string returns from SwarmCoordinator
            if isinstance(resp, dict):
                sw_action = resp.get("action", "SEARCH")
                sw_target_id = resp.get("target_id")
                sw_assigned_local_id = resp.get("tracker_id")
                sw_lat = resp.get("lat")
                sw_lon = resp.get("lon")
            elif isinstance(resp, tuple) and len(resp) >= 3:
                sw_action, sw_target_id, sw_assigned_local_id = resp[0], resp[1], resp[2]
                sw_lat = resp[3] if len(resp) > 3 else None
                sw_lon = resp[4] if len(resp) > 4 else None
            elif isinstance(resp, str):
                sw_action = resp
                sw_target_id = None
                sw_assigned_local_id = None
                sw_lat = None
                sw_lon = None

            # Dual-factor geo-validation
            if sw_action in ("ATTACK", "CENTER", "ATTACK_ASSIGN", "REASSIGN") and sw_lat is not None and sw_lon is not None:
                locked_t = next((r for r in all_reports if r['track_id'] == sw_assigned_local_id), None)
                if locked_t:
                    world_xyz = locked_t.get('world_xyz', (None, None, None))
                    t_lat, t_lon = world_xyz[0], world_xyz[1]
                    if t_lat is not None and t_lon is not None:
                        dist = GeoMath.haversine_distance(t_lat, t_lon, sw_lat, sw_lon)
                        if dist > SWARM_GEO_VALIDATION_THRESHOLD_M:
                            SwarmLogger.log("MISMATCH", f"DRONE_{self.port}", 
                                f"Geo-Validation FAILED! Target {sw_target_id} is {dist:.1f}m away. Dropping lock.")
                            sw_action = "MISMATCH"
                            mismatch_packet = {"action": "MISMATCH", "target_id": sw_target_id}
                            self.drone_manager.swarm_manager.check_mission_updates(self.port, mismatch_packet)
            self._handle_protocol_action(
                vehicle, sw_action, sw_target_id, sw_assigned_local_id,
                target_port, loop_count, result, d_lat, d_lon, d_alt, state, all_reports)

        # Phase timeouts
        self._check_phase_timeouts(target_port, state)

        # Leader timeout failsafe
        self._check_leader_timeout(target_port, state)

    def _handle_protocol_action(self, vehicle, sw_action, sw_target_id,
                                  sw_assigned_local_id, target_port, loop_count,
                                  result, d_lat, d_lon, d_alt, state, all_reports):
        """Handle 7-step protocol actions from swarm manager."""

        if sw_action == "ATTACK_ASSIGN":
            state['last_leader_response'] = time.time()
            state['search_consecutive_count'] = 0

            if state['attack_phase'] is None:
                self.current_target_id = sw_target_id
                if sw_assigned_local_id is not None:
                    self.assigned_local_tracker_id = sw_assigned_local_id
                    self.human_tracker.set_primary_target(sw_assigned_local_id)
                    self.ibvs_guidance.set_target_id(sw_assigned_local_id)

                state['attack_phase'] = "ECHO"
                state['phase_start_time'] = time.time()

                SwarmLogger.log("PROTOCOL", f"DRONE_{target_port}",
                    f"Step 2: ECHO — Target={sw_target_id} LocalID={sw_assigned_local_id}",
                    "ATTACK_FLOW")

                # Send echo back
                echo_packet = {
                    "action": "ECHO_TARGET",
                    "target_id": sw_target_id,
                    "drone_gps": (d_lat, d_lon, d_alt),
                    "local_tracker_id": sw_assigned_local_id,
                    "bbox": result.get('bbox')
                }
                echo_resp = self.drone_manager.swarm_manager.check_mission_updates(
                    self.port, echo_packet)

                echo_action = echo_resp[0] if isinstance(echo_resp, tuple) else echo_resp
                echo_lat = echo_resp[3] if isinstance(echo_resp, tuple) and len(echo_resp) > 3 else None
                echo_lon = echo_resp[4] if isinstance(echo_resp, tuple) and len(echo_resp) > 4 else None

                # Geo-validation for ECHO
                if echo_action in ("ATTACK_CONFIRMED", "REASSIGN", "CENTER") and echo_lat is not None and echo_lon is not None:
                    locked_t = next((r for r in all_reports if r['track_id'] == sw_assigned_local_id), None)
                    if locked_t:
                        world_xyz = locked_t.get('world_xyz', (None, None, None))
                        t_lat, t_lon = world_xyz[0], world_xyz[1]
                        if t_lat is not None and t_lon is not None:
                            dist = GeoMath.haversine_distance(t_lat, t_lon, echo_lat, echo_lon)
                            if dist > SWARM_GEO_VALIDATION_THRESHOLD_M:
                                SwarmLogger.log("MISMATCH", f"DRONE_{self.port}", 
                                    f"Geo-Validation FAILED during ECHO! Target is {dist:.1f}m off.")
                                echo_action = "MISMATCH"
                                mismatch_packet = {"action": "MISMATCH", "target_id": sw_target_id}
                                self.drone_manager.swarm_manager.check_mission_updates(self.port, mismatch_packet)

                if echo_action == "ATTACK_CONFIRMED":
                    state['attack_phase'] = "CENTERING"
                    state['phase_start_time'] = time.time()
                    state['center_confirm_count'] = 0
                    self.attack_approved = True
                    state['attack_start_time'] = time.time()
                    SwarmLogger.log("PROTOCOL", f"DRONE_{target_port}",
                        f"Step 5→6: CONFIRMED → CENTERING | Target={sw_target_id}",
                        "ATTACK_FLOW")
                elif echo_action == "REASSIGN":
                    new_target_id = echo_resp[1] if isinstance(echo_resp, tuple) else None
                    new_local_id = echo_resp[2] if isinstance(echo_resp, tuple) and len(echo_resp) > 2 else None
                    self.current_target_id = new_target_id
                    if new_local_id:
                        self.assigned_local_tracker_id = new_local_id
                        self.human_tracker.set_primary_target(new_local_id)
                        self.ibvs_guidance.set_target_id(new_local_id)
                    state['attack_phase'] = None
                elif echo_action == "SEARCH":
                    state['attack_phase'] = None
                    self.current_target_id = None
                else:
                    state['attack_phase'] = "WAIT_CONFIRM"
                    state['phase_start_time'] = time.time()

        elif sw_action == "CENTER":
            state['last_leader_response'] = time.time()
            state['search_consecutive_count'] = 0
            
            # Mid-Attack Guard
            if self.attack_approved and state.get('attack_phase') in ("CENTERING", "DIVING", "CONFIRMED_ATTACK"):
                if sw_target_id is not None and self.current_target_id is not None and sw_target_id != self.current_target_id:
                    SwarmLogger.log("GUARD", f"DRONE_{self.port}", f"Center Ignored: Mid-attack ID swap blocked.", "ATTACK_FLOW")
                    sw_target_id = self.current_target_id
            
            if state['attack_phase'] in ("WAIT_CONFIRM", "ECHO", None):
                state['attack_phase'] = "CENTERING"
                state['phase_start_time'] = time.time()
                state['center_confirm_count'] = 0
                self.attack_approved = True
                state['attack_start_time'] = time.time()
                self.current_target_id = sw_target_id
                if sw_assigned_local_id is not None:
                    self.assigned_local_tracker_id = sw_assigned_local_id
                    self.human_tracker.set_primary_target(sw_assigned_local_id)
                    self.ibvs_guidance.set_target_id(sw_assigned_local_id)

        elif sw_action == "ATTACK":
            state['last_leader_response'] = time.time()
            state['search_consecutive_count'] = 0
            
            # Mid-Attack Guard
            is_active_attack = self.attack_approved and state.get('attack_phase') in ("CENTERING", "DIVING", "CONFIRMED_ATTACK")
            if is_active_attack and self.current_target_id is not None and sw_target_id is not None and sw_target_id != self.current_target_id:
                SwarmLogger.log("GUARD", f"DRONE_{self.port}", f"Attack Ignored: Mid-attack ID swap blocked.", "ATTACK_FLOW")
            else:
                if sw_target_id is not None:
                    self.current_target_id = sw_target_id
                
            if not self.attack_approved:
                state['attack_start_time'] = time.time()
                self.attack_approved = True
                state['attack_phase'] = "DIVING"
                state['phase_start_time'] = time.time()
                if sw_assigned_local_id is not None:
                    self.human_tracker.set_primary_target(sw_assigned_local_id)
                    self.assigned_local_tracker_id = sw_assigned_local_id
                    self.ibvs_guidance.set_target_id(sw_assigned_local_id)
            else:
                if sw_assigned_local_id is not None and sw_assigned_local_id != self.assigned_local_tracker_id:
                    self.assigned_local_tracker_id = sw_assigned_local_id
                    self.ibvs_guidance.set_target_id(sw_assigned_local_id)

        elif sw_action == "HOVER":
            state['last_leader_response'] = time.time()
            state['search_consecutive_count'] = 0
            
            if sw_target_id is not None:
                self.current_target_id = sw_target_id
                
            if sw_assigned_local_id is not None:
                self.assigned_local_tracker_id = sw_assigned_local_id
                self.human_tracker.set_primary_target(sw_assigned_local_id)

        elif sw_action == "TRACK":
            state['last_leader_response'] = time.time()
            state['search_consecutive_count'] = 0
            
            # Mid-Attack Guard for TRACK
            is_active_attack = self.attack_approved and state.get('attack_phase') in ("CENTERING", "DIVING", "CONFIRMED_ATTACK")
            if is_active_attack:
                SwarmLogger.log("GUARD", f"DRONE_{self.port}", f"Track Ignored: Cannot switch to passive track while engaging.", "ATTACK_FLOW")
            else:
                self.attack_approved = False
                state['attack_phase'] = None
                
                if sw_target_id is not None:
                    self.current_target_id = sw_target_id
                    
                if sw_assigned_local_id is not None:
                    self.assigned_local_tracker_id = sw_assigned_local_id
                    self.human_tracker.set_primary_target(sw_assigned_local_id)
                    self.ibvs_guidance.set_target_id(sw_assigned_local_id)

        elif sw_action == "SEARCH":
            from config import SWARM_IMMUTABLE_TIMEOUT_S
            state['last_leader_response'] = time.time()

            is_active_attack = self.attack_approved and state.get('attack_phase') in ("CENTERING", "DIVING", "CONFIRMED_ATTACK")
            if not is_active_attack:
                self.current_target_id = None
                self.assigned_local_tracker_id = None
                self.human_tracker.set_primary_target(None)

            if self.attack_approved and state['attack_phase'] in ("CENTERING", "DIVING", "CONFIRMED_ATTACK"):
                state['search_consecutive_count'] += 1
                if loop_count % 30 == 0:
                    SwarmLogger.log("GUARD", f"DRONE_{target_port}",
                        f"SEARCH ignored — phase={state['attack_phase']}. Frames: {state['search_consecutive_count']}",
                        "ATTACK_FLOW")
                
                # Failsafe: abort zombie lock if target lost too long
                if state['search_consecutive_count'] >= (30 * SWARM_IMMUTABLE_TIMEOUT_S):
                    SwarmLogger.log("TIMEOUT", f"DRONE_{target_port}",
                        f"Immutable Lock Failsafe: Target lost for {SWARM_IMMUTABLE_TIMEOUT_S}s",
                        "ATTACK_FLOW")
                    self.attack_approved = False
                    self.current_target_id = None
                    self.assigned_local_tracker_id = None
                    self.human_tracker.set_primary_target(None)
                    self.drone_manager.mission_status_message = "TARGET BURNT - IMMUTABLE LOCK ABORTED"
                    state['should_break'] = True
                else:
                    state['should_continue'] = True
                return
                
            elif self.attack_approved:
                state['should_continue'] = True
                return

            state['search_consecutive_count'] += 1
            # Wait 45 frames to avoid premature canceling
            if state['search_consecutive_count'] >= 45:
                if self.attack_approved and state.get('attack_phase') in ("CENTERING", "DIVING", "CONFIRMED_ATTACK"):
                    SwarmLogger.log("STATE", f"DRONE_{target_port}",
                        f"45 consecutive SEARCH in attack phase — Fallback HOVER.")
                    self.drone_manager.mission_status_message = "FALLBACK HOVER - RETRYING LOCK"
                    state['should_continue'] = True
                    return
                else:
                    SwarmLogger.log("STATE", f"DRONE_{target_port}",
                        f"45 consecutive SEARCH — Ending Tracking.")
                    self.attack_approved = False
                    self.current_target_id = None
                    self.assigned_local_tracker_id = None
                    self.human_tracker.set_primary_target(None)
                    self.drone_manager.mission_status_message = "TARGET CANCELLED - RETURNING TO SCAN"
                    state['should_break'] = True

        elif sw_action == "STOP":
            self.attack_approved = False
            state['attack_phase'] = None
            self.current_target_id = None
            self.assigned_local_tracker_id = None
            self.human_tracker.set_primary_target(None)

        elif sw_action == "ABORT" or sw_action == "MISMATCH":
            self.attack_approved = False
            state['attack_phase'] = None
            self.current_target_id = None
            self.assigned_local_tracker_id = None
            self.human_tracker.set_primary_target(None)
            state['should_break'] = True

        elif sw_action == "REASSIGN":
            if self.attack_approved and state['attack_phase'] in ("CENTERING", "DIVING", "CONFIRMED_ATTACK"):
                SwarmLogger.log("GUARD", f"DRONE_{target_port}",
                    f"REASSIGN to {sw_target_id} ignored — Immutable Lock Active on phase {state['attack_phase']}",
                    "ATTACK_FLOW")
            else:
                self.current_target_id = sw_target_id
                self.assigned_local_tracker_id = sw_assigned_local_id
                self.human_tracker.set_primary_target(sw_assigned_local_id)
                self.ibvs_guidance.set_target_id(sw_assigned_local_id)
                state['attack_phase'] = None

    def _check_phase_timeouts(self, target_port, state):
        """Check for phase-specific timeouts (zombie guard)."""
        if state['attack_phase'] == "ECHO" and state['phase_start_time']:
            if time.time() - state['phase_start_time'] > ATTACK_ECHO_TIMEOUT_S:
                SwarmLogger.log("TIMEOUT", f"DRONE_{target_port}",
                    f"ECHO phase timeout ({ATTACK_ECHO_TIMEOUT_S}s)", "ATTACK_FLOW")
                self.attack_approved = False
                state['attack_phase'] = None
                state['should_break'] = True

        if state['attack_phase'] == "WAIT_CONFIRM" and state['phase_start_time']:
            if time.time() - state['phase_start_time'] > ATTACK_ECHO_TIMEOUT_S:
                SwarmLogger.log("TIMEOUT", f"DRONE_{target_port}",
                    f"WAIT_CONFIRM timeout ({ATTACK_ECHO_TIMEOUT_S}s)", "ATTACK_FLOW")
                self.attack_approved = False
                state['attack_phase'] = None
                state['should_break'] = True

        if state['attack_phase'] == "CENTERING" and state['phase_start_time']:
            if time.time() - state['phase_start_time'] > ATTACK_CENTER_TIMEOUT_S:
                SwarmLogger.log("TIMEOUT", f"DRONE_{target_port}",
                    f"CENTERING timeout — forcing dive", "ATTACK_FLOW")
                state['attack_phase'] = "DIVING"
                state['phase_start_time'] = time.time()

    def _check_leader_timeout(self, target_port, state):
        """Check for leader timeout failsafe."""
        if state['attack_phase'] not in ("CENTERING", "DIVING"):
            if time.time() - state['last_leader_response'] > LEADER_TIMEOUT_S:
                if time.time() - state['last_msg_time'] > 5.0:
                    SwarmLogger.log("WARNING", f"DRONE_{target_port}",
                        f"LEADER TIMEOUT ({LEADER_TIMEOUT_S}s) — Failsafe Active")
                    state['last_msg_time'] = time.time()

                if not self.attack_approved:
                    self.drone_manager.mission_status_message = "FAILSAFE: HOVER — No Leader Connection"

                state['last_leader_response'] = time.time()

    def _generate_velocity_commands(self, vehicle, result, frame, target_port,
                                      loop_count, attack_phase, center_confirm_count,
                                      attack_start_time, state):
        """Generate and send velocity commands based on detection results and attack phase."""
        if not result['detected']:
            self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["HUMAN_LOST"]
            vel_cmd = result['velocity_cmd']
            vx, vy, vz = vel_cmd['vx'], vel_cmd['vy'], vel_cmd['vz']
            yaw_rate = vel_cmd.get('yaw_rate', 0.0)

            if not self.attack_approved:
                vx, yaw_rate = 0.0, 0.0

            try:
                self.flight_ctrl.safe_track_velocity(
                    vehicle, vx, vy, vz, yaw_rate, self.attack_approved)
            except Exception:
                pass
            return

        coverage = result['screen_coverage'] * 100

        # Observer yaw control (pre-attack)
        if not self.attack_approved:
            candidates = result.get('detected_candidates', [])
            if candidates:
                sum_cx = sum([c['center'][0] for c in candidates])
                avg_cx = sum_cx / len(candidates)
                frame_width = frame.shape[1] if frame is not None else CAMERA_RESOLUTION_WIDTH
                center_x = frame_width / 2
                error_x = avg_cx - center_x
                DEAD_ZONE = 40
                yaw_cmd = 0.0

                if abs(error_x) > DEAD_ZONE:
                    k_yaw = 0.0020
                    yaw_cmd = error_x * k_yaw
                    yaw_cmd = max(min(yaw_cmd, 0.5), -0.5)

                self.drone_manager.mission_status_message = f"OBSERVER ({len(candidates)} Targets)"

            self.drone_manager.mission_status_message = f"Tracking - Waiting Approval ({coverage:.1f}%)"
            vel_cmd = result['velocity_cmd']
            vx = 0.0
            vy = vel_cmd.get('vy', 0.0)
            yaw_rate = vel_cmd.get('yaw_rate', 0.0)
            if abs(yaw_rate) < 0.05:
                yaw_rate = 0.0

            try:
                self.flight_ctrl.send_ned_velocity_with_yaw_rate(vehicle, vx, vy, 0.0, yaw_rate)
            except Exception:
                pass
            return

        # Attack mode
        self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["TRACKING_HUMAN"].format(coverage=coverage)

        if result['collision_imminent'] and not self.attack_approved:
            self.collision_detected = True

        # Attack timeout
        if self.attack_approved and attack_start_time is not None:
            if (time.time() - attack_start_time) > ATTACK_TIMEOUT_S:
                SwarmLogger.log("TIMEOUT", f"DRONE_{target_port}",
                    f"ATTACK TIMEOUT ({ATTACK_TIMEOUT_S}s) — Aborting", "ATTACK_FLOW")
                state['should_break'] = True
                return

        # IBVS Guidance
        current_track_id = result.get('track_id')
        bbox = result.get('bbox')
        vx, vy, vz, yaw_rate = 0.0, 0.0, 0.0, 0.0

        # Only process locked target in attack mode
        if self.attack_approved and self.assigned_local_tracker_id is not None:
            if current_track_id is not None and int(current_track_id) != int(self.assigned_local_tracker_id):
                SwarmLogger.log("GUARD", f"DRONE_{target_port}",
                    f"IBVS blocked: track_id {current_track_id} != locked {self.assigned_local_tracker_id}",
                    "ATTACK_FLOW")
                bbox = None
                current_track_id = self.assigned_local_tracker_id

        if bbox and len(bbox) == 4:
            x1, y1, x2, y2 = bbox
            bbox_center = ((x1 + x2) / 2, (y1 + y2) / 2)
            bbox_size = (x2 - x1, y2 - y1)
            frame_size = (CAMERA_RESOLUTION_WIDTH, CAMERA_HEIGHT)

            if state['attack_phase'] == "CENTERING":
                center_x = frame_size[0] / 2
                error_x_norm = (bbox_center[0] - center_x) / center_x

                # Initialize yaw filter if needed
                if self.centering_yaw_filter is None:
                    from modules.mission.ibvs_guidance import LowPassFilter
                    self.centering_yaw_filter = LowPassFilter(alpha=CENTERING_YAW_FILTER_ALPHA)

                # Deadzone to prevent oscillation
                if abs(error_x_norm) < CENTERING_YAW_DEADZONE:
                    error_x_norm = 0.0

                # Check if centered
                if abs(error_x_norm) < ATTACK_CENTER_ERROR_THRESHOLD:
                    state['center_confirm_count'] += 1
                    yaw_rate = 0.0
                    self.centering_yaw_filter.reset()
                else:
                    state['center_confirm_count'] = 0
                    # Reduced yaw gain with low-pass filter for stability
                    yaw_rate_raw = error_x_norm * CENTERING_YAW_KP
                    yaw_rate = self.centering_yaw_filter.update(yaw_rate_raw)
                    yaw_rate = max(min(yaw_rate, CENTERING_YAW_RATE_MAX), -CENTERING_YAW_RATE_MAX)

                vx, vy, vz = 0.0, 0.0, 0.0

                # Require settle frames after reaching threshold
                if state['center_confirm_count'] >= ATTACK_CENTER_CONFIRM_FRAMES:
                    self.centering_settle_count += 1
                    if self.centering_settle_count >= CENTERING_SETTLE_FRAMES:
                        SwarmLogger.log("PROTOCOL", f"DRONE_{target_port}",
                            f"Step 7: TARGET CENTERED → DIVING! (settled {self.centering_settle_count} frames)", "ATTACK_FLOW")

                        if self.drone_manager.swarm_manager and self.current_target_id:
                            attacking_msg = {"action": "ATTACKING", "target_id": self.current_target_id}
                            self.drone_manager.swarm_manager.check_mission_updates(
                                self.port, attacking_msg)

                        state['attack_phase'] = "DIVING"
                        state['phase_start_time'] = time.time()
                        self.phase_transition_start = time.time()  # For transition buffer
                        self.centering_settle_count = 0
                    else:
                        # Still settling, hold position
                        self.drone_manager.mission_status_message = \
                            f"CENTERING - SETTLING ({self.centering_settle_count}/{CENTERING_SETTLE_FRAMES})"
                else:
                    self.centering_settle_count = 0
                    self.drone_manager.mission_status_message = \
                        f"CENTERING ({state['center_confirm_count']}/{ATTACK_CENTER_CONFIRM_FRAMES})"

            elif state['attack_phase'] == "DIVING" or state['attack_phase'] is None:
                # Phase transition buffer - hold briefly when entering DIVING
                if self.phase_transition_start is not None:
                    transition_elapsed = time.time() - self.phase_transition_start
                    if transition_elapsed < CENTERING_TO_DIVING_HOLD_S:
                        # Hold position during transition
                        vx, vy, vz, yaw_rate = 0.0, 0.0, 0.0, 0.0
                        self.drone_manager.mission_status_message = "TRANSITION → DIVING"
                        if loop_count % 10 == 0:
                            SwarmLogger.log("TRANSITION", f"DRONE_{target_port}",
                                f"Holding before dive: {transition_elapsed:.2f}s", "ATTACK_FLOW")
                    else:
                        self.phase_transition_start = None  # Transition complete

                if self.phase_transition_start is None:
                    try:
                        drone_pitch = vehicle.attitude.pitch
                    except Exception:
                        drone_pitch = 0.0
                    
                    vx, vy, vz, yaw_rate = self.ibvs_guidance.compute_velocity(
                        bbox_center=bbox_center, bbox_size=bbox_size,
                        frame_size=frame_size, current_tracker_id=current_track_id,
                        drone_pitch=drone_pitch)
                    vx, vy, vz = self.velocity_smoother.smooth(vx, vy, vz)

                    # Deadlock guard - gentle descent
                    current_alt = vehicle.location.global_relative_frame.alt or 0
                    if abs(vx) < 0.3 and coverage < 5.0:
                        if current_alt > IBVS_ALTITUDE_FLOOR_M:
                            vz = min(0.5, IBVS_SHALLOW_DIVE_VZ_MAX)
                        else:
                            vx = IBVS_VX_MIN
                            vz = 0.0

                    # Altitude floor
                    if current_alt <= IBVS_ALTITUDE_FLOOR_M and vz > 0:
                        vz = 0.0

                    # ========== IMPACT DETECTION ==========
                    # Progressive altitude-based coverage threshold
                    if IMPACT_ALTITUDE_PROGRESSIVE and current_alt <= IMPACT_ALTITUDE_LOW_M:
                        # Low altitude: reduce coverage requirement
                        effective_coverage_threshold = IMPACT_COVERAGE_THRESHOLD * 0.5
                    elif IMPACT_ALTITUDE_PROGRESSIVE and current_alt <= IMPACT_ALTITUDE_HIGH_M:
                        # Medium altitude: linear interpolation
                        alt_ratio = (current_alt - IMPACT_ALTITUDE_LOW_M) / (IMPACT_ALTITUDE_HIGH_M - IMPACT_ALTITUDE_LOW_M)
                        effective_coverage_threshold = IMPACT_COVERAGE_THRESHOLD * (0.5 + 0.5 * alt_ratio)
                    else:
                        effective_coverage_threshold = IMPACT_COVERAGE_THRESHOLD
                    
                    # Check for impact conditions
                    if coverage >= effective_coverage_threshold:
                        self.impact_confirm_count += 1
                        if self.impact_confirm_count >= IMPACT_CONFIRM_FRAMES:
                            if not self.impact_detected:
                                self.impact_detected = True
                                SwarmLogger.log("SUCCESS", f"DRONE_{target_port}",
                                    f"IMPACT CONFIRMED! Coverage={coverage:.1f}% (thresh={effective_coverage_threshold:.1f}%) Alt={current_alt:.1f}m",
                                    "ATTACK_FLOW")
                                self.drone_manager.mission_status_message = "MISSION COMPLETE - TARGET ENGAGED"
                                # Notify swarm manager
                                if self.drone_manager.swarm_manager and self.current_target_id:
                                    impact_msg = {
                                        "action": "IMPACT_CONFIRMED",
                                        "target_id": self.current_target_id,
                                        "coverage": coverage,
                                        "altitude": current_alt
                                    }
                                    self.drone_manager.swarm_manager.check_mission_updates(
                                        self.port, impact_msg)
                    elif current_alt <= IMPACT_ALTITUDE_MIN_M:
                        # Ground impact detection
                        self.impact_confirm_count += 1
                        if self.impact_confirm_count >= IMPACT_CONFIRM_FRAMES:
                            if not self.impact_detected:
                                self.impact_detected = True
                                SwarmLogger.log("SUCCESS", f"DRONE_{target_port}",
                                    f"GROUND IMPACT! Alt={current_alt:.1f}m",
                                    "ATTACK_FLOW")
                                self.drone_manager.mission_status_message = "MISSION COMPLETE - GROUND IMPACT"
                    else:
                        # Reset counter if conditions not met
                        if self.impact_confirm_count > 0:
                            self.impact_confirm_count = max(0, self.impact_confirm_count - 1)

                    if loop_count % 10 == 0:
                        impact_status = " [IMPACT]" if self.impact_detected else ""
                        SwarmLogger.log("IBVS_CMD", f"DRONE_{target_port}",
                            f"vx={vx:.2f} vz={vz:.2f} yaw={yaw_rate:.2f} pitch={math.degrees(drone_pitch):.1f}° | "
                            f"alt={current_alt:.1f}m cov={coverage:.1f}%{impact_status}",
                            "ATTACK_FLOW")
            else:
                vx, vy, vz, yaw_rate = 0.0, 0.0, 0.0, 0.0
        else:
            vx, vy, vz, yaw_rate = self.ibvs_guidance.compute_velocity(
                None, None, (CAMERA_RESOLUTION_WIDTH, CAMERA_HEIGHT), None)

        # Send final velocity command
        try:
            vz = max(0.0, min(vz, IBVS_VZ_MAX))
            if self.attack_approved:
                self.flight_ctrl.send_ned_velocity_with_yaw_rate(vehicle, vx, vy, vz, yaw_rate)
            else:
                self.flight_ctrl.safe_track_velocity(vehicle, vx, vy, vz, yaw_rate, self.attack_approved)
        except Exception as e:
            SwarmLogger.log("ERROR", f"DRONE_{target_port}", f"Velocity command failed: {e}")
