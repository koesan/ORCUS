"""Human Tracking Module - YOLOv12 + BoT-SORT for detection and tracking."""

import sys
import os
import cv2
import math
import time
import torch
import numpy as np
from typing import Tuple, Dict, List
import functools
import threading
from modules.core.pid_controller import FilteredPID
from modules.core.logger import SwarmLogger

from config import (
    BoTSORTConfig,
    COLLISION_FORWARD_SPEED, 
    COLLISION_SCREEN_THRESHOLD, 
    HUMAN_LOST_TIMEOUT,
    TRACKING_YAW_PID_TAU, TRACKING_YAW_PID_KP, TRACKING_YAW_PID_KI, TRACKING_YAW_PID_KD,
    TRACKING_YAW_PID_INT_MAX, TRACKING_YAW_PID_INT_MIN, TRACKING_YAW_PID_OUT_MAX, TRACKING_YAW_PID_OUT_MIN,
    TRACKING_PITCH_PID_TAU, TRACKING_PITCH_PID_KP, TRACKING_PITCH_PID_KI, TRACKING_PITCH_PID_KD,
    TRACKING_PITCH_PID_INT_MAX, TRACKING_PITCH_PID_INT_MIN, TRACKING_PITCH_PID_OUT_MAX, TRACKING_PITCH_PID_OUT_MIN,
    CAMERA_FX, CAMERA_WIDTH, CAMERA_RESOLUTION_WIDTH,
    TRACKING_TARGET_OFFSET_Y_RATIO, TRACKING_SCREEN_OFFSET_Y_RATIO,
    TRACKING_LATERAL_CORRECTION_GAIN, TRACKING_VERTICAL_FALLBACK_GAIN,
    APPROACH_MIN_SCREEN_COVERAGE, APPROACH_BOOST_SPEED,
    TRACKER_FRAME_RATE, DEBUG_FONT_SCALE, DEBUG_FONT_THICKNESS,
    COLOR_PRIMARY, COLOR_SECONDARY,
    GROUP_VISUAL_COLOR_BGR,
    IOU_THRESHOLD
)
from modules.core.logger import SwarmLogger
from modules.vision.group_tracker import GroupClusterEngine, GroupResult

# Monkey-patch torch.load for legacy weights compatibility
_original_torch_load = torch.load
def _safe_legacy_load(*args, **kwargs):
    if 'weights_only' not in kwargs:
        kwargs['weights_only'] = False
    return _original_torch_load(*args, **kwargs)
torch.load = _safe_legacy_load


# Path setup
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
TRACKER_ROOT = os.path.join(PROJECT_ROOT, "modules", "vision", "tracker")
YOLO_ENGINE_ROOT = os.path.join(PROJECT_ROOT, "YOLOv12-BoT-SORT-ReID-main", "BoT-SORT")

if TRACKER_ROOT not in sys.path:
    sys.path.insert(0, TRACKER_ROOT)

if YOLO_ENGINE_ROOT not in sys.path:
    sys.path.append(YOLO_ENGINE_ROOT)
if os.path.join(YOLO_ENGINE_ROOT, "yolov12") not in sys.path:
    sys.path.append(os.path.join(YOLO_ENGINE_ROOT, "yolov12"))

try:
    from ultralytics import YOLO
except ImportError as e:
    SwarmLogger.log("ERROR", "HumanTracker", f"CRITICAL YOLO IMPORT ERROR: {e}", "IMPORT")
    YOLO = None

try:
    import termcolor
except ImportError:
    from types import ModuleType
    m = ModuleType("termcolor")
    m.cprint = lambda *args, **kwargs: None
    m.colored = lambda text, *args, **kwargs: text
    sys.modules["termcolor"] = m

try:
    from modules.vision.tracker.mc_bot_sort import BoTSORT
except ImportError as e:
    SwarmLogger.log("ERROR", "HumanTracker", f"CRITICAL TRACKER IMPORT ERROR (BoTSORT): {e}", "IMPORT")
    BoTSORT = None


class MockBox:
    """Adapter class to mimic Ultralytics Box format."""
    def __init__(self, tlwh, conf, cls, t_id, xyxy):
        self.conf = [torch.tensor(conf)]
        self.id = [torch.tensor(t_id)] if t_id is not None else None
        self.cls = [torch.tensor(cls)]
        
        cx = tlwh[0] + tlwh[2] / 2
        cy = tlwh[1] + tlwh[3] / 2
        self.xywh = [torch.tensor([cx, cy, tlwh[2], tlwh[3]])]
        
        self.xyxy = [torch.tensor(xyxy)]


class MockResult:
    """Adapter class to mimic Ultralytics Results format."""
    def __init__(self, boxes, frame):
        self.boxes = boxes
        self.orig_img = frame
        
    def plot(self):
        return self.orig_img


class HumanTracker:
    """Main tracking class using YOLOv12 and BoT-SORT."""
    def __init__(self, enable_tracking: bool = True):
        self.opt = BoTSORTConfig()
        
        self.device = self.opt.device
        self.half = self.device != 'cpu'
        
        self._inference_lock = threading.Lock()

        try:
            if YOLO is not None:
                self.model = YOLO(self.opt.weights)
            else:
                self.model = None
        except Exception as e:
            SwarmLogger.log("ERROR", "HumanTracker", f"Model Load Error: {e}", "MODEL")
            self.model = None

        try:
            self.tracker = BoTSORT(self.opt, frame_rate=TRACKER_FRAME_RATE)
        except Exception as e:
            SwarmLogger.log("ERROR", "HumanTracker", f"Tracker Init Error: {e}", "MODEL")
            self.tracker = None
            
        self.primary_track_id = None
        
        self.yaw_pid = FilteredPID(
            TRACKING_YAW_PID_TAU, TRACKING_YAW_PID_KP, TRACKING_YAW_PID_KI, TRACKING_YAW_PID_KD,
            TRACKING_YAW_PID_INT_MAX, TRACKING_YAW_PID_INT_MIN, 
            TRACKING_YAW_PID_OUT_MAX, TRACKING_YAW_PID_OUT_MIN
        )
        self.pitch_pid = FilteredPID(
            TRACKING_PITCH_PID_TAU, TRACKING_PITCH_PID_KP, TRACKING_PITCH_PID_KI, TRACKING_PITCH_PID_KD,
            TRACKING_PITCH_PID_INT_MAX, TRACKING_PITCH_PID_INT_MIN,
            TRACKING_PITCH_PID_OUT_MAX, TRACKING_PITCH_PID_OUT_MIN
        )

        self.focal_length_px = CAMERA_FX
        self.resolution_width = CAMERA_RESOLUTION_WIDTH
        self.pitch = 0.0
        self.roll = 0.0

        self.last_detection_time = 0
        self.last_velocity_command = {'vx': 0, 'vy': 0, 'vz': 0, 'yaw_rate': 0}
        self.human_lost_timeout = HUMAN_LOST_TIMEOUT
        self.collision_threshold = COLLISION_SCREEN_THRESHOLD
        self.forward_speed = COLLISION_FORWARD_SPEED

        self.track_history: Dict[int, float] = {} 
        
        # FPS calibration state
        self._last_frame_time: float = 0.0
        self._fps_samples: list = []
        self._fps_calibrated: bool = False
        
        # Lock hysteresis state
        self._lock_challenger_id: int = None
        self._lock_challenger_streak: int = 0
        self._LOCK_HYSTERESIS_FRAMES: int = 5
        
        # IoU ID stabilization
        self._last_primary_bbox = None
        self._primary_lost_frames = 0
        self._IOU_GRACE_FRAMES = 5
        self._IOU_THRESHOLD = IOU_THRESHOLD

        # Group clustering
        self.group_engine = GroupClusterEngine()
        self._drone_pose = None

    def set_attitude(self, pitch: float, roll: float):
        self.pitch = pitch
        self.roll = roll

    def set_drone_pose(self, lat: float, lon: float, alt: float,
                       roll: float, pitch: float, yaw: float):
        """Set drone telemetry for group clustering."""
        self._drone_pose = (lat, lon, alt, roll, pitch, yaw)

    def calculate_screen_coverage(self, box: Tuple[float, float, float, float], 
                                  img_width: int, img_height: int) -> float:
        x1, y1, x2, y2 = box
        box_area = (x2 - x1) * (y2 - y1)
        screen_area = img_width * img_height
        return box_area / screen_area if screen_area > 0 else 0.0

    def detect_and_track(self, frame: np.ndarray, debug: bool = True, attack_mode: bool = False) -> Dict:
        with self._inference_lock:
            return self._detect_and_track_impl(frame, debug, attack_mode)

    def _detect_and_track_impl(self, frame: np.ndarray, debug: bool, attack_mode: bool) -> Dict:
        timestamp = time.time()
        
        # FPS calibration
        if self._last_frame_time > 0:
            dt = timestamp - self._last_frame_time
            if dt > 0:
                self._fps_samples.append(1.0 / dt)
                if len(self._fps_samples) >= 30 and (not self._fps_calibrated or len(self._fps_samples) % 100 == 0):
                    actual_fps = sum(self._fps_samples[-30:]) / 30.0
                    if self.tracker and abs(actual_fps - TRACKER_FRAME_RATE) > 2.0:
                        new_buffer = int(actual_fps / 30.0 * self.opt.track_buffer)
                        self.tracker.buffer_size = max(1, new_buffer)
                        self.tracker.max_time_lost = self.tracker.buffer_size
                        if not self._fps_calibrated:
                            SwarmLogger.log("INFO", "TRACKER",
                                f"FPS calibrated to {actual_fps:.1f}fps | buffer={self.tracker.buffer_size}", "FPS")
                    self._fps_calibrated = True
        self._last_frame_time = timestamp
        
        if self.model is None or self.tracker is None:
            return self._create_empty_result(frame)

        # Inference
        results = self.model.predict(
            source=frame,
            imgsz=self.opt.img_size,
            conf=self.opt.conf_thres,
            iou=self.opt.iou_thres,
            classes=self.opt.classes,
            augment=self.opt.augment,
            device=self.device,
            verbose=False
        )

        detections = []
        if len(results) > 0 and results[0].boxes is not None:
            boxes = results[0].boxes.data.cpu().numpy()
            detections = boxes
            
        detections = np.array(detections) if len(detections) > 0 else np.empty((0, 6))

        # Tracking update
        online_targets, lost_targets = self.tracker.update(detections, frame)
        
        mock_boxes = []
        current_visible_tracks = {}
        annotated_frame = frame.copy()
        
        for t in online_targets:
            tlwh = t.tlwh
            tid = t.track_id
            score = t.score
            
            x1, y1, w, h = tlwh
            x2, y2 = x1 + w, y1 + h
            
            mb = MockBox(tlwh, score, 0.0, tid, [x1, y1, x2, y2])
            mock_boxes.append(mb)
            
            current_visible_tracks[tid] = mb
            self.track_history[tid] = timestamp
            
            color = COLOR_PRIMARY if tid == self.primary_track_id else COLOR_SECONDARY
            cv2.rectangle(annotated_frame, (int(x1), int(y1)), (int(x2), int(y2)), color, DEBUG_FONT_THICKNESS)
            cv2.putText(annotated_frame, f"ID:{tid}", (int(x1), int(y1)-5), cv2.FONT_HERSHEY_SIMPLEX, DEBUG_FONT_SCALE, color, DEBUG_FONT_THICKNESS)

        mock_result = MockResult(mock_boxes, annotated_frame)
        all_detections = [mock_result] 

        # Selection logic
        selected_box = None
        current_track_id = None
        
        def calculate_iou(boxA, boxB):
            xA = max(boxA[0], boxB[0])
            yA = max(boxA[1], boxB[1])
            xB = min(boxA[2], boxB[2])
            yB = min(boxA[3], boxB[3])
            interArea = max(0, float(xB - xA)) * max(0, float(yB - yA))
            if interArea == 0: return 0.0
            boxAArea = (boxA[2] - boxA[0]) * (boxA[3] - boxA[1])
            boxBArea = (boxB[2] - boxB[0]) * (boxB[3] - boxB[1])
            return interArea / float(boxAArea + boxBArea - interArea)

        # Primary lock & IoU fallback
        if self.primary_track_id is not None:
            if self.primary_track_id in current_visible_tracks:
                selected_box = current_visible_tracks[self.primary_track_id]
                current_track_id = self.primary_track_id
                self._last_primary_bbox = selected_box.xyxy[0].cpu().numpy()
                self._primary_lost_frames = 0
            else:
                self._primary_lost_frames += 1
                if self._primary_lost_frames <= self._IOU_GRACE_FRAMES and self._last_primary_bbox is not None:
                    best_iou = 0.0
                    best_candidate_id = None
                    for tid, box in current_visible_tracks.items():
                        iou = calculate_iou(self._last_primary_bbox, box.xyxy[0].cpu().numpy())
                        if iou > best_iou:
                            best_iou = iou
                            best_candidate_id = tid
                            
                    if best_iou >= self._IOU_THRESHOLD and best_candidate_id is not None:
                        SwarmLogger.log("INFO", "TRACK", 
                            f"IoU Fallback: {self.primary_track_id} -> {best_candidate_id} (IoU: {best_iou:.2f})",
                            "TRACKER")
                        self.primary_track_id = best_candidate_id
                        selected_box = current_visible_tracks[best_candidate_id]
                        current_track_id = best_candidate_id
                        self._last_primary_bbox = selected_box.xyxy[0].cpu().numpy()
                        self._primary_lost_frames = 0
                else:
                    if self._primary_lost_frames == self._IOU_GRACE_FRAMES + 1:
                        SwarmLogger.log("WARNING", "TRACK",
                            f"Primary ID {self.primary_track_id} lost after {self._IOU_GRACE_FRAMES} frames",
                            "TRACKER")
                        self._last_primary_bbox = None
                        
        elif attack_mode and len(online_targets) > 0:
            if len(current_visible_tracks) == 1:
                new_tid = list(current_visible_tracks.keys())[0]
                selected_box = current_visible_tracks[new_tid]
                current_track_id = new_tid
                self.primary_track_id = new_tid
                self._lock_challenger_id = None
                self._lock_challenger_streak = 0
                self._last_primary_bbox = selected_box.xyxy[0].cpu().numpy()
                self._primary_lost_frames = 0
                SwarmLogger.log("INFO", "TRACK",
                    f"Primary ID re-mapped: {self.primary_track_id} -> {new_tid}",
                    "TRACKER")
            elif len(current_visible_tracks) > 1:
                # Lock hysteresis
                best_conf = 0.0
                best_tid = None
                for tid, box in current_visible_tracks.items():
                    conf = float(box.conf[0].item())
                    if conf > best_conf:
                        best_conf = conf
                        best_tid = tid
                
                if best_tid is not None:
                    if best_tid == self._lock_challenger_id:
                        self._lock_challenger_streak += 1
                    else:
                        self._lock_challenger_id = best_tid
                        self._lock_challenger_streak = 1
                    
                    if self._lock_challenger_streak >= self._LOCK_HYSTERESIS_FRAMES:
                        selected_box = current_visible_tracks[best_tid]
                        current_track_id = best_tid
                        self.primary_track_id = best_tid
                        self._lock_challenger_id = None
                        self._lock_challenger_streak = 0
                        self._last_primary_bbox = selected_box.xyxy[0].cpu().numpy()
                        self._primary_lost_frames = 0
                        SwarmLogger.log("INFO", "TRACK",
                            f"Lock switched to {best_tid} after {self._LOCK_HYSTERESIS_FRAMES} frames",
                            "TRACKER")
            
        # Control loop
        velocity_cmd = {'vx': 0, 'vy': 0, 'vz': 0, 'yaw_rate': 0}
        collision_imminent = False
        screen_coverage = 0.0
        
        if selected_box:
            xyxy = selected_box.xyxy[0].cpu().numpy()
            x1, y1, x2, y2 = xyxy
            
            img_height, img_width = frame.shape[:2]
            screen_coverage = self.calculate_screen_coverage((x1, y1, x2, y2), img_width, img_height)

            if screen_coverage >= self.collision_threshold:
                collision_imminent = True
            
            box_midpoint_x = (x2 + x1) / 2
            box_midpoint_y = y2 * TRACKING_TARGET_OFFSET_Y_RATIO + y1 * (1 - TRACKING_TARGET_OFFSET_Y_RATIO)
            
            midpoint_offset_y = img_height * TRACKING_SCREEN_OFFSET_Y_RATIO
            img_midpoint_x = img_width / 2
            img_midpoint_y = img_height / 2 + midpoint_offset_y

            length_x = img_midpoint_x - box_midpoint_x
            length_y = img_midpoint_y - box_midpoint_y
            
            # Smoothed deadzone
            DEADZONE = 60.0
            TRANSITION = 40.0
            
            abs_x = abs(length_x)
            if abs_x <= DEADZONE:
                gain_x = 0.0
                freeze_integral_x = True
            elif abs_x < DEADZONE + TRANSITION:
                t = (abs_x - DEADZONE) / TRANSITION
                gain_x = t * t * (3 - 2 * t)
                freeze_integral_x = False
            else:
                gain_x = 1.0
                freeze_integral_x = False
                
            abs_y = abs(length_y)
            if abs_y <= DEADZONE:
                gain_y = 0.0
                freeze_integral_y = True
            elif abs_y < DEADZONE + TRANSITION:
                t = (abs_y - DEADZONE) / TRANSITION
                gain_y = t * t * (3 - 2 * t)
                freeze_integral_y = False
            else:
                gain_y = 1.0
                freeze_integral_y = False
                
            length_x *= gain_x
            length_y *= gain_y

            # Roll adjustment
            length_x_adjusted = length_x * math.cos(self.roll) + length_y * math.sin(self.roll)
            length_y_adjusted = -length_x * math.sin(self.roll) + length_y * math.cos(self.roll)

            theta_x = math.atan2(length_x_adjusted, self.focal_length_px)
            
            delta_height_adjusted = -length_y_adjusted * TRACKING_VERTICAL_FALLBACK_GAIN

            yaw_control_raw = self.yaw_pid.update(theta_x, timestamp, theta_x, freeze_integral=freeze_integral_x)
            pitch_control = self.pitch_pid.update(delta_height_adjusted, timestamp, delta_height_adjusted, freeze_integral=freeze_integral_y)
            yaw_control = -yaw_control_raw 

            lateral_correction = -theta_x * TRACKING_LATERAL_CORRECTION_GAIN

            # Dynamic speed
            if screen_coverage < APPROACH_MIN_SCREEN_COVERAGE:
                forward_speed_dynamic = APPROACH_BOOST_SPEED
            else:
                forward_speed_dynamic = self.forward_speed
            
            velocity_cmd = {
                'vx': forward_speed_dynamic,
                'vy': float(lateral_correction),
                'vz': float(pitch_control),
                'yaw_rate': float(yaw_control)
            }

            SwarmLogger.log("DEBUG", "CONTROL", 
                f"CMD: vx={forward_speed_dynamic:.2f}, yaw={yaw_control:.2f}, cov={screen_coverage:.2f}",
                f"TRACKER_{current_track_id}")
            
            self.last_velocity_command = velocity_cmd
            self.last_detection_time = timestamp
        
        if not selected_box and timestamp - self.last_detection_time > self.human_lost_timeout:
             self.yaw_pid.reset()
             self.pitch_pid.reset()

        bbox = None
        if selected_box:
            xyxy = selected_box.xyxy[0].cpu().numpy()
            bbox = tuple(xyxy)

        # Group clustering
        detected_candidates = self._get_track_candidates(mock_boxes, timestamp)
        group_result = GroupResult(groups=[], singles=detected_candidates)

        if self._drone_pose and len(detected_candidates) >= 2:
            try:
                lat, lon, alt, d_roll, d_pitch, d_yaw = self._drone_pose
                group_result = self.group_engine.update(
                    detected_candidates, lat, lon, alt, d_roll, d_pitch, d_yaw
                )

                for g in group_result.groups:
                    gx1, gy1, gx2, gy2 = g.bounding_rect_px
                    pad = 8
                    gx1, gy1 = max(0, gx1 - pad), max(0, gy1 - pad)
                    gx2 = min(frame.shape[1], gx2 + pad)
                    gy2 = min(frame.shape[0], gy2 + pad)
                    cv2.rectangle(annotated_frame, (gx1, gy1), (gx2, gy2),
                                  GROUP_VISUAL_COLOR_BGR, 2)
                    label = f"x{g.member_count}"
                    cv2.putText(annotated_frame, label,
                                (gx1 + 4, gy1 - 6),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                                GROUP_VISUAL_COLOR_BGR, 2)
            except Exception as e:
                SwarmLogger.log("WARNING", "GROUP",
                    f"Clustering error: {e}", "TRACKER")

        return {
            'detected': len(online_targets) > 0,
            'velocity_cmd': velocity_cmd,
            'collision_imminent': collision_imminent,
            'screen_coverage': screen_coverage,
            'annotated_frame': annotated_frame,
            'track_id': current_track_id,
            'bbox': bbox,
            'all_detections': all_detections,
            'detected_candidates': detected_candidates,
            'group_info': group_result,
            'is_ghost': False
        }

    def _get_track_candidates(self, mock_boxes, timestamp):
        candidates = []
        for box in mock_boxes:
            xyxy = box.xyxy[0].cpu().numpy()
            x1, y1, x2, y2 = xyxy
            w = x2 - x1
            h = y2 - y1
            cx = x1 + w/2
            cy = y1 + h/2
            
            candidates.append({
                'id': int(box.id[0].item()),
                'conf': float(box.conf[0].item()),
                'box_xywh': [cx, cy, w, h],
                'box_xyxy': xyxy,
                'timestamp': timestamp,
                'center': (cx, cy),
                'dims': (w, h)
            })
        return candidates

    def set_primary_target(self, track_id):
        if track_id is not None and track_id != self.primary_track_id:
            SwarmLogger.log("INFO", "HumanTracker", f"LOCKED: Track ID {track_id}", "TRACKING")
        self.primary_track_id = track_id

    def reset(self):
        self.yaw_pid.reset()
        self.pitch_pid.reset()
        self.last_detection_time = 0
        self.primary_track_id = None
        self._last_primary_bbox = None
        self._primary_lost_frames = 0
        if self.tracker:
            self.tracker.tracked_stracks = []
            self.tracker.lost_stracks = []
            self.tracker.removed_stracks = []
            self.tracker.frame_id = 0
            
        SwarmLogger.log("INFO", "HumanTracker", "Tracker reset", "TRACKING")

    def _create_empty_result(self, frame):
         return {
            'detected': False,
            'velocity_cmd': {'vx': 0, 'vy': 0, 'vz': 0, 'yaw_rate': 0},
            'collision_imminent': False,
            'screen_coverage': 0.0,
            'annotated_frame': frame,
            'track_id': None,
            'all_detections': [],
            'detected_candidates': [],
            'is_ghost': False
        }
