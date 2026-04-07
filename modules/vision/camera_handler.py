"""ROS camera ingestion and JPEG streaming."""

from __future__ import annotations

import cv2
import numpy as np
import rospy
import threading
import time
from dataclasses import dataclass
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from config import (
    CAMERA_TOPICS, JPEG_QUALITY,
    PLACEHOLDER_IMAGE_SIZE, FONT_SCALE, FONT_THICKNESS,
    FRAME_STREAM_STALE_TIMEOUT_S,
)
from modules.core.logger import SwarmLogger
from modules.core.runtime import now_stamp, monotonic_now


@dataclass(frozen=True)
class FramePacket:
    """Latest frame snapshot with runtime timing metadata."""

    frame: np.ndarray
    seq: int
    captured_at_wall: float
    captured_at_monotonic: float

class CameraAIHandler:

    def __init__(self):
        self.bridge = CvBridge()
        self.camera_feeds = {}
        self.lock = threading.Lock()
        self._subscribe_lock = threading.Lock()
        self.is_ros_node_initialized = False
        self.subscribed_ports = set()
        self.last_log_time = {}
        self._placeholder_cache = {}
        self._frame_seq = {}
        self._last_frame_mono = {}

    def init_ros_node(self):
        if not self.is_ros_node_initialized:
            try:
                rospy.init_node('web_camera_listener', anonymous=True, disable_signals=True)
                self.is_ros_node_initialized = True
            except rospy.exceptions.ROSException as e:
                SwarmLogger.log("WARNING", "Camera", f"ROS init error: {e}", "ROS")
                self.is_ros_node_initialized = True

    def subscribe_to_camera_topic_for_port(self, port):
        if port is None:
            return False
        with self._subscribe_lock:
            if port in self.subscribed_ports:
                return True
            topic = CAMERA_TOPICS.get(port)
            if not topic:
                SwarmLogger.log("WARNING", "Camera", f"Port not found in CAMERA_TOPICS: {port}", "ROS")
                return False

            self.init_ros_node()

            try:
                rospy.Subscriber(topic, Image, self._camera_callback_wrapper, callback_args=port, queue_size=1)
                self.subscribed_ports.add(port)
                SwarmLogger.log("INFO", "Camera", f"Subscribed: {topic} (port: {port})", "ROS")
                return True
            except Exception as e:
                SwarmLogger.log("ERROR", "Camera", f"Error creating subscriber: {e}", "ROS")
                return False

    def _camera_callback_wrapper(self, msg, port):
        """Receive camera message and store as RAW."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            SwarmLogger.log("ERROR", "Camera", f"CvBridge error: {e}", "ROS")
            return
        except Exception as e:
            SwarmLogger.log("ERROR", "Camera", f"Image->CV conversion error: {e}", "ROS")
            return

        stamp = now_stamp()
        with self.lock:
            seq = int(self._frame_seq.get(port, 0)) + 1
            self._frame_seq[port] = seq
            prev_mono = float(self._last_frame_mono.get(port, 0.0) or 0.0)
            dt = 0.0 if prev_mono <= 0.0 else max(0.0, stamp.monotonic_time - prev_mono)
            self._last_frame_mono[port] = stamp.monotonic_time
            fps = (1.0 / dt) if dt > 1e-6 else None
            self.camera_feeds[port] = {
                'raw': cv_image,
                'annotated': None,
                'jpeg': None,
                'seq': seq,
                'frame_seq': seq,
                'captured_at_wall': stamp.wall_time,
                'captured_at_monotonic': stamp.monotonic_time,
                'source_fps': fps,
            }


    def get_latest_frame(self, port):
        """Return latest JPEG frame for web stream (lazy encoding).
        
        Prefers annotated frame (with bounding boxes) over raw.
        """
        with self.lock:
            data = self.camera_feeds.get(port)
            if data is None or not isinstance(data, dict):
                return None
            
            if data.get('jpeg') is not None:
                return data['jpeg']
            
            # Prefer annotated over raw for streaming
            source = data.get('annotated') or data.get('raw')
            if source is None:
                return None
            
            try:
                ret, buf = cv2.imencode('.jpg', source, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
                if ret:
                    jpeg_bytes = buf.tobytes()
                    data['jpeg'] = jpeg_bytes
                    return jpeg_bytes
            except Exception as e:
                SwarmLogger.log("ERROR", "Camera", f"Lazy JPEG encode error: {e}", "STREAM")
            
            return None

    def get_latest_frame_as_array(self, port):
        """Return latest raw frame (numpy array) for tracker."""
        with self.lock:
            data = self.camera_feeds.get(port)
            if data and isinstance(data, dict) and 'raw' in data:
                return data['raw']
            return None

    def get_latest_frame_packet(self, port):
        """Return latest raw frame plus sequence/timing metadata."""
        with self.lock:
            data = self.camera_feeds.get(port)
            if not data or not isinstance(data, dict):
                return None
            frame = data.get("raw")
            seq = data.get("seq", data.get("frame_seq"))
            if frame is None or seq is None:
                return None
            return FramePacket(
                frame=frame,
                seq=int(seq),
                captured_at_wall=float(data.get("captured_at_wall", 0.0) or 0.0),
                captured_at_monotonic=float(data.get("captured_at_monotonic", 0.0) or 0.0),
            )

    def set_annotated_frame(self, port, annotated_frame):
        """Store processed frame from tracker for streaming.
        
        Raw frame is preserved separately for inference.
        """
        with self.lock:
            if port not in self.camera_feeds:
                self.camera_feeds[port] = {}
            
            self.camera_feeds[port]['annotated'] = annotated_frame
            self.camera_feeds[port]['jpeg'] = None  # Invalidate cache

    def clear_frame(self, port):
        """Clear frame info for specified port."""
        with self.lock:
            if port in self.camera_feeds:
                del self.camera_feeds[port]

    def reset_runtime_state(self, ports=None):
        """Clear volatile frame caches without dropping ROS subscriptions."""
        with self.lock:
            target_ports = set(self.camera_feeds.keys())
            if ports is None:
                target_ports.update(self.subscribed_ports)
            else:
                target_ports.update(int(port) for port in ports if port is not None)
            for port in target_ports:
                self.camera_feeds[port] = {
                    "raw": None,
                    "annotated": None,
                    "jpeg": None,
                    "seq": self._frame_seq.get(port, self.camera_feeds.get(port, {}).get("frame_seq", 0)),
                    "frame_seq": self._frame_seq.get(port, self.camera_feeds.get(port, {}).get("frame_seq", 0)),
                    "captured_at_wall": 0.0,
                    "captured_at_monotonic": 0.0,
                    "source_fps": None,
                }

    def generate_frames(self, active_drone_port):
        if active_drone_port is not None:
            subscribed = self.subscribe_to_camera_topic_for_port(active_drone_port)
            if not subscribed:
                SwarmLogger.log("WARNING", "Camera", f"Subscription failed for {active_drone_port}", "STREAM")

        boundary = b'--frame\r\n'
        content_type = b'Content-Type: image/jpeg\r\n\r\n'
        last_valid_frame = None
        last_valid_mono = 0.0

        while True:
            try:
                frame = self.get_latest_frame(active_drone_port)
                if frame:
                    last_valid_frame = frame
                    last_valid_mono = monotonic_now()
                    yield boundary + content_type + frame + b'\r\n'
                else:
                    if (
                        last_valid_frame is not None
                        and (monotonic_now() - last_valid_mono) < float(FRAME_STREAM_STALE_TIMEOUT_S)
                    ):
                        yield boundary + content_type + last_valid_frame + b'\r\n'
                    else:
                        placeholder = self._create_placeholder_image(f"No Camera Feed (port: {active_drone_port})")
                        if placeholder:
                            yield boundary + content_type + placeholder + b'\r\n'
                time.sleep(0.05)
            except GeneratorExit:
                break
            except Exception as e:
                SwarmLogger.log("ERROR", "Camera", f"generate_frames error: {e}", "STREAM")
                time.sleep(0.2)

    def _create_placeholder_image(self, text):
        try:
            cached = self._placeholder_cache.get(text)
            if cached is not None:
                return cached
            w, h = PLACEHOLDER_IMAGE_SIZE
            img = np.zeros((h, w, 3), dtype=np.uint8)
            font = cv2.FONT_HERSHEY_SIMPLEX
            (tw, th), _ = cv2.getTextSize(text, font, FONT_SCALE, FONT_THICKNESS)
            x = max(10, (w - tw) // 2)
            y = max(20, (h + th) // 2)
            cv2.putText(img, text, (x, y), font, FONT_SCALE, (200, 200, 200), FONT_THICKNESS, cv2.LINE_AA)
            ret, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
            if not ret:
                return None
            jpeg = buf.tobytes()
            self._placeholder_cache[text] = jpeg
            return jpeg
        except Exception as e:
            SwarmLogger.log("ERROR", "Camera", f"Placeholder creation error: {e}", "STREAM")
            return None
