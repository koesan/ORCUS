"""ROS camera ingestion and JPEG streaming."""

import cv2
import numpy as np
import rospy
import threading
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from config import (
    CAMERA_TOPICS, JPEG_QUALITY,
    PLACEHOLDER_IMAGE_SIZE, FONT_SCALE, FONT_THICKNESS,
    FRAME_BUFFER_TIMEOUT_FRAMES
)
from modules.core.logger import SwarmLogger

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

        with self.lock:
            self.camera_feeds[port] = {'raw': cv_image, 'annotated': None, 'jpeg': None}


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
                self.camera_feeds[port] = {"raw": None, "annotated": None, "jpeg": None}

    def generate_frames(self, active_drone_port):
        if active_drone_port is not None:
            subscribed = self.subscribe_to_camera_topic_for_port(active_drone_port)
            if not subscribed:
                SwarmLogger.log("WARNING", "Camera", f"Subscription failed for {active_drone_port}", "STREAM")

        boundary = b'--frame\r\n'
        content_type = b'Content-Type: image/jpeg\r\n\r\n'
        last_valid_frame = None
        no_frame_count = 0

        while True:
            try:
                frame = self.get_latest_frame(active_drone_port)
                if frame:
                    last_valid_frame = frame
                    no_frame_count = 0
                    yield boundary + content_type + frame + b'\r\n'
                else:
                    no_frame_count += 1
                    if last_valid_frame is not None and no_frame_count < FRAME_BUFFER_TIMEOUT_FRAMES:
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
