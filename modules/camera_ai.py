"""TepeGöz - Kamera Handler Modülü

ROS üzerinden kamera feedlerini alır, JPEG formatına çevirir ve web üzerinden stream eder.
Multi-drone desteği ile birden fazla kamera feed'i yönetir.
"""

import cv2
import numpy as np
import rospy
import threading
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from config import (
    CAMERA_TOPICS, LOG_THROTTLE_SEC, JPEG_QUALITY, 
    PLACEHOLDER_IMAGE_SIZE, FONT_SCALE, FONT_THICKNESS,
    FRAME_BUFFER_TIMEOUT_FRAMES
)

class CameraAIHandler:

    def __init__(self):
        self.bridge = CvBridge()
        self.camera_feeds = {}
        self.lock = threading.Lock()
        self.is_ros_node_initialized = False
        self.subscribed_ports = set()
        self.last_log_time = {}

    def init_ros_node(self):
        if not self.is_ros_node_initialized:
            try:
                rospy.init_node('web_camera_listener', anonymous=True, disable_signals=True)
                self.is_ros_node_initialized = True
            except rospy.exceptions.ROSException as e:
                print(f"[ROS Kamera] init hatası / zaten başlatılmış olabilir: {e}")
                self.is_ros_node_initialized = True

    def subscribe_to_camera_topic_for_port(self, port):
        
        if port is None:
            return False
        if port in self.subscribed_ports:
            return True
        topic = CAMERA_TOPICS.get(port)
        if not topic:
            print(f"[ROS Kamera] CAMERA_TOPICS içinde port yok: {port}")
            return False

        self.init_ros_node()

        try:
            rospy.Subscriber(topic, Image, self._camera_callback_wrapper, callback_args=port, queue_size=1)
            self.subscribed_ports.add(port)
            print(f"[ROS Kamera] Abone olundu: {topic} (port: {port})")
            return True
        except Exception as e:
            print(f"[ROS Kamera] Subscriber oluşturulurken hata: {e}")
            return False

    # Kamera mesajını alır ve JPEG olarak saklar
    def _camera_callback_wrapper(self, msg, port):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(f"[ROS Kamera] CvBridge hata: {e}")
            return
        except Exception as e:
            print(f"[ROS Kamera] Image->CV çevirme hatası: {e}")
            return

        try:
            ret, buf = cv2.imencode('.jpg', cv_image, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
            if not ret:
                print(f"[ROS Kamera] JPEG encode başarısız (port: {port})")
                return

            jpeg_bytes = buf.tobytes()
            with self.lock:
                self.camera_feeds[port] = jpeg_bytes
        except Exception as e:
            print(f"[ROS Kamera] Encode/saklama hatası: {e}")

    def get_latest_frame(self, port):
        """Son JPEG frame'i döner"""
        with self.lock:
            return self.camera_feeds.get(port)

    def get_latest_frame_as_array(self, port):
        """Son frame'i numpy array olarak döner (tracker için)"""
        jpeg_bytes = self.get_latest_frame(port)
        if jpeg_bytes is None:
            return None
        
        try:
            nparr = np.frombuffer(jpeg_bytes, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            return frame
        except Exception as e:
            return None

    def clear_frame(self, port):
        """Belirtilen portun frame bilgisini siler"""
        with self.lock:
            if port in self.camera_feeds:
                del self.camera_feeds[port]

    def generate_frames(self, active_drone_port):
        
        if active_drone_port is not None:
            subscribed = self.subscribe_to_camera_topic_for_port(active_drone_port)
            if not subscribed:
                print(f"[CameraAIHandler] {active_drone_port} için abonelik başarısız.")

        boundary = b'--frame\r\n'
        content_type = b'Content-Type: image/jpeg\r\n\r\n'
        last_valid_frame = None  # Son geçerli frame'i cache'le
        no_frame_count = 0

        while True:
            try:
                frame = self.get_latest_frame(active_drone_port)
                if frame:
                    last_valid_frame = frame  # Cache'e kaydet
                    no_frame_count = 0  # Reset counter
                    yield boundary + content_type + frame + b'\r\n'
                else:
                    no_frame_count += 1
                    # Son geçerli frame varsa onu göster (Config'den timeout değeri)
                    if last_valid_frame is not None and no_frame_count < FRAME_BUFFER_TIMEOUT_FRAMES:
                        yield boundary + content_type + last_valid_frame + b'\r\n'
                    else:
                        # Uzun süreli kayıp - placeholder göster
                        placeholder = self._create_placeholder_image(f"Kamera Beslemesi Yok (port: {active_drone_port})")
                        if placeholder:
                            yield boundary + content_type + placeholder + b'\r\n'
                time.sleep(0.05)
            except GeneratorExit:
                break
            except Exception as e:
                print(f"[CameraAIHandler] generate_frames hatası: {e}")
                time.sleep(0.2)

    def _create_placeholder_image(self, text):
        
        try:
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
            return buf.tobytes()
        except Exception as e:
            print(f"[CameraAIHandler] Placeholder oluşturma hatası: {e}")
            return None