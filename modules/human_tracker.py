"""TepeGöz - İnsan Takip Modülü

YOLOv8 ile insan tespiti ve PID kontrol ile takip.
Sabit ileri hız + yön düzeltmesi ile çarpma hareketi.
Ekran kaplaması %40 eşiği ile çarpma tespiti.
"""

import numpy as np
import cv2
import math
import time
from typing import Tuple, Dict
from ultralytics import YOLO
from config import (
    COLLISION_FORWARD_SPEED, 
    COLLISION_SCREEN_THRESHOLD, 
    HUMAN_LOST_TIMEOUT,
    YOLO_TRACKING_ENABLED,
    YOLO_TRACKER_TYPE,
    YOLO_TRACK_PERSIST,
    YOLO_CONF_THRESHOLD,
    # PID Parametreleri
    TRACKING_YAW_PID_TAU, TRACKING_YAW_PID_KP, TRACKING_YAW_PID_KI, TRACKING_YAW_PID_KD,
    TRACKING_YAW_PID_INT_MAX, TRACKING_YAW_PID_INT_MIN, TRACKING_YAW_PID_OUT_MAX, TRACKING_YAW_PID_OUT_MIN,
    TRACKING_PITCH_PID_TAU, TRACKING_PITCH_PID_KP, TRACKING_PITCH_PID_KI, TRACKING_PITCH_PID_KD,
    TRACKING_PITCH_PID_INT_MAX, TRACKING_PITCH_PID_INT_MIN, TRACKING_PITCH_PID_OUT_MAX, TRACKING_PITCH_PID_OUT_MIN,
    # Kamera Parametreleri
    CAMERA_FOCAL_LENGTH, CAMERA_SENSOR_WIDTH, CAMERA_RESOLUTION_WIDTH, CAMERA_PIXEL_SIZE_UM,
    # Takip Parametreleri
    TRACKING_TARGET_OFFSET_Y_RATIO, TRACKING_SCREEN_OFFSET_Y_RATIO,
    TRACKING_LATERAL_CORRECTION_GAIN, TRACKING_VERTICAL_FALLBACK_GAIN
)


class PID:
    """PID kontrolcüsü - hedef takibi için"""
    
    def __init__(self, tau, kp, ki, kd, integrator_max, integrator_min, pid_max, pid_min):
        self.previous_measurement = 0
        self.previous_error = 0
        self.previous_timestamp = 0
        self.i = 0
        self.d = 0
        self.tau = tau
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integrator_max = integrator_max
        self.integrator_min = integrator_min
        self.pid_max = pid_max
        self.pid_min = pid_min

    def update(self, error, timestamp, measurement):
        delta_t = timestamp - self.previous_timestamp

        if delta_t <= 0:
            return 0.0

        p = self.kp * error
        i = self.i + self.ki * delta_t * (error + self.previous_error) / 2

        if i > self.integrator_max:
            i = self.integrator_max
        elif i < self.integrator_min:
            i = self.integrator_min

        d = -(2 * self.kd * (measurement - self.previous_measurement) + 
              (2 * self.tau - delta_t) * self.d) / (2 * self.tau + delta_t)

        pid = p + i + d

        if pid > self.pid_max:
            pid = self.pid_max
        elif pid < self.pid_min:
            pid = self.pid_min

        self.previous_error = error
        self.previous_measurement = measurement
        self.previous_timestamp = timestamp
        self.i = i
        self.d = d

        return pid

    def reset(self):
        self.previous_measurement = 0
        self.previous_error = 0
        self.previous_timestamp = 0
        self.i = 0
        self.d = 0


class HumanTracker:

    def __init__(self, model_path: str = "./models/yolov8n.pt", enable_tracking: bool = YOLO_TRACKING_ENABLED):
        try:
            self.model = YOLO(model_path)
            print(f"[HumanTracker] YOLOv8 model yüklendi: {model_path}")
        except Exception as e:
            print(f"[HumanTracker] Model yükleme hatası: {e}")
            self.model = None
        
        # Tracking özellikleri
        self.tracking_enabled = enable_tracking
        self.tracker_type = YOLO_TRACKER_TYPE
        self.track_persist = YOLO_TRACK_PERSIST
        self.primary_track_id = None  # Takip edilen ana hedef ID'si
        self.track_switch_threshold = 0.3  # Hedef değiştirme için ekran kaplama eşiği
        
        if self.tracking_enabled:
            print(f"[HumanTracker] 🎯 TRACKING AKTIF - Tracker: {self.tracker_type}")
        else:
            print(f"[HumanTracker] ⚠️ TRACKING PASIF - Sadece detection kullanılıyor")

        # PID kontrolcüleri - Config'den yüklenen parametreler
        self.yaw_pid = PID(
            TRACKING_YAW_PID_TAU, TRACKING_YAW_PID_KP, TRACKING_YAW_PID_KI, TRACKING_YAW_PID_KD,
            TRACKING_YAW_PID_INT_MAX, TRACKING_YAW_PID_INT_MIN, 
            TRACKING_YAW_PID_OUT_MAX, TRACKING_YAW_PID_OUT_MIN
        )
        self.pitch_pid = PID(
            TRACKING_PITCH_PID_TAU, TRACKING_PITCH_PID_KP, TRACKING_PITCH_PID_KI, TRACKING_PITCH_PID_KD,
            TRACKING_PITCH_PID_INT_MAX, TRACKING_PITCH_PID_INT_MIN,
            TRACKING_PITCH_PID_OUT_MAX, TRACKING_PITCH_PID_OUT_MIN
        )

        # Kamera optik parametreleri - Config'den yüklenen değerler
        self.effective_focal_length = CAMERA_FOCAL_LENGTH  # mm
        self.width = CAMERA_SENSOR_WIDTH  # pixels
        self.resolution_width = CAMERA_RESOLUTION_WIDTH  # pixels
        self.pixel_size = CAMERA_PIXEL_SIZE_UM  # um
        self.pixel_size = self.width * self.pixel_size / (self.resolution_width * 1000)  # um to mm

        self.pitch = 0.0
        self.roll = 0.0

        self.last_detection_time = 0
        self.last_velocity_command = {'vx': 0, 'vy': 0, 'vz': 0, 'yaw_rate': 0}
        self.human_lost_timeout = 2.0
        self.collision_threshold = COLLISION_SCREEN_THRESHOLD
        self.forward_speed = COLLISION_FORWARD_SPEED

        self.current_depth_image = None
        self.frame_count = 0
        self.track_history = {}  # Track ID -> son görülme zamanı

    def set_attitude(self, pitch: float, roll: float):
        self.pitch = pitch
        self.roll = roll

    def set_depth_image(self, depth_image: np.ndarray):
        self.current_depth_image = depth_image

    def calculate_screen_coverage(self, box: Tuple[float, float, float, float], 
                                  img_width: int, img_height: int) -> float:
        x1, y1, x2, y2 = box
        box_area = (x2 - x1) * (y2 - y1)
        screen_area = img_width * img_height
        return box_area / screen_area if screen_area > 0 else 0.0

    def detect_and_track(self, frame: np.ndarray, debug: bool = True) -> Dict:
        """
        Görüntüde insan tespit et ve takip komutu üret
        
        Args:
            frame: RGB/BGR görüntü karesi
            debug: Debug logları yazdır (her 10 frame'de bir)
            
        Returns:
            Dict: {
                'detected': bool,
                'velocity_cmd': {'vx': float, 'vy': float, 'vz': float, 'yaw': float},
                'collision_imminent': bool,
                'screen_coverage': float,
                'annotated_frame': np.ndarray,
                'track_id': int or None  # Takip edilen hedefin ID'si
            }
        """
        timestamp = time.time()
        
        # Frame sayacı (debug için)
        if not hasattr(self, '_frame_count'):
            self._frame_count = 0
        self._frame_count += 1
        self.frame_count = self._frame_count
        
        if self.model is None:
            return {
                'detected': False,
                'velocity_cmd': {'vx': 0, 'vy': 0, 'vz': 0, 'yaw': 0},
                'collision_imminent': False,
                'screen_coverage': 0.0,
                'annotated_frame': frame,
                'track_id': None
            }

        # YOLO tracking veya detection kullan
        if self.tracking_enabled:
            # TRACKING MODU - Track ID'leri korunur (LAP gerektirir)
            results = self.model.track(
                frame, 
                classes=[0],  # Sadece 'person' sınıfı
                persist=self.track_persist,  # Track ID'leri koru
                tracker=self.tracker_type,  # ByteTrack veya BoT-SORT
                verbose=False,
                conf=YOLO_CONF_THRESHOLD  # Config'den al (0.35)
            )
        else:
            # DETECTION MODU - Track ID olmadan çalışır
            results = self.model.predict(frame, classes=[0], conf=YOLO_CONF_THRESHOLD, verbose=False)

        velocity_cmd = {'vx': 0, 'vy': 0, 'vz': 0, 'yaw': 0}
        detected = False
        collision_imminent = False
        screen_coverage = 0.0
        annotated_frame = frame.copy()
        current_track_id = None

        if len(results) > 0 and len(results[0].boxes) > 0:
            detected = True
            self.last_detection_time = timestamp

            boxes = results[0].boxes
            
            # Tracking aktifse, hedef seçimi farklı
            if self.tracking_enabled and hasattr(boxes[0], 'id') and boxes[0].id is not None:
                # TRACKING MODU: Track ID'lere göre hedef seç
                selected_box, current_track_id = self._select_target_with_tracking(boxes, timestamp)
                
                if debug and self._frame_count % 10 == 0:
                    all_track_ids = [int(b.id[0].item()) if b.id is not None else None for b in boxes]
                    print(f"\n{'='*80}")
                    print(f"🎯 [TRACKING] Frame #{self._frame_count}")
                    print(f"{'='*80}")
                    print(f"📊 Tespit edilen kişi sayısı: {len(boxes)}")
                    print(f"🔢 Track ID'ler: {all_track_ids}")
                    print(f"🎯 Seçilen hedef Track ID: {current_track_id}")
                    print(f"🏆 Primary Track ID: {self.primary_track_id}")
                    print(f"{'='*80}")
            else:
                # DETECTION MODU: En büyük hedefi seç (eski davranış)
                areas = []
                for box in boxes:
                    xyxy = box.xyxy[0].cpu().numpy() if box.xyxy[0].is_cuda else box.xyxy[0].numpy()
                    areas.append((xyxy[2] - xyxy[0]) * (xyxy[3] - xyxy[1]))
                
                largest_idx = np.argmax(areas)
                selected_box = boxes[largest_idx]
                current_track_id = None
            
            box = selected_box

            xyxy = box.xyxy[0].cpu().numpy() if box.xyxy[0].is_cuda else box.xyxy[0].numpy()
            x1, y1, x2, y2 = xyxy
            img_height, img_width = frame.shape[:2]

            screen_coverage = self.calculate_screen_coverage((x1, y1, x2, y2), img_width, img_height)

            if screen_coverage >= self.collision_threshold:
                collision_imminent = True

            box_midpoint_x = (x2 + x1) / 2
            # Hedef nokta hesaplama - Config'den yüklenen oran kullanılıyor
            # 0.75 = (y2 + 3*y1)/4 formülü (göğüs seviyesi)
            box_midpoint_y = y2 * TRACKING_TARGET_OFFSET_Y_RATIO + y1 * (1 - TRACKING_TARGET_OFFSET_Y_RATIO)

            midpoint_offset_x = 0
            midpoint_offset_y = img_height * TRACKING_SCREEN_OFFSET_Y_RATIO  # Config'den: 1/16
            img_midpoint_x = img_width / 2 + midpoint_offset_x
            img_midpoint_y = img_height / 2 + midpoint_offset_y

            length_x = img_midpoint_x - box_midpoint_x  # pixels
            length_y = img_midpoint_y - box_midpoint_y  # pixels
            
            # DEBUG: Hedef konumu
            if debug and self._frame_count % 10 == 0:
                hedef_yatay = "SOLDA" if length_x > 0 else "SAĞDA" if length_x < 0 else "MERKEZDE"
                hedef_dikey = "YUKARIDA" if length_y > 0 else "AŞAĞIDA" if length_y < 0 else "MERKEZDE"
                print(f"\n{'='*80}")
                print(f"[FRAME #{self._frame_count}] HEDEF TESPİT EDİLDİ")
                print(f"{'='*80}")
                print(f"📍 HEDEF KONUMU:")
                print(f"   Bounding Box: x1={x1:.0f}, y1={y1:.0f}, x2={x2:.0f}, y2={y2:.0f}")
                print(f"   Hedef Merkezi: ({box_midpoint_x:.0f}, {box_midpoint_y:.0f})")
                print(f"   Ekran Merkezi: ({img_midpoint_x:.0f}, {img_midpoint_y:.0f})")
                print(f"   Sapma (piksel): Yatay={length_x:.1f}px ({hedef_yatay}), Dikey={length_y:.1f}px ({hedef_dikey})")

            length_x_adjusted = length_x * math.cos(self.roll) + length_y * math.sin(self.roll)
            length_y_adjusted = -length_x * math.sin(self.roll) + length_y * math.cos(self.roll)

            theta_x = math.atan(length_x_adjusted * self.pixel_size / self.effective_focal_length)  # rad
            theta_y = math.atan(length_y_adjusted * self.pixel_size / self.effective_focal_length)  # rad
            
            # DEBUG: Açı bilgileri
            if debug and self._frame_count % 10 == 0:
                print(f"\n📐 AÇI HESAPLAMALARI:")
                print(f"   theta_x (yatay): {theta_x:.4f} rad = {math.degrees(theta_x):+.2f}° {'(SOL)' if theta_x > 0 else '(SAĞ)' if theta_x < 0 else '(MERKEZ)'}")
                print(f"   theta_y (dikey): {theta_y:.4f} rad = {math.degrees(theta_y):+.2f}°")
                print(f"   Roll kompensasyonu: {math.degrees(self.roll):.2f}°")

            if self.current_depth_image is not None:
                try:
                    point_1_y = int(y1 / img_height * len(self.current_depth_image))
                    point_1_x = int(x1 / img_width * len(self.current_depth_image[0]))
                    point_2_y = int(y2 / img_height * len(self.current_depth_image))
                    point_2_x = int(x2 / img_width * len(self.current_depth_image[0]))
                    point_3_x = int(box_midpoint_x / img_width * len(self.current_depth_image[0]))
                    point_3_y = int(box_midpoint_y / img_height * len(self.current_depth_image))
                    
                    depth_box = self.current_depth_image[point_1_y:point_2_y, point_1_x:point_2_x]
                    depth = min(np.median(depth_box), np.mean(depth_box), 
                              self.current_depth_image[point_3_y][point_3_x])
                    
                    theta_y_adjusted = self.pitch + theta_y
                    delta_height_adjusted = -depth * math.sin(theta_y_adjusted)
                except Exception as e:
                    print(f"[HumanTracker] Derinlik hesaplama hatası: {e}")
                    delta_height_adjusted = -length_y_adjusted * 0.002  # Piksel başına 0.002 m/s
            else:
                # Derinlik kamerası yok - basit yöntem kullan (Config'den kazanc parametresi)
                # İnsan merkezin altındaysa (length_y negatif) aşağı in (pozitif vz)
                # İnsan merkezin üstündeyse (length_y pozitif) yukarı çık (negatif vz)
                delta_height_adjusted = -length_y_adjusted * TRACKING_VERTICAL_FALLBACK_GAIN

            yaw_control_raw = self.yaw_pid.update(theta_x, timestamp, theta_x)
            pitch_control = self.pitch_pid.update(delta_height_adjusted, timestamp, delta_height_adjusted)
            
            # YAW işaretini ters çevir (koordinat sistemi dönüşümü)
            # theta_x pozitif (insan SOLDA) → SOLA dön → yaw_rate pozitif
            # theta_x negatif (insan SAĞDA) → SAĞA dön → yaw_rate negatif
            # MAVLink body frame'de yaw_rate pozitif = sola dön
            # Ama bizim PID çıkışı ters olduğu için işareti çeviriyoruz
            yaw_control = -yaw_control_raw  # İŞARET TERSİNE ÇEVRİLDİ!
            
            # DEBUG: PID çıkışları
            if debug and self._frame_count % 10 == 0:
                print(f"\n🎯 PID KONTROL ÇIKIŞLARI:")
                print(f"   YAW PID (ham): {yaw_control_raw:+.4f} rad/s")
                print(f"   YAW PID (son): {yaw_control:+.4f} rad/s {'(SOLA DÖN)' if yaw_control < 0 else '(SAĞA DÖN)' if yaw_control > 0 else '(DÜZ)'}")
                print(f"   PITCH PID: {pitch_control:+.3f} m/s {'(AŞAĞI)' if pitch_control > 0 else '(YUKARI)' if pitch_control < 0 else '(SABİT)'}")
            
            # Yatay düzeltme (sağa/sola hareket) - Config'den kazanc parametresi
            # Body frame FRD (Forward-Right-Down): vy pozitif = SAĞA, vy negatif = SOLA
            # theta_x pozitif = insan SOLDA → SOLA git → vy negatif
            # theta_x negatif = insan SAĞDA → SAĞA git → vy pozitif
            lateral_correction = -theta_x * TRACKING_LATERAL_CORRECTION_GAIN

            # DİNAMİK HIZ KONTROLÜ: Hedef çok küçükse daha hızlı yaklaş
            from config import APPROACH_MIN_SCREEN_COVERAGE, APPROACH_BOOST_SPEED
            
            if screen_coverage < APPROACH_MIN_SCREEN_COVERAGE:
                # Hedef çok uzak/küçük - agresif yaklaşma
                forward_speed_dynamic = APPROACH_BOOST_SPEED
                if debug and self._frame_count % 10 == 0:
                    print(f"   ⚡ AGRESİF YAKLAŞMA: {forward_speed_dynamic} m/s (hedef çok küçük: {screen_coverage:.3%})")
            else:
                # Normal hız
                forward_speed_dynamic = self.forward_speed
            
            # BODY FRAME - FRD: Forward-Right-Down
            # Bu hızlar collision_mission_controller'da NED (world) frame'e dönüştürülecek
            # vx = ileri/geri (pozitif = ileri) - Body frame'de "ileri"
            # vy = sağ/sol (pozitif = SAĞA, negatif = SOLA) - Body frame'de "yan"
            # vz = yukarı/aşağı (pozitif = aşağı NED'de)
            # yaw_rate = dönüş hızı (rad/s)
            velocity_cmd = {
                'vx': forward_speed_dynamic,  # Dinamik ileri hız (body frame → NED'e dönüşecek)
                'vy': float(lateral_correction),  # Yan hareket (body frame → NED'e dönüşecek)
                'vz': float(pitch_control),  # Dikey düzeltme (NED: pozitif=aşağı)
                'yaw_rate': float(yaw_control)  # Yaw rate (rad/s)
            }

            self.last_velocity_command = velocity_cmd
            
            # DEBUG: Hız komutları (DETAYLI)
            if debug and self._frame_count % 10 == 0:
                print(f"\n🚁 HIZ KOMUTLARI (Body Frame):")
                print(f"   vx (ileri):     {velocity_cmd['vx']:+.2f} m/s (SABİT İLERİ)")
                print(f"   vy (yan):       {velocity_cmd['vy']:+.2f} m/s {'(SOLA)' if velocity_cmd['vy'] < 0 else '(SAĞA)' if velocity_cmd['vy'] > 0 else '(DÜZ)'}")
                print(f"   vz (dikey):     {velocity_cmd['vz']:+.2f} m/s {'(AŞAĞI)' if velocity_cmd['vz'] > 0 else '(YUKARI)' if velocity_cmd['vz'] < 0 else '(SABİT)'}")
                print(f"   yaw_rate (dön): {velocity_cmd['yaw_rate']:+.3f} rad/s = {math.degrees(velocity_cmd['yaw_rate']):+.1f}°/s {'(SOLA)' if velocity_cmd['yaw_rate'] < 0 else '(SAĞA)' if velocity_cmd['yaw_rate'] > 0 else '(DÜZ)'}")
                print(f"\n📊 EKRAN KAPLAMA: {screen_coverage*100:.1f}% {'⚠️ ÇARPMA!' if collision_imminent else '✅ Devam'}")
                print(f"{'='*80}\n")

            annotated_frame = results[0].plot()
            
            cv2.circle(annotated_frame, (int(box_midpoint_x), int(box_midpoint_y)), 
                      5, (0, 0, 255), -1)
            
            cv2.line(annotated_frame, (int(img_midpoint_x), 0), 
                    (int(img_midpoint_x), int(img_height)), (0, 255, 0), 2)
            cv2.line(annotated_frame, (0, int(img_midpoint_y)), 
                    (int(img_width), int(img_midpoint_y)), (0, 255, 0), 2)
            
            cv2.putText(annotated_frame, f"Coverage: {screen_coverage*100:.1f}%", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            
            if collision_imminent:
                cv2.putText(annotated_frame, "COLLISION IMMINENT!", 
                           (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)

        else:
            time_since_last = timestamp - self.last_detection_time
            
            if time_since_last < self.human_lost_timeout:
                velocity_cmd = self.last_velocity_command.copy()
                cv2.putText(annotated_frame, "Target Lost - Maintaining Course", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
                
                # DEBUG: Hedef kaybı
                if debug and self._frame_count % 10 == 0:
                    print(f"\n{'='*80}")
                    print(f"[FRAME #{self._frame_count}] ⚠️ HEDEF KAYBEDİLDİ")
                    print(f"{'='*80}")
                    print(f"⏱️  Son görülme: {time_since_last:.1f} saniye önce")
                    print(f"🚁 Son hareket sürdürülüyor: vx={velocity_cmd['vx']:.2f}, vy={velocity_cmd['vy']:.2f}, vz={velocity_cmd['vz']:.2f}")
                    print(f"{'='*80}\n")
            else:
                velocity_cmd = {'vx': 0, 'vy': 0, 'vz': 0, 'yaw_rate': 0}
                self.yaw_pid.reset()
                self.pitch_pid.reset()
                cv2.putText(annotated_frame, "No Target Detected", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
                # DEBUG: Tam kayıp
                if debug and self._frame_count % 20 == 0:
                    print(f"\n{'='*80}")
                    print(f"[FRAME #{self._frame_count}] ❌ HEDEF TAM KAYBEDİLDİ")
                    print(f"{'='*80}")
                    print(f"⏱️  Son görülme: {time_since_last:.1f} saniye önce (timeout: {self.human_lost_timeout}s)")
                    print(f"🛑 Drone durduruluyor, PID resetlendi")
                    print(f"{'='*80}\n")

        return {
            'detected': detected,
            'velocity_cmd': velocity_cmd,
            'collision_imminent': collision_imminent,
            'screen_coverage': screen_coverage,
            'annotated_frame': annotated_frame,
            'track_id': current_track_id
        }

    def _select_target_with_tracking(self, boxes, timestamp: float):
        """
        Tracking modunda hedef seçimi - mevcut hedefi takip et, yoksa yenisini seç
        
        Args:
            boxes: YOLO detection boxes (track ID'leri içerir)
            timestamp: Şu anki zaman
            
        Returns:
            (selected_box, track_id): Seçilen box ve track ID'si
        """
        # Tüm track ID'leri ve alanları hesapla
        track_data = []
        for box in boxes:
            if hasattr(box, 'id') and box.id is not None:
                track_id = int(box.id[0].item())
                xyxy = box.xyxy[0].cpu().numpy() if box.xyxy[0].is_cuda else box.xyxy[0].numpy()
                area = (xyxy[2] - xyxy[0]) * (xyxy[3] - xyxy[1])
                track_data.append({
                    'box': box,
                    'track_id': track_id,
                    'area': area,
                    'xyxy': xyxy
                })
                # Track history güncelle
                self.track_history[track_id] = timestamp
        
        if not track_data:
            # Track ID yok, en büyüğü seç
            areas = []
            for box in boxes:
                xyxy = box.xyxy[0].cpu().numpy() if box.xyxy[0].is_cuda else box.xyxy[0].numpy()
                areas.append((xyxy[2] - xyxy[0]) * (xyxy[3] - xyxy[1]))
            largest_idx = np.argmax(areas)
            return boxes[largest_idx], None
        
        # Primary track ID varsa ve hala görünüyorsa, onu takip et
        if self.primary_track_id is not None:
            for data in track_data:
                if data['track_id'] == self.primary_track_id:
                    return data['box'], self.primary_track_id
        
        # Primary track kaybolmuş veya hiç seçilmemiş - yeni hedef seç
        # En büyük alana sahip hedefi seç
        largest = max(track_data, key=lambda x: x['area'])
        self.primary_track_id = largest['track_id']
        
        print(f"\n🎯 [TRACKING] YENİ HEDEF SEÇİLDİ: Track ID = {self.primary_track_id}")
        
        return largest['box'], largest['track_id']
    
    def reset(self):
        """Tracker'ı sıfırla - PID, track history ve primary target"""
        self.yaw_pid.reset()
        self.pitch_pid.reset()
        self.last_detection_time = 0
        self.last_velocity_command = {'vx': 0, 'vy': 0, 'vz': 0, 'yaw_rate': 0}
        self.primary_track_id = None
        self.track_history.clear()
        self.frame_count = 0
        print("[HumanTracker] 🔄 Tracker resetlendi - Track ID'ler temizlendi")
