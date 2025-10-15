"""TepeGöz - Çarpma Görevi Kontrol Modülü

Tek hücre: 360° dönerek tarama
Çoklu hücre: Boustrophedon pattern ile tarama
İnsan tespiti: Takip ve çarpma modu (ekran kaplaması %40 eşiği)
"""

import threading
import time
import math
from collections import deque
from statistics import median
from dronekit import VehicleMode
from pymavlink import mavutil
from modules.human_tracker import HumanTracker
from config import (
    TAKEOFF_ALTITUDE, SQUARE_SIZE, DRONE_SPEED,
    MAX_SPEED_M_S, CONTROL_INTERVAL_S, CELL_REACHED_THRESHOLD_M,
    FINE_APPROACH_THRESHOLD_M, FINE_APPROACH_SCALE, CENTER_CONFIRM_TOLERANCE_M,
    CENTER_CONFIRM_HOLD_S, FINE_APPROACH_HOLD_S, PER_CELL_TIMEOUT_S,
    RETRY_LIMIT, GPS_MEDIAN_WINDOW, STUCK_MOVED_THRESHOLD_M, STUCK_TIMEOUT_S,
    PID_KP, PID_KI, PID_KD,
    COLLISION_SCREEN_THRESHOLD, COLLISION_FORWARD_SPEED, HUMAN_LOST_TIMEOUT,
    ROTATION_YAW_INCREMENT, ROTATION_YAW_SPEED, ROTATION_DIRECTION,
    YOLO_CONF_THRESHOLD,
    MISSION_STATUS_MESSAGES
)


class PID:
    """PID kontrolcüsü - dronun hedefe doğru hareketini ayarlar"""

    def __init__(self, kp=PID_KP, ki=PID_KI, kd=PID_KD, integral_limit=10.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = 0.0
        self.integral_limit = integral_limit

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def step(self, error, dt):
        if dt <= 0:
            return 0.0
        self.integral += error * dt
        self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
        deriv = (error - self.prev_error) / dt
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * deriv


class CollisionMissionController:
    """Çarpma görevi için özel kontrol sınıfı"""

    def __init__(self, drone_manager, assigned_port=None):
        self.drone_manager = drone_manager
        self.assigned_port = assigned_port  # Bu controller'ın hangi drone'a ait olduğu
        self.current_mission_point = None
        self.current_mission_type = None
        self.mission_coordinates = None
        self.centers_2d = []
        self.grid_status = {}
        self.num_rows = 0
        self.num_cols = 0
        self.is_mission_active = False
        self.mission_path_points = []
        self._mission_thread = None
        self.camera_handler = self.drone_manager.camera_handler
        
        # İnsan takip modülü
        self.human_tracker = HumanTracker(model_path="./models/yolov8n.pt")
        self.tracking_mode = False
        self.collision_detected = False
        self.scanning_active = False
        self.last_annotated_frame = None

    def _haversine_distance_m(self, lat1, lon1, lat2, lon2):
        """İki GPS noktası arasındaki mesafeyi hesaplar"""
        R = 6371000.0
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        a = math.sin(dphi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2.0) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return R * c

    def _latlon_to_north_east_m(self, lat1, lon1, lat2, lon2):
        d_north = (lat2 - lat1) * 111320.0
        mean_lat = math.radians((lat1 + lat2) / 2.0)
        d_east = (lon2 - lon1) * 111320.0 * math.cos(mean_lat)
        return d_north, d_east

    def send_ned_velocity(self, vehicle, vx, vy, vz=0):
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, 0)
        try:
            vehicle.send_mavlink(msg)
            vehicle.flush()
        except Exception as e:
            print('send_ned_velocity hata:', e)

    def send_yaw_rate(self, vehicle, yaw_rate):
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000111111000111,  # yaw rate kullan
            0, 0, 0,  # position
            0, 0, 0,  # velocity
            0, 0, 0,  # acceleration
            0, yaw_rate)  # yaw, yaw_rate
        try:
            vehicle.send_mavlink(msg)
            vehicle.flush()
        except Exception as e:
            print('send_yaw_rate hata:', e)

    def send_ned_velocity_with_yaw_rate(self, vehicle, vx_body, vy_body, vz, yaw_rate):
        """Drone'a NED (world) frame'de hız + yaw rate komutu gönderir
        
        Body frame hızlarını alıp NED (world) frame'e dönüştürür.
        Böylece drone dönerken de hedefe doğru ilerler.
        
        Args:
            vx_body: Body frame ileri hız (m/s)
            vy_body: Body frame yan hız (m/s, FRD: pozitif=sağa)
            vz: Dikey hız (m/s, NED: pozitif=aşağı)
            yaw_rate: Dönüş hızı (rad/s)
        """
        # Drone'un mevcut yaw açısını al
        try:
            current_yaw = vehicle.attitude.yaw  # radyan
        except:
            current_yaw = 0.0
            
        # Body frame'den NED (world) frame'e dönüştürme
        # Body frame: x=ileri, y=sağa (FRD)
        # NED frame: x=kuzey, y=doğu
        # Dönüş matrisi:
        # vx_ned = vx_body * cos(yaw) - vy_body * sin(yaw)
        # vy_ned = vx_body * sin(yaw) + vy_body * cos(yaw)
        import math
        cos_yaw = math.cos(current_yaw)
        sin_yaw = math.sin(current_yaw)
        
        vx_ned = vx_body * cos_yaw - vy_body * sin_yaw
        vy_ned = vx_body * sin_yaw + vy_body * cos_yaw
        
        # Debug: Dönüşüm bilgisi
        if not hasattr(self, '_transform_log_count'):
            self._transform_log_count = 0
        self._transform_log_count += 1
        
        if self._transform_log_count % 10 == 0:
            print(f"\n🔄 BODY→NED DÖNÜŞÜMÜ:")
            print(f"   Drone Yaw: {math.degrees(current_yaw):.1f}°")
            print(f"   Body Frame: vx={vx_body:+.2f}, vy={vy_body:+.2f} m/s")
            print(f"   NED Frame:  vx={vx_ned:+.2f} (kuzey), vy={vy_ned:+.2f} (doğu) m/s")
        
        # NED frame'de hız + yaw rate gönder
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # NED (world) frame
            0b0000011111000111,  # type_mask: velocity + yaw_rate kullan
            0, 0, 0,  # position (ignore)
            vx_ned, vy_ned, vz,  # velocity (NED frame: north, east, down)
            0, 0, 0,  # acceleration (ignore)
            0,  # yaw (ignore)
            yaw_rate  # yaw_rate (rad/s)
        )
        try:
            vehicle.send_mavlink(msg)
            vehicle.flush()
        except Exception as e:
            print('send_ned_velocity_with_yaw_rate hata:', e)

    def create_grid(self, cell_centers):
        self.mission_coordinates = cell_centers
        self.grid_status = {}
        self.centers_2d = []
        if not cell_centers:
            self.num_rows = self.num_cols = 0
            return
        total = len(cell_centers)
        num_cols = int(math.sqrt(total)) or 1
        num_rows = math.ceil(total / num_cols)
        self.num_rows, self.num_cols = num_rows, num_cols
        self.centers_2d = [[None for _ in range(num_cols)] for _ in range(num_rows)]
        idx = 0
        for r in range(num_rows):
            for c in range(num_cols):
                if idx < total:
                    lat, lon = cell_centers[idx]
                    self.centers_2d[r][c] = (lat, lon)
                    self.grid_status[(r, c)] = 'unvisited'
                    idx += 1
                else:
                    self.centers_2d[r][c] = None
        self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["AREA_SET"].format(
            rows=num_rows, cols=num_cols)
        print(f"Grid oluşturuldu: {num_rows}x{num_cols} (row x col).")

    def _next_indices(self, row, col, horiz_dir, vert_dir):
        next_col = col + horiz_dir

        if 0 <= next_col < self.num_cols and self.centers_2d[row][next_col] is not None:
            return row, next_col, horiz_dir, vert_dir

        next_row = row + vert_dir
        if 0 <= next_row < self.num_rows:
            horiz_dir *= -1
            new_col = 0 if horiz_dir == 1 else self.num_cols - 1
            return next_row, new_col, horiz_dir, vert_dir

        vert_dir *= -1
        horiz_dir *= -1
        new_row = row + vert_dir
        new_col = 0 if horiz_dir == 1 else self.num_cols - 1

        return new_row, new_col, horiz_dir, vert_dir

    def _get_median_location(self, vehicle, samples_deque):
        if not samples_deque:
            loc = vehicle.location.global_relative_frame
            return (loc.lat, loc.lon) if loc else (None, None)
        lats = [s[0] for s in samples_deque]
        lons = [s[1] for s in samples_deque]
        return median(lats), median(lons)

    def confirm_center_stability(self, vehicle, target_lat, target_lon, hold_s=None, tol_m=None):
        hold_s = CENTER_CONFIRM_HOLD_S if hold_s is None else hold_s
        tol_m = CENTER_CONFIRM_TOLERANCE_M if tol_m is None else tol_m
        start = time.time()
        while time.time() - start < hold_s:
            if not self.is_mission_active:
                return False
            loc = vehicle.location.global_relative_frame
            if not loc:
                return False
            d = self._haversine_distance_m(loc.lat, loc.lon, target_lat, target_lon)
            self.send_ned_velocity(vehicle, 0, 0, 0)
            if d > tol_m:
                return False
            time.sleep(0.05)
        return True

    def _approach_target(self, vehicle, target_lat, target_lon, from_row, from_col, to_row, to_col):
        for attempt in range(RETRY_LIMIT + 1):
            pid_n = PID(kp=PID_KP, ki=PID_KI, kd=PID_KD)
            pid_e = PID(kp=PID_KP, ki=PID_KI, kd=PID_KD)
            pid_n.reset()
            pid_e.reset()

            samples = deque(maxlen=GPS_MEDIAN_WINDOW)
            start_ts = time.time()
            last_move_ts = time.time()
            last_pos = None

            while self.is_mission_active and not self.tracking_mode:
                loc = vehicle.location.global_relative_frame
                if loc and loc.lat is not None:
                    samples.append((loc.lat, loc.lon))

                if not samples:
                    time.sleep(CONTROL_INTERVAL_S)
                    continue

                filt_lat, filt_lon = self._get_median_location(vehicle, samples)
                d_north, d_east = self._latlon_to_north_east_m(filt_lat, filt_lon, target_lat, target_lon)
                dist = math.hypot(d_north, d_east)
                speed = math.hypot(vehicle.velocity[0], vehicle.velocity[1])

                self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["APPROACHING"].format(
                    from_row=from_row, from_col=from_col, to_col=to_col, to_row=to_row, distance=dist, speed=speed)

                if dist <= CELL_REACHED_THRESHOLD_M:
                    self.send_ned_velocity(vehicle, 0, 0, 0)
                    hold_ok = self.confirm_center_stability(
                        vehicle, target_lat, target_lon, hold_s=FINE_APPROACH_HOLD_S, tol_m=CELL_REACHED_THRESHOLD_M)
                    if hold_ok:
                        return True
                    else:
                        pid_n.reset()
                        pid_e.reset()
                        time.sleep(0.12)
                        continue

                vx = pid_n.step(d_north, CONTROL_INTERVAL_S)
                vy = pid_e.step(d_east, CONTROL_INTERVAL_S)

                if dist < FINE_APPROACH_THRESHOLD_M:
                    vx *= FINE_APPROACH_SCALE
                    vy *= FINE_APPROACH_SCALE

                speed = math.hypot(vx, vy)
                if speed > MAX_SPEED_M_S:
                    scale = MAX_SPEED_M_S / speed
                    vx *= scale
                    vy *= scale

                try:
                    self.send_ned_velocity(vehicle, vx, vy, 0)
                except Exception as e:
                    print('Velocity gönderme hatası:', e)

                if last_pos is None and loc:
                    last_pos = (loc.lat, loc.lon)
                    last_move_ts = time.time()
                elif loc:
                    moved = self._haversine_distance_m(last_pos[0], last_pos[1], loc.lat, loc.lon)
                    if moved > STUCK_MOVED_THRESHOLD_M:
                        last_pos = (loc.lat, loc.lon)
                        last_move_ts = time.time()
                    else:
                        if time.time() - last_move_ts > STUCK_TIMEOUT_S:
                            self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["STUCK"]
                            print(MISSION_STATUS_MESSAGES["STUCK"])
                            try:
                                self.send_ned_velocity(vehicle, -vx * 0.4, -vy * 0.4, 0)
                                time.sleep(0.4)
                                self.send_ned_velocity(vehicle, vx * 1.1, vy * 1.1, 0)
                            except Exception as e:
                                print('Manevra hatası:', e)
                            last_move_ts = time.time()

                elapsed = time.time() - start_ts
                timeout = PER_CELL_TIMEOUT_S * (1 + attempt * 0.5)
                if elapsed > timeout:
                    print(MISSION_STATUS_MESSAGES["TIMEOUT"])
                    self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["TIMEOUT"]
                    break

                time.sleep(CONTROL_INTERVAL_S)

        return False

    def check_human_detection(self):
        """Kamera handler'dan insan tespiti olup olmadığını kontrol eder"""
        if not hasattr(self, 'scanning_active') or not self.scanning_active:
            return False
        try:
            # Bu drone'un kamerasından frame al
            target_port = self.assigned_port if self.assigned_port else self.drone_manager.active_drone_port
            frame = self.camera_handler.get_latest_frame_as_array(target_port)
            if frame is None:
                return False
            
            # YOLOv8 ile manuel tespit yap
            from ultralytics import YOLO
            if not hasattr(self, '_detection_model'):
                model_path = "./TepeGoz-main/models/yolov8n.pt"
                self._detection_model = YOLO(model_path)
                print("\n" + "="*80)
                print(f"🤖 [YOLO DRONE {target_port}] MODEL YÜKLENDI - TARAMA BAŞLADI")
                print("="*80)
                print(f"📁 Model: {model_path}")
                print(f"🔌 Port: {target_port}")
                print(f"🎯 Hedef sınıf: person (0)")
                print(f"📊 Güven eşiği: {YOLO_CONF_THRESHOLD}")
                print("="*80 + "\n")
            
            results = self._detection_model.predict(frame, classes=[0], conf=YOLO_CONF_THRESHOLD, verbose=False)
            
            if len(results) > 0 and len(results[0].boxes) > 0:
                # İnsan tespit edildi
                boxes = results[0].boxes
                print("\n" + "="*80)
                print(f"🎯 [DRONE {target_port}] İNSAN TESPİT EDİLDİ!")
                print("="*80)
                print(f"🔌 Port: {target_port}")
                print(f"👥 Tespit edilen kişi sayısı: {len(boxes)}")
                for i, box in enumerate(boxes):
                    conf = box.conf[0].item() if hasattr(box.conf[0], 'item') else box.conf[0]
                    xyxy = box.xyxy[0].cpu().numpy() if box.xyxy[0].is_cuda else box.xyxy[0].numpy()
                    print(f"   Kişi #{i+1}: Güven={conf:.2f}, BBox=({xyxy[0]:.0f},{xyxy[1]:.0f},{xyxy[2]:.0f},{xyxy[3]:.0f})")
                print("="*80 + "\n")
                return True
                
        except Exception as e:
            print(f"❌ [HATA] İnsan tespit kontrolü hatası: {e}")
        
        return False

    def perform_360_rotation_scan(self, vehicle):
        self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["ROTATING_360"]
        print(MISSION_STATUS_MESSAGES["ROTATING_360"])
        print("Sürekli dönüş başlıyor. İnsan tespit edilene kadar dönecek.")

        self.scanning_active = True

        target_yaw = 0  # Hedef yaw açısı
        rotation_count = 0
        
        print(f"[DÖNÜŞ] Başlangıç - sürekli dönüş başlıyor (Config: {ROTATION_YAW_INCREMENT}° adımlar)")
        
        while self.is_mission_active:
            rotation_count += 1
            target_yaw = (target_yaw + ROTATION_YAW_INCREMENT) % 360
            
            print(f"[DÖNÜŞ] #{rotation_count} - Hedef: {target_yaw}° (göreceli +{ROTATION_YAW_INCREMENT}°)")
            
            # MAV_CMD_CONDITION_YAW komutu - Config'den parametreler
            msg = vehicle.message_factory.command_long_encode(
                0, 0,  # target system, target component
                mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
                0,  # confirmation
                ROTATION_YAW_INCREMENT,  # param1: dönüş açısı (derece) - Config'den
                ROTATION_YAW_SPEED,      # param2: dönüş hızı (derece/saniye) - Config'den
                ROTATION_DIRECTION,      # param3: yön (1=saat yönü, -1=tersi) - Config'den
                1,  # param4: göreceli (1=göreceli, 0=mutlak)
                0, 0, 0  # param5-7: kullanılmıyor
            )
            
            try:
                vehicle.send_mavlink(msg)
                vehicle.flush()
                print(f"[DÖNÜŞ] Komut gönderildi - {ROTATION_YAW_INCREMENT}° dönüyor...")
            except Exception as e:
                print(f'[HATA] Dönüş komutu: {e}')
            
            # Dönüş için bekleme - Config'den hesaplanan süre
            wait_time = (ROTATION_YAW_INCREMENT / ROTATION_YAW_SPEED) + 0.25  # +0.25s buffer
            time.sleep(wait_time)
            
            if self.check_human_detection():
                print(MISSION_STATUS_MESSAGES["HUMAN_DETECTED"])
                self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["HUMAN_DETECTED"]
                return True

        return False

    def execute_tracking_mode(self, vehicle):
        self.tracking_mode = True
        
        target_port = self.assigned_port if self.assigned_port else self.drone_manager.active_drone_port
        print("\n" + "="*80)
        print(f"🎯 [DRONE {target_port}] TAKİP MODU BAŞLATILDI")
        print("="*80)
        print("📋 İlk Durum:")
        print(f"   - Port: {target_port}")
        print(f"   - Drone Modu: {vehicle.mode.name}")
        print(f"   - Yükseklik: {vehicle.location.global_relative_frame.alt:.2f}m")
        print(f"   - GPS: ({vehicle.location.global_relative_frame.lat:.6f}, {vehicle.location.global_relative_frame.lon:.6f})")
        print("="*80 + "\n")
        
        print("[TAKİP] Human tracker reset ediliyor...")
        
        try:
            self.human_tracker.reset()
            print("✅ [TAKİP] Human tracker hazır\n")
        except Exception as e:
            print(f"❌ [HATA] Human tracker reset hatası: {e}")
            return

        try:
            loop_count = 0
            last_log_time = time.time()
            
            while self.is_mission_active and self.tracking_mode:
                loop_count += 1
                
                # Bu drone'un kamerasından frame al
                target_port = self.assigned_port if self.assigned_port else self.drone_manager.active_drone_port
                frame = self.camera_handler.get_latest_frame_as_array(target_port)
                
                if frame is None:
                    if loop_count % 20 == 0:  # Her 1 saniyede bir log
                        print(f"⏳ [TAKİP] Frame alınamıyor, bekleniyor... (döngü #{loop_count})")
                    time.sleep(0.05)
                    continue
                
                if loop_count == 1:
                    print(f"✅ [DRONE {target_port} TAKİP] İlk frame alındı: {frame.shape}\n")

                try:
                    attitude = vehicle.attitude
                    self.human_tracker.set_attitude(attitude.pitch, attitude.roll)
                except Exception as e:
                    if loop_count == 1:
                        print(f"⚠️ [TAKİP] Attitude bilgisi alınamadı: {e}\n")
                
                try:
                    result = self.human_tracker.detect_and_track(frame)
                    
                    if 'annotated_frame' in result and result['annotated_frame'] is not None:
                        import cv2
                        annotated = result['annotated_frame'].copy()
                        
                        # Tracking bilgilerini ekle (varsa)
                        if result.get('track_id') is not None:
                            track_text = f"Track ID: {result['track_id']}"
                            cv2.putText(annotated, track_text, (10, 100), 
                                      cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
                        
                        ret, buf = cv2.imencode('.jpg', annotated, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
                        if ret:
                            jpeg_bytes = buf.tobytes()
                            target_port = self.assigned_port if self.assigned_port else self.drone_manager.active_drone_port
                            with self.camera_handler.lock:
                                self.camera_handler.camera_feeds[target_port] = jpeg_bytes
                    
                except Exception as e:
                    print(f"[HATA] detect_and_track hatası: {e}")
                    import traceback
                    traceback.print_exc()
                    break

                if result['detected']:
                    coverage = result['screen_coverage'] * 100
                    
                    self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["TRACKING_HUMAN"].format(
                        coverage=coverage)

                    if result['collision_imminent']:
                        self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["COLLISION_DETECTED"].format(
                            coverage=coverage)
                        print(f"\n{'='*80}")
                        print(f"💥 [DRONE {target_port}] ÇARPMA TESPİT EDİLDİ!")
                        print(f"{'='*80}")
                        print(f"🔌 Port: {target_port}")
                        print(f"📊 Ekran Kaplaması: {coverage:.1f}%")
                        print(f"{'='*80}\n")
                        self.collision_detected = True

                    # Hız komutlarını uygula - BODY FRAME + YAW RATE
                    vel_cmd = result['velocity_cmd']
                    vx = vel_cmd['vx']  # İleri hız (body frame)
                    vy = vel_cmd['vy']  # Yan hız (body frame)
                    vz = vel_cmd['vz']  # Dikey hız (NED)
                    yaw_rate = vel_cmd.get('yaw_rate', 0.0)  # Yaw rate (rad/s)
                    
                    # NED (world) frame'de hız + yaw rate gönder
                    try:
                        self.send_ned_velocity_with_yaw_rate(vehicle, vx, vy, vz, yaw_rate)
                        
                        # Komut gönderimi başarılı log (her 10 frame'de)
                        if loop_count % 10 == 0:
                            current_time = time.time()
                            elapsed = current_time - last_log_time
                            last_log_time = current_time
                            # Drone'un mevcut yaw'u
                            try:
                                current_yaw_deg = math.degrees(vehicle.attitude.yaw)
                                print(f"✅ [KOMUT GÖNDER] NED frame hız komutu gönderildi (yaw={current_yaw_deg:.1f}°, son {elapsed:.1f}s)")
                            except:
                                print(f"✅ [KOMUT GÖNDER] NED frame hız komutu gönderildi (son {elapsed:.1f}s)")
                    except Exception as e:
                        print(f"❌ [HATA] Hız komutu gönderilemedi: {e}")

                else:
                    # İnsan tespit edilmedi
                    self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["HUMAN_LOST"]
                    vel_cmd = result['velocity_cmd']
                    vx = vel_cmd['vx']
                    vy = vel_cmd['vy']
                    vz = vel_cmd['vz']
                    yaw_rate = vel_cmd.get('yaw_rate', 0.0)
                    
                    try:
                        self.send_ned_velocity_with_yaw_rate(vehicle, vx, vy, vz, yaw_rate)
                    except Exception as e:
                        if loop_count % 10 == 0:
                            print(f"❌ [HATA] Hız komutu gönderilemedi (hedef kayıp): {e}")

                time.sleep(0.05)

        except Exception as e:
            print(f"\n{'='*80}")
            print(f"❌ [HATA] TAKİP MODU HATASI")
            print(f"{'='*80}")
            print(f"Hata: {e}")
            import traceback
            traceback.print_exc()
            print(f"{'='*80}\n")
            print("🛑 [TAKİP] Takip modu sonlandırılıyor, drone durduruluyor")
        finally:
            from config import RESUME_SCAN_AFTER_LOST
            
            target_port = self.assigned_port if self.assigned_port else self.drone_manager.active_drone_port
            print(f"\n{'='*80}")
            print(f"🛑 [DRONE {target_port}] TAKİP MODU SONLANDI")
            print(f"{'='*80}")
            print(f"🔌 Port: {target_port}")
            print(f"📊 İstatistikler:")
            print(f"   - Toplam frame: {loop_count}")
            print(f"   - Çarpma durumu: {'✅ BAŞARILI' if self.collision_detected else '❌ GERÇEKLEŞMEDI'}")
            print(f"{'='*80}\n")
            
            print("🛑 Drone durduruluyor...")
            try:
                self.send_ned_velocity(vehicle, 0, 0, 0)
                print("✅ Drone durduruldu (NED velocity: 0,0,0)")
            except Exception as e:
                print(f"⚠️ Drone durdurma hatası: {e}")
            
            self.tracking_mode = False
            print("✅ Tracking mode kapatıldı\n")
            
            # Çarpma gerçekleşmediyse ve RESUME_SCAN_AFTER_LOST aktifse taramaya dön
            if not self.collision_detected and RESUME_SCAN_AFTER_LOST and self.is_mission_active:
                print("🔄 Hedef kayboldu ama çarpma gerçekleşmedi.")
                print("📡 Tarama moduna geri dönülüyor...\n")
                # is_mission_active True kalacak, tarama devam edecek

    def _run_mission_loop(self, mission_type, start_point=None):
        """Ana görev döngüsü"""
        # Bu controller'ın atandığı drone'u al
        if self.assigned_port and self.assigned_port in self.drone_manager.drones:
            vehicle = self.drone_manager.drones[self.assigned_port]
        else:
            vehicle = self.drone_manager.active_drone
        
        if not vehicle or vehicle == "connecting" or not self.centers_2d:
            self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["NO_AREA"]
            return

        try:
            self.mission_path_points = []
            self.mission_start_time = time.time()

            # Kalkış işlemi (kalkış sırasında insan tespiti YOK)
            self.arm_and_takeoff(vehicle, TAKEOFF_ALTITUDE)
            self.is_mission_active = True

            self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["BEGIN_MISSION_FLIGHT"]
            print(self.drone_manager.mission_status_message)

            # Tek hücre mi çoklu hücre mi kontrol et
            total_cells = sum(1 for r in range(self.num_rows) for c in range(self.num_cols) 
                            if self.centers_2d[r][c] is not None)

            if total_cells == 1:
                # Tek hücre: Merkeze git ve 360° dön
                port_info = f" (Port: {self.assigned_port})" if self.assigned_port else ""
                print(f"Tek hücre tespit edildi{port_info}. Merkeze gidip 360° dönüş yapılacak.")
                
                # Tek hücrenin koordinatlarını bul
                target_lat, target_lon = None, None
                for r in range(self.num_rows):
                    for c in range(self.num_cols):
                        if self.centers_2d[r][c] is not None:
                            target_lat, target_lon = self.centers_2d[r][c]
                            break
                    if target_lat is not None:
                        break

                if target_lat is not None:
                    # Merkeze git
                    ok = self._approach_target(vehicle, target_lat, target_lon, 0, 0, 0, 0)
                    
                    if ok and not self.tracking_mode:
                        # Merkeze ulaşıldı, 360° dönüş yap
                        human_detected = self.perform_360_rotation_scan(vehicle)
                        
                        if human_detected:
                            # İnsan tespit edildi, takip moduna geç
                            self.execute_tracking_mode(vehicle)
                    elif self.tracking_mode:
                        # Merkeze giderken insan tespit edildi
                        self.execute_tracking_mode(vehicle)

            else:
                # Çoklu hücre: Normal boustrophedon tarama
                port_info = f" (Port: {self.assigned_port})" if self.assigned_port else ""
                print(f"Çoklu hücre tespit edildi{port_info}. Normal tarama yapılacak.")
                
                # Tarama modunu aktif et - artık insan tespiti yapılabilir
                self.scanning_active = True
                
                if start_point:
                    cur_row, cur_col, horiz_dir, vert_dir = self._find_start_indices(start_point)
                else:
                    cur_row = self.num_rows - 1
                    cur_col = 0
                    horiz_dir = 1
                    vert_dir = -1

                prev_row, prev_col = None, None

                while self.is_mission_active and not self.tracking_mode:
                    self.current_mission_point = (cur_row, cur_col, horiz_dir, vert_dir)

                    target = self.centers_2d[cur_row][cur_col]

                    if target is None:
                        prev_row, prev_col = cur_row, cur_col
                        cur_row, cur_col, horiz_dir, vert_dir = self._next_indices(cur_row, cur_col, horiz_dir, vert_dir)

                        if cur_row == -1 and cur_col == -1:
                            print('Tüm hücreler ziyaret edildi. Görev sonlandırılıyor.')
                            self.is_mission_active = False
                        continue

                    lat, lon = target
                    port_info = f" (Port {self.assigned_port})" if self.assigned_port else ""
                    self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["SCANNING_AREA"].format(
                        row=cur_row, col=cur_col)
                    print(self.drone_manager.mission_status_message + port_info)

                    ok = self._approach_target(vehicle, lat, lon, prev_row, prev_col, cur_row, cur_col)

                    if self.tracking_mode:
                        # İnsan tespit edildi, takip moduna geç
                        self.execute_tracking_mode(vehicle)
                        break

                    if ok:
                        self.grid_status[(cur_row, cur_col)] = 'visited'
                        self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["CELL_CONFIRMED"].format(
                            row=cur_row, col=cur_col)
                        print(self.drone_manager.mission_status_message)

                        stable = self.confirm_center_stability(vehicle, lat, lon, 
                                                              hold_s=CENTER_CONFIRM_HOLD_S, 
                                                              tol_m=CENTER_CONFIRM_TOLERANCE_M)
                        if not stable:
                            print('Center confirmation failed after visit — retrying approach')
                            continue
                    else:
                        self.grid_status[(cur_row, cur_col)] = 'old'
                        self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["CELL_SKIPPED"].format(
                            row=cur_row, col=cur_col)
                        print(self.drone_manager.mission_status_message)

                    prev_row, prev_col = cur_row, cur_col
                    cur_row, cur_col, horiz_dir, vert_dir = self._next_indices(cur_row, cur_col, horiz_dir, vert_dir)

                    if cur_row == -1 and cur_col == -1:
                        self.is_mission_active = False

                    if cur_row != prev_row:
                        target_next = self.centers_2d[cur_row][cur_col]
                        if target_next is not None:
                            time.sleep(0.12)

                    time.sleep(0.08)

        except Exception as e:
            self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["MISSION_ERROR"].format(error=str(e))
            print(self.drone_manager.mission_status_message)
            try:
                vehicle.mode = VehicleMode('RTL')
            except Exception:
                pass

        finally:
            if vehicle and self.is_mission_active:
                try:
                    self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["MISSION_COMPLETE"]
                    vehicle.mode = VehicleMode('RTL')
                except Exception:
                    pass
            self.is_mission_active = False
            self.tracking_mode = False
            self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["STOPPED"]

    def _find_start_indices(self, start_point):
        start_row, start_col, start_horiz_dir, start_vert_dir = start_point
        return start_row, start_col, start_horiz_dir, start_vert_dir

    def arm_and_takeoff(self, vehicle, aTargetAltitude):
        self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["ARMING"]
        print(MISSION_STATUS_MESSAGES["ARMING"])
        while not vehicle.is_armable:
            print('Waiting for armable...')
            time.sleep(1)
        vehicle.mode = VehicleMode('GUIDED')
        vehicle.armed = True
        while not vehicle.armed:
            print('Arming...')
            time.sleep(0.5)
        self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["TAKING_OFF"].format(
            altitude=aTargetAltitude)
        print(self.drone_manager.mission_status_message)
        vehicle.simple_takeoff(aTargetAltitude)
        while True:
            alt = vehicle.location.global_relative_frame.alt
            print(f' Yükseklik: {alt:.2f}')
            if alt >= aTargetAltitude * 0.95:
                self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["TAKEOFF_SUCCESS"].format(
                    altitude=aTargetAltitude)
                print(self.drone_manager.mission_status_message)
                break
            time.sleep(0.5)

        self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["BEGIN_MISSION_FLIGHT"]
        print(self.drone_manager.mission_status_message)
        time.sleep(1)

    def start_mission(self, mission_type, resume_point=None):
        # Bu controller'a atanmış drone'u kontrol et
        target_vehicle = None
        if self.assigned_port and self.assigned_port in self.drone_manager.drones:
            target_vehicle = self.drone_manager.drones[self.assigned_port]
        else:
            target_vehicle = self.drone_manager.active_drone
        
        if not target_vehicle or target_vehicle == "connecting":
            self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["NO_ACTIVE"]
            return {'status': 'error', 'message': MISSION_STATUS_MESSAGES["NO_ACTIVE"]}

        self.current_mission_type = mission_type

        if self.is_mission_active:
            self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["ALREADY_ACTIVE"]
            return {"status": "error", "message": MISSION_STATUS_MESSAGES["ALREADY_ACTIVE"]}

        if not self.mission_coordinates:
            self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["NO_AREA"]
            return {'status': 'error', 'message': MISSION_STATUS_MESSAGES["NO_AREA"]}

        if resume_point:
            self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["RESUMING"]
            self._mission_thread = threading.Thread(
                target=self._run_mission_loop,
                args=(mission_type, resume_point),
                daemon=True
            )
        else:
            self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["STARTING"]
            self._mission_thread = threading.Thread(
                target=self._run_mission_loop,
                args=(mission_type,),
                daemon=True
            )

        self._mission_thread.start()

        return {'status': 'ok', 'message': self.drone_manager.mission_status_message}

    def stop_mission(self):
        # Bu controller'a atanmış drone'u al
        target_vehicle = None
        if self.assigned_port and self.assigned_port in self.drone_manager.drones:
            target_vehicle = self.drone_manager.drones[self.assigned_port]
        else:
            target_vehicle = self.drone_manager.active_drone
        
        if target_vehicle and target_vehicle != "connecting" and self.is_mission_active:
            try:
                target_vehicle.mode = VehicleMode('RTL')
            except Exception:
                pass

            self.is_mission_active = False
            self.tracking_mode = False
            self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["STOPPED"]

            return {'status': 'ok', 'message': MISSION_STATUS_MESSAGES["STOPPED"]}

        self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["NO_ACTIVE"]
        return {'status': 'error', 'message': MISSION_STATUS_MESSAGES["NO_ACTIVE"]}

    def get_status(self):
        """Görevle ilgili güncel durum bilgilerini döndürür"""
        elapsed = 0
        if self.is_mission_active and hasattr(self, 'mission_start_time'):
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
                        current_location = {"lat": float(g.lat), "lon": float(g.lon), 
                                          "alt": float(getattr(g, "alt", 0.0))}
                batt = getattr(vehicle, "battery", None)
                if batt is not None and hasattr(batt, "level"):
                    battery_level = batt.level
        except Exception as e:
            print("get_status: konum/batarya okunurken hata:", e)

        json_friendly_grid_status = {}
        for (row, col), status in self.grid_status.items():
            json_friendly_grid_status[f"{row},{col}"] = status

        return {
            "is_mission_active": self.is_mission_active,
            "tracking_mode": self.tracking_mode,
            "collision_detected": self.collision_detected,
            "elapsed_time": elapsed,
            "grid_status": json_friendly_grid_status,
            "mission_path_points": self.mission_path_points,
            "current_location": current_location,
            "battery_level": battery_level,
            "status_message": status_message
        }

    def set_area(self, coordinates):
        """Gözlem alanını ayarlar ve gridi oluşturur"""
        self.create_grid(coordinates)
        return {'status': 'ok', 'message': MISSION_STATUS_MESSAGES["AREA_SET"].format(
            rows=self.num_rows, cols=self.num_cols)}
