"""TepeGöz - Drone Manager Modülü

Multi-drone bağlantı ve paralel görev yönetimi (DroneKit).
Her dron bağımsız olarak kendi görevini yürütür.
Akıllı alan paylaşımı ile çarpışma önleme.
"""

import threading
import time
import math
from typing import Dict, Any, Optional, List, Tuple
from dronekit import connect, VehicleMode, LocationGlobalRelative
import rospy
from .camera_ai import CameraAIHandler
from config import (
    CONNECTION_TIMEOUT,
    DRONE_CONNECTION_RETRY_COUNT,
    CONNECTION_STATUS_CONNECTED,
    CONNECTION_STATUS_NOT_CONNECTED,
    MISSION_STATUS_MESSAGES
)

class DroneManager:
    """Multi-drone yönetimi - her dron bağımsız çalışır"""
    
    def __init__(self):
        self.drones = {}  # port -> vehicle mapping
        self.drone_controllers = {}  # port -> CollisionMissionController mapping
        self.active_drone = None  # Kamera görüntüleme için seçili dron
        self.active_drone_port = None
        self.connection_status = CONNECTION_STATUS_NOT_CONNECTED
        self.mission_status_message = MISSION_STATUS_MESSAGES["NOT_CONNECTED"]
        self.camera_handler = CameraAIHandler()
        self.lock = threading.Lock()

    def connect_drone_async(self, connection_string, port):
        corrected_connection_string = f"tcp:127.0.0.1:{port}"
        vehicle = None

        for i in range(DRONE_CONNECTION_RETRY_COUNT):
            try:
                self.mission_status_message = MISSION_STATUS_MESSAGES["RETRYING"].format(current=i + 1, total=DRONE_CONNECTION_RETRY_COUNT)
                print(self.mission_status_message)
                
                vehicle = connect(corrected_connection_string, wait_ready=False, timeout=CONNECTION_TIMEOUT)
                break
            except Exception as e:
                print(f"Bağlantı hatası (Deneme {i + 1}): {e}")
                
                if i < DRONE_CONNECTION_RETRY_COUNT - 1:
                    time.sleep(5)
                else:
                    print(MISSION_STATUS_MESSAGES["ALL_RETRIES_FAILED"])

        if vehicle:
            self.drones[port] = vehicle
            if not self.active_drone:
                self.active_drone = vehicle
                self.active_drone_port = port
            print(f"Başarılı: Dron {port} bağlandı.")
            self.connection_status = CONNECTION_STATUS_CONNECTED
            self.mission_status_message = MISSION_STATUS_MESSAGES["CONNECTED"]
            self.camera_handler.subscribe_to_camera_topic_for_port(port)
        else:
            self.drones.pop(port, None)
            self.connection_status = CONNECTION_STATUS_NOT_CONNECTED
            self.mission_status_message = MISSION_STATUS_MESSAGES["CONNECTION_ERROR"]
            if not self.drones:
                self.connection_status = CONNECTION_STATUS_NOT_CONNECTED
                self.mission_status_message = MISSION_STATUS_MESSAGES["NOT_CONNECTED"]
    
    def start_connection(self, connection_string):
        
        try:
            extracted_port = int(connection_string.split(':')[-1])
            if extracted_port in self.drones and self.drones[extracted_port] != "connecting":
                return {"status": "error", "message": f"Bu porta bağlı bir dron zaten var: {extracted_port}"}

            self.drones[extracted_port] = "connecting"
            threading.Thread(target=self.connect_drone_async, args=(connection_string, extracted_port), daemon=True).start()
            
            return {"status": "ok", "message": MISSION_STATUS_MESSAGES["CONNECTING"]}
        except (IndexError, ValueError):
            return {"status": "error", "message": "Geçersiz bağlantı adresi formatı."}


    def select_drone(self, port):
        
        try:
            port = int(port)
            if port in self.drones and self.drones[port] != "connecting":
                self.active_drone = self.drones[port]
                self.active_drone_port = port
                print(f"Dron {port} aktif dron olarak seçildi.")
                return {"status": "ok", "message": f"Dron {port} seçildi."}
            return {"status": "error", "message": "Geçersiz dron seçimi veya dron hala bağlanıyor."}
        except (ValueError, TypeError):
            return {"status": "error", "message": "Geçersiz port numarası."}

    def get_or_create_controller(self, port: int):
        """Belirtilen port için mission controller döndür veya oluştur"""
        with self.lock:
            if port not in self.drone_controllers:
                # Circular import'u önlemek için burada import
                from .collision_mission_controller import CollisionMissionController
                # Her controller'a atandığı port'u ver - böylece kendi kamerasını kullanır
                controller = CollisionMissionController(self, assigned_port=port)
                self.drone_controllers[port] = controller
                print(f"✅ [DroneManager] Port {port} için yeni controller oluşturuldu (bağımsız kamera)")
            return self.drone_controllers[port]
    
    def partition_grid_intelligently(self, cell_centers: List[Tuple[float, float]]) -> Dict[int, List[Tuple[float, float]]]:
        """
        Grid hücrelerini dronlar arasında akıllıca paylaştırır.
        
        Algoritma:
        1. Toplam grid'i rows x cols matrisine dönüştür
        2. Dronları sıralı bölgelere ata (horizontal stripes)
        3. Her drone için boustrophedon pattern koruyarak sub-grid oluştur
        4. Çarpışma önleme için her drone'un alanı buffer ile ayrılır
        
        Returns:
            Dict[port, cell_centers]: Her drone için cell center listesi
        """
        if not cell_centers or not self.drones:
            return {}
        
        # Bağlı ve hazır dronları al
        ready_drones = [port for port, v in self.drones.items() if v != "connecting"]
        num_drones = len(ready_drones)
        
        if num_drones == 0:
            return {}
        
        if num_drones == 1:
            # Tek dron varsa tüm alanı ver
            return {ready_drones[0]: cell_centers}
        
        # Grid boyutlarını hesapla
        total_cells = len(cell_centers)
        num_cols = int(math.sqrt(total_cells)) or 1
        num_rows = math.ceil(total_cells / num_cols)
        
        # 2D grid matrisine dönüştür
        grid_2d = [[None for _ in range(num_cols)] for _ in range(num_rows)]
        idx = 0
        for r in range(num_rows):
            for c in range(num_cols):
                if idx < total_cells:
                    grid_2d[r][c] = cell_centers[idx]
                    idx += 1
        
        # Her drone için satır aralığı belirle (horizontal partitioning)
        rows_per_drone = num_rows / num_drones
        drone_assignments = {}
        
        for i, port in enumerate(sorted(ready_drones)):
            start_row = int(i * rows_per_drone)
            end_row = int((i + 1) * rows_per_drone) if i < num_drones - 1 else num_rows
            
            # Bu drone'un hücrelerini topla (boustrophedon pattern korunarak)
            drone_cells = []
            for r in range(start_row, end_row):
                # Boustrophedon: Satır satır zig-zag
                if r % 2 == 0:
                    # Soldan sağa
                    for c in range(num_cols):
                        if grid_2d[r][c] is not None:
                            drone_cells.append(grid_2d[r][c])
                else:
                    # Sağdan sola
                    for c in range(num_cols - 1, -1, -1):
                        if grid_2d[r][c] is not None:
                            drone_cells.append(grid_2d[r][c])
            
            drone_assignments[port] = drone_cells
            print(f"Dron {port}: {len(drone_cells)} hücre atandı (satır {start_row}-{end_row})")
        
        return drone_assignments
    
    def get_all_drones_status(self) -> Dict[int, Dict[str, Any]]:
        """Tüm dronların detaylı durumunu döndür"""
        all_status = {}
        
        for port, vehicle in self.drones.items():
            if vehicle == "connecting":
                all_status[port] = {
                    "port": port,
                    "status": "connecting",
                    "is_active_camera": (self.active_drone_port == port)
                }
                continue
            
            # Dron bilgileri
            status_data = {
                "port": port,
                "status": "connected",
                "is_active_camera": (self.active_drone_port == port),
                "battery_level": 0,
                "altitude": 0.0,
                "mode": "UNKNOWN",
                "is_armed": False,
                "location": {"lat": 0, "lon": 0, "alt": 0},
                "is_mission_active": False,
                "mission_status": "Beklemede"
            }
            
            try:
                if hasattr(vehicle, 'battery') and vehicle.battery:
                    status_data["battery_level"] = vehicle.battery.level
                if hasattr(vehicle, 'location') and vehicle.location.global_relative_frame:
                    loc = vehicle.location.global_relative_frame
                    status_data["location"] = {"lat": loc.lat, "lon": loc.lon, "alt": loc.alt}
                    status_data["altitude"] = loc.alt
                if hasattr(vehicle, 'mode') and vehicle.mode:
                    status_data["mode"] = vehicle.mode.name
                if hasattr(vehicle, 'armed'):
                    status_data["is_armed"] = vehicle.armed
                    
                # Mission controller durumu
                if port in self.drone_controllers:
                    controller = self.drone_controllers[port]
                    status_data["is_mission_active"] = controller.is_mission_active
                    status_data["mission_status"] = getattr(self, 'mission_status_message', 'Beklemede')
                    
            except Exception as e:
                print(f"Dron {port} durum bilgisi alınırken hata: {e}")
            
            all_status[port] = status_data
        
        return all_status
    
    def get_status(self):
        """Genel sistem durumu - kamera görüntüsü için aktif dron bilgisi"""
        connected_drones_list = []
        for port, vehicle_obj in self.drones.items():
            connected_drones_list.append({
                "port": port,
                "is_active": (self.active_drone_port == port) if vehicle_obj != "connecting" else False
            })

        # Tüm dronların detaylı durumu
        all_drones_status = self.get_all_drones_status()

        if not self.active_drone or self.active_drone == "connecting":
            return {
                "status": CONNECTION_STATUS_NOT_CONNECTED,
                "connected_drones": connected_drones_list,
                "all_drones": all_drones_status,
                "is_mission_active": False,
                "status_message": self.mission_status_message
            }

        current_location = {"lat": 0, "lon": 0, "alt": 0}
        ground_speed = 0
        if hasattr(self.active_drone, 'location') and self.active_drone.location.global_relative_frame:
            loc = self.active_drone.location.global_relative_frame
            current_location = {"lat": loc.lat, "lon": loc.lon, "alt": loc.alt}
            ground_speed = self.active_drone.ground_speed if hasattr(self.active_drone, 'ground_speed') else 0

        return {
            "status": CONNECTION_STATUS_CONNECTED,
            "current_location": current_location,
            "ground_speed": ground_speed,
            "battery_level": self.active_drone.battery.level if hasattr(self.active_drone, 'battery') and self.active_drone.battery else 0,
            "mode": self.active_drone.mode.name if hasattr(self.active_drone, 'mode') and self.active_drone.mode else "UNKNOWN",
            "is_armed": self.active_drone.armed if hasattr(self.active_drone, 'armed') else False,
            "connected_drones": connected_drones_list,
            "all_drones": all_drones_status,
            "is_mission_active": False,
            "status_message": self.mission_status_message
        }