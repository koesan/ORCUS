"""ORCUS Drone Manager - Multi-drone connection and mission management."""

import threading
import time
import math
from contextlib import contextmanager
from typing import Dict, Any, Optional, List, Tuple
from dronekit import connect, VehicleMode, LocationGlobalRelative
import rospy
from modules.core.logger import SwarmLogger
from config import (
    CONNECTION_TIMEOUT,
    DRONE_CONNECTION_RETRY_COUNT,
    DRONE_CONNECTION_RETRY_DELAY,
    CONNECTION_STATUS_CONNECTED,
    CONNECTION_STATUS_NOT_CONNECTED,
    MISSION_STATUS_MESSAGES
)

class DroneManager:
    """Multi-drone management with independent mission threads."""
    
    def __init__(self):
        self.drones = {}  # port -> vehicle mapping
        self.drone_controllers = {}  # port -> CollisionMissionController mapping
        self.active_drone = None
        self.active_drone_port = None
        self.connection_status = CONNECTION_STATUS_NOT_CONNECTED
        self.mission_status_message = MISSION_STATUS_MESSAGES["NOT_CONNECTED"]
        
        # Lazy import — avoids core→vision circular dependency
        from modules.vision.camera_handler import CameraAIHandler
        self.camera_handler = CameraAIHandler()
        self.lock = threading.Lock()
        
        # Swarm Manager (lazy import)
        from modules.swarm.coordinator import SwarmManager
        self.swarm_manager = SwarmManager(self)

    def shutdown(self):
        """Best-effort shutdown for background loops and active missions."""
        with self.lock:
            controllers_snapshot = list(self.drone_controllers.values())
            swarm_manager = self.swarm_manager

        for controller in controllers_snapshot:
            try:
                controller.is_mission_active = False
            except Exception:
                pass

        if swarm_manager is not None:
            try:
                swarm_manager.stop()
            except Exception:
                pass

    def connect_drone_async(self, connection_string, port):
        """Asynchronously connects to a single drone on the specified port."""
        # ------------------------------------------------------------------
        # CONNECTION MANAGEMENT
        # ------------------------------------------------------------------
        corrected_connection_string = f"tcp:127.0.0.1:{port}"
        vehicle = None

        for i in range(DRONE_CONNECTION_RETRY_COUNT):
            try:
                self.mission_status_message = MISSION_STATUS_MESSAGES["RETRYING"].format(current=i + 1, total=DRONE_CONNECTION_RETRY_COUNT)
                SwarmLogger.log("INFO", "DroneManager", self.mission_status_message, "CONNECTION")
                
                vehicle = connect(corrected_connection_string, wait_ready=False, timeout=CONNECTION_TIMEOUT)
                break
            except Exception as e:
                SwarmLogger.log("WARNING", "DroneManager", f"Connection error (Attempt {i + 1}): {e}", "CONNECTION")
                
                if i < DRONE_CONNECTION_RETRY_COUNT - 1:
                    time.sleep(DRONE_CONNECTION_RETRY_DELAY)
                else:
                    SwarmLogger.log("ERROR", "DroneManager", MISSION_STATUS_MESSAGES["ALL_RETRIES_FAILED"], "CONNECTION")

        if vehicle:
            with self.lock:
                self.drones[port] = vehicle
                if not self.active_drone:
                    self.active_drone = vehicle
                    self.active_drone_port = port
            SwarmLogger.log("SUCCESS", "DroneManager", f"Drone {port} connected.", "CONNECTION")
            self.connection_status = CONNECTION_STATUS_CONNECTED
            self.mission_status_message = MISSION_STATUS_MESSAGES["CONNECTED"]
            self.camera_handler.subscribe_to_camera_topic_for_port(port)
        else:
            with self.lock:
                self.drones.pop(port, None)
            # Connection failed — drone was never connected, no disconnect needed
            SwarmLogger.log("WARNING", "DroneManager",
                            f"Port {port} connection failed — skipping disconnect.",
                            "CONNECTION")
            self.connection_status = CONNECTION_STATUS_NOT_CONNECTED
            self.mission_status_message = MISSION_STATUS_MESSAGES["CONNECTION_ERROR"]
            if not self.drones:
                self.connection_status = CONNECTION_STATUS_NOT_CONNECTED
                self.mission_status_message = MISSION_STATUS_MESSAGES["NOT_CONNECTED"]

        # Failsafe: verify connection is alive
        if vehicle and port in self.drones:
            try:
                _ = vehicle.location.global_relative_frame
            except Exception as e:
                SwarmLogger.log("ERROR", "DroneManager", f"FAILSAFE: Drone {port} link test failed: {e}", "CONNECTION")
                with self.lock:
                    self.drones.pop(port, None)
                self.connection_status = CONNECTION_STATUS_NOT_CONNECTED
                self.mission_status_message = MISSION_STATUS_MESSAGES["CONNECTION_ERROR"]
    
    def start_connection(self, connection_string):
        """Initiates connection process for a drone."""
        try:
            extracted_port = int(connection_string.split(':')[-1])
            with self.lock:
                existing = self.drones.get(extracted_port)
                if existing is not None:
                    if existing == "connecting":
                        return {"status": "error", "message": f"Drone on port {extracted_port} is already connecting"}
                    return {"status": "error", "message": f"Drone already connected on port: {extracted_port}"}
                self.drones[extracted_port] = "connecting"
            threading.Thread(target=self.connect_drone_async, args=(connection_string, extracted_port), daemon=True).start()
            
            return {"status": "ok", "message": MISSION_STATUS_MESSAGES["CONNECTING"]}
        except (IndexError, ValueError):
            return {"status": "error", "message": "Invalid connection string format."}


    def select_drone(self, port):
        """Sets the active drone for UI/Telemetry."""
        try:
            port = int(port)
            with self.lock:
                vehicle = self.drones.get(port)
                if vehicle is not None and vehicle != "connecting":
                    self.active_drone = vehicle
                    self.active_drone_port = port
                else:
                    vehicle = None
            if vehicle is not None:
                SwarmLogger.log("INFO", "DroneManager", f"Drone {port} selected as active.", "SELECTION")
                return {"status": "ok", "message": f"Drone {port} selected."}
            return {"status": "error", "message": "Invalid selection or drone still connecting."}
        except (ValueError, TypeError):
            return {"status": "error", "message": "Invalid port number."}

    # ------------------------------------------------------------------
    # CONTROLLER FACTORY
    # ------------------------------------------------------------------

    def get_or_create_controller(self, port: int):
        """Returns or creates a mission controller for the specified port."""
        with self.lock:
            if port not in self.drone_controllers:
                from modules.mission.mission_controller import MissionController as CollisionMissionController
                controller = CollisionMissionController(self, assigned_port=port, swarm_manager=self.swarm_manager)
                self.drone_controllers[port] = controller
                SwarmLogger.log("SUCCESS", "DroneManager", f"Controller created for Port {port}", "CONTROLLER")
            return self.drone_controllers[port]

    @contextmanager
    def drone_context(self, port: int, vehicle):
        """Context manager for temporarily switching active drone."""
        with self.lock:
            original_active = self.active_drone
            original_port = self.active_drone_port
            self.active_drone = vehicle
            self.active_drone_port = port
        
        try:
            yield
        finally:
            with self.lock:
                if original_active:
                    self.active_drone = original_active
                    self.active_drone_port = original_port

    
    # ------------------------------------------------------------------
    # GRID PARTITION
    # ------------------------------------------------------------------

    def partition_grid_intelligently(self, cell_centers: List[Tuple[float, float]]) -> Dict[int, List[Tuple[float, float]]]:
        """Partition grid cells among drones using horizontal stripes.

        Grid dimensions: num_cols = int(sqrt(N)), num_rows = ceil(N / num_cols).
        Must match navigation.py create_grid() formula for consistency.
        """
        if not cell_centers or not self.drones:
            return {}
        
        ready_drones = [port for port, v in self.drones.items() if v != "connecting"]
        num_drones = len(ready_drones)
        
        if num_drones == 0:
            return {}
        
        if num_drones == 1:
            return {ready_drones[0]: cell_centers}
        
        # Calculate grid dimensions
        total_cells = len(cell_centers)
        num_cols = int(math.sqrt(total_cells)) or 1
        num_rows = math.ceil(total_cells / num_cols)
        
        # Convert to 2D grid matrix
        grid_2d = [[None for _ in range(num_cols)] for _ in range(num_rows)]
        idx = 0
        for r in range(num_rows):
            for c in range(num_cols):
                if idx < total_cells:
                    grid_2d[r][c] = cell_centers[idx]
                    idx += 1
        
        # Horizontal partitioning (stripes)
        rows_per_drone = num_rows / num_drones
        drone_assignments = {}
        
        for i, port in enumerate(sorted(ready_drones)):
            start_row = int(i * rows_per_drone)
            end_row = int((i + 1) * rows_per_drone) if i < num_drones - 1 else num_rows
            
            # Collect cells preserving boustrophedon pattern
            drone_cells = []
            for r in range(start_row, end_row):
                # Boustrophedon: zig-zag row by row
                if r % 2 == 0:
                    for c in range(num_cols):
                        if grid_2d[r][c] is not None:
                            drone_cells.append(grid_2d[r][c])
                else:
                    for c in range(num_cols - 1, -1, -1):
                        if grid_2d[r][c] is not None:
                            drone_cells.append(grid_2d[r][c])
            
            drone_assignments[port] = drone_cells
            SwarmLogger.log("INFO", "DroneManager", f"Drone {port}: Assigned {len(drone_cells)} cells (Rows {start_row}-{end_row})", "PARTITION")
        
        return drone_assignments
    
    # ------------------------------------------------------------------
    # TELEMETRY & STATUS
    # ------------------------------------------------------------------

    def get_all_drones_status(self) -> Dict[int, Dict[str, Any]]:
        """Returns detailed status for all drones."""
        all_status = {}
        with self.lock:
            drones_snapshot = dict(self.drones)
            controllers_snapshot = dict(self.drone_controllers)

        for port, vehicle in drones_snapshot.items():
            if vehicle == "connecting":
                all_status[port] = {
                    "port": port,
                    "status": "connecting",
                    "is_active_camera": (self.active_drone_port == port)
                }
                continue
            
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
                "is_paused": False,
                "has_area": False,
                "mission_status": "Idle"
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
                    
                if port in controllers_snapshot:
                    controller = controllers_snapshot[port]
                    status_data["is_mission_active"] = controller.is_mission_active
                    status_data["is_paused"] = bool(getattr(controller, "is_paused", False))
                    status_data["has_area"] = bool(getattr(controller, "mission_coordinates", None))
                    status_data["mission_status"] = getattr(self, 'mission_status_message', 'Idle')
                    
            except Exception as e:
                SwarmLogger.log("WARNING", "DroneManager", f"Error reading status for Drone {port}: {e}", "STATUS")
            
            all_status[port] = status_data
        
        return all_status
    
    def get_status(self):
        """General system status including active drone details."""
        with self.lock:
            drones_snapshot = dict(self.drones)
            controllers_snapshot = dict(self.drone_controllers)
            active_drone = self.active_drone
            active_drone_port = self.active_drone_port
        connected_drones_list = []
        for port, vehicle_obj in drones_snapshot.items():
            connected_drones_list.append({
                "port": port,
                "is_active": (active_drone_port == port) if vehicle_obj != "connecting" else False
            })

        # Detailed status for all drones
        all_drones_status = self.get_all_drones_status()

        if not active_drone or active_drone == "connecting":
            return {
                "status": CONNECTION_STATUS_NOT_CONNECTED,
                "connected_drones": connected_drones_list,
                "all_drones": all_drones_status,
                "is_mission_active": any(
                    c.is_mission_active for c in controllers_snapshot.values()
                ),
                "status_message": self.mission_status_message
            }

        current_location = {"lat": 0, "lon": 0, "alt": 0}
        ground_speed = 0
        if hasattr(active_drone, 'location') and active_drone.location.global_relative_frame:
            loc = active_drone.location.global_relative_frame
            current_location = {"lat": loc.lat, "lon": loc.lon, "alt": loc.alt}
            ground_speed = active_drone.ground_speed if hasattr(active_drone, 'ground_speed') else 0

        try:
            return {
                "status": CONNECTION_STATUS_CONNECTED,
                "current_location": current_location,
                "ground_speed": ground_speed,
                "battery_level": active_drone.battery.level if hasattr(active_drone, 'battery') and active_drone.battery else 0,
                "mode": active_drone.mode.name if hasattr(active_drone, 'mode') and active_drone.mode else "UNKNOWN",
                "is_armed": active_drone.armed if hasattr(active_drone, 'armed') else False,
                "connected_drones": connected_drones_list,
                "all_drones": all_drones_status,
                "is_mission_active": any(
                    c.is_mission_active for c in controllers_snapshot.values()
                ),
                "status_message": self.mission_status_message
            }
        except Exception as e:
            SwarmLogger.log("ERROR", "DroneManager", f"FAILSAFE get_status error: {e}", "STATUS")
            return {
                "status": CONNECTION_STATUS_NOT_CONNECTED,
                "connected_drones": connected_drones_list,
                "all_drones": all_drones_status,
                "is_mission_active": any(
                    c.is_mission_active for c in controllers_snapshot.values()
                ),
                "status_message": f"Status error: {e}"
            }
