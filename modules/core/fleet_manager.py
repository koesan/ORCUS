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
    MISSION_STATUS_MESSAGES,
    SWARM_CORRIDOR_WIDTH_M,
    SWARM_CORRIDOR_TIME_HEADWAY_S,
    SWARM_CORRIDOR_TTL_S,
    SWARM_CORRIDOR_PRIORITY_DISTANCE_SCALE_M,
    SWARM_CORRIDOR_LANE_OFFSET_M,
    SWARM_CORRIDOR_LANE_CLEARANCE_M,
    SWARM_TRANSIT_ALTITUDE_STEP_M,
    SWARM_TRANSIT_ALTITUDE_BANDS,
    STATUS_SNAPSHOT_LOG_INTERVAL_S,
)

class DroneManager:
    """Multi-drone management with independent mission threads."""
    
    def __init__(self):
        self.drones = {}  # port -> vehicle mapping
        self.drone_controllers = {}  # port -> MissionController mapping
        self.active_drone = None
        self.active_drone_port = None
        self.connection_status = CONNECTION_STATUS_NOT_CONNECTED
        self.mission_status_message = MISSION_STATUS_MESSAGES["NOT_CONNECTED"]
        self.transit_altitude_slots = {}
        self.transit_lane_slots = {}
        
        # Lazy import — avoids core→vision circular dependency
        from modules.vision.camera_handler import CameraAIHandler
        self.camera_handler = CameraAIHandler()
        self.lock = threading.Lock()
        self.airspace_reservations = {}
        self._last_status_log_signature = None
        
        # Swarm Manager (lazy import)
        from modules.swarm.coordinator import SwarmManager
        self.swarm_manager = SwarmManager(self)

    # ------------------------------------------------------------------
    # AIRSPACE RESERVATION
    # ------------------------------------------------------------------

    @staticmethod
    def _airspace_priority(phase: Optional[str]) -> int:
        phase = str(phase or "").upper()
        if phase in {"ATTACK", "DIVING"}:
            return 0
        if phase in {"ATTACK_ABORT", "RTL"}:
            return 1
        if phase == "RETURN_TO_SCAN":
            return 2
        if phase == "TRANSIT":
            return 3
        if phase == "TAKEOFF":
            return 4
        return 5

    @staticmethod
    def _project_xy(lat: float, lon: float, ref_lat: float, ref_lon: float) -> Tuple[float, float]:
        meters_per_deg_lat = 111320.0
        meters_per_deg_lon = meters_per_deg_lat * math.cos(math.radians(ref_lat))
        return (
            (float(lon) - float(ref_lon)) * meters_per_deg_lon,
            (float(lat) - float(ref_lat)) * meters_per_deg_lat,
        )

    @classmethod
    def _segment_conflict_distance(
        cls,
        a0: Tuple[float, float],
        a1: Tuple[float, float],
        b0: Tuple[float, float],
        b1: Tuple[float, float],
    ) -> float:
        ref_lat = (a0[0] + a1[0] + b0[0] + b1[0]) / 4.0
        ref_lon = (a0[1] + a1[1] + b0[1] + b1[1]) / 4.0
        ax0, ay0 = cls._project_xy(a0[0], a0[1], ref_lat, ref_lon)
        ax1, ay1 = cls._project_xy(a1[0], a1[1], ref_lat, ref_lon)
        bx0, by0 = cls._project_xy(b0[0], b0[1], ref_lat, ref_lon)
        bx1, by1 = cls._project_xy(b1[0], b1[1], ref_lat, ref_lon)

        def point_seg(px, py, x0, y0, x1, y1):
            dx = x1 - x0
            dy = y1 - y0
            if abs(dx) < 1e-6 and abs(dy) < 1e-6:
                return math.hypot(px - x0, py - y0)
            t = ((px - x0) * dx + (py - y0) * dy) / max(dx * dx + dy * dy, 1e-6)
            t = max(0.0, min(1.0, t))
            qx = x0 + t * dx
            qy = y0 + t * dy
            return math.hypot(px - qx, py - qy)

        distances = (
            point_seg(ax0, ay0, bx0, by0, bx1, by1),
            point_seg(ax1, ay1, bx0, by0, bx1, by1),
            point_seg(bx0, by0, ax0, ay0, ax1, ay1),
            point_seg(bx1, by1, ax0, ay0, ax1, ay1),
        )
        return min(distances)

    def refresh_airspace_reservation(
        self,
        port: int,
        phase: Optional[str],
        start_lat: float,
        start_lon: float,
        end_lat: float,
        end_lon: float,
        altitude: float = 0.0,
        corridor_width_m: float = SWARM_CORRIDOR_WIDTH_M,
        ttl_s: float = SWARM_CORRIDOR_TTL_S,
    ):
        now = time.time()
        with self.lock:
            lane_offset = self.get_transit_lane_offset(port)
            self.airspace_reservations[port] = {
                "phase": str(phase or ""),
                "priority": self._airspace_priority(phase),
                "start": (float(start_lat), float(start_lon)),
                "end": (float(end_lat), float(end_lon)),
                "altitude": float(altitude or 0.0),
                "width_m": float(corridor_width_m),
                "lane_offset_m": float(lane_offset),
                "updated_at": now,
                "expires_at": now + max(0.5, float(ttl_s)),
            }

    def clear_airspace_reservation(self, port: int):
        with self.lock:
            self.airspace_reservations.pop(port, None)

    def has_peer_airspace_reservations(self, port: int) -> bool:
        now = time.time()
        with self.lock:
            stale = [p for p, res in self.airspace_reservations.items() if res.get("expires_at", 0.0) < now]
            for stale_port in stale:
                self.airspace_reservations.pop(stale_port, None)
            return any(other_port != port for other_port in self.airspace_reservations)

    def get_corridor_conflict(self, port: int, altitude_guard_m: float) -> Optional[Dict[str, float]]:
        now = time.time()
        with self.lock:
            stale = [p for p, res in self.airspace_reservations.items() if res.get("expires_at", 0.0) < now]
            for stale_port in stale:
                self.airspace_reservations.pop(stale_port, None)

            own = self.airspace_reservations.get(port)
            if own is None:
                return None

            own_start = own["start"]
            own_end = own["end"]
            own_remaining = self._segment_length_m(own_start, own_end)
            own_order = (
                int(own["priority"]),
                min(1.0, own_remaining / max(1.0, SWARM_CORRIDOR_PRIORITY_DISTANCE_SCALE_M)),
                int(port),
            )

            best_conflict = None
            for other_port, other in self.airspace_reservations.items():
                if other_port == port:
                    continue
                lane_sep = abs(float(own.get("lane_offset_m", 0.0)) - float(other.get("lane_offset_m", 0.0)))
                if (
                    abs(float(own["altitude"]) - float(other["altitude"])) > float(altitude_guard_m)
                    and lane_sep < SWARM_CORRIDOR_LANE_CLEARANCE_M
                ):
                    continue
                time_gap = abs(float(own["updated_at"]) - float(other["updated_at"]))
                if time_gap > SWARM_CORRIDOR_TIME_HEADWAY_S:
                    continue
                overlap_m = self._segment_conflict_distance(own_start, own_end, other["start"], other["end"])
                width_m = max(float(own["width_m"]), float(other["width_m"]))
                effective_width_m = max(1.0, width_m - lane_sep)
                if overlap_m > effective_width_m:
                    continue
                other_remaining = self._segment_length_m(other["start"], other["end"])
                other_order = (
                    int(other["priority"]),
                    min(1.0, other_remaining / max(1.0, SWARM_CORRIDOR_PRIORITY_DISTANCE_SCALE_M)),
                    int(other_port),
                )
                if own_order <= other_order:
                    continue
                candidate = {
                    "other_port": other_port,
                    "critical": overlap_m < (width_m * 0.5),
                    "lane_sep": lane_sep,
                    "lateral_sep": overlap_m,
                    "altitude_sep": abs(float(own["altitude"]) - float(other["altitude"])),
                }
                if best_conflict is None or candidate["lateral_sep"] < best_conflict["lateral_sep"]:
                    best_conflict = candidate
            return best_conflict

    @staticmethod
    def _segment_length_m(start: Tuple[float, float], end: Tuple[float, float]) -> float:
        ref_lat = (start[0] + end[0]) / 2.0
        ref_lon = (start[1] + end[1]) / 2.0
        sx, sy = DroneManager._project_xy(start[0], start[1], ref_lat, ref_lon)
        ex, ey = DroneManager._project_xy(end[0], end[1], ref_lat, ref_lon)
        return math.hypot(ex - sx, ey - sy)

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
                from modules.mission.mission_controller import MissionController
                controller = MissionController(self, assigned_port=port, swarm_manager=self.swarm_manager)
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
        """Partition grid cells among drones using contiguous, ordered scan stripes.

        Goals:
        - no duplicate cell coverage
        - preserve boustrophedon traversal inside each assigned stripe
        - reduce initial path crossing by matching stripe order to drone geometry
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
        
        stripe_axis = "row" if num_rows >= num_cols else "col"
        stripe_count = num_rows if stripe_axis == "row" else num_cols
        drone_order = self._ordered_scan_ports(ready_drones, stripe_axis, grid_2d, num_rows, num_cols)
        stripe_ranges = self._balanced_stripe_ranges(stripe_count, len(drone_order))

        drone_assignments = {}
        if not hasattr(self, "transit_altitude_slots") or self.transit_altitude_slots is None:
            self.transit_altitude_slots = {}
        if not hasattr(self, "transit_lane_slots") or self.transit_lane_slots is None:
            self.transit_lane_slots = {}
        for idx, port in enumerate(drone_order):
            start_idx, end_idx = stripe_ranges[idx]
            drone_cells = self._collect_stripe_cells(
                grid_2d, num_rows, num_cols, stripe_axis, start_idx, end_idx
            )
            drone_assignments[port] = drone_cells
            self.transit_altitude_slots[port] = idx
            self.transit_lane_slots[port] = idx
            SwarmLogger.log(
                "INFO",
                "DroneManager",
                f"Drone {port}: Assigned {len(drone_cells)} cells "
                f"({stripe_axis}s {start_idx}-{max(start_idx, end_idx - 1)})",
                "PARTITION",
            )

        return drone_assignments

    def get_transit_altitude_offset(self, port: int) -> float:
        slots = getattr(self, "transit_altitude_slots", None)
        if slots is None:
            slots = {}
            self.transit_altitude_slots = slots
        slot = int(slots.get(port, 0) or 0)
        band_count = max(1, int(SWARM_TRANSIT_ALTITUDE_BANDS))
        band = slot % band_count
        return float(band) * float(SWARM_TRANSIT_ALTITUDE_STEP_M)

    def get_transit_lane_offset(self, port: int) -> float:
        slots = getattr(self, "transit_lane_slots", None)
        if slots is None:
            slots = {}
            self.transit_lane_slots = slots
        slot = int(slots.get(port, 0) or 0)
        if slot <= 0:
            return 0.0
        magnitude = (slot + 1) // 2
        direction = 1.0 if (slot % 2) == 1 else -1.0
        return direction * magnitude * float(SWARM_CORRIDOR_LANE_OFFSET_M)

    def _ordered_scan_ports(
        self,
        ready_drones: List[int],
        stripe_axis: str,
        grid_2d: List[List[Optional[Tuple[float, float]]]],
        num_rows: int,
        num_cols: int,
    ) -> List[int]:
        """Order drones along the dominant scan axis to reduce crossing transits."""
        grid_points = [cell for row in grid_2d for cell in row if cell is not None]
        if not grid_points:
            return sorted(ready_drones)

        grid_lats = [pt[0] for pt in grid_points]
        grid_lons = [pt[1] for pt in grid_points]
        lat_center = sum(grid_lats) / len(grid_lats)
        lon_center = sum(grid_lons) / len(grid_lons)

        # Determine whether stripe index increases north->south or west->east.
        if stripe_axis == "row":
            first_point = next((grid_2d[r][0] for r in range(num_rows) if grid_2d[r][0] is not None), grid_points[0])
            axis_sign = -1.0 if first_point[0] >= lat_center else 1.0
        else:
            first_point = next((grid_2d[0][c] for c in range(num_cols) if grid_2d[0][c] is not None), grid_points[0])
            axis_sign = -1.0 if first_point[1] >= lon_center else 1.0

        def sort_key(port: int):
            vehicle = self.drones.get(port)
            if vehicle == "connecting":
                return (1, port)
            try:
                loc = vehicle.location.global_relative_frame
                if stripe_axis == "row":
                    return (0, axis_sign * float(loc.lat or 0.0))
                return (0, axis_sign * float(loc.lon or 0.0))
            except Exception:
                return (1, port)

        return [port for port in sorted(ready_drones, key=sort_key)]

    @staticmethod
    def _balanced_stripe_ranges(stripe_count: int, drone_count: int) -> List[Tuple[int, int]]:
        """Return contiguous stripe ranges distributed as evenly as possible."""
        if drone_count <= 0:
            return []
        base = stripe_count // drone_count
        remainder = stripe_count % drone_count
        ranges = []
        cursor = 0
        for idx in range(drone_count):
            width = base + (1 if idx < remainder else 0)
            start = cursor
            end = cursor + width
            ranges.append((start, end))
            cursor = end
        return ranges

    @staticmethod
    def _collect_stripe_cells(
        grid_2d: List[List[Optional[Tuple[float, float]]]],
        num_rows: int,
        num_cols: int,
        stripe_axis: str,
        start_idx: int,
        end_idx: int,
    ) -> List[Tuple[float, float]]:
        """Collect cells from a contiguous stripe while preserving sweep order."""
        if start_idx >= end_idx:
            return []

        cells: List[Tuple[float, float]] = []
        if stripe_axis == "row":
            for r in range(start_idx, min(end_idx, num_rows)):
                cols = range(num_cols) if r % 2 == 0 else range(num_cols - 1, -1, -1)
                for c in cols:
                    cell = grid_2d[r][c]
                    if cell is not None:
                        cells.append(cell)
            return cells

        for c in range(start_idx, min(end_idx, num_cols)):
            rows = range(num_rows) if c % 2 == 0 else range(num_rows - 1, -1, -1)
            for r in rows:
                cell = grid_2d[r][c]
                if cell is not None:
                    cells.append(cell)
        return cells
    
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
                    status_data["mission_phase"] = getattr(controller, "mission_phase", "IDLE")
                    try:
                        controller_status = controller.get_status()
                        status_data["mission_status"] = controller_status.get("status_message", status_data["mission_status"])
                        status_data["route_visual"] = controller_status.get("route_visual")
                        status_data["mission_phase"] = controller_status.get("mission_phase", status_data["mission_phase"])
                        status_data["planned_terminal_point"] = controller_status.get("planned_terminal_point")
                        status_data["terminal_event"] = controller_status.get("terminal_event")
                        status_data["attack_phase"] = controller_status.get("attack_phase")
                        status_data["attack_target_id"] = controller_status.get("attack_target_id")
                        status_data["attack_autonomous"] = controller_status.get("attack_autonomous", False)
                    except Exception:
                        status_data["route_visual"] = None
                        status_data["planned_terminal_point"] = None
                        status_data["terminal_event"] = None
                else:
                    status_data["route_visual"] = None
                    status_data["planned_terminal_point"] = None
                    status_data["terminal_event"] = None
                    
            except Exception as e:
                SwarmLogger.log("WARNING", "DroneManager", f"Error reading status for Drone {port}: {e}", "STATUS")
            
            all_status[port] = status_data
        
        return all_status

    def _log_fleet_status_snapshot(self, all_status: Dict[int, Dict[str, Any]]) -> None:
        ports = sorted(all_status.keys())
        connected = 0
        missioning = 0
        tracking = 0
        phases = []
        for port in ports:
            status = all_status.get(port) or {}
            if status.get("status") == "connected":
                connected += 1
            if status.get("is_mission_active"):
                missioning += 1
            if status.get("mission_phase") in {"TRACK", "ATTACK"}:
                tracking += 1
            phases.append(f"{port}:{status.get('mission_phase', '-')}")
        signature = (
            tuple(ports),
            connected,
            missioning,
            tracking,
            self.active_drone_port,
            tuple(phases),
            self.mission_status_message,
        )
        if signature == self._last_status_log_signature:
            return
        self._last_status_log_signature = signature
        SwarmLogger.log_throttled(
            "STATUS",
            "DroneManager",
            f"Fleet snapshot | connected={connected}/{len(ports)} mission_active={missioning} "
            f"tracking_or_attack={tracking} active_camera={self.active_drone_port} "
            f"phases={','.join(phases) if phases else '-'} status=\"{self.mission_status_message}\"",
            "STATUS",
            interval_s=STATUS_SNAPSHOT_LOG_INTERVAL_S,
        )
    
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
        self._log_fleet_status_snapshot(all_drones_status)

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
