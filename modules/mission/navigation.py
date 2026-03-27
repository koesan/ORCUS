"""Mission navigation, transit control, and scan movement."""

import math
import time
from pymavlink import mavutil
from collections import deque
from statistics import median

from modules.mission.flight_controller import FlightController
from modules.core.geo_math import GeoMath
from modules.core.pid_controller import PID
from modules.core.logger import SwarmLogger
from config import (
    MISSION_STATUS_MESSAGES,
    PID_KP, PID_KI, PID_KD,
    MAX_SPEED_M_S, CONTROL_INTERVAL_S, CELL_REACHED_THRESHOLD_M,
    FINE_APPROACH_THRESHOLD_M, FINE_APPROACH_SCALE,
    CENTER_CONFIRM_TOLERANCE_M, CENTER_CONFIRM_HOLD_S,
    FINE_APPROACH_HOLD_S, PER_CELL_TIMEOUT_S,
    RETRY_LIMIT, GPS_MEDIAN_WINDOW,
    STUCK_MOVED_THRESHOLD_M, STUCK_TIMEOUT_S,
    STUCK_MANEUVER_SCALE_BACK, STUCK_MANEUVER_SCALE_FWD,
    STUCK_MANEUVER_DURATION, TIMEOUT_PENALTY_SCALE,
    SWARM_DRONE_SEPARATION_MIN_M, SWARM_TRANSIT_RELEASE_DISTANCE_M,
    SWARM_TRANSIT_ALTITUDE_GUARD_M,
    SWARM_TRANSIT_PREDICTION_HORIZON_S,
    TAKEOFF_ALTITUDE, SWARM_CORRIDOR_FLOW_HOLD_S,
    HOME_LATITUDE, HOME_LONGITUDE,
    ROTATION_YAW_INCREMENT, ROTATION_YAW_SPEED, ROTATION_DIRECTION,
    SCAN_DETECTION_POLL_INTERVAL_S, SCAN_YAW_SETTLE_MARGIN_S,
)


class Scanner:
    """360-degree scan controller."""

    def __init__(self, port=None, flight_ctrl=None):
        self.port = port
        self.flight_ctrl = flight_ctrl

    def perform_360_rotation_scan(self, vehicle, is_active_check=None,
                                  detection_check=None, status_callback=None,
                                  pause_check=None, pause_waiter=None):
        """Run a scan sweep and stop on detection."""
        try:
            if status_callback:
                status_callback(MISSION_STATUS_MESSAGES["ROTATING_360"])
            SwarmLogger.log("INFO", f"DRONE_{self.port}",
                            MISSION_STATUS_MESSAGES["ROTATING_360"], "SCANNER")

            rotated_total = 0
            if self._poll_detection(
                detection_check,
                is_active_check,
                status_callback,
                pause_check=pause_check,
                pause_waiter=pause_waiter,
            ):
                return True

            while rotated_total < 360:
                if pause_check and pause_check():
                    if pause_waiter:
                        pause_waiter()
                    continue
                if is_active_check and not is_active_check():
                    return False

                rotated_total += ROTATION_YAW_INCREMENT
                try:
                    if self.flight_ctrl:
                        self.flight_ctrl.command_scan_yaw(
                            vehicle, ROTATION_YAW_INCREMENT,
                            ROTATION_YAW_SPEED, ROTATION_DIRECTION,
                            reason="360 scan step")
                    else:
                        msg = vehicle.message_factory.command_long_encode(
                            0, 0, mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0,
                            ROTATION_YAW_INCREMENT, ROTATION_YAW_SPEED,
                            ROTATION_DIRECTION, 1, 0, 0, 0)
                        vehicle.send_mavlink(msg)
                        vehicle.flush()
                except Exception as exc:
                    SwarmLogger.log("ERROR", f"DRONE_{self.port}",
                                    f"Rotation command error: {exc}", "SCANNER")

                wait_time = (ROTATION_YAW_INCREMENT / max(ROTATION_YAW_SPEED, 1e-3)
                             + SCAN_YAW_SETTLE_MARGIN_S)
                deadline = time.time() + wait_time
                while time.time() < deadline:
                    if pause_check and pause_check():
                        if pause_waiter:
                            pause_waiter()
                        deadline = time.time() + SCAN_YAW_SETTLE_MARGIN_S
                        continue
                    if is_active_check and not is_active_check():
                        return False
                    if self._poll_detection(
                        detection_check,
                        is_active_check,
                        status_callback,
                        pause_check=pause_check,
                        pause_waiter=pause_waiter,
                    ):
                        return True
                    remaining = deadline - time.time()
                    if remaining <= 0:
                        break
                    time.sleep(min(SCAN_DETECTION_POLL_INTERVAL_S, remaining))
            return False
        finally:
            if self.flight_ctrl:
                self.flight_ctrl.command_hold(vehicle, reason="scan complete")

    def _poll_detection(self, detection_check=None, is_active_check=None,
                        status_callback=None, pause_check=None, pause_waiter=None):
        """Run one scan-mode detection probe."""
        if pause_check and pause_check():
            if pause_waiter:
                pause_waiter()
            return False
        if is_active_check and not is_active_check():
            return False
        if detection_check and detection_check():
            SwarmLogger.log("SUCCESS", f"DRONE_{self.port}",
                            MISSION_STATUS_MESSAGES["HUMAN_DETECTED"], "SCANNER")
            if status_callback:
                status_callback(MISSION_STATUS_MESSAGES["HUMAN_DETECTED"])
            return True
        return False


class Navigation:
    """Grid-based navigation with boustrophedon path planning and PID control."""

    def __init__(self, flight_ctrl: FlightController, status_callback=None, drone_manager=None):
        self.flight_ctrl = flight_ctrl
        self._status_cb = status_callback
        self._drone_manager = drone_manager
        self.scanner = Scanner(port=getattr(flight_ctrl, "port", None), flight_ctrl=flight_ctrl)
        self.centers_2d = []
        self.grid_status = {}
        self.num_rows = 0
        self.num_cols = 0
        self.mission_coordinates = None
        self._transit_sep_memory = {}
        self._transit_yield_peers = set()
        self._last_route_visual_signature = None

    # ------------------------------------------------------------------
    # Status
    # ------------------------------------------------------------------

    def _status(self, msg):
        if self._status_cb:
            self._status_cb(msg)

    # ------------------------------------------------------------------
    # Grid creation
    # ------------------------------------------------------------------

    def create_grid(self, cell_centers):
        """Create traversal grid from list of (lat, lon) cell centers."""
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

        self._status(MISSION_STATUS_MESSAGES["AREA_SET"].format(rows=num_rows, cols=num_cols))
        SwarmLogger.log("INFO", "Navigation", f"Grid created: {num_rows}x{num_cols}", "GRID")

    def reset_runtime_state(self):
        """Clear mission-local grid state without touching live vehicle links."""
        self.centers_2d = []
        self.grid_status = {}
        self.num_rows = 0
        self.num_cols = 0
        self.mission_coordinates = None
        self._last_route_visual_signature = None

    def compute_scan_route_points(self):
        """Return assigned scan cells in traversal order."""
        if self.num_rows <= 0 or self.num_cols <= 0 or not self.centers_2d:
            return []

        total_cells = sum(
            1 for r in range(self.num_rows) for c in range(self.num_cols)
            if self.centers_2d[r][c] is not None
        )
        if total_cells <= 0:
            return []

        if total_cells == 1:
            for r in range(self.num_rows):
                for c in range(self.num_cols):
                    target = self.centers_2d[r][c]
                    if target is not None:
                        return [{"lat": float(target[0]), "lon": float(target[1]), "row": r, "col": c}]
            return []

        route = []
        cur_row, cur_col = self.num_rows - 1, 0
        horiz_dir, vert_dir = 1, -1
        visited = set()

        while 0 <= cur_row < self.num_rows and 0 <= cur_col < self.num_cols:
            key = (cur_row, cur_col)
            if key in visited:
                break
            visited.add(key)
            target = self.centers_2d[cur_row][cur_col]
            if target is not None:
                route.append({
                    "lat": float(target[0]),
                    "lon": float(target[1]),
                    "row": cur_row,
                    "col": cur_col,
                })
            cur_row, cur_col, horiz_dir, vert_dir = self.next_indices(
                cur_row, cur_col, horiz_dir, vert_dir
            )
            if cur_row == -1 and cur_col == -1:
                break

        return route

    @staticmethod
    def _phase_priority(phase: str) -> int:
        phase = str(phase or "").upper()
        if phase == "RTL":
            return 0
        if phase in {"ATTACK_ABORT", "RETURN_TO_SCAN"}:
            return 1
        if phase == "TRANSIT":
            return 2
        if phase == "TAKEOFF":
            return 3
        return 4

    def _transit_conflict(self, vehicle, target_lat=None, target_lon=None):
        """Return higher-priority drone context if we should yield during transit."""
        if self._drone_manager is None:
            return None

        own_port = getattr(self.flight_ctrl, "port", None)
        if own_port is None:
            return None

        try:
            own_loc = vehicle.location.global_relative_frame
            own_lat = float(own_loc.lat)
            own_lon = float(own_loc.lon)
            own_alt = float(getattr(own_loc, "alt", 0.0) or 0.0)
        except Exception:
            return None

        controllers = getattr(self._drone_manager, "drone_controllers", {}) or {}
        own_phase = None
        own_target_remaining = float("inf")
        own_ctrl = controllers.get(own_port)
        if own_ctrl is not None:
            own_phase = getattr(own_ctrl, "mission_phase", None)
            if target_lat is None or target_lon is None:
                own_target = getattr(own_ctrl, "current_nav_target", None)
                if own_target is not None:
                    target_lat, target_lon = own_target
        if target_lat is not None and target_lon is not None:
            own_target_remaining = GeoMath.haversine_distance(
                own_lat, own_lon, float(target_lat), float(target_lon)
            )
            reserve_alt = self._desired_transit_altitude(vehicle)
            refresh_fn = getattr(self._drone_manager, "refresh_airspace_reservation", None)
            if callable(refresh_fn):
                refresh_fn(
                    own_port,
                    own_phase,
                    own_lat,
                    own_lon,
                    float(target_lat),
                    float(target_lon),
                    altitude=reserve_alt,
                )
            corridor_conflict = getattr(self._drone_manager, "get_corridor_conflict", None)
            if callable(corridor_conflict):
                conflict = corridor_conflict(own_port, SWARM_TRANSIT_ALTITUDE_GUARD_M)
                if conflict:
                    return conflict
        for other_port, other_vehicle in getattr(self._drone_manager, "drones", {}).items():
            if other_port == own_port or other_vehicle == "connecting":
                continue

            try:
                other_loc = other_vehicle.location.global_relative_frame
                other_lat = float(other_loc.lat)
                other_lon = float(other_loc.lon)
                other_alt = float(getattr(other_loc, "alt", 0.0) or 0.0)
            except Exception:
                continue

            other_ctrl = controllers.get(other_port)
            other_phase = str(getattr(other_ctrl, "mission_phase", "") or "").upper()
            if other_phase not in {"TAKEOFF", "TRANSIT", "RETURN_TO_SCAN"}:
                continue

            lateral_sep = GeoMath.haversine_distance(own_lat, own_lon, other_lat, other_lon)
            if lateral_sep > SWARM_TRANSIT_RELEASE_DISTANCE_M:
                self._transit_yield_peers.discard(other_port)
                continue

            own_sep_alt = self._desired_transit_altitude(vehicle) if str(own_phase or "").upper() in {"TAKEOFF", "TRANSIT", "RETURN_TO_SCAN"} else own_alt
            other_sep_alt = self._peer_transit_altitude(other_port, other_vehicle)
            altitude_sep = abs(own_sep_alt - other_sep_alt)
            if altitude_sep > SWARM_TRANSIT_ALTITUDE_GUARD_M:
                self._transit_yield_peers.discard(other_port)
                continue

            now = time.time()
            prev_key = (own_port, other_port)
            prev_sample = self._transit_sep_memory.get(prev_key)
            self._transit_sep_memory[prev_key] = (now, lateral_sep)
            closing_speed = 0.0
            if prev_sample is not None:
                prev_ts, prev_sep = prev_sample
                dt = max(now - float(prev_ts), 1e-3)
                closing_speed = max(0.0, (float(prev_sep) - lateral_sep) / dt)
            predicted_sep = max(0.0, lateral_sep - (closing_speed * SWARM_TRANSIT_PREDICTION_HORIZON_S))

            other_target_remaining = float("inf")
            if other_ctrl is not None:
                other_target = getattr(other_ctrl, "current_nav_target", None)
                if other_target is not None:
                    other_target_remaining = GeoMath.haversine_distance(
                        other_lat, other_lon, float(other_target[0]), float(other_target[1])
                    )

            own_priority = (self._phase_priority(own_phase), own_target_remaining, int(own_port))
            other_priority = (self._phase_priority(other_phase), other_target_remaining, int(other_port))

            should_hold_existing_yield = (
                other_port in self._transit_yield_peers
                and lateral_sep <= SWARM_TRANSIT_RELEASE_DISTANCE_M
            )
            should_preemptively_yield = predicted_sep <= (SWARM_DRONE_SEPARATION_MIN_M * 1.25)

            if should_hold_existing_yield or (own_priority > other_priority and should_preemptively_yield):
                self._transit_yield_peers.add(other_port)
                return {
                    "other_port": other_port,
                    "lateral_sep": lateral_sep,
                    "predicted_sep": predicted_sep,
                    "closing_speed": closing_speed,
                    "altitude_sep": altitude_sep,
                    "critical": (
                        lateral_sep < SWARM_DRONE_SEPARATION_MIN_M
                        or predicted_sep < SWARM_DRONE_SEPARATION_MIN_M
                    ),
                }

            if own_priority > other_priority:
                self._transit_yield_peers.add(other_port)
                return {
                    "other_port": other_port,
                    "lateral_sep": lateral_sep,
                    "predicted_sep": predicted_sep,
                    "closing_speed": closing_speed,
                    "altitude_sep": altitude_sep,
                    "critical": lateral_sep < SWARM_DRONE_SEPARATION_MIN_M,
                }
            self._transit_yield_peers.discard(other_port)

        return None

    def _desired_transit_altitude(self, vehicle) -> float:
        try:
            current_alt = float(getattr(vehicle.location.global_relative_frame, "alt", 0.0) or 0.0)
        except Exception:
            current_alt = float(TAKEOFF_ALTITUDE)
        offset_fn = getattr(self._drone_manager, "get_transit_altitude_offset", None)
        if callable(offset_fn):
            try:
                offset = float(offset_fn(getattr(self.flight_ctrl, "port", None)) or 0.0)
                return max(float(TAKEOFF_ALTITUDE), current_alt, float(TAKEOFF_ALTITUDE) + offset)
            except Exception:
                pass
        return max(float(TAKEOFF_ALTITUDE), current_alt)

    def _peer_transit_altitude(self, port, vehicle) -> float:
        try:
            current_alt = float(getattr(vehicle.location.global_relative_frame, "alt", 0.0) or 0.0)
        except Exception:
            current_alt = float(TAKEOFF_ALTITUDE)
        offset_fn = getattr(self._drone_manager, "get_transit_altitude_offset", None)
        if callable(offset_fn):
            try:
                offset = float(offset_fn(port) or 0.0)
                return max(float(TAKEOFF_ALTITUDE), current_alt, float(TAKEOFF_ALTITUDE) + offset)
            except Exception:
                pass
        return max(float(TAKEOFF_ALTITUDE), current_alt)

    def _clear_transit_reservation(self):
        clear_fn = getattr(self._drone_manager, "clear_airspace_reservation", None)
        if callable(clear_fn):
            try:
                clear_fn(getattr(self.flight_ctrl, "port", None))
            except Exception:
                pass

    def clear_transit_reservation(self):
        """Clear the current transit reservation."""
        self._clear_transit_reservation()

    def home_target(self, vehicle):
        """Return home coordinates for RTL."""
        try:
            home = getattr(vehicle, "home_location", None)
            if home and getattr(home, "lat", None) is not None and getattr(home, "lon", None) is not None:
                return float(home.lat), float(home.lon)
        except Exception:
            pass
        return float(HOME_LATITUDE), float(HOME_LONGITUDE)

    def wait_for_rtl_slot(self, vehicle, timeout_s: float = 3.0):
        """Yield until the RTL corridor is clear."""
        refresh_fn = getattr(self._drone_manager, "refresh_airspace_reservation", None)
        conflict_fn = getattr(self._drone_manager, "get_corridor_conflict", None)
        own_port = getattr(self.flight_ctrl, "port", None)
        if not callable(refresh_fn) or not callable(conflict_fn) or vehicle is None or vehicle == "connecting":
            return

        try:
            loc = vehicle.location.global_relative_frame
            if not loc or loc.lat is None or loc.lon is None:
                return
            home_lat, home_lon = self.home_target(vehicle)
            start = time.time()
            while time.time() - start < max(0.2, float(timeout_s)):
                refresh_fn(
                    own_port,
                    "RTL",
                    float(loc.lat),
                    float(loc.lon),
                    home_lat,
                    home_lon,
                    altitude=float(getattr(loc, "alt", 0.0) or 0.0),
                    ttl_s=max(4.0, float(timeout_s) + 1.0),
                )
                conflict = conflict_fn(own_port, altitude_guard_m=1.5)
                if not conflict:
                    return
                other_port = conflict.get("other_port", "?")
                try:
                    lateral_sep = float(conflict.get("lateral_sep", 0.0) or 0.0)
                except (TypeError, ValueError):
                    lateral_sep = 0.0
                try:
                    altitude_sep = float(conflict.get("altitude_sep", 0.0) or 0.0)
                except (TypeError, ValueError):
                    altitude_sep = 0.0
                self.flight_ctrl.command_hold(
                    vehicle,
                    reason=f"rtl slot wait for {other_port}",
                    failsafe=True,
                )
                SwarmLogger.log_throttled(
                    "COLLISION",
                    f"DRONE_{own_port}",
                    f"RTL slot wait: yielding to Drone_{other_port} | sep={lateral_sep:.1f}m altΔ={altitude_sep:.1f}m",
                    "NAVIGATION",
                    interval_s=1.0,
                )
                time.sleep(0.2)
        except Exception as exc:
            SwarmLogger.log(
                "WARNING",
                f"DRONE_{own_port}",
                f"RTL slot coordination failed: {exc}",
                "NAVIGATION",
            )

    def build_route_visual(self, current_location=None, mission_phase=None,
                           current_nav_target=None, vehicle=None):
        """Build mission route presentation data."""
        scan_route = self.compute_scan_route_points()
        phase = str(mission_phase or "IDLE")

        current_lat = current_lon = None
        if current_location:
            try:
                current_lat = float(current_location.get("lat"))
                current_lon = float(current_location.get("lon"))
            except (TypeError, ValueError, AttributeError):
                current_lat = current_lon = None

        transit_target = None
        transit_phases = {"TAKEOFF", "TRANSIT", "RETURN_TO_SCAN", "ATTACK_ABORT", "RTL"}
        if phase == "RTL":
            if vehicle and vehicle != "connecting":
                home_lat, home_lon = self.home_target(vehicle)
                transit_target = {"lat": float(home_lat), "lon": float(home_lon), "kind": "home"}
        elif phase in transit_phases and current_nav_target is not None:
            transit_target = {
                "lat": float(current_nav_target[0]),
                "lon": float(current_nav_target[1]),
                "kind": "mission",
            }
        elif phase in transit_phases and scan_route and current_lat is not None and current_lon is not None:
            first_point = scan_route[0]
            transit_target = {
                "lat": float(first_point["lat"]),
                "lon": float(first_point["lon"]),
                "kind": "entry",
            }

        transit_path = []
        if current_lat is not None and current_lon is not None and transit_target is not None:
            transit_path = [
                {"lat": current_lat, "lon": current_lon},
                {"lat": float(transit_target["lat"]), "lon": float(transit_target["lon"])},
            ]

        if len(scan_route) == 1:
            center = scan_route[0]
            ring = []
            for d_north, d_east in ((3.0, 0.0), (0.0, 3.0), (-3.0, 0.0), (0.0, -3.0), (3.0, 0.0)):
                lat_r, lon_r = GeoMath.ned_to_gps(center["lat"], center["lon"], d_north, d_east)
                ring.append({"lat": lat_r, "lon": lon_r})
            scan_route = [center, *ring]

        route_signature = (
            phase,
            round(float(transit_target["lat"]), 6) if transit_target else None,
            round(float(transit_target["lon"]), 6) if transit_target else None,
            len(scan_route),
        )
        own_port = getattr(self.flight_ctrl, "port", None)
        if route_signature != self._last_route_visual_signature:
            transit_kind = transit_target.get("kind") if transit_target else "none"
            SwarmLogger.log_throttled(
                "STATE",
                f"DRONE_{own_port}",
                f"Route visual updated | phase={phase} transit={transit_kind} scan_points={len(scan_route)}",
                "ROUTE",
                interval_s=1.0,
            )
            self._last_route_visual_signature = route_signature

        return {
            "phase": phase,
            "transit_path": transit_path,
            "scan_path": [{"lat": p["lat"], "lon": p["lon"]} for p in scan_route],
            "current_nav_target": transit_target,
        }

    def _apply_transit_lane_offset(self, own_lat, own_lon, target_lat, target_lon, dist_m):
        if self._drone_manager is None or dist_m <= 0.0:
            return target_lat, target_lon
        lane_fn = getattr(self._drone_manager, "get_transit_lane_offset", None)
        if not callable(lane_fn):
            return target_lat, target_lon
        try:
            lane_offset_m = float(lane_fn(getattr(self.flight_ctrl, "port", None)) or 0.0)
        except Exception:
            return target_lat, target_lon
        if abs(lane_offset_m) < 0.1:
            return target_lat, target_lon

        fade = max(0.0, min(1.0, (dist_m - CELL_REACHED_THRESHOLD_M) / max(SWARM_TRANSIT_RELEASE_DISTANCE_M, 1.0)))
        applied_offset = lane_offset_m * fade
        if abs(applied_offset) < 0.1:
            return target_lat, target_lon

        d_north, d_east = FlightController.latlon_to_north_east_m(own_lat, own_lon, target_lat, target_lon)
        norm = max(math.hypot(d_north, d_east), 1e-3)
        left_north = -d_east / norm
        left_east = d_north / norm
        offset_north = left_north * applied_offset
        offset_east = left_east * applied_offset
        meters_per_deg_lat = 111320.0
        meters_per_deg_lon = meters_per_deg_lat * math.cos(math.radians(target_lat))
        return (
            float(target_lat) + (offset_north / max(meters_per_deg_lat, 1e-6)),
            float(target_lon) + (offset_east / max(meters_per_deg_lon, 1e-6)),
        )

    # ------------------------------------------------------------------
    # Boustrophedon traversal
    # ------------------------------------------------------------------

    def next_indices(self, row, col, horiz_dir, vert_dir):
        """Compute next cell indices in boustrophedon pattern."""
        if self.num_rows <= 0 or self.num_cols <= 0:
            return -1, -1, horiz_dir, vert_dir

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

        if not (0 <= new_row < self.num_rows):
            return -1, -1, horiz_dir, vert_dir

        return new_row, new_col, horiz_dir, vert_dir

    # ------------------------------------------------------------------
    # GPS filtering
    # ------------------------------------------------------------------

    @staticmethod
    def get_median_location(vehicle, samples_deque):
        """Get median-filtered GPS position from sample buffer."""
        if not samples_deque:
            loc = vehicle.location.global_relative_frame
            return (loc.lat, loc.lon) if loc else (None, None)
        lats = [s[0] for s in samples_deque]
        lons = [s[1] for s in samples_deque]
        return median(lats), median(lons)

    # ------------------------------------------------------------------
    # Position stability
    # ------------------------------------------------------------------

    def confirm_center_stability(self, vehicle, target_lat, target_lon,
                                  hold_s=None, tol_m=None, is_active_check=None,
                                  pause_check=None, pause_waiter=None):
        """Hold position and verify drone stays within tolerance."""
        hold_s = CENTER_CONFIRM_HOLD_S if hold_s is None else hold_s
        tol_m = CENTER_CONFIRM_TOLERANCE_M if tol_m is None else tol_m
        start = time.time()

        while time.time() - start < hold_s:
            if pause_check and pause_check():
                if pause_waiter:
                    pause_waiter()
                start = time.time()
                continue
            if is_active_check and not is_active_check():
                return False
            loc = vehicle.location.global_relative_frame
            if not loc:
                return False
            d = GeoMath.haversine_distance(loc.lat, loc.lon, target_lat, target_lon)
            self.flight_ctrl.command_hold(vehicle, reason="center stability hold")
            if d > tol_m:
                return False
            time.sleep(0.05)
        return True

    # ------------------------------------------------------------------
    # Approach with PID + stuck detection
    # ------------------------------------------------------------------

    def approach_target(self, vehicle, target_lat, target_lon,
                        from_row, from_col, to_row, to_col,
                        is_active_check=None, scanning_check=None,
                        pause_check=None, pause_waiter=None):
        """Navigate to target GPS with PID control and stuck detection."""
        port = self.flight_ctrl.port

        try:
            for attempt in range(RETRY_LIMIT + 1):
                pid_n = PID(kp=PID_KP, ki=PID_KI, kd=PID_KD)
                pid_e = PID(kp=PID_KP, ki=PID_KI, kd=PID_KD)

                samples = deque(maxlen=GPS_MEDIAN_WINDOW)
                start_ts = time.time()
                last_move_ts = time.time()
                last_pos = None

                while True:
                    if pause_check and pause_check():
                        if pause_waiter:
                            pause_waiter()
                        continue
                    if is_active_check and not is_active_check():
                        return False

                    loc = vehicle.location.global_relative_frame
                    if loc and loc.lat is not None:
                        samples.append((loc.lat, loc.lon))

                    conflict = self._transit_conflict(vehicle, target_lat=target_lat, target_lon=target_lon)
                    if conflict:
                        severity = "critical" if conflict["critical"] else "yield"
                        SwarmLogger.log_throttled(
                            "COLLISION",
                            f"DRONE_{port}",
                            f"Transit deconfliction {severity}: yielding to Drone_{conflict['other_port']} "
                            f"| sep={conflict['lateral_sep']:.1f}m laneΔ={float(conflict.get('lane_sep', 0.0)):.1f}m "
                            f"altΔ={conflict['altitude_sep']:.1f}m pred={float(conflict.get('predicted_sep', conflict['lateral_sep'])):.1f}m "
                            f"closing={float(conflict.get('closing_speed', 0.0)):.1f}m/s",
                            "NAVIGATION",
                            interval_s=1.0,
                        )
                        self.flight_ctrl.command_hold(
                            vehicle, reason=f"transit deconflict yield to {conflict['other_port']}", failsafe=True
                        )
                        time.sleep(min(CONTROL_INTERVAL_S, SWARM_CORRIDOR_FLOW_HOLD_S))
                        continue

                    # Check for human detection during cruise
                    if scanning_check and scanning_check():
                        SwarmLogger.log("SUCCESS", f"DRONE_{port}", "CRUISE: Human detected!", "DETECTION")
                        return False

                    if not samples:
                        time.sleep(CONTROL_INTERVAL_S)
                        continue

                    filt_lat, filt_lon = self.get_median_location(vehicle, samples)
                    d_north, d_east = FlightController.latlon_to_north_east_m(
                        filt_lat, filt_lon, target_lat, target_lon)
                    dist = math.hypot(d_north, d_east)
                    nav_target_lat, nav_target_lon = self._apply_transit_lane_offset(
                        filt_lat, filt_lon, target_lat, target_lon, dist
                    )
                    d_north, d_east = FlightController.latlon_to_north_east_m(
                        filt_lat, filt_lon, nav_target_lat, nav_target_lon
                    )
                    speed = math.hypot(vehicle.velocity[0], vehicle.velocity[1])

                    self._status(MISSION_STATUS_MESSAGES["APPROACHING"].format(
                        from_row=from_row, from_col=from_col,
                        to_col=to_col, to_row=to_row, distance=dist, speed=speed))
                    SwarmLogger.log_throttled(
                        "STATE",
                        f"DRONE_{port}",
                        f"Transit progress | cell=({to_row},{to_col}) attempt={attempt + 1}/{RETRY_LIMIT + 1} "
                        f"dist={dist:.1f}m speed={speed:.2f}m/s target=({target_lat:.6f},{target_lon:.6f})",
                        "NAV_PROGRESS",
                        interval_s=1.5,
                    )

                    # Cell reached check
                    if dist <= CELL_REACHED_THRESHOLD_M:
                        self.flight_ctrl.command_hold(vehicle, reason="cell reached hold")
                        hold_ok = self.confirm_center_stability(
                            vehicle, target_lat, target_lon,
                            hold_s=FINE_APPROACH_HOLD_S, tol_m=CELL_REACHED_THRESHOLD_M,
                            is_active_check=is_active_check,
                            pause_check=pause_check,
                            pause_waiter=pause_waiter)
                        if hold_ok:
                            SwarmLogger.log(
                                "SUCCESS", f"DRONE_{port}",
                                f"Transit complete | cell=({to_row},{to_col}) dist={dist:.1f}m "
                                f"attempt={attempt + 1}",
                                "NAVIGATION",
                            )
                            return True
                        else:
                            SwarmLogger.log_throttled(
                                "WARNING", f"DRONE_{port}",
                                f"Center hold unstable after arrival | cell=({to_row},{to_col}) retrying fine approach",
                                "NAVIGATION",
                                interval_s=2.0,
                            )
                            pid_n.reset()
                            pid_e.reset()
                            time.sleep(0.12)
                            continue

                    # PID velocity computation
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

                    desired_alt = self._desired_transit_altitude(vehicle)
                    alt_error = desired_alt - float(getattr(loc, "alt", 0.0) or 0.0)
                    if abs(alt_error) > 0.25:
                        vz = -max(-0.6, min(0.6, alt_error * 0.5))
                    else:
                        vz = 0.0

                    try:
                        self.flight_ctrl.command_world_velocity(
                            vehicle, "TRANSIT", vx, vy, vz, reason=f"cell {to_row},{to_col}"
                        )
                    except Exception as e:
                        SwarmLogger.log("ERROR", f"DRONE_{port}", f"Velocity send error: {e}", "NAVIGATION")

                    # Stuck detection
                    if last_pos is None and loc:
                        last_pos = (loc.lat, loc.lon)
                        last_move_ts = time.time()
                    elif loc:
                        moved = GeoMath.haversine_distance(
                            last_pos[0], last_pos[1], loc.lat, loc.lon)
                        if moved > STUCK_MOVED_THRESHOLD_M:
                            last_pos = (loc.lat, loc.lon)
                            last_move_ts = time.time()
                        elif time.time() - last_move_ts > STUCK_TIMEOUT_S:
                            self._status(MISSION_STATUS_MESSAGES["STUCK"])
                            SwarmLogger.log("WARNING", f"DRONE_{port}", MISSION_STATUS_MESSAGES["STUCK"], "NAVIGATION")
                            try:
                                self.flight_ctrl.command_world_velocity(
                                    vehicle,
                                    "BREAKAWAY",
                                    -vx * STUCK_MANEUVER_SCALE_BACK,
                                    -vy * STUCK_MANEUVER_SCALE_BACK,
                                    0.0,
                                    reason="stuck recovery backoff",
                                )
                                time.sleep(STUCK_MANEUVER_DURATION)
                                self.flight_ctrl.command_world_velocity(
                                    vehicle,
                                    "BREAKAWAY",
                                    vx * STUCK_MANEUVER_SCALE_FWD,
                                    vy * STUCK_MANEUVER_SCALE_FWD,
                                    0.0,
                                    reason="stuck recovery forward pulse",
                                )
                            except Exception as e:
                                SwarmLogger.log("ERROR", f"DRONE_{port}", f"Maneuver error: {e}", "NAVIGATION")
                            last_move_ts = time.time()

                    # Timeout check
                    elapsed = time.time() - start_ts
                    timeout = PER_CELL_TIMEOUT_S * (1 + attempt * TIMEOUT_PENALTY_SCALE)
                    if elapsed > timeout:
                        SwarmLogger.log("WARNING", f"DRONE_{port}", MISSION_STATUS_MESSAGES["TIMEOUT"], "NAVIGATION")
                        SwarmLogger.log(
                            "WARNING", f"DRONE_{port}",
                            f"Transit timeout | cell=({to_row},{to_col}) attempt={attempt + 1}/{RETRY_LIMIT + 1} "
                            f"dist={dist:.1f}m timeout={timeout:.1f}s",
                            "NAVIGATION",
                        )
                        self._status(MISSION_STATUS_MESSAGES["TIMEOUT"])
                        break

                    time.sleep(CONTROL_INTERVAL_S)

            return False
        finally:
            self._clear_transit_reservation()

    def perform_scan(self, vehicle, detection_check=None, is_active_check=None,
                     pause_check=None, pause_waiter=None):
        """Run scan movement at the current position."""
        self.scanner.port = getattr(self.flight_ctrl, "port", None)
        self.scanner.flight_ctrl = self.flight_ctrl
        return self.scanner.perform_360_rotation_scan(
            vehicle,
            is_active_check=is_active_check,
            detection_check=detection_check,
            status_callback=self._status,
            pause_check=pause_check,
            pause_waiter=pause_waiter,
        )

    def scan_confirmed_cell(self, vehicle, lat, lon, row, col,
                            detection_check=None, is_active_check=None,
                            pause_check=None, pause_waiter=None):
        """Hold over a cell and run the scan sweep."""
        hold_ok = self.confirm_center_stability(
            vehicle,
            lat,
            lon,
            hold_s=CENTER_CONFIRM_HOLD_S,
            tol_m=CENTER_CONFIRM_TOLERANCE_M,
            is_active_check=is_active_check,
            pause_check=pause_check,
            pause_waiter=pause_waiter,
        )
        human_detected = False
        if hold_ok:
            human_detected = self.perform_scan(
                vehicle,
                detection_check=detection_check,
                is_active_check=is_active_check,
                pause_check=pause_check,
                pause_waiter=pause_waiter,
            )
        self.grid_status[(row, col)] = "visited"
        return hold_ok, human_detected
