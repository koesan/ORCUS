"""Flight Controller - MAVLink command abstraction for drone movement."""

import math
import time
from dronekit import VehicleMode
from pymavlink import mavutil
from modules.core.logger import SwarmLogger
from config import (
    TAKEOFF_ALTITUDE,
    TAKEOFF_ARM_WAIT_S,
    TAKEOFF_ARMING_CYCLE_S,
    TAKEOFF_POST_DELAY_S,
    MISSION_STATUS_MESSAGES,
    METERS_PER_DEGREE_LAT
)

class FlightController:
    """MAVLink command interface for NED velocity, yaw rate, and takeoff."""
    
    def __init__(self, port=None):
        self.port = port
        self._transform_log_count = 0

    def send_ned_velocity(self, vehicle, vx, vy, vz=0):
        """Send NED velocity command (world frame)."""
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
            SwarmLogger.log("ERROR", f"DRONE_{self.port}", f"send_ned_velocity error: {e}", "FLIGHT")

    def send_yaw_rate(self, vehicle, yaw_rate):
        """Send yaw rate command (rad/s)."""
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000111111000111,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0,
            0, yaw_rate)
        try:
            vehicle.send_mavlink(msg)
            vehicle.flush()
        except Exception as e:
            SwarmLogger.log("ERROR", f"DRONE_{self.port}", f"send_yaw_rate error: {e}", "FLIGHT")

    def send_ned_velocity_with_yaw_rate(self, vehicle, vx_body, vy_body, vz, yaw_rate):
        """Send velocity + yaw rate, converting body frame to NED."""
        try:
            current_yaw = vehicle.attitude.yaw
        except Exception:
            current_yaw = 0.0

        # Body -> NED rotation
        cos_yaw = math.cos(current_yaw)
        sin_yaw = math.sin(current_yaw)

        vx_ned = vx_body * cos_yaw - vy_body * sin_yaw
        vy_ned = vx_body * sin_yaw + vy_body * cos_yaw

        # Rate-limited debug logging
        self._transform_log_count += 1
        if self._transform_log_count % 50 == 0:
            SwarmLogger.log("DEBUG", f"DRONE_{self.port}",
                f"Body->NED: ({vx_body:.2f}, {vy_body:.2f}) -> ({vx_ned:.2f}, {vy_ned:.2f})", "FLIGHT")

        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000011111000111,
            0, 0, 0,
            vx_ned, vy_ned, vz,
            0, 0, 0,
            0, yaw_rate)
        try:
            vehicle.send_mavlink(msg)
            vehicle.flush()
        except Exception as e:
            SwarmLogger.log("ERROR", f"DRONE_{self.port}", f"send_ned_velocity_with_yaw_rate error: {e}", "FLIGHT")

    def safe_track_velocity(self, vehicle, vx, vy, vz, yaw_rate, attack_approved):
        """Safety gatekeeper: blocks forward velocity if attack not approved."""
        if not attack_approved:
            vx = 0.0
        self.send_ned_velocity_with_yaw_rate(vehicle, vx, vy, vz, yaw_rate)

    def arm_and_takeoff(self, vehicle, target_altitude, status_callback=None):
        """Arm motors and takeoff to target altitude."""
        def _status(msg):
            if status_callback:
                status_callback(msg)
            SwarmLogger.log("INFO", f"DRONE_{self.port}", msg, "TAKEOFF")

        _status(MISSION_STATUS_MESSAGES["ARMING"])
        while not vehicle.is_armable:
            SwarmLogger.log("DEBUG", f"DRONE_{self.port}", "Waiting for armable...", "TAKEOFF")
            time.sleep(TAKEOFF_ARM_WAIT_S)

        vehicle.mode = VehicleMode('GUIDED')
        vehicle.armed = True
        while not vehicle.armed:
            SwarmLogger.log("DEBUG", f"DRONE_{self.port}", "Arming...", "TAKEOFF")
            time.sleep(TAKEOFF_ARMING_CYCLE_S)

        msg = MISSION_STATUS_MESSAGES["TAKING_OFF"].format(altitude=target_altitude)
        _status(msg)
        vehicle.simple_takeoff(target_altitude)

        while True:
            alt = vehicle.location.global_relative_frame.alt
            if alt >= target_altitude * 0.95:
                msg = MISSION_STATUS_MESSAGES["TAKEOFF_SUCCESS"].format(altitude=target_altitude)
                _status(msg)
                break
            time.sleep(0.5)

        _status(MISSION_STATUS_MESSAGES["BEGIN_MISSION_FLIGHT"])
        time.sleep(TAKEOFF_POST_DELAY_S)

    @staticmethod
    def haversine_distance_m(lat1, lon1, lat2, lon2):
        """Calculate distance between two GPS points in meters."""
        R = 6371000.0
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        a = math.sin(dphi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2.0) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return R * c

    @staticmethod
    def latlon_to_north_east_m(lat1, lon1, lat2, lon2):
        """Convert lat/lon difference to north/east offset in meters."""
        d_north = (lat2 - lat1) * METERS_PER_DEGREE_LAT
        mean_lat = math.radians((lat1 + lat2) / 2.0)
        d_east = (lon2 - lon1) * METERS_PER_DEGREE_LAT * math.cos(mean_lat)
        return d_north, d_east
