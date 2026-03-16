"""Flight Controller - MAVLink command abstraction for drone movement."""

import math
import time
from dronekit import VehicleMode
from pymavlink import mavutil
from modules.core.logger import SwarmLogger
from modules.core.geo_math import GeoMath
from config import (
    TAKEOFF_ALTITUDE,
    TAKEOFF_ARM_WAIT_S,
    TAKEOFF_ARMING_CYCLE_S,
    TAKEOFF_POST_DELAY_S,
    TAKEOFF_ARMABLE_TIMEOUT_S,
    TAKEOFF_ALTITUDE_TIMEOUT_S,
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
        """Send yaw rate command (rad/s).
        
        typeMask düzeltmesi:
        bit0-2: ignore pos (x,y,z)      = 0b111
        bit3-5: ignore vel (vx,vy,vz)   = 0b111000
        bit6-8: ignore accel (ax,ay,az) = 0b111000000
        bit9:   force set-point         = 0b0000000000
        bit10:  ignore yaw              = 0b10000000000
        bit11:  ignore yaw_rate         = 0b0 (KULLAN)
        = 0b0000_1001_1111_1111 = 0x09FF
        """
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000100111111111,  # Sadece yaw_rate aktif
            0, 0, 0,   # position (ignored)
            0, 0, 0,   # velocity (ignored)
            0, 0, 0,   # acceleration (ignored)
            0, yaw_rate)  # yaw=0 (ignored), yaw_rate=active
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

        # Rate-limited debug logging — skip zero-velocity hover
        self._transform_log_count += 1
        if self._transform_log_count % 50 == 0 and (abs(vx_body) > 0.01 or abs(vy_body) > 0.01):
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
        armable_deadline = time.time() + TAKEOFF_ARMABLE_TIMEOUT_S
        while not vehicle.is_armable:
            if time.time() > armable_deadline:
                raise TimeoutError(f"Vehicle not armable within {TAKEOFF_ARMABLE_TIMEOUT_S:.0f}s")
            SwarmLogger.log("DEBUG", f"DRONE_{self.port}", "Waiting for armable...", "TAKEOFF")
            time.sleep(TAKEOFF_ARM_WAIT_S)

        vehicle.mode = VehicleMode('GUIDED')
        vehicle.armed = True
        arming_deadline = time.time() + TAKEOFF_ARMABLE_TIMEOUT_S
        while not vehicle.armed:
            if time.time() > arming_deadline:
                raise TimeoutError(f"Vehicle failed to arm within {TAKEOFF_ARMABLE_TIMEOUT_S:.0f}s")
            SwarmLogger.log("DEBUG", f"DRONE_{self.port}", "Arming...", "TAKEOFF")
            time.sleep(TAKEOFF_ARMING_CYCLE_S)

        msg = MISSION_STATUS_MESSAGES["TAKING_OFF"].format(altitude=target_altitude)
        _status(msg)
        vehicle.simple_takeoff(target_altitude)

        takeoff_deadline = time.time() + TAKEOFF_ALTITUDE_TIMEOUT_S
        while True:
            alt = vehicle.location.global_relative_frame.alt
            if alt >= target_altitude * 0.95:
                msg = MISSION_STATUS_MESSAGES["TAKEOFF_SUCCESS"].format(altitude=target_altitude)
                _status(msg)
                break
            if time.time() > takeoff_deadline:
                raise TimeoutError(f"Takeoff altitude not reached within {TAKEOFF_ALTITUDE_TIMEOUT_S:.0f}s")
            time.sleep(0.5)

        _status(MISSION_STATUS_MESSAGES["BEGIN_MISSION_FLIGHT"])
        time.sleep(TAKEOFF_POST_DELAY_S)

    @staticmethod
    def haversine_distance_m(lat1, lon1, lat2, lon2):
        """Calculate distance between two GPS points in meters.
        
        Backward-compat wrapper — delegates to GeoMath.haversine_distance.
        """
        return GeoMath.haversine_distance(lat1, lon1, lat2, lon2)

    @staticmethod
    def latlon_to_north_east_m(lat1, lon1, lat2, lon2):
        """Convert lat/lon difference to north/east offset in meters."""
        d_north = (lat2 - lat1) * METERS_PER_DEGREE_LAT
        mean_lat = math.radians((lat1 + lat2) / 2.0)
        d_east = (lon2 - lon1) * METERS_PER_DEGREE_LAT * math.cos(mean_lat)
        return d_north, d_east
