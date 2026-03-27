"""Unified flight controller — MAVLink commands + motion authority.

Single entry point for all drone movement. Handles:
  - NED/body velocity commands with profile-based filtering
  - Yaw rate and relative yaw commands
  - Arm & takeoff sequence
  - Intent-based motion profiles (TRANSIT, ATTACK, SCAN, etc.)
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional, Tuple

from dronekit import VehicleMode
from pymavlink import mavutil

from modules.core.logger import SwarmLogger
from config import (
    TAKEOFF_ARM_WAIT_S,
    TAKEOFF_ARMING_CYCLE_S,
    TAKEOFF_POST_DELAY_S,
    TAKEOFF_ARMABLE_TIMEOUT_S,
    TAKEOFF_ALTITUDE_TIMEOUT_S,
    TAKEOFF_ALTITUDE,
    MAX_SPEED_M_S,
    ROTATION_YAW_SPEED,
    IBVS_VX_MAX,
    IBVS_VZ_MAX,
    CENTERING_YAW_RATE_MAX,
    METERS_PER_DEGREE_LAT,
    MISSION_STATUS_MESSAGES,
    MOTION_BASE_ALTITUDE_M,
    MOTION_ALTITUDE_SCALE_MIN,
    MOTION_ALTITUDE_SCALE_MAX,
    MOTION_FAILSAFE_HOLD_XY_MPS,
    MOTION_FAILSAFE_HOLD_CLIMB_MPS,
    MOTION_FAILSAFE_HOLD_YAW_RAD_S,
    MOTION_TRANSIT_CLIMB_MPS,
    MOTION_TRANSIT_YAW_RAD_S,
    MOTION_ATTACK_RUN_IN_XY_MPS,
    MOTION_ATTACK_RUN_IN_CLIMB_MPS,
    MOTION_ATTACK_RUN_IN_YAW_RAD_S,
    MOTION_TRACK_SOFT_XY_MPS,
    MOTION_TRACK_SOFT_DESCENT_MPS,
    MOTION_TRACK_SOFT_CLIMB_MPS,
    MOTION_TRACK_SOFT_YAW_RAD_S,
    MOTION_BREAKAWAY_XY_MPS,
    MOTION_BREAKAWAY_CLIMB_MPS,
    MOTION_BREAKAWAY_YAW_RAD_S,
    MOTION_SCAN_MIN_YAW_RATE_DEG_S,
)


_EPS = 1e-3
_DEFAULT_BASE_ALT_M = max(float(MOTION_BASE_ALTITUDE_M), float(TAKEOFF_ALTITUDE))

@dataclass(frozen=True)
class MotionProfile:
    """Intent-specific command envelope."""
    max_xy_speed: float
    max_down_speed: float
    max_up_speed: float
    max_yaw_rate: float
    body_forward_only: bool = False
    translation_blocked: bool = False


@dataclass(frozen=True)
class MotionCommand:
    """Normalized command payload after profile filtering."""
    vx: float
    vy: float
    vz: float
    yaw_rate: float

class MotionAuthority:
    """Single gate for movement commands before MAVLink emission."""

    _PROFILES = {
        "HOLD":             MotionProfile(0.0, 0.0, 0.6, 0.15, translation_blocked=True),
        "FAILSAFE_HOLD":    MotionProfile(MOTION_FAILSAFE_HOLD_XY_MPS, 0.0, MOTION_FAILSAFE_HOLD_CLIMB_MPS, MOTION_FAILSAFE_HOLD_YAW_RAD_S),
        "TRANSIT":          MotionProfile(MAX_SPEED_M_S, MOTION_TRANSIT_CLIMB_MPS, MOTION_TRANSIT_CLIMB_MPS, MOTION_TRANSIT_YAW_RAD_S),
        "SCAN_SWEEP":       MotionProfile(0.3, 0.0, 0.4, math.radians(max(5.0, float(ROTATION_YAW_SPEED)))),
        "TRACK_SOFT":       MotionProfile(MOTION_TRACK_SOFT_XY_MPS, MOTION_TRACK_SOFT_DESCENT_MPS, MOTION_TRACK_SOFT_CLIMB_MPS, MOTION_TRACK_SOFT_YAW_RAD_S),
        "ATTACK":           MotionProfile(max(IBVS_VX_MAX, MOTION_ATTACK_RUN_IN_XY_MPS), IBVS_VZ_MAX, MOTION_ATTACK_RUN_IN_CLIMB_MPS, max(0.32, MOTION_ATTACK_RUN_IN_YAW_RAD_S), body_forward_only=True),
        "ATTACK_CENTERING": MotionProfile(0.0, 0.45, 0.45, CENTERING_YAW_RATE_MAX),
        "BREAKAWAY":        MotionProfile(MOTION_BREAKAWAY_XY_MPS, 0.0, MOTION_BREAKAWAY_CLIMB_MPS, MOTION_BREAKAWAY_YAW_RAD_S),
        "RTL_PREP":         MotionProfile(0.5, 0.0, 0.8, 0.25),
    }

    def __init__(self, port=None):
        self.port = port
        self.current_intent = "HOLD"
        self._last_log_sig: Optional[Tuple[str, int, int, int, int]] = None

    def set_intent(self, intent: Optional[str], reason: Optional[str] = None):
        """Update active movement intent."""
        chosen = intent or "HOLD"
        if chosen not in self._PROFILES:
            chosen = "HOLD"
        if self.current_intent != chosen:
            msg = f"Motion intent -> {chosen}"
            if reason:
                msg = f"{msg} ({reason})"
            SwarmLogger.log("STATE", f"DRONE_{self.port}", msg, "MOTION")
        self.current_intent = chosen

    def filter_world_velocity(self, vehicle, vx: float, vy: float, vz: float) -> MotionCommand:
        """Filter NED/world-frame velocity command."""
        cmd = MotionCommand(vx=float(vx), vy=float(vy), vz=float(vz), yaw_rate=0.0)
        filtered = self._apply_profile(vehicle, cmd)
        self._log_filter("WORLD", cmd, filtered)
        return filtered

    def filter_body_velocity(
        self, vehicle, vx: float, vy: float, vz: float, yaw_rate: float,
        attack_approved: bool = False,
    ) -> MotionCommand:
        """Filter body-frame velocity + yaw rate command."""
        profile = self._profile()
        if not attack_approved and self.current_intent not in {"TRANSIT", "BREAKAWAY"}:
            vx = 0.0
        cmd = MotionCommand(vx=float(vx), vy=float(vy), vz=float(vz), yaw_rate=float(yaw_rate))
        filtered = self._apply_profile(vehicle, cmd)
        self._log_filter("BODY", cmd, filtered)
        return filtered

    def filter_yaw_rate(self, vehicle, yaw_rate: float) -> float:
        """Filter yaw-only command."""
        profile = self._profile()
        scale = self._altitude_scale(vehicle)
        limit = max(_EPS, profile.max_yaw_rate * min(1.0, scale))
        return max(-limit, min(limit, float(yaw_rate)))

    def filter_relative_yaw(self, vehicle, angle_deg: float, yaw_rate_deg_s: float) -> Tuple[float, float]:
        """Filter scan/orbit yaw requests."""
        profile = self._profile()
        scale = self._altitude_scale(vehicle)
        max_rate_deg = math.degrees(profile.max_yaw_rate * min(1.0, scale))
        safe_rate = max(float(MOTION_SCAN_MIN_YAW_RATE_DEG_S), min(float(yaw_rate_deg_s), max_rate_deg))
        safe_angle = max(-90.0, min(90.0, float(angle_deg)))
        return safe_angle, safe_rate

    def _profile(self) -> MotionProfile:
        return self._PROFILES.get(self.current_intent, self._PROFILES["HOLD"])

    def _apply_profile(self, vehicle, cmd: MotionCommand) -> MotionCommand:
        profile = self._profile()
        scale = self._altitude_scale(vehicle)
        vx, vy, vz, yaw_rate = cmd.vx, cmd.vy, cmd.vz, cmd.yaw_rate

        if profile.translation_blocked:
            vx = vy = 0.0
            vz = min(vz, 0.0)

        xy_limit = max(_EPS, profile.max_xy_speed * scale)
        xy_mag = math.hypot(vx, vy)
        if xy_mag > xy_limit:
            clamp = xy_limit / xy_mag
            vx *= clamp
            vy *= clamp

        if profile.body_forward_only:
            vx = max(0.0, vx)
            if self.current_intent == "ATTACK":
                vy_limit = min(max(0.28, xy_limit * 0.18), 0.75)
            else:
                vy_limit = min(max(0.18, xy_limit * 0.12), 0.35)
            vy = max(-vy_limit, min(vy_limit, vy))

        down_limit = max(0.0, profile.max_down_speed * scale)
        up_limit = max(0.0, profile.max_up_speed * scale)
        vz = max(-up_limit, min(down_limit, vz))

        yaw_limit = max(_EPS, profile.max_yaw_rate * min(1.0, scale))
        yaw_rate = max(-yaw_limit, min(yaw_limit, yaw_rate))

        vx = 0.0 if abs(vx) < _EPS else vx
        vy = 0.0 if abs(vy) < _EPS else vy
        vz = 0.0 if abs(vz) < _EPS else vz
        yaw_rate = 0.0 if abs(yaw_rate) < _EPS else yaw_rate

        return MotionCommand(vx=vx, vy=vy, vz=vz, yaw_rate=yaw_rate)

    def _altitude_scale(self, vehicle) -> float:
        try:
            alt = float(getattr(vehicle.location.global_relative_frame, "alt", 0.0) or 0.0)
        except Exception:
            alt = 0.0
        if alt <= 0.0:
            return 1.0
        return max(MOTION_ALTITUDE_SCALE_MIN, min(MOTION_ALTITUDE_SCALE_MAX,
                   math.sqrt(max(alt, 0.25) / _DEFAULT_BASE_ALT_M)))

    def _log_filter(self, frame: str, raw: MotionCommand, filtered: MotionCommand):
        sig = (frame, round(raw.vx * 100), round(raw.vy * 100), round(raw.vz * 100), round(raw.yaw_rate * 100))
        if sig == self._last_log_sig:
            return
        self._last_log_sig = sig
        changed = (
            abs(raw.vx - filtered.vx) > 0.05
            or abs(raw.vy - filtered.vy) > 0.05
            or abs(raw.vz - filtered.vz) > 0.05
            or abs(raw.yaw_rate - filtered.yaw_rate) > 0.05
        )
        if changed:
            SwarmLogger.log_throttled(
                "GUARD", f"DRONE_{self.port}",
                f"{frame} filtered by {self.current_intent}: "
                f"({raw.vx:.2f},{raw.vy:.2f},{raw.vz:.2f},{raw.yaw_rate:.2f}) -> "
                f"({filtered.vx:.2f},{filtered.vy:.2f},{filtered.vz:.2f},{filtered.yaw_rate:.2f})",
                "MOTION", interval_s=1.0,
            )

class FlightController:
    """Unified MAVLink command interface with integrated motion authority."""

    def __init__(self, port=None):
        self.port = port
        self._transform_log_count = 0
        self.motion = MotionAuthority(port=port)

    def set_motion_intent(self, intent, reason=None):
        """Set active motion profile for subsequent commands."""
        self.motion.port = self.port
        self.motion.set_intent(intent, reason=reason)

    def command_world_velocity(self, vehicle, intent, vx, vy, vz=0.0, reason=None):
        """World-frame velocity through active motion profile."""
        self.set_motion_intent(intent, reason=reason)
        self.send_ned_velocity(vehicle, vx, vy, vz)

    def command_body_velocity(self, vehicle, intent, vx, vy, vz, yaw_rate,
                              attack_approved=False, reason=None):
        """Body-frame velocity through active motion profile."""
        self.set_motion_intent(intent, reason=reason)
        self.send_ned_velocity_with_yaw_rate(
            vehicle, vx, vy, vz, yaw_rate, attack_approved=attack_approved)

    def command_hold(self, vehicle, reason=None, failsafe=False):
        """Hold position safely."""
        intent = "FAILSAFE_HOLD" if failsafe else "HOLD"
        self.command_world_velocity(vehicle, intent, 0.0, 0.0, 0.0, reason=reason)

    def command_breakaway(self, vehicle, climb_rate, reason=None):
        """Controlled climb/escape maneuver."""
        self.command_world_velocity(vehicle, "BREAKAWAY", 0.0, 0.0, -abs(climb_rate), reason=reason)

    def command_scan_yaw(self, vehicle, angle_deg, yaw_rate_deg_s, direction, reason=None):
        """Relative yaw for scanning."""
        self.set_motion_intent("SCAN_SWEEP", reason=reason)
        self.condition_yaw_relative(vehicle, angle_deg, yaw_rate_deg_s, direction)

    def send_ned_velocity(self, vehicle, vx, vy, vz=0):
        """Send NED velocity command (world frame)."""
        self.motion.port = self.port
        cmd = self.motion.filter_world_velocity(vehicle, vx, vy, vz)
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,
            0, 0, 0,
            cmd.vx, cmd.vy, cmd.vz,
            0, 0, 0,
            0, 0)
        try:
            vehicle.send_mavlink(msg)
            vehicle.flush()
        except Exception as e:
            SwarmLogger.log("ERROR", f"DRONE_{self.port}", f"send_ned_velocity: {e}", "FLIGHT")

    def send_ned_velocity_with_yaw_rate(self, vehicle, vx_body, vy_body, vz, yaw_rate,
                                         attack_approved=False):
        """Send body velocity + yaw rate, converting body frame to NED."""
        self.motion.port = self.port
        cmd = self.motion.filter_body_velocity(
            vehicle, vx_body, vy_body, vz, yaw_rate, attack_approved=attack_approved)
        try:
            current_yaw = vehicle.attitude.yaw
        except Exception:
            current_yaw = 0.0

        cos_yaw = math.cos(current_yaw)
        sin_yaw = math.sin(current_yaw)
        vx_ned = cmd.vx * cos_yaw - cmd.vy * sin_yaw
        vy_ned = cmd.vx * sin_yaw + cmd.vy * cos_yaw

        self._transform_log_count += 1
        if self._transform_log_count % 50 == 0 and (abs(cmd.vx) > 0.01 or abs(cmd.vy) > 0.01):
            SwarmLogger.log("DEBUG", f"DRONE_{self.port}",
                f"Body->NED: ({cmd.vx:.2f},{cmd.vy:.2f}) -> ({vx_ned:.2f},{vy_ned:.2f})", "FLIGHT")

        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000011111000111,
            0, 0, 0,
            vx_ned, vy_ned, cmd.vz,
            0, 0, 0,
            0, cmd.yaw_rate)
        try:
            vehicle.send_mavlink(msg)
            vehicle.flush()
        except Exception as e:
            SwarmLogger.log("ERROR", f"DRONE_{self.port}", f"send_ned_velocity_with_yaw_rate: {e}", "FLIGHT")

    def condition_yaw_relative(self, vehicle, angle_deg, yaw_rate_deg_s, direction):
        """Execute relative yaw command through motion filter."""
        self.motion.port = self.port
        angle_deg, yaw_rate_deg_s = self.motion.filter_relative_yaw(vehicle, angle_deg, yaw_rate_deg_s)
        msg = vehicle.message_factory.command_long_encode(
            0, 0,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            angle_deg, yaw_rate_deg_s, direction, 1,
            0, 0, 0)
        try:
            vehicle.send_mavlink(msg)
            vehicle.flush()
        except Exception as e:
            SwarmLogger.log("ERROR", f"DRONE_{self.port}", f"condition_yaw_relative: {e}", "FLIGHT")

    def arm_and_takeoff(self, vehicle, target_altitude, status_callback=None):
        """Arm motors and takeoff to target altitude."""
        self.set_motion_intent("HOLD", reason="takeoff init")

        def _status(msg):
            if status_callback:
                status_callback(msg)
            SwarmLogger.log("INFO", f"DRONE_{self.port}", msg, "TAKEOFF")

        _status(MISSION_STATUS_MESSAGES["ARMING"])
        deadline = time.time() + TAKEOFF_ARMABLE_TIMEOUT_S
        while not vehicle.is_armable:
            if time.time() > deadline:
                raise TimeoutError(f"Vehicle not armable within {TAKEOFF_ARMABLE_TIMEOUT_S:.0f}s")
            SwarmLogger.log("DEBUG", f"DRONE_{self.port}", "Waiting for armable...", "TAKEOFF")
            time.sleep(TAKEOFF_ARM_WAIT_S)

        vehicle.mode = VehicleMode("GUIDED")
        vehicle.armed = True
        deadline = time.time() + TAKEOFF_ARMABLE_TIMEOUT_S
        while not vehicle.armed:
            if time.time() > deadline:
                raise TimeoutError(f"Vehicle failed to arm within {TAKEOFF_ARMABLE_TIMEOUT_S:.0f}s")
            SwarmLogger.log("DEBUG", f"DRONE_{self.port}", "Arming...", "TAKEOFF")
            time.sleep(TAKEOFF_ARMING_CYCLE_S)

        _status(MISSION_STATUS_MESSAGES["TAKING_OFF"].format(altitude=target_altitude))
        vehicle.simple_takeoff(target_altitude)

        deadline = time.time() + TAKEOFF_ALTITUDE_TIMEOUT_S
        while True:
            alt = vehicle.location.global_relative_frame.alt
            if alt >= target_altitude * 0.95:
                _status(MISSION_STATUS_MESSAGES["TAKEOFF_SUCCESS"].format(altitude=target_altitude))
                break
            if time.time() > deadline:
                raise TimeoutError(f"Takeoff altitude not reached within {TAKEOFF_ALTITUDE_TIMEOUT_S:.0f}s")
            time.sleep(0.5)

        _status(MISSION_STATUS_MESSAGES["BEGIN_MISSION_FLIGHT"])
        time.sleep(TAKEOFF_POST_DELAY_S)

    @staticmethod
    def latlon_to_north_east_m(lat1, lon1, lat2, lon2):
        """Convert lat/lon difference to north/east offset in meters."""
        d_north = (lat2 - lat1) * METERS_PER_DEGREE_LAT
        mean_lat = math.radians((lat1 + lat2) / 2.0)
        d_east = (lon2 - lon1) * METERS_PER_DEGREE_LAT * math.cos(mean_lat)
        return d_north, d_east
