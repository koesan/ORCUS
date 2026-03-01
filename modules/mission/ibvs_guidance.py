"""IBVS Kamikaze Guidance - Image-based visual servoing for terminal attack."""

import math
import time
from typing import Tuple, Optional, Dict
from dataclasses import dataclass

from config import (
    IBVS_KP_YAW, IBVS_KP_DIVE, IBVS_VX_MAX, IBVS_VX_MIN,
    IBVS_TERMINAL_COVERAGE, IBVS_GHOST_TIMEOUT,
    IBVS_VZ_MAX, IBVS_VZ_MIN, CAMERA_PITCH_OFFSET,
    CAMERA_WIDTH, CAMERA_HEIGHT,
    IBVS_SHALLOW_DIVE_VZ_MAX, IBVS_STEEP_DIVE_COVERAGE, IBVS_STEEP_DIVE_VZ_MAX,
    IBVS_YAW_RATE_MAX, IBVS_MAX_PITCH_TAN, IBVS_APPROACH_PITCH_TAN,
    IBVS_STARTUP_PITCH_TAN, IBVS_STARTUP_VX_MIN, IBVS_RAMP_UP_TIME_S, IBVS_RAMP_UP_COVERAGE,
    IBVS_YAW_FILTER_ALPHA, IBVS_VZ_FILTER_ALPHA, IBVS_VX_FILTER_ALPHA,
    IBVS_PITCH_COMPENSATION_FACTOR, IBVS_VELOCITY_SMOOTHER_DT
)


@dataclass
class IBVSState:
    """IBVS state information."""
    error_x: float = 0.0
    error_y: float = 0.0
    coverage: float = 0.0
    is_locked: bool = False
    is_terminal: bool = False
    last_seen: float = 0.0


class LowPassFilter:
    """Low-pass filter for noise reduction."""
    
    def __init__(self, alpha: float = 0.3):
        self.alpha = alpha
        self.value = None
    
    def update(self, new_value: float) -> float:
        """Filter new value and return."""
        if self.value is None:
            self.value = new_value
        else:
            self.value = self.alpha * new_value + (1 - self.alpha) * self.value
        return self.value
    
    def reset(self):
        """Reset filter state."""
        self.value = None


class IBVSGuidance:
    """Image-Based Visual Servoing guidance for GPS-independent terminal attack.
    
    Generates NED velocity commands from target pixel position.
    Bee-like dive profile: STARTUP -> APPROACH -> TERMINAL.
    """
    
    def __init__(self, target_tracker_id: Optional[int] = None):
        self.target_id = target_tracker_id
        self.state = IBVSState()
        
        # Filters for noise reduction
        self.yaw_filter = LowPassFilter(alpha=IBVS_YAW_FILTER_ALPHA)
        self.vz_filter = LowPassFilter(alpha=IBVS_VZ_FILTER_ALPHA)
        self.vx_filter = LowPassFilter(alpha=IBVS_VX_FILTER_ALPHA)
        
        # Ghost mode
        self.last_valid_command = (0.0, 0.0, 0.0, 0.0)
        self.ghost_start_time = None
        
        # Attack startup tracking
        self.attack_start_time = None
        self.attack_phase = "STARTUP"
        self.last_attack_phase = None
        self.phase_transition_time = None
        
        # Impact detection
        self.impact_detected = False
        
    def set_target_id(self, tracker_id: int):
        """Set the target tracker ID"""
        if self.target_id != tracker_id:
            self.reset()
        self.target_id = tracker_id
        
    def reset(self):
        """Reset all state and filters to initial values."""
        self.state = IBVSState()
        self.yaw_filter.reset()
        self.vz_filter.reset()
        self.vx_filter.reset()
        self.ghost_start_time = None
        self.last_valid_command = (0.0, 0.0, 0.0, 0.0)
        self.attack_start_time = None
        self.attack_phase = "STARTUP"
        self.last_attack_phase = None
        self.phase_transition_time = None
        self.impact_detected = False
    
    def compute_velocity(
        self,
        bbox_center: Optional[Tuple[float, float]],
        bbox_size: Optional[Tuple[float, float]],
        frame_size: Tuple[int, int] = (CAMERA_WIDTH, CAMERA_HEIGHT),
        current_tracker_id: Optional[int] = None,
        drone_pitch: float = 0.0
    ) -> Tuple[float, float, float, float]:
        """Generate NED velocity command from pixel error.
        
        Returns (vx, vy, vz, yaw_rate). vz is always >= 0 (no climb).
        """
        img_w, img_h = frame_size
        
        # Target loss check
        if bbox_center is None or bbox_size is None:
            return self._handle_target_lost()
        
        # Calculate pixel error
        cx, cy = bbox_center
        bw, bh = bbox_size
        
        error_x = (cx - img_w / 2) / (img_w / 2)
        error_y_raw = (cy - img_h / 2) / (img_h / 2)
        
        # Drone pitch compensation (fixed-mount camera)
        pitch_compensation = drone_pitch * IBVS_PITCH_COMPENSATION_FACTOR
        error_y = error_y_raw + pitch_compensation
        error_y = max(-1.0, min(1.0, error_y))
        
        coverage = (bw * bh) / (img_w * img_h) * 100
        
        # Update state
        self.state.error_x = error_x
        self.state.error_y = error_y
        self.state.coverage = coverage
        self.state.is_locked = True
        self.state.last_seen = time.time()
        self.state.is_terminal = coverage >= IBVS_TERMINAL_COVERAGE
        
        self.ghost_start_time = None
        
        # Calculate yaw rate
        yaw_rate_raw = IBVS_KP_YAW * error_x
        yaw_rate = self.yaw_filter.update(yaw_rate_raw)
        yaw_rate = max(min(yaw_rate, IBVS_YAW_RATE_MAX), -IBVS_YAW_RATE_MAX)
        
        # Calculate vz (dive)
        vz_from_error = IBVS_KP_DIVE * max(0.0, error_y)
        ff_pitch = math.tan(CAMERA_PITCH_OFFSET)
        vx_estimated = IBVS_VX_MAX * (1.0 - 0.5 * min(math.sqrt(error_x**2 + error_y**2), 1.0))
        vz_feedforward = vx_estimated * ff_pitch * 0.5
        vz_raw = vz_from_error + vz_feedforward
        
        # Limit vz to prevent pitch-down
        vz_pitch_limited = vx_estimated * IBVS_MAX_PITCH_TAN
        vz_raw = min(vz_raw, vz_pitch_limited)
        vz = self.vz_filter.update(vz_raw)
        
        # Phase-based limits
        if self.state.is_terminal:
            vz = max(IBVS_VZ_MIN, min(vz, IBVS_STEEP_DIVE_VZ_MAX))
        elif coverage >= IBVS_STEEP_DIVE_COVERAGE:
            vz = max(IBVS_VZ_MIN, min(vz, IBVS_STEEP_DIVE_VZ_MAX))
        else:
            vz = max(IBVS_VZ_MIN, min(vz, IBVS_SHALLOW_DIVE_VZ_MAX))
        
        # Calculate vx (forward speed)
        error_mag = math.sqrt(error_x**2 + error_y**2)
        
        if self.attack_start_time is None:
            self.attack_start_time = time.time()
        
        attack_elapsed = time.time() - self.attack_start_time
        
        # Determine attack phase
        prev_phase = self.attack_phase
        if coverage >= IBVS_TERMINAL_COVERAGE:
            self.attack_phase = "TERMINAL"
        elif coverage >= IBVS_RAMP_UP_COVERAGE or attack_elapsed >= IBVS_RAMP_UP_TIME_S:
            self.attack_phase = "APPROACH"
        else:
            self.attack_phase = "STARTUP"
        
        # Phase transition smoothing - reset filters on phase change
        if prev_phase != self.attack_phase:
            self.last_attack_phase = prev_phase
            self.phase_transition_time = time.time()
            # Reset velocity filter for smooth transition
            self.vx_filter.reset()
            self.vz_filter.reset()
        
        # Calculate vx based on phase
        if self.attack_phase == "TERMINAL":
            vx = IBVS_VX_MAX
            yaw_rate *= 0.3
            
        elif self.attack_phase == "STARTUP":
            vx_raw = IBVS_STARTUP_VX_MIN + (IBVS_VX_MIN - IBVS_STARTUP_VX_MIN) * (1.0 - min(error_mag, 1.0))
            vx = self.vx_filter.update(vx_raw)
            yaw_rate *= 1.2
            
        else:  # APPROACH
            ramp_progress = min(1.0, attack_elapsed / IBVS_RAMP_UP_TIME_S)
            if ramp_progress < 0.5:
                ease = 2 * ramp_progress * ramp_progress
            else:
                ease = 1 - (-2 * ramp_progress + 2) ** 2 / 2
            vx_base = IBVS_STARTUP_VX_MIN + (IBVS_VX_MAX - IBVS_STARTUP_VX_MIN) * ease
            vx_raw = vx_base * (1.0 - 0.4 * min(error_mag, 1.0))
            vx_raw = max(vx_raw, IBVS_STARTUP_VX_MIN)
            vx = self.vx_filter.update(vx_raw)
        
        # Final vz limit based on actual vx and phase
        if self.attack_phase == "TERMINAL":
            max_pitch_tan = IBVS_MAX_PITCH_TAN
        elif self.attack_phase == "STARTUP":
            max_pitch_tan = IBVS_STARTUP_PITCH_TAN
        else:
            max_pitch_tan = IBVS_APPROACH_PITCH_TAN
        
        vz = min(vz, vx * max_pitch_tan)
        
        self.last_valid_command = (vx, 0.0, vz, yaw_rate)
        
        return vx, 0.0, vz, yaw_rate
    
    def _handle_target_lost(self) -> Tuple[float, float, float, float]:
        """Ghost mode - continue in last known direction."""
        self.state.is_locked = False
        
        if self.ghost_start_time is None:
            self.ghost_start_time = time.time()
        
        ghost_duration = time.time() - self.ghost_start_time
        
        if ghost_duration < IBVS_GHOST_TIMEOUT:
            vx, vy, vz, yaw_rate = self.last_valid_command
            decay = 1.0 - (ghost_duration / IBVS_GHOST_TIMEOUT) * 0.5
            return vx * decay, vy, vz * decay, yaw_rate * decay
        else:
            return 0.0, 0.0, 0.0, 0.0
    
    def get_state(self) -> Dict:
        """Return guidance state."""
        return {
            'error_x': self.state.error_x,
            'error_y': self.state.error_y,
            'coverage': self.state.coverage,
            'is_locked': self.state.is_locked,
            'is_terminal': self.state.is_terminal,
            'target_id': self.target_id
        }


class VelocitySmoother:
    """Smooth velocity commands to prevent sudden changes."""
    
    def __init__(self, max_accel: float = 2.0, max_decel: float = 4.0):
        self.max_accel = max_accel
        self.max_decel = max_decel
        self.last_vx = 0.0
        self.last_vy = 0.0
        self.last_vz = 0.0
        self.last_time = None
    
    def smooth(self, vx: float, vy: float, vz: float) -> Tuple[float, float, float]:
        """Smooth velocity commands."""
        current_time = time.time()
        
        if self.last_time is None:
            self.last_time = current_time
            self.last_vx = vx
            self.last_vy = vy
            self.last_vz = vz
            return vx, vy, vz
        
        dt = current_time - self.last_time
        if dt <= 0:
            dt = IBVS_VELOCITY_SMOOTHER_DT
        self.last_time = current_time
        
        vx = self._smooth_axis(vx, self.last_vx, dt)
        vy = self._smooth_axis(vy, self.last_vy, dt)
        vz = self._smooth_axis(vz, self.last_vz, dt)
        
        self.last_vx = vx
        self.last_vy = vy
        self.last_vz = vz
        
        return vx, vy, vz
    
    def _smooth_axis(self, target: float, current: float, dt: float) -> float:
        """Smooth single axis."""
        delta = target - current
        
        if delta > 0:
            max_delta = self.max_accel * dt
        else:
            max_delta = self.max_decel * dt
        
        if abs(delta) > max_delta:
            delta = max_delta if delta > 0 else -max_delta
        
        return current + delta
    
    def reset(self):
        """Reset smoother state."""
        self.last_vx = 0.0
        self.last_vy = 0.0
        self.last_vz = 0.0
        self.last_time = None
