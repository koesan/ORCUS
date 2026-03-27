"""PID controllers and lightweight signal filters used across the stack."""

import time
from typing import Tuple

from config import PID_KP, PID_KI, PID_KD


class PID:
    """Simple PID controller for drone navigation and movement.

    Used by: mission/navigation.py, mission/attack_controller.py
    Interface: step(error, dt) -> control_output
    """

    def __init__(self, kp=PID_KP, ki=PID_KI, kd=PID_KD, integral_limit=10.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = 0.0
        self.integral_limit = integral_limit

    def reset(self):
        """Clear accumulated state for fresh start."""
        self.integral = 0.0
        self.prev_error = 0.0

    def step(self, error, dt):
        """Compute PID output for given error and time step dt."""
        if dt <= 0:
            return 0.0
        self.integral += error * dt
        self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
        deriv = (error - self.prev_error) / dt
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * deriv


class FilteredPID:

    def __init__(self, tau, kp, ki, kd,
                 integrator_max, integrator_min,
                 pid_max, pid_min):
        """Create filtered PID with anti-windup and derivative filter.

        Args:
            tau: Derivative filter time constant (s)
            kp, ki, kd: PID gains
            integrator_max/min: Anti-windup clamp limits
            pid_max/min: Output saturation limits
        """

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

    def update(self, error, timestamp, measurement, freeze_integral=False):
        """Compute PID output with derivative filter and trapezoidal integral."""

        delta_t = timestamp - self.previous_timestamp
        if delta_t <= 0:
            return 0.0

        # Proportional
        p = self.kp * error

        # Integral (trapezoidal rule) with Anti-Windup check
        if freeze_integral:
            i = self.i
        else:
            i = self.i + self.ki * delta_t * (error + self.previous_error) / 2
            if i > self.integrator_max:
                i = self.integrator_max
            elif i < self.integrator_min:
                i = self.integrator_min

        # Derivative (filtered, measurement-based to avoid setpoint kick)
        d = -(2 * self.kd * (measurement - self.previous_measurement) +
              (2 * self.tau - delta_t) * self.d) / (2 * self.tau + delta_t)

        # Total output with saturation
        pid = p + i + d
        if pid > self.pid_max:
            pid = self.pid_max
        elif pid < self.pid_min:
            pid = self.pid_min

        # Store state
        self.previous_error = error
        self.previous_measurement = measurement
        self.previous_timestamp = timestamp
        self.i = i
        self.d = d
        return pid

    def reset(self):
        """Clear all accumulated state."""
        self.previous_measurement = 0
        self.previous_error = 0
        self.previous_timestamp = 0
        self.i = 0
        self.d = 0


class LowPassFilter:
    """Single-pole low-pass filter for noisy scalar signals."""

    def __init__(self, alpha: float = 0.3):
        self.alpha = alpha
        self.value = None

    def update(self, new_value: float) -> float:
        """Filter a new sample and return the smoothed value."""
        if self.value is None:
            self.value = new_value
        else:
            self.value = self.alpha * new_value + (1 - self.alpha) * self.value
        return self.value

    def reset(self) -> None:
        """Clear filter state."""
        self.value = None


class VelocitySmoother:
    """Rate-limit velocity commands to avoid abrupt motion changes."""

    def __init__(self, max_accel: float = 2.0, max_decel: float = 4.0, dt_floor: float = 0.02):
        self.max_accel = max_accel
        self.max_decel = max_decel
        self.dt_floor = dt_floor
        self.last_vx = 0.0
        self.last_vy = 0.0
        self.last_vz = 0.0
        self.last_time = None

    def smooth(self, vx: float, vy: float, vz: float) -> Tuple[float, float, float]:
        """Smooth velocity commands on each axis independently."""
        current_time = time.time()

        if self.last_time is None:
            self.last_time = current_time
            self.last_vx = vx
            self.last_vy = vy
            self.last_vz = vz
            return vx, vy, vz

        dt = current_time - self.last_time
        if dt <= 0:
            dt = self.dt_floor
        self.last_time = current_time

        vx = self._smooth_axis(vx, self.last_vx, dt)
        vy = self._smooth_axis(vy, self.last_vy, dt)
        vz = self._smooth_axis(vz, self.last_vz, dt)

        self.last_vx = vx
        self.last_vy = vy
        self.last_vz = vz
        return vx, vy, vz

    def _smooth_axis(self, target: float, current: float, dt: float) -> float:
        delta = target - current
        max_delta = self.max_accel * dt if delta > 0 else self.max_decel * dt
        if abs(delta) > max_delta:
            delta = max_delta if delta > 0 else -max_delta
        return current + delta

    def reset(self) -> None:
        """Reset smoother state."""
        self.last_vx = 0.0
        self.last_vy = 0.0
        self.last_vz = 0.0
        self.last_time = None
