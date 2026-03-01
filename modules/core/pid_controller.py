"""PID Controllers - Simple PID and FilteredPID with anti-windup."""

from config import PID_KP, PID_KI, PID_KD


class PID:
    """Simple PID controller for drone navigation and movement.

    Used by: mission/navigation.py, mission/tracking_controller.py
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
