from dataclasses import dataclass


def _clamp(value: float, limits):
    if limits is None:
        return value
    lo, hi = limits
    if lo >= hi:
        raise ValueError("Controller limits must satisfy lower < upper.")
    return max(lo, min(hi, value))


@dataclass
class PController:
    kp: float
    output_limits: tuple[float, float] | None = (-24.0, 24.0)

    def reset(self):
        pass

    def update(self, error: float, dt: float):
        if dt <= 0:
            raise ValueError("dt must be positive.")
        return _clamp(self.kp * error, self.output_limits)


@dataclass
class PIController:
    kp: float
    ki: float
    output_limits: tuple[float, float] | None = (-24.0, 24.0)

    def __post_init__(self):
        self.integral = 0.0

    def reset(self):
        self.integral = 0.0

    def update(self, error: float, dt: float):
        if dt <= 0:
            raise ValueError("dt must be positive.")
        candidate = self.integral + error * dt
        raw = self.kp * error + self.ki * candidate
        clipped = _clamp(raw, self.output_limits)
        # Conditional integration anti-windup: only accept the new integral
        # when the output is unsaturated or the error drives it back inward.
        if self.output_limits is None or clipped == raw:
            self.integral = candidate
        else:
            lo, hi = self.output_limits
            if (raw > hi and error < 0) or (raw < lo and error > 0):
                self.integral = candidate
        return clipped


@dataclass
class PIDController:
    kp: float
    ki: float
    kd: float
    output_limits: tuple[float, float] | None = (-24.0, 24.0)
    derivative_filter: float = 0.15

    def __post_init__(self):
        if not 0.0 <= self.derivative_filter <= 1.0:
            raise ValueError("derivative_filter must be between 0 and 1.")
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.previous_error = None
        self.filtered_derivative = 0.0

    def update(self, error: float, dt: float):
        if dt <= 0:
            raise ValueError("dt must be positive.")
        derivative = 0.0 if self.previous_error is None else (error - self.previous_error) / dt
        alpha = self.derivative_filter
        self.filtered_derivative = alpha * derivative + (1.0 - alpha) * self.filtered_derivative
        candidate = self.integral + error * dt
        raw = self.kp * error + self.ki * candidate + self.kd * self.filtered_derivative
        clipped = _clamp(raw, self.output_limits)
        if self.output_limits is None or clipped == raw:
            self.integral = candidate
        else:
            lo, hi = self.output_limits
            if (raw > hi and error < 0) or (raw < lo and error > 0):
                self.integral = candidate
        self.previous_error = error
        return clipped
