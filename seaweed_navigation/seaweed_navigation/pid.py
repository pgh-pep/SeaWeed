#!/usr/bin/env python3


class PID:
    def __init__(
        self,
        kp: float = 0.0,
        ki: float = 0.0,
        kd: float = 0.0,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = None
        self.integral_error = 0.0

        self.output_min = None
        self.output_max = None

        self.integral_min = None
        self.integral_max = None

    def reset(self, reset_integral: bool, reset_derivative: bool):
        if reset_integral:
            self.integral_error = 0.0
        if reset_derivative:
            self.prev_error = None

    def set_gains(self, kp: float, ki: float, kd: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def set_output_limits(self, output_min: float, output_max: float):
        self.output_min = output_min
        self.output_max = output_max

    def set_integral_windup_limits(self, integral_min: float, integral_max: float):
        self.integral_min = integral_min
        self.integral_max = integral_max

    def update(self, error: float, dt: float) -> float:
        if dt <= 0.0:
            return 0.0

        p = self.kp * error

        self.integral_error += error * dt
        if self.integral_min is not None:
            self.integral_error = max(self.integral_min, self.integral_error)
        if self.integral_max is not None:
            self.integral_error = min(self.integral_max, self.integral_error)

        i = self.ki * self.integral_error

        if self.prev_error is None:
            d = 0.0
        else:
            error_derv = (error - self.prev_error) / dt
            d = self.kd * error_derv

        self.prev_error = error

        output = p + i + d
        if self.output_min is not None:
            output = max(self.output_min, output)
        if self.output_max is not None:
            output = min(self.output_max, output)

        return output
