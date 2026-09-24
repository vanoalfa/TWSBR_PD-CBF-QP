from __future__ import annotations
from dataclasses import dataclass
import tunning


@dataclass
class PDControlState:
    error_psi: float
    error_dot_psi: float
    target_psi: float
    error_theta: float
    error_dot_theta: float
    target_theta: float
    base_uPD: float
    left_uPD: float
    right_uPD: float


class BalancePDController:
    def __init__(self) -> None:
        self.kp = float(tunning.Kp)
        self.kd = float(tunning.Kd)
        self.output_limit = float(tunning.OUTPUT_LIMIT)
        self.deadband_deg = float(tunning.CONTROLLER_DEADBAND_DEG)
        self.balance_direction_sign = float(tunning.BALANCE_DIRECTION_SIGN)
        self.turn_fraction = float(tunning.TURN_OUTPUT_FRACTION)

    @staticmethod
    def clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    def uPD_compute(
        self,
        psi: float,
        dot_psi: float,
        target_psi: float = 0.0,
        theta: float,
        dot_theta: float,
        target_theta: float = 0.0,
        turn_command: float = 0.0,
    ) -> PDControlState:
        error_psi = float(target_psi) - float(psi)
        if abs(error_deg) < self.deadband_deg:
            error_deg = 0.0
        error_theta = float(target_theta) - float(theta)

        # Rumus PD
        error_rate_deg_s = -float(angular_rate_deg_s)
        base_output = (self.kp * error_deg) + (self.kd * error_rate_deg_s)
        base_output *= self.balance_direction_sign
        base_output = self.clamp(base_output, -self.output_limit, self.output_limit)

        turn_term = self.clamp(float(turn_command), -1.0, 1.0) * self.turn_fraction
        left_output = self.clamp(base_output - turn_term, -self.output_limit, self.output_limit)
        right_output = self.clamp(base_output + turn_term, -self.output_limit, self.output_limit)

        return PDControlState(
            error_deg=error_deg,
            error_rate_deg_s=error_rate_deg_s,
            target_angle_deg=float(target_angle_deg),
            base_output=base_output,
            left_output=left_output,
            right_output=right_output,
        )