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
        theta: float,
        dot_theta: float,
        target_psi: float = 0.0,
        target_theta: float = 0.0,
        turn_command: float = 0.0,
    ) -> PDControlState:
        # Perhitungan Error Psi
        error_psi = float(target_psi) - float(psi)
        if abs(error_psi) < self.deadband_deg:
            error_psi = 0.0

        error_dot_psi = -float(dot_psi)

        # Perhitungan Error Theta
        error_theta = float(target_theta) - float(theta)
        error_dot_theta = -float(dot_theta)

        # Rumus PD
        base_uPD = (self.kp * error_psi) + (self.kd * error_dot_psi)
        base_uPD *= self.balance_direction_sign
        base_uPD = self.clamp(base_uPD, -self.output_limit, self.output_limit)

        turn_term = self.clamp(float(turn_command), -1.0, 1.0) * self.turn_fraction
        left_uPD = self.clamp(base_uPD - turn_term, -self.output_limit, self.output_limit)
        right_uPD = self.clamp(base_uPD + turn_term, -self.output_limit, self.output_limit)

        return PDControlState(
            error_psi=error_psi,
            error_dot_psi=error_dot_psi,
            target_psi=float(target_psi),
            error_theta=error_theta,
            error_dot_theta=error_dot_theta,
            target_theta=float(target_theta),
            base_uPD=base_uPD,
            left_uPD=left_uPD,
            right_uPD=right_uPD,
        )