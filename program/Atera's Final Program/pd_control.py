from __future__ import annotations
from dataclasses import dataclass
import config


@dataclass
class PDControlState:
    error_IMU_psi: float
    error_IMU_dot_psi: float
    error_DDSM_theta: float
    error_DDSM_dot_theta: float
    setpoint_IMU: float
    setpoint_DDSM: float
    base_output: float
    left_output: float
    right_output: float


class PDController:
    def __init__(self) -> None:
        self.psi_kp = float(config.IMU_KP)
        self.psi_kd = float(config.IMU_KD)
        self.theta_kp = float(config.MOTOR_KP)
        self.theta_kd = float(config.MOTOR_KD)
        self.output_limit = float(config.OUTPUT_LIMIT)
        self.deadband_deg = float(config.CONTROLLER_DEADBAND_DEG)
        self.balance_direction_sign = float(config.BALANCE_DIRECTION_SIGN)
        self.turn_fraction = float(config.TURN_OUTPUT_FRACTION)

    @staticmethod
    def clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    def PD_compute(
        self,
        psi: float,
        dot_psi: float,
        theta: float,
        dot_theta: float,
        setpoint_psi: float = 0.0,
        setpoint_theta: float = 0.0,
        turn_command: float = 0.0,
    ) -> PDControlState:
        
        # 1. Kalkulasi Error IMU (Pitch/Tilt)
        error_psi = float(setpoint_psi) - float(psi)
        error_dot_psi = -float(dot_psi)
        if abs(error_psi) < self.deadband_deg:
            error_psi = 0.0

        # 2. Kalkulasi Error DDSM (Wheel Position & Speed)
        error_theta = float(setpoint_theta) - float(theta)
        error_dot_theta = -float(dot_theta)

        # 3. Penggabungan PD Dual-Loop (Cascade / Full State Feedback)
        base_output = (
            (self.psi_kp * error_psi)
            + (self.psi_kd * error_dot_psi)
            + (self.theta_kp * error_theta)
            + (self.theta_kd * error_dot_theta)
        )
        
        base_output *= self.balance_direction_sign
        base_output = self.clamp(base_output, -self.output_limit, self.output_limit)

        # 4. Mixing dengan Komando Belok
        turn_term = self.clamp(float(turn_command), -1.0, 1.0) * self.turn_fraction
        left_output = self.clamp(base_output - turn_term, -self.output_limit, self.output_limit)
        right_output = self.clamp(base_output + turn_term, -self.output_limit, self.output_limit)

        return PDControlState(
            error_IMU_psi=error_psi,
            error_IMU_dot_psi=error_dot_psi,
            error_DDSM_theta=error_theta,
            error_DDSM_dot_theta=error_dot_theta,
            setpoint_IMU=float(setpoint_psi),
            setpoint_DDSM: float(setpoint_theta),
            base_output=base_output,
            left_output=left_output,
            right_output=right_output,
        )