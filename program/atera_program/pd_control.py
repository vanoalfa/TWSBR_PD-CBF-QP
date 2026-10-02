from __future__ import annotations
from dataclasses import dataclass
import config


@dataclass
class PDControlState:
    angle_psi: float            # body angle (MPU6050) [deg]
    angular_dot_psi: float      # body angular rate (MPU6050) [deg/s]
    angle_theta: float          # wheel angle (DDSM115 encoder) [deg]
    angular_dot_theta: float    # wheel angular rate (DDSM115 encoder) [deg/s]
    setpoint_psi: float         # body setpoint [deg]
    setpoint_theta: float       # wheel setpoint [deg]
    error_psi: float            # setpoint_psi - angle_psi [deg]
    error_theta: float          # setpoint_theta - angle_theta [deg]
    u_PD: float                 # single PD output [raw DDSM115 current count per wheel]


class PDController:
    def __init__(self) -> None:
        self.Kp_psi = self._as_gain("Kp_psi", config.Kp_psi)
        self.Kd_psi = self._as_gain("Kd_psi", config.Kd_psi)
        self.Kp_theta = self._as_gain("Kp_theta", config.Kp_theta)
        self.Kd_theta = self._as_gain("Kd_theta", config.Kd_theta)
        self.K_vel = int(config.K_vel)
        self.deadband_deg = float(config.CONTROLLER_DEADBAND_DEG)
        self.balance_direction_sign = float(config.BALANCE_DIRECTION_SIGN)

    @staticmethod
    def _as_gain(name: str, value) -> int:
        gain = int(value)
        if gain != value or gain < 0:
            raise ValueError(f"{name} must be an integer >= 0, got {value!r}")
        return gain

    def compute_PD(
        self,
        angle_psi: float,
        angular_dot_psi: float,
        angle_theta: float = 0.0,
        angular_dot_theta: float = 0.0,
        setpoint_psi: float = 0.0,
        setpoint_theta: float = 0.0,
    ) -> PDControlState:
        error_psi = float(setpoint_psi) - float(angle_psi)
        if abs(error_psi) < self.deadband_deg:
            error_psi = 0.0
        error_theta = float(setpoint_theta) - float(angle_theta)

        # u_PD = Kp*psi + Kd*psi_dot + Kp*theta + Kd*theta_dot + K_vel
        # (angles taken relative to their setpoints, no clamp).
        u_PD = (
            self.Kp_psi * (-error_psi)
            + self.Kd_psi * float(angular_dot_psi)
            + self.Kp_theta * (-error_theta)
            + self.Kd_theta * float(angular_dot_theta)
            + self.K_vel
        )
        # BALANCE_DIRECTION_SIGN = -1.0 (default) keeps the formula above as written.
        u_PD *= -self.balance_direction_sign

        return PDControlState(
            angle_psi=float(angle_psi),
            angular_dot_psi=float(angular_dot_psi),
            angle_theta=float(angle_theta),
            angular_dot_theta=float(angular_dot_theta),
            setpoint_psi=float(setpoint_psi),
            setpoint_theta=float(setpoint_theta),
            error_psi=error_psi,
            error_theta=error_theta,
            u_PD=u_PD,
        )


def wheel_output_from_u(u: float) -> float:
    if config.USE_WHEEL_DISTANCE_FORMULA:
        if not config.WHEEL_DISTANCE_M:
            raise ValueError("WHEEL_DISTANCE_M belum diisi di config.py")
        return float(u) / float(config.WHEEL_DISTANCE_M)
    return float(u)