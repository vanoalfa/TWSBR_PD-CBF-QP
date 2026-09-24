from __future__ import annotations
from dataclasses import dataclass, asdict
from typing import Dict, Any, Tuple
import config


@dataclass
class PDControlState:
    """Dataclass penampung status error, setpoint, dan sinyal luaran PD controller."""
    psi: float
    dot_psi: float
    theta: float
    dot_theta: float
    error_psi: float
    error_dot_psi: float
    error_theta: float
    error_dot_theta: float
    setpoint_psi: float
    setpoint_theta: float
    u_pd: float
    left_output: float
    right_output: float
    mode: str


class BalancePDController:
    """
    Kelas pengontrol PD Dual-Loop (IMU Pendulum + Encoder Roda)
    1. Balance Mode   : Diam seimbang di tempat (left_output == right_output).
    2. Mono Mode      : Keseimbangan + Maju/Mundur (setpoint_psi dinamis).
    3. Nugget Mode    : Navigasi Penuh (Maju/Mundur + Belok Kiri/Kanan).
    """

    def __init__(self) -> None:
        # Gain IMU (Pitch / Pendulum Body) -> PSI
        self.psi_kp = float(config.IMU_KP)
        self.psi_kd = float(config.IMU_KD)

        # Gain DDSM (Wheel Position & Speed) -> THETA
        self.theta_kp = float(config.MOTOR_KP)
        self.theta_kd = float(config.MOTOR_KD)

        # Safety & Limits
        self.output_limit = float(config.OUTPUT_LIMIT)
        self.deadband_deg = float(config.CONTROLLER_DEADBAND_DEG)
        self.balance_direction_sign = float(config.BALANCE_DIRECTION_SIGN)
        self.turn_fraction = float(config.TURN_OUTPUT_FRACTION)

        # Penampung state kalkulasi terakhir untuk snapshot debug
        self.last_state: PDControlState | None = None

    @staticmethod
    def clamp(value: float, low: float, high: float) -> float:
        """Fungsi pembantu untuk membatasi nilai dalam rentang [low, high]."""
        return max(low, min(high, value))

    def compute_uPD(
        self,
        psi: float,
        dot_psi: float,
        theta: float,
        dot_theta: float,
        setpoint_psi: float = 0.0,
        setpoint_theta: float = 0.0,
    ) -> Tuple[float, float, float, float, float]:
        """
        Fungsi inti kalkulasi uPD (Nominal Output PD Dual-Loop).
        Mengolah deviasi PSI (IMU) dan THETA (Encoder Roda).
        """
        # 1. Error IMU (PSI)
        error_psi = setpoint_psi - psi
        if abs(error_psi) < self.deadband_deg:
            error_psi = 0.0
        error_dot_psi = -dot_psi

        # 2. Error DDSM Wheel (THETA)
        error_theta = setpoint_theta - theta
        error_dot_theta = -dot_theta

        # 3. Kalkulasi Persamaan PD Dual-Loop (uPD)
        u_pd = (
            (self.psi_kp * error_psi)
            + (self.psi_kd * error_dot_psi)
            + (self.theta_kp * error_theta)
            + (self.theta_kd * error_dot_theta)
        )

        # Koreksi Arah Balancing & Saturation Guard
        u_pd *= self.balance_direction_sign
        u_pd = self.clamp(u_pd, -self.output_limit, self.output_limit)

        return u_pd, error_psi, error_dot_psi, error_theta, error_dot_theta

    def compute_balance_mode(
        self,
        psi: float,
        dot_psi: float,
        theta: float,
        dot_theta: float,
    ) -> PDControlState:
        """
        Mode 1: Balance (Diam Seimbang).
        Setpoint PSI = 0, Perintah belok = 0. Both wheels output identically.
        """
        u_pd, e_psi, ed_psi, e_th, ed_th = self.compute_uPD(
            psi=psi,
            dot_psi=dot_psi,
            theta=theta,
            dot_theta=dot_theta,
            setpoint_psi=0.0,
            setpoint_theta=0.0,
        )

        state = PDControlState(
            psi=psi,
            dot_psi=dot_psi,
            theta=theta,
            dot_theta=dot_theta,
            error_psi=e_psi,
            error_dot_psi=ed_psi,
            error_theta=e_th,
            error_dot_theta=ed_th,
            setpoint_psi=0.0,
            setpoint_theta=0.0,
            u_pd=u_pd,
            left_output=u_pd,
            right_output=u_pd,
            mode="BALANCE",
        )
        self.last_state = state
        return state

    def compute_mono_mode(
        self,
        psi: float,
        dot_psi: float,
        theta: float,
        dot_theta: float,
        target_pitch_deg: float,
    ) -> PDControlState:
        """
        Mode 2: Mono (Maju/Mundur Linear).
        Setpoint PSI diubah sesuai masukan target kemiringan, tanpa belok (turn = 0).
        """
        u_pd, e_psi, ed_psi, e_th, ed_th = self.compute_uPD(
            psi=psi,
            dot_psi=dot_psi,
            theta=theta,
            dot_theta=dot_theta,
            setpoint_psi=target_pitch_deg,
            setpoint_theta=0.0,
        )

        state = PDControlState(
            psi=psi,
            dot_psi=dot_psi,
            theta=theta,
            dot_theta=dot_theta,
            error_psi=e_psi,
            error_dot_psi=ed_psi,
            error_theta=e_th,
            error_dot_theta=ed_th,
            setpoint_psi=target_pitch_deg,
            setpoint_theta=0.0,
            u_pd=u_pd,
            left_output=u_pd,
            right_output=u_pd,
            mode="MONO",
        )
        self.last_state = state
        return state

    def compute_nugget_mode(
        self,
        psi: float,
        dot_psi: float,
        theta: float,
        dot_theta: float,
        target_pitch_deg: float,
        turn_command: float,
    ) -> PDControlState:
        """
        Mode 3: Nugget (Navigasi Penuh AWSD).
        Mendukung pergerakan linear (pitch setpoint) dan manuver diferensial (turn command).
        """
        u_pd, e_psi, ed_psi, e_th, ed_th = self.compute_uPD(
            psi=psi,
            dot_psi=dot_psi,
            theta=theta,
            dot_theta=dot_theta,
            setpoint_psi=target_pitch_deg,
            setpoint_theta=0.0,
        )

        # Differential Steering Mixer
        turn_term = self.clamp(turn_command, -1.0, 1.0) * self.turn_fraction
        left_output = self.clamp(u_pd - turn_term, -self.output_limit, self.output_limit)
        right_output = self.clamp(u_pd + turn_term, -self.output_limit, self.output_limit)

        state = PDControlState(
            psi=psi,
            dot_psi=dot_psi,
            theta=theta,
            dot_theta=dot_theta,
            error_psi=e_psi,
            error_dot_psi=ed_psi,
            error_theta=e_th,
            error_dot_theta=ed_th,
            setpoint_psi=target_pitch_deg,
            setpoint_theta=0.0,
            u_pd=u_pd,
            left_output=left_output,
            right_output=right_output,
            mode="NUGGET",
        )
        self.last_state = state
        return state

    def get_debug_snapshot(self, state: PDControlState | None = None) -> Dict[str, Any]:
        """
        Mengambil ringkasan snapshot data kalkulasi PD dan gain parameter
        untuk kebutuhan debugging terminal UI atau logging file.
        """
        target = state if state is not None else self.last_state

        if target is None:
            return {
                "active": False,
                "mode": "IDLE",
                "u_pd": 0.0,
                "left_output": 0.0,
                "right_output": 0.0,
                "error_psi": 0.0,
                "error_theta": 0.0,
            }

        snapshot = asdict(target)
        snapshot["active"] = True
        snapshot["gains"] = {
            "psi_kp": self.psi_kp,
            "psi_kd": self.psi_kd,
            "theta_kp": self.theta_kp,
            "theta_kd": self.theta_kd,
        }
        return snapshot