"""Program utama self-balancing robot ATERA (Versi Joystick).

Fitur utama:
- Integrasi MPU6050 + Kalman filter + PD controller + dua motor DDSM115
- Satu output kontrol (u_PD) untuk kedua roda, opsional difilter CBF-QP (acados/HPIPM)
- Evaluasi ISE + plot otomatis (ise.py) setiap kali MULAI ditekan
- State machine sederhana: IDLE, CALIBRATING, READY, BALANCING, FAULT, EXITING
- Kontrol joystick non-blocking via evdev menggunakan mapping nama dari joystick_mapping.py
- UI terminal sederhana dengan beberapa halaman (LT / RT untuk pindah)
- Shutdown aman: motor dihentikan saat fault, tilt berlebih, atau quit

Pemetaan Kontrol Joystick (berdasarkan komentar di joystick_mapping.py):
- Tombol A     : Mode JOYSTICK (analog / D-Pad aktif untuk maju, mundur, belok)
- Tombol B     : Mode BALANCE (hanya balancing, joystick gerak diabaikan)
- Tombol X     : Matikan roda (stop balancing)
- Tombol Y     : Kalibrasi gyro + zero angle
- LB           : Kontroler PD
- RB           : Kontroler PD + CBF-QP
- LT / RT      : Pindah halaman UI ke kiri / kanan
- MULAI        : Start/Stop balancing (Toggle), plot ISE langsung muncul
- QUIT         : Quit aman
"""

from __future__ import annotations

import logging
import select
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, Optional

import evdev
from evdev import ecodes

import config
import joystick_mapping
from cbf_qp import CBFQPFilter, CBFResult
from ddsm115 import DDSM115Error, DualDDSM115, MotorFeedback, u_limit_raw
from ise import ISEEvaluator
from mpu6050 import AngleState, MPU6050Reader
from pd_control import PDController, PDControlState, wheel_output_from_u

LOGGER = logging.getLogger("atera")
PAGES = ("status", "motor", "cbf", "help")

CONTROL_PD = "PD"
CONTROL_PD_CBF = "PD+CBF-QP"
DRIVE_BALANCE = "BALANCE"      # Tombol B: balancing only, joystick motion ignored
DRIVE_JOYSTICK = "JOYSTICK"    # Tombol A: analog / D-Pad active


@dataclass
class RuntimeState:
    mode: str = "IDLE"
    calibrated: bool = False
    exit_requested: bool = False
    page_index: int = 0
    last_message: str = "Tekan Tombol Y untuk kalibrasi, lalu MULAI untuk balancing."
    fault_reason: str = ""
    latest_angle: Optional[AngleState] = None
    latest_pd: Optional[PDControlState] = None
    latest_motor_fb: Dict[str, Optional[MotorFeedback]] = field(
        default_factory=lambda: {"left": None, "right": None}
    )
    latest_cbf: Optional[CBFResult] = None
    control_mode: str = CONTROL_PD
    drive_mode: str = DRIVE_BALANCE
    setpoint_psi: float = 0.0
    setpoint_theta_left: float = 0.0
    setpoint_theta_right: float = 0.0
    angle_theta_left: float = 0.0
    angle_theta_right: float = 0.0
    u_cmd: float = 0.0
    turn_command: float = 0.0
    last_loop_dt: float = 0.0
    loop_hz_est: float = 0.0
    zero_offset_deg: float = 0.0
    gyro_bias_x: float = 0.0
    gyro_bias_y: float = 0.0
    balance_started_at: Optional[float] = None
    last_status_log_ts: float = 0.0


class JoystickController:
    """Pembaca Joystick Non-blocking berbasis evdev & joystick_mapping."""

    def __init__(self, dev_path: str) -> None:
        self.dev_path = dev_path
        self.device: Optional[evdev.InputDevice] = None
        self.analog_y: float = 0.0  # -1.0 (mundur) s/d +1.0 (maju)
        self.analog_x: float = 0.0  # -1.0 (kiri) s/d +1.0 (kanan)
        self.dpad_y: float = 0.0    # -1.0 (maju) s/d +1.0 (mundur)
        self.dpad_x: float = 0.0    # -1.0 (kiri) s/d +1.0 (kanan)

    def open(self) -> bool:
        try:
            self.device = evdev.InputDevice(self.dev_path)
            LOGGER.info("Joystick terhubung: %s (%s)", self.device.name, self.dev_path)
            return True
        except Exception as exc:
            LOGGER.error("Gagal membuka joystick pada %s: %s", self.dev_path, exc)
            self.device = None
            return False

    def _normalize_axis(self, value: int, deadzone: float = 0.15) -> float:
        """Mengubah rentang mentah axis EV_ABS (-32768 s/d 32767) ke -1.0 s/d 1.0 dengan deadzone."""
        if value > 0:
            norm = value / 32767.0
        elif value < 0:
            norm = value / 32768.0
        else:
            norm = 0.0

        if abs(norm) < deadzone:
            return 0.0
        return max(-1.0, min(1.0, norm))

    def poll_events(self, app: AteraMainApp) -> None:
        if self.device is None:
            return

        try:
            r, _, _ = select.select([self.device], [], [], 0)
            if not r:
                return

            for event in self.device.read():
                self._process_event(event, app)
        except (OSError, RuntimeError) as exc:
            LOGGER.error("Koneksi joystick terputus: %s", exc)
            self.device = None
            # A lost joystick must not keep the last stick position.
            self.analog_y = self.analog_x = self.dpad_y = self.dpad_x = 0.0

    def _process_event(self, event: evdev.InputEvent, app: AteraMainApp) -> None:
        # 1. Tombol Digital (EV_KEY)
        if event.type == ecodes.EV_KEY:
            if event.value == 1:  # Button Down Event
                btn_name = joystick_mapping.BTN_MAP.get(event.code)

                if btn_name == "MULAI":
                    app.toggle_balancing()
                elif btn_name == "Tombol Y":
                    app.calibrate()
                elif btn_name == "Tombol X":
                    app.stop_balancing("Roda dimatikan dari joystick.")
                elif btn_name == "Tombol A":
                    app.set_drive_mode(DRIVE_JOYSTICK)
                elif btn_name == "Tombol B":
                    app.set_drive_mode(DRIVE_BALANCE)
                elif btn_name == "LB":
                    app.set_control_mode(CONTROL_PD)
                elif btn_name == "RB":
                    app.set_control_mode(CONTROL_PD_CBF)
                elif btn_name == "LT (Digital)":
                    app.state.page_index = (app.state.page_index - 1) % len(PAGES)
                elif btn_name == "RT (Digital)":
                    app.state.page_index = (app.state.page_index + 1) % len(PAGES)
                elif btn_name == "QUIT":
                    app.state.exit_requested = True
                    app.set_message("Quit diminta dari joystick.")

        # 2. Sumbu Analog & D-Pad (EV_ABS)
        elif event.type == ecodes.EV_ABS:
            code = event.code
            val = event.value

            # Cek Pemetaan D-Pad (DPAD_MAP)
            if code in joystick_mapping.DPAD_MAP:
                dpad_dict = joystick_mapping.DPAD_MAP[code]
                dpad_name = dpad_dict.get(val, "")

                if code == 17:  # Sumbu Y D-Pad
                    if dpad_name == "PAD Atas":
                        self.dpad_y = -1.0  # -1 = Maju
                    elif dpad_name == "PAD Bawah":
                        self.dpad_y = 1.0   # 1 = Mundur
                    else:
                        self.dpad_y = 0.0
                elif code == 16:  # Sumbu X D-Pad
                    if dpad_name == "PAD Kiri":
                        self.dpad_x = -1.0  # -1 = Belok Kiri
                    elif dpad_name == "PAD Kanan":
                        self.dpad_x = 1.0   # 1 = Belok Kanan
                    else:
                        self.dpad_x = 0.0

            # Cek Pemetaan Analog Stick (ABS_MAP)
            elif code in joystick_mapping.ABS_MAP:
                abs_name = joystick_mapping.ABS_MAP[code]
                norm = self._normalize_axis(val)

                if abs_name == "Analog Kiri (Y)":
                    self.analog_y = -norm  # Dibalik agar nilai positif = Maju
                elif abs_name in ("Analog Kiri (X)", "Analog Kanan (X)"):
                    self.analog_x = norm

    def get_commands(self) -> tuple[float, float]:
        """Menghasilkan pasangan (offset setpoint_psi [deg], turn_command)."""
        target_angle = 0.0
        turn_command = 0.0

        # Maju / Mundur: Prioritas Analog Y, fallback ke D-Pad Y
        if abs(self.analog_y) > 0.0:
            if self.analog_y > 0:
                target_angle = self.analog_y * float(config.MANUAL_FORWARD_TARGET_DEG)
            else:
                target_angle = abs(self.analog_y) * float(config.MANUAL_BACKWARD_TARGET_DEG)
        elif self.dpad_y != 0.0:
            if self.dpad_y < 0:  # PAD Atas / Maju
                target_angle = float(config.MANUAL_FORWARD_TARGET_DEG)
            elif self.dpad_y > 0:  # PAD Bawah / Mundur
                target_angle = float(config.MANUAL_BACKWARD_TARGET_DEG)

        # Belok Kiri / Kanan: Prioritas Analog X, fallback ke D-Pad X
        if abs(self.analog_x) > 0.0:
            turn_command = self.analog_x
        elif self.dpad_x != 0.0:
            turn_command = self.dpad_x

        return target_angle, turn_command


class AteraMainApp:
    def __init__(self) -> None:
        self.motors = DualDDSM115()
        self.imu = MPU6050Reader()
        self.controller = PDController()
        self.cbf = CBFQPFilter()
        self.ise = ISEEvaluator()
        self.joystick = JoystickController(config.JOYSTICK_PATH)
        self.state = RuntimeState()
        self._running = False
        self._motors_stopped = False
        self._last_ui_ts = 0.0

    def setup(self) -> None:
        self.motors.open()
        self.motors.initialize()
        self.imu.open()
        if not self.joystick.open():
            LOGGER.warning("Gagal menginisialisasi joystick pada boot initial.")
        self.ise.open()
        # Builds/loads the acados solver only when the [CBF-QP] section of config.py is complete.
        self.cbf.setup()
        self._motors_stopped = True
        self.state.mode = "IDLE"
        self.state.last_message = "Hardware siap. Tekan Tombol Y untuk kalibrasi."
        LOGGER.info("ATERA main setup complete")

    def cleanup(self) -> None:
        self.state.mode = "EXITING"
        try:
            self.ensure_motors_stopped("cleanup")
        except Exception as exc:
            LOGGER.error("Failed stopping motors during cleanup: %s", exc)
        try:
            self.ise.close()
        except Exception as exc:
            LOGGER.error("Failed closing ISE evaluation: %s", exc)
        try:
            self.motors.close()
        except Exception as exc:
            LOGGER.error("Failed closing motors: %s", exc)
        try:
            self.imu.close()
        except Exception as exc:
            LOGGER.error("Failed closing IMU: %s", exc)

    def ensure_motors_stopped(self, reason: str = "") -> None:
        if self._motors_stopped:
            return
        self.motors.stop_all()
        self._motors_stopped = True
        if reason:
            LOGGER.info("Motors stopped: %s", reason)

    def set_message(self, message: str) -> None:
        self.state.last_message = message
        LOGGER.info(message)

    def set_fault(self, reason: str) -> None:
        self.state.mode = "FAULT"
        self.state.fault_reason = reason
        self.state.balance_started_at = None
        try:
            self.ensure_motors_stopped("fault")
        except Exception as exc:
            LOGGER.error("Failed to stop motors after fault: %s", exc)
        self.set_message(f"FAULT: {reason}{self.finish_ise()}")

    def finish_ise(self) -> str:
        """Stop the ISE run (if any) and save the plot. Returns a text for the UI message."""
        try:
            base = self.ise.stop()
        except Exception as exc:
            LOGGER.error("ISE stop failed: %s", exc)
            return ""
        if not base:
            return ""
        return f" | ISE disimpan: {config.ISE_PLOT_DIR}/{self.ise.run_name}.png"

    def set_drive_mode(self, drive_mode: str) -> None:
        self.state.drive_mode = drive_mode
        if drive_mode == DRIVE_JOYSTICK:
            self.set_message("Mode JOYSTICK: analog / D-Pad aktif.")
        else:
            self.set_message("Mode BALANCE: hanya balancing, joystick gerak diabaikan.")

    def set_control_mode(self, control_mode: str) -> None:
        if self.state.mode == "BALANCING":
            self.set_message("Ganti kontroler hanya saat roda mati (tekan Tombol X dulu).")
            return
        if control_mode == CONTROL_PD_CBF and not self.cbf.available:
            self.set_message(f"PD + CBF-QP belum bisa dipakai: {self.cbf.reason}")
            return
        self.state.control_mode = control_mode
        self.set_message(f"Kontroler aktif: {control_mode}")

    def stop_balancing(self, reason: str = "Balancing dihentikan.") -> None:
        try:
            self.ensure_motors_stopped("stop balancing")
        finally:
            self.state.balance_started_at = None
            self.state.latest_pd = None
            self.state.latest_cbf = None
            self.state.u_cmd = 0.0
            self.state.turn_command = 0.0
            if self.state.mode != "FAULT":
                self.state.mode = "READY" if self.state.calibrated else "IDLE"
            self.set_message(reason + self.finish_ise())

    def calibrate(self) -> None:
        if self.state.mode == "BALANCING":
            self.set_message("Kalibrasi hanya saat roda mati (tekan Tombol X dulu).")
            return
        previous_mode = self.state.mode
        self.state.mode = "CALIBRATING"
        self.state.fault_reason = ""
        self.set_message("Kalibrasi berjalan. Jaga robot diam tegak lurus.")
        try:
            self.ensure_motors_stopped("before calibration")
            gyro = self.imu.calibrate_gyro_bias()
            zero = self.imu.calibrate_zero_angle()
            self.state.gyro_bias_x = gyro["gyro_bias_x"]
            self.state.gyro_bias_y = gyro["gyro_bias_y"]
            self.state.zero_offset_deg = zero
            self.state.calibrated = True
            self.state.mode = "READY"
            self.set_message(
                f"Kalibrasi selesai. zero_offset={zero:.3f} deg | "
                f"gyro_bias_x={self.state.gyro_bias_x:.3f} | gyro_bias_y={self.state.gyro_bias_y:.3f}"
            )
        except Exception as exc:
            self.state.mode = previous_mode
            self.set_fault(f"Kalibrasi gagal: {exc}")

    def start_balancing(self) -> None:
        if not self.state.calibrated:
            self.set_message("Belum dikalibrasi. Tekan Tombol Y dulu.")
            return
        try:
            angle = self.imu.read_angles()
        except Exception as exc:
            self.set_fault(f"Gagal membaca IMU sebelum start: {exc}")
            return
        if abs(angle.angle_deg) > float(config.SAFE_TILT_DEG):
            self.set_message(
                f"Sudut awal terlalu besar ({angle.angle_deg:.2f} deg). Tegakkan robot dulu."
            )
            return
        try:
            # Zero command -> fresh encoder feedback; this wheel position becomes theta = 0.
            feedback = self.motors.command_u(0.0)
            self.motors.reset_wheel_angles()
        except Exception as exc:
            self.set_fault(f"Gagal membaca motor sebelum start: {exc}")
            return
        self.state.latest_motor_fb["left"] = feedback.get("left")
        self.state.latest_motor_fb["right"] = feedback.get("right")
        self.state.setpoint_theta_left = float(config.SETPOINT_THETA_DEG)
        self.state.setpoint_theta_right = float(config.SETPOINT_THETA_DEG)
        self.state.latest_angle = angle
        self.state.mode = "BALANCING"
        self.state.fault_reason = ""
        self.state.balance_started_at = angle.timestamp
        self._motors_stopped = False
        c = self.controller
        run_name = self.ise.start(
            f"{self.state.control_mode} | Kp_psi={c.Kp_psi} Kd_psi={c.Kd_psi} "
            f"Kp_theta={c.Kp_theta} Kd_theta={c.Kd_theta} K_vel={c.K_vel}"
        )
        self.set_message(f"Balancing aktif ({self.state.control_mode}). Evaluasi ISE: {run_name}")

    def toggle_balancing(self) -> None:
        if self.state.mode == "BALANCING":
            self.stop_balancing("Balancing dihentikan oleh user.")
        else:
            self.start_balancing()

    def get_angular_rate_from_state(self, angle_state: AngleState) -> float:
        if angle_state.axis_used == "roll":
            return angle_state.gyro_rate_x
        return angle_state.gyro_rate_y

    def control_step(self) -> None:
        angle_state = self.imu.read_angles()
        self.state.latest_angle = angle_state

        if self.state.mode != "BALANCING":
            return

        if abs(angle_state.angle_deg) >= float(config.SAFE_TILT_DEG):
            self.set_fault(
                f"Tilt melebihi batas aman: {angle_state.angle_deg:.2f} deg >= {config.SAFE_TILT_DEG:.2f} deg"
            )
            return

        st = self.state

        # Measurements
        angle_psi = angle_state.angle_deg                                  # body angle (MPU6050)
        angular_dot_psi = self.get_angular_rate_from_state(angle_state)    # body rate (MPU6050)
        angle_theta_left = self.motors.left.angle_theta                    # wheel angles (DDSM115)
        angle_theta_right = self.motors.right.angle_theta
        angle_theta = 0.5 * (angle_theta_left + angle_theta_right)
        angular_dot_theta = 0.5 * (self.motors.left.angular_dot_theta + self.motors.right.angular_dot_theta)

        # Setpoints
        if st.drive_mode == DRIVE_JOYSTICK:
            psi_offset, turn_command = self.joystick.get_commands()
        else:
            psi_offset, turn_command = 0.0, 0.0
        setpoint_psi = float(config.SETPOINT_PSI_DEG) + psi_offset
        if psi_offset != 0.0 or turn_command != 0.0:
            # While driving, the wheel setpoints follow the wheels; on release the robot holds the new position.
            st.setpoint_theta_left = angle_theta_left
            st.setpoint_theta_right = angle_theta_right
        setpoint_theta = 0.5 * (st.setpoint_theta_left + st.setpoint_theta_right)

        # PD: one output for both wheels
        pd_state = self.controller.compute_PD(
            angle_psi=angle_psi,
            angular_dot_psi=angular_dot_psi,
            angle_theta=angle_theta,
            angular_dot_theta=angular_dot_theta,
            setpoint_psi=setpoint_psi,
            setpoint_theta=setpoint_theta,
        )
        u_cmd = wheel_output_from_u(pd_state.u_PD)

        # Optional safety filter
        cbf_result = None
        if st.control_mode == CONTROL_PD_CBF:
            cbf_result = self.cbf.filter(u_cmd, angle_psi, angular_dot_psi, angle_theta, angular_dot_theta)
            u_cmd = cbf_result.u_safe

        # Same u for both wheels; steering only adds a left/right difference.
        turn_u = max(-1.0, min(1.0, turn_command)) * float(config.TURN_OUTPUT_FRACTION) * u_limit_raw()
        feedback = self.motors.command_u(u_cmd, turn_u)

        self.ise.add_sample(
            angle_state.timestamp - (st.balance_started_at or angle_state.timestamp),
            setpoint_psi, angle_psi,
            st.setpoint_theta_left, angle_theta_left,
            st.setpoint_theta_right, angle_theta_right,
            pd_state.u_PD, u_cmd,
            bool(cbf_result.active) if cbf_result else False,
            cbf_result.solve_time_ms if cbf_result else 0.0,
        )

        st.setpoint_psi = setpoint_psi
        st.angle_theta_left = angle_theta_left
        st.angle_theta_right = angle_theta_right
        st.u_cmd = u_cmd
        st.latest_cbf = cbf_result
        self.state.turn_command = turn_command
        self.state.latest_pd = pd_state
        self.state.latest_motor_fb["left"] = feedback.get("left")
        self.state.latest_motor_fb["right"] = feedback.get("right")
        self._motors_stopped = False

    def maybe_log_status(self) -> None:
        now = time.monotonic()
        if (now - self.state.last_status_log_ts) < float(config.STATUS_LOG_PERIOD_S):
            return
        self.state.last_status_log_ts = now

        angle = self.state.latest_angle.angle_deg if self.state.latest_angle else None
        rate = self.get_angular_rate_from_state(self.state.latest_angle) if self.state.latest_angle else None
        pd = self.state.latest_pd
        LOGGER.info(
            "mode=%s ctrl=%s drive=%s calibrated=%s angle_psi=%s angular_dot_psi=%s setpoint_psi=%.3f "
            "angle_theta=%.2f turn=%.2f u_PD=%.1f u_cmd=%.1f msg=%s",
            self.state.mode,
            self.state.control_mode,
            self.state.drive_mode,
            self.state.calibrated,
            f"{angle:.3f}" if angle is not None else "-",
            f"{rate:.3f}" if rate is not None else "-",
            self.state.setpoint_psi,
            pd.angle_theta if pd else 0.0,
            self.state.turn_command,
            pd.u_PD if pd else 0.0,
            self.state.u_cmd,
            self.state.last_message,
        )

    def render_ui(self) -> None:
        page = PAGES[self.state.page_index]
        angle = self.state.latest_angle
        pd = self.state.latest_pd
        left_fb = self.state.latest_motor_fb.get("left")
        right_fb = self.state.latest_motor_fb.get("right")

        lines = []
        lines.append("\x1b[2J\x1b[H")
        lines.append("ATERA SELF-BALANCING ROBOT (JOYSTICK CONTROL)")
        lines.append("=" * 72)
        lines.append(
            f"Mode: {self.state.mode:<12} | Calibrated: {str(self.state.calibrated):<5} | "
            f"Page: {page} ({self.state.page_index + 1}/{len(PAGES)})"
        )
        lines.append(
            f"Controller: {self.state.control_mode:<10} (LB=PD, RB=PD+CBF-QP) | "
            f"Drive: {self.state.drive_mode:<8} (A=JOYSTICK, B=BALANCE)"
        )
        lines.append(
            f"Loop: {self.state.loop_hz_est:7.1f} Hz | Control target: {config.CONTROL_HZ:.1f} Hz | "
            f"UI target: {config.UI_HZ:.1f} Hz"
        )
        lines.append(f"Message: {self.state.last_message}")
        if self.state.fault_reason:
            lines.append(f"Fault : {self.state.fault_reason}")
        lines.append("-" * 72)

        if page == "status":
            if angle is None:
                lines.append("IMU belum ada data.")
            else:
                angular_rate = self.get_angular_rate_from_state(angle)
                lines.append(f"Axis used           : {angle.axis_used}")
                lines.append(f"angle_psi           : {angle.angle_deg:+8.3f} deg")
                lines.append(f"angular_dot_psi     : {angular_rate:+8.3f} deg/s")
                lines.append(f"Kalman X / Y        : {angle.kalman_x:+8.3f} / {angle.kalman_y:+8.3f} deg")
                lines.append(f"Accel angle X / Y   : {angle.acc_angle_x:+8.3f} / {angle.acc_angle_y:+8.3f} deg")
                lines.append(f"dt                  : {angle.dt * 1000.0:8.3f} ms")
            lines.append("")
            if pd is None:
                lines.append("PD output           : belum aktif")
            else:
                lines.append(
                    f"angle_theta L/R/avg : {self.state.angle_theta_left:+9.2f} / "
                    f"{self.state.angle_theta_right:+9.2f} / {pd.angle_theta:+9.2f} deg"
                )
                lines.append(f"angular_dot_theta   : {pd.angular_dot_theta:+8.2f} deg/s")
                lines.append(f"setpoint_psi        : {pd.setpoint_psi:+8.3f} deg")
                lines.append(
                    f"setpoint_theta L/R  : {self.state.setpoint_theta_left:+9.2f} / "
                    f"{self.state.setpoint_theta_right:+9.2f} deg"
                )
                lines.append(f"error_psi / theta   : {pd.error_psi:+8.3f} / {pd.error_theta:+8.3f} deg")
                lines.append(f"Turn command        : {self.state.turn_command:+8.3f}")
                lines.append(f"u_PD                : {pd.u_PD:+10.1f}  (raw count per wheel)")
                lines.append(
                    f"u ke kedua roda     : {self.state.u_cmd:+10.1f}  (batas aktuator +-{u_limit_raw():.0f})"
                )
            lines.append("")
            c = self.controller
            lines.append(
                f"Gain: Kp_psi={c.Kp_psi} Kd_psi={c.Kd_psi} Kp_theta={c.Kp_theta} "
                f"Kd_theta={c.Kd_theta} K_vel={c.K_vel}"
            )
            lines.append(
                f"ISE {self.ise.run_name or '-':<13}: badan={self.ise.ise_psi:.4f} | "
                f"roda kiri={self.ise.ise_theta_left:.4f} | roda kanan={self.ise.ise_theta_right:.4f} (deg^2.s)"
            )

        elif page == "motor":
            lines.append(f"Motor control mode  : {config.MOTOR_CONTROL_MODE}")
            lines.append(f"Current limit       : {config.MAX_CURRENT_A:.3f} A")
            lines.append(f"Speed limit         : {config.MAX_SPEED_RPM:.3f} rpm")
            lines.append("")
            lines.append("LEFT MOTOR")
            if left_fb is None:
                lines.append("  belum ada feedback")
            else:
                lines.append(
                    f"  torque={left_fb.torque_ampere:+7.3f} A | speed={left_fb.speed_rpm:+5d} rpm | "
                    f"pos={left_fb.position_deg:7.2f} deg | err=0x{left_fb.error_code:02X}"
                )
            lines.append("RIGHT MOTOR")
            if right_fb is None:
                lines.append("  belum ada feedback")
            else:
                lines.append(
                    f"  torque={right_fb.torque_ampere:+7.3f} A | speed={right_fb.speed_rpm:+5d} rpm | "
                    f"pos={right_fb.position_deg:7.2f} deg | err=0x{right_fb.error_code:02X}"
                )
            lines.append("")
            lines.append("Kalibrasi")
            lines.append(f"  zero_offset_deg   : {self.state.zero_offset_deg:+8.4f}")
            lines.append(f"  gyro_bias_x       : {self.state.gyro_bias_x:+8.4f}")
            lines.append(f"  gyro_bias_y       : {self.state.gyro_bias_y:+8.4f}")
            lines.append(f"  safe_tilt_deg     : {config.SAFE_TILT_DEG:.2f}")

        elif page == "cbf":
            cbf = self.state.latest_cbf
            lines.append(f"CBF-QP (acados/HPIPM, N=1) : {'SIAP' if self.cbf.available else 'TIDAK AKTIF'}")
            lines.append(f"  status            : {self.cbf.reason}")
            lines.append(
                f"  Alpha_1 / Alpha_2 / Alpha_3 : {config.Alpha_1} / {config.Alpha_2} / {config.Alpha_3}"
            )
            lines.append(
                f"  psi_max / theta_dot_max     : {config.CBF_PSI_MAX_DEG} deg / "
                f"{config.CBF_THETA_DOT_MAX_DEG_S} deg/s"
            )
            lines.append("")
            if cbf is None:
                lines.append("Filter belum berjalan (pilih RB lalu MULAI).")
            else:
                lines.append(f"  u_PD -> u_safe    : {cbf.u_PD:+10.1f} -> {cbf.u_safe:+10.1f}")
                lines.append(f"  filter aktif      : {cbf.active} | acados status={cbf.status}")
                lines.append(f"  waktu QP          : {cbf.solve_time_ms:.3f} ms | gagal={self.cbf.fail_count}")
                lines.append(
                    f"  h1 / h2 (psi)     : {cbf.h[0]:+7.4f} / {cbf.h[1]:+7.4f} rad"
                )
                lines.append(
                    f"  h3 / h4 (theta_d) : {cbf.h[2]:+7.3f} / {cbf.h[3]:+7.3f} rad/s"
                )

        else:
            lines.append("BANTUAN KONTROL JOYSTICK")
            lines.append("  Tombol Y            : Kalibrasi gyro + zero angle")
            lines.append("  MULAI               : Start / Stop balancing (plot ISE muncul & tersimpan)")
            lines.append("  Tombol X            : Matikan roda (stop balancing)")
            lines.append("  Tombol A            : Mode JOYSTICK (analog / D-Pad aktif)")
            lines.append("  Tombol B            : Mode BALANCE (hanya balancing)")
            lines.append("  Analog Kiri / D-Pad Y : Maju / Mundur, lepas = berhenti")
            lines.append("  Analog / D-Pad X      : Belok Kiri / Kanan, lepas = berhenti")
            lines.append("  LB / RB             : Kontroler PD / PD + CBF-QP (saat roda mati)")
            lines.append("  LT / RT             : Pindah halaman UI kiri / kanan")
            lines.append("  QUIT                : Quit aman")

        lines.append("-" * 72)
        lines.append("Keys: Y calib | MULAI balance | X off | A joystick | B balance | LB PD | RB CBF | LT/RT page | QUIT")

        sys.stdout.write("\n".join(lines) + "\n")
        sys.stdout.flush()

    def run(self) -> None:
        control_period = 1.0 / float(config.CONTROL_HZ)
        ui_period = 1.0 / float(config.UI_HZ)
        self._running = True
        next_tick = time.monotonic()

        while self._running and not self.state.exit_requested:
            loop_start = time.monotonic()

            self.joystick.poll_events(self)

            try:
                self.control_step()
            except (OSError, RuntimeError, DDSM115Error, ValueError) as exc:
                self.set_fault(str(exc))
            except Exception as exc:
                self.set_fault(f"Unhandled error: {exc}")

            now = time.monotonic()
            self.state.last_loop_dt = now - loop_start
            self.state.loop_hz_est = 1.0 / self.state.last_loop_dt if self.state.last_loop_dt > 0 else 0.0
            self.maybe_log_status()

            if (now - self._last_ui_ts) >= ui_period:
                self.render_ui()
                self._last_ui_ts = now

            next_tick += control_period
            sleep_time = next_tick - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_tick = time.monotonic()

        self.cleanup()


def configure_logging() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
    )


def main() -> int:
    configure_logging()
    app = AteraMainApp()
    try:
        app.setup()
        app.run()
        return 0
    except KeyboardInterrupt:
        LOGGER.info("KeyboardInterrupt received, shutting down safely")
        app.cleanup()
        return 0
    except Exception as exc:
        LOGGER.exception("Fatal error in atera_main: %s", exc)
        try:
            app.cleanup()
        except Exception:
            LOGGER.exception("Cleanup after fatal error also failed")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())