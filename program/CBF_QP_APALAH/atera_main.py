"""Program utama self-balancing robot ATERA (Versi PD + CBF-QP).

Fitur utama:
- Integrasi MPU6050 + Kalman filter + PD controller + dua motor DDSM115
- Safety filter CBF-QP (HOCBF orde-2) via acados + HPIPM (N=1)
- State machine: IDLE, CALIBRATING, READY, BALANCING, FAULT, EXITING
- Kontrol joystick non-blocking via evdev (joystick_mapping.py)
- UI terminal dengan halaman tambahan CBF (LB / RB untuk pindah)
- Logging evaluasi ISE (evaluation.py) untuk keperluan skripsi
- Shutdown aman: motor dihentikan saat fault, tilt berlebih, atau quit

Pemetaan Kontrol Joystick:
- Tombol Y     : Kalibrasi gyro + zero angle
- MULAI        : Start/Stop balancing (Toggle)
- Tombol X/B   : Stop balancing
- LB / RB      : Ganti halaman UI
- Analog/D-Pad : Maju, mundur, dan belok (proporsional)
- QUIT         : Quit aman

Arsitektur kontrol (saat BALANCING):

    IMU (psi, psi_dot) ──┐
                         ├──> PD nominal -> CBF-QP safety filter -> motor
    Motor fb (theta_dot) ┘         (acados HPIPM, N=1)
"""

from __future__ import annotations

import logging
import select
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, Optional

import evdev
import numpy as np
from evdev import ecodes

import joystick_mapping
import tunning
from ddsm115 import DDSM115Error, DualDDSM115, MotorFeedback
from mpu6050 import AngleState, MPU6050Reader
from pd_control import BalancePDController, PDControlState

# Modul CBF-QP (baru)
from cbf_qp_controller import CbfQpController, CbfQpState
from robot_model import theta_dot_dari_feedback

# Modul evaluasi ISE (opsional, untuk skripsi)
try:
    from evaluation import EvaluasiISE
    _EVAL_TERSEDIA = True
except Exception:  # noqa: BLE001
    _EVAL_TERSEDIA = False

LOGGER = logging.getLogger("atera")

# Daftar halaman UI: ditambah halaman "cbf" untuk monitoring safety filter
PAGES = ("status", "cbf", "motor", "help")


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
    latest_cbf: Optional[CbfQpState] = None          # <-- hasil CBF-QP terbaru
    latest_motor_fb: Dict[str, Optional[MotorFeedback]] = field(
        default_factory=lambda: {"left": None, "right": None}
    )
    target_angle_deg: float = 0.0
    turn_command: float = 0.0
    theta_dot_rad_s: float = 0.0                     # kecepatan roda rata-rata
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
        """Mengubah rentang mentah axis EV_ABS (-32768 s/d 32767) ke -1.0 s/d 1.0."""
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

    def _process_event(self, event: evdev.InputEvent, app: AteraMainApp) -> None:
        # 1. Tombol Digital (EV_KEY)
        if event.type == ecodes.EV_KEY:
            if event.value == 1:  # Button Down Event
                btn_name = joystick_mapping.BTN_MAP.get(event.code)

                if btn_name == "MULAI":
                    app.toggle_balancing()
                elif btn_name == "Tombol Y":
                    app.calibrate()
                elif btn_name in ("Tombol X", "Tombol B"):
                    app.stop_balancing("Stop manual dari joystick.")
                elif btn_name in ("LB", "RB"):
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
        """Menghasilkan pasangan (target_angle_deg, turn_command)."""
        target_angle = 0.0
        turn_command = 0.0

        # Maju / Mundur: Prioritas Analog Y, fallback ke D-Pad Y
        if abs(self.analog_y) > 0.0:
            if self.analog_y > 0:
                target_angle = self.analog_y * float(tunning.MANUAL_FORWARD_TARGET_DEG)
            else:
                target_angle = abs(self.analog_y) * float(tunning.MANUAL_BACKWARD_TARGET_DEG)
        elif self.dpad_y != 0.0:
            if self.dpad_y < 0:  # PAD Atas / Maju
                target_angle = float(tunning.MANUAL_FORWARD_TARGET_DEG)
            elif self.dpad_y > 0:  # PAD Bawah / Mundur
                target_angle = float(tunning.MANUAL_BACKWARD_TARGET_DEG)

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
        self.controller = BalancePDController()
        self.joystick = JoystickController(tunning.JOYSTICK_PATH)
        self.state = RuntimeState()
        self._running = False
        self._motors_stopped = False
        self._last_ui_ts = 0.0

        # --- Safety filter CBF-QP (baru) ---
        self.cbf_aktif = bool(tunning.CBF_ENABLED)
        self.cbf: Optional[CbfQpController] = None
        if self.cbf_aktif:
            self.cbf = CbfQpController()
            if not self.cbf.siap:
                LOGGER.warning("CBF-QP tidak siap: %s", self.cbf.pesan_error)
                LOGGER.warning("Lanjut dengan PD saja (CBF dinonaktifkan sementara).")
                self.cbf_aktif = False

        # --- Modul evaluasi ISE (opsional, untuk skripsi) ---
        self.evaluasi: Optional[object] = None
        if _EVAL_TERSEDIA and bool(tunning.EVAL_ENABLED):
            try:
                self.evaluasi = EvaluasiISE()
            except Exception as exc:  # noqa: BLE001
                LOGGER.warning("Evaluasi ISE gagal diinisialisasi: %s", exc)
                self.evaluasi = None

    def setup(self) -> None:
        self.motors.open()
        self.motors.initialize()
        self.imu.open()
        if not self.joystick.open():
            LOGGER.warning("Gagal menginisialisasi joystick pada boot initial.")
        self._motors_stopped = True
        self.state.mode = "IDLE"
        self.state.last_message = "Hardware siap. Tekan Tombol Y untuk kalibrasi."
        LOGGER.info("ATERA main setup complete (CBF aktif=%s)", self.cbf_aktif)

    def cleanup(self) -> None:
        self.state.mode = "EXITING"
        # Tutup sesi evaluasi & simpan hasil
        if self.evaluasi is not None:
            try:
                self.evaluasi.tutup_dan_simpan()
            except Exception as exc:  # noqa: BLE001
                LOGGER.error("Gagal menyimpan hasil evaluasi: %s", exc)
        try:
            self.ensure_motors_stopped("cleanup")
        except Exception as exc:
            LOGGER.error("Failed stopping motors during cleanup: %s", exc)
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
        # Tandai sesi evaluasi berakhir karena fault
        if self.evaluasi is not None:
            try:
                self.evaluasi.hentikan()
            except Exception:  # noqa: BLE001
                pass
        self.set_message(f"FAULT: {reason}")


    def stop_balancing(self, reason: str = "Balancing dihentikan.") -> None:
        try:
            self.ensure_motors_stopped("stop balancing")
        finally:
            self.state.balance_started_at = None
            self.state.latest_pd = None
            self.state.latest_cbf = None
            self.state.target_angle_deg = 0.0
            self.state.turn_command = 0.0
            if self.state.mode != "FAULT":
                self.state.mode = "READY" if self.state.calibrated else "IDLE"
            self.set_message(reason)
            # Hentikan sesi evaluasi saat balancing berhenti
            if self.evaluasi is not None:
                try:
                    self.evaluasi.hentikan()
                except Exception:  # noqa: BLE001
                    pass

    def calibrate(self) -> None:
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
        if abs(angle.angle_deg) > float(tunning.SAFE_TILT_DEG):
            self.set_message(
                f"Sudut awal terlalu besar ({angle.angle_deg:.2f} deg). Tegakkan robot dulu."
            )
            return
        self.state.latest_angle = angle
        self.state.mode = "BALANCING"
        self.state.fault_reason = ""
        self.state.balance_started_at = time.monotonic()
        self._motors_stopped = False
        self.set_message("Balancing aktif.")
        # Mulai sesi evaluasi baru saat balancing dimulai
        if self.evaluasi is not None:
            try:
                self.evaluasi.mulai()
            except Exception as exc:  # noqa: BLE001
                LOGGER.warning("Gagal memulai evaluasi: %s", exc)

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

        # ------------------------------------------------------------------
        # Cutoff KERAS: |psi| >= 30 deg -> FAULT (sesuai permintaan Anda).
        # CBF bekerja di bawah batas ini (PSI_MAX_DEG = 15 deg) agar
        # cutoff keras tidak pernah tercapai selama kondisi normal.
        # ------------------------------------------------------------------
        if abs(angle_state.angle_deg) >= float(tunning.SAFE_TILT_DEG):
            self.set_fault(
                f"Tilt melebihi batas aman: {angle_state.angle_deg:.2f} deg >= {tunning.SAFE_TILT_DEG:.2f} deg"
            )
            return

        target_angle, turn_command = self.joystick.get_commands()
        angular_rate = self.get_angular_rate_from_state(angle_state)

        # ------------------------------------------------------------------
        # 1. PD nominal controller -> output ternormalisasi -1..1
        # ------------------------------------------------------------------
        pd_state = self.controller.compute(
            angle_deg=angle_state.angle_deg,
            angular_rate_deg_s=angular_rate,
            target_angle_deg=target_angle,
            turn_command=turn_command,
        )

        # ------------------------------------------------------------------
        # 2. Baca kecepatan roda (theta_dot) dari feedback motor terakhir
        # ------------------------------------------------------------------
        left_fb = self.state.latest_motor_fb.get("left")
        right_fb = self.state.latest_motor_fb.get("right")
        left_rpm = float(left_fb.speed_rpm) if left_fb is not None else 0.0
        right_rpm = float(right_fb.speed_rpm) if right_fb is not None else 0.0
        theta_dot_rad = theta_dot_dari_feedback(left_rpm, right_rpm)
        self.state.theta_dot_rad_s = theta_dot_rad

        # ------------------------------------------------------------------
        # 3. Safety filter CBF-QP (jika aktif)
        #    Input state dalam satuan rad & rad/s.
        #    Output PD ternormalisasi dikonversi ke Ampere di dalam CBF.
        # ------------------------------------------------------------------
        if self.cbf_aktif and self.cbf is not None:
            psi_rad = float(np.deg2rad(angle_state.angle_deg))
            psi_dot_rad = float(np.deg2rad(angular_rate))

            cbf_state = self.cbf.compute(
                psi_rad=psi_rad,
                psi_dot_rad_s=psi_dot_rad,
                theta_dot_rad_s=theta_dot_rad,
                u_pd_left_norm=pd_state.left_output,
                u_pd_right_norm=pd_state.right_output,
            )
            self.state.latest_cbf = cbf_state
            out_left = cbf_state.left_normalized
            out_right = cbf_state.right_normalized
        else:
            # Tanpa CBF: lewatkan output PD langsung (perilaku program lama)
            self.state.latest_cbf = None
            out_left = pd_state.left_output
            out_right = pd_state.right_output

        # ------------------------------------------------------------------
        # 4. Kirim perintah ke motor (mode current)
        # ------------------------------------------------------------------
        feedback = self.motors.command_normalized(out_left, out_right)

        # ------------------------------------------------------------------
        # 5. Simpan state & feedback untuk UI / logging
        # ------------------------------------------------------------------
        self.state.target_angle_deg = target_angle
        self.state.turn_command = turn_command
        self.state.latest_pd = pd_state
        self.state.latest_motor_fb["left"] = feedback.get("left")
        self.state.latest_motor_fb["right"] = feedback.get("right")
        self._motors_stopped = False

        # ------------------------------------------------------------------
        # 6. Catat data evaluasi ISE (error sudut dari target)
        # ------------------------------------------------------------------
        if self.evaluasi is not None and self.state.balance_started_at is not None:
            try:
                self.evaluasi.catat(
                    t_rel=time.monotonic() - self.state.balance_started_at,
                    angle_deg=angle_state.angle_deg,
                    target_deg=target_angle,
                )
            except Exception:  # noqa: BLE001
                pass


    def maybe_log_status(self) -> None:
        now = time.monotonic()
        if (now - self.state.last_status_log_ts) < float(tunning.STATUS_LOG_PERIOD_S):
            return
        self.state.last_status_log_ts = now

        angle = self.state.latest_angle.angle_deg if self.state.latest_angle else None
        rate = self.get_angular_rate_from_state(self.state.latest_angle) if self.state.latest_angle else None
        pd = self.state.latest_pd
        cbf = self.state.latest_cbf
        LOGGER.info(
            "mode=%s calibrated=%s angle=%s rate=%s target=%.3f turn=%.2f left=%.3f right=%.3f cbf=%s msg=%s",
            self.state.mode,
            self.state.calibrated,
            f"{angle:.3f}" if angle is not None else "-",
            f"{rate:.3f}" if rate is not None else "-",
            self.state.target_angle_deg,
            self.state.turn_command,
            pd.left_output if pd else 0.0,
            pd.right_output if pd else 0.0,
            "ON" if (self.cbf_aktif and cbf is not None) else "off",
            self.state.last_message,
        )

    def render_ui(self) -> None:
        page = PAGES[self.state.page_index]
        angle = self.state.latest_angle
        pd = self.state.latest_pd
        cbf = self.state.latest_cbf
        left_fb = self.state.latest_motor_fb.get("left")
        right_fb = self.state.latest_motor_fb.get("right")

        lines = []
        lines.append("\x1b[2J\x1b[H")
        lines.append("ATERA SELF-BALANCING ROBOT (PD + CBF-QP)")
        lines.append("=" * 72)
        lines.append(
            f"Mode: {self.state.mode:<12} | Calibrated: {str(self.state.calibrated):<5} | "
            f"Page: {page} ({self.state.page_index + 1}/{len(PAGES)})"
        )
        lines.append(
            f"Loop: {self.state.loop_hz_est:7.1f} Hz | Control target: {tunning.CONTROL_HZ:.1f} Hz | "
            f"UI target: {tunning.UI_HZ:.1f} Hz"
        )
        cbf_status = "AKTIF" if self.cbf_aktif else "NONAKTIF"
        lines.append(f"CBF-QP: {cbf_status} | Batas psi: +/-{tunning.PSI_MAX_DEG:.0f} deg | "
                     f"theta_dot: +/-{tunning.THETA_DOT_MAX_DEG_S:.0f} deg/s")
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
                lines.append(f"Angle (psi)         : {angle.angle_deg:+8.3f} deg")
                lines.append(f"Angular rate        : {angular_rate:+8.3f} deg/s")
                lines.append(f"Kalman X / Y        : {angle.kalman_x:+8.3f} / {angle.kalman_y:+8.3f} deg")
                lines.append(f"Accel angle X / Y   : {angle.acc_angle_x:+8.3f} / {angle.acc_angle_y:+8.3f} deg")
                lines.append(f"dt                  : {angle.dt * 1000.0:8.3f} ms")
            lines.append("")
            lines.append(f"Target angle        : {self.state.target_angle_deg:+8.3f} deg")
            lines.append(f"Turn command        : {self.state.turn_command:+8.3f}")
            if pd is None:
                lines.append("PD output           : belum aktif")
            else:
                lines.append(f"Error / rate err    : {pd.error_deg:+8.3f} / {pd.error_rate_deg_s:+8.3f}")
                lines.append(f"Base output         : {pd.base_output:+8.3f}")
                lines.append(f"Left / Right output : {pd.left_output:+8.3f} / {pd.right_output:+8.3f}")

        elif page == "cbf":
            lines.append("MONITORING CBF-QP (SAFETY FILTER)")
            lines.append(f"  Status solver     : {'SIAP' if (self.cbf and self.cbf.siap) else 'TIDAK SIAP'}")
            lines.append(f"  Alpha_1 / Alpha_2 : {tunning.Alpha_1:.2f} / {tunning.Alpha_2:.2f}")
            lines.append("")
            if cbf is None:
                lines.append("  Belum ada data CBF (balancing belum aktif).")
            else:
                lines.append(f"  psi (state)       : {self.state.latest_angle.angle_deg if self.state.latest_angle else 0.0:+8.3f} deg")
                lines.append(f"  theta_dot (state) : {np.rad2deg(self.state.theta_dot_rad_s):+8.3f} deg/s")
                lines.append("")
                lines.append("  BARRIER FUNCTION h(x) (harus >= 0 agar aman):")
                lines.append(f"    h1 (-psi+max)   : {cbf.h1:+9.4f} rad")
                lines.append(f"    h2 (+psi+max)   : {cbf.h2:+9.4f} rad")
                lines.append(f"    h3 (-thd+max)   : {cbf.h3:+9.4f} rad/s")
                lines.append(f"    h4 (+thd+max)   : {cbf.h4:+9.4f} rad/s")
                lines.append("")
                lines.append("  SLACK (besar = PD dilanggar/dikoreksi CBF):")
                s = cbf.slack
                lines.append(f"    s1={s[0]:.2e}  s2={s[1]:.2e}  s3={s[2]:.2e}  s4={s[3]:.2e}")
                lines.append("")
                lines.append("  OUTPUT (Ampere):")
                lines.append(f"    u_PD   L / R    : {cbf.u_pd_left_a:+7.3f} / {cbf.u_pd_right_a:+7.3f} A")
                lines.append(f"    u_CBF  L / R    : {cbf.u_opt_left_a:+7.3f} / {cbf.u_opt_right_a:+7.3f} A")
                lines.append(f"    Filter aktif?   : {'YA (PD dikoreksi)' if cbf.filter_aktif else 'tidak (PD diteruskan)'}")
                lines.append(f"    Solve time      : {cbf.solve_time_ms:.3f} ms | status={cbf.solver_status}")

        elif page == "motor":
            lines.append(f"Motor control mode  : {tunning.MOTOR_CONTROL_MODE}")
            lines.append(f"Current limit       : {tunning.MAX_CURRENT_A:.3f} A")
            lines.append(f"Speed limit         : {tunning.MAX_SPEED_RPM:.3f} rpm")
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
            lines.append(f"  safe_tilt_deg     : {tunning.SAFE_TILT_DEG:.2f}")

        else:
            lines.append("BANTUAN KONTROL JOYSTICK")
            lines.append("  Tombol Y            : Kalibrasi gyro + zero angle")
            lines.append("  MULAI               : Start / Stop balancing")
            lines.append("  Analog Kiri / D-Pad Y : Maju / Mundur (Proporsional)")
            lines.append("  Analog / D-Pad X      : Belok Kiri / Kanan")
            lines.append("  Tombol X / B        : Stop balancing")
            lines.append("  LB / RB             : Pindah halaman UI")
            lines.append("  QUIT                : Quit aman")
            lines.append("")
            lines.append("HALAMAN UI:")
            lines.append("  status : info IMU & PD")
            lines.append("  cbf    : monitoring safety filter CBF-QP")
            lines.append("  motor  : feedback motor & kalibrasi")
            lines.append("  help   : halaman ini")

        lines.append("-" * 72)
        lines.append("Keys: Y calibrate | MULAI balance | Analog/DPAD move | LB/RB page | QUIT exit")

        sys.stdout.write("\n".join(lines) + "\n")
        sys.stdout.flush()


    def run(self) -> None:
        """Loop utama: kontrol pada CONTROL_HZ, UI pada UI_HZ (non-blocking)."""
        self._running = True
        dt_kontrol = 1.0 / float(tunning.CONTROL_HZ)
        dt_ui = 1.0 / float(tunning.UI_HZ)
        next_kontrol = time.monotonic()
        next_ui = time.monotonic()

        LOGGER.info(
            "Masuk run loop: kontrol=%.0f Hz, UI=%.0f Hz, CBF=%s",
            float(tunning.CONTROL_HZ), float(tunning.UI_HZ),
            "AKTIF" if self.cbf_aktif else "nonaktif",
        )

        while self._running and not self.state.exit_requested:
            now = time.monotonic()

            # 1. Selalu polling joystick (menangani tombol & axis, non-blocking)
            try:
                self.joystick.poll_events(self)
            except Exception as exc:  # noqa: BLE001
                LOGGER.error("Joystick poll error: %s", exc)

            # 2. Langkah kontrol pada frekuensi CONTROL_HZ
            if now >= next_kontrol:
                t0 = time.monotonic()
                try:
                    self.control_step()
                except DDSM115Error as exc:
                    self.set_fault(f"Kesalahan komunikasi motor: {exc}")
                except Exception as exc:  # noqa: BLE001
                    LOGGER.exception("Kesalahan tak terduga di control_step")
                    self.set_fault(f"control_step exception: {exc}")
                self.state.last_loop_dt = time.monotonic() - t0
                if self.state.last_loop_dt > 1e-9:
                    self.state.loop_hz_est = 1.0 / max(dt_kontrol, self.state.last_loop_dt)
                next_kontrol += dt_kontrol
                # Jika loop tertinggal jauh (mis. logging lama), reset jadwal
                if next_kontrol < time.monotonic() - 5.0 * dt_kontrol:
                    next_kontrol = time.monotonic() + dt_kontrol

            # 3. Logging status berkala (stdout journal)
            self.maybe_log_status()

            # 4. Render UI terminal pada frekuensi UI_HZ
            if now >= next_ui:
                try:
                    self.render_ui()
                except Exception:  # noqa: BLE001
                    pass  # UI tidak boleh merusak loop kontrol
                next_ui += dt_ui

            # 5. Tidur singkat agar CPU tidak 100% (target ~0.2 ms)
            sisa = min(next_kontrol, next_ui) - time.monotonic()
            if sisa > 0.001:
                time.sleep(min(sisa, 0.002))

        self._running = False
        LOGGER.info("Keluar dari run loop (mode=%s).", self.state.mode)

    def minta_keluar(self) -> None:
        """Set flag keluar dengan aman (dipanggil dari handler joystick/sinyal)."""
        self.state.exit_requested = True
        self.set_message("Permintaan keluar diterima. Mematikan...")


def _pasang_handler_sinyal(app: AteraMainApp) -> None:
    """Tangkap SIGINT/SIGTERM agar motor selalu berhenti dengan aman."""
    import signal

    def _handler(signum, _frame):  # noqa: ANN001, ANN202
        LOGGER.warning("Sinyal %s diterima -> keluar aman.", signum)
        app.minta_keluar()

    signal.signal(signal.SIGINT, _handler)
    signal.signal(signal.SIGTERM, _handler)


def main() -> int:
    """Entry point program."""
    import argparse

    parser = argparse.ArgumentParser(
        description="ATERA TWSBR - PD nominal + CBF-QP safety filter (acados HPIPM N=1)."
    )
    parser.add_argument(
        "--tanpa-cbf", action="store_true",
        help="Nonaktifkan safety filter CBF-QP (jalankan PD murni).",
    )
    parser.add_argument(
        "--tanpa-eval", action="store_true",
        help="Nonaktifkan pencatatan evaluasi ISE.",
    )
    parser.add_argument(
        "--log-level", default="INFO",
        choices=("DEBUG", "INFO", "WARNING", "ERROR"),
        help="Level logging terminal (default: INFO).",
    )
    args = parser.parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        stream=sys.stderr,
    )

    if args.tanpa_cbf:
        tunning.CBF_ENABLED = False
        LOGGER.info("Opsi --tanpa-cbf: safety filter dinonaktifkan.")
    if args.tanpa_eval:
        tunning.EVAL_ENABLED = False
        LOGGER.info("Opsi --tanpa-eval: evaluasi ISE dinonaktifkan.")

    app = AteraMainApp()
    _pasang_handler_sinyal(app)

    try:
        app.setup()
    except Exception as exc:  # noqa: BLE001
        LOGGER.exception("Setup hardware gagal: %s", exc)
        return 2

    kode_keluar = 0
    try:
        app.run()
    except KeyboardInterrupt:
        LOGGER.warning("KeyboardInterrupt -> keluar aman.")
    except Exception as exc:  # noqa: BLE001
        LOGGER.exception("Kesalahan fatal di run loop: %s", exc)
        kode_keluar = 1
    finally:
        try:
            app.cleanup()
        except Exception as exc:  # noqa: BLE001
            LOGGER.error("Cleanup gagal: %s", exc)

    LOGGER.info("Program selesai dengan kode %d.", kode_keluar)
    return kode_keluar


if __name__ == "__main__":
    sys.exit(main())
