"""Program utama self-balancing robot ATERA (Versi Joystick).

Fitur utama:
- Integrasi MPU6050 + Kalman filter + PD controller (loop psi + loop theta) + dua motor DDSM115
- State machine sederhana: IDLE, CALIBRATING, READY, BALANCING, FAULT, EXITING
- Kontrol joystick non-blocking via evdev menggunakan mapping nama dari joystick_mapping.py
- UI terminal sederhana dengan beberapa halaman (LB / RB untuk pindah)
- Shutdown aman: motor dihentikan saat fault, tilt berlebih, atau quit
- Evaluasi error (ise.py): merekam error psi/theta selama balancing dan
  menyimpan grafiknya otomatis ke folder PLOT_EVALUASI saat balancing berhenti

Pemetaan Kontrol Joystick (berdasarkan joystick_mapping.py):
- Tombol Y     : Kalibrasi gyro + zero angle
- MULAI        : Start/Stop balancing (Toggle)
- Tombol X/B   : Stop balancing
- LB / RB      : Ganti halaman UI
- Analog/D-Pad : Maju, mundur, dan belok (proporsional & responsif)
- QUIT         : Quit aman

CATATAN PERUBAHAN vs versi sebelumnya (lihat juga config.py, pd_control.py,
ise.py untuk detail masing-masing):
1. tunning.py di-rename menjadi config.py (isi sama persis, hanya nama file
   yang berubah). Baris import di file ini (dan file lain yang tadinya
   `import tunning`) cukup diubah menjadi `import config as tunning` supaya
   seluruh pemanggilan `tunning.NAMA_KONSTANTA` di kode lama TETAP berjalan
   tanpa perlu diubah satu per satu (perubahan seminimal mungkin).
2. pd_control.py sekarang punya loop tambahan (theta / sudut roda dari
   encoder DDSM115) di samping loop psi (sudut badan dari MPU6050) yang
   sudah terbukti stabil. Loop theta nonaktif secara default
   (Kp_theta = Kd_theta = 0 di config.py) sehingga perilaku default tetap
   semirip mungkin dengan versi lama.
3. Ditambahkan integrasi dengan ise.py (EvaluasiError): mulai mencatat saat
   tombol MULAI ditekan, lalu menyimpan grafik evaluasi otomatis ke folder
   PLOT_EVALUASI saat balancing berhenti (baik dihentikan manual, fault,
   maupun saat program keluar).
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

import joystick_mapping
import config as tunning
from ddsm115 import DDSM115Error, DualDDSM115, MotorFeedback
from mpu6050 import AngleState, MPU6050Reader
from pd_control import BalancePDController, PDControlState
from ise import EvaluasiError

LOGGER = logging.getLogger("atera")
PAGES = ("status", "motor", "help")


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
    target_angle_deg: float = 0.0
    turn_command: float = 0.0
    last_loop_dt: float = 0.0
    loop_hz_est: float = 0.0

    zero_offset_deg: float = 0.0
    gyro_bias_x: float = 0.0
    gyro_bias_y: float = 0.0
    balance_started_at: Optional[float] = None
    last_status_log_ts: float = 0.0

    # --- Referensi loop theta (roda), lihat perbaikan "robot berputar" ---
    # Diintegrasikan dari kecepatan roda (bukan dibaca langsung dari posisi
    # absolut encoder yang wrap 0-360 derajat & tidak sinkron antara roda
    # kiri & kanan), dan di-nol-kan ulang setiap start_balancing().
    wheel_theta_left_deg: float = 0.0
    wheel_theta_right_deg: float = 0.0


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
        self.evaluator = EvaluasiError()
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
        self._motors_stopped = True
        self.state.mode = "IDLE"
        self.state.last_message = "Hardware siap. Tekan Tombol Y untuk kalibrasi."
        LOGGER.info("ATERA main setup complete")

    def cleanup(self) -> None:
        self.state.mode = "EXITING"
        try:
            self.evaluator.stop_and_save_session()
        except Exception as exc:
            LOGGER.error("Failed saving evaluation plot during cleanup: %s", exc)
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
            self.evaluator.stop_and_save_session()
        except Exception as exc:
            LOGGER.error("Failed saving evaluation plot after fault: %s", exc)
        try:
            self.ensure_motors_stopped("fault")
        except Exception as exc:
            LOGGER.error("Failed to stop motors after fault: %s", exc)
        self.set_message(f"FAULT: {reason}")

    def stop_balancing(self, reason: str = "Balancing dihentikan.") -> None:
        try:
            self.ensure_motors_stopped("stop balancing")
        finally:
            self.evaluator.stop_and_save_session()
            self.state.balance_started_at = None
            self.state.latest_pd = None
            self.state.target_angle_deg = 0.0
            self.state.turn_command = 0.0
            if self.state.mode != "FAULT":
                self.state.mode = "READY" if self.state.calibrated else "IDLE"
            self.set_message(reason)

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
        # Nol-kan referensi loop theta setiap sesi balancing baru dimulai,
        # supaya roda kiri & kanan mulai dari error yang sama-sama nol
        # (lihat perbaikan "robot berputar saat Kp_theta > 0").
        self.state.wheel_theta_left_deg = 0.0
        self.state.wheel_theta_right_deg = 0.0
        # Mulai sesi evaluasi error (ise.py) tepat saat balancing benar-benar aktif.
        self.evaluator.start_session()
        self.set_message("Balancing aktif.")

    def toggle_balancing(self) -> None:
        if self.state.mode == "BALANCING":
            self.stop_balancing("Balancing dihentikan oleh user.")
        else:
            self.start_balancing()

    def get_angular_rate_from_state(self, angle_state: AngleState) -> float:
        if angle_state.axis_used == "roll":
            return angle_state.gyro_rate_x
        return angle_state.gyro_rate_y

    def _update_theta_from_feedback(self, dt: float) -> None:
        """Perbarui estimasi sudut roda (theta) kiri & kanan dengan
        MENGINTEGRASIKAN kecepatan roda (speed_rpm) dari feedback encoder
        DDSM115 siklus kontrol SEBELUMNYA terhadap dt, relatif ke posisi
        saat start_balancing() (di-nol-kan di sana).

        PENTING - kenapa TIDAK memakai fb.position_deg langsung (versi lama):
        1. position_deg adalah posisi ABSOLUT single-turn (0-360 derajat)
           hasil wrap dari register encoder, bukan jarak tempuh relatif.
           Nilainya tergantung di sudut mana motor kebetulan berhenti
           terakhir kali - tidak ada hubungan antara nilai roda kiri &
           kanan. Kalau dipakai langsung sebagai error_theta, robot yang
           diam & tegak sempurna pun akan mendapat error_theta_left !=
           error_theta_right -> torsi kiri/kanan jadi tidak simetris ->
           robot 'dipaksa' berputar walau tidak pernah diminta belok.
        2. position_deg juga wrap setiap 360 derajat, menyebabkan lompatan
           error yang tiba-tiba (mis. dari -1 ke -361) saat roda berputar
           terus selama balancing.
        Dengan mengintegrasikan kecepatan (dan menolkan referensi di awal
        setiap sesi balancing), roda kiri & kanan mulai dari error yang
        sama-sama nol dan tidak pernah wrap, sehingga loop theta hanya
        bereaksi terhadap PERGESERAN NYATA robot, bukan artefak encoder.
        """
        left_fb = self.state.latest_motor_fb.get("left")
        right_fb = self.state.latest_motor_fb.get("right")
        left_dot = (left_fb.speed_rpm * 6.0 * self.motors.left.sign) if left_fb else 0.0
        right_dot = (right_fb.speed_rpm * 6.0 * self.motors.right.sign) if right_fb else 0.0
        self.state.wheel_theta_left_deg += left_dot * dt
        self.state.wheel_theta_right_deg += right_dot * dt

    def _theta_from_feedback(self, side: str) -> tuple[float, float]:
        """Mengembalikan (angle_theta, angular_dot_theta) dalam derajat &
        derajat/detik untuk salah satu roda, hasil integrasi terhadap
        referensi nol di awal balancing (lihat _update_theta_from_feedback)."""
        fb = self.state.latest_motor_fb.get(side)
        sign = self.motors.left.sign if side == "left" else self.motors.right.sign
        angular_dot_theta = (fb.speed_rpm * 6.0 * sign) if fb else 0.0  # rpm -> deg/s
        angle_theta = self.state.wheel_theta_left_deg if side == "left" else self.state.wheel_theta_right_deg
        return angle_theta, angular_dot_theta

    def control_step(self) -> None:
        angle_state = self.imu.read_angles()
        self.state.latest_angle = angle_state

        if self.state.mode != "BALANCING":
            return

        if abs(angle_state.angle_deg) >= float(tunning.SAFE_TILT_DEG):
            self.set_fault(
                f"Tilt melebihi batas aman: {angle_state.angle_deg:.2f} deg >= {tunning.SAFE_TILT_DEG:.2f} deg"
            )
            return

        target_angle, turn_command = self.joystick.get_commands()
        angular_rate = self.get_angular_rate_from_state(angle_state)

        # angle_theta / angular_dot_theta (loop roda) dihitung dari feedback
        # encoder DDSM115 hasil siklus kontrol SEBELUMNYA (feedback siklus
        # saat ini baru tersedia setelah motor dikirim perintah, beberapa
        # baris di bawah) — pola umum untuk feedback loop berbasis serial.
        # Diintegrasikan (bukan dibaca langsung dari posisi absolut) supaya
        # kiri & kanan punya referensi nol yang sama — lihat
        # _update_theta_from_feedback() untuk alasan detailnya.
        self._update_theta_from_feedback(angle_state.dt)
        angle_theta_left, angular_dot_theta_left = self._theta_from_feedback("left")
        angle_theta_right, angular_dot_theta_right = self._theta_from_feedback("right")

        pd_state = self.controller.compute(
            angle_deg=angle_state.angle_deg,
            angular_rate_deg_s=angular_rate,
            target_angle_deg=target_angle,
            turn_command=turn_command,
            angle_theta_left=angle_theta_left,
            angle_theta_right=angle_theta_right,
            angular_dot_theta_left=angular_dot_theta_left,
            angular_dot_theta_right=angular_dot_theta_right,
        )
        feedback = self.motors.command_normalized(pd_state.left_output, pd_state.right_output)

        self.state.target_angle_deg = target_angle
        self.state.turn_command = turn_command
        self.state.latest_pd = pd_state
        self.state.latest_motor_fb["left"] = feedback.get("left")
        self.state.latest_motor_fb["right"] = feedback.get("right")
        self._motors_stopped = False

        # Catat error untuk evaluasi (ise.py): badan (psi), roda kiri & kanan (theta).
        self.evaluator.record(
            error_psi=pd_state.error_psi,
            error_theta_left=pd_state.error_theta_left,
            error_theta_right=pd_state.error_theta_right,
        )

    def maybe_log_status(self) -> None:
        now = time.monotonic()
        if (now - self.state.last_status_log_ts) < float(tunning.STATUS_LOG_PERIOD_S):
            return
        self.state.last_status_log_ts = now

        angle = self.state.latest_angle.angle_deg if self.state.latest_angle else None
        rate = self.get_angular_rate_from_state(self.state.latest_angle) if self.state.latest_angle else None
        pd = self.state.latest_pd
        LOGGER.info(
            "mode=%s calibrated=%s angle=%s rate=%s target=%.3f turn=%.2f left=%.3f right=%.3f msg=%s",
            self.state.mode,
            self.state.calibrated,
            f"{angle:.3f}" if angle is not None else "-",
            f"{rate:.3f}" if rate is not None else "-",
            self.state.target_angle_deg,
            self.state.turn_command,
            pd.left_output if pd else 0.0,
            pd.right_output if pd else 0.0,
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
            f"Loop: {self.state.loop_hz_est:7.1f} Hz | Control target: {tunning.CONTROL_HZ:.1f} Hz | "
            f"UI target: {tunning.UI_HZ:.1f} Hz"
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
                lines.append(f"Angle               : {angle.angle_deg:+8.3f} deg")
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
                lines.append(f"Base output (psi)   : {pd.base_output:+8.3f}")
                lines.append(f"Left / Right output : {pd.left_output:+8.3f} / {pd.right_output:+8.3f}")
                lines.append(f"Theta kiri  err/dot : {pd.error_theta_left:+8.3f} / {pd.angular_dot_theta_left:+8.3f}")
                lines.append(f"Theta kanan err/dot : {pd.error_theta_right:+8.3f} / {pd.angular_dot_theta_right:+8.3f}")
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
        lines.append("-" * 72)
        lines.append("Keys: Y calibrate | MULAI balance | Analog/DPAD move | LB/RB page | QUIT exit")
        sys.stdout.write("\n".join(lines) + "\n")
        sys.stdout.flush()

    def run(self) -> None:
        control_period = 1.0 / float(tunning.CONTROL_HZ)
        ui_period = 1.0 / float(tunning.UI_HZ)
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