"""Program Utama Atera (atera_main.py) - Master Safety Arming Version

Mengintegrasikan MPU6050 IMU, DDSM115 RS485 Motors, Balance PD Controller,
Joystick Input Event Reader, serta Master Safety Arming Switch (Tombol MULAI).
"""

from __future__ import annotations

import logging
import os
import select
import struct
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

# Import modul lokal
import config
from ddsm115 import Angle_THETA, DDSM115Dual, DDSM115Error, MotorFeedback
from joystick_mapping import ABS_MAP, BTN_MAP, DPAD_MAP
from mpu6050 import AngleState, MPU6050Sensor
from pd_control import BalancePDController, PDControlState

# Impor opsional evdev untuk fleksibilitas pembacaan joystick
try:
    import evdev
except ImportError:
    evdev = None

LOGGER = logging.getLogger("atera_main")


@dataclass
class InputEventData:
    """Dataclass pembungkus data event biner/analog joystick."""

    type: int
    code: int
    value: int


@dataclass
class RobotRuntimeState:
    """Dataclass status runtime sistem untuk pemantauan GUI & FSM."""

    mode: str = "IDLE"  # IDLE, CALIBRATING, READY, BALANCE, MONO, NUGGET, FAULT, EXITING
    control_algorithm: str = "PD"  # PD atau PD+CBF-QP
    calibrated: bool = False
    is_armed: bool = False  # Feature Opsi 1: Master Safety Arming Switch
    exit_requested: bool = False
    last_message: str = (
        "Sistem Siap. Tekan Y (Kalibrasi), lalu MULAI (Master Arming)."
    )
    fault_reason: str = ""

    # Telemetri Hardware & Controller
    latest_angle_state: Optional[AngleState] = None
    latest_theta_state: Optional[Angle_THETA] = None
    latest_pd_state: Optional[PDControlState] = None
    latest_motor_fb: Dict[str, Optional[MotorFeedback]] = field(
        default_factory=lambda: {"left": None, "right": None}
    )

    # Command & Timing
    target_pitch_deg: float = 0.0
    turn_command: float = 0.0
    combo_a_pressed: bool = False
    loop_dt: float = 0.0
    loop_hz_est: float = 0.0
    zero_offset_deg: float = 0.0
    gyro_bias_x: float = 0.0
    gyro_bias_y: float = 0.0
    last_ui_render_ts: float = 0.0


class JoystickHandler:
    """Pengelola input Joystick non-blocking via Linux Input Event (/dev/input/event*)."""

    def __init__(self, device_path: str = config.JOYSTICK_PATH) -> None:
        self.device_path = device_path
        self.evdev_device = None
        self.fd: Optional[int] = None
        self.enabled = False
        self.use_evdev = False

    def __enter__(self) -> JoystickHandler:
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def open(self) -> None:
        if evdev is not None:
            try:
                self.evdev_device = evdev.InputDevice(self.device_path)
                self.use_evdev = True
                self.enabled = True
                LOGGER.info(
                    "Joystick terhubung via evdev: %s (%s)",
                    self.device_path,
                    self.evdev_device.name,
                )
                return
            except Exception as exc:
                LOGGER.warning(
                    "Gagal membuka joystick via evdev di %s: %s",
                    self.device_path,
                    exc,
                )

        try:
            self.fd = os.open(self.device_path, os.O_RDONLY | os.O_NONBLOCK)
            self.use_evdev = False
            self.enabled = True
            LOGGER.info(
                "Joystick terhubung via raw event descriptor: %s",
                self.device_path,
            )
        except Exception as exc:
            LOGGER.error("Gagal membuka joystick di %s: %s", self.device_path, exc)
            self.enabled = False

    def close(self) -> None:
        if self.use_evdev and self.evdev_device:
            try:
                self.evdev_device.close()
            except Exception:
                pass
            self.evdev_device = None
        elif self.fd is not None:
            try:
                os.close(self.fd)
            except Exception:
                pass
            self.fd = None
        self.enabled = False

    def read_events(self) -> List[InputEventData]:
        if not self.enabled:
            return []

        events: List[InputEventData] = []
        if self.use_evdev and self.evdev_device:
            try:
                for ev in self.evdev_device.read():
                    events.append(InputEventData(ev.type, ev.code, ev.value))
            except BlockingIOError:
                pass
            except Exception as exc:
                LOGGER.error("Error reading evdev events: %s", exc)

        elif self.fd is not None:
            is_64bit = sys.maxsize > 2**32
            event_format = "llHHi" if is_64bit else "iiHHi"
            event_size = struct.calcsize(event_format)

            while True:
                ready, _, _ = select.select([self.fd], [], [], 0)
                if not ready:
                    break
                try:
                    data = os.read(self.fd, event_size)
                    if not data or len(data) < event_size:
                        break
                    _, _, ev_type, ev_code, ev_value = struct.unpack(
                        event_format, data[:event_size]
                    )
                    events.append(InputEventData(ev_type, ev_code, ev_value))
                except (BlockingIOError, OSError):
                    break

        return events


class AteraMainApp:
    """Aplikasi Utama Orkestrator Robot Penyeimbang Atera."""

    def __init__(self) -> None:
        self.motors = DDSM115Dual()
        self.imu = MPU6050Sensor()
        self.controller = BalancePDController()
        self.state = RobotRuntimeState()
        self._running = False
        self._motors_stopped = False

        # Status Sumbu Joystick
        self._joy_dpad_fb = 0.0
        self._joy_dpad_turn = 0.0
        self._joy_analog_fb = 0.0
        self._joy_analog_turn = 0.0

    def setup(self) -> None:
        """Inisialisasi perangkat keras dan state awal."""
        self.motors.open()
        self.motors.initialize()
        self.imu.open()
        self._motors_stopped = True
        self.state.mode = "IDLE"
        self.set_message("Hardware Berhasil Dinisialisasi. Tekan Y untuk Kalibrasi.")

    def cleanup(self) -> None:
        """Pembersihan dan penutupan koneksi hardware secara aman."""
        self.state.mode = "EXITING"
        try:
            self.ensure_motors_stopped("Cleanup Shutdown")
        except Exception as exc:
            LOGGER.error("Gagal menghentikan motor saat cleanup: %s", exc)
        try:
            self.motors.close()
        except Exception as exc:
            LOGGER.error("Gagal menutup port motor: %s", exc)
        try:
            self.imu.close()
        except Exception as exc:
            LOGGER.error("Gagal menutup koneksi IMU: %s", exc)

    def ensure_motors_stopped(self, reason: str = "") -> None:
        if self._motors_stopped:
            return
        self.motors.stop_all()
        self._motors_stopped = True
        if reason:
            LOGGER.info("Motor dihentikan: %s", reason)

    def set_message(self, msg: str) -> None:
        self.state.last_message = msg
        LOGGER.info(msg)

    def set_fault(self, reason: str) -> None:
        self.state.mode = "FAULT"
        self.state.is_armed = False
        self.state.fault_reason = reason
        try:
            self.ensure_motors_stopped("FAULT Safe Trigger")
        except Exception as exc:
            LOGGER.error("Gagal menghentikan motor saat FAULT: %s", exc)
        self.set_message(f"SYSTEM FAULT: {reason}")

    def toggle_arming(self) -> None:
        """Fitur Opsi 1: Toggle Master Safety Arming Switch."""
        if not self.state.calibrated:
            self.set_message("Robot Belum Dikalibrasi! Tekan Tombol Y Terlebih Dahulu.")
            return

        if self.state.is_armed:
            self.state.is_armed = False
            self.stop_balancing("SYSTEM DISARMED: Standby Mode Aktif via MULAI.")
            self.set_message("SYSTEM DISARMED (STANDBY). Motor Terkunci & Aman.")
        else:
            self.state.is_armed = True
            self.set_message(
                "SYSTEM ARMED! Siap Menerima Perintah (Tombol B / Combo LT/RT)."
            )

    def stop_balancing(self, reason: str = "Balancing Dihentikan.") -> None:
        try:
            self.ensure_motors_stopped("User Stop")
        finally:
            self.state.latest_pd_state = None
            self.state.target_pitch_deg = 0.0
            self.state.turn_command = 0.0
            if self.state.mode != "FAULT":
                self.state.mode = "READY" if self.state.calibrated else "IDLE"
            self.set_message(reason)

    def calibrate(self) -> None:
        prev_mode = self.state.mode
        self.state.mode = "CALIBRATING"
        self.set_message("Proses Kalibrasi Gyro & Zero Offset... Jaga Robot Diam Tegak!")
        try:
            self.ensure_motors_stopped("Kalibrasi Berjalan")
            res = self.imu.run_full_calibration()
            self.state.gyro_bias_x = res["gyro_bias_x"]
            self.state.gyro_bias_y = res["gyro_bias_y"]
            self.state.zero_offset_deg = res["zero_offset_deg"]
            self.state.calibrated = True
            self.state.mode = "READY"
            self.set_message(
                f"Kalibrasi Sukses! Zero Offset: {self.state.zero_offset_deg:.2f}°. Tekan MULAI untuk Arming."
            )
        except Exception as exc:
            self.state.mode = prev_mode
            self.set_fault(f"Gagal Kalibrasi: {exc}")

    def start_mode(self, target_mode: str) -> None:
        if not self.state.calibrated:
            self.set_message("Robot Belum Dikalibrasi! Tekan Tombol Y Terlebih Dahulu.")
            return

        # Pengecekan Kunci Master Safety Arming
        if not self.state.is_armed:
            self.set_message(
                "PERINTAH DITOLAK: Sistem STANDBY (LOCKED)! Tekan MULAI untuk Arming."
            )
            return

        try:
            state = self.imu.read_angles()
        except Exception as exc:
            self.set_fault(f"Gagal Membaca Sensor IMU: {exc}")
            return

        if abs(state.angle_deg) > float(config.HARD_SAFE_TILT_DEG):
            self.set_message(
                f"Sudut Terlalu Miring ({state.angle_deg:.1f}°). Posisikan Robot Tegak!"
            )
            return

        self.state.mode = target_mode
        self.state.fault_reason = ""
        self._motors_stopped = False
        self.set_message(f"Mode {target_mode} Aktif!")

    def process_joystick_events(self, events: List[InputEventData]) -> None:
        for ev in events:
            # 1. Tombol Biner Digital (EV_KEY = 1)
            if ev.type == 1:
                btn_name = BTN_MAP.get(ev.code, "")

                if btn_name == "Tombol A":
                    self.state.combo_a_pressed = ev.value == 1

                elif ev.value == 1:  # Button Down Press
                    if btn_name == "MULAI":
                        # Toggle Master Safety Switch
                        self.toggle_arming()

                    elif btn_name == "Tombol Y":
                        self.calibrate()

                    elif btn_name == "Tombol B":
                        self.start_mode("BALANCE")

                    elif btn_name == "LT (Digital)":
                        if self.state.combo_a_pressed:
                            self.start_mode("MONO")
                        else:
                            self.set_message("Tekan Combo Tombol A + LT untuk Mode MONO")

                    elif btn_name == "RT (Digital)":
                        if self.state.combo_a_pressed:
                            self.start_mode("NUGGET")
                        else:
                            self.set_message("Tekan Combo Tombol A + RT untuk Mode NUGGET")

                    elif btn_name == "LB":
                        self.state.control_algorithm = "PD"
                        self.set_message("Algoritma Pengontrol: PD Mode")

                    elif btn_name == "RB":
                        self.state.control_algorithm = "PD + CBF-QP"
                        self.set_message("Algoritma Pengontrol: PD + CBF-QP Mode")

                    elif btn_name == "Tombol X":
                        self.stop_balancing("Perintah Stop Manual (Tombol X)")

                    elif btn_name == "QUIT":
                        self.state.exit_requested = True
                        self.set_message("Permintaan Quit via Joystick.")

            # 2. Sumbu Analog & D-Pad (EV_ABS = 3)
            elif ev.type == 3:
                if ev.code in DPAD_MAP:
                    if ev.code == 17:  # D-Pad Y (Maju / Mundur)
                        self._joy_dpad_fb = -1.0 if ev.value < 0 else (1.0 if ev.value > 0 else 0.0)
                    elif ev.code == 16:  # D-Pad X (Kiri / Kanan)
                        self._joy_dpad_turn = -1.0 if ev.value < 0 else (1.0 if ev.value > 0 else 0.0)

                elif ev.code in ABS_MAP:
                    raw_val = float(ev.value)
                    max_val = 32767.0 if abs(raw_val) > 255 else 127.0
                    norm_val = raw_val / max_val
                    deadzone = 0.15
                    if abs(norm_val) < deadzone:
                        norm_val = 0.0

                    if ev.code == 0:  # Analog Kiri Y
                        self._joy_analog_fb = -norm_val
                    elif ev.code in (1, 5):  # Analog Kiri/Kanan X
                        self._joy_analog_turn = norm_val

    def compute_navigation_inputs(self) -> Tuple[float, float]:
        if self.state.mode == "BALANCE":
            return 0.0, 0.0

        fb_axis = self._joy_dpad_fb if abs(self._joy_dpad_fb) > 0 else self._joy_analog_fb
        turn_axis = self._joy_dpad_turn if abs(self._joy_dpad_turn) > 0 else self._joy_analog_turn

        target_pitch = 0.0
        if fb_axis > 0:
            target_pitch = float(config.MANUAL_FORWARD_TARGET_DEG) * abs(fb_axis)
        elif fb_axis < 0:
            target_pitch = float(config.MANUAL_BACKWARD_TARGET_DEG) * abs(fb_axis)

        turn_cmd = 0.0
        if self.state.mode == "NUGGET":
            turn_cmd = max(-1.0, min(1.0, turn_axis))

        return target_pitch, turn_cmd

    def control_step(self) -> None:
        """Siklus eksekusi kontrol utama pada frekuensi 200 Hz."""
        # 1. Baca Sensor Telemetri
        angle_state = self.imu.read_angles()
        psi = self.imu.Angle_PSI()
        dot_psi = self.imu.Angular_dot_PSI()

        theta_state = self.motors.get_angle_theta()
        theta = theta_state.avg_theta_deg
        dot_theta = theta_state.avg_speed_rad_s

        self.state.latest_angle_state = angle_state
        self.state.latest_theta_state = theta_state

        if self.state.mode not in ("BALANCE", "MONO", "NUGGET"):
            return

        # 2. Hard Safety Tilt Safeguard
        if abs(psi) >= float(config.HARD_SAFE_TILT_DEG):
            self.set_fault(
                f"Sudut Melebihi Limit Aman: {psi:.2f}° >= {config.HARD_SAFE_TILT_DEG:.2f}°"
            )
            return

        # 3. Pengolahan Input Navigasi
        target_pitch, turn_cmd = self.compute_navigation_inputs()

        # 4. Kalkulasi PD Controller Sesuai Mode
        if self.state.mode == "BALANCE":
            pd_state = self.controller.compute_balance_mode(
                psi=psi, dot_psi=dot_psi, theta=theta, dot_theta=dot_theta
            )
        elif self.state.mode == "MONO":
            pd_state = self.controller.compute_mono_mode(
                psi=psi,
                dot_psi=dot_psi,
                theta=theta,
                dot_theta=dot_theta,
                target_pitch_deg=target_pitch,
            )
        else:  # NUGGET
            pd_state = self.controller.compute_nugget_mode(
                psi=psi,
                dot_psi=dot_psi,
                theta=theta,
                dot_theta=dot_theta,
                target_pitch_deg=target_pitch,
                turn_command=turn_cmd,
            )

        # 5. Kirim Perintah ke Aktuator DDSM115
        fb_dict = self.motors.drive_individual(
            left_value=pd_state.left_output, right_value=pd_state.right_output
        )

        # Update Runtime State
        self.state.target_pitch_deg = target_pitch
        self.state.turn_command = turn_cmd
        self.state.latest_pd_state = pd_state
        self.state.latest_motor_fb["left"] = fb_dict.get("left")
        self.state.latest_motor_fb["right"] = fb_dict.get("right")
        self._motors_stopped = False

    def render_single_page_gui(self) -> None:
        """Single-Page Dashboard UI Debugging Real-Time Terminal."""
        ang = self.state.latest_angle_state
        th = self.state.latest_theta_state
        pd = self.state.latest_pd_state
        fb_l = self.state.latest_motor_fb.get("left")
        fb_r = self.state.latest_motor_fb.get("right")

        armed_str = "[ ARMED ]" if self.state.is_armed else "[ DISARMED / STANDBY ]"

        lines = []
        lines.append("\x1b[2J\x1b[H")  # ANSI Clear Terminal & Home Cursor
        lines.append("=" * 80)
        lines.append(
            f"   ATERA TWSBR SYSTEM DASHBOARD | FREQ: {self.state.loop_hz_est:5.1f} Hz (Target 200Hz)"
        )
        lines.append("=" * 80)
        lines.append(
            f" SYSTEM MODE : {self.state.mode:<10} | ARMING: {armed_str:<20} | CALIB: {str(self.state.calibrated):<5}"
        )
        lines.append(
            f" CONTROL ALG : {self.state.control_algorithm:<10} | ZERO OFFSET: {self.state.zero_offset_deg:+6.2f}°"
        )
        lines.append(f" STATUS MSG  : {self.state.last_message}")
        if self.state.fault_reason:
            lines.append(f" FAULT CRIT  : {self.state.fault_reason}")
        lines.append("-" * 80)

        # Section 1: MPU6050 & IMU State
        lines.append(" [IMU MPU6050 TELEMETRY]")
        if ang:
            lines.append(
                f"  Angle (Psi)   : {ang.angle_deg:+7.3f}° | Gyro Rate Dot Psi : {ang.gyro_rate_y:+7.3f}°/s"
            )
            lines.append(
                f"  Kalman Pitch  : {ang.kalman_y:+7.3f}° | Accel Pitch      : {ang.acc_angle_y:+7.3f}°"
            )
            lines.append(
                f"  Zero Offset   : {self.state.zero_offset_deg:+7.3f}° | dt               : {ang.dt*1000.2f} ms"
            )
        else:
            lines.append("  [!] IMU Data Disconnected / Loading...")
        lines.append("-" * 80)

        # Section 2: DDSM115 Wheel Encoders & Motor Feedback
        lines.append(" [DDSM115 WHEEL ENCODERS & MOTORS TELEMETRY]")
        if th and fb_l and fb_r:
            lines.append(
                f"  Theta Avg     : {th.avg_theta_deg:+7.2f}° | Speed Avg        : {th.avg_speed_rpm:+6.1f} RPM ({th.avg_speed_rad_s:+5.2f} rad/s)"
            )
            lines.append(
                f"  Left Motor    : {fb_l.torque_ampere:+5.2f}A | {fb_l.speed_rpm:+4d} RPM | Pos: {fb_l.position_deg:6.1f}° | Err: 0x{fb_l.error_code:02X}"
            )
            lines.append(
                f"  Right Motor   : {fb_r.torque_ampere:+5.2f}A | {fb_r.speed_rpm:+4d} RPM | Pos: {fb_r.position_deg:6.1f}° | Err: 0x{fb_r.error_code:02X}"
            )
        else:
            lines.append("  [!] Motor Data Disconnected / Idle")
        lines.append("-" * 80)

        # Section 3: Controller State & Outputs
        lines.append(" [PD DUAL-LOOP CONTROLLER STATS]")
        if pd:
            lines.append(
                f"  Setpt Pitch   : {pd.setpoint_psi:+7.2f}° | Setpt Theta      : {pd.setpoint_theta:+7.2f}°"
            )
            lines.append(
                f"  Err Psi/Dot   : {pd.error_psi:+7.3f} / {pd.error_dot_psi:+7.3f} | Err Theta/Dot    : {pd.error_theta:+7.3f} / {pd.error_dot_theta:+7.3f}"
            )
            lines.append(
                f"  uPD Nominal   : {pd.u_pd:+7.3f}  | Left/Right Output: {pd.left_output:+6.3f} / {pd.right_output:+6.3f}"
            )
        else:
            lines.append("  [!] Controller Inactive")
        lines.append("-" * 80)

        # Section 4: Joystick Guide & Quick Keys
        lines.append(" [JOYSTICK MAPPING GUIDE]")
        lines.append(
            "  Y: Kalibrasi   | MULAI: Master Arming/Disarm | B: Mode Balance (Aktif saat Armed)"
        )
        lines.append(
            "  Combo A+LT: Mode Mono | Combo A+RT: Mode Nugget | LB/RB: PD / PD+CBF-QP"
        )
        lines.append(
            "  X: Stop Wheel  | QUIT: Keluar Sistem"
        )
        lines.append("=" * 80)

        sys.stdout.write("\n".join(lines) + "\n")
        sys.stdout.flush()

    def run(self) -> None:
        control_period = 1.0 / float(config.CONTROL_HZ)
        ui_period = 1.0 / float(config.UI_HZ)
        self._running = True
        next_tick = time.monotonic()

        with JoystickHandler(config.JOYSTICK_PATH) as joy:
            while self._running and not self.state.exit_requested:
                loop_start = time.monotonic()

                # 1. Baca Joystick Event
                events = joy.read_events()
                self.process_joystick_events(events)

                # 2. Eksekusi Loop Kendali Utama
                try:
                    self.control_step()
                except (OSError, RuntimeError, DDSM115Error, ValueError) as exc:
                    self.set_fault(str(exc))
                except Exception as exc:
                    self.set_fault(f"Unhandled Runtime Error: {exc}")

                now = time.monotonic()
                self.state.loop_dt = now - loop_start
                self.state.loop_hz_est = (
                    1.0 / self.state.loop_dt if self.state.loop_dt > 0 else 0.0
                )

                # 3. Render GUI Terminal (20 Hz)
                if (now - self.state.last_ui_render_ts) >= ui_period:
                    self.render_single_page_gui()
                    self.state.last_ui_render_ts = now

                # 4. Frekuensi Loop Precision Timing (200 Hz)
                next_tick += control_period
                sleep_time = next_tick - time.monotonic()
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    next_tick = time.monotonic()

        self.cleanup()


def main() -> int:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
    )
    app = AteraMainApp()
    try:
        app.setup()
        app.run()
        return 0
    except KeyboardInterrupt:
        LOGGER.info("KeyboardInterrupt diterima. Mematikan robot dengan aman.")
        app.cleanup()
        return 0
    except Exception as exc:
        LOGGER.exception("Fatal error di atera_main: %s", exc)
        try:
            app.cleanup()
        except Exception:
            pass
        return 1


if __name__ == "__main__":
    raise SystemExit(main())