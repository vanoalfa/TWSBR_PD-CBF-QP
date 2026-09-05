"""Program utama self-balancing robot ATERA.

Fitur utama:
- Integrasi MPU6050 + Kalman filter + PD controller + dua motor DDSM115
- State machine sederhana: IDLE, CALIBRATING, READY, BALANCING, FAULT, EXITING
- Keyboard non-blocking di terminal
- UI terminal sederhana dengan beberapa halaman (TAB untuk pindah)
- Shutdown aman: motor dihentikan saat fault, tilt berlebih, atau quit

Kontrol keyboard:
- K : kalibrasi gyro + zero angle
- M : start/stop balancing
- W : maju (memberi target angle kecil ke depan)
- S : mundur
- A : belok kiri
- D : belok kanan
- TAB : ganti halaman UI
- SPACE/X : stop balancing
- Q : quit aman
"""

from __future__ import annotations

import logging
import select
import sys
import termios
import time
import tty
from dataclasses import dataclass, field
from typing import Dict, Optional

import tunning
from ddsm115 import DDSM115Error, DualDDSM115, MotorFeedback
from mpu6050 import AngleState, MPU6050Reader
from pd_control import BalancePDController, PDControlState

LOGGER = logging.getLogger("atera")
PAGES = ("status", "motor", "help")


@dataclass
class RuntimeState:
    mode: str = "IDLE"
    calibrated: bool = False
    exit_requested: bool = False
    page_index: int = 0
    last_message: str = "Tekan K untuk kalibrasi, lalu M untuk mulai balancing."
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


class TerminalKeyboard:
    """Keyboard non-blocking untuk terminal Linux/Raspberry Pi."""

    def __init__(self) -> None:
        self.fd: Optional[int] = None
        self.old_settings = None
        self.enabled = False

    def __enter__(self) -> "TerminalKeyboard":
        if not sys.stdin.isatty():
            raise RuntimeError("stdin bukan TTY; jalankan script ini langsung dari terminal.")
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        self.enabled = True
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.restore()

    def restore(self) -> None:
        if self.enabled and self.fd is not None and self.old_settings is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
        self.enabled = False

    def read_keys(self) -> list[str]:
        if not self.enabled or self.fd is None:
            return []
        keys: list[str] = []
        while True:
            ready, _, _ = select.select([sys.stdin], [], [], 0)
            if not ready:
                break
            ch = sys.stdin.read(1)
            if not ch:
                break
            keys.append(ch)
        return keys


class AteraMainApp:
    def __init__(self) -> None:
        self.motors = DualDDSM115()
        self.imu = MPU6050Reader()
        self.controller = BalancePDController()
        self.state = RuntimeState()
        self._running = False
        self._motors_stopped = False
        self._last_ui_ts = 0.0
        self._last_key_ts = {
            "w": 0.0,
            "s": 0.0,
            "a": 0.0,
            "d": 0.0,
        }

    def setup(self) -> None:
        self.motors.open()
        self.motors.initialize()
        self.imu.open()
        self._motors_stopped = True
        self.state.mode = "IDLE"
        self.state.last_message = "Hardware siap. Tekan K untuk kalibrasi."
        LOGGER.info("ATERA main setup complete")

    def cleanup(self) -> None:
        self.state.mode = "EXITING"
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
        self.set_message(f"FAULT: {reason}")

    def stop_balancing(self, reason: str = "Balancing dihentikan.") -> None:
        try:
            self.ensure_motors_stopped("stop balancing")
        finally:
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
            self.set_message("Belum dikalibrasi. Tekan K dulu.")
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

    def toggle_balancing(self) -> None:
        if self.state.mode == "BALANCING":
            self.stop_balancing("Balancing dihentikan oleh user.")
        else:
            self.start_balancing()

    def process_key(self, ch: str) -> None:
        key = ch.lower()
        now = time.monotonic()

        if ch == "\t":
            self.state.page_index = (self.state.page_index + 1) % len(PAGES)
            return

        if key == "q":
            self.state.exit_requested = True
            self.set_message("Quit diminta user.")
            return

        if key == "k":
            self.calibrate()
            return

        if key == "m":
            self.toggle_balancing()
            return

        if key in ("x", " "):
            self.stop_balancing("Stop manual.")
            return

        if key in self._last_key_ts:
            self._last_key_ts[key] = now

    def get_manual_commands(self, now_ts: float) -> tuple[float, float]:
        timeout = float(tunning.KEY_HOLD_TIMEOUT_S)
        forward = (now_ts - self._last_key_ts["w"]) <= timeout
        backward = (now_ts - self._last_key_ts["s"]) <= timeout
        left = (now_ts - self._last_key_ts["a"]) <= timeout
        right = (now_ts - self._last_key_ts["d"]) <= timeout

        target_angle = 0.0
        if forward and not backward:
            target_angle = float(tunning.MANUAL_FORWARD_TARGET_DEG)
        elif backward and not forward:
            target_angle = float(tunning.MANUAL_BACKWARD_TARGET_DEG)

        turn_command = 0.0
        if left and not right:
            turn_command = -1.0
        elif right and not left:
            turn_command = 1.0

        return target_angle, turn_command

    def get_angular_rate_from_state(self, angle_state: AngleState) -> float:
        if angle_state.axis_used == "roll":
            return angle_state.gyro_rate_x
        return angle_state.gyro_rate_y

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

        now = time.monotonic()
        target_angle, turn_command = self.get_manual_commands(now)
        angular_rate = self.get_angular_rate_from_state(angle_state)

        pd_state = self.controller.compute(
            angle_deg=angle_state.angle_deg,
            angular_rate_deg_s=angular_rate,
            target_angle_deg=target_angle,
            turn_command=turn_command,
        )
        feedback = self.motors.command_normalized(pd_state.left_output, pd_state.right_output)

        self.state.target_angle_deg = target_angle
        self.state.turn_command = turn_command
        self.state.latest_pd = pd_state
        self.state.latest_motor_fb["left"] = feedback.get("left")
        self.state.latest_motor_fb["right"] = feedback.get("right")
        self._motors_stopped = False

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
        lines.append("ATERA SELF-BALANCING ROBOT")
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
                lines.append(f"Base output         : {pd.base_output:+8.3f}")
                lines.append(f"Left / Right output : {pd.left_output:+8.3f} / {pd.right_output:+8.3f}")

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
            lines.append("BANTUAN KONTROL")
            lines.append("  K      : kalibrasi gyro + zero angle")
            lines.append("  M      : start / stop balancing")
            lines.append("  W / S  : maju / mundur (hold via timeout key)")
            lines.append("  A / D  : belok kiri / kanan")
            lines.append("  SPACE  : stop balancing")
            lines.append("  TAB    : pindah halaman")
            lines.append("  Q      : quit aman")
            lines.append("")
            lines.append("CATATAN")
            lines.append("  - Saat kalibrasi, robot harus diam.")
            lines.append("  - Jika robot malah jatuh saat balancing mulai aktif,")
            lines.append("    cek BALANCE_DIRECTION_SIGN di tunning.py.")
            lines.append("  - Jika arah roda terbalik, cek LEFT_MOTOR_SIGN / RIGHT_MOTOR_SIGN.")
            lines.append("  - Jika steering terbalik, tukar makna A/D atau ubah sign turn di logika.")

        lines.append("-" * 72)
        lines.append("Keys: K calibrate | M balance | W/A/S/D move | TAB page | Q quit")

        sys.stdout.write("\n".join(lines) + "\n")
        sys.stdout.flush()

    def run(self) -> None:
        control_period = 1.0 / float(tunning.CONTROL_HZ)
        ui_period = 1.0 / float(tunning.UI_HZ)
        self._running = True
        next_tick = time.monotonic()

        with TerminalKeyboard() as keyboard:
            while self._running and not self.state.exit_requested:
                loop_start = time.monotonic()

                for ch in keyboard.read_keys():
                    self.process_key(ch)

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