from __future__ import annotations

import math
import sys
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Optional

import config
from ddsm115 import DualDDSM115
from mpu6050 import MPU6050Reader
from pd_control import PDController, PDControlState

try:
    import keyboard as keyboard_lib  # type: ignore
except Exception:
    keyboard_lib = None

try:
    import msvcrt  # type: ignore
except Exception:
    msvcrt = None

try:
    from cbf_qp import CBFQPController  # type: ignore
except Exception:
    CBFQPController = None


class ActiveMode(str, Enum):
    PD_ONLY = "PD_ONLY"
    PD_PLUS_CBF_QP = "PD_PLUS_CBF_QP"


class RobotMode(str, Enum):
    BALANCE = "BALANCE"
    KALIBRASI = "KALIBRASI"
    MONO = "MONO"
    NUGGET = "NUGGET"


@dataclass
class MotionCommand:
    forward: int = 0
    turn: int = 0
    active: bool = False
    last_update: float = 0.0


@dataclass
class RuntimeState:
    active_mode: ActiveMode = ActiveMode.PD_ONLY
    robot_mode: RobotMode = RobotMode.BALANCE
    running: bool = True
    theta_deg: float = 0.0
    theta_dot_deg_s: float = 0.0
    psi_deg: float = 0.0
    psi_dot_deg_s: float = 0.0
    left_motor: float = 0.0
    right_motor: float = 0.0
    total_motor: float = 0.0
    cbf_status: str = "INACTIVE"
    control_path: str = "PD_ONLY"
    setpoint_psi: float = 0.0
    setpoint_theta: float = 0.0
    turn_command: float = 0.0
    last_error: str = ""


class KeyboardManager:
    def __init__(self, motion: MotionCommand, runtime: RuntimeState) -> None:
        self.motion = motion
        self.runtime = runtime
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._supports_keyboard = keyboard_lib is not None
        self._supports_msvcrt = (msvcrt is not None) and sys.platform.startswith("win")

    def start(self) -> None:
        if self._supports_keyboard:
            self._thread = threading.Thread(target=self._keyboard_loop, daemon=True)
            self._thread.start()
        elif self._supports_msvcrt:
            self._thread = threading.Thread(target=self._msvcrt_loop, daemon=True)
            self._thread.start()
        else:
            raise RuntimeError("keyboard library tidak tersedia dan fallback msvcrt tidak bisa dipakai di platform ini")

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=0.5)

    def _handle_key(self, key: str) -> None:
        now = time.time()
        key = key.lower()
        with self._lock:
            if key == "1":
                self.runtime.active_mode = ActiveMode.PD_ONLY
            elif key == "2":
                self.runtime.active_mode = ActiveMode.PD_PLUS_CBF_QP
            elif key == "b":
                self.runtime.robot_mode = RobotMode.BALANCE
            elif key == "k":
                self.runtime.robot_mode = RobotMode.KALIBRASI
            elif key == "m":
                self.runtime.robot_mode = RobotMode.MONO
            elif key == "n":
                self.runtime.robot_mode = RobotMode.NUGGET
            elif key == "w":
                self.motion.forward = 1
                self.motion.active = True
                self.motion.last_update = now
            elif key == "s":
                self.motion.forward = -1
                self.motion.active = True
                self.motion.last_update = now
            elif key == "a":
                self.motion.turn = -1
                self.motion.active = True
                self.motion.last_update = now
            elif key == "d":
                self.motion.turn = 1
                self.motion.active = True
                self.motion.last_update = now
            elif key in ("x", "z"):
                self.motion.forward = 0
                self.motion.turn = 0
                self.motion.active = False
                self.motion.last_update = now
            elif key == "r":
                self.motion.forward = 0
                self.motion.turn = 0
                self.motion.active = False
                self.motion.last_update = now
                self.runtime.setpoint_psi = 0.0
                self.runtime.setpoint_theta = 0.0
                self.runtime.turn_command = 0.0

    def _keyboard_loop(self) -> None:
        assert keyboard_lib is not None
        watched = ["1", "2", "b", "k", "m", "n", "w", "a", "s", "d", "x", "z", "r"]
        while not self._stop_event.is_set():
            handled = False
            for key in watched:
                try:
                    if keyboard_lib.is_pressed(key):
                        self._handle_key(key)
                        handled = True
                except Exception:
                    pass
            if not handled:
                time.sleep(0.01)
            else:
                time.sleep(0.02)

    def _msvcrt_loop(self) -> None:
        assert msvcrt is not None
        while not self._stop_event.is_set():
            if msvcrt.kbhit():
                try:
                    raw = msvcrt.getwch()
                except Exception:
                    raw = ""
                if raw:
                    self._handle_key(raw)
            else:
                time.sleep(0.01)


class AteraMainController:
    def __init__(self) -> None:
        self.runtime = RuntimeState()
        self.motion = MotionCommand()
        self.imu = MPU6050Reader()
        self.motors = DualDDSM115()
        self.pd = PDController()
        self.cbf = CBFQPController() if CBFQPController is not None else None
        self.keyboard = KeyboardManager(self.motion, self.runtime)
        self.prev_time: Optional[float] = None
        self.prev_theta: Optional[float] = None
        self.prev_psi: Optional[float] = None
        self.last_ui = 0.0
        self._last_positions_deg = {"left": 0.0, "right": 0.0}

    def setup(self) -> None:
        self.motors.open()
        self.motors.initialize()
        self.imu.ensure_open()

    def safe_stop(self) -> None:
        try:
            self.motors.stop_all()
        except Exception as exc:
            self.runtime.last_error = str(exc)

    def close(self) -> None:
        try:
            self.safe_stop()
        finally:
            try:
                self.motors.close()
            finally:
                try:
                    self.keyboard.stop()
                except Exception:
                    pass

    def apply_key_timeout(self, now: float) -> None:
        if self.motion.active and (now - self.motion.last_update) > float(config.KEY_HOLD_TIMEOUT_S):
            self.motion.forward = 0
            self.motion.turn = 0
            self.motion.active = False

    def read_theta(self, motor_feedback: Optional[dict]) -> float:
        if motor_feedback:
            left_deg = float(motor_feedback["left"].position_deg)
            right_deg = float(motor_feedback["right"].position_deg)
            self._last_positions_deg["left"] = left_deg
            self._last_positions_deg["right"] = right_deg
        return (self._last_positions_deg["left"] + self._last_positions_deg["right"]) * 0.5

    def compute_rates(self, theta_deg: float, psi_deg: float, now: float) -> tuple[float, float, float]:
        if self.prev_time is None:
            self.prev_time = now
            self.prev_theta = theta_deg
            self.prev_psi = psi_deg
            return 0.0, 0.0, 1.0 / float(config.CONTROL_HZ)

        dt = max(1e-4, now - self.prev_time)
        theta_dot = (theta_deg - float(self.prev_theta)) / dt
        psi_dot = (psi_deg - float(self.prev_psi)) / dt
        self.prev_time = now
        self.prev_theta = theta_deg
        self.prev_psi = psi_deg
        return theta_dot, psi_dot, dt

    def resolve_setpoints(self) -> tuple[float, float, float]:
        setpoint_psi = 0.0
        setpoint_theta = 0.0
        turn_command = 0.0

        if self.runtime.robot_mode == RobotMode.BALANCE:
            if self.motion.forward > 0:
                setpoint_psi = float(config.MANUAL_FORWARD_TARGET_DEG)
            elif self.motion.forward < 0:
                setpoint_psi = float(config.MANUAL_BACKWARD_TARGET_DEG)
            if self.motion.turn != 0:
                turn_command = float(self.motion.turn)

        elif self.runtime.robot_mode == RobotMode.KALIBRASI:
            setpoint_psi = 0.0
            setpoint_theta = 0.0
            turn_command = 0.0

        elif self.runtime.robot_mode == RobotMode.MONO:
            if self.motion.forward > 0:
                setpoint_psi = float(config.MANUAL_FORWARD_TARGET_DEG)
            elif self.motion.forward < 0:
                setpoint_psi = float(config.MANUAL_BACKWARD_TARGET_DEG)
            turn_command = 0.0

        elif self.runtime.robot_mode == RobotMode.NUGGET:
            setpoint_psi = 0.0
            setpoint_theta = 0.0
            turn_command = float(self.motion.turn)

        return setpoint_psi, setpoint_theta, turn_command

    def apply_safety(self, psi_deg: float) -> bool:
        if abs(psi_deg) >= float(config.HARD_SAFE_TILT_DEG):
            self.runtime.cbf_status = "HARD_SAFE_TILT"
            self.runtime.control_path = "SAFETY_STOP"
            self.runtime.left_motor = 0.0
            self.runtime.right_motor = 0.0
            self.runtime.total_motor = 0.0
            self.safe_stop()
            return False
        return True

    def run_pd(self, psi: float, psi_dot: float, theta: float, theta_dot: float, setpoint_psi: float, setpoint_theta: float, turn_command: float) -> PDControlState:
        return self.pd.PD_compute(
            psi=psi,
            dot_psi=psi_dot,
            theta=theta,
            dot_theta=theta_dot,
            setpoint_psi=setpoint_psi,
            setpoint_theta=setpoint_theta,
            turn_command=turn_command,
        )

    def maybe_apply_cbf(self, pd_state: PDControlState, psi: float, psi_dot: float) -> tuple[float, float, str, str]:
        if self.runtime.active_mode != ActiveMode.PD_PLUS_CBF_QP:
            return pd_state.left_output, pd_state.right_output, "INACTIVE", "PD_ONLY"

        if self.cbf is None:
            return pd_state.left_output, pd_state.right_output, "UNAVAILABLE", "PD_FALLBACK"

        try:
            result = self.cbf.filter(
                nominal_left=float(pd_state.left_output),
                nominal_right=float(pd_state.right_output),
                psi=float(psi),
                psi_dot=float(psi_dot),
                alpha_1=float(config.ALPHA_1),
                alpha_2=float(config.ALPHA_2),
            )
            left = float(result.get("left", pd_state.left_output))
            right = float(result.get("right", pd_state.right_output))
            return left, right, "ACTIVE", "PD_PLUS_CBF_QP"
        except Exception as exc:
            self.runtime.last_error = str(exc)
            return pd_state.left_output, pd_state.right_output, "ERROR", "PD_FALLBACK"

    def print_status(self) -> None:
        line = (
            f"mode={self.runtime.active_mode.value} | "
            f"robot={self.runtime.robot_mode.value} | "
            f"theta={self.runtime.theta_deg:+7.3f} deg | "
            f"theta_dot={self.runtime.theta_dot_deg_s:+8.3f} deg/s | "
            f"psi={self.runtime.psi_deg:+7.3f} deg | "
            f"psi_dot={self.runtime.psi_dot_deg_s:+8.3f} deg/s | "
            f"left={self.runtime.left_motor:+6.3f} | "
            f"right={self.runtime.right_motor:+6.3f} | "
            f"total={self.runtime.total_motor:+6.3f} | "
            f"cbf={self.runtime.cbf_status} | "
            f"path={self.runtime.control_path}"
        )
        if self.runtime.last_error:
            line += f" | err={self.runtime.last_error}"
        print(line)

    def loop(self) -> None:
        control_period = 1.0 / float(config.CONTROL_HZ)
        ui_period = 1.0 / float(config.UI_HZ)

        self.keyboard.start()
        motor_feedback = None

        while self.runtime.running:
            start = time.time()
            self.apply_key_timeout(start)

            imu_state = self.imu.read_angles()
            psi_deg = float(imu_state.angle_deg)

            if not self.apply_safety(psi_deg):
                time.sleep(control_period)
                continue

            if motor_feedback is None:
                try:
                    motor_feedback = self.motors.query_both()
                except Exception:
                    motor_feedback = None

            theta_deg = self.read_theta(motor_feedback)
            theta_dot, psi_dot, _dt = self.compute_rates(theta_deg, psi_deg, start)
            setpoint_psi, setpoint_theta, turn_command = self.resolve_setpoints()

            pd_state = self.run_pd(
                psi=psi_deg,
                psi_dot=psi_dot,
                theta=theta_deg,
                theta_dot=theta_dot,
                setpoint_psi=setpoint_psi,
                setpoint_theta=setpoint_theta,
                turn_command=turn_command,
            )

            left_cmd, right_cmd, cbf_status, control_path = self.maybe_apply_cbf(pd_state, psi_deg, psi_dot)

            motor_feedback = self.motors.command_normalized(left_cmd, right_cmd)

            self.runtime.theta_deg = theta_deg
            self.runtime.theta_dot_deg_s = theta_dot
            self.runtime.psi_deg = psi_deg
            self.runtime.psi_dot_deg_s = psi_dot
            self.runtime.left_motor = left_cmd
            self.runtime.right_motor = right_cmd
            self.runtime.total_motor = 0.5 * (left_cmd + right_cmd)
            self.runtime.cbf_status = cbf_status
            self.runtime.control_path = control_path
            self.runtime.setpoint_psi = setpoint_psi
            self.runtime.setpoint_theta = setpoint_theta
            self.runtime.turn_command = turn_command

            now = time.time()
            if (now - self.last_ui) >= ui_period:
                self.print_status()
                self.last_ui = now

            elapsed = time.time() - start
            sleep_time = max(0.0, control_period - elapsed)
            time.sleep(sleep_time)

    def run(self) -> None:
        try:
            self.setup()
            self.loop()
        except KeyboardInterrupt:
            self.runtime.running = False
        finally:
            self.close()


def main() -> None:
    app = AteraMainController()
    app.run()


if __name__ == "__main__":
    main()
