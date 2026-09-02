from __future__ import annotations

import importlib
import logging
import math
import queue
import select
import signal
import sys
import termios
import threading
import time
import tty
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Optional, Protocol

import config
from ddsm115 import DDSM115Error, DualDDSM115, MotorFeedback
from mpu6050 import AngleState, MPU6050Reader
from pd_control import PDControlState, PDController

LOGGER = logging.getLogger(__name__)


class RobotMode(str, Enum):
    IDLE = "idle"
    CALIBRATION = "calibration"
    BALANCE = "balance"
    MONO = "mono"
    NUGGET = "nugget"
    QUIT = "quit"


class DriveIntent(str, Enum):
    STOP = "stop"
    FORWARD = "forward"
    BACKWARD = "backward"


class SteerIntent(str, Enum):
    STRAIGHT = "straight"
    LEFT = "left"
    RIGHT = "right"


@dataclass
class ControlTargets:
    setpoint_psi: float = 0.0
    setpoint_theta: float = 0.0
    turn_command: float = 0.0


@dataclass
class WheelState:
    theta_left: float = 0.0
    theta_right: float = 0.0
    dot_theta_left: float = 0.0
    dot_theta_right: float = 0.0
    left_feedback: Optional[MotorFeedback] = None
    right_feedback: Optional[MotorFeedback] = None

    @property
    def theta_avg(self) -> float:
        return 0.5 * (self.theta_left + self.theta_right)

    @property
    def dot_theta_avg(self) -> float:
        return 0.5 * (self.dot_theta_left + self.dot_theta_right)


@dataclass
class RobotSnapshot:
    timestamp: float
    dt: float
    mode: RobotMode
    imu_state: AngleState
    imu_rate_deg_s: float
    wheel: WheelState
    targets: ControlTargets
    drive_intent: DriveIntent
    steer_intent: SteerIntent
    controller_state: Optional[PDControlState] = None


@dataclass
class ActuationCommand:
    left: float
    right: float
    source: str
    u_pd_nominal: float
    metadata: dict[str, Any] = field(default_factory=dict)


@dataclass
class CBFQPInput:
    u_PD: float
    snapshot: RobotSnapshot
    pd_state: PDControlState
    state_vector: dict[str, float]
    target_vector: dict[str, float]
    wheel_vector: dict[str, float]


class ControlBackend(Protocol):
    def compute(self, snapshot: RobotSnapshot) -> PDControlState:
        ...


class CommandFilter(Protocol):
    def compute(self, snapshot: RobotSnapshot, pd_state: PDControlState) -> ActuationCommand:
        ...


class PDBackend:
    def __init__(self) -> None:
        self.controller = PDController()

    def compute(self, snapshot: RobotSnapshot) -> PDControlState:
        return self.controller.PD_compute(
            psi=snapshot.imu_state.angle_deg,
            dot_psi=snapshot.imu_rate_deg_s,
            theta=snapshot.wheel.theta_avg,
            dot_theta=snapshot.wheel.dot_theta_avg,
            setpoint_psi=snapshot.targets.setpoint_psi,
            setpoint_theta=snapshot.targets.setpoint_theta,
            turn_command=snapshot.targets.turn_command,
        )


class PDOutputFilter:
    def compute(self, snapshot: RobotSnapshot, pd_state: PDControlState) -> ActuationCommand:
        return ActuationCommand(
            left=float(pd_state.left_output),
            right=float(pd_state.right_output),
            source="pd",
            u_pd_nominal=float(pd_state.u_PD),
            metadata={
                "setpoint_imu": float(pd_state.setpoint_IMU),
                "setpoint_ddsm": float(pd_state.setpoint_DDSM),
            },
        )


class OptionalCBFQPFilter:
    def __init__(self) -> None:
        self.enabled = bool(getattr(config, "USE_CBF_QP", False))
        self._impl: Any = None
        self._impl_name = "pd_fallback"
        if self.enabled:
            self._load_impl()

    def _load_impl(self) -> None:
        try:
            module = importlib.import_module("cbf_qp")
        except ModuleNotFoundError:
            LOGGER.warning("USE_CBF_QP aktif, tetapi cbf_qp.py belum tersedia. Fallback ke PD.")
            self.enabled = False
            return

        if hasattr(module, "build_filter"):
            self._impl = module.build_filter()
            self._impl_name = "cbf_qp.build_filter"
        elif hasattr(module, "CBFQPFilter"):
            self._impl = module.CBFQPFilter()
            self._impl_name = "cbf_qp.CBFQPFilter"
        elif hasattr(module, "CBFQPController"):
            self._impl = module.CBFQPController()
            self._impl_name = "cbf_qp.CBFQPController"
        elif hasattr(module, "filter_control"):
            self._impl = module.filter_control
            self._impl_name = "cbf_qp.filter_control"
        elif hasattr(module, "filter"):
            self._impl = module.filter
            self._impl_name = "cbf_qp.filter"
        else:
            LOGGER.warning(
                "cbf_qp.py ditemukan, tetapi tidak ada entry point yang dikenali. "
                "Gunakan build_filter, CBFQPFilter, CBFQPController, filter_control, atau filter."
            )
            self.enabled = False
            return

        LOGGER.info("CBF-QP hook aktif via %s", self._impl_name)

    @staticmethod
    def _build_qp_input(snapshot: RobotSnapshot, pd_state: PDControlState) -> CBFQPInput:
        return CBFQPInput(
            u_PD=float(pd_state.u_PD),
            snapshot=snapshot,
            pd_state=pd_state,
            state_vector={
                "psi": float(snapshot.imu_state.angle_deg),
                "dot_psi": float(snapshot.imu_rate_deg_s),
                "theta": float(snapshot.wheel.theta_avg),
                "dot_theta": float(snapshot.wheel.dot_theta_avg),
                "theta_left": float(snapshot.wheel.theta_left),
                "theta_right": float(snapshot.wheel.theta_right),
                "dot_theta_left": float(snapshot.wheel.dot_theta_left),
                "dot_theta_right": float(snapshot.wheel.dot_theta_right),
                "dt": float(snapshot.dt),
            },
            target_vector={
                "setpoint_psi": float(snapshot.targets.setpoint_psi),
                "setpoint_theta": float(snapshot.targets.setpoint_theta),
                "turn_command": float(snapshot.targets.turn_command),
            },
            wheel_vector={
                "left_output_pd": float(pd_state.left_output),
                "right_output_pd": float(pd_state.right_output),
                "u_PD": float(pd_state.u_PD),
            },
        )

    def _invoke_impl(self, qp_input: CBFQPInput) -> Any:
        impl = self._impl
        if impl is None:
            raise RuntimeError("CBF-QP implementation belum dimuat")

        if hasattr(impl, "filter"):
            method = impl.filter
            try:
                return method(qp_input=qp_input)
            except TypeError:
                try:
                    return method(u_PD=qp_input.u_PD, snapshot=qp_input.snapshot, pd_state=qp_input.pd_state)
                except TypeError:
                    return method(qp_input.u_PD, qp_input.snapshot, qp_input.pd_state)

        if hasattr(impl, "compute"):
            method = impl.compute
            try:
                return method(qp_input=qp_input)
            except TypeError:
                try:
                    return method(u_PD=qp_input.u_PD, snapshot=qp_input.snapshot, pd_state=qp_input.pd_state)
                except TypeError:
                    return method(qp_input.u_PD, qp_input.snapshot, qp_input.pd_state)

        if callable(impl):
            try:
                return impl(qp_input=qp_input)
            except TypeError:
                try:
                    return impl(u_PD=qp_input.u_PD, snapshot=qp_input.snapshot, pd_state=qp_input.pd_state)
                except TypeError:
                    return impl(qp_input.u_PD, qp_input.snapshot, qp_input.pd_state)

        raise RuntimeError("CBF-QP entry point tidak callable")

    @staticmethod
    def _normalize_result(result: Any, pd_state: PDControlState) -> ActuationCommand:
        if isinstance(result, ActuationCommand):
            return result

        if isinstance(result, (int, float)):
            value = float(result)
            return ActuationCommand(
                left=value,
                right=value,
                source="cbf_qp_scalar",
                u_pd_nominal=float(pd_state.u_PD),
            )

        if isinstance(result, (tuple, list)) and len(result) == 2:
            return ActuationCommand(
                left=float(result[0]),
                right=float(result[1]),
                source="cbf_qp_pair",
                u_pd_nominal=float(pd_state.u_PD),
            )

        if isinstance(result, dict):
            left = result.get("left", result.get("left_output"))
            right = result.get("right", result.get("right_output"))
            if left is not None and right is not None:
                return ActuationCommand(
                    left=float(left),
                    right=float(right),
                    source=str(result.get("source", "cbf_qp_dict")),
                    u_pd_nominal=float(pd_state.u_PD),
                    metadata={k: v for k, v in result.items() if k not in {"left", "right", "left_output", "right_output"}},
                )

            u_safe = result.get("u_safe", result.get("u"))
            if u_safe is not None:
                value = float(u_safe)
                return ActuationCommand(
                    left=value,
                    right=value,
                    source=str(result.get("source", "cbf_qp_symmetric")),
                    u_pd_nominal=float(pd_state.u_PD),
                    metadata={k: v for k, v in result.items() if k not in {"u_safe", "u"}},
                )

        raise RuntimeError("Output CBF-QP tidak dikenali")

    def compute(self, snapshot: RobotSnapshot, pd_state: PDControlState) -> ActuationCommand:
        fallback = ActuationCommand(
            left=float(pd_state.left_output),
            right=float(pd_state.right_output),
            source="pd_fallback",
            u_pd_nominal=float(pd_state.u_PD),
        )
        if not self.enabled:
            return fallback

        try:
            qp_input = self._build_qp_input(snapshot, pd_state)
            result = self._invoke_impl(qp_input)
            command = self._normalize_result(result, pd_state)
            if "u_pd_input" not in command.metadata:
                command.metadata["u_pd_input"] = float(pd_state.u_PD)
            command.metadata.setdefault("cbf_qp_entry", self._impl_name)
            return command
        except Exception as exc:
            LOGGER.exception("CBF-QP gagal, fallback ke PD: %s", exc)
            return fallback


class WheelOdometry:
    def __init__(self, sign: float) -> None:
        self.sign = 1.0 if sign >= 0 else -1.0
        self._last_wrapped_deg: Optional[float] = None
        self._continuous_deg = 0.0

    def reset(self, feedback: Optional[MotorFeedback]) -> None:
        if feedback is None:
            self._last_wrapped_deg = None
            self._continuous_deg = 0.0
            return
        wrapped = float(feedback.position_deg) * self.sign
        self._last_wrapped_deg = wrapped
        self._continuous_deg = 0.0

    def update(self, feedback: Optional[MotorFeedback]) -> tuple[float, float]:
        if feedback is None:
            return self._continuous_deg, 0.0

        wrapped = float(feedback.position_deg) * self.sign
        if self._last_wrapped_deg is None:
            self._last_wrapped_deg = wrapped
            self._continuous_deg = 0.0
        else:
            delta = wrapped - self._last_wrapped_deg
            if delta > 180.0:
                delta -= 360.0
            elif delta < -180.0:
                delta += 360.0
            self._continuous_deg += delta
            self._last_wrapped_deg = wrapped

        dot_theta_deg_s = float(feedback.speed_rpm) * 6.0 * self.sign
        return self._continuous_deg, dot_theta_deg_s


class KeyboardThread(threading.Thread):
    def __init__(self, event_queue: queue.Queue[str]) -> None:
        super().__init__(daemon=True, name="keyboard-thread")
        self.event_queue = event_queue
        self._running = True

    def stop(self) -> None:
        self._running = False

    def run(self) -> None:
        if not sys.stdin.isatty():
            LOGGER.warning("stdin bukan TTY. Keyboard thread dinonaktifkan.")
            return

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while self._running:
                ready, _, _ = select.select([sys.stdin], [], [], 0.05)
                if not ready:
                    continue
                ch = sys.stdin.read(1)
                if not ch:
                    continue
                self.event_queue.put(ch.lower())
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


class AteraRobotApp:
    def __init__(self) -> None:
        self.running = True
        self.mode = RobotMode.IDLE
        self.previous_mode = RobotMode.IDLE
        self.drive_intent = DriveIntent.STOP
        self.steer_intent = SteerIntent.STRAIGHT
        self.targets = ControlTargets(
            setpoint_psi=float(getattr(config, "SETPOINT_IMU", 0.0)),
            setpoint_theta=float(getattr(config, "SETPOINT_MOTOR", 0.0)),
            turn_command=0.0,
        )
        self.event_queue: queue.Queue[str] = queue.Queue()
        self.keyboard = KeyboardThread(self.event_queue)
        self.imu = MPU6050Reader()
        self.motors = DualDDSM115()
        self.control_backend: ControlBackend = PDBackend()
        self.command_filter: CommandFilter = OptionalCBFQPFilter()
        self.left_odometry = WheelOdometry(float(config.LEFT_MOTOR_SIGN))
        self.right_odometry = WheelOdometry(float(config.RIGHT_MOTOR_SIGN))
        self.last_snapshot: Optional[RobotSnapshot] = None
        self.last_motor_feedback: Optional[dict[str, MotorFeedback]] = None
        self._last_status_log_t = 0.0
        self._last_drive_input_t = 0.0
        self._last_steer_input_t = 0.0
        self._idle_stop_sent = False

    def setup(self) -> None:
        LOGGER.info("Membuka IMU dan motor...")
        self.imu.open()
        self.motors.initialize()

        LOGGER.info("Kalibrasi gyro bias...")
        self.imu.calibrate_gyro_bias()
        LOGGER.info("Kalibrasi zero angle...")
        self.imu.calibrate_zero_angle()

        try:
            feedback = self.motors.query_both()
        except Exception:
            feedback = None
            LOGGER.warning("Gagal seed query motor. Feedback awal akan diambil dari command pertama.")

        self.last_motor_feedback = feedback
        if feedback is not None:
            self.left_odometry.reset(feedback.get("left"))
            self.right_odometry.reset(feedback.get("right"))

        self.keyboard.start()
        self.request_mode(RobotMode.BALANCE)
        LOGGER.info("Setup selesai. Mode awal: %s", self.mode.value)

    def shutdown(self) -> None:
        self.running = False
        self.keyboard.stop()
        try:
            self.motors.stop_all()
        except Exception as exc:
            LOGGER.warning("Stop motors saat shutdown gagal: %s", exc)
        try:
            self.motors.close()
        except Exception as exc:
            LOGGER.warning("Close motors gagal: %s", exc)
        try:
            self.imu.close()
        except Exception as exc:
            LOGGER.warning("Close IMU gagal: %s", exc)

    def request_mode(self, mode: RobotMode) -> None:
        if mode == self.mode:
            return
        LOGGER.info("Mode change: %s -> %s", self.mode.value, mode.value)
        self.previous_mode = self.mode
        self.mode = mode
        self.drive_intent = DriveIntent.STOP
        self.steer_intent = SteerIntent.STRAIGHT
        self.targets = ControlTargets(
            setpoint_psi=float(getattr(config, "SETPOINT_IMU", 0.0)),
            setpoint_theta=float(getattr(config, "SETPOINT_MOTOR", 0.0)),
            turn_command=0.0,
        )
        self._idle_stop_sent = False
        if mode == RobotMode.QUIT:
            self.running = False

    def _run_calibration(self) -> None:
        LOGGER.info("Kalibrasi dimulai. Robot harus diam.")
        self.request_mode(RobotMode.CALIBRATION)
        try:
            self.motors.stop_all()
        except Exception as exc:
            LOGGER.warning("Gagal stop motor sebelum kalibrasi: %s", exc)
        self.imu.calibrate_gyro_bias()
        zero = self.imu.calibrate_zero_angle()
        self.drive_intent = DriveIntent.STOP
        self.steer_intent = SteerIntent.STRAIGHT
        LOGGER.info("Kalibrasi selesai. zero_offset_deg=%.4f", zero)
        self.request_mode(RobotMode.BALANCE)

    def handle_key(self, key: str) -> None:
        key = key.lower().strip()
        if not key:
            return

        now = time.monotonic()

        if key == "k":
            self._run_calibration()
            return
        if key == "b":
            self.request_mode(RobotMode.BALANCE)
            return
        if key == "m":
            self.request_mode(RobotMode.MONO)
            return
        if key == "n":
            self.request_mode(RobotMode.NUGGET)
            return
        if key == "q":
            self.request_mode(RobotMode.QUIT)
            return
        if key == "x":
            self.drive_intent = DriveIntent.STOP
            self.steer_intent = SteerIntent.STRAIGHT
            self._last_drive_input_t = now
            self._last_steer_input_t = now
            return

        if key == "w":
            self.drive_intent = DriveIntent.FORWARD
            self._last_drive_input_t = now
            return
        if key == "s":
            self.drive_intent = DriveIntent.BACKWARD
            self._last_drive_input_t = now
            return

        if key == "a" and self.mode == RobotMode.NUGGET:
            self.steer_intent = SteerIntent.LEFT
            self._last_steer_input_t = now
            return
        if key == "d" and self.mode == RobotMode.NUGGET:
            self.steer_intent = SteerIntent.RIGHT
            self._last_steer_input_t = now
            return

    def _expire_key_intents(self, now: float) -> None:
        timeout_s = float(config.KEY_HOLD_TIMEOUT_S)
        if self.drive_intent != DriveIntent.STOP and (now - self._last_drive_input_t) > timeout_s:
            self.drive_intent = DriveIntent.STOP
        if self.steer_intent != SteerIntent.STRAIGHT and (now - self._last_steer_input_t) > timeout_s:
            self.steer_intent = SteerIntent.STRAIGHT

    def build_targets(self) -> ControlTargets:
        setpoint_theta = float(getattr(config, "SETPOINT_MOTOR", 0.0))
        setpoint_psi = float(getattr(config, "SETPOINT_IMU", 0.0))
        turn_command = 0.0

        if self.mode == RobotMode.MONO or self.mode == RobotMode.NUGGET:
            if self.drive_intent == DriveIntent.FORWARD:
                setpoint_psi = float(config.MANUAL_FORWARD_TARGET_DEG)
            elif self.drive_intent == DriveIntent.BACKWARD:
                setpoint_psi = float(config.MANUAL_BACKWARD_TARGET_DEG)
            else:
                setpoint_psi = float(getattr(config, "SETPOINT_IMU", 0.0))

        if self.mode == RobotMode.NUGGET:
            if self.steer_intent == SteerIntent.LEFT:
                turn_command = -1.0
            elif self.steer_intent == SteerIntent.RIGHT:
                turn_command = 1.0

        if self.mode in {RobotMode.IDLE, RobotMode.CALIBRATION, RobotMode.BALANCE, RobotMode.QUIT}:
            if self.mode != RobotMode.BALANCE:
                setpoint_psi = float(getattr(config, "SETPOINT_IMU", 0.0))
                setpoint_theta = float(getattr(config, "SETPOINT_MOTOR", 0.0))
                turn_command = 0.0

        self.targets = ControlTargets(
            setpoint_psi=setpoint_psi,
            setpoint_theta=setpoint_theta,
            turn_command=turn_command,
        )
        return self.targets

    def _seed_feedback_if_needed(self) -> None:
        if self.last_motor_feedback is not None:
            return
        self.last_motor_feedback = self.motors.query_both()
        self.left_odometry.reset(self.last_motor_feedback.get("left"))
        self.right_odometry.reset(self.last_motor_feedback.get("right"))

    def read_wheel_state(self) -> WheelState:
        self._seed_feedback_if_needed()
        left_fb = None if self.last_motor_feedback is None else self.last_motor_feedback.get("left")
        right_fb = None if self.last_motor_feedback is None else self.last_motor_feedback.get("right")

        theta_left, dot_theta_left = self.left_odometry.update(left_fb)
        theta_right, dot_theta_right = self.right_odometry.update(right_fb)

        return WheelState(
            theta_left=theta_left,
            theta_right=theta_right,
            dot_theta_left=dot_theta_left,
            dot_theta_right=dot_theta_right,
            left_feedback=left_fb,
            right_feedback=right_fb,
        )

    @staticmethod
    def _select_imu_rate(imu_state: AngleState) -> float:
        if imu_state.axis_used == "roll":
            return float(imu_state.gyro_rate_x)
        return float(imu_state.gyro_rate_y)

    def _motor_feedback_has_fault(self, feedback: Optional[MotorFeedback], side: str) -> bool:
        if feedback is None:
            return False
        if int(feedback.error_code) == 0:
            return False

        decoder = self.motors.left if side == "left" else self.motors.right
        flags = decoder.decode_error_flags(feedback.error_code)
        active = [name for name, value in flags.items() if value]
        LOGGER.error("Motor %s fault: error_code=%s flags=%s", side, feedback.error_code, active)
        return True

    def _enter_safe_idle(self, reason: str) -> None:
        LOGGER.error("SAFE IDLE: %s", reason)
        try:
            self.motors.stop_all()
        except Exception as exc:
            LOGGER.warning("Stop all saat safe idle gagal: %s", exc)
        self.request_mode(RobotMode.IDLE)
        self.last_motor_feedback = None

    def _validate_command(self, command: ActuationCommand) -> ActuationCommand:
        limit = float(config.OUTPUT_LIMIT)
        left = max(-limit, min(limit, float(command.left)))
        right = max(-limit, min(limit, float(command.right)))
        if not math.isfinite(left) or not math.isfinite(right):
            raise RuntimeError("Command tidak finite")
        command.left = left
        command.right = right
        return command

    def _log_status(self, snapshot: RobotSnapshot, command: ActuationCommand) -> None:
        now = snapshot.timestamp
        period = float(config.STATUS_LOG_PERIOD_S)
        if (now - self._last_status_log_t) < period:
            return
        self._last_status_log_t = now

        pd_state = snapshot.controller_state
        u_pd = 0.0 if pd_state is None else float(pd_state.u_PD)
        LOGGER.info(
            "mode=%s drive=%s steer=%s psi=%.3f dpsi=%.3f theta=%.3f dtheta=%.3f u_PD=%.3f cmd=(%.3f, %.3f) src=%s",
            snapshot.mode.value,
            snapshot.drive_intent.value,
            snapshot.steer_intent.value,
            snapshot.imu_state.angle_deg,
            snapshot.imu_rate_deg_s,
            snapshot.wheel.theta_avg,
            snapshot.wheel.dot_theta_avg,
            u_pd,
            command.left,
            command.right,
            command.source,
        )

    def _run_idle_step(self) -> None:
        if self._idle_stop_sent:
            return
        try:
            self.last_motor_feedback = self.motors.command_normalized(0.0, 0.0)
        except Exception as exc:
            LOGGER.warning("Idle stop command gagal: %s", exc)
        self._idle_stop_sent = True

    def run_control_step(self) -> None:
        now = time.monotonic()
        self._expire_key_intents(now)

        if self.mode in {RobotMode.IDLE, RobotMode.CALIBRATION, RobotMode.QUIT}:
            self._run_idle_step()
            return

        imu_state = self.imu.read_angles()
        imu_rate_deg_s = self._select_imu_rate(imu_state)
        wheel_state = self.read_wheel_state()
        targets = self.build_targets()

        snapshot = RobotSnapshot(
            timestamp=now,
            dt=float(imu_state.dt),
            mode=self.mode,
            imu_state=imu_state,
            imu_rate_deg_s=imu_rate_deg_s,
            wheel=wheel_state,
            targets=targets,
            drive_intent=self.drive_intent,
            steer_intent=self.steer_intent,
        )

        if abs(float(imu_state.angle_deg)) > float(config.HARD_SAFE_TILT_DEG):
            self._enter_safe_idle(f"Tilt melebihi HARD_SAFE_TILT_DEG: {imu_state.angle_deg:.3f}")
            return

        if self._motor_feedback_has_fault(wheel_state.left_feedback, "left") or self._motor_feedback_has_fault(wheel_state.right_feedback, "right"):
            self._enter_safe_idle("Motor driver melaporkan fault")
            return

        pd_state = self.control_backend.compute(snapshot)
        snapshot.controller_state = pd_state

        command = self.command_filter.compute(snapshot, pd_state)
        command = self._validate_command(command)

        self.last_motor_feedback = self.motors.command_normalized(command.left, command.right)
        self.last_snapshot = snapshot
        self._idle_stop_sent = False
        self._log_status(snapshot, command)

    def run(self) -> None:
        self.setup()
        period = 1.0 / float(config.CONTROL_HZ)
        next_tick = time.monotonic()
        try:
            while self.running and self.mode != RobotMode.QUIT:
                while not self.event_queue.empty():
                    self.handle_key(self.event_queue.get_nowait())
                self.run_control_step()
                next_tick += period
                sleep_s = next_tick - time.monotonic()
                if sleep_s > 0.0:
                    time.sleep(sleep_s)
                else:
                    next_tick = time.monotonic()
        except KeyboardInterrupt:
            LOGGER.info("KeyboardInterrupt diterima")
        except (DDSM115Error, OSError, RuntimeError) as exc:
            LOGGER.exception("Fatal runtime error: %s", exc)
            self._enter_safe_idle(str(exc))
        finally:
            self.shutdown()


def configure_logging() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
    )


def main() -> int:
    configure_logging()
    app = AteraRobotApp()
    signal.signal(signal.SIGINT, lambda *_: app.request_mode(RobotMode.QUIT))
    signal.signal(signal.SIGTERM, lambda *_: app.request_mode(RobotMode.QUIT))
    app.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
