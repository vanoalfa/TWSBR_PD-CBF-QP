from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List
import time


class AppMode(Enum):
    IDLE = auto()
    BALANCING = auto()
    STOPPED = auto()


class ControlMode(Enum):
    PD_ONLY = auto()
    PD_PLUS_CBF_QP = auto()


@dataclass
class ControllerState:
    mode: AppMode = AppMode.IDLE
    control_mode: ControlMode = ControlMode.PD_ONLY
    running: bool = True

    # Existing/common runtime state
    theta: float = 0.0
    theta_dot: float = 0.0
    x: float = 0.0
    x_dot: float = 0.0
    target_theta: float = 0.0
    target_x: float = 0.0
    last_control: float = 0.0

    # Requested fields
    cbf_qp_available: bool = True
    cbf_qp_effective: bool = False
    last_control_path: str = "none"
    last_cbf_qp_status: str = "not_used"

    logs: List[str] = field(default_factory=list)


class AteraMaint:
    def __init__(self) -> None:
        self.state = ControllerState()
        self.kp = 24.0
        self.kd = 4.5
        self.max_control = 18.0

    def log(self, message: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        line = f"[{stamp}] {message}"
        self.state.logs.append(line)
        if len(self.state.logs) > 12:
            self.state.logs = self.state.logs[-12:]

    def help_text(self) -> str:
        active = self.state.control_mode.name
        return (
            "Atera Maintenance Console\n"
            "\n"
            "Commands:\n"
            "  h / help         Tampilkan bantuan ini\n"
            "  b / balance      Masuk ke mode BALANCING\n"
            "  i / idle         Kembali ke mode IDLE\n"
            "  t / toggle       Ganti mode kontrol aktif\n"
            "  m pd             Pakai kontrol PD_ONLY\n"
            "  m cbf            Pakai kontrol PD_PLUS_CBF_QP\n"
            "  s / step         Jalankan satu siklus kontrol\n"
            "  p / perturb      Tambahkan gangguan kecil\n"
            "  q / quit         Keluar\n"
            "\n"
            "Control modes:\n"
            "  - PD_ONLY: pakai pengendali PD saja\n"
            "  - PD_PLUS_CBF_QP: pakai PD lalu disaring/dikoreksi oleh CBF-QP jika tersedia\n"
            "\n"
            "Cara ganti mode kontrol:\n"
            "  - ketik 't' untuk toggle cepat\n"
            "  - ketik 'm pd' untuk pilih PD_ONLY\n"
            "  - ketik 'm cbf' untuk pilih PD_PLUS_CBF_QP\n"
            f"\nMode kontrol aktif saat ini: {active}\n"
        )

    def ui_indicator(self) -> str:
        if self.state.control_mode is ControlMode.PD_ONLY:
            mode_badge = "PD_ONLY"
            mode_desc = "PD controller only"
        else:
            mode_badge = "PD_PLUS_CBF_QP"
            mode_desc = "PD with CBF-QP safety filter"

        availability = "READY" if self.state.cbf_qp_available else "UNAVAILABLE"
        effective = "ACTIVE" if self.state.cbf_qp_effective else "BYPASSED"

        return (
            f"[STATE:{self.state.mode.name}] "
            f"[CONTROL:{mode_badge}] "
            f"[CBF-QP:{availability}/{effective}] "
            f"[PATH:{self.state.last_control_path}] "
            f"[STATUS:{self.state.last_cbf_qp_status}] "
            f"[INFO:{mode_desc}]"
        )

    def toggle_control_mode(self) -> None:
        if self.state.control_mode is ControlMode.PD_ONLY:
            self.state.control_mode = ControlMode.PD_PLUS_CBF_QP
        else:
            self.state.control_mode = ControlMode.PD_ONLY
        self.log(f"Control mode diubah ke {self.state.control_mode.name}")

    def set_control_mode(self, mode: ControlMode) -> None:
        self.state.control_mode = mode
        self.log(f"Control mode di-set ke {mode.name}")

    def pd_control(self) -> float:
        error = self.state.target_theta - self.state.theta
        error_dot = -self.state.theta_dot
        u = (self.kp * error) + (self.kd * error_dot)
        u = max(-self.max_control, min(self.max_control, u))
        return u

    def apply_cbf_qp(self, nominal_u: float) -> float:
        if not self.state.cbf_qp_available:
            self.state.cbf_qp_effective = False
            self.state.last_cbf_qp_status = "solver_unavailable"
            self.state.last_control_path = "pd_fallback"
            return nominal_u

        safe_limit = 10.0
        filtered_u = max(-safe_limit, min(safe_limit, nominal_u))
        self.state.cbf_qp_effective = filtered_u != nominal_u
        if self.state.cbf_qp_effective:
            self.state.last_cbf_qp_status = "clamped_for_safety"
        else:
            self.state.last_cbf_qp_status = "pass_through"
        self.state.last_control_path = "pd_plus_cbf_qp"
        return filtered_u

    def compute_control(self) -> float:
        nominal_u = self.pd_control()

        if self.state.control_mode is ControlMode.PD_ONLY:
            self.state.cbf_qp_effective = False
            self.state.last_cbf_qp_status = "disabled_by_mode"
            self.state.last_control_path = "pd_only"
            return nominal_u

        return self.apply_cbf_qp(nominal_u)

    def integrate_dynamics(self, control_u: float, dt: float = 0.02) -> None:
        disturbance = 0.15 * self.state.x_dot
        theta_acc = (control_u * 0.11) - (self.state.theta * 1.7) - (self.state.theta_dot * 0.42) - disturbance
        x_acc = (control_u * 0.05) - (self.state.x_dot * 0.18)

        self.state.theta_dot += theta_acc * dt
        self.state.theta += self.state.theta_dot * dt
        self.state.x_dot += x_acc * dt
        self.state.x += self.state.x_dot * dt
        self.state.last_control = control_u

    def step_balancing(self) -> None:
        control_u = self.compute_control()
        self.integrate_dynamics(control_u)
        self.log(
            "BALANCING tick | "
            f"mode={self.state.control_mode.name} | "
            f"u={control_u:.3f} | "
            f"theta={self.state.theta:.3f} | "
            f"path={self.state.last_control_path} | "
            f"cbf={self.state.last_cbf_qp_status}"
        )

    def state_machine_step(self) -> None:
        if self.state.mode is AppMode.IDLE:
            self.state.last_control_path = "idle"
            self.state.last_cbf_qp_status = "not_running"
            self.state.cbf_qp_effective = False
            return

        if self.state.mode is AppMode.BALANCING:
            # Requested behavior: use the selected control_mode while balancing.
            self.step_balancing()
            return

        if self.state.mode is AppMode.STOPPED:
            self.state.running = False
            self.state.last_control_path = "stopped"
            self.state.last_cbf_qp_status = "stopped"
            self.state.cbf_qp_effective = False

    def perturb(self) -> None:
        self.state.theta += 0.18
        self.state.theta_dot += 0.05
        self.log("Gangguan kecil ditambahkan ke plant")

    def print_status(self) -> None:
        print()
        print(self.ui_indicator())
        print(
            "theta={:.3f} theta_dot={:.3f} x={:.3f} x_dot={:.3f} u={:.3f}".format(
                self.state.theta,
                self.state.theta_dot,
                self.state.x,
                self.state.x_dot,
                self.state.last_control,
            )
        )
        if self.state.logs:
            print("Recent logs:")
            for line in self.state.logs[-5:]:
                print(f"  {line}")

    def handle_command(self, raw: str) -> None:
        cmd = raw.strip().lower()
        if not cmd:
            return

        if cmd in {"h", "help"}:
            print(self.help_text())
            return

        if cmd in {"b", "balance"}:
            self.state.mode = AppMode.BALANCING
            self.log("Masuk ke mode BALANCING")
            return

        if cmd in {"i", "idle"}:
            self.state.mode = AppMode.IDLE
            self.log("Masuk ke mode IDLE")
            return

        if cmd in {"t", "toggle"}:
            self.toggle_control_mode()
            return

        if cmd == "m pd":
            self.set_control_mode(ControlMode.PD_ONLY)
            return

        if cmd == "m cbf":
            self.set_control_mode(ControlMode.PD_PLUS_CBF_QP)
            return

        if cmd in {"s", "step"}:
            self.state_machine_step()
            self.print_status()
            return

        if cmd in {"p", "perturb"}:
            self.perturb()
            self.print_status()
            return

        if cmd in {"q", "quit"}:
            self.state.mode = AppMode.STOPPED
            self.state_machine_step()
            return

        print("Perintah tidak dikenal. Ketik 'help' untuk bantuan.")

    def run(self) -> None:
        print(self.help_text())
        self.print_status()
        while self.state.running:
            try:
                raw = input("atera> ")
            except (EOFError, KeyboardInterrupt):
                print()
                raw = "quit"
            self.handle_command(raw)

        print("Atera maintenance console selesai.")


if __name__ == "__main__":
    AteraMaint().run()
