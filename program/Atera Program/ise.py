from __future__ import annotations

import os
from typing import Dict, List, Optional
import matplotlib.pyplot as plt
import config


class ISE:
    """Kelas pengumpul data dan pencatat akumulasi nilai ISE real-time."""

    def __init__(self, mode_name: str, eval_time: float = getattr(config, "EVALUATION_TIME", 10.0)) -> None:
        self.mode_name = mode_name
        self.eval_time = eval_time
        self.elapsed_time = 0.0
        self.ise_psi = 0.0
        self.ise_theta = 0.0
        self.is_completed = False

        # Riwayat data untuk perplotan
        self.time_history: List[float] = []
        self.error_psi_history: List[float] = []
        self.error_theta_history: List[float] = []
        self.ise_psi_history: List[float] = []
        self.ise_theta_history: List[float] = []

    def reset((self) -> None:
        """Mereset seluruh akumulator evaluasi."""
        self.elapsed_time = 0.0
        self.ise_psi = 0.0
        self.ise_theta = 0.0
        self.is_completed = False
        self.time_history.clear()
        self.error_psi_history.clear()
        self.error_theta_history.clear()
        self.ise_psi_history.clear()
        self.ise_theta_history.clear()

    def update(self, dt: float, error_psi: float, error_theta: float) -> Dict[str, float | bool]:
        """Memperbarui akumulasi ISE secara diskrit setiap loop control."""
        if self.is_completed:
            return {
                "completed": True,
                "elapsed_time": self.elapsed_time,
                "ise_psi": self.ise_psi,
                "ise_theta": self.ise_theta,
            }

        self.elapsed_time += dt
        self.ise_psi += (error_psi**2) * dt
        self.ise_theta += (error_theta**2) * dt

        self.time_history.append(self.elapsed_time)
        self.error_psi_history.append(error_psi)
        self.error_theta_history.append(error_theta)
        self.ise_psi_history.append(self.ise_psi)
        self.ise_theta_history.append(self.ise_theta)

        if self.elapsed_time >= self.eval_time:
            self.is_completed = True
            self.generate_plot()

        return {
            "completed": self.is_completed,
            "elapsed_time": self.elapsed_time,
            "ise_psi": self.ise_psi,
            "ise_theta": self.ise_theta,
        }

    def generate_plot(self) -> str:
        """Menghasilkan plot evaluasi Error dan ISE lalu menyimpannya sebagai file gambar."""
        fig, axs = plt.subplots(2, 2, figsize=(12, 8))
        fig.suptitle(
            f"Evaluasi Performa ISE Mode [{self.mode_name}] (Target: {self.eval_time}s)\n"
            f"Final ISE Psi: {self.ise_psi:.4f} | Final ISE Theta: {self.ise_theta:.4f}",
            fontsize=12,
            fontweight="bold",
        )

        # Plot Error Psi
        axs[0, 0].plot(self.time_history, self.error_psi_history, "r-", label=r"Error $\psi$")
        axs[0, 0].set_title(r"Error Kemiringan Pitch ($e_\psi$)")
        axs[0, 0].set_ylabel("Error (deg)")
        axs[0, 0].grid(True)
        axs[0, 0].legend()

        # Plot ISE Psi
        axs[0, 1].plot(self.time_history, self.ise_psi_history, "m-", label=r"ISE $\psi$")
        axs[0, 1].set_title(r"Integral Square Error Pitch ($ISE_\psi$)")
        axs[0, 1].set_ylabel(r"$\text{deg}^2 \cdot \text{s}$")
        axs[0, 1].grid(True)
        axs[0, 1].legend()

        # Plot Error Theta
        axs[1, 0].plot(self.time_history, self.error_theta_history, "b-", label=r"Error $\theta$")
        axs[1, 0].set_title(r"Error Posisi Roda ($e_\theta$)")
        axs[1, 0].set_xlabel("Waktu (detik)")
        axs[1, 0].set_ylabel("Error (deg)")
        axs[1, 0].grid(True)
        axs[1, 0].legend()

        # Plot ISE Theta
        axs[1, 1].plot(self.time_history, self.ise_theta_history, "c-", label=r"ISE $\theta$")
        axs[1, 1].set_title(r"Integral Square Error Roda ($ISE_\theta$)")
        axs[1, 1].set_xlabel("Waktu (detik)")
        axs[1, 1].set_ylabel(r"$\text{deg}^2 \cdot \text{s}$")
        axs[1, 1].grid(True)
        axs[1, 1].legend()

        plt.tight_layout()
        filename = f"ise_plot_{self.mode_name.lower()}.png"
        plt.savefig(filename, dpi=300)
        plt.close(fig)
        return filename


# Global Evaluator Instances
_eval_pd = ISE(mode_name="PD")
_eval_cbfqp = ISE(mode_name="CBF_QP")


def ISE_PD(dt: float, error_psi: float, error_theta: float) -> Dict[str, float | bool]:
    """Fungsi pemanggil evaluasi ISE untuk Mode PD murni."""
    return _eval_pd.update(dt, error_psi, error_theta)


def ISE_CBFQP(dt: float, error_psi: float, error_theta: float) -> Dict[str, float | bool]:
    """Fungsi pemanggil evaluasi ISE untuk Mode PD + CBF-QP."""
    return _eval_cbfqp.update(dt, error_psi, error_theta)


def reset_ise_evaluator(mode: str = "ALL") -> None:
    """Mereset status pengujian evaluasi ISE."""
    if mode in ("PD", "ALL"):
        _eval_pd.reset()
    if mode in ("CBF_QP", "ALL"):
        _eval_cbfqp.reset()