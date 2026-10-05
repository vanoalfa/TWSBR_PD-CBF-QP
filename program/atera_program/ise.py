"""ise.py - evaluasi ISE (Integral Square Error) ATERA.

    e(t) = referensi - nilai terukur          ISE = integral e(t)^2 dt   (aturan trapesium)

ISE dihitung untuk keempat state; yang utama adalah psi dan dpsi (batasan masalah skripsi).
Tiap percobaan (dari MULAI sampai roda dimatikan / ganti mode / FAULT) disimpan di folder
PLOT_EVALUASI sebagai:

    YYYYMMDD_COBA PD_PERCOBAAN KE-XX.csv / .png
    YYYYMMDD_COBA PD+CBF_PERCOBAAN KE-XX.csv / .png

XX naik otomatis per tanggal dan per mode. CSV berisi data mentah (SI: rad, rad/s, N m);
PNG berisi plot error (sumbu y) terhadap waktu (sumbu x).

Penyimpanan berjalan di PROSES TERPISAH supaya matplotlib tidak mengganggu loop kontrol.
"""

from __future__ import annotations

import csv
import logging
import math
import multiprocessing as mp
import os
import re
import time
from typing import List, Optional

import config

LOGGER = logging.getLogger(__name__)

MODE_PD = "PD"
MODE_PD_CBF = "PD+CBF"

CSV_COLUMNS = (
    "t_s",
    "theta_rad", "dtheta_rad_s", "psi_rad", "dpsi_rad_s",
    "theta_ref_rad", "dtheta_ref_rad_s", "psi_ref_rad", "dpsi_ref_rad_s",
    "e_theta_rad", "e_dtheta_rad_s", "e_psi_rad", "e_dpsi_rad_s",
    "u_PD_Nm", "u_Nm", "cbf_active", "feasible",
    "h1", "h2", "h3", "h4",
    "ISE_theta", "ISE_dtheta", "ISE_psi", "ISE_dpsi",
)
_E0 = CSV_COLUMNS.index("e_theta_rad")
_ISE0 = CSV_COLUMNS.index("ISE_theta")


def plot_dir() -> str:
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), config.ISE_PLOT_DIR)


def next_name(mode: str, reserved=()) -> str:
    """Nama dasar berikutnya: YYYYMMDD_COBA <mode>_PERCOBAAN KE-XX."""
    date = time.strftime("%Y%m%d")
    prefix = f"{date}_COBA {mode}_PERCOBAAN KE-"
    pattern = re.compile("^" + re.escape(prefix) + r"(\d+)\.")
    used = [0]
    os.makedirs(plot_dir(), exist_ok=True)
    for name in list(os.listdir(plot_dir())) + [r + "." for r in reserved]:
        match = pattern.match(name)
        if match:
            used.append(int(match.group(1)))
    return f"{prefix}{max(used) + 1:02d}"


class ISERecorder:
    def __init__(self) -> None:
        self.recording = False
        self.mode = MODE_PD
        self.name = ""
        self.rows: List[tuple] = []
        self.ise = [0.0, 0.0, 0.0, 0.0]         # theta, dtheta, psi, dpsi
        self.duration = 0.0
        self.active_count = 0
        self.infeasible_count = 0
        self.last_saved = ""                    # nama dasar percobaan terakhir yang disimpan
        self.last_summary = ""
        self._t0 = 0.0
        self._prev_t = 0.0
        self._prev_e2 = (0.0, 0.0, 0.0, 0.0)
        self._reserved: List[str] = []
        self._workers: List[mp.Process] = []

    def start(self, mode: str) -> None:
        self.mode = mode
        self.name = next_name(mode, self._reserved)
        self._reserved.append(self.name)
        self.rows = []
        self.ise = [0.0, 0.0, 0.0, 0.0]
        self.duration = 0.0
        self.active_count = 0
        self.infeasible_count = 0
        self._t0 = time.monotonic()
        self._prev_t = 0.0
        self._prev_e2 = (0.0, 0.0, 0.0, 0.0)
        self.recording = True

    def add(self, x, x_ref, u_pd: float, u: float, cbf_active: bool, feasible: bool, h) -> None:
        """Tambah satu sampel. Hanya operasi ringan: aman dipanggil tiap siklus kontrol."""
        if not self.recording:
            return
        t = time.monotonic() - self._t0
        e = tuple(float(x_ref[i]) - float(x[i]) for i in range(4))
        e2 = tuple(v * v for v in e)
        if self.rows:
            dt = t - self._prev_t
            for i in range(4):
                self.ise[i] += 0.5 * (e2[i] + self._prev_e2[i]) * dt
        self._prev_t, self._prev_e2 = t, e2
        self.duration = t
        self.active_count += int(cbf_active)
        self.infeasible_count += int(not feasible)
        self.rows.append((
            t, float(x[0]), float(x[1]), float(x[2]), float(x[3]),
            float(x_ref[0]), float(x_ref[1]), float(x_ref[2]), float(x_ref[3]),
            e[0], e[1], e[2], e[3],
            float(u_pd), float(u), int(cbf_active), int(feasible),
            float(h[0]), float(h[1]), float(h[2]), float(h[3]),
            self.ise[0], self.ise[1], self.ise[2], self.ise[3],
        ))

    @property
    def ise_psi_deg2s(self) -> float:
        return self.ise[2] * math.degrees(1.0) ** 2

    @property
    def ise_dpsi_deg2s(self) -> float:
        return self.ise[3] * math.degrees(1.0) ** 2

    def stop(self, reason: str = "") -> Optional[str]:
        """Akhiri percobaan dan simpan .csv + .png di proses terpisah. Kembalikan nama dasarnya."""
        if not self.recording:
            return None
        self.recording = False
        rows, self.rows = self.rows, []
        if not rows or self.duration < float(config.ISE_MIN_DURATION_S):
            self.last_summary = f"{self.name}: terlalu singkat, tidak disimpan"
            return None
        meta = {
            "name": self.name, "mode": self.mode, "reason": reason,
            "psi_max": config.psi_max, "dtheta_max": config.dtheta_max, "u_max": config.u_max,
            "zeta_deg": config.zeta_DEG,
            "gains": (config.Kp_theta, config.Kd_dtheta, config.Kp_psi, config.Kd_dpsi, config.Kvel),
            "alphas": (config.Alpha_1, config.Alpha_2),
        }
        base = os.path.join(plot_dir(), self.name)
        try:
            worker = mp.get_context("spawn").Process(target=save_run, args=(rows, meta, base))
            worker.start()
            self._workers.append(worker)
        except Exception as exc:
            LOGGER.error("Proses penyimpanan ISE gagal dibuat (%s), simpan langsung", exc)
            save_run(rows, meta, base)
        self.last_saved = self.name
        self.last_summary = (f"{self.name} | {self.duration:.1f} s | "
                             f"ISE psi = {self.ise_psi_deg2s:.3f} deg^2 s")
        self._workers = [w for w in self._workers if w.is_alive()]
        return self.name

    def close(self) -> None:
        """Tunggu semua penyimpanan selesai (dipanggil saat program keluar)."""
        for worker in self._workers:
            worker.join(timeout=30.0)
        self._workers = []


# ------------------------------------------------------------------ penyimpanan --
def save_run(rows, meta, base: str) -> None:
    """Tulis <base>.csv dan <base>.png. Berjalan di proses terpisah."""
    os.makedirs(os.path.dirname(base), exist_ok=True)
    with open(base + ".csv", "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(CSV_COLUMNS)
        writer.writerows(rows)
    try:
        save_plot(rows, meta, base + ".png")
    except Exception as exc:
        logging.getLogger(__name__).error("Plot ISE gagal: %s", exc)


def save_plot(rows, meta, path: str) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    deg = math.degrees(1.0)
    t = [r[0] for r in rows]
    e_psi = [r[_E0 + 2] * deg for r in rows]
    e_dpsi = [r[_E0 + 3] * deg for r in rows]
    dtheta = [r[2] for r in rows]
    u_pd = [r[13] for r in rows]
    u = [r[14] for r in rows]
    active = [r[15] for r in rows]
    ise_psi = rows[-1][_ISE0 + 2] * deg * deg
    ise_dpsi = rows[-1][_ISE0 + 3] * deg * deg
    psi_max = meta["psi_max"] * deg

    ink, muted, grid = "#0b0b0b", "#898781", "#e4e3df"
    blue, orange, limit = "#2a78d6", "#eb6834", "#d03b3b"
    plt.rcParams.update({"font.size": 9, "axes.edgecolor": muted, "axes.labelcolor": ink,
                         "xtick.color": muted, "ytick.color": muted, "text.color": ink})
    fig, axes = plt.subplots(4, 1, figsize=(10, 10.5), sharex=True, facecolor="#fcfcfb")

    def style(ax, ylabel, title):
        ax.set_facecolor("#fcfcfb")
        ax.grid(True, color=grid, linewidth=0.8)
        ax.set_axisbelow(True)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        ax.set_ylabel(ylabel)
        ax.set_title(title, loc="left", fontsize=9.5, fontweight="bold")
        ax.axhline(0.0, color=muted, linewidth=0.8)

    def shade_active(ax):
        start = None
        for i, flag in enumerate(active):
            if flag and start is None:
                start = t[i]
            if start is not None and (not flag or i == len(active) - 1):
                ax.axvspan(start, t[i], color=orange, alpha=0.12, linewidth=0)
                start = None

    ax = axes[0]
    style(ax, "error psi [deg]", f"Error sudut badan psi   |   ISE = {ise_psi:.4f} deg^2 s")
    ax.plot(t, e_psi, color=blue, linewidth=1.6)
    for sign in (1.0, -1.0):
        ax.axhline(sign * psi_max, color=limit, linewidth=1.0, linestyle="--")
    ax.text(t[-1], psi_max, f" batas safety set +-{psi_max:.0f} deg", color=limit,
            fontsize=8, va="bottom", ha="right")
    shade_active(ax)

    ax = axes[1]
    style(ax, "error dpsi [deg/s]", f"Error kecepatan sudut badan dpsi   |   ISE = {ise_dpsi:.2f} (deg/s)^2 s")
    ax.plot(t, e_dpsi, color=blue, linewidth=1.2)
    shade_active(ax)

    ax = axes[2]
    style(ax, "dtheta [rad/s]", "Kecepatan sudut roda dtheta")
    ax.plot(t, dtheta, color=blue, linewidth=1.4)
    span = max(abs(min(dtheta)), abs(max(dtheta)))
    if span > 0.5 * meta["dtheta_max"]:
        for sign in (1.0, -1.0):
            ax.axhline(sign * meta["dtheta_max"], color=limit, linewidth=1.0, linestyle="--")
    shade_active(ax)

    ax = axes[3]
    style(ax, "torsi [N m]", "Sinyal kontrol (torsi total dua roda)")
    ax.plot(t, u_pd, color=orange, linewidth=1.2, label="u_PD")
    ax.plot(t, u, color=blue, linewidth=1.6, label="u terkirim" if meta["mode"] == MODE_PD else "u_QP terkirim")
    top = 1.15 * meta["u_max"]
    ax.set_ylim(-top, top)
    ax.legend(loc="upper right", frameon=False, ncol=2)
    shade_active(ax)
    ax.set_xlabel("waktu [s]")

    kp_t, kd_t, kp_p, kd_p, kvel = meta["gains"]
    subtitle = (f"Kp_theta={kp_t:g}  Kd_dtheta={kd_t:g}  Kp_psi={kp_p:g}  Kd_dpsi={kd_p:g}  Kvel={kvel:g}"
                f"   |   zeta={meta['zeta_deg']:g} deg")
    if meta["mode"] == MODE_PD_CBF:
        n_active = sum(active)
        subtitle += (f"   |   Alpha_1={meta['alphas'][0]:g}  Alpha_2={meta['alphas'][1]:g}"
                     f"   |   CBF aktif {100.0 * n_active / len(rows):.1f}% (area jingga)")
    fig.suptitle(f"ATERA - {meta['name']}   ({t[-1]:.1f} s)", x=0.01, y=0.985, ha="left",
                 fontsize=12, fontweight="bold")
    fig.text(0.01, 0.948, subtitle, fontsize=8.5, color="#52514e")
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(path, dpi=130)
    plt.close(fig)