"""ise.py - Evaluasi error PD (ISE-style logging & plotting) untuk ATERA.

Modul ini TIDAK ikut campur dalam aksi kontrol robot sama sekali. Tugasnya
murni mencatat data error (selisih sudut terhadap setpoint) selama satu sesi
balancing berjalan, lalu membuat & menyimpan grafik evaluasinya begitu sesi
tersebut berakhir:

    - Plot 1: error badan      (error_psi)          dari MPU6050   vs waktu
    - Plot 2: error roda kiri  (error_theta_left)    dari DDSM115   vs waktu
    - Plot 3: error roda kanan (error_theta_right)   dari DDSM115   vs waktu

Alur pemakaian (lihat atera_main.py):

    evaluator = EvaluasiError()
    ...
    # saat tombol MULAI ditekan / start_balancing() berhasil:
    evaluator.start_session()
    ...
    # setiap siklus kontrol selama mode BALANCING:
    evaluator.record(error_psi, error_theta_left, error_theta_right)
    ...
    # saat balancing berhenti (user stop / fault / quit):
    evaluator.stop_and_save_session()

File hasil evaluasi disimpan ke folder PLOT_EVALUASI (relatif terhadap
direktori kerja program, sesuai config.PLOT_EVALUASI_DIR) dengan format nama
"YYYYMMDD-URUTAN.png", di mana URUTAN otomatis bertambah tiap kali sesi baru
disimpan pada tanggal yang sama (mis. 20260927-1.png, 20260927-2.png, dst).

CATATAN DESAIN (penting dibaca):
Permintaan awal menyebutkan "plot langsung muncul ketika tombol MULAI
ditekan". Menampilkan jendela matplotlib interaktif secara live di tengah
loop kontrol real-time 200 Hz pada Raspberry Pi berisiko tinggi memblokir
atau memperlambat loop tersebut (matplotlib tidak dirancang untuk update
secepat itu, apalagi lewat SSH/headless). Karena itu modul ini mengadaptasi
permintaan tersebut menjadi: PENCATATAN dimulai persis saat tombol MULAI
ditekan (start_session dipanggil dari start_balancing()), sedangkan
PENGGAMBARAN & PENYIMPANAN grafik dilakukan sekali saja tepat setelah sesi
balancing berakhir (stop_and_save_session). Backend matplotlib diset ke
"Agg" (non-interaktif) sehingga aman dipanggil headless dan tidak pernah
membuka jendela GUI. Jika Anda tetap ingin jendela plot muncul secara
interaktif di layar (mis. saat development di desktop, bukan di robot),
lihat catatan di bagian bawah file ini.
"""
from __future__ import annotations

import logging
import os
import time
from dataclasses import dataclass, field
from typing import List, Optional

import matplotlib

matplotlib.use("Agg") 
import matplotlib.pyplot as plt

import config as tunning

LOGGER = logging.getLogger("atera.ise")


@dataclass
class _SessionBuffer:
    t: List[float] = field(default_factory=list)
    error_psi: List[float] = field(default_factory=list)
    error_theta_left: List[float] = field(default_factory=list)
    error_theta_right: List[float] = field(default_factory=list)
    t0: float = 0.0


class EvaluasiError:
    """Mencatat error PD selama satu sesi balancing dan membuat plot evaluasi."""

    def __init__(self, plot_dir: Optional[str] = None) -> None:
        self.plot_dir = plot_dir or getattr(tunning, "PLOT_EVALUASI_DIR", "PLOT_EVALUASI")
        self._buffer: Optional[_SessionBuffer] = None
        self._active: bool = False

    # ------------------------------------------------------------------
    # Kontrol sesi
    # ------------------------------------------------------------------
    def start_session(self) -> None:
        """Dipanggil tepat saat tombol MULAI ditekan (balancing benar-benar mulai)."""
        self._buffer = _SessionBuffer(t0=time.monotonic())
        self._active = True
        LOGGER.info("Sesi evaluasi ISE dimulai.")

    def record(
        self,
        error_psi: float,
        error_theta_left: float,
        error_theta_right: float,
    ) -> None:
        """Dipanggil tiap siklus kontrol selama mode BALANCING. Ringan (list
        append saja) sehingga aman dipanggil di loop 200 Hz."""
        if not self._active or self._buffer is None:
            return
        now = time.monotonic() - self._buffer.t0
        self._buffer.t.append(now)
        self._buffer.error_psi.append(float(error_psi))
        self._buffer.error_theta_left.append(float(error_theta_left))
        self._buffer.error_theta_right.append(float(error_theta_right))

    def stop_and_save_session(self) -> Optional[str]:
        """Dipanggil saat balancing berhenti (stop manual / fault / quit).

        Membuat 3 subplot (badan, roda kiri, roda kanan) dalam satu figure,
        lalu menyimpannya ke PLOT_EVALUASI/YYYYMMDD-URUTAN.png.

        Mengembalikan path file yang tersimpan, atau None jika tidak ada
        sesi aktif / datanya terlalu sedikit untuk digambar.
        """
        if not self._active:
            return None
        self._active = False
        buffer = self._buffer
        self._buffer = None
        if buffer is None or len(buffer.t) < 2:
            LOGGER.info("Sesi evaluasi ISE dilewati (data terlalu sedikit untuk diplot).")
            return None
        try:
            path = self._save_plot(buffer)
            LOGGER.info("Plot evaluasi ISE disimpan: %s", path)
            return path
        except Exception as exc:  # pragma: no cover - jangan sampai crash robot
            LOGGER.error("Gagal menyimpan plot evaluasi ISE: %s", exc)
            return None

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------
    def _next_filepath(self) -> str:
        os.makedirs(self.plot_dir, exist_ok=True)
        today = time.strftime("%Y%m%d")
        urutan = 1
        while True:
            candidate = os.path.join(self.plot_dir, f"{today}-{urutan}.png")
            if not os.path.exists(candidate):
                return candidate
            urutan += 1

    def _save_plot(self, buffer: _SessionBuffer) -> str:
        fig, axes = plt.subplots(3, 1, figsize=(9, 9), sharex=True)

        axes[0].plot(buffer.t, buffer.error_psi, color="tab:red")
        axes[0].set_title("Error Badan (psi)")
        axes[0].set_ylabel("error_psi (deg)")
        axes[0].grid(True, alpha=0.3)

        axes[1].plot(buffer.t, buffer.error_theta_left, color="tab:blue")
        axes[1].set_title("Error Roda Kiri (theta)")
        axes[1].set_ylabel("error_theta_left (deg)")
        axes[1].grid(True, alpha=0.3)

        axes[2].plot(buffer.t, buffer.error_theta_right, color="tab:green")
        axes[2].set_title("Error Roda Kanan (theta)")
        axes[2].set_ylabel("error_theta_right (deg)")
        axes[2].set_xlabel("Waktu (s)")
        axes[2].grid(True, alpha=0.3)

        fig.suptitle("Evaluasi Error PD - ATERA Self Balancing Robot")
        fig.tight_layout()

        path = self._next_filepath()
        fig.savefig(path, dpi=150)
        plt.close(fig)
        return path


# ----------------------------------------------------------------------
# CATATAN: menampilkan plot secara interaktif (opsional, non-robot)
# ----------------------------------------------------------------------
# Jika file ini dijalankan di komputer development (BUKAN di Raspberry Pi
# saat robot berjalan) dan Anda ingin jendela grafik benar-benar muncul di
# layar, ganti backend di bagian atas file dari:
#       matplotlib.use("Agg")
# menjadi backend interaktif seperti "TkAgg" atau "QtAgg" (perlu paket GUI
# terpasang), lalu tambahkan `plt.show()` setelah `fig.savefig(...)` di
# dalam `_save_plot`. Untuk penggunaan normal di robot (headless, real-time),
# biarkan tetap "Agg" seperti sekarang agar tidak memblokir loop kontrol.