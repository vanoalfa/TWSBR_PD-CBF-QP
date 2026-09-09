"""Modul evaluasi performa balancing untuk keperluan skripsi.

Menghitung metrik ISE (Integral of Squared Error) dari error sudut
kemiringan badan robot (psi) terhadap target, menyimpan data mentah
ke CSV, dan membuat grafik respons (jika matplotlib tersedia).

Metrik yang dihitung:
- ISE  : ∫ e(t)² dt            (error = angle - target, dikuadratkan)
- IAE  : ∫ |e(t)| dt           (integral absolut error)
- ITAE : ∫ t·|e(t)| dt         (waktu × absolut error)
- RMSE : sqrt( (1/N) Σ e² )    (akar rata-rata kuadrat error)
- MAE  : (1/N) Σ |e|           (rata-rata absolut error)
- Error maksimum & rata-rata sudut
- Waktu settling kasar (pertama kali |e| < 2 deg dan bertahan 1 s)

Pemakaian (sudah terintegrasi di atera_main.py):
    evaluasi = EvaluasiISE()
    evaluasi.catat(t_rel=0.005, angle_deg=1.23, target_deg=0.0)
    ...
    evaluasi.tutup_dan_simpan()

File output (di folder tunning.EVAL_LOG_DIR):
    <prefix>_YYYYMMDD_HHMMSS.csv      -> data mentah per sampel
    <prefix>_YYYYMMDD_HHMMSS.png      -> grafik respons (opsional)
    <prefix>_YYYYMMDD_HHMMSS_ringkasan.txt -> ringkasan metrik teks
"""

from __future__ import annotations

import csv
import logging
import math
import os
import time
from typing import Dict, List, Optional

import tunning

LOGGER = logging.getLogger("atera.evaluasi")

# Ambang settling untuk metrik waktu settling kasar
_SETTLING_THRESHOLD_DEG = 2.0
_SETTLING_HOLD_S = 1.0


class EvaluasiISE:
    """Pencatat & penghitung metrik performa balancing (ISE dkk)."""

    def __init__(self) -> None:
        self._t0 = time.monotonic()
        self._ditutup = False

        # Buffer data mentah: list of dict
        self._data: List[Dict[str, float]] = []

        # Akumulator integral (trapezoidal)
        self._ise = 0.0
        self._iae = 0.0
        self._itae = 0.0
        self._sum_e2 = 0.0
        self._sum_abse = 0.0
        self._n = 0
        self._max_abs_e = 0.0
        self._t_sebelum: Optional[float] = None
        self._e_sebelum: Optional[float] = None

        # Deteksi settling kasar
        self._t_settling: Optional[float] = None
        self._t_masuk_band: Optional[float] = None

        # Siapkan folder output
        self._dir = str(tunning.EVAL_LOG_DIR)
        os.makedirs(self._dir, exist_ok=True)

        # Nama file berdasarkan timestamp
        ts = time.strftime("%Y%m%d_%H%M%S")
        prefix = str(tunning.EVAL_FILENAME_PREFIX)
        self._path_csv = os.path.join(self._dir, f"{prefix}_{ts}.csv")
        self._path_png = os.path.join(self._dir, f"{prefix}_{ts}.png")
        self._path_txt = os.path.join(self._dir, f"{prefix}_{ts}_ringkasan.txt")

        LOGGER.info("Evaluasi ISE dimulai. Output -> %s", self._path_csv)

    # ------------------------------------------------------------------
    def catat(self, t_rel: float, angle_deg: float, target_deg: float) -> None:
        """Catat satu sampel error. Dipanggil dari loop kontrol (200 Hz)."""
        if self._ditutup:
            return
        # Batasi durasi maksimum perekaman
        if t_rel > float(tunning.EVAL_MAX_DURATION_S):
            return

        e = float(angle_deg) - float(target_deg)
        abs_e = abs(e)
        t = float(t_rel)

        # Integrasi trapezoidal jika ada sampel sebelumnya
        if self._t_sebelum is not None:
            dt = t - self._t_sebelum
            if dt > 0.0 and self._e_sebelum is not None:
                e2_mid = 0.5 * (e * e + self._e_sebelum * self._e_sebelum)
                abs_mid = 0.5 * (abs_e + abs(self._e_sebelum))
                t_mid = 0.5 * (t + self._t_sebelum)
                self._ise += e2_mid * dt
                self._iae += abs_mid * dt
                self._itae += t_mid * abs_mid * dt

        self._sum_e2 += e * e
        self._sum_abse += abs_e
        self._n += 1
        if abs_e > self._max_abs_e:
            self._max_abs_e = abs_e

        # Settling kasar: pertama kali |e| < ambang dan bertahan >= hold
        if self._t_settling is None:
            if abs_e <= _SETTLING_THRESHOLD_DEG:
                if self._t_masuk_band is None:
                    self._t_masuk_band = t
                elif (t - self._t_masuk_band) >= _SETTLING_HOLD_S:
                    self._t_settling = self._t_masuk_band
            else:
                self._t_masuk_band = None

        self._data.append({
            "t_s": round(t, 4),
            "angle_deg": round(float(angle_deg), 4),
            "target_deg": round(float(target_deg), 4),
            "error_deg": round(e, 4),
        })
        self._t_sebelum = t
        self._e_sebelum = e

    # ------------------------------------------------------------------
    def ringkasan(self) -> Dict[str, float]:
        """Kembalikan dict metrik performa saat ini."""
        if self._n == 0:
            return {}
        rmse = math.sqrt(self._sum_e2 / self._n)
        mae = self._sum_abse / self._n
        return {
            "jumlah_sampel": float(self._n),
            "durasi_s": round(self._data[-1]["t_s"] if self._data else 0.0, 3),
            "ISE_deg2s": round(self._ise, 5),
            "IAE_degs": round(self._iae, 5),
            "ITAE_degs2": round(self._itae, 5),
            "RMSE_deg": round(rmse, 5),
            "MAE_deg": round(mae, 5),
            "error_maks_deg": round(self._max_abs_e, 4),
            "settling_s": (round(self._t_settling, 3)
                           if self._t_settling is not None else -1.0),
        }

    # ------------------------------------------------------------------
    def tutup_dan_simpan(self) -> None:
        """Tulis CSV, ringkasan teks, dan grafik (jika matplotlib ada)."""
        if self._ditutup:
            return
        self._ditutup = True

        if self._n == 0:
            LOGGER.warning("Evaluasi: tidak ada data terekam, tidak menyimpan file.")
            return

        self._simpan_csv()
        self._simpan_ringkasan_txt()
        self._simpan_grafik()

        ring = self.ringkasan()
        LOGGER.info(
            "Evaluasi selesai: ISE=%.4f deg^2.s, RMSE=%.4f deg, max|e|=%.3f deg, N=%d",
            ring.get("ISE_deg2s", 0.0), ring.get("RMSE_deg", 0.0),
            ring.get("error_maks_deg", 0.0), self._n,
        )

    # ------------------------------------------------------------------
    def _simpan_csv(self) -> None:
        try:
            with open(self._path_csv, "w", newline="") as f:
                writer = csv.DictWriter(
                    f, fieldnames=("t_s", "angle_deg", "target_deg", "error_deg"))
                writer.writeheader()
                writer.writerows(self._data)
            LOGGER.info("CSV evaluasi tersimpan: %s (%d sampel)",
                        self._path_csv, self._n)
        except OSError as exc:
            LOGGER.error("Gagal menulis CSV evaluasi: %s", exc)

    def _simpan_ringkasan_txt(self) -> None:
        try:
            ring = self.ringkasan()
            with open(self._path_txt, "w") as f:
                f.write("Ringkasan Evaluasi Balancing ATERA (PD + CBF-QP)\n")
                f.write("=" * 55 + "\n")
                f.write(f"Waktu rekam   : {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"CBF aktif     : {bool(tunning.CBF_ENABLED)}\n")
                f.write(f"PSI_MAX       : {tunning.CBF_PSI_MAX_DEG} deg\n")
                f.write(f"THETA_DOT_MAX : {tunning.CBF_THETA_DOT_MAX_DEG_S} deg/s\n")
                f.write("-" * 55 + "\n")
                for kunci, nilai in ring.items():
                    f.write(f"{kunci:>18s} : {nilai}\n")
                f.write("-" * 55 + "\n")
                f.write("Catatan: settling_s = -1 berarti tidak pernah settle\n")
                f.write(f"dalam ambang {_SETTLING_THRESHOLD_DEG} deg "
                        f"selama {_SETTLING_HOLD_S} s.\n")
            LOGGER.info("Ringkasan teks tersimpan: %s", self._path_txt)
        except OSError as exc:
            LOGGER.error("Gagal menulis ringkasan evaluasi: %s", exc)

    def _simpan_grafik(self) -> None:
        try:
            import matplotlib
            matplotlib.use("Agg")  # backend tanpa display (headless di Pi)
            import matplotlib.pyplot as plt
        except Exception:  # noqa: BLE001
            LOGGER.info("matplotlib tidak tersedia; grafik dilewati.")
            return

        try:
            t = [d["t_s"] for d in self._data]
            sudut = [d["angle_deg"] for d in self._data]
            target = [d["target_deg"] for d in self._data]
            error = [d["error_deg"] for d in self._data]
            ring = self.ringkasan()

            fig, ax = plt.subplots(2, 1, figsize=(9, 6), sharex=True)

            # Subplot 1: sudut vs target + batas barrier CBF
            ax[0].plot(t, sudut, lw=1.0, label=r"$\psi$ (aktual)")
            ax[0].plot(t, target, lw=0.9, ls="--", label="Target")
            psi_max = float(tunning.CBF_PSI_MAX_DEG)
            ax[0].axhline(psi_max, color="r", ls=":", lw=0.9,
                          label=f"Batas CBF ±{psi_max:.0f}°")
            ax[0].axhline(-psi_max, color="r", ls=":", lw=0.9)
            ax[0].set_ylabel("Sudut (deg)")
            ax[0].set_title(
                "Respons Balancing ATERA — PD + CBF-QP\n"
                f"ISE={ring.get('ISE_deg2s', 0):.3f} deg²·s, "
                f"RMSE={ring.get('RMSE_deg', 0):.3f} deg")
            ax[0].legend(loc="upper right", fontsize=8)
            ax[0].grid(True, alpha=0.3)

            # Subplot 2: error
            ax[1].plot(t, error, lw=0.9, color="tab:orange")
            ax[1].axhline(0.0, color="k", lw=0.7)
            ax[1].set_xlabel("Waktu (s)")
            ax[1].set_ylabel("Error (deg)")
            ax[1].grid(True, alpha=0.3)

            fig.tight_layout()
            fig.savefig(self._path_png, dpi=150)
            LOGGER.info("Grafik evaluasi tersimpan: %s", self._path_png)
        except Exception as exc:  # noqa: BLE001
            LOGGER.error("Gagal membuat grafik evaluasi: %s", exc)
