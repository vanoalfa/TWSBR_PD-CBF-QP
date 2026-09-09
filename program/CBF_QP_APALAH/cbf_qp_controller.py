"""cbf_qp_controller.py -- SAFETY FILTER CBF-QP (acados + HPIPM, N=1).

Kelas CbfQpController menjembatani PD controller nominal dengan motor DDSM115:

    PD nominal (ternormalisasi -1..1)
        -> dikonversi ke arus (Ampere)
        -> CBF-QP (HPIPM) memodifikasi seminimal mungkin agar aman
        -> output arus optimal [I_L, I_R] (Ampere)
        -> dikonversi balik ke ternormalisasi untuk dikirim ke motor

Pemakaian:
    cbf = CbfQpController()          # muat solver hasil build_solver.py
    hasil = cbf.compute(
        psi_rad, psi_dot_rad_s, theta_dot_rad_s,
        u_pd_left, u_pd_right        # output PD ternormalisasi -1..1
    )
    # hasil.left_normalized / hasil.right_normalized -> ke command_normalized()
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

import tunning


# ============================================================================
# Struktur data hasil CBF
# ============================================================================
@dataclass
class CbfQpState:
    """Hasil satu langkah CBF-QP untuk monitoring & logging."""
    # Input nominal dari PD (sudah dalam Ampere)
    u_pd_left_a: float = 0.0
    u_pd_right_a: float = 0.0
    # Output optimal hasil QP (Ampere)
    u_opt_left_a: float = 0.0
    u_opt_right_a: float = 0.0
    # Output ternormalisasi -1..1 (untuk dikirim ke motor)
    left_normalized: float = 0.0
    right_normalized: float = 0.0
    # Nilai 4 barrier function h1..h4
    h1: float = 0.0
    h2: float = 0.0
    h3: float = 0.0
    h4: float = 0.0
    # Slack tiap constraint (besar = constraint sedang aktif/dilanggar PD)
    slack: tuple = (0.0, 0.0, 0.0, 0.0)
    # Waktu solve QP (milidetik)
    solve_time_ms: float = 0.0
    # Status solver (0 = sukses)
    solver_status: int = -1
    # True jika safety filter memodifikasi output PD
    filter_aktif: bool = False


# ============================================================================
# Kelas utama CBF-QP
# ============================================================================
class CbfQpController:
    """Safety filter CBF-QP berbasis acados/HPIPM (N=1)."""

    def __init__(self) -> None:
        self._solver = None
        self._siap = False
        self._psi_max_rad = float(np.deg2rad(tunning.PSI_MAX_DEG))
        self._thdot_max_rad = float(np.deg2rad(tunning.THETA_DOT_MAX_DEG_S))
        self._max_current = float(tunning.CBF_U_MAX_A)
        self._err_msg = ""

        # Coba muat solver hasil generate
        self._muat_solver()

    # ------------------------------------------------------------------
    def _muat_solver(self) -> None:
        """Memuat solver acados dari folder hasil build_solver.py."""
        try:
            from acados_template import AcadosOcpSolver
            json_file = f"{tunning.CBF_SOLVER_FOLDER}.json"
            self._solver = AcadosOcpSolver(
                ocp=None,
                json_file=json_file,
                generate=False,
                build=False,
            )
            self._siap = True
        except Exception as exc:  # noqa: BLE001
            self._solver = None
            self._siap = False
            self._err_msg = (
                f"Gagal memuat solver CBF dari '{json_file}': {exc}. "
                "Jalankan 'python3 build_solver.py' dulu."
            )

    # ------------------------------------------------------------------
    @property
    def siap(self) -> bool:
        return self._siap

    @property
    def pesan_error(self) -> str:
        return self._err_msg

    # ------------------------------------------------------------------
    def _hitung_barrier(self, psi: float, theta_dot: float) -> tuple:
        """Menghitung nilai 4 barrier function untuk monitoring."""
        h1 = -psi + self._psi_max_rad
        h2 = psi + self._psi_max_rad
        h3 = -theta_dot + self._thdot_max_rad
        h4 = theta_dot + self._thdot_max_rad
        return h1, h2, h3, h4

    # ------------------------------------------------------------------
    def compute(
        self,
        psi_rad: float,
        psi_dot_rad_s: float,
        theta_dot_rad_s: float,
        u_pd_left_norm: float,
        u_pd_right_norm: float,
    ) -> CbfQpState:
        """Hitung output aman via CBF-QP.

        Parameter
        ---------
        psi_rad          : sudut kemiringan badan (rad)
        psi_dot_rad_s    : laju sudut badan (rad/s)
        theta_dot_rad_s  : kecepatan sudut roda rata-rata (rad/s)
        u_pd_left_norm   : output PD kiri, ternormalisasi -1..1
        u_pd_right_norm  : output PD kanan, ternormalisasi -1..1

        Return
        ------
        CbfQpState berisi arus optimal & info monitoring.
        """
        # Konversi output PD ternormalisasi -> Ampere
        u_pd_left_a = float(np.clip(u_pd_left_norm, -1.0, 1.0)) * self._max_current
        u_pd_right_a = float(np.clip(u_pd_right_norm, -1.0, 1.0)) * self._max_current

        hasil = CbfQpState(
            u_pd_left_a=u_pd_left_a,
            u_pd_right_a=u_pd_right_a,
        )

        # Nilai barrier untuk monitoring (selalu dihitung)
        h1, h2, h3, h4 = self._hitung_barrier(psi_rad, theta_dot_rad_s)
        hasil.h1, hasil.h2, hasil.h3, hasil.h4 = h1, h2, h3, h4

        # Jika solver tidak siap, fallback: lewatkan PD apa adanya
        if not self._siap:
            hasil.u_opt_left_a = u_pd_left_a
            hasil.u_opt_right_a = u_pd_right_a
            hasil.left_normalized = float(np.clip(u_pd_left_norm, -1.0, 1.0))
            hasil.right_normalized = float(np.clip(u_pd_right_norm, -1.0, 1.0))
            hasil.solver_status = -1
            return hasil

        # --------------------------------------------------------------
        # Set kondisi awal state x0 = [psi, psi_dot, theta_dot]
        # --------------------------------------------------------------
        x0 = np.array([psi_rad, psi_dot_rad_s, theta_dot_rad_s], dtype=float)
        self._solver.set(0, "x", x0)

        # --------------------------------------------------------------
        # Set parameter online u_pd (Ampere)
        # --------------------------------------------------------------
        p_val = np.array([u_pd_left_a, u_pd_right_a], dtype=float)
        self._solver.set_p(0, p_val)

        # --------------------------------------------------------------
        # Solve QP
        # --------------------------------------------------------------
        t0 = time.perf_counter()
        status = self._solver.solve()
        t1 = time.perf_counter()

        hasil.solver_status = int(status)
        hasil.solve_time_ms = (t1 - t0) * 1000.0

        # --------------------------------------------------------------
        # Ambil solusi input optimal u* = [I_L, I_R] pada node 0
        # --------------------------------------------------------------
        if status == 0:
            u_opt = self._solver.get(0, "u")
            hasil.u_opt_left_a = float(u_opt[0])
            hasil.u_opt_right_a = float(u_opt[1])
            # Ambil slack jika tersedia (untuk monitoring)
            try:
                slack = self._solver.get(0, "sl")
                if slack is not None and len(slack) >= 4:
                    hasil.slack = tuple(float(s) for s in slack[:4])
            except Exception:  # noqa: BLE001
                pass
        else:
            # Solver gagal -> fallback aman: pakai PD apa adanya
            hasil.u_opt_left_a = u_pd_left_a
            hasil.u_opt_right_a = u_pd_right_a

        # --------------------------------------------------------------
        # Konversi balik ke ternormalisasi untuk motor
        # --------------------------------------------------------------
        if abs(self._max_current) > 1e-9:
            hasil.left_normalized = float(
                np.clip(hasil.u_opt_left_a / self._max_current, -1.0, 1.0)
            )
            hasil.right_normalized = float(
                np.clip(hasil.u_opt_right_a / self._max_current, -1.0, 1.0)
            )
        else:
            hasil.left_normalized = 0.0
            hasil.right_normalized = 0.0

        # Deteksi apakah safety filter mengubah output secara berarti
        deviasi = max(
            abs(hasil.u_opt_left_a - u_pd_left_a),
            abs(hasil.u_opt_right_a - u_pd_right_a),
        )
        hasil.filter_aktif = deviasi > 1e-3

        return hasil


# ============================================================================
# Uji cepat manual (python cbf_qp_controller.py)
# ============================================================================
if __name__ == "__main__":
    print("=" * 64)
    print(" UJI CBF-QP CONTROLLER (tanpa hardware)")
    print("=" * 64)
    cbf = CbfQpController()
    if not cbf.siap:
        print(f"[PERINGATAN] {cbf.pesan_error}")
        print("Jalankan 'python3 build_solver.py' dulu untuk generate solver.\n")
    else:
        print("[OK] Solver berhasil dimuat.\n")
        # Skenario: robot miring 10 deg (mendekati batas 15 deg), PD minta besar
        psi = float(np.deg2rad(10.0))
        psi_dot = float(np.deg2rad(20.0))
        theta_dot = float(np.deg2rad(5.0))
        hasil = cbf.compute(psi, psi_dot, theta_dot, 0.8, 0.8)
        print("Skenario: psi=10deg, psi_dot=20deg/s, theta_dot=5deg/s, u_PD=0.8")
        print(f"  u_PD     (A) : L={hasil.u_pd_left_a:+.3f}  R={hasil.u_pd_right_a:+.3f}")
        print(f"  u_opt CBF(A) : L={hasil.u_opt_left_a:+.3f}  R={hasil.u_opt_right_a:+.3f}")
        print(f"  barrier h1..h4: {hasil.h1:+.4f} {hasil.h2:+.4f} "
              f"{hasil.h3:+.4f} {hasil.h4:+.4f}")
        print(f"  slack        : {['%.2e' % s for s in hasil.slack]}")
        print(f"  solve time   : {hasil.solve_time_ms:.3f} ms | status={hasil.solver_status}")
        print(f"  filter aktif : {hasil.filter_aktif}")
