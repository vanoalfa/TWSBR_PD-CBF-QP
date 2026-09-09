"""robot_model.py -- MODEL DINAMIKA TWSBR "ATERA" UNTUK CBF-QP.

Modul ini mendefinisikan model matematis robot dua roda self-balancing
yang dipakai untuk merumuskan kondisi HOCBF (High Order Control Barrier
Function) orde-2 di dalam QP.

STATE (3):
    x[0] = psi        : sudut kemiringan badan robot dari tegak (rad)
                        -> diukur dari MPU6050 + Kalman filter
    x[1] = psi_dot    : laju sudut kemiringan badan (rad/s)
                        -> diukur dari gyro MPU6050
    x[2] = theta_dot  : kecepatan sudut roda rata-rata L & R (rad/s)
                        -> dihitung dari feedback speed_rpm DDSM115

    Catatan: posisi roda absolut (theta) tidak dimasukkan sebagai state
    karena barrier hanya bergantung pada kecepatan roda (theta_dot).
    Ini membuat state minimal & sesuai sensor yang tersedia.

INPUT (2):
    u[0] = I_L : arus motor kiri  (Ampere)
    u[1] = I_R : arus motor kanan (Ampere)

MODEL TERLINEARISASI di sekitar posisi tegak (psi ~= 0):

    psi_ddot   = A1*psi + A2*psi_dot + A3*theta_dot + B1*(I_L + I_R)
    theta_ddot = A4*psi + A5*psi_dot + A6*theta_dot + B2*(I_L + I_R)

Koefisien A1..A6, B1, B2 dihitung otomatis dari parameter fisik robot
(massa, inersia, geometri, konstanta motor) yang Anda isi di tunning.py.
"""

from __future__ import annotations

import numpy as np
import casadi as ca

import tunning


# ============================================================================
# 1. Perhitungan koefisien model linear dari parameter fisik
# ============================================================================
def hitung_linear_coefficients() -> dict:
    """Menghitung koefisien A1..A6, B1, B2 dari parameter fisik di tunning.py.

    Penurunan (ringkasan Euler-Lagrange terlinearisasi untuk wheel + pendulum):

        M11 = m_b*l^2 + J_b                    (inersia efektif pendulum)
        M22 = J_w + (m_b + 2*m_w)*r^2          (inersia efektif roda)
        M12 = m_b*l*r                          (kopling pendulum-roda)

        persamaan:
            [M11  M12] [psi_ddot  ]   [m_b*g*l*psi - b_p*psi_dot          ]
            [M12  M22] [theta_ddot] = [tau_roda     - b_w*theta_dot        ]

        tau_roda = 2 * GEAR_RATIO * Kt * (I_L + I_R) / r  (per satuan roda)

    Setelah inversi matriks massa M, diperoleh bentuk:
        psi_ddot   = A1*psi + A2*psi_dot + A3*theta_dot + B1*(I_L+I_R)
        theta_ddot = A4*psi + A5*psi_dot + A6*theta_dot + B2*(I_L+I_R)
    """
    # --- Ambil parameter fisik dari tunning.py ---
    m_b = float(tunning.MASS_BODY_KG)
    m_w = float(tunning.MASS_WHEEL_KG)
    l = float(tunning.LENGTH_COM_M)
    r = float(tunning.WHEEL_RADIUS_M)
    J_b = float(tunning.INERTIA_BODY_KGM2)
    J_w = float(tunning.INERTIA_WHEEL_KGM2)
    g = float(tunning.GRAVITY_M_S2)
    Kt = float(tunning.TORQUE_CONSTANT_NM_PER_A)
    gr = float(tunning.GEAR_RATIO)
    b_p = float(tunning.DAMPING_PENDULUM_NMS)
    b_w = float(tunning.DAMPING_WHEEL_NMS)

    # --- Matriks massa (2x2) ---
    M11 = m_b * l * l + J_b
    M22 = J_w + (m_b + 2.0 * m_w) * r * r
    M12 = m_b * l * r

    M = np.array([[M11, M12],
                  [M12, M22]], dtype=float)

    det_M = float(M11 * M22 - M12 * M12)
    if abs(det_M) < 1e-12:
        raise ValueError(
            "Matriks massa singular! Periksa parameter fisik di tunning.py "
            "(massa/inersia/geometri masih placeholder 0.0?)."
        )
    M_inv = np.linalg.inv(M)

    # --- Sisi kanan persamaan (linearisasi) ---
    # Baris 1 (pendulum):  m_b*g*l*psi - b_p*psi_dot  + 0*theta_dot + 0*u
    # Baris 2 (roda)    :  0*psi + 0*psi_dot - b_w*theta_dot + G_roda*(I_L+I_R)
    G_roda = 2.0 * gr * Kt / max(r, 1e-9)     # gain arus->gaya pada kontak roda

    # Koefisien hasil perkalian M_inv dengan vektor sisi kanan
    # psi_ddot   = M_inv[0,0]*(m_b*g*l*psi - b_p*psi_dot) + M_inv[0,1]*(G_roda*(I_L+I_R) - b_w*theta_dot)
    # theta_ddot = M_inv[1,0]*(m_b*g*l*psi - b_p*psi_dot) + M_inv[1,1]*(G_roda*(I_L+I_R) - b_w*theta_dot)

    coeff = {}
    coeff["A1"] = float(M_inv[0, 0] * (m_b * g * l))
    coeff["A2"] = float(M_inv[0, 0] * (-b_p))
    coeff["A3"] = float(M_inv[0, 1] * (-b_w))
    coeff["B1"] = float(M_inv[0, 1] * G_roda)

    coeff["A4"] = float(M_inv[1, 0] * (m_b * g * l))
    coeff["A5"] = float(M_inv[1, 0] * (-b_p))
    coeff["A6"] = float(M_inv[1, 1] * (-b_w))
    coeff["B2"] = float(M_inv[1, 1] * G_roda)

    return coeff


# ============================================================================
# 2. Ekspor model CasADi untuk acados
# ============================================================================
def export_model_atera():
    """Membangun model CasADi (explicit ODE) untuk acados.

    Model dinamika kontinu:
        psi_dot    = x1
        psi_ddot   = A1*x0 + A2*x1 + A3*x2 + B1*(u0+u1)
        theta_ddot = A4*x0 + A5*x1 + A6*x2 + B2*(u0+u1)

    Mengembalikan objek AcadosModel yang siap dipakai oleh AcadosOcp.
    """
    from acados_template import AcadosModel

    # --- State ---
    psi = ca.SX.sym("psi")
    psi_dot = ca.SX.sym("psi_dot")
    theta_dot = ca.SX.sym("theta_dot")
    x = ca.vertcat(psi, psi_dot, theta_dot)

    # --- Input ---
    I_L = ca.SX.sym("I_L")
    I_R = ca.SX.sym("I_R")
    u = ca.vertcat(I_L, I_R)

    # --- Turunan state (xdot) ---
    xdot = ca.SX.sym("xdot", 3)

    # --- Koefisien model dari parameter fisik ---
    coeff = hitung_linear_coefficients()
    A1, A2, A3, B1 = coeff["A1"], coeff["A2"], coeff["A3"], coeff["B1"]
    A4, A5, A6, B2 = coeff["A4"], coeff["A5"], coeff["A6"], coeff["B2"]

    # --- Dinamika eksplisit f(x,u) ---
    psi_ddot = A1 * psi + A2 * psi_dot + A3 * theta_dot + B1 * (I_L + I_R)
    theta_ddot_expr = A4 * psi + A5 * psi_dot + A6 * theta_dot + B2 * (I_L + I_R)

    f_expl = ca.vertcat(psi_dot, psi_ddot, theta_ddot_expr)

    # --- Kemas ke AcadosModel ---
    model = AcadosModel()
    model.name = "atera_twsbr"
    model.x = x
    model.u = u
    model.xdot = xdot
    model.f_expl_expr = f_expl
    # f_impl_expr untuk acados: xdot - f_expl = 0
    model.f_impl_expr = xdot - f_expl

    return model


# ============================================================================
# 3. Fungsi bantu: hitung theta_dot rata-rata dari feedback motor
# ============================================================================
def theta_dot_dari_feedback(left_speed_rpm: float, right_speed_rpm: float) -> float:
    """Mengubah feedback kecepatan motor (rpm) menjadi theta_dot (rad/s).

    theta_dot = rata-rata kecepatan sudut roda kiri & kanan.
    Konversi: rad/s = rpm * (2*pi / 60).
    """
    rpm_avg = 0.5 * (float(left_speed_rpm) + float(right_speed_rpm))
    return rpm_avg * (2.0 * np.pi / 60.0)


# ============================================================================
# 4. Uji cepat manual (python robot_model.py)
# ============================================================================
if __name__ == "__main__":
    print("=" * 60)
    print("UJI MODEL ROBOT ATERA")
    print("=" * 60)
    try:
        coeff = hitung_linear_coefficients()
        print("Koefisien model linear (dari parameter fisik tunning.py):")
        for k in ("A1", "A2", "A3", "B1", "A4", "A5", "A6", "B2"):
            print(f"  {k} = {coeff[k]:+.6f}")
    except ValueError as exc:
        print(f"[PERINGATAN] {exc}")
        print("Isi dulu parameter fisik di tunning.py bagian 3!")
