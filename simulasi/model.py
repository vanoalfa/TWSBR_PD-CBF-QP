"""model.py - model dinamika TWSBR ATERA (tanpa kontrol).

Bentuk laporan (Bab 3, bidang miring; bidang datar = zeta 0):

    M(q)*ddq + C(q, dq)*dq + G(q) = tau_q,      q = [theta, psi]

    [ 2A          C(psi,zeta) ] [ddtheta]   [ -MRL*sin(psi+zeta)*dpsi^2 ]   [  D            ]   [  2u ]
    [ C(psi,zeta) B           ] [ddpsi  ] + [  0                        ] + [ -MgL*sin(psi) ] = [ -2u ]

    A           = m_r*R^2 + 1/2*m_b*R^2 + j_theta
    B           = m_b*L^2 + j_psi
    C(psi,zeta) = m_b*R*L*cos(psi + zeta)
    D           = (m_b + 2*m_r)*g*R*sin(zeta)

Bentuk control-affine untuk CBF-QP:   dx/dt = f(x) + g(x)*u
    x = [theta, dtheta, psi, dpsi]      u = torsi motor SATU roda [N m], torsi total = 2u (N_WHEEL*u)

Semua nilai dibaca dari config.py.
"""

from __future__ import annotations

import math

import numpy as np

import config


def _zeta(zeta):
    """zeta dari config.py bila None. zeta boleh angka, atau simbol CasADi (dipakai cbf_qp.py)."""
    if zeta is None:
        return float(config.zeta)
    if isinstance(zeta, (int, float)):
        return float(zeta)
    return zeta


def coefficients(zeta=None, sin=math.sin):
    """Koefisien bantu laporan: (A, B, MRL, D, MgL), dengan C(psi,zeta) = MRL*cos(psi + zeta)."""
    z = _zeta(zeta)
    A = config.m_r * config.R ** 2 + 0.5 * config.m_b * config.R ** 2 + config.j_theta
    B = config.m_b * config.L ** 2 + config.j_psi
    MRL = config.m_b * config.R * config.L
    D = (config.m_b + 2.0 * config.m_r) * config.g * config.R * sin(z)
    MgL = config.m_b * config.g * config.L
    return A, B, MRL, D, MgL


def _fg(x, zeta, sin, cos):
    """Komponen f(x) dan g(x); dipakai untuk float (math) maupun simbolik (casadi)."""
    dtheta = x[config.IDX_DTHETA]
    psi = x[config.IDX_PSI]
    dpsi = x[config.IDX_DPSI]

    A, B, MRL, D, MgL = coefficients(zeta, sin)
    C = MRL * cos(psi + zeta)
    det = 2.0 * A * B - C * C

    # 2A*ddtheta + C*ddpsi = r_theta + N_WHEEL*u
    #  C*ddtheta + B*ddpsi = r_psi   - N_WHEEL*u
    n = float(config.N_WHEEL)
    r_theta = MRL * sin(psi + zeta) * dpsi * dpsi - D
    r_psi = MgL * sin(psi)

    f = (dtheta,
         (B * r_theta - C * r_psi) / det,
         dpsi,
         (2.0 * A * r_psi - C * r_theta) / det)
    g = (0.0,
         n * (B + C) / det,
         0.0,
         -n * (2.0 * A + C) / det)
    return f, g


def f(x, zeta=None) -> np.ndarray:
    """Drift f(x), ndarray(4)."""
    fx, _ = _fg(np.asarray(x, dtype=float), _zeta(zeta), math.sin, math.cos)
    return np.array(fx, dtype=float)


def g(x, zeta=None) -> np.ndarray:
    """Vektor input g(x), ndarray(4)."""
    _, gx = _fg(np.asarray(x, dtype=float), _zeta(zeta), math.sin, math.cos)
    return np.array(gx, dtype=float)


def dynamics(x, u: float, zeta=None) -> np.ndarray:
    """dx/dt = f(x) + g(x)*u, ndarray(4)."""
    fx, gx = _fg(np.asarray(x, dtype=float), _zeta(zeta), math.sin, math.cos)
    return np.array(fx, dtype=float) + np.array(gx, dtype=float) * float(u)


def fg_casadi(x, zeta=None):
    """f(x) dan g(x) simbolik (CasADi 4x1) untuk x simbolik; dipakai cbf_qp.py (Lie derivative).
    zeta boleh angka atau simbol CasADi (cbf_qp.py memakai simbol supaya zeta bisa diubah saat berjalan)."""
    import casadi as ca

    fx, gx = _fg([x[i] for i in range(config.NX)], _zeta(zeta), ca.sin, ca.cos)
    return ca.vertcat(*fx), ca.vertcat(*gx)


def equilibrium(zeta=None):
    """Titik setimbang diam di bidang miring zeta: (x_eq ndarray(4), u_eq float).

    u_eq = D / N_WHEEL,   sin(psi_eq) = D / MgL
    """
    z = _zeta(zeta)
    _, _, _, D, MgL = coefficients(z)
    s = D / MgL
    if abs(s) > 1.0:
        raise ValueError(f"Tidak ada titik setimbang untuk zeta = {math.degrees(z):g} deg")
    x_eq = np.zeros(config.NX)
    x_eq[config.IDX_PSI] = math.asin(s)
    return x_eq, float(D / config.N_WHEEL)


def linearize(zeta=None):
    """Linearisasi di titik setimbang: d(x - x_eq)/dt = A_lin*(x - x_eq) + B_lin*(u - u_eq)."""
    z = _zeta(zeta)
    x_eq, _ = equilibrium(z)
    psi_eq = x_eq[config.IDX_PSI]

    A, B, MRL, _, MgL = coefficients(z)
    C = MRL * math.cos(psi_eq + z)
    det = 2.0 * A * B - C * C
    k_psi = MgL * math.cos(psi_eq)

    A_lin = np.zeros((config.NX, config.NX))
    A_lin[config.IDX_THETA, config.IDX_DTHETA] = 1.0
    A_lin[config.IDX_DTHETA, config.IDX_PSI] = -C * k_psi / det
    A_lin[config.IDX_PSI, config.IDX_DPSI] = 1.0
    A_lin[config.IDX_DPSI, config.IDX_PSI] = 2.0 * A * k_psi / det

    B_lin = np.zeros(config.NX)
    B_lin[config.IDX_DTHETA] = config.N_WHEEL * (B + C) / det
    B_lin[config.IDX_DPSI] = -config.N_WHEEL * (2.0 * A + C) / det
    return A_lin, B_lin


if __name__ == "__main__":
    np.set_printoptions(precision=4, suppress=True)
    for zeta_deg in (0.0, 8.0):
        z = math.radians(zeta_deg)
        A, B, MRL, D, MgL = coefficients(z)
        x_eq, u_eq = equilibrium(z)
        A_lin, B_lin = linearize(z)
        print(f"zeta = {zeta_deg:g} deg | 2A = {2 * A:.6f} | B = {B:.6f} | MRL = {MRL:.6f} | "
              f"D = {D:.4f} | MgL = {MgL:.4f}")
        print(f"  psi_eq = {math.degrees(x_eq[config.IDX_PSI]):+.3f} deg | u_eq = {u_eq:+.4f} N m | "
              f"residu = {np.abs(dynamics(x_eq, u_eq, z)).max():.1e}")
        print("  A_lin =\n", A_lin)
        print("  B_lin =", B_lin)
        print("  pole open-loop =", np.linalg.eigvals(A_lin))