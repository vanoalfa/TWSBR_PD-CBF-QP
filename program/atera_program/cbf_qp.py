"""cbf_qp.py - safety filter CBF-QP ATERA (acados + HPIPM, N = 1).

    uQP = argmin_u (u - u_PD)^2
          s.t.  constraint CBF h1..h4,   -u_max <= u <= u_max

Barrier function (config.psi_max, config.dtheta_max):
    h1 = psi_max - psi            h2 = psi + psi_max           -> relative degree 2
    h3 = dtheta_max - dtheta      h4 = dtheta + dtheta_max     -> relative degree 1

h1, h2 - HOCBF orde-2 dengan alpha linear (Alpha_1, Alpha_2):
    L_f^2 h + L_g L_f h u + Alpha_1 L_f h + Alpha_2 (L_f h + Alpha_1 h) >= 0
h3, h4 - CBF orde-1 (L_g h != 0, jadi bentuk orde-2 tidak berlaku):
    L_f h + L_g h u + Alpha_1 h >= 0

Lie derivative, dengan x_dot = f(x) + g(x) u dari model.py dan x = [theta, dtheta, psi, dpsi]:
    h1: L_f h = -dpsi      L_f^2 h = -f4     L_g L_f h = -g4
    h2: L_f h = +dpsi      L_f^2 h = +f4     L_g L_f h = +g4
    h3: L_f h = -f2        L_g h   = -g2
    h4: L_f h = +f2        L_g h   = +g2

Keempat constraint affine terhadap u:  a_i u + b_i >= 0.  Tiap baris dibagi |a_i| supaya
slack-nya bersatuan N m (QP terskala baik), lalu (a_i, b_i) dikirim ke acados sebagai
parameter. Karena u skalar, satu iterasi SQP-RTI = tepat satu QP yang diselesaikan HPIPM.

Constraint dibuat soft (slack berbobot besar) supaya QP selalu punya solusi;
feasible = False berarti ada constraint yang terpaksa dilanggar.

Kontrak:  cbf_qp.filter(x, uPD) -> (uQP: float, feasible: bool)
"""

from __future__ import annotations

import logging
import math
import os
import time

import config
import model

LOGGER = logging.getLogger(__name__)

_HERE = os.path.dirname(os.path.abspath(__file__))
CODE_DIR = os.path.join(_HERE, "c_generated_code")
JSON_FILE = os.path.join(CODE_DIR, "acados_ocp_cbf.json")
SIGNATURE_FILE = os.path.join(CODE_DIR, "atera_signature.txt")
MODEL_NAME = "atera_cbf_qp"
_STRUCTURE_VERSION = "v1"       # naikkan bila struktur OCP di _build_ocp berubah
_BIG = 1.0e9
H_NAMES = ("h1 = psi_max - psi", "h2 = psi + psi_max",
           "h3 = dtheta_max - dtheta", "h4 = dtheta + dtheta_max")


def barrier_values(x) -> tuple:
    """h1..h4 pada state x."""
    dtheta, psi = float(x[1]), float(x[2])
    return (config.psi_max - psi, psi + config.psi_max,
            config.dtheta_max - dtheta, dtheta + config.dtheta_max)


def constraint_rows(x) -> tuple:
    """(a, b) tiap constraint, dengan arti a_i u + b_i >= 0 (belum dinormalkan)."""
    fx, gx = model.fg(x)
    dpsi = float(x[3])
    f2, f4, g2, g4 = fx[1], fx[3], gx[1], gx[3]
    a1, a2 = float(config.Alpha_1), float(config.Alpha_2)
    h1, h2, h3, h4 = barrier_values(x)
    # HOCBF: L_f^2 h + L_gL_f h u + Alpha_1 L_f h + Alpha_2 (L_f h + Alpha_1 h) >= 0
    b1 = -f4 + a1 * (-dpsi) + a2 * (-dpsi + a1 * h1)
    b2 = f4 + a1 * dpsi + a2 * (dpsi + a1 * h2)
    # CBF orde-1: L_f h + L_g h u + Alpha_1 h >= 0
    b3 = -f2 + a1 * h3
    b4 = f2 + a1 * h4
    return (-g4, g4, -g2, g2), (b1, b2, b3, b4)


def solve_analytic(a, b, u_ref: float, u_max: float) -> tuple:
    """Solusi tertutup QP 1 variabel. Kembalikan (u, feasible).

    Tiap baris a_i u + b_i >= 0 adalah batas bawah (a_i > 0) atau batas atas (a_i < 0) u.
    Bila himpunannya kosong: batas psi (baris 0, 1) dipenuhi dulu, batas dtheta sedekat mungkin.
    """
    def interval(rows):
        lo, hi = -math.inf, math.inf
        for i in rows:
            bound = -b[i] / a[i]
            if a[i] > 0.0:
                lo = max(lo, bound)
            else:
                hi = min(hi, bound)
        return lo, hi

    def clip(v, lo, hi):
        return max(lo, min(hi, v))

    feasible = True
    lo, hi = -u_max, u_max
    for rows in ((0, 1), (2, 3)):
        r_lo, r_hi = interval(rows)
        n_lo, n_hi = max(lo, r_lo), min(hi, r_hi)
        if n_lo <= n_hi:
            lo, hi = n_lo, n_hi
        else:
            feasible = False
            if r_lo > r_hi:                 # baris saling bertentangan: ambil tengahnya
                r_lo = r_hi = 0.5 * (r_lo + r_hi)
            point = clip(r_lo if r_lo > hi else r_hi, lo, hi)
            lo = hi = point
    return clip(u_ref, lo, hi), feasible


class CBFQP:
    def __init__(self) -> None:
        self.available = False
        self.reason = "CBF-QP belum di-setup."
        self.backend = ""
        self._solver = None
        self._np = None
        # hasil siklus terakhir (dibaca GUI dan ise.py)
        self.h = (0.0, 0.0, 0.0, 0.0)
        self.u_bound = (-config.u_max, config.u_max)   # selang u yang diizinkan CBF
        self.active = False
        self.feasible = True
        self.status = 0
        self.solve_ms = 0.0
        self.fail_count = 0

    # ------------------------------------------------------------------ setup --
    @staticmethod
    def check_config() -> str:
        """'' bila config.py lengkap untuk CBF-QP, selain itu alasannya."""
        for name in ("Alpha_1", "Alpha_2"):
            if float(getattr(config, name)) <= 0.0:
                return f"{name} harus > 0 di config.py (sekarang {getattr(config, name)})"
        if config.psi_max_DEG >= config.SAFE_TILT_DEG:
            return "psi_max_DEG harus lebih kecil dari SAFE_TILT_DEG"
        if int(config.CBF_N_HORIZON) != 1:
            return "CBF_N_HORIZON harus 1"
        return ""

    def setup(self) -> bool:
        """Siapkan solver. Panggil sekali saat start (kompilasi kode C bisa makan waktu)."""
        self.available = False
        self._solver = None
        self.reason = self.check_config()
        if self.reason:
            return False
        if str(config.CBF_SOLVER).strip().lower() != "acados":
            self.backend = "analitik"
            self.available = True
            self.reason = "siap (solusi analitik, tanpa acados)"
            return True
        try:
            self._setup_acados()
        except Exception as exc:
            self.reason = f"acados gagal di-setup: {exc}"
            LOGGER.error("CBF-QP %s", self.reason)
            return False
        self.backend = "acados/HPIPM"
        self.available = True
        LOGGER.info("CBF-QP %s", self.reason)
        return True

    def _build_ocp(self):
        import casadi as ca
        import numpy as np
        from acados_template import AcadosModel, AcadosOcp

        # State tiruan (acados butuh nx >= 1). State robot masuk lewat parameter p,
        # yaitu koefisien constraint yang sudah dihitung dari model.py.
        x = ca.SX.sym("x", 1)
        u = ca.SX.sym("u", 1)                 # torsi total [N m]
        p = ca.SX.sym("p", 8)                 # [a1..a4, b1..b4] ternormalisasi

        mdl = AcadosModel()
        mdl.name = MODEL_NAME
        mdl.x, mdl.u, mdl.p = x, u, p
        mdl.disc_dyn_expr = x
        mdl.con_h_expr_0 = p[0:4] * u[0] + p[4:8]      # a_i u + b_i >= 0

        ocp = AcadosOcp()
        ocp.model = mdl
        ocp.code_export_directory = CODE_DIR
        n_horizon = int(config.CBF_N_HORIZON)
        if hasattr(ocp.solver_options, "N_horizon"):
            ocp.solver_options.N_horizon = n_horizon
        else:                                           # acados versi lama
            ocp.dims.N = n_horizon
        ocp.solver_options.tf = 1.0 / float(config.CONTROL_HZ)
        ocp.parameter_values = np.array([1.0, -1.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0])

        # cost node 0: 1/2 * 2 * (u - u_PD)^2 = (u - u_PD)^2
        ocp.cost.cost_type_0 = "LINEAR_LS"
        ocp.cost.Vx_0 = np.zeros((1, 1))
        ocp.cost.Vu_0 = np.eye(1)
        ocp.cost.W_0 = 2.0 * np.eye(1)
        ocp.cost.yref_0 = np.zeros(1)
        # tidak ada cost di node lain
        ocp.cost.cost_type = "LINEAR_LS"
        ocp.cost.Vx = np.zeros((0, 1))
        ocp.cost.Vu = np.zeros((0, 1))
        ocp.cost.W = np.zeros((0, 0))
        ocp.cost.yref = np.zeros(0)
        ocp.cost.cost_type_e = "LINEAR_LS"
        ocp.cost.Vx_e = np.zeros((0, 1))
        ocp.cost.W_e = np.zeros((0, 0))
        ocp.cost.yref_e = np.zeros(0)

        # constraint CBF (soft)
        ocp.constraints.lh_0 = np.zeros(4)
        ocp.constraints.uh_0 = _BIG * np.ones(4)
        ocp.constraints.idxsh_0 = np.arange(4)
        w_psi = float(config.CBF_SLACK_WEIGHT_PSI)
        w_dth = float(config.CBF_SLACK_WEIGHT_DTHETA)
        weights = np.array([w_psi, w_psi, w_dth, w_dth])
        ocp.cost.zl_0 = weights            # penalti L1 (eksak selama bobot > pengali Lagrange)
        ocp.cost.zu_0 = weights
        ocp.cost.Zl_0 = weights            # penalti L2
        ocp.cost.Zu_0 = weights

        # batas aktuator dan state awal
        ocp.constraints.idxbu = np.array([0])
        ocp.constraints.lbu = np.array([-float(config.u_max)])
        ocp.constraints.ubu = np.array([float(config.u_max)])
        ocp.constraints.x0 = np.zeros(1)

        ocp.solver_options.integrator_type = "DISCRETE"
        ocp.solver_options.nlp_solver_type = "SQP_RTI"
        ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
        ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
        ocp.solver_options.print_level = 0
        return ocp

    def _setup_acados(self) -> None:
        import acados_template
        import numpy as np
        from acados_template import AcadosOcpSolver

        signature = "|".join(str(v) for v in (
            _STRUCTURE_VERSION, getattr(acados_template, "__version__", "?"),
            config.CBF_N_HORIZON, config.CBF_SLACK_WEIGHT_PSI, config.CBF_SLACK_WEIGHT_DTHETA,
        ))
        reuse = False
        if not config.CBF_FORCE_REBUILD and os.path.isfile(SIGNATURE_FILE) and os.path.isfile(JSON_FILE):
            with open(SIGNATURE_FILE) as handle:
                reuse = handle.read() == signature
        ocp = self._build_ocp()
        cwd = os.getcwd()
        os.chdir(_HERE)
        try:
            self._solver = AcadosOcpSolver(
                ocp, json_file=JSON_FILE, generate=not reuse, build=not reuse, verbose=False)
        finally:
            os.chdir(cwd)
        if not reuse:
            with open(SIGNATURE_FILE, "w") as handle:
                handle.write(signature)
        self._np = np
        self.reason = "siap (kode acados dipakai ulang)" if reuse else "siap (kode acados baru di-generate)"

    # --------------------------------------------------------------- runtime --
    def _solve_acados(self, a_n, b_n, u_ref: float, u_max: float) -> tuple:
        np = self._np
        solver = self._solver
        solver.set(0, "p", np.array(a_n + b_n, dtype=float))
        solver.constraints_set(0, "lbu", np.array([-u_max]))
        solver.constraints_set(0, "ubu", np.array([u_max]))
        solver.cost_set(0, "yref", np.array([u_ref]))
        solver.set(0, "u", np.array([u_ref]))
        status = int(solver.solve())
        u = float(solver.get(0, "u")[0])
        if status != 0 or not math.isfinite(u):
            raise RuntimeError(f"acados status {status}")
        slack = float(np.max(solver.get(0, "sl")))
        return u, slack <= float(config.CBF_SLACK_TOL)

    def filter(self, x, uPD) -> tuple:
        """Kembalikan (uQP, feasible). x = [theta, dtheta, psi, dpsi], uPD dan uQP dalam N m."""
        t0 = time.perf_counter()
        u_max = float(config.u_max)
        # Acuan = uPD yang sudah dijenuhkan ke batas aktuator. Untuk u skalar minimizer-nya
        # sama, dan bobot slack tetap lebih besar dari gradien cost.
        u_ref = max(-u_max, min(u_max, float(uPD)))
        self.h = barrier_values(x)

        if not self.available:
            self.active, self.feasible, self.status = False, False, -1
            return u_ref, False

        a, b = constraint_rows(x)
        a_n = [1.0 if ai > 0.0 else -1.0 for ai in a]
        b_n = [bi / abs(ai) for ai, bi in zip(a, b)]
        lower = max(-bi for ai, bi in zip(a_n, b_n) if ai > 0.0)
        upper = min(bi for ai, bi in zip(a_n, b_n) if ai < 0.0)
        self.u_bound = (lower, upper)

        self.status = 0
        if self._solver is not None:
            try:
                u, feasible = self._solve_acados(a_n, b_n, u_ref, u_max)
            except Exception as exc:
                # Jangan pernah meneruskan u_PD mentah saat solver gagal: pakai solusi tertutup.
                self.fail_count += 1
                self.status = 1
                if self.fail_count <= 3:
                    LOGGER.error("CBF-QP solve gagal (%s), pakai solusi analitik", exc)
                u, feasible = solve_analytic(a_n, b_n, u_ref, u_max)
        else:
            u, feasible = solve_analytic(a_n, b_n, u_ref, u_max)

        u = max(-u_max, min(u_max, u))
        self.active = abs(u - u_ref) > float(config.CBF_ACTIVE_TOL)
        self.feasible = bool(feasible)
        self.solve_ms = (time.perf_counter() - t0) * 1000.0
        return u, self.feasible


_default = CBFQP()


def setup() -> bool:
    return _default.setup()


def filter(x, uPD) -> tuple:
    """Kontrak: cbf_qp.filter(x, uPD) -> (uQP: float, feasible: bool)."""
    return _default.filter(x, uPD)