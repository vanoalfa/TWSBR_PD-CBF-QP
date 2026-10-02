"""CBF-QP safety filter for ATERA (acados + HPIPM, horizon N = 1).

    u_safe = argmin_u  1/2 (u - u_PD)^2
             s.t.      CBF conditions of h1..h4,   u_min <= u <= u_max

(u_PD is first saturated to the actuator limit; the CBF conditions are soft with a
large penalty so that the QP stays feasible, the tilt conditions have priority.)

Model (from the Lagrangian of the thesis, q = [theta, psi]):

    [2A  C] [theta_dd]   [-M R L sin(psi+zeta) psi_d^2]   [      D        ]   [ u]
    [C   B] [ psi_dd ] + [             0              ] + [-M g L sin(psi)] = [-u]

    A = m R^2 + 1/2 M R^2 + j_omega      B = M L^2 + j_psi
    C = M R L cos(psi + zeta)            D = (M + 2 m) g R sin(zeta)
    u = torque_left + torque_right  [N m]

    NOTE: the gravity term of the psi row is -M g L sin(psi). This is what
    d/dt(dL/d psi_dot) - dL/d psi gives for the Lagrangian with -M g L cos(psi)
    (inverted pendulum: gravity pushes the body AWAY from psi = 0).

Barrier functions:

    h1 =  psi_max - psi          h2 = psi + psi_max           (relative degree 2 -> HOCBF)
    h3 =  theta_dot_max - theta_dot    h4 = theta_dot + theta_dot_max   (relative degree 1)

    h1, h2:  Psi_1 = h_dot + Alpha_1 h,   Psi_1_dot + Alpha_2 Psi_1 >= 0
             <=>  h_ddot + (Alpha_1 + Alpha_2) h_dot + Alpha_1 Alpha_2 h >= 0
    h3, h4:  h_dot + Alpha_3 h >= 0

All four conditions are affine in u, so the problem is a QP. It is solved by one
SQP-RTI step of acados (= exactly one HPIPM QP solve) on a horizon of N = 1.

Units: this module works in SI (rad, rad/s, N m). The public method `filter()`
takes and returns the same units as the PD (deg, deg/s, raw current count).
"""

from __future__ import annotations

import logging
import math
import os
from dataclasses import dataclass
from typing import Optional, Tuple

import config
from ddsm115 import RAW_PER_AMP, u_limit_raw

LOGGER = logging.getLogger(__name__)

_HERE = os.path.dirname(os.path.abspath(__file__))
CODE_DIR = os.path.join(_HERE, "c_generated_code_cbf")
JSON_FILE = os.path.join(CODE_DIR, "acados_ocp_cbf.json")
SIGNATURE_FILE = os.path.join(CODE_DIR, "atera_signature.txt")
MODEL_NAME = "atera_cbf_qp"

# Order of the runtime parameter vector p of the acados model.
P_NAMES = ("A", "B", "MRL", "MgL", "D", "zeta", "psi_max", "theta_dot_max", "alpha_1", "alpha_2", "alpha_3")

_REQUIRED = (
    "CBF_M_BODY_KG", "CBF_M_WHEEL_KG", "CBF_R_WHEEL_M", "CBF_L_COM_M",
    "CBF_J_WHEEL_KGM2", "CBF_J_BODY_KGM2", "CBF_TORQUE_CONSTANT_NM_PER_A",
    "CBF_PSI_MAX_DEG", "CBF_THETA_DOT_MAX_DEG_S",
)


@dataclass
class CBFResult:
    u_safe: float            # filtered command [raw current count per wheel]
    u_PD: float              # nominal command [raw current count per wheel]
    active: bool             # True if the filter changed the (saturated) PD command
    status: int              # acados status (0 = OK), -1 = fallback to u_PD
    solve_time_ms: float     # acados time_tot of this cycle
    h: Tuple[float, float, float, float]   # h1..h4 (rad, rad, rad/s, rad/s)


def model_coefficients() -> dict:
    """Auxiliary coefficients A, B, MRL, MgL, D of the thesis from config.py."""
    M = float(config.CBF_M_BODY_KG)
    m = float(config.CBF_M_WHEEL_KG)
    R = float(config.CBF_R_WHEEL_M)
    L = float(config.CBF_L_COM_M)
    j_w = float(config.CBF_J_WHEEL_KGM2)
    j_psi = float(config.CBF_J_BODY_KGM2)
    g = float(config.CBF_GRAVITY_M_S2)
    zeta = math.radians(float(config.CBF_ZETA_DEG))
    return {
        "A": m * R * R + 0.5 * M * R * R + j_w,
        "B": M * L * L + j_psi,
        "MRL": M * R * L,
        "MgL": M * g * L,
        "D": (M + 2.0 * m) * g * R * math.sin(zeta),
        "zeta": zeta,
    }


def accelerations(x, u, p, cos, sin):
    """theta_dd, psi_dd of the model. Works with casadi symbols and with floats."""
    psi, psi_d = x[1], x[3]
    A, B, MRL, MgL, D, zeta = p[0], p[1], p[2], p[3], p[4], p[5]
    C = MRL * cos(psi + zeta)
    det = 2.0 * A * B - C * C
    rhs_theta = u + MRL * sin(psi + zeta) * psi_d * psi_d - D
    rhs_psi = -u + MgL * sin(psi)
    theta_dd = (B * rhs_theta - C * rhs_psi) / det
    psi_dd = (-C * rhs_theta + 2.0 * A * rhs_psi) / det
    return theta_dd, psi_dd


def cbf_conditions(x, u, p, cos, sin):
    """The four CBF expressions; each one must be >= 0."""
    psi, theta_d, psi_d = x[1], x[2], x[3]
    psi_max, theta_d_max = p[6], p[7]
    a1, a2, a3 = p[8], p[9], p[10]
    theta_dd, psi_dd = accelerations(x, u, p, cos, sin)
    # Each condition is divided by its (positive) class-K gain: same inequality, but the
    # values (and the slacks) are then in rad and rad/s, which keeps the QP well scaled.
    k = a1 * a2
    c1 = (-psi_dd - (a1 + a2) * psi_d) / k + (psi_max - psi)          # h1 = psi_max - psi
    c2 = (psi_dd + (a1 + a2) * psi_d) / k + (psi + psi_max)           # h2 = psi + psi_max
    c3 = -theta_dd / a3 + (theta_d_max - theta_d)                     # h3 = theta_dot_max - theta_dot
    c4 = theta_dd / a3 + (theta_d + theta_d_max)                      # h4 = theta_dot + theta_dot_max
    return c1, c2, c3, c4


class CBFQPFilter:
    def __init__(self) -> None:
        self.available = False
        self.reason = "CBF-QP belum di-setup."
        self.solver = None
        self.p = None
        self.torque_per_raw = 0.0
        self.fail_count = 0
        self.last: Optional[CBFResult] = None

    # ---- configuration -----------------------------------------------------------
    @staticmethod
    def check_config() -> str:
        """Return '' if config.py is complete for CBF-QP, otherwise the reason."""
        missing = [name for name in _REQUIRED if getattr(config, name, None) is None]
        if missing:
            return "Isi dulu di config.py [CBF-QP]: " + ", ".join(missing)
        for name in ("Alpha_1", "Alpha_2", "Alpha_3"):
            if float(getattr(config, name)) <= 0.0:
                return f"{name} harus > 0 untuk CBF-QP (sekarang {getattr(config, name)})."
        if float(config.CBF_PSI_MAX_DEG) >= float(config.SAFE_TILT_DEG):
            return "CBF_PSI_MAX_DEG harus lebih kecil dari SAFE_TILT_DEG."
        if str(config.MOTOR_CONTROL_MODE).strip().lower() != "current":
            return "CBF-QP butuh MOTOR_CONTROL_MODE = 'current' (u adalah torsi)."
        return ""

    def parameter_vector(self) -> list:
        c = model_coefficients()
        return [
            c["A"], c["B"], c["MRL"], c["MgL"], c["D"], c["zeta"],
            math.radians(float(config.CBF_PSI_MAX_DEG)),
            math.radians(float(config.CBF_THETA_DOT_MAX_DEG_S)),
            float(config.Alpha_1), float(config.Alpha_2), float(config.Alpha_3),
        ]

    # ---- acados problem ----------------------------------------------------------
    def _build_ocp(self, dt: float):
        import casadi as ca
        import numpy as np
        from acados_template import AcadosModel, AcadosOcp

        x = ca.SX.sym("x", 4)     # [theta, psi, theta_dot, psi_dot]
        u = ca.SX.sym("u", 1)     # torque_left + torque_right [N m]
        p = ca.SX.sym("p", len(P_NAMES))
        theta_dd, psi_dd = accelerations(x, u[0], p, ca.cos, ca.sin)

        model = AcadosModel()
        model.name = MODEL_NAME
        model.x, model.u, model.p = x, u, p
        model.disc_dyn_expr = x + dt * ca.vertcat(x[2], x[3], theta_dd, psi_dd)
        model.con_h_expr_0 = ca.vertcat(*cbf_conditions(x, u[0], p, ca.cos, ca.sin))

        ocp = AcadosOcp()
        ocp.model = model
        ocp.code_export_directory = CODE_DIR
        ocp.solver_options.N_horizon = 1
        ocp.solver_options.tf = dt
        ocp.parameter_values = np.asarray(self.p, dtype=float)

        # cost: (u - u_PD)^2 on the first (and only) shooting node
        ocp.cost.cost_type_0 = "LINEAR_LS"
        ocp.cost.Vx_0 = np.zeros((1, 4))
        ocp.cost.Vu_0 = np.eye(1)
        ocp.cost.W_0 = np.eye(1)
        ocp.cost.yref_0 = np.zeros(1)
        ocp.cost.cost_type = "LINEAR_LS"
        ocp.cost.cost_type_e = "LINEAR_LS"

        # CBF conditions (soft, so the QP is always feasible)
        big = 1.0e9
        ocp.constraints.lh_0 = np.zeros(4)
        ocp.constraints.uh_0 = big * np.ones(4)
        ocp.constraints.idxsh_0 = np.arange(4)
        w_psi = float(config.CBF_SLACK_WEIGHT_PSI)
        w_vel = float(config.CBF_SLACK_WEIGHT_THETA_DOT)
        weights = np.array([w_psi, w_psi, w_vel, w_vel])
        ocp.cost.zl_0 = weights
        ocp.cost.zu_0 = weights
        ocp.cost.Zl_0 = weights
        ocp.cost.Zu_0 = weights

        # actuator limits (updated at run time) and the measured state
        ocp.constraints.idxbu = np.array([0])
        ocp.constraints.lbu = np.array([-1.0])
        ocp.constraints.ubu = np.array([1.0])
        ocp.constraints.x0 = np.zeros(4)

        ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
        ocp.solver_options.nlp_solver_type = "SQP_RTI"
        ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
        ocp.solver_options.integrator_type = "DISCRETE"
        ocp.solver_options.print_level = 0
        ocp.solver_options.hpipm_mode = "ROBUST"
        ocp.solver_options.qp_solver_iter_max = 100
        return ocp

    def _signature(self, dt: float) -> str:
        import acados_template
        return "|".join(str(v) for v in (
            getattr(acados_template, "__version__", "?"), dt,
            config.CBF_SLACK_WEIGHT_PSI, config.CBF_SLACK_WEIGHT_THETA_DOT, len(P_NAMES), 4,
        ))

    def setup(self) -> bool:
        """Build (or reuse) the acados solver. Call once at start-up, never while balancing."""
        self.available = False
        self.reason = self.check_config()
        if self.reason:
            return False
        try:
            from acados_template import AcadosOcpSolver

            self.p = self.parameter_vector()
            dt = 1.0 / float(config.CONTROL_HZ)
            signature = self._signature(dt)
            reuse = False
            if not config.CBF_FORCE_REBUILD and os.path.isfile(SIGNATURE_FILE) and os.path.isfile(JSON_FILE):
                with open(SIGNATURE_FILE) as handle:
                    reuse = handle.read() == signature
            ocp = self._build_ocp(dt)
            cwd = os.getcwd()
            os.chdir(_HERE)
            try:
                self.solver = AcadosOcpSolver(
                    ocp, json_file=JSON_FILE, generate=not reuse, build=not reuse, verbose=False
                )
            finally:
                os.chdir(cwd)
            if not reuse:
                with open(SIGNATURE_FILE, "w") as handle:
                    handle.write(signature)

            import numpy as np
            self._np = np
            for stage in (0, 1):
                self.solver.set(stage, "p", np.asarray(self.p, dtype=float))
            # raw current count per wheel -> total torque of both wheels [N m]
            self.torque_per_raw = 2.0 * float(config.CBF_TORQUE_CONSTANT_NM_PER_A) / RAW_PER_AMP
            self.available = True
            self.reason = "siap (kode acados dipakai ulang)" if reuse else "siap (kode acados baru di-generate)"
            LOGGER.info("CBF-QP %s", self.reason)
            return True
        except Exception as exc:
            self.solver = None
            self.reason = f"acados gagal di-setup: {exc}"
            LOGGER.error("CBF-QP %s", self.reason)
            return False

    # ---- run time ----------------------------------------------------------------
    def state_si(self, angle_psi, angular_dot_psi, angle_theta, angular_dot_theta) -> list:
        """PD units (deg, encoder angle) -> model state [theta, psi, theta_dot, psi_dot] (rad).

        The model's theta is the ABSOLUTE wheel angle. The DDSM115 encoder measures the
        wheel relative to the body, so theta = theta_encoder + psi.
        """
        psi = math.radians(angle_psi)
        psi_d = math.radians(angular_dot_psi)
        return [math.radians(angle_theta) + psi, psi, math.radians(angular_dot_theta) + psi_d, psi_d]

    def barrier_values(self, x) -> Tuple[float, float, float, float]:
        psi_max, theta_d_max = self.p[6], self.p[7]
        return (psi_max - x[1], x[1] + psi_max, theta_d_max - x[2], x[2] + theta_d_max)

    def filter(self, u_PD: float, angle_psi: float, angular_dot_psi: float,
               angle_theta: float, angular_dot_theta: float) -> CBFResult:
        """u_PD and the returned u_safe are raw current counts per wheel (same unit as the PD)."""
        np = self._np
        limit_raw = u_limit_raw()
        u_nominal_raw = max(-limit_raw, min(limit_raw, float(u_PD)))
        x = self.state_si(angle_psi, angular_dot_psi, angle_theta, angular_dot_theta)
        h = self.barrier_values(x)

        u_max = limit_raw * self.torque_per_raw
        # Reference = PD command saturated to the actuator limit (same minimiser, better conditioning).
        u_ref = u_nominal_raw * self.torque_per_raw
        x_np = np.asarray(x, dtype=float)
        solver = self.solver
        try:
            solver.set(0, "lbx", x_np)
            solver.set(0, "ubx", x_np)
            solver.set(0, "x", x_np)       # linearisation point = measured state -> the QP is exact
            solver.set(0, "u", np.array([u_nominal_raw * self.torque_per_raw]))
            solver.constraints_set(0, "lbu", np.array([-u_max]))
            solver.constraints_set(0, "ubu", np.array([u_max]))
            solver.cost_set(0, "yref", np.array([u_ref]))
            status = int(solver.solve())
            u_opt = float(solver.get(0, "u")[0])
            solve_ms = float(solver.get_stats("time_tot")) * 1000.0
        except Exception as exc:
            LOGGER.error("CBF-QP solve error: %s", exc)
            status, u_opt, solve_ms = -1, float("nan"), 0.0

        if status != 0 or not math.isfinite(u_opt):
            # Fail safe: fall back to the (saturated) PD command.
            self.fail_count += 1
            result = CBFResult(u_nominal_raw, float(u_PD), False, status if status != 0 else -1, solve_ms, h)
        else:
            u_safe_raw = u_opt / self.torque_per_raw
            active = abs(u_safe_raw - u_nominal_raw) > 1.0     # more than 1 raw count
            result = CBFResult(u_safe_raw, float(u_PD), active, status, solve_ms, h)
        self.last = result
        return result