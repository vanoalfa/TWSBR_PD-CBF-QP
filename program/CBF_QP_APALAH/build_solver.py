"""build_solver.py -- GENERATOR SOLVER acados UNTUK CBF-QP (HPIPM, N=1).

Jalankan script ini SEKALI sebelum menjalankan robot:

    python3 build_solver.py

Script ini akan:
  1. Membangun model robot (robot_model.py)
  2. Merumuskan QP CBF sesuai persamaan Anda:

         min   (1/2) u^T (2I) u + (-2 u_PD)^T u
         s.t.  LgLf h_i(x) u >= -Lf^2 h_i(x) - Lf[alpha1(h_i)]
                                 - alpha2( Lf h_i(x) + alpha1(h_i) )   (i=1..4)
               |u| <= CBF_U_MAX_A

     dengan 4 barrier function (HOCBF orde-2):
         h1 = -psi + PSI_MAX
         h2 =  psi + PSI_MAX
         h3 = -theta_dot + THETA_DOT_MAX
         h4 =  theta_dot + THETA_DOT_MAX

  3. Men-generate kode C solver HPIPM ke folder CBF_SOLVER_FOLDER.

Setelah generate selesai, solver dipanggil real-time oleh cbf_qp_controller.py.
"""

from __future__ import annotations

import numpy as np
import casadi as ca
from acados_template import AcadosOcp, AcadosOcpSolver

import tunning
from robot_model import export_model_atera, hitung_linear_coefficients


def bangun_ocp() -> AcadosOcp:
    """Merumuskan Optimal Control Problem (OCP) untuk CBF-QP N=1."""

    # ------------------------------------------------------------------
    # 0. Parameter & koefisien model
    # ------------------------------------------------------------------
    coeff = hitung_linear_coefficients()
    A1, A2, A3, B1 = coeff["A1"], coeff["A2"], coeff["A3"], coeff["B1"]
    A4, A5, A6, B2 = coeff["A4"], coeff["A5"], coeff["A6"], coeff["B2"]

    alpha1 = float(tunning.Alpha_1)
    alpha2 = float(tunning.Alpha_2)
    psi_max = np.deg2rad(float(tunning.PSI_MAX_DEG))
    thdot_max = np.deg2rad(float(tunning.THETA_DOT_MAX_DEG_S))
    u_max = float(tunning.CBF_U_MAX_A)
    w_slack = float(tunning.W_SLACK)

    # ------------------------------------------------------------------
    # 1. Model robot
    # ------------------------------------------------------------------
    model = export_model_atera()
    x = model.x
    u = model.u

    psi = x[0]
    psi_dot = x[1]
    theta_dot = x[2]

    # ------------------------------------------------------------------
    # 2. OCP dasar
    # ------------------------------------------------------------------
    ocp = AcadosOcp()
    ocp.model = model

    # Horizon N = 1 (sesuai rencana Anda)
    N = int(tunning.CBF_SOLVER_N)
    ocp.dims.N = N

    # Sampling time (detik) -- hanya dipakai untuk diskritisasi internal
    ocp.solver_options.tf = 1.0 / float(tunning.CONTROL_HZ)

    # ------------------------------------------------------------------
    # 3. Fungsi biaya  min (1/2)u^T(2I)u + (-2 u_PD)^T u  + slack
    # ------------------------------------------------------------------
    # Kita pakai cost tipe NONLINEAR_LS agar fleksibel, tetapi karena yang
    # diminimalkan adalah (u - u_PD)^2, ekuivalen dengan bentuk di atas.
    #
    # Namun untuk mencerminkan PERSIS bentuk QP Anda, kita pakai EXTERNAL cost:
    #   cost = 0.5 * u^T * (2I) * u + (-2*u_PD)^T * u + W_SLACK * sum(delta^2)
    #
    # u_PD diberikan sebagai PARAMETER ONLINE (p) yang di-update tiap loop.
    ocp.model.p = ca.SX.sym("u_pd", 2)      # parameter online: u_PD = [I_L_pd, I_R_pd]

    # Slack untuk 4 constraint CBF ditangani mekanisme soft-constraint bawaan
    # acados (lihat ocp.constraints.idxs + ocp.cost.Zl/Zu di bagian constraint).
    # Dengan begitu QP tidak pernah infeasible dan cost tetap pada variabel u.

    # Cost EXTERNAL pada node 0 (dan terminal):
    #   0.5 * u^T * (2I) * u + (-2*u_pd)^T * u
    H = 2.0 * np.eye(2)                     # Hessian (2I)
    cost_expr = 0.5 * u.T @ H @ u + (-2.0 * ocp.model.p).T @ u
    ocp.cost.cost_type = "EXTERNAL"
    ocp.model.cost_expr_ext_cost = cost_expr
    # Terminal cost = 0 (N=1, tidak ada cost state terminal)
    ocp.cost.cost_type_e = "EXTERNAL"
    ocp.model.cost_expr_ext_cost_e = ca.SX.zeros(1, 1)

    # ------------------------------------------------------------------
    # 4. Constraint HOCBF orde-2 (4 barrier)
    # ------------------------------------------------------------------
    # Untuk model linear di atas, turunan Lie dari setiap barrier dihitung
    # secara ANALITIS (karena model linear, hasilnya juga linear/afin).
    #
    # h1 = -psi + psi_max
    #   Lf h1  = -psi_dot
    #   Lf^2h1 = -psi_ddot  = -(A1*psi + A2*psi_dot + A3*theta_dot)
    #   LgLfh1 = -B1 * [1 1]          (baris vektor)
    #
    # h2 = psi + psi_max
    #   Lf h2  = psi_dot
    #   Lf^2h2 = psi_ddot   =  (A1*psi + A2*psi_dot + A3*theta_dot)
    #   LgLfh2 = B1 * [1 1]
    #
    # h3 = -theta_dot + thdot_max
    #   Lf h3  = -theta_ddot = -(A4*psi + A5*psi_dot + A6*theta_dot)
    #   Lf^2h3 = -(turunan Lf h3 sepanjang f)  -> dihitung numerik via CasADi jacobian
    #   LgLfh3 = -B2 * [1 1]
    #
    # h4 = theta_dot + thdot_max
    #   Lf h4  = theta_ddot = (A4*psi + A5*psi_dot + A6*theta_dot)
    #   LgLfh4 = B2 * [1 1]
    #
    # Kondisi HOCBF orde-2 untuk setiap barrier i:
    #   LgLf h_i * u >= -Lf^2 h_i - Lf[alpha1(h_i)] - alpha2( Lf h_i + alpha1(h_i) )
    #
    # dengan alpha1(h) = alpha1*h, alpha2(h) = alpha2*h (linear class-K).
    #
    # Kita definisikan h_i sebagai fungsi CasADi dan pakai jacobian untuk Lie
    # derivative secara umum (lebih aman & otomatis).

    # --- f(x) bagian drift (tanpa input) ---
    f_expl = model.f_expl_expr
    # f_expl = f(x) + g(x) u ; drift f(x) = f_expl dengan u=0
    f_drift = ca.substitute(f_expl, u, ca.DM.zeros(2, 1))
    # g(x) = jacobian f_expl terhadap u (3x2)
    g_mat = ca.jacobian(f_expl, u)

    # --- Definisi 4 barrier function ---
    h_list = [
        -psi + psi_max,          # h1
        psi + psi_max,          # h2
        -theta_dot + thdot_max,  # h3
        theta_dot + thdot_max,  # h4
    ]

    constraints = []   # kumpulan ekspresi constraint dalam bentuk con_h >= 0

    for h in h_list:
        # Lf h = grad(h) . f_drift
        Lf_h = ca.jacobian(h, x) @ f_drift
        # Lf^2 h = grad(Lf_h) . f_drift
        Lf2_h = ca.jacobian(Lf_h, x) @ f_drift
        # LgLf h = grad(Lf_h) . g_mat   -> vektor 1x2
        LgLf_h = ca.jacobian(Lf_h, x) @ g_mat

        # Sisi kanan kondisi HOCBF:
        #   RHS = -Lf2_h - Lf[alpha1(h)] - alpha2( Lf_h + alpha1(h) )
        # dengan alpha1(h)=alpha1*h, alpha2(h)=alpha2*h
        # Lf[alpha1(h)] = alpha1 * Lf_h
        RHS = -Lf2_h - alpha1 * Lf_h - alpha2 * (Lf_h + alpha1 * h)

        # Constraint: LgLf_h * u >= RHS
        # acados nonlinear constraint: con_h(x,u) >= 0
        con_expr = LgLf_h @ u - RHS
        constraints.append(con_expr)

    # Gabungkan jadi satu vektor constraint nonlinear (4x1)
    con_h_expr = ca.vertcat(*constraints)

    # Pasang ke model sebagai constraint pada node 0
    ocp.model.con_h_expr = con_h_expr
    ocp.constraints.lh = np.zeros(4)            # lower bound = 0 (>= 0)
    ocp.constraints.uh = np.full(4, 1e15)       # upper bound tak hingga
    # Soft constraint (slack) untuk 4 barrier
    ocp.constraints.idxs = np.array([0, 1, 2, 3])
    nsh = 4
    ocp.cost.Zl = w_slack * np.ones(nsh)       # penalti kuadratik lower slack
    ocp.cost.Zu = w_slack * np.ones(nsh)
    ocp.cost.zl = np.zeros(nsh)                # penalti linear lower slack
    ocp.cost.zu = np.zeros(nsh)

    # Terminal constraint: tidak ada (N=1)
    ocp.model.con_h_expr_e = None

    # ------------------------------------------------------------------
    # 5. Batas input (hard box constraint)  |u| <= u_max
    # ------------------------------------------------------------------
    ocp.constraints.lbu = np.array([-u_max, -u_max])
    ocp.constraints.ubu = np.array([u_max, u_max])
    ocp.constraints.idxbu = np.array([0, 1])

    # Batas state (longgar, hanya untuk kestabilan solver)
    ocp.constraints.lbx = np.array([-np.pi, -10.0 * np.pi, -100.0 * np.pi])
    ocp.constraints.ubx = np.array([np.pi, 10.0 * np.pi, 100.0 * np.pi])
    ocp.constraints.idxbx = np.array([0, 1, 2])

    # Kondisi awal x0 (akan di-set tiap loop via set)
    ocp.constraints.x0 = np.zeros(3)

    # ------------------------------------------------------------------
    # 6. Nilai awal parameter u_pd (akan di-update tiap loop)
    # ------------------------------------------------------------------
    ocp.parameter_values = np.zeros(2)

    # ------------------------------------------------------------------
    # 7. Opsi solver HPIPM
    # ------------------------------------------------------------------
    ocp.solver_options.qp_solver = "HPIPM"
    ocp.solver_options.hpipm_mode = str(tunning.CBF_SOLVER_HPIPM_MODE)
    ocp.solver_options.hessian_approx = "EXACT"
    ocp.solver_options.integrator_type = "DISCRETE"  # N=1: model diskrit 1 langkah
    ocp.solver_options.nlp_solver_type = "SQP"       # 1 iterasi SQP cukup (QP murni)
    ocp.solver_options.qp_solver_cond_N = 1
    ocp.solver_options.qp_solver_iter_max = 50
    ocp.solver_options.nlp_solver_max_iter = 1
    ocp.solver_options.print_level = 0

    # Folder output kode C hasil generate
    ocp.code_export_directory = str(tunning.CBF_SOLVER_FOLDER)

    return ocp


def main() -> None:
    """Titik masuk utama: bangun OCP lalu generate solver C."""
    print("=" * 64)
    print(" GENERATOR SOLVER CBF-QP (acados + HPIPM, N=1)")
    print("=" * 64)

    # Pastikan parameter fisik sudah diisi
    try:
        coeff = hitung_linear_coefficients()
    except ValueError as exc:
        print(f"\n[ERROR] {exc}")
        print("Isi dulu parameter fisik robot di tunning.py bagian 3!\n")
        return

    print("\nKoefisien model yang dipakai:")
    for k in ("A1", "A2", "A3", "B1", "A4", "A5", "A6", "B2"):
        print(f"  {k} = {coeff[k]:+.6f}")

    print(f"\nParameter CBF:")
    print(f"  Alpha_1 = {tunning.Alpha_1}")
    print(f"  Alpha_2 = {tunning.Alpha_2}")
    print(f"  PSI_MAX = {tunning.PSI_MAX_DEG} deg")
    print(f"  THETA_DOT_MAX = {tunning.THETA_DOT_MAX_DEG_S} deg/s")
    print(f"  U_MAX = {tunning.CBF_U_MAX_A} A")
    print(f"  N = {tunning.CBF_SOLVER_N}, HPIPM mode = {tunning.CBF_SOLVER_HPIPM_MODE}")

    print("\nMembangun OCP ...")
    ocp = bangun_ocp()

    print("Men-generate kode C solver ...")
    solver = AcadosOcpSolver(ocp, json_file=f"{tunning.CBF_SOLVER_FOLDER}.json")

    print(f"\n[SELESAI] Solver berhasil di-generate ke folder:")
    print(f"  -> {tunning.CBF_SOLVER_FOLDER}/")
    print(f"  -> {tunning.CBF_SOLVER_FOLDER}.json")
    print("\nSelanjutnya jalankan robot dengan:  python3 atera_main.py\n")


if __name__ == "__main__":
    main()
