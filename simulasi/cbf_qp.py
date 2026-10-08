"""cbf_qp.py - safety filter CBF-QP ATERA, diselesaikan acados (HPIPM, N = 1).

    u* = argmin_u  || u - u_PD ||^2
    s.t.  c_k(x, u) >= 0      untuk setiap c_k di DAFTAR_C   (ditulis di WORKPLACE PROGRAM, di bawah)
          -i_max <= u / K_t <= i_max                           batas arus motor

    u = torsi SATU roda [N m].   Hard constraint, tanpa slack.
    Alpha_1, Alpha_2, ... diambil dari config.py (semua Alpha_<nomor> yang berurutan mulai dari 1).
    zeta (kemiringan bidang) juga parameter: nilainya dibaca dari config.zeta tiap siklus, jadi bisa diubah
    saat program berjalan tanpa membangun ulang solver.

Susunan di acados (OCP dengan N = 1):
    stage 0 : state x dikunci ke state terukur (lbx_0 = ubx_0 = x), variabel keputusan u,
              cost LINEAR_LS (u - u_PD)^2, semua constraint DAFTAR_C di con_h_expr_0, batas arus di lbu/ubu.
    stage 1 : x1 = x + DT*(f(x) + g(x)*u) (prediksi satu langkah), tanpa cost dan tanpa constraint.
    Solver  : SQP_RTI (satu QP per panggilan) + PARTIAL_CONDENSING_HPIPM. Karena x dikunci dan
              constraint affine terhadap u, QP yang diselesaikan HPIPM persis CBF-QP di atas.

Kebutuhan: acados sudah di-build (ada <folder acados>/lib/libacados.so) dan casadi terpasang.
Folder acados dicari otomatis (lihat siapkan_acados): variabel ACADOS_SOURCE_DIR, lalu folder "acados"
di sebelah folder program ini atau di atasnya, lalu ~/acados. Jadi tidak perlu export apa pun.

Semua nilai fisik dan gain dibaca dari config.py; f(x) dan g(x) dari model.py.
"""

from __future__ import annotations

import ctypes
import glob
import os
import sys
import time

import casadi as ca
import numpy as np

import config
import model

STATUS_OK = "OK"                    # u = u* dari acados
STATUS_INFEASIBLE = "INFEASIBLE"    # constraint saling bertentangan, tidak ada u yang memenuhi
STATUS_SOLVER = "SOLVER_GAGAL"      # acados/HPIPM gagal atau hasilnya melanggar constraint
STATUS_ALPHA = "ALPHA_NOL"          # ada Alpha <= 0, CBF-QP tidak dijalankan

# Toleransi numerik (bukan besaran fisik)
ACADOS_INFTY = 1.0e9                # batas atas "tak hingga" constraint h
CONSTRAINT_TOL = 1.0e-6             # pelanggaran constraint yang masih diterima saat memeriksa hasil solver
BOUND_TOL = 1.0e-6                  # [N m] pelanggaran batas u yang masih diterima
ACTIVE_TOL = 1.0e-4                 # [N m] |u* - u_PD| di atas ini -> filter dianggap mengoreksi
COEFF_EPS = 1.0e-12                 # |koefisien u| di bawah ini dianggap nol


def cari_folder_acados():
    """Folder acados (yang berisi interfaces/acados_template), atau "" bila tidak ditemukan."""
    calon = []
    if os.environ.get("ACADOS_SOURCE_DIR"):
        calon.append(os.environ["ACADOS_SOURCE_DIR"])
    folder = os.path.dirname(os.path.abspath(__file__))
    for _ in range(4):
        calon.append(os.path.join(folder, "acados"))
        folder = os.path.dirname(folder)
    calon.append(os.path.join(os.path.expanduser("~"), "acados"))
    for path in calon:
        if os.path.isdir(os.path.join(path, "interfaces", "acados_template")):
            return os.path.abspath(path)
    return ""


def siapkan_acados():
    """Siapkan acados supaya bisa di-import tanpa export manual. Mengembalikan folder acados."""
    folder = cari_folder_acados()
    if folder == "":
        raise RuntimeError("folder acados tidak ditemukan. Taruh folder 'acados' di sebelah folder program ini, "
                           "atau isi variabel ACADOS_SOURCE_DIR.")

    lib = os.path.join(folder, "lib")
    if len(glob.glob(os.path.join(lib, "libacados.*"))) == 0:
        raise RuntimeError("acados belum di-build (tidak ada %s/libacados.so). Jalankan:\n"
                           "    cd %s && git submodule update --recursive --init\n"
                           "    mkdir -p build && cd build && cmake -DACADOS_WITH_QPOASES=OFF .. && make install -j4"
                           % (lib, folder))

    os.environ["ACADOS_SOURCE_DIR"] = folder
    os.environ["LD_LIBRARY_PATH"] = lib + os.pathsep + os.environ.get("LD_LIBRARY_PATH", "")

    # Muat library acados lebih dulu, supaya solver hasil generate menemukannya walaupun
    # LD_LIBRARY_PATH tidak diatur sebelum Python dijalankan.
    for nama in ("libblasfeo.so", "libhpipm.so", "libacados.so"):
        path = os.path.join(lib, nama)
        if os.path.exists(path):
            ctypes.CDLL(path, mode=ctypes.RTLD_GLOBAL)

    # Pakai acados_template dari folder acados bila belum di-install dengan pip.
    try:
        import acados_template  # noqa: F401
    except ImportError as kesalahan:
        if kesalahan.name != "acados_template":
            raise RuntimeError("acados_template butuh modul '%s'. Jalankan: pip install -e %s"
                               % (kesalahan.name, os.path.join(folder, "interfaces", "acados_template")))
        sys.path.insert(0, os.path.join(folder, "interfaces", "acados_template"))
        try:
            import acados_template  # noqa: F401
        except ImportError as kesalahan2:
            raise RuntimeError("acados_template butuh modul '%s'. Jalankan: pip install -e %s"
                               % (kesalahan2.name, os.path.join(folder, "interfaces", "acados_template")))
    return folder


def cari_nama_alpha():
    """Nama Alpha di config.py: Alpha_1, Alpha_2, ... (berurutan, berhenti di nomor pertama yang tidak ada)."""
    hasil = []
    nomor = 1
    while hasattr(config, "Alpha_%d" % nomor):
        hasil.append("Alpha_%d" % nomor)
        nomor = nomor + 1
    return hasil


def workplace(x, u, f, g, zeta):
    """Tempat menulis barrier function h dan constraint CBF. Mengembalikan DAFTAR_C.

    Yang bisa dipakai di dalam WORKPLACE:
        theta, dtheta, psi, dpsi  : state (simbol)
        u                         : torsi satu roda (simbol)
        f, g                      : model dx/dt = f(x) + g(x)*u   (dari model.py, sudah memakai zeta)
        zeta                      : kemiringan bidang [rad] (simbol, nilainya dari config.zeta)
        lie(h, v)                 : turunan Lie, L_v h = (dh/dx) * v
        Alpha_1, Alpha_2, ...     : semua Alpha_<nomor> di config.py (otomatis, nilainya bisa diubah di jendela Kontrol)
        config.<nama>             : nilai lain dari config.py (psi_max, dpsi_max, dtheta_max, MAX_CURRENT_A, ...)
    """
    theta = x[config.IDX_THETA]
    dtheta = x[config.IDX_DTHETA]
    psi = x[config.IDX_PSI]
    dpsi = x[config.IDX_DPSI]
    s = config.R * theta                # jarak tempuh [m], dihitung dari posisi awal (R = reset)

    def lie(h, v):
        """Turunan Lie: L_v h = (dh/dx) * v"""
        return ca.jacobian(h, x) @ v

    # =========================================================================================
    #                                   WORKPLACE PROGRAM
    # =========================================================================================
    # FORMAT (salin, lalu ganti h dan Alpha):
    #
    #   CBF orde 1 (h yang turunan pertamanya sudah memuat u, contoh: dpsi, dtheta):
    #       c_hX = lie(hX, f) + lie(hX, g) * u + Alpha_k * hX
    #
    #   HOCBF orde 2 (h yang baru memuat u di turunan kedua, contoh: psi, theta, jarak s):
    #       Lf_hX = lie(hX, f)
    #       c_hX = lie(Lf_hX, f) + lie(Lf_hX, g) * u + Alpha_k * Lf_hX + Alpha_m * (Lf_hX + Alpha_k * hX)
    #
    #   Batas langsung pada u (bukan barrier, tanpa Alpha), contoh arus motor i = u / K_t:
    #       c_arus_atas  = config.MAX_CURRENT_A - u / config.MOTOR_KT        # i <= i_max
    #       c_arus_bawah = u / config.MOTOR_KT - config.MIN_CURRENT_A        # i >= i_min
    #
    #   Alpha baru: tambahkan Alpha_4 = ... (dst, nomor berurutan) di config.py, lalu pakai di sini.
    #   Constraint hanya dipakai bila dimasukkan ke DAFTAR_C.
    # -----------------------------------------------------------------------------------------

    # 1. Tulis definisi h di bawah
    h1 = -psi + config.psi_max
    h2 = psi + config.psi_max
    h3 = -dpsi + config.dpsi_max
    h4 = dpsi + config.dpsi_max
    h5 = -s + config.s_max              # s <= s_max
    h6 = s - config.s_min               # s >= s_min
    h7 = -dtheta + config.dtheta_max
    h8 = dtheta + config.dtheta_max

    # 2. Tulis constraint di bawah
    Lf_h1 = lie(h1, f)
    c_h1 = lie(Lf_h1, f) + lie(Lf_h1, g) * u + Alpha_1 * Lf_h1 + Alpha_2 * (Lf_h1 + Alpha_1 * h1)
    Lf_h2 = lie(h2, f)
    c_h2 = lie(Lf_h2, f) + lie(Lf_h2, g) * u + Alpha_1 * Lf_h2 + Alpha_2 * (Lf_h2 + Alpha_1 * h2)

    c_h3 = lie(h3, f) + lie(h3, g) * u + Alpha_3 * h3
    c_h4 = lie(h4, f) + lie(h4, g) * u + Alpha_3 * h4

    Lf_h5 = lie(h5, f)
    c_h5 = lie(Lf_h5, f) + lie(Lf_h5, g) * u + Alpha_4 * Lf_h5 + Alpha_5 * (Lf_h5 + Alpha_4 * h5)
    Lf_h6 = lie(h6, f)
    c_h6 = lie(Lf_h6, f) + lie(Lf_h6, g) * u + Alpha_4 * Lf_h6 + Alpha_5 * (Lf_h6 + Alpha_4 * h6)
    
    c_h7 = lie(h7, f) + lie(h7, g) * u + Alpha_6 * h7
    c_h8 = lie(h8, f) + lie(h8, g) * u + Alpha_6 * h8

    # 3. Daftarkan constraint yang dipakai
    DAFTAR_C = [c_h1, c_h2, c_h3, c_h4, c_h5, c_h6]

    # =========================================================================================
    #                                 AKHIR WORKPLACE PROGRAM
    # =========================================================================================
    return DAFTAR_C


def _cbf_expressions():
    """Simbol x, u, p = [Alpha_1, Alpha_2, ..., zeta] dan ekspresi c(x, u, p) dari WORKPLACE, constraint: c >= 0."""
    nama_alpha = cari_nama_alpha()
    x = ca.SX.sym("x", config.NX)
    u = ca.SX.sym("u", config.NU)
    p = ca.SX.sym("p", len(nama_alpha) + 1)        # elemen terakhir = zeta
    zeta = p[len(nama_alpha)]

    # Alpha_1, Alpha_2, ... dibuat sebagai nama global supaya bisa langsung ditulis di WORKPLACE.
    for i in range(len(nama_alpha)):
        globals()[nama_alpha[i]] = p[i]

    f, g = model.fg_casadi(x, zeta)
    try:
        daftar_c = workplace(x, u, f, g, zeta)
    except NameError as kesalahan:
        raise RuntimeError("WORKPLACE PROGRAM memakai nama yang belum ada (%s). Bila itu Alpha, tambahkan dulu "
                           "di config.py dengan nomor berurutan (Alpha_1, Alpha_2, ...)." % kesalahan)
    except AttributeError as kesalahan:
        raise RuntimeError("WORKPLACE PROGRAM memakai nilai config yang belum ada (%s). Tambahkan dulu di "
                           "config.py." % kesalahan)
    if len(daftar_c) == 0:
        raise RuntimeError("DAFTAR_C di WORKPLACE PROGRAM (cbf_qp.py) masih kosong.")
    for i in range(len(daftar_c)):
        if daftar_c[i].shape != (1, 1):
            raise RuntimeError("isi DAFTAR_C nomor %d bukan satu angka (ukuran %s)." % (i + 1, daftar_c[i].shape))
    return x, u, p, f, g, ca.vertcat(*daftar_c), nama_alpha


class CBFQP:
    def __init__(self, zeta=None, build: bool = True) -> None:
        """zeta: kemiringan bidang [rad] yang dipakai terus. None = baca config.zeta tiap siklus (bisa berubah).
        build=False memakai kode C yang sudah di-generate sebelumnya (config.py tidak boleh berubah)."""
        self.zeta_tetap = zeta
        self.folder_acados = siapkan_acados()
        from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver

        self.N = int(config.CBF_N_HORIZON)
        x, u, p, f, g, c, self.nama_alpha = _cbf_expressions()
        self.NH = int(c.shape[0])                       # jumlah constraint di DAFTAR_C

        # Alpha yang benar-benar dipakai di DAFTAR_C (hanya ini yang wajib > 0)
        self.alpha_dipakai = []
        for i in range(len(self.nama_alpha)):
            if ca.depends_on(c, p[i]):
                self.alpha_dipakai.append(i)

        # c(x, u, p) = a(x, p)*u + b(x, p): dipakai untuk memeriksa hasil solver dan untuk diagnosa
        a = ca.jacobian(c, u)
        b = ca.substitute(c, u, 0.0)
        self._coeff = ca.Function("cbf_coeff", [x, p], [a, b])

        acados_model = AcadosModel()
        acados_model.name = "atera_cbf_qp"
        acados_model.x = x
        acados_model.u = u
        acados_model.p = p
        acados_model.disc_dyn_expr = x + config.DT * (f + g * u)
        acados_model.con_h_expr_0 = c

        ocp = AcadosOcp()
        ocp.model = acados_model
        if hasattr(ocp.solver_options, "N_horizon"):
            ocp.solver_options.N_horizon = self.N
        else:                                          # acados < 0.4
            ocp.dims.N = self.N
        ocp.solver_options.tf = float(self.N)          # langkah waktu 1 -> cost tidak diskalakan
        ocp.parameter_values = np.ones(len(self.nama_alpha) + 1)

        # cost stage 0: 1/2*(u - yref)' W (u - yref), W = 2  ->  (u - u_PD)^2 ; yref = u_PD diisi tiap siklus
        ocp.cost.cost_type_0 = "LINEAR_LS"
        ocp.cost.Vx_0 = np.zeros((config.NU, config.NX))
        ocp.cost.Vu_0 = np.eye(config.NU)
        ocp.cost.W_0 = 2.0 * np.eye(config.NU)
        ocp.cost.yref_0 = np.zeros(config.NU)
        # tanpa cost di stage antara dan stage akhir
        ocp.cost.cost_type = "LINEAR_LS"
        ocp.cost.Vx = np.zeros((0, config.NX))
        ocp.cost.Vu = np.zeros((0, config.NU))
        ocp.cost.W = np.zeros((0, 0))
        ocp.cost.yref = np.zeros(0)
        ocp.cost.cost_type_e = "LINEAR_LS"
        ocp.cost.Vx_e = np.zeros((0, config.NX))
        ocp.cost.W_e = np.zeros((0, 0))
        ocp.cost.yref_e = np.zeros(0)

        # constraint
        ocp.constraints.x0 = np.zeros(config.NX)                     # diganti state terukur tiap siklus
        ocp.constraints.lh_0 = np.zeros(self.NH)                     # c(x, u, p) >= 0
        ocp.constraints.uh_0 = ACADOS_INFTY * np.ones(self.NH)
        ocp.constraints.idxbu = np.arange(config.NU)                 # batas arus: u_min <= u <= u_max
        ocp.constraints.lbu = np.array([config.MIN_CURRENT_A * config.MOTOR_KT])
        ocp.constraints.ubu = np.array([config.MAX_CURRENT_A * config.MOTOR_KT])

        ocp.solver_options.integrator_type = "DISCRETE"
        ocp.solver_options.nlp_solver_type = "SQP_RTI"
        ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
        ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
        ocp.solver_options.print_level = 0

        code_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "c_generated_code")
        json_name = "acados_ocp_atera_cbf_qp.json"
        if hasattr(ocp, "code_gen_options"):           # acados >= 0.5.4
            ocp.code_gen_options.code_export_directory = code_dir
            ocp.code_gen_options.json_file = json_name
            self.solver = AcadosOcpSolver(ocp, build=build, generate=build, verbose=False)
        else:
            ocp.code_export_directory = code_dir
            self.solver = AcadosOcpSolver(ocp, json_file=os.path.join(code_dir, json_name),
                                          build=build, generate=build, verbose=False)

        self.u_lb = float(ocp.constraints.lbu[0])
        self.u_ub = float(ocp.constraints.ubu[0])

        # hasil siklus terakhir
        self.status = STATUS_OK
        self.acados_status = 0
        self.feasible = True
        self.active = False
        self.c = np.zeros(self.NH)                     # nilai tiap constraint pada u terakhir (>= 0 = terpenuhi)
        self.u_pd = 0.0
        self.u = 0.0
        self.u_interval = (self.u_lb, self.u_ub)
        self.solve_ms = 0.0
        self.count = {STATUS_OK: 0, STATUS_INFEASIBLE: 0, STATUS_SOLVER: 0, STATUS_ALPHA: 0}

    def zeta(self) -> float:
        """Kemiringan bidang yang dipakai sekarang [rad]."""
        if self.zeta_tetap is not None:
            return float(self.zeta_tetap)
        return float(config.zeta)

    def _alphas(self) -> np.ndarray:
        """Nilai Alpha_1, Alpha_2, ... dari config (dibaca tiap siklus, jadi perubahan di jendela Kontrol langsung dipakai)."""
        nilai = []
        for nama in self.nama_alpha:
            nilai.append(float(getattr(config, nama)))
        return np.array(nilai)

    def _parameter(self, alphas=None) -> np.ndarray:
        """p = [Alpha_1, Alpha_2, ..., zeta]."""
        if alphas is None:
            alphas = self._alphas()
        return np.append(np.asarray(alphas, dtype=float), self.zeta())

    def constraint_coefficients(self, x, alphas=None):
        """a, b (masing-masing ndarray(NH)) pada constraint a*u + b >= 0."""
        p = self._parameter(alphas)
        a, b = self._coeff(np.asarray(x, dtype=float), p)
        return np.array(a).ravel(), np.array(b).ravel()

    def allowed_interval(self, x, alphas=None):
        """Selang u yang memenuhi semua constraint (termasuk batas arus). lo > hi berarti infeasible."""
        a, b = self.constraint_coefficients(x, alphas)
        lo, hi = self.u_lb, self.u_ub
        for ak, bk in zip(a, b):
            if ak > COEFF_EPS:
                lo = max(lo, -bk / ak)
            elif ak < -COEFF_EPS:
                hi = min(hi, -bk / ak)
            elif bk < 0.0:
                return np.inf, -np.inf
        return lo, hi

    def _fallback(self, status: str) -> float:
        # ASUMSI: bila CBF-QP tidak punya solusi atau solver gagal, u = u_PD yang disaturasi ke batas arus.
        # Kejadiannya tercatat di self.status / self.count supaya terlihat di log dan CSV.
        self.status = status
        self.feasible = False
        self.active = False
        self.count[status] += 1
        self.u = float(min(max(self.u_pd, self.u_lb), self.u_ub))
        return self.u

    def _catat_c(self, a, b):
        self.c = a * self.u + b

    def compute(self, x, u_pd: float) -> float:
        """u* [N m] (torsi satu roda) untuk state x = [theta, dtheta, psi, dpsi] dan u_PD dari PDControl."""
        x = np.asarray(x, dtype=float).reshape(config.NX)
        self.u_pd = float(u_pd)
        p = self._parameter()

        alpha_nol = False
        for i in self.alpha_dipakai:
            if p[i] <= 0.0:
                alpha_nol = True
        if alpha_nol:
            self.acados_status = -1
            self.u_interval = (self.u_lb, self.u_ub)
            self.solve_ms = 0.0
            return self._fallback(STATUS_ALPHA)

        a, b = self.constraint_coefficients(x, p[:len(self.nama_alpha)])
        self.u_interval = self.allowed_interval(x, p[:len(self.nama_alpha)])

        t0 = time.perf_counter()
        solver = self.solver
        solver.set(0, "lbx", x)
        solver.set(0, "ubx", x)
        solver.set(0, "x", x)
        solver.set(0, "u", np.array([min(max(self.u_pd, self.u_lb), self.u_ub)]))
        for stage in range(1, self.N + 1):
            solver.set(stage, "x", x)
        for stage in range(self.N + 1):
            solver.set(stage, "p", p)
        solver.cost_set(0, "yref", np.array([self.u_pd]))
        self.acados_status = int(solver.solve())
        u = float(solver.get(0, "u")[0])
        self.solve_ms = (time.perf_counter() - t0) * 1000.0

        scale = np.maximum(1.0, np.abs(a) + np.abs(b))
        solved = (self.acados_status == 0 and np.isfinite(u)
                  and self.u_lb - BOUND_TOL <= u <= self.u_ub + BOUND_TOL
                  and bool(np.all(a * u + b >= -CONSTRAINT_TOL * scale)))
        if not solved:
            lo, hi = self.u_interval
            self._fallback(STATUS_INFEASIBLE if lo > hi else STATUS_SOLVER)
            self._catat_c(a, b)
            return self.u

        self.status = STATUS_OK
        self.feasible = True
        self.count[STATUS_OK] += 1
        self.active = abs(u - self.u_pd) > ACTIVE_TOL
        self.u = u
        self._catat_c(a, b)
        return u


if __name__ == "__main__":
    np.set_printoptions(precision=4, suppress=True)
    for nama in cari_nama_alpha():
        if getattr(config, nama) <= 0.0:
            print("%s di config.py masih 0; uji ini memakai nilai sementara 20." % nama)
            setattr(config, nama, 20.0)
    cbf = CBFQP()
    print("jumlah constraint (DAFTAR_C):", cbf.NH, "| Alpha:", cbf.nama_alpha,
          "| Alpha yang dipakai:", [cbf.nama_alpha[i] for i in cbf.alpha_dipakai])
    rng = np.random.default_rng(0)
    worst = 0.0
    for _ in range(2000):
        x = rng.uniform(-1.0, 1.0, config.NX) * np.array([5.0, config.dtheta_max, config.psi_max, config.dpsi_max])
        u_pd = rng.uniform(2.0 * config.u_min, 2.0 * config.u_max)
        u = cbf.compute(x, u_pd)
        lo, hi = cbf.u_interval
        if lo <= hi:
            worst = max(worst, abs(u - min(max(u_pd, lo), hi)))
    print("jumlah per status:", cbf.count)
    print(f"selisih maksimum u* acados terhadap solusi eksak (kasus feasible): {worst:.2e} N m")
    print(f"waktu solve terakhir: {cbf.solve_ms:.3f} ms")