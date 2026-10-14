"""cbf_qp.py - safety filter CBF-QP ATERA, diselesaikan acados (HPIPM, N = 1). Sama dengan simulasi/cbf_qp.py.

    u* = argmin_u  || u - u_PD ||^2
    s.t.  c_k(x, u) >= 0      untuk setiap c_k di DAFTAR_C yang aktif di mode gerak sekarang
                              (DAFTAR_C dan pilihan constraint per mode ditulis di WORKPLACE PROGRAM, di bawah)
          -i_max <= u / K_t <= i_max                           batas arus motor

    u = torsi SATU roda [N m].   Hard constraint, tanpa slack.
    Alpha_1, Alpha_2, ... diambil dari config.py (semua Alpha_<nomor> yang berurutan mulai dari 1).
    zeta (kemiringan bidang) dan psi_max (batas h1, h2) juga parameter: nilainya dibaca dari config.zeta dan
    config.psi_max tiap siklus, jadi bisa diubah saat program berjalan tanpa membangun ulang solver.

Mode gerak (mode_cbf): Balancing dan Navigate memakai SATU solver acados yang sama. Setiap constraint diberi
parameter mask m_k (1 = aktif, 0 = dilewati): c_k dipakai sebagai  m_k*c_k + (1 - m_k),  jadi constraint dengan
m_k = 0 selalu terpenuhi dan tidak ikut membatasi u. Yang berganti saat mode berganti hanya mask, bukan solver.

Model torsi: u adalah torsi PERINTAH (arus perintah x K_t datasheet). Torsi yang benar-benar bekerja di roda adalah
rasio_torsi * u, dengan rasio_torsi = K_t efektif / K_t datasheet (config.RASIO_TORSI).
g(x) di model CBF-QP dikalikan rasio_torsi, jadi CBF-QP menghitung dengan tenaga roda yang sungguh ada. Tanpa ini
CBF-QP menganggap roda 1 / rasio_torsi kali (sekitar 3x) lebih kuat dari kenyataan: koreksi torsinya terlalu kecil,
dan constraint yang dikira terpenuhi ternyata dilanggar. rasio_torsi adalah parameter solver, jadi bisa diganti
saat berjalan (cbf.rasio_torsi = nilai) tanpa membangun ulang solver. Batas arus lbu/ubu tetap dalam torsi perintah.

Susunan di acados (OCP dengan N = 1):
    stage 0 : state x dikunci ke state terukur (lbx_0 = ubx_0 = x), variabel keputusan u,
              cost LINEAR_LS (u - u_PD)^2, semua constraint DAFTAR_C di con_h_expr_0, batas arus di lbu/ubu.
    stage 1 : x1 = x + DT*(f(x) + g(x)*u) (prediksi satu langkah), tanpa cost dan tanpa constraint.
    Solver  : SQP_RTI (satu QP per panggilan) + PARTIAL_CONDENSING_HPIPM. Karena x dikunci dan
              constraint affine terhadap u, QP yang diselesaikan HPIPM persis CBF-QP di atas.

Kebutuhan: acados sudah di-build (ada <folder acados>/lib/libacados.so) dan casadi terpasang.
Folder acados dicari otomatis (lihat siapkan_acados): variabel ACADOS_SOURCE_DIR, lalu folder "acados"
di sebelah folder program ini atau di atasnya, lalu ~/acados, lalu lokasi acados_template hasil pip install -e.
Jadi tidak perlu export apa pun.

Semua nilai fisik dan gain dibaca dari config.py; f(x) dan g(x) dari model.py.
"""

from __future__ import annotations

import ctypes
import glob
import importlib.util
import os
import sys
import time

import casadi as ca
import numpy as np

import config
import model
from pd_control import MODE_BALANCING, MODE_NAVIGATE

STATUS_AMAN = "AMAN"                # u = u* dari acados
STATUS_GAGAL = "GAGAL"              # tidak ada u yang memenuhi semua constraint, atau acados gagal secara numerik
STATUS_ALPHA = "ALPHA_NOL"          # ada Alpha <= 0, CBF-QP tidak dijalankan


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
    try:                                            # acados_template yang sudah di-install pip -e
        spec = importlib.util.find_spec("acados_template")
        if spec is not None and spec.origin:
            calon.append(os.path.join(os.path.dirname(spec.origin), "..", "..", ".."))
    except (ImportError, ValueError):
        pass
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


def workplace(x, u, f, g, zeta, psi_max):
    """Tempat menulis barrier function h dan constraint CBF. Mengembalikan (DAFTAR_C, C_MODE).

    Yang bisa dipakai di dalam WORKPLACE:
        theta, dtheta, psi, dpsi  : state (simbol)
        u                         : torsi satu roda (simbol)
        f, g                      : model dx/dt = f(x) + g(x)*u   (dari model.py, sudah memakai zeta dan rasio_torsi,
                                    u = torsi perintah satu roda)
        zeta                      : kemiringan bidang [rad] (simbol, nilainya dari config.zeta)
        psi_max                   : batas psi [rad] (simbol, nilainya dari config.psi_max tiap siklus)
        lie(h, v)                 : turunan Lie, L_v h = (dh/dx) * v
        Alpha_1, Alpha_2, ...     : semua Alpha_<nomor> di config.py (otomatis)
        MODE_BALANCING, MODE_NAVIGATE : nama mode gerak (kunci C_MODE)
        config.<nama>             : nilai lain dari config.py (dpsi_max, dtheta_max, s_max, MAX_CURRENT_A, ...)
    """
    theta = x[config.IDX_THETA]
    dtheta = x[config.IDX_DTHETA]
    psi = x[config.IDX_PSI]
    dpsi = x[config.IDX_DPSI]
    s = model.hitung_s(x)               # jarak tempuh [m] dari titik nol s (direset tiap masuk mode Balancing)

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
    #   Constraint hanya dipakai bila dimasukkan ke DAFTAR_C, dan hanya aktif di mode yang memuatnya di C_MODE.
    # -----------------------------------------------------------------------------------------

    # 1. Tulis definisi h di bawah
    h1 = -psi + psi_max
    h2 = psi + psi_max
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

    # 3. Daftarkan semua constraint yang bisa dipakai
    DAFTAR_C = [c_h1, c_h2, c_h3, c_h4, c_h5, c_h6, c_h7, c_h8]

    # 4. Pilih constraint yang aktif di tiap mode gerak (urutan di dalam daftar bebas, isinya dari DAFTAR_C)
    C_MODE = {
        MODE_BALANCING: [c_h1, c_h2, c_h3, c_h4, c_h5, c_h6, c_h7, c_h8],
        MODE_NAVIGATE:  [c_h1, c_h2, c_h3, c_h4, c_h7, c_h8],      # tanpa h5, h6 (barrier jarak s)
    }

    # =========================================================================================
    #                                 AKHIR WORKPLACE PROGRAM
    # =========================================================================================
    return DAFTAR_C, C_MODE


def _cbf_expressions():
    """Simbol x, u, p = [Alpha_1, Alpha_2, ..., zeta, rasio_torsi, psi_max], f, g, DAFTAR_C, C_MODE dan nama Alpha
    dari WORKPLACE. Constraint: c >= 0."""
    nama_alpha = config.NAMA_ALPHA
    x = ca.SX.sym("x", config.NX)
    u = ca.SX.sym("u", config.NU)
    p = ca.SX.sym("p", len(nama_alpha) + 3)        # tiga elemen terakhir: zeta, rasio_torsi, psi_max
    zeta = p[len(nama_alpha)]
    rasio_torsi = p[len(nama_alpha) + 1]
    psi_max = p[len(nama_alpha) + 2]

    # Alpha_1, Alpha_2, ... dibuat sebagai nama global supaya bisa langsung ditulis di WORKPLACE.
    for i in range(len(nama_alpha)):
        globals()[nama_alpha[i]] = p[i]

    f, g = model.fg_casadi(x, zeta)
    g = rasio_torsi * g                            # u = torsi perintah, torsi nyata = rasio_torsi * u (lihat docstring modul)
    try:
        daftar_c, c_mode = workplace(x, u, f, g, zeta, psi_max)
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
    return x, u, p, f, g, daftar_c, c_mode, nama_alpha


def _mask_tiap_mode(daftar_c, c_mode):
    """Mask 0 / 1 tiap mode: 1 bila constraint di DAFTAR_C ikut di daftar C_MODE mode itu."""
    mask = {}
    for mode in (MODE_BALANCING, MODE_NAVIGATE):
        if mode not in c_mode or len(c_mode[mode]) == 0:
            raise RuntimeError("C_MODE di WORKPLACE PROGRAM belum memuat constraint untuk mode %s." % mode)
        mask[mode] = np.zeros(len(daftar_c))
        for c in c_mode[mode]:
            ditemukan = False
            for k in range(len(daftar_c)):
                if c is daftar_c[k]:
                    mask[mode][k] = 1.0
                    ditemukan = True
            if not ditemukan:
                raise RuntimeError("C_MODE mode %s memuat constraint yang tidak ada di DAFTAR_C." % mode)
    return mask


class CBFQP:
    def __init__(self, zeta=None, build: bool = True) -> None:
        """zeta: kemiringan bidang [rad] yang dipakai terus. None = baca config.zeta tiap siklus (bisa berubah).
        build=False memakai kode C yang sudah di-generate sebelumnya (config.py tidak boleh berubah)."""
        self.zeta_tetap = zeta
        self.folder_acados = siapkan_acados()
        from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver

        self.N = int(config.CBF_N_HORIZON)
        x, u, p, f, g, daftar_c, c_mode, self.nama_alpha = _cbf_expressions()
        self.NH = len(daftar_c)                         # jumlah constraint di DAFTAR_C

        # Mask tiap mode sebagai parameter solver: c_efektif = m*c + (1 - m)  (m = 0 -> selalu terpenuhi, a = 0)
        m = ca.SX.sym("m", self.NH)
        c = ca.vertcat(*[m[k] * daftar_c[k] + (1.0 - m[k]) for k in range(self.NH)])
        p_semua = ca.vertcat(p, m)                      # p_semua = [Alpha..., zeta, rasio_torsi, psi_max, mask...]

        # Alpha yang dipakai constraint aktif tiap mode (hanya ini yang wajib > 0 di mode itu)
        self._daftar_c = daftar_c
        self._p = p
        self.mask_mode = _mask_tiap_mode(daftar_c, c_mode)
        self.alpha_mode = {mode: self._alpha_dipakai(self.mask_mode[mode]) for mode in self.mask_mode}

        # c(x, u, p) = a(x, p)*u + b(x, p): dipakai untuk memeriksa hasil solver dan untuk diagnosa
        a = ca.jacobian(c, u)
        b = ca.substitute(c, u, 0.0)
        self._coeff = ca.Function("cbf_coeff", [x, p_semua], [a, b])

        acados_model = AcadosModel()
        acados_model.name = "atera_cbf_qp"
        acados_model.x = x
        acados_model.u = u
        acados_model.p = p_semua
        acados_model.disc_dyn_expr = x + config.DT * (f + g * u)
        acados_model.con_h_expr_0 = c

        ocp = AcadosOcp()
        ocp.model = acados_model
        if hasattr(ocp.solver_options, "N_horizon"):
            ocp.solver_options.N_horizon = self.N
        else:                                          # acados < 0.4
            ocp.dims.N = self.N
        ocp.solver_options.tf = float(self.N)          # langkah waktu 1 -> cost tidak diskalakan
        ocp.parameter_values = np.ones(len(self.nama_alpha) + 3 + self.NH)

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
        ocp.constraints.uh_0 = config.CBF_INFTY * np.ones(self.NH)
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

        # torsi nyata / torsi perintah yang dipakai g(x) (lihat docstring modul); boleh diganti saat berjalan
        self.rasio_torsi = float(config.RASIO_TORSI)

        # mode gerak awal = Balancing
        self.mode = MODE_BALANCING
        self.mask = self.mask_mode[MODE_BALANCING]
        self.alpha_dipakai = self.alpha_mode[MODE_BALANCING]

        # hasil siklus terakhir
        self.status = STATUS_AMAN
        self.acados_status = 0
        self.feasible = True
        self.active = False
        self.c = np.zeros(self.NH)                     # nilai tiap constraint pada u terakhir (>= 0 = terpenuhi)
        self.u_pd = 0.0
        self.u = 0.0
        self.u_interval = (self.u_lb, self.u_ub)
        self.solve_ms = 0.0
        self.count = {STATUS_AMAN: 0, STATUS_GAGAL: 0, STATUS_ALPHA: 0}

    def mode_cbf(self, mode: str) -> None:
        """Pilih constraint CBF sesuai mode gerak (MODE_BALANCING atau MODE_NAVIGATE).

        Balancing : semua constraint di C_MODE[MODE_BALANCING] (h1 .. h8)
        Navigate  : constraint di C_MODE[MODE_NAVIGATE] (tanpa h5, h6, supaya robot boleh berjalan jauh)
        Solver acados tidak dibangun ulang: hanya mask-nya yang berganti. Aman dipanggil tiap siklus.
        """
        if mode == self.mode:
            return
        if mode not in self.mask_mode:
            raise ValueError("mode CBF tidak dikenal: %s" % mode)
        self.mode = mode
        self.mask = self.mask_mode[mode]
        self.alpha_dipakai = self.alpha_mode[mode]

    def _alpha_dipakai(self, mask) -> list:
        """Indeks Alpha yang muncul di constraint dengan mask 1 (hanya Alpha ini yang wajib > 0)."""
        hasil = []
        for i in range(len(self.nama_alpha)):
            for k in range(self.NH):
                if mask[k] > 0.0 and ca.depends_on(self._daftar_c[k], self._p[i]):
                    hasil.append(i)
                    break
        return hasil

    def zeta(self) -> float:
        """Kemiringan bidang yang dipakai sekarang [rad]."""
        if self.zeta_tetap is not None:
            return float(self.zeta_tetap)
        return float(config.zeta)

    def _alphas(self) -> np.ndarray:
        """Nilai Alpha_1, Alpha_2, ... dari config (dibaca tiap siklus)."""
        nilai = []
        for nama in self.nama_alpha:
            nilai.append(float(getattr(config, nama)))
        return np.array(nilai)

    def _parameter(self, alphas=None) -> np.ndarray:
        """p = [Alpha_1, Alpha_2, ..., zeta, rasio_torsi, psi_max, mask_1, ..., mask_NH]."""
        if alphas is None:
            alphas = self._alphas()
        return np.concatenate([np.asarray(alphas, dtype=float), [self.zeta(), self.rasio_torsi, float(config.psi_max)],
                               self.mask])

    def constraint_coefficients(self, x, alphas=None):
        """a, b (masing-masing ndarray(NH)) pada constraint a*u + b >= 0. Constraint yang dimask: a = 0, b = 1."""
        p = self._parameter(alphas)
        a, b = self._coeff(np.asarray(x, dtype=float), p)
        return np.array(a).ravel(), np.array(b).ravel()

    def allowed_interval(self, x, alphas=None):
        """Selang u yang memenuhi semua constraint aktif (termasuk batas arus). lo > hi berarti tidak ada solusi."""
        a, b = self.constraint_coefficients(x, alphas)
        lo, hi = self.u_lb, self.u_ub
        for ak, bk in zip(a, b):
            if ak > config.CBF_TOL_KOEFISIEN:
                lo = max(lo, -bk / ak)
            elif ak < -config.CBF_TOL_KOEFISIEN:
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
            self.c = np.full(self.NH, np.nan)          # CBF-QP tidak dijalankan: nilai constraint tidak diketahui
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
                  and self.u_lb - config.CBF_TOL_BATAS_U <= u <= self.u_ub + config.CBF_TOL_BATAS_U
                  and bool(np.all(a * u + b >= -config.CBF_TOL_CONSTRAINT * scale)))
        if not solved:
            self._fallback(STATUS_GAGAL)
            self._catat_c(a, b)
            return self.u

        self.status = STATUS_AMAN
        self.feasible = True
        self.count[STATUS_AMAN] += 1
        self.active = abs(u - self.u_pd) > config.CBF_TOL_AKTIF
        self.u = u
        self._catat_c(a, b)
        return u


if __name__ == "__main__":
    np.set_printoptions(precision=4, suppress=True)
    for nama in config.NAMA_ALPHA:
        if getattr(config, nama) <= 0.0:
            print("%s di config.py masih 0; uji ini memakai nilai sementara 20." % nama)
            setattr(config, nama, 20.0)
    cbf = CBFQP()
    rng = np.random.default_rng(0)
    for mode in (MODE_BALANCING, MODE_NAVIGATE):
        cbf.mode_cbf(mode)
        print("mode %s | constraint aktif: %d dari %d | Alpha yang dipakai: %s"
              % (mode, int(cbf.mask.sum()), cbf.NH, [cbf.nama_alpha[i] for i in cbf.alpha_dipakai]))
        worst = 0.0
        for _ in range(2000):
            x = rng.uniform(-1.0, 1.0, config.NX) * np.array([5.0, config.dtheta_max, config.psi_max, config.dpsi_max])
            u_pd = rng.uniform(2.0 * config.u_min, 2.0 * config.u_max)
            u = cbf.compute(x, u_pd)
            lo, hi = cbf.u_interval
            if lo <= hi:
                worst = max(worst, abs(u - min(max(u_pd, lo), hi)))
        print("  jumlah per status:", cbf.count)
        print(f"  selisih maksimum u* acados terhadap solusi eksak (kasus feasible): {worst:.2e} N m")
    print(f"waktu solve terakhir: {cbf.solve_ms:.3f} ms")