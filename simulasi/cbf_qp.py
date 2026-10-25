"""cbf_qp.py - safety filter CBF-QP ATERA. QP solver dipilih di config.CBF_QP_SOLVER: acados (HPIPM, N = 1),
plugin ca.conic CasADi, atau "eksak".
Bagian di luar WORKPLACE PROGRAM sama persis di simulasi/ dan program/atera_program/ (WORKPLACE boleh berbeda).

Bagian safety filter selain constraint WORKPLACE:
    1. SLACK  : setiap constraint diberi slack SENDIRI d_k >= 0 dengan bobot rho_k (config.CBF_RHO_SLACK, satu nilai
                per h, urutan DAFTAR_C). Penalti slack LINEAR (exact penalty) + suku kuadrat kecil eps:
                    min  (u - u_PD)^2 + sum_k (rho_k * d_k + eps * d_k^2)          eps = config.CBF_SLACK_EPS
                    s.t. c_k(x, u) + d_k >= 0   untuk setiap constraint,   u_min <= u <= u_max,   d_k >= 0
                Sifat exact penalty: bila rho_k lebih besar dari pengali Lagrange constraint k di QP lama, maka
                d = 0 dan u SAMA PERSIS dengan QP lama setiap kali semua constraint bisa dipenuhi. Bila tidak ada u
                yang memenuhi semua (constraint bertabrakan, atau torsi motor tidak cukup), QP tetap punya solusi dan
                constraint yang bobotnya KECIL dilonggarkan lebih dulu (PRIORITAS). Bobot besar di h1/h2 = psi didahulukan.
                Kenapa tidak persis segway_cbf.py dosen (SATU slack bersama, penalti kuadrat rho*d^2)?
                  - Dosen hanya punya constraint tilt. Dengan h1..h4, satu slack bersama membagi pelanggaran ke semua
                    constraint tanpa prioritas; bobot per constraint membuat psi selalu didahulukan.
                  - Penalti kuadrat membuat constraint yang sedang aktif selalu sedikit dilanggar (turunan rho*d^2 di d = 0
                    adalah 0), jadi status SLACK muncul walau tidak ada tabrakan. Penalti linear tidak.
                  Catatan uji loop tertutup 16-10 (model planar): yang menentukan adalah dtheta_max: dengan 13 rad/s
                  barrier dtheta melarang roda mengejar badan dan robot jatuh di dorongan 10 N, dengan 20 rad/s tidak.
                Status "SLACK" = ada constraint yang dilonggarkan (CSV kolom slack = d_k terbesar).
                CBF_SLACK_AKTIF = False -> d dipaksa 0 (perilaku lama, GAGAL -> u_PD dibatasi arus).
    2. DOB    : Disturbance Observer (class DOB). Model f(x) + g(x)*u tidak tahu dorongan tangan, gesekan, batas
                kecepatan motor, atau torsi motor yang berbeda dari datasheet. DOB menaksir selisihnya:
                    d_hat = low-pass( percepatan terukur - percepatan prediksi model )   untuk ddtheta dan ddpsi
                lalu f(x) di WORKPLACE diganti f(x) + [0, d_hat_dtheta, 0, d_hat_dpsi]. Rumus constraint di WORKPLACE
                TIDAK berubah: lie(h, f) otomatis memuat gangguan. Contoh h1 (HOCBF): L_f^2 h1 = -(f_dpsi + d_hat_dpsi).
                DOB_AKTIF = False -> d_hat tetap dihitung dan dicatat, tetapi TIDAK dipakai CBF-QP.
    3. MARGIN : barrier psi memakai psi_max - CBF_MARGIN_DEG (menutup jeda DOB beberapa ms saat dorongan mulai).
                Garis merah di plot tetap di psi_max (spesifikasi).
    4. SOLVER : config.CBF_QP_SOLVER boleh
                    "acados" : OCP acados N = ACADOS_N_HORIZON (HPIPM), disusun di WORKSPACE ACADOS (di bawah),
                               pengaturannya ACADOS_* di config.py (simulasi 6b, robot fisik 5b)
                    "eksak"  : solusi pasti QP ini dengan numpy (tanpa iterasi)
                    plugin ca.conic CasADi: "daqp", "qpoases", "osqp", "proxqp", "qrqp" (juga "hpipm", "highs",
                               "ipqp", tetapi tidak cocok untuk QP ini, lihat config.CBF_QP_SOLVER_BANDING)
                Semua solver memakai constraint yang SAMA dari WORKPLACE PROGRAM. Bandingkan (waktu dan selisih
                terhadap "eksak"):  python3 cbf_qp.py --banding-solver

    u = torsi SATU roda [N m].
    Alpha_1, Alpha_2, ... diambil dari config.py (semua Alpha_<nomor> yang berurutan mulai dari 1).
    zeta (kemiringan bidang) dan psi_max (batas h1, h2) juga parameter: nilainya dibaca dari config.zeta dan
    config.psi_max tiap siklus, jadi bisa diubah saat program berjalan tanpa membangun ulang solver.

Mode gerak (mode_cbf, pd_control.py): MODE_DIAM, MODE_BALANCING, dan MODE_BEBAS memakai SATU solver yang sama.
Setiap constraint diberi parameter mask m_k (1 = aktif, 0 = dilewati): c_k dipakai sebagai m_k*c_k + (1 - m_k), jadi
constraint dengan m_k = 0 selalu terpenuhi dan tidak ikut membatasi u. Yang berganti saat mode berganti hanya mask,
bukan solver, jadi aman berganti tiap siklus.

Model torsi: u adalah torsi PERINTAH (arus perintah x K_t datasheet). Torsi yang benar-benar bekerja di roda adalah
rasio_torsi * u, dengan rasio_torsi = K_t efektif / K_t datasheet (config.RASIO_TORSI).
g(x) di model CBF-QP dikalikan rasio_torsi, jadi CBF-QP menghitung dengan tenaga roda yang sungguh ada.

Tuning bertahap (simulasi/tuning_optuna.py): pilih_barrier(nomor) mengaktifkan hanya constraint bernomor itu
(1 = h1, dst), tanpa membangun ulang solver, dengan cara yang sama seperti mask mode gerak.

Susunan QP di CasADi / "eksak" (variabel keputusan z = [u, d_1, ..., d_NH]). Setiap c_k affine terhadap u:
c(x, u, p) = a(x, p)*u + b(x, p).
    min_z  1/2 * z' H z + G' z      H = diag(2, 2*eps, ..., 2*eps), G = [-2*u_PD, rho_1, ..., rho_NH]
                                    (= (u - u_PD)^2 + sum (rho_k*d_k + eps*d_k^2) - konstanta)
    s.t.   -b(x, p) <= a(x, p)*u + d <= CBF_INFTY          (matriks constraint [a | I], c + d >= 0)
           [u_min, 0, ..] <= z <= [u_max, d_maks, ..]      (batas arus, slack >= 0; d_maks = 0 bila slack mati)
    a(x, p) dan b(x, p) dihitung dari ekspresi CasADi WORKPLACE (ca.Function) tiap siklus.
    Tidak ada kode C yang di-generate / di-build: solver langsung siap saat CBFQP() dibuat.
Susunan di acados: lihat WORKSPACE ACADOS (cost dan slack sama, constraint c(x, u, p) yang sama di con_h_expr_0).

Kebutuhan: pip install casadi   (plugin qpoases, osqp, proxqp, qrqp, hpipm, daqp, highs, ipqp ikut di paket pip casadi
           3.6 ke atas; yang tidak ada di versi Anda dilaporkan "tidak tersedia" oleh --banding-solver).
           Solver "acados": acados sudah di-build (<folder acados>/lib/libacados.so) dan acados_template terpasang
           (pip install -e <folder acados>/interfaces/acados_template). Folder acados dicari otomatis, lihat
           cari_folder_acados().

Semua nilai fisik dan gain dibaca dari config.py; f(x) dan g(x) dari model.py.
"""

from __future__ import annotations

import ctypes
import glob
import importlib.util
import math
import os
import sys
import time

import casadi as ca
import numpy as np

import config
import model
from pd_control import MODE_BALANCING, MODE_BEBAS, MODE_DIAM

STATUS_AMAN = "AMAN"                # u = u* dari QP solver, semua constraint terpenuhi (slack = 0)
STATUS_SLACK = "SLACK"              # QP selesai, tetapi constraint terpaksa dilonggarkan (slack > 0)
STATUS_GAGAL = "GAGAL"              # QP solver gagal secara numerik (atau tanpa slack: tidak ada u yang memenuhi)
STATUS_ALPHA = "ALPHA_NOL"          # ada Alpha <= 0, CBF-QP tidak dijalankan

# urutan parameter tambahan di vektor p (sesudah semua Alpha)
_P_ZETA, _P_RASIO, _P_PSI_MAX, _P_D_DTHETA, _P_D_DPSI = range(5)
JUMLAH_P_EKSTRA = 5


def workplace(x, u, f, g, zeta, psi_max):
    """Tempat menulis barrier function h dan constraint CBF. Mengembalikan (DAFTAR_C, C_MODE).

    Yang bisa dipakai di dalam WORKPLACE:
        dtheta, psi               : state (simbol); theta = x[config.IDX_THETA], dpsi = x[config.IDX_DPSI] bila perlu
        u                         : torsi satu roda (simbol)
        f, g                      : model dx/dt = f(x) + g(x)*u   (dari model.py, sudah memakai zeta dan rasio_torsi,
                                    u = torsi perintah satu roda)
        zeta                      : kemiringan bidang [rad] (simbol, nilainya dari config.zeta)
        psi_max                   : batas psi [rad] (simbol, nilainya dari config.psi_max tiap siklus)
        lie(h, v)                 : turunan Lie, L_v h = (dh/dx) * v
        Alpha_1, Alpha_2, ...     : semua Alpha_<nomor> di config.py (otomatis, nilainya bisa diubah di jendela Kontrol)
        MODE_DIAM, MODE_BALANCING, MODE_BEBAS : nama mode gerak (kunci C_MODE, ketiganya wajib ada)
        config.<nama>             : nilai lain dari config.py (dtheta_max, MAX_CURRENT_A, ...)
    """
    dtheta = x[config.IDX_DTHETA]
    psi = x[config.IDX_PSI]

    def lie(h, v):
        """Turunan Lie: L_v h = (dh/dx) * v"""
        return ca.jacobian(h, x) @ v

    # =========================================================================================
    #                                   WORKPLACE PROGRAM
    # =========================================================================================
    # FORMAT (salin, lalu ganti h dan Alpha):
    #
    #   CBF orde 1 (h yang turunan pertamanya sudah memuat u, contoh: dtheta):
    #       c_hX = lie(hX, f) + lie(hX, g) * u + Alpha_k * hX
    #
    #   HOCBF orde 2 (h yang baru memuat u di turunan kedua, contoh: psi):
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
    h1 = -psi + psi_max                 # psi <= psi_max
    h2 = psi + psi_max                  # psi >= -psi_max
    h3 = -dtheta + config.dtheta_max    # dtheta <= dtheta_max
    h4 = dtheta + config.dtheta_max     # dtheta >= -dtheta_max

    # 2. Tulis constraint di bawah
    Lf_h1 = lie(h1, f)
    c_h1 = lie(Lf_h1, f) + lie(Lf_h1, g) * u + Alpha_1 * Lf_h1 + Alpha_2 * (Lf_h1 + Alpha_1 * h1)
    Lf_h2 = lie(h2, f)
    c_h2 = lie(Lf_h2, f) + lie(Lf_h2, g) * u + Alpha_1 * Lf_h2 + Alpha_2 * (Lf_h2 + Alpha_1 * h2)

    c_h3 = lie(h3, f) + lie(h3, g) * u + Alpha_3 * h3
    c_h4 = lie(h4, f) + lie(h4, g) * u + Alpha_3 * h4

    # 3. Daftarkan semua constraint yang bisa dipakai
    DAFTAR_C = [c_h1, c_h2, c_h3, c_h4]

    # 4. Pilih constraint yang aktif di tiap mode gerak (urutan di dalam daftar bebas, isinya dari DAFTAR_C)
    C_MODE = {
        MODE_DIAM:      [c_h1, c_h2, c_h3, c_h4],
        MODE_BALANCING: [c_h1, c_h2, c_h3, c_h4],
        MODE_BEBAS:     [c_h1, c_h2, c_h3, c_h4],
    }

    # =========================================================================================
    #                                 AKHIR WORKPLACE PROGRAM
    # =========================================================================================
    return DAFTAR_C, C_MODE


def _cbf_expressions():
    """Simbol x, u, p = [Alpha_1, ..., zeta, rasio_torsi, psi_max, d_dtheta, d_dpsi], f, g, DAFTAR_C, C_MODE dan
    nama Alpha dari WORKPLACE. Constraint: c >= 0."""
    nama_alpha = config.NAMA_ALPHA
    n_alpha = len(nama_alpha)
    x = ca.SX.sym("x", config.NX)
    u = ca.SX.sym("u", config.NU)
    p = ca.SX.sym("p", n_alpha + JUMLAH_P_EKSTRA)
    zeta = p[n_alpha + _P_ZETA]
    rasio_torsi = p[n_alpha + _P_RASIO]
    psi_max = p[n_alpha + _P_PSI_MAX]
    d_dtheta = p[n_alpha + _P_D_DTHETA]           # gangguan ddtheta hasil DOB [rad/s^2]
    d_dpsi = p[n_alpha + _P_D_DPSI]               # gangguan ddpsi hasil DOB [rad/s^2]

    # Alpha_1, Alpha_2, ... dibuat sebagai nama global supaya bisa langsung ditulis di WORKPLACE.
    for i in range(n_alpha):
        globals()[nama_alpha[i]] = p[i]

    f, g = model.fg_casadi(x, zeta)
    g = rasio_torsi * g                            # u = torsi perintah, torsi nyata = rasio_torsi * u (lihat docstring modul)

    # DOB: f yang dipakai WORKPLACE = f model + gangguan taksiran DOB (0 bila DOB_AKTIF = False).
    # Gangguan hanya masuk ke baris percepatan (ddtheta, ddpsi), baris dtheta dan dpsi (kecepatan) tetap.
    gangguan = [0.0] * config.NX
    gangguan[config.IDX_DTHETA] = d_dtheta
    gangguan[config.IDX_DPSI] = d_dpsi
    f = f + ca.vertcat(*gangguan)

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
    for mode in (MODE_DIAM, MODE_BALANCING, MODE_BEBAS):
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


# =============================================================================
# QP solver
# =============================================================================
SOLVER_EKSAK = "eksak"


def _opsi_solver(plugin):
    """Opsi ca.conic tiap plugin: tanpa cetakan di terminal, dan tidak melempar error saat QP tidak punya solusi
    (hasilnya diperiksa sendiri di compute()). Opsi yang tidak dikenal versi casadi Anda -> dibuat tanpa opsi."""
    opsi = {"error_on_fail": False}
    if plugin == "qpoases":
        opsi["printLevel"] = "none"
    elif plugin == "osqp":
        opsi["osqp"] = {"verbose": False, "eps_abs": 1e-9, "eps_rel": 1e-9, "polish": True, "max_iter": 20000}
    elif plugin in ("qrqp", "ipqp"):
        opsi.update({"print_iter": False, "print_header": False, "print_info": False})
    return opsi


def bobot_slack(nh):
    """rho_k tiap constraint (urutan DAFTAR_C) dari config.CBF_RHO_SLACK: satu angka -> semua sama;
    list -> per constraint (bila lebih pendek dari jumlah constraint, nilai terakhir diulang)."""
    rho = config.CBF_RHO_SLACK
    if isinstance(rho, (int, float)):
        return np.full(nh, float(rho))
    rho = [float(v) for v in rho]
    if len(rho) == 0 or min(rho) <= 0.0:
        raise RuntimeError("config.CBF_RHO_SLACK harus angka > 0 atau list angka > 0 (satu per constraint h).")
    return np.array((rho + [rho[-1]] * nh)[:nh])


def buat_solver(plugin, nh):
    """ca.conic untuk QP 1 + nh variabel (u dan satu slack per constraint) dengan nh constraint.
    plugin "eksak" -> None (dipakai qp_eksak)."""
    if plugin == SOLVER_EKSAK:
        return None
    nv = config.NU + nh
    struktur = {"h": ca.Sparsity.dense(nv, nv), "a": ca.Sparsity.dense(nh, nv)}
    try:
        return ca.conic("atera_cbf_qp", plugin, struktur, _opsi_solver(plugin))
    except RuntimeError:
        # casadi lama tidak mengenal sebagian opsi: buat tanpa opsi tambahan (kegagalan tetap ditangkap di compute)
        try:
            return ca.conic("atera_cbf_qp", plugin, struktur)
        except RuntimeError as kesalahan:
            raise RuntimeError("%s | QP solver '%s' tidak tersedia di casadi ini (cek config.CBF_QP_SOLVER, misalnya "
                               "'qpoases' atau 'eksak')" % (str(kesalahan).strip().splitlines()[-1][:90], plugin))


def qp_eksak(a, b, u_pd, u_lb, u_ub, rho, pakai_slack=True, eps=None):
    """Solusi PASTI QP CBF-QP ini (1 variabel u + 1 slack per constraint) tanpa iterasi, dengan numpy.

        min (u - u_pd)^2 + sum_k (rho_k*d_k + eps*d_k^2)   s.t.  a_k*u + b_k + d_k >= 0,  u_lb <= u <= u_ub,  d_k >= 0

    Untuk u tetap, slack terbaik tiap constraint d_k(u) = max(0, l_k(u)),  l_k(u) = -a_k*u - b_k  (garis lurus).
    Jadi QP = minimasi 1 dimensi  J(u) = (u - u_pd)^2 + sum_k (rho_k*phi_k + eps*phi_k^2),  phi_k = max(0, l_k(u)),
    di [u_lb, u_ub] (cembung). Titik l_k(u) = 0 membagi selang menjadi potongan; di tiap potongan himpunan constraint
    yang dilanggar (P) tetap dan J kuadratik biasa, dengan minimum
        u = (2*u_pd - sum_P rho_k*s_k - 2*eps*sum_P s_k*q_k) / (2 + 2*eps*sum_P s_k^2),   s = -a, q = -b
    lalu dijepit ke potongan itu. Minimum terkecil dari semua potongan = solusi QP (pasti, bukan perkiraan).
    Mengembalikan (u, d ndarray(NH), ok). pakai_slack=False: d = 0 dipaksa, ok = False bila tidak ada u yang memenuhi.
    """
    a = np.asarray(a, dtype=float).ravel()
    b = np.asarray(b, dtype=float).ravel()
    rho = np.asarray(rho, dtype=float).ravel() * np.ones(len(a))
    eps = float(config.CBF_SLACK_EPS if eps is None else eps)
    lo, hi = float(u_lb), float(u_ub)
    tol = float(config.CBF_TOL_KOEFISIEN)
    nol = np.zeros(len(a))
    if not pakai_slack:
        # tanpa slack: selang u yang memenuhi semua constraint (QP lama)
        lo_c, hi_c, kosong = lo, hi, False
        for ak, bk in zip(a, b):
            if ak > tol:
                lo_c = max(lo_c, -bk / ak)
            elif ak < -tol:
                hi_c = min(hi_c, -bk / ak)
            elif bk < 0.0:
                kosong = True
        if not kosong and lo_c <= hi_c:
            return float(min(max(u_pd, lo_c), hi_c)), nol, True
        return float(min(max(u_pd, lo), hi)), nol, False
    s, q = -a, -b
    titik = [lo, hi]
    for sk, qk in zip(s, q):
        if abs(sk) > tol and lo < -qk / sk < hi:
            titik.append(-qk / sk)
    titik = np.unique(np.array(titik))
    if len(titik) == 1:                                          # lo == hi
        return float(lo), np.maximum(0.0, s * lo + q), True

    def biaya(u):
        phi = np.maximum(0.0, s * u + q)
        return (u - u_pd) ** 2 + float(np.sum(rho * phi + eps * phi * phi))

    terbaik = (np.inf, lo)
    for p0, p1 in zip(titik[:-1], titik[1:]):
        P = (s * (0.5 * (p0 + p1)) + q) > 0.0                    # constraint yang dilanggar di potongan ini
        u = ((2.0 * u_pd - np.sum(rho[P] * s[P]) - 2.0 * eps * np.sum(s[P] * q[P]))
             / (2.0 + 2.0 * eps * np.sum(s[P] ** 2)))
        u = min(max(u, p0), p1)
        j = biaya(u)
        if j < terbaik[0]:
            terbaik = (j, u)
    u = float(terbaik[1])
    d = np.maximum(0.0, s * u + q)
    d[d < 1e-9 * (1.0 + np.abs(q))] = 0.0                       # sisa pembulatan di titik patah = 0
    return u, d, True


# =========================================================================================
# =========================================================================================
#                                   WORKSPACE ACADOS
#   Solver config.CBF_QP_SOLVER = "acados". Pengaturan ACADOS_* di config.py (simulasi bagian 6b, robot fisik 5b).
#   Constraint TIDAK ditulis di sini: acados memakai c(x, u, p) yang sama dari WORKPLACE PROGRAM.
# =========================================================================================
# =========================================================================================
SOLVER_ACADOS = "acados"
NAMA_OCP_ACADOS = "atera_cbf_qp"


def cari_folder_acados():
    """Folder acados (yang berisi interfaces/acados_template), atau "" bila tidak ditemukan. Urutan pencarian:
    config.ACADOS_SOURCE_DIR, variabel ACADOS_SOURCE_DIR, folder "acados" di sebelah folder program ini (sampai 4
    tingkat ke atas), ~/acados, lalu lokasi acados_template hasil pip install -e."""
    calon = []
    if str(config.ACADOS_SOURCE_DIR).strip():
        calon.append(os.path.expanduser(str(config.ACADOS_SOURCE_DIR).strip()))
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
        raise RuntimeError("folder acados tidak ditemukan. Isi config.ACADOS_SOURCE_DIR, atau taruh folder 'acados' "
                           "di sebelah folder program ini / di ~/acados.")
    lib = os.path.join(folder, "lib")
    if len(glob.glob(os.path.join(lib, "libacados.*"))) == 0:
        raise RuntimeError("acados belum di-build (tidak ada %s/libacados.so). Jalankan:\n"
                           "    cd %s && git submodule update --recursive --init\n"
                           "    mkdir -p build && cd build && cmake -DACADOS_WITH_QPOASES=OFF .. && make install -j4"
                           % (lib, folder))
    os.environ["ACADOS_SOURCE_DIR"] = folder
    os.environ["LD_LIBRARY_PATH"] = lib + os.pathsep + os.environ.get("LD_LIBRARY_PATH", "")
    # Muat library acados lebih dulu, supaya solver hasil generate menemukannya walaupun LD_LIBRARY_PATH tidak
    # diatur sebelum Python dijalankan.
    for nama in ("libblasfeo.so", "libhpipm.so", "libacados.so"):
        path = os.path.join(lib, nama)
        if os.path.exists(path):
            ctypes.CDLL(path, mode=ctypes.RTLD_GLOBAL)
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


def workspace_acados(x, u, p_semua, c, f, g, rho, u_lb, u_ub):
    """AcadosOcp CBF-QP (dipanggil SolverAcados). Mengembalikan (ocp, N).

        stage 0 : state x dikunci ke state terukur (lbx_0 = ubx_0 = x tiap siklus), variabel keputusan u
                  cost LINEAR_LS  1/2*(u - u_PD)' W (u - u_PD), W = 2  ->  (u - u_PD)^2   (yref_0 = u_PD tiap siklus)
                  constraint c(x, u, p)/s >= 0 di con_h_expr_0 (c = WORKPLACE PROGRAM, sudah dimask dan memuat DOB;
                  s_k = max(|a_k|, 1) diisi tiap siklus, jadi c/s bersatuan torsi [N m] dan HPIPM tidak menerima
                  angka ribuan saat dtheta jauh di luar batas)
                  slack (CBF_SLACK_AKTIF): soft constraint pada c/s, cost  zl*e + 1/2*Zl*e^2  dengan e = d/s,
                  zl = rho_k*s_k, Zl = 2*eps*s_k^2  ->  sama persis dengan rho_k*d_k + eps*d_k^2
                  batas arus u_min <= u <= u_max (lbu / ubu)
        stage 1..N : prediksi x_{k+1} = x_k + DT*(f + g*u), tanpa cost dan tanpa constraint
    Karena x dikunci dan c affine terhadap u, SQP_RTI menyelesaikan tepat SATU QP yang sama dengan QP solver lain.
    """
    from acados_template import AcadosModel, AcadosOcp

    nh = int(c.shape[0])
    n_horizon = int(config.ACADOS_N_HORIZON)
    if n_horizon < 1:
        raise RuntimeError("config.ACADOS_N_HORIZON harus >= 1.")

    # ---- 1. Model: x, u, parameter p, dinamika diskrit, constraint stage 0
    model_ocp = AcadosModel()
    model_ocp.name = NAMA_OCP_ACADOS
    model_ocp.x = x
    model_ocp.u = u
    model_ocp.p = p_semua
    model_ocp.disc_dyn_expr = x + config.DT * (f + g * u)
    model_ocp.con_h_expr_0 = c

    ocp = AcadosOcp()
    ocp.model = model_ocp
    if hasattr(ocp.solver_options, "N_horizon"):
        ocp.solver_options.N_horizon = n_horizon
    else:                                           # acados < 0.4
        ocp.dims.N = n_horizon
    ocp.solver_options.tf = float(n_horizon)        # langkah waktu 1 -> cost tidak diskalakan
    ocp.parameter_values = np.zeros(int(p_semua.shape[0]))

    # ---- 2. Cost: (u - u_PD)^2 hanya di stage 0
    ocp.cost.cost_type_0 = "LINEAR_LS"
    ocp.cost.Vx_0 = np.zeros((config.NU, config.NX))
    ocp.cost.Vu_0 = np.eye(config.NU)
    ocp.cost.W_0 = 2.0 * np.eye(config.NU)
    ocp.cost.yref_0 = np.zeros(config.NU)
    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.Vx = np.zeros((0, config.NX))
    ocp.cost.Vu = np.zeros((0, config.NU))
    ocp.cost.W = np.zeros((0, 0))
    ocp.cost.yref = np.zeros(0)
    ocp.cost.cost_type_e = "LINEAR_LS"
    ocp.cost.Vx_e = np.zeros((0, config.NX))
    ocp.cost.W_e = np.zeros((0, 0))
    ocp.cost.yref_e = np.zeros(0)

    # ---- 3. Constraint: x terukur, c >= 0, batas arus
    # Batas atas c "tak hingga": HPIPM memakai ACADOS_INFTY (batas itu tidak dimasukkan ke QP = constraint satu sisi).
    # Dengan angka besar biasa (1e9) HPIPM ikut menghitung batas atas itu dan sering gagal numerik (NaN).
    try:
        from acados_template import ACADOS_INFTY as tak_hingga
    except ImportError:                             # acados lama
        tak_hingga = 1.0e10
    if "HPIPM" not in str(config.ACADOS_QP_SOLVER):
        tak_hingga = float(config.CBF_INFTY)        # QP solver lain tidak menerima constraint satu sisi
    ocp.constraints.x0 = np.zeros(config.NX)                     # diganti state terukur tiap siklus
    ocp.constraints.lh_0 = np.zeros(nh)
    ocp.constraints.uh_0 = float(tak_hingga) * np.ones(nh)
    ocp.constraints.idxbu = np.arange(config.NU)
    ocp.constraints.lbu = np.array([float(u_lb)])
    ocp.constraints.ubu = np.array([float(u_ub)])

    # ---- 4. Slack per constraint (sama dengan QP solver lain: rho_k*d_k + CBF_SLACK_EPS*d_k^2)
    if config.CBF_SLACK_AKTIF:
        if not hasattr(ocp.constraints, "idxsh_0"):
            raise RuntimeError("acados versi ini belum punya soft constraint di stage 0 (idxsh_0). Perbarui acados, "
                               "atau isi CBF_SLACK_AKTIF = False.")
        # nilai awal; tiap siklus diisi ulang SolverAcados.selesaikan() sesuai skala s_k
        z_lin = np.asarray(rho, dtype=float)
        z_kuad = 2.0 * float(config.CBF_SLACK_EPS) * np.ones(nh)
        ocp.constraints.idxsh_0 = np.arange(nh)
        ocp.cost.zl_0 = z_lin
        ocp.cost.zu_0 = z_lin
        ocp.cost.Zl_0 = z_kuad
        ocp.cost.Zu_0 = z_kuad

    # ---- 5. Opsi solver (config.py bagian ACADOS)
    opsi = ocp.solver_options
    opsi.integrator_type = "DISCRETE"
    opsi.nlp_solver_type = str(config.ACADOS_NLP_SOLVER)
    opsi.qp_solver = str(config.ACADOS_QP_SOLVER)
    opsi.hessian_approx = "GAUSS_NEWTON"
    opsi.qp_solver_iter_max = int(config.ACADOS_QP_ITER_MAX)
    if "HPIPM" in opsi.qp_solver:
        opsi.hpipm_mode = str(config.ACADOS_HPIPM_MODE)
        # mu0 = parameter barrier awal HPIPM. Bobot slack rho_k*s_k bisa ~1e8: dengan mu0 bawaan (1) HPIPM sering
        # berhenti dengan NaN tepat saat slack dibutuhkan. Hanya ada di acados yang cukup baru.
        if float(config.ACADOS_QP_MU0) > 0.0 and isinstance(getattr(type(opsi), "qp_solver_mu0", None), property):
            opsi.qp_solver_mu0 = float(config.ACADOS_QP_MU0)
    opsi.print_level = int(config.ACADOS_PRINT_LEVEL)
    return ocp, n_horizon


class SolverAcados:
    """Pembungkus AcadosOcpSolver untuk CBFQP: selesaikan(x, p, u_pd, a) -> (u, d, ok, teks)."""

    def __init__(self, x, u, p_semua, c, f, g, rho, u_lb, u_ub):
        siapkan_acados()
        from acados_template import AcadosOcpSolver

        self.nh = int(c.shape[0])
        self.rho = np.asarray(rho, dtype=float)
        self.eps = float(config.CBF_SLACK_EPS)
        self.pakai_slack = bool(config.CBF_SLACK_AKTIF)
        self.u_lb, self.u_ub = float(u_lb), float(u_ub)
        # Skala tiap constraint s_k (parameter tambahan di belakang p): acados menyelesaikan c/s >= 0
        skala = ca.SX.sym("skala", self.nh)
        ocp, self.N = workspace_acados(x, u, ca.vertcat(p_semua, skala), c / skala, f, g, rho, u_lb, u_ub)

        folder_kode = str(config.ACADOS_FOLDER_KODE)
        if not os.path.isabs(folder_kode):
            folder_kode = os.path.join(os.path.dirname(os.path.abspath(__file__)), folder_kode)
        nama_json = "acados_ocp_%s.json" % NAMA_OCP_ACADOS
        path_json = os.path.join(folder_kode, nama_json)
        # ACADOS_BUILD = False dan kode C sudah ada: langsung dipakai (cepat). Selain itu kode C dibuat ulang.
        bangun = bool(config.ACADOS_BUILD) or not os.path.exists(path_json)
        if hasattr(ocp, "code_gen_options"):         # acados baru
            ocp.code_gen_options.code_export_directory = folder_kode
            ocp.code_gen_options.json_file = path_json
            self.solver = AcadosOcpSolver(ocp, build=bangun, generate=bangun, verbose=False)
        else:
            ocp.code_export_directory = folder_kode
            self.solver = AcadosOcpSolver(ocp, json_file=path_json, build=bangun, generate=bangun, verbose=False)

    def selesaikan(self, x, p, u_pd, a):
        """x, p (parameter WORKPLACE), u_pd, a = koefisien u tiap constraint (dari CBFQP.constraint_coefficients)."""
        solver = self.solver
        skala = np.maximum(np.abs(np.asarray(a, dtype=float)), 1.0)
        p_acados = np.concatenate([p, skala])
        solver.set(0, "lbx", x)
        solver.set(0, "ubx", x)
        for stage in range(self.N + 1):
            solver.set(stage, "x", x)
            solver.set(stage, "p", p_acados)
        solver.set(0, "u", np.array([min(max(u_pd, self.u_lb), self.u_ub)]))
        solver.cost_set(0, "yref", np.array([u_pd]))
        if self.pakai_slack:
            z_lin = self.rho * skala
            z_kuad = 2.0 * self.eps * skala ** 2
            for nama, nilai in (("zl", z_lin), ("zu", z_lin), ("Zl", z_kuad), ("Zu", z_kuad)):
                solver.cost_set(0, nama, nilai)
        status = int(solver.solve())
        u = float(solver.get(0, "u")[0])
        if self.pakai_slack:
            d = np.asarray(solver.get(0, "sl"), dtype=float).ravel()[:self.nh] * skala
        else:
            d = np.zeros(self.nh)
        return u, d, status == 0, "OK" if status == 0 else "acados status %d" % status


# =========================================================================================
#                                 AKHIR WORKSPACE ACADOS
# =========================================================================================


# =============================================================================
# DOB (Disturbance Observer)
# =============================================================================
class DOB:
    """Penaksir gangguan percepatan d_hat = [d_dtheta, d_dpsi] [rad/s^2].

    Tiap siklus kontrol (urutan WAJIB):
        dob.perbarui(x, dt)    di awal siklus, dengan state baru x
        ... hitung u ...
        dob.catat(x, u)        setelah u dikirim ke motor (u = torsi perintah SATU roda yang benar-benar dipakai)

    perbarui():  percepatan terukur  = (x_baru - x_lalu) / dt                        (dtheta dari encoder, dpsi gyro)
                 percepatan prediksi = f(x_lalu) + g(x_lalu) * rasio_torsi * u_lalu  (model.py, TANPA gangguan)
                 sisa = terukur - prediksi  -> d_hat += a*(sisa - d_hat),  a = dt / (DOB_TAU_S + dt)  (low-pass)
    Yang tertangkap: dorongan / tarikan tangan, lantai miring yang tidak diketahui model, gesekan, torsi motor
    yang lebih kecil dari datasheet, batas kecepatan motor (back-EMF). Noise turunan gyro diredam DOB_TAU_S.
    """

    def __init__(self):
        self.reset()

    def reset(self):
        self.d = np.zeros(2)                        # [d_dtheta, d_dpsi]
        self._x_lalu = None
        self._a_prediksi = None

    def perbarui(self, x, dt):
        x = np.asarray(x, dtype=float).ravel()
        if self._x_lalu is not None and self._a_prediksi is not None and dt > 1e-6:
            a_ukur = np.array([x[config.IDX_DTHETA] - self._x_lalu[config.IDX_DTHETA],
                               x[config.IDX_DPSI] - self._x_lalu[config.IDX_DPSI]]) / dt
            sisa = a_ukur - self._a_prediksi
            bobot = dt / (float(config.DOB_TAU_S) + dt)
            self.d = self.d + bobot * (sisa - self.d)
            batas = float(config.DOB_BATAS)
            self.d = np.clip(self.d, -batas, batas)
        self._x_lalu = x.copy()
        self._a_prediksi = None                     # wajib diisi catat() di siklus ini

    def catat(self, x, u, rasio_torsi, zeta):
        x = np.asarray(x, dtype=float).ravel()
        dx = model.f(x, zeta) + model.g(x, zeta) * float(rasio_torsi) * float(u)
        self._a_prediksi = np.array([dx[config.IDX_DTHETA], dx[config.IDX_DPSI]])


# =============================================================================
# CBF-QP
# =============================================================================
class CBFQP:
    def __init__(self, zeta=None, plugin=None) -> None:
        """zeta: kemiringan bidang [rad] yang dipakai terus. None = baca config.zeta tiap siklus (bisa berubah).
        plugin: QP solver (None = config.CBF_QP_SOLVER)."""
        self.zeta_tetap = zeta
        x, u, p, f, g, daftar_c, c_mode, self.nama_alpha = _cbf_expressions()
        self.NH = len(daftar_c)                         # jumlah constraint di DAFTAR_C

        # Mask tiap mode sebagai parameter: c_efektif = m*c + (1 - m)  (m = 0 -> selalu terpenuhi, a = 0)
        m = ca.SX.sym("m", self.NH)
        c = ca.vertcat(*[m[k] * daftar_c[k] + (1.0 - m[k]) for k in range(self.NH)])
        p_semua = ca.vertcat(p, m)                      # p_semua = [Alpha..., zeta, rasio, psi_max, d_dtheta, d_dpsi, mask...]

        # Alpha yang dipakai constraint aktif tiap mode (hanya ini yang wajib > 0 di mode itu)
        self._daftar_c = daftar_c
        self._p = p
        self.mask_mode = _mask_tiap_mode(daftar_c, c_mode)
        self.alpha_mode = {mode: self._alpha_dipakai(self.mask_mode[mode]) for mode in self.mask_mode}

        # c(x, u, p) = a(x, p)*u + b(x, p): koefisien QP tiap siklus
        a = ca.jacobian(c, u)
        b = ca.substitute(c, u, 0.0)
        if ca.depends_on(a, u):
            raise RuntimeError("constraint di WORKPLACE PROGRAM tidak affine terhadap u (ada u^2, sin(u), ...). "
                               "CBF-QP butuh c = a(x)*u + b(x).")
        self._coeff = ca.Function("cbf_coeff", [x, p_semua], [a, b])

        self.u_lb = float(config.MIN_CURRENT_A * config.MOTOR_KT)       # batas arus: u_min <= u <= u_max
        self.u_ub = float(config.MAX_CURRENT_A * config.MOTOR_KT)

        # QP solver (1 + NH variabel: u dan satu slack per constraint, bobot rho_k)
        self.plugin = str(config.CBF_QP_SOLVER if plugin is None else plugin)
        self.rho = bobot_slack(self.NH)
        if self.plugin == SOLVER_ACADOS:
            self.solver = SolverAcados(x, u, p_semua, c, f, g, self.rho, self.u_lb, self.u_ub)
        else:
            self.solver = buat_solver(self.plugin, self.NH)
        self._uba = np.full(self.NH, float(config.CBF_INFTY))          # c + d <= CBF_INFTY ("tak hingga")

        # torsi nyata / torsi perintah yang dipakai g(x) (lihat docstring modul); boleh diganti saat berjalan
        self.rasio_torsi = float(config.RASIO_TORSI)

        # DOB
        self.dob = DOB()

        # mode gerak awal = DIAM
        self.mode = MODE_DIAM
        self.mask = self.mask_mode[MODE_DIAM]
        self.alpha_dipakai = self.alpha_mode[MODE_DIAM]

        # hasil siklus terakhir
        self.status = STATUS_AMAN
        self.solver_ok = True                          # hasil QP solver (sebelum diperiksa ulang)
        self.solver_status = ""                        # teks status dari QP solver
        self.feasible = True                           # True = semua constraint terpenuhi (status AMAN)
        self.active = False
        self.c = np.zeros(self.NH)                     # nilai tiap constraint pada u terakhir (>= 0 = terpenuhi, TANPA slack)
        self.slack = 0.0                               # slack terbesar siklus terakhir
        self.slack_vektor = np.zeros(self.NH)          # slack tiap constraint (urutan DAFTAR_C)
        self.u_pd = 0.0
        self.u = 0.0
        self.u_interval = (self.u_lb, self.u_ub)
        self.solve_ms = 0.0
        self.count = {STATUS_AMAN: 0, STATUS_SLACK: 0, STATUS_GAGAL: 0, STATUS_ALPHA: 0}

    # ------------------------------------------------------------------ mode
    def mode_cbf(self, mode: str) -> None:
        """Pilih constraint CBF sesuai mode gerak: constraint di C_MODE[mode] (MODE_DIAM, MODE_BALANCING, MODE_BEBAS).
        Solver tidak dibangun ulang: hanya mask-nya yang berganti. Aman dipanggil tiap siklus.
        """
        if mode == self.mode:
            return
        if mode not in self.mask_mode:
            raise ValueError("mode CBF tidak dikenal: %s" % mode)
        self.mode = mode
        self.mask = self.mask_mode[mode]
        self.alpha_dipakai = self.alpha_mode[mode]

    def pilih_barrier(self, nomor) -> None:
        """Aktifkan HANYA constraint bernomor `nomor` (1 = constraint pertama di DAFTAR_C, yaitu h1, dst).

        Dipakai tuning bertahap (tuning_optuna.py) dan kotak centang simulasi; mode gerak diabaikan sampai
        mode_cbf() dipanggil lagi. Solver tidak dibangun ulang: hanya mask-nya yang berganti.
        """
        mask = np.zeros(self.NH)
        for n in nomor:
            if not 1 <= n <= self.NH:
                raise ValueError("nomor constraint %s di luar 1 .. %d" % (n, self.NH))
            mask[n - 1] = 1.0
        self.mode = None
        self.mask = mask
        self.alpha_dipakai = self._alpha_dipakai(mask)

    def _alpha_dipakai(self, mask) -> list:
        """Indeks Alpha yang muncul di constraint dengan mask 1 (hanya Alpha ini yang wajib > 0)."""
        hasil = []
        for i in range(len(self.nama_alpha)):
            for k in range(self.NH):
                if mask[k] > 0.0 and ca.depends_on(self._daftar_c[k], self._p[i]):
                    hasil.append(i)
                    break
        return hasil

    # ------------------------------------------------------------------- DOB
    def reset_dob(self) -> None:
        """Kosongkan DOB: dipanggil saat MULAI, KOMET / roda mati, dan robot diletakkan ulang (simulasi)."""
        self.dob.reset()

    def dob_perbarui(self, x, dt) -> None:
        """Awal siklus kontrol (semua mode, juga mode PD supaya d_hat selalu siap dan tercatat)."""
        self.dob.perbarui(x, dt)

    def dob_catat(self, x, u) -> None:
        """Akhir siklus: u = torsi perintah SATU roda yang dikirim (setelah batas arus)."""
        self.dob.catat(x, u, self.rasio_torsi, self.zeta())

    @property
    def d_hat(self):
        """(d_dtheta, d_dpsi) taksiran DOB [rad/s^2]."""
        return float(self.dob.d[0]), float(self.dob.d[1])

    # ------------------------------------------------------------- parameter
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

    def psi_max_barrier(self) -> float:
        """psi_max yang dipakai barrier = config.psi_max - CBF_MARGIN_DEG (tidak kurang dari 0)."""
        return max(0.0, float(config.psi_max) - math.radians(float(config.CBF_MARGIN_DEG)))

    def _parameter(self, alphas=None) -> np.ndarray:
        """p = [Alpha_1, ..., zeta, rasio_torsi, psi_max, d_dtheta, d_dpsi, mask_1, ..., mask_NH]."""
        if alphas is None:
            alphas = self._alphas()
        d = self.dob.d if config.DOB_AKTIF else np.zeros(2)
        return np.concatenate([np.asarray(alphas, dtype=float),
                               [self.zeta(), self.rasio_torsi, self.psi_max_barrier(), float(d[0]), float(d[1])],
                               self.mask])

    def constraint_coefficients(self, x, alphas=None):
        """a, b (masing-masing ndarray(NH)) pada constraint a*u + b >= 0. Constraint yang dimask: a = 0, b = 1."""
        p = self._parameter(alphas)
        a, b = self._coeff(np.asarray(x, dtype=float), p)
        return np.array(a).ravel(), np.array(b).ravel()

    def _interval(self, a, b):
        lo, hi = self.u_lb, self.u_ub
        for ak, bk in zip(a, b):
            if ak > config.CBF_TOL_KOEFISIEN:
                lo = max(lo, -bk / ak)
            elif ak < -config.CBF_TOL_KOEFISIEN:
                hi = min(hi, -bk / ak)
            elif bk < 0.0:
                return np.inf, -np.inf
        return lo, hi

    # -------------------------------------------------------------- hitung u
    def _fallback(self, status: str) -> float:
        # ASUMSI: bila CBF-QP tidak dijalankan (ALPHA_NOL) atau solver gagal numerik, u = u_PD yang disaturasi ke
        # batas arus. Kejadiannya tercatat di self.status / self.count supaya terlihat di log dan CSV.
        self.status = status
        self.feasible = False
        self.active = False
        self.slack = 0.0
        self.slack_vektor = np.zeros(self.NH)
        self.count[status] += 1
        self.u = float(min(max(self.u_pd, self.u_lb), self.u_ub))
        return self.u

    def _catat_c(self, a, b):
        self.c = a * self.u + b

    def _selesaikan(self, a, b, x, p):
        """QP u + slack per constraint. Mengembalikan (u, d ndarray(NH), solver_ok, teks status)."""
        pakai_slack = bool(config.CBF_SLACK_AKTIF)
        if isinstance(self.solver, SolverAcados):                       # "acados" (WORKSPACE ACADOS)
            return self.solver.selesaikan(x, p, self.u_pd, a)
        if self.solver is None:                                         # "eksak"
            u, d, ok = qp_eksak(a, b, self.u_pd, self.u_lb, self.u_ub, self.rho, pakai_slack)
            return u, d, ok, "eksak" if ok else "tidak ada u yang memenuhi"
        d_maks = float(config.CBF_INFTY) if pakai_slack else 0.0
        A = np.hstack([a.reshape(self.NH, 1), np.eye(self.NH)])
        try:
            eps = float(config.CBF_SLACK_EPS)
            hasil = self.solver(h=np.diag(np.concatenate([[2.0], np.full(self.NH, 2.0 * eps)])),
                                g=np.concatenate([[-2.0 * self.u_pd], self.rho]), a=A,
                                lba=-b, uba=self._uba, lbx=np.concatenate([[self.u_lb], np.zeros(self.NH)]),
                                ubx=np.concatenate([[self.u_ub], np.full(self.NH, d_maks)]))
            z = np.asarray(hasil["x"], dtype=float).ravel()
            u, d = float(z[0]), z[1:1 + self.NH]
            statistik = self.solver.stats()
            ok = bool(statistik.get("success", True))
            teks = str(statistik.get("return_status", "OK" if ok else "gagal"))
        except RuntimeError as kesalahan:                               # QP tidak punya solusi / gagal numerik
            u, d, ok = float("nan"), np.full(self.NH, np.nan), False
            teks = str(kesalahan).strip().splitlines()[-1][:80] if str(kesalahan).strip() else "error"
        return u, d, ok, teks

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
            self.solver_ok = False
            self.solver_status = "tidak dijalankan (Alpha = 0)"
            self.u_interval = (self.u_lb, self.u_ub)
            self.solve_ms = 0.0
            self.c = np.full(self.NH, np.nan)          # CBF-QP tidak dijalankan: nilai constraint tidak diketahui
            return self._fallback(STATUS_ALPHA)

        a, b = self.constraint_coefficients(x, p[:len(self.nama_alpha)])
        self.u_interval = self._interval(a, b)

        t0 = time.perf_counter()
        u, d, self.solver_ok, self.solver_status = self._selesaikan(a, b, x, p)
        self.solve_ms = (time.perf_counter() - t0) * 1000.0

        # Periksa ulang hasil solver (jangan percaya begitu saja): batas u, slack >= 0, dan c + d >= 0.
        scale = np.maximum(1.0, np.abs(a) + np.abs(b))
        solved = (self.solver_ok and np.isfinite(u) and bool(np.all(np.isfinite(d)))
                  and bool(np.all(d >= -config.CBF_TOL_BATAS_U))
                  and self.u_lb - config.CBF_TOL_BATAS_U <= u <= self.u_ub + config.CBF_TOL_BATAS_U
                  and bool(np.all(a * u + b + np.maximum(d, 0.0) >= -config.CBF_TOL_CONSTRAINT * scale)))
        if not solved:
            self._fallback(STATUS_GAGAL)
            self._catat_c(a, b)
            return self.u

        u = float(min(max(u, self.u_lb), self.u_ub))
        self.slack_vektor = np.maximum(0.0, np.asarray(d, dtype=float))
        self.slack = float(np.max(self.slack_vektor)) if self.NH else 0.0
        if self.slack > config.CBF_TOL_SLACK:
            self.status = STATUS_SLACK                  # constraint dilonggarkan, u = paling sedikit melanggar
            self.feasible = False
        else:
            self.status = STATUS_AMAN
            self.feasible = True
        self.count[self.status] += 1
        self.active = abs(u - self.u_pd) > config.CBF_TOL_AKTIF
        self.u = u
        self._catat_c(a, b)
        return u


# =============================================================================
# Uji mandiri:  python3 cbf_qp.py                 (solver config.CBF_QP_SOLVER dibanding solusi eksak)
#               python3 cbf_qp.py --banding-solver (semua QP solver di config.CBF_QP_SOLVER_BANDING)
# =============================================================================
def _sampel_uji(cbf, jumlah, seed=0):
    """State acak di sekitar safety set + u_PD acak + gangguan DOB acak (termasuk kasus yang butuh slack)."""
    rng = np.random.default_rng(seed)
    hasil = []
    for _ in range(jumlah):
        x = rng.uniform(-1.3, 1.3, config.NX) * np.array([20.0, config.dtheta_max, config.psi_max, 2.0 * math.pi])
        u_pd = rng.uniform(2.0 * config.u_min, 2.0 * config.u_max)
        d = rng.normal(0.0, 20.0, 2)
        hasil.append((x, u_pd, d))
    return hasil


def _uji_solver(plugin, sampel, mode):
    """(tersedia, teks) untuk satu plugin: selisih terhadap qp_eksak, persen gagal, waktu solve."""
    try:
        cbf = CBFQP(plugin=plugin)
    except Exception as kesalahan:
        return False, "tidak tersedia (%s)" % str(kesalahan).strip().splitlines()[-1].split(" | ")[0][:90]
    cbf.mode_cbf(mode)
    selisih_u, selisih_j, waktu, gagal, pakai_slack = [], [], [], 0, 0
    for x, u_pd, d in sampel:
        cbf.dob.d = d.copy()
        u = cbf.compute(x, u_pd)
        waktu.append(cbf.solve_ms)
        if cbf.status == STATUS_GAGAL:
            gagal += 1
            continue
        a, b = cbf.constraint_coefficients(x)
        u_e, d_e, _ = qp_eksak(a, b, u_pd, cbf.u_lb, cbf.u_ub, cbf.rho, bool(config.CBF_SLACK_AKTIF))
        pakai_slack += bool(np.max(d_e) > config.CBF_TOL_SLACK)
        eps = float(config.CBF_SLACK_EPS)
        j_e = (u_e - u_pd) ** 2 + float(np.sum(cbf.rho * d_e + eps * d_e ** 2))
        d_s = cbf.slack_vektor
        j = (u - u_pd) ** 2 + float(np.sum(cbf.rho * d_s + eps * d_s ** 2))
        selisih_u.append(abs(u - u_e))
        selisih_j.append((j - j_e) / max(1.0, abs(j_e)))
    n = len(sampel)
    teks = ("gagal %5.1f %% | |u - u_eksak| maks %.1e N m | biaya relatif maks %+.1e | solve rata %.3f ms, maks %.3f ms"
            " | kasus butuh slack %d" % (100.0 * gagal / n, max(selisih_u, default=float("nan")),
                                         max(selisih_j, default=float("nan")), np.mean(waktu), np.max(waktu),
                                         pakai_slack))
    return True, teks


if __name__ == "__main__":
    np.set_printoptions(precision=4, suppress=True)
    for nama in config.NAMA_ALPHA:
        if getattr(config, nama) <= 0.0:
            print("%s di config.py masih 0; uji ini memakai nilai sementara 20." % nama)
            setattr(config, nama, 20.0)
    if "--banding-solver" in sys.argv[1:]:
        cbf = CBFQP(plugin=SOLVER_EKSAK)
        sampel = _sampel_uji(cbf, 3000)
        print("Perbandingan QP solver CBF-QP (3000 kasus acak, mode DIAM, slack %s, rho %s)"
              % ("AKTIF" if config.CBF_SLACK_AKTIF else "MATI", " ".join("%.0e" % r for r in cbf.rho)))
        print("Batas waktu satu siklus kontrol: %.1f ms" % (1000.0 * config.DT))
        # acados selalu terakhir: library HPIPM milik acados (dimuat global) bentrok dengan plugin "hpipm" CasADi
        # bila keduanya dipakai di satu program (crash "double free").
        daftar = [s for s in config.CBF_QP_SOLVER_BANDING if s != SOLVER_ACADOS]
        daftar += [SOLVER_ACADOS] if SOLVER_ACADOS in config.CBF_QP_SOLVER_BANDING else []
        for plugin in daftar:
            _, teks = _uji_solver(plugin, sampel, MODE_DIAM)
            print("  %-8s %s" % (plugin, teks))
        raise SystemExit(0)

    cbf = CBFQP()
    print("QP solver: %s | slack %s (rho %s) | DOB %s | margin psi %.1f deg"
          % (cbf.plugin, "AKTIF" if config.CBF_SLACK_AKTIF else "MATI", " ".join("%.0e" % r for r in cbf.rho),
             "AKTIF" if config.DOB_AKTIF else "MATI (hanya dicatat)", config.CBF_MARGIN_DEG))
    for mode in (MODE_DIAM, MODE_BALANCING, MODE_BEBAS):
        cbf.mode_cbf(mode)
        print("mode %s | constraint aktif: %d dari %d | Alpha yang dipakai: %s"
              % (mode, int(cbf.mask.sum()), cbf.NH, [cbf.nama_alpha[i] for i in cbf.alpha_dipakai]))
        _, teks = _uji_solver(cbf.plugin, _sampel_uji(cbf, 2000), mode)
        print("  " + teks)