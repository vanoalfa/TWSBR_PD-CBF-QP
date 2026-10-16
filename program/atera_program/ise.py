"""ise.py - Evaluasi robot fisik ATERA: perekaman, ISE, grafik, dan perbandingan PD vs PD+CBF-QP.

Folder hasil (di dalam atera_program, HHmmSS = jam saat disimpan):
    PLOT_EVALUASI/ATERA_RASPI/PERCOBAAN/PD/     YYYYMMDD_HHmmSS_COBA PD_PERCOBAAN KE-XX.csv / .png / .eps
    PLOT_EVALUASI/ATERA_RASPI/PERCOBAAN/CBFQP/  YYYYMMDD_HHmmSS_COBA PD+CBF_PERCOBAAN KE-XX.csv / .png / .eps
    PLOT_EVALUASI/ATERA_RASPI/PENGUJIAN/PD/ dan .../PENGUJIAN/CBFQP/   (L3, DURASI_PENGUJIAN detik)
    PLOT_EVALUASI/ATERA_RASPI/<PERCOBAAN atau PENGUJIAN>/BANDING/      (hasil --bandingkan)
Nomor KE-XX naik otomatis per tanggal, mode, dan jenis. Kolom CSV sama dengan simulasi/ise.py.
[REVISI 16-10] Kolom CSV tambahan (KOLOM_EKSTRA, sesudah zeta_DEG): yaw_rad, dyaw_rad_s, yaw_ref_rad (PD heading),
theta_ref_rad, dtheta_ref_rad_s (setpoint bergerak), u_belok_Nm, dob_dtheta_rad_s2, dob_dpsi_rad_s2 (DOB), slack.
Status CBF baru "SLACK" (constraint dilonggarkan) dihitung di metrik. Tidak ada grafik baru.
Penyimpanan saat robot masih balancing berjalan di PROSES TERPISAH (simpan(latar=True)) supaya loop kontrol tidak
tertahan matplotlib.
Gambar: .png dan .eps (vektor, untuk LaTeX). Batas sumbu tetap (config.PLOT_BATAS): psi +-1 rad, dpsi +-10 rad/s,
theta +-60 rad, dtheta +-30 rad/s, s +-3 m. Barrier yang digambar: psi, dpsi, dtheta, s. Warna garis:
    merah          : barrier CBF-QP yang aktif (kolom c_h tidak kosong; psi_max dari CSV kolom psi_max_DEG)
    abu-abu tipis  : barrier yang constraint-nya tidak dipakai di mode gerak itu
    hitam          : batas KOMET (|psi| = SAFE_TILT_DEG, |s| = S_KOMET)
    kuning, oranye : penanda |psi| = 5 deg dan 10 deg (config.PLOT_PENANDA_PSI), hanya gambar

Evaluasi (sesuai rumusan masalah skripsi):
    1. kestabilan pitch  : psi(t), dpsi(t), ISE psi dan dpsi, RMS psi, |psi| maks
    2. PD vs PD+CBF-QP   : --bandingkan, grafik kedua mode ditumpuk + tabel selisih
    3. reaksi gangguan   : PENGUJIAN (L3, robot didorong tangan): |psi| maks, waktu pulih,
                           lama di luar safety set, koreksi CBF-QP
Metrik (lihat metrik()):
    ISE x          = jumlah (setpoint - x)^2 * dt
    RMS psi        = akar(ISE psi / durasi)
    lama di luar   = total waktu |x| > batas safety set (psi_max dari CSV, dpsi, dtheta, s)
    waktu pulih    = dari saat |psi| maksimum sampai |psi| < EVAL_PSI_PULIH_DEG terus-menerus EVAL_PULIH_LAMA_S detik
    CBF mengoreksi = persen waktu u berbeda dari u_PD (yang dibatasi arus), hanya mode PD+CBF

Perintah (tambahkan --pengujian untuk hasil PENGUJIAN):
    python3 ise.py --daftar                                         daftar dan metrik semua percobaan
    python3 ise.py --bandingkan                                     PD terbaru vs PD+CBF terbaru
    python3 ise.py --bandingkan --pd 3 --cbf 5 [--tanggal YYYYMMDD] PD KE-03 vs PD+CBF KE-05
    python3 ise.py --gambar-ulang                                   buat ulang PNG + EPS semua CSV (format terbaru)

Plot realtime (python3 atera_main.py --plot): proses_plot() di bawah, berjalan di proses terpisah. Isinya sama dengan
plot realtime simulasi: Gambar 1 psi - dpsi, Gambar 2 theta - dtheta, Gambar 3 s, dan arus terhadap waktu.
"""

import argparse
import csv
import logging
import math
import multiprocessing
import os
import re
from datetime import datetime

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
plt.rcParams["legend.framealpha"] = 1.0     # EPS tidak mendukung transparansi
plt.rcParams["axes.axisbelow"] = True       # grid di bawah garis batas

import config

LOGGER = logging.getLogger(__name__)
FOLDER_PLOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), config.ISE_PLOT_DIR)

MODE_PD = "PD"
MODE_PD_CBF = "PD+CBF"
FOLDER_MODE = {MODE_PD: "PD", MODE_PD_CBF: "CBFQP"}
WARNA_MODE = {MODE_PD: config.PLOT_WARNA_PD, MODE_PD_CBF: config.PLOT_WARNA_PD_CBF}
JENIS_PERCOBAAN = "PERCOBAAN"       # MULAI sampai X / R3 / KOMET
JENIS_PENGUJIAN = "PENGUJIAN"       # L3: DURASI_PENGUJIAN detik sejak tombol ditekan

KOLOM_GAIN_PD = ["Kp_psi", "Kd_psi", "Kp_theta", "Kd_theta", "Kvel"]
KOLOM_GAIN = KOLOM_GAIN_PD + config.NAMA_ALPHA + ["psi_max_DEG"]
# [REVISI 16-10] kolom tambahan (sama dengan simulasi/ise.py); file lama tanpa kolom ini tetap terbaca (nilai 0)
KOLOM_EKSTRA = ["yaw_rad", "dyaw_rad_s", "yaw_ref_rad", "theta_ref_rad", "dtheta_ref_rad_s", "u_belok_Nm",
                "dob_dtheta_rad_s2", "dob_dpsi_rad_s2", "slack"]
KOLOM_CSV = ["t_s", "theta_rad", "dtheta_rad_s", "psi_rad", "dpsi_rad_s",
             "error_theta", "error_dtheta", "error_psi", "error_dpsi",
             "u_PD_Nm", "u_Nm", "status_cbf", "dorong_N",
             "ISE_theta", "ISE_dtheta", "ISE_psi", "ISE_dpsi"] + KOLOM_GAIN + ["zeta_DEG"] + KOLOM_EKSTRA
KOLOM_ANGKA = [k for k in KOLOM_CSV if k != "status_cbf"]
POLA_NAMA = re.compile(r"^(\d{8})_(\d{6})_COBA (PD\+CBF|PD)_(PERCOBAAN|PENGUJIAN) KE-(\d+)\.csv$")
DEG = 180.0 / math.pi

_NAMA_DIPAKAI = []                  # nama yang sudah dibagikan (file-nya bisa belum selesai ditulis di latar)
_PROSES = []                        # proses penyimpanan latar yang masih berjalan


# =============================================================================
# Nama dan folder
# =============================================================================
def folder_jenis(jenis, mode=None):
    """PLOT_EVALUASI/ATERA_RASPI/<jenis>[/<PD atau CBFQP>]."""
    folder = os.path.join(FOLDER_PLOT, jenis)
    return folder if mode is None else os.path.join(folder, FOLDER_MODE[mode])


def daftar_percobaan(jenis=JENIS_PERCOBAAN):
    """Semua file satu jenis (kedua mode), urut dari yang paling lama."""
    hasil = []
    for mode in FOLDER_MODE:
        folder = folder_jenis(jenis, mode)
        for nama_file in (os.listdir(folder) if os.path.isdir(folder) else []):
            cocok = POLA_NAMA.match(nama_file)
            if cocok is None or cocok.group(4) != jenis:
                continue
            hasil.append({"tanggal": cocok.group(1), "jam": cocok.group(2), "mode": cocok.group(3),
                          "jenis": jenis, "nomor": int(cocok.group(5)), "nama": nama_file[:-4],
                          "path": os.path.join(folder, nama_file)})
    hasil.sort(key=lambda p: (p["tanggal"], p["nomor"], p["jam"], p["mode"]))
    return hasil


def nama_percobaan(mode, jenis):
    """Contoh: 20261014_093015_COBA PD+CBF_PENGUJIAN KE-03 (nomor per tanggal, mode, dan jenis)."""
    sekarang = datetime.now()
    tanggal = sekarang.strftime("%Y%m%d")
    terpakai = [p["nomor"] for p in daftar_percobaan(jenis) if p["tanggal"] == tanggal and p["mode"] == mode]
    for nama in _NAMA_DIPAKAI:
        cocok = POLA_NAMA.match(nama + ".csv")
        if cocok and cocok.group(1) == tanggal and cocok.group(3) == mode and cocok.group(4) == jenis:
            terpakai.append(int(cocok.group(5)))
    nama = "%s_%s_COBA %s_%s KE-%02d" % (tanggal, sekarang.strftime("%H%M%S"), mode, jenis, max(terpakai, default=0) + 1)
    _NAMA_DIPAKAI.append(nama)
    return nama


def simpan_berjalan():
    """Jumlah penyimpanan latar yang belum selesai."""
    _PROSES[:] = [proses for proses in _PROSES if proses.is_alive()]
    return len(_PROSES)


def tunggu_simpan(batas_waktu=30.0):
    """Tunggu semua penyimpanan latar selesai (dipanggil saat program keluar)."""
    for proses in _PROSES:
        proses.join(batas_waktu)
    simpan_berjalan()


# =============================================================================
# Perekam (dipakai atera_main.py)
# =============================================================================
class PerekamISE:
    def __init__(self):
        self.mulai("")
        self.file_terakhir = ""
        self.path_terakhir = ""
        self.alasan = ""

    def mulai(self, mode, jenis=JENIS_PERCOBAAN):
        """Perekaman baru. mode = MODE_PD / MODE_PD_CBF, jenis = JENIS_PERCOBAAN / JENIS_PENGUJIAN."""
        self.mode = mode
        self.jenis = jenis
        self.data = []
        self.t_terakhir = 0.0
        self.ise_theta = self.ise_dtheta = self.ise_psi = self.ise_dpsi = 0.0
        self.jumlah_c = 0

    def tambah(self, t, x, error_theta, error_dtheta, error_psi, error_dpsi, u_PD, u, status_cbf, gain, dorong=0.0,
               nilai_c=None, ekstra=None):
        """Satu baris per siklus kontrol. gain: dict Kp_psi .. Alpha_n, psi_max_DEG, zeta_DEG. nilai_c: c_h1.. / None.
        ekstra: [REVISI 16-10] dict kolom KOLOM_EKSTRA (yang tidak ada ditulis 0)."""
        dt = t - self.t_terakhir
        self.t_terakhir = t
        self.ise_theta += error_theta ** 2 * dt
        self.ise_dtheta += error_dtheta ** 2 * dt
        self.ise_psi += error_psi ** 2 * dt
        self.ise_dpsi += error_dpsi ** 2 * dt
        baris = [t, x[0], x[1], x[2], x[3], error_theta, error_dtheta, error_psi, error_dpsi, u_PD, u, status_cbf,
                 dorong, self.ise_theta, self.ise_dtheta, self.ise_psi, self.ise_dpsi]
        baris += [float(gain.get(nama, getattr(config, nama, 0.0))) for nama in KOLOM_GAIN]
        baris += [float(gain.get("zeta_DEG", 0.0))]
        ekstra = ekstra or {}
        baris += [float(ekstra.get(nama, 0.0)) for nama in KOLOM_EKSTRA]          # [REVISI 16-10]
        if nilai_c is not None:
            self.jumlah_c = max(self.jumlah_c, len(nilai_c))
            baris += ["" if c is None else c for c in nilai_c]
        self.data.append(baris)

    def simpan(self, latar=False):
        """Simpan CSV dan PNG. Mengembalikan nama percobaan, atau "" bila belum ada data.
        latar=True: ditulis di proses terpisah (fork), nama tetap ditentukan di sini."""
        if len(self.data) == 0:
            self.alasan = "belum ada data"
            return ""
        nama = nama_percobaan(self.mode, self.jenis)
        if latar:
            proses = multiprocessing.get_context("fork").Process(target=self._tulis_latar, args=(nama,))
            proses.start()
            _PROSES.append(proses)
        else:
            self._tulis(nama)
        self.file_terakhir = nama
        self.path_terakhir = os.path.join(folder_jenis(self.jenis, self.mode), nama)
        self.data = []
        return nama

    def _tulis_latar(self, nama):
        """Isi proses latar: keluaran layar dibuang (GUI tidak terganggu), kesalahan ditulis ke log."""
        kosong = os.open(os.devnull, os.O_WRONLY)
        os.dup2(kosong, 1)
        os.dup2(kosong, 2)
        try:
            self._tulis(nama)
        except Exception:
            LOGGER.exception("gagal menyimpan %s", nama)

    def _tulis(self, nama):
        folder = folder_jenis(self.jenis, self.mode)
        os.makedirs(folder, exist_ok=True)
        path = os.path.join(folder, nama)
        with open(path + ".csv", "w", newline="") as f:
            tulis = csv.writer(f)
            tulis.writerow(KOLOM_CSV + ["c_h%d" % (i + 1) for i in range(self.jumlah_c)])
            for baris in self.data:
                tulis.writerow([v if isinstance(v, str) else "%.8g" % v for v in baris])
        gambar_percobaan(baca_csv(path + ".csv"), nama, path + ".png")


# =============================================================================
# Membaca CSV dan menghitung metrik
# =============================================================================
def baca_csv(path):
    """Isi CSV sebagai dict kolom -> list (status_cbf berupa teks), ditambah 'gain', 'zeta_deg', 'psi_max_deg',
    'mask' (constraint CBF yang dipakai, dari kolom c_h)."""
    with open(path, newline="") as f:
        pembaca = csv.DictReader(f)
        nama_alpha = [k for k in (pembaca.fieldnames or []) if re.fullmatch(r"Alpha_\d+", k)]
        kolom_c = sorted((k for k in (pembaca.fieldnames or []) if re.fullmatch(r"c_h\d+", k)),
                         key=lambda k: int(k[3:]))
        c_dipakai = [False] * len(kolom_c)
        data = {k: [] for k in KOLOM_ANGKA + nama_alpha}
        data["status_cbf"] = []
        for baris in pembaca:
            for k in KOLOM_ANGKA + nama_alpha:
                teks = baris.get(k)
                data[k].append(float(teks) if teks not in (None, "") else 0.0)
            data["status_cbf"].append(baris.get("status_cbf") or "-")
            for i, k in enumerate(kolom_c):
                if baris.get(k) not in (None, ""):
                    c_dipakai[i] = True
    # constraint CBF yang dipakai (kolom c_h tidak kosong). None = mode PD / file lama: semua barrier merah
    data["mask"] = c_dipakai if any(c_dipakai) else None
    # File lama tanpa kolom psi_max_DEG: dipakai config.psi_max_DEG
    psi_max_deg = data["psi_max_DEG"][-1] if data["psi_max_DEG"] and data["psi_max_DEG"][-1] > 0 else config.psi_max_DEG
    data["gain"] = {k: (data[k][0] if data[k] else 0.0) for k in KOLOM_GAIN_PD + nama_alpha}
    data["gain"]["psi_max_DEG"] = psi_max_deg
    data["nama_alpha"] = nama_alpha
    data["zeta_deg"] = data["zeta_DEG"][0] if data["zeta_DEG"] else 0.0
    data["psi_max_deg"] = psi_max_deg
    data["s_m"] = [config.R * v for v in data["theta_rad"]]
    return data


def potong(data, t_akhir):
    """Data dari awal sampai t_akhir."""
    n = sum(1 for t in data["t_s"] if t <= t_akhir + 1e-9)
    return {k: (v[:n] if isinstance(v, list) else v) for k, v in data.items()}


def metrik(data):
    """Angka evaluasi satu percobaan (lihat docstring modul)."""
    t = data["t_s"]
    n = len(t)
    dt = [t[0]] + [t[i] - t[i - 1] for i in range(1, n)] if n else []
    durasi = t[-1] if n else 0.0
    psi, dpsi, dtheta = data["psi_rad"], data["dpsi_rad_s"], data["dtheta_rad_s"]
    s = [config.R * v for v in data["theta_rad"]]

    def luar(nilai, batas):
        return sum(d for v, d in zip(nilai, dt) if abs(v) > batas)

    def maks(nilai):
        return max((abs(v) for v in nilai), default=0.0)

    m = {"durasi": durasi,
         "ise_psi": data["ISE_psi"][-1] if n else 0.0, "ise_dpsi": data["ISE_dpsi"][-1] if n else 0.0,
         "ise_theta": data["ISE_theta"][-1] if n else 0.0, "ise_dtheta": data["ISE_dtheta"][-1] if n else 0.0,
         "psi_maks_deg": maks(psi) * DEG, "dpsi_maks": maks(dpsi), "dtheta_maks": maks(dtheta), "s_maks": maks(s),
         "u_maks": maks(data["u_Nm"]),
         "psi_max_deg": data["psi_max_deg"],
         "luar_psi": luar(psi, math.radians(data["psi_max_deg"])), "luar_dpsi": luar(dpsi, config.dpsi_max),
         "luar_dtheta": luar(dtheta, config.dtheta_max), "luar_s": luar(s, config.s_max)}
    m["rms_psi_deg"] = math.sqrt(m["ise_psi"] / durasi) * DEG if durasi > 0 else 0.0

    # waktu pulih: dari |psi| maksimum sampai |psi| < ambang terus-menerus EVAL_PULIH_LAMA_S detik
    m["pulih"] = None
    if n:
        k = max(range(n), key=lambda i: abs(psi[i]))
        ambang = math.radians(config.EVAL_PSI_PULIH_DEG)
        mulai = None
        for i in range(k, n):
            if abs(psi[i]) >= ambang:
                mulai = None
            elif mulai is None:
                mulai = i
            if mulai is not None and t[i] - t[mulai] >= config.EVAL_PULIH_LAMA_S:
                m["pulih"] = t[mulai] - t[k]
                break

    # CBF-QP (hanya baris mode PD+CBF, status "-" = mode PD)
    cbf = [i for i in range(n) if data["status_cbf"][i] != "-"]
    m["cbf_aman"] = m["cbf_gagal"] = m["cbf_koreksi"] = m["cbf_slack"] = None
    if cbf:
        m["cbf_aman"] = 100.0 * sum(data["status_cbf"][i] == "AMAN" for i in cbf) / len(cbf)
        m["cbf_gagal"] = 100.0 * sum(data["status_cbf"][i] == "GAGAL" for i in cbf) / len(cbf)
        m["cbf_slack"] = 100.0 * sum(data["status_cbf"][i] == "SLACK" for i in cbf) / len(cbf)   # [REVISI 16-10]
        m["cbf_koreksi"] = 100.0 * sum(koreksi_cbf(data, i) for i in cbf) / len(cbf)

    m["loop_hz"] = (n - 1) / (t[-1] - t[0]) if n > 1 and t[-1] > t[0] else 0.0
    m["dt_maks_ms"] = max(dt[1:], default=0.0) * 1000.0
    return m


def koreksi_cbf(data, i):
    """True bila u berbeda dari u_PD yang dibatasi arus (CBF-QP sedang mengoreksi)."""
    u_pd = max(config.u_min, min(config.u_max, data["u_PD_Nm"][i]))
    return abs(data["u_Nm"][i] - u_pd) > config.CBF_TOL_AKTIF


def _f(nilai, format_angka):
    return "-" if nilai is None else format_angka % nilai


# (judul baris, kunci metrik, format, dibandingkan dalam persen)
BARIS_METRIK = [
    ("Durasi [s]", "durasi", "%.1f", False),
    ("ISE psi [rad^2 s]", "ise_psi", "%.6f", True),
    ("ISE dpsi [(rad/s)^2 s]", "ise_dpsi", "%.6f", True),
    ("ISE theta [rad^2 s]", "ise_theta", "%.4f", True),
    ("ISE dtheta [(rad/s)^2 s]", "ise_dtheta", "%.4f", True),
    ("RMS psi [deg]", "rms_psi_deg", "%.3f", True),
    ("psi_max CBF [deg]", "psi_max_deg", "%g", False),
    ("|psi| maks [deg]", "psi_maks_deg", "%.2f", True),
    ("|dpsi| maks [rad/s]  (batas %.2f)" % config.dpsi_max, "dpsi_maks", "%.3f", True),
    ("|dtheta| maks [rad/s]  (batas %.1f)" % config.dtheta_max, "dtheta_maks", "%.2f", True),
    ("|s| maks [m]  (batas %.1f)" % config.s_max, "s_maks", "%.3f", True),
    ("|u| maks [N m]  (batas %.2f)" % config.u_max, "u_maks", "%.3f", True),
    ("Lama |psi| > psi_max [s]", "luar_psi", "%.2f", True),
    ("Lama |dpsi| > dpsi_max [s]", "luar_dpsi", "%.2f", True),
    ("Lama |dtheta| > dtheta_max [s]", "luar_dtheta", "%.2f", True),
    ("Lama |s| > s_max [s]", "luar_s", "%.2f", True),
    ("Waktu pulih [s]  (|psi| < %g deg)" % config.EVAL_PSI_PULIH_DEG, "pulih", "%.2f", True),
    ("CBF-QP AMAN [% waktu]", "cbf_aman", "%.1f", False),
    ("CBF-QP SLACK [% waktu]", "cbf_slack", "%.1f", False),          # [REVISI 16-10] constraint dilonggarkan
    ("CBF-QP GAGAL [% waktu]", "cbf_gagal", "%.1f", False),
    ("CBF-QP mengoreksi u [% waktu]", "cbf_koreksi", "%.1f", False),
    ("Loop rata-rata [Hz]", "loop_hz", "%.1f", False),
    ("dt maks [ms]", "dt_maks_ms", "%.1f", False),
]


def teks_gain(mode, gain):
    nama = KOLOM_GAIN_PD + ([k for k in gain if k.startswith("Alpha_")] + ["psi_max_DEG"] if mode == MODE_PD_CBF else [])
    return " | ".join("%s %g" % (k, gain[k]) for k in nama if k in gain)


def bungkus(teks, lebar=150):
    """Potong teks 'a | b | c' menjadi beberapa baris (judul gambar tidak terpotong)."""
    baris = [""]
    for bagian in teks.split(" | "):
        if baris[-1] and len(baris[-1]) + len(bagian) + 3 > lebar:
            baris.append("")
        baris[-1] = bagian if not baris[-1] else baris[-1] + " | " + bagian
    return "\n".join(baris)


# =============================================================================
# Grafik
# =============================================================================
WARNA_KOREKSI = "#c8e6c9"           # warna isi padat (EPS tidak mendukung transparansi)
WARNA_GAGAL = "#ffcdd2"


def simpan_gambar(fig, path_png):
    """Simpan .png dan .eps (nama sama)."""
    fig.savefig(path_png, dpi=120)
    fig.savefig(path_png[:-4] + ".eps", format="eps")


def _komet(ax, batas, label, sumbu="y"):
    """Garis hitam batas KOMET di +-batas. sumbu = "y" (garis mendatar) atau "x" (garis tegak)."""
    garis = ax.axhline if sumbu == "y" else ax.axvline
    for tanda in (1, -1):
        garis(tanda * batas, color=config.PLOT_KOMET_WARNA, linewidth=config.PLOT_KOMET_TEBAL, zorder=1.6,
              label=label if tanda == 1 else None)


def _penanda_psi(ax, sumbu="y"):
    """Garis penanda |psi| (config.PLOT_PENANDA_PSI: 5 deg kuning, 10 deg oranye). Hanya penanda, bukan batas CBF."""
    garis = ax.axhline if sumbu == "y" else ax.axvline
    for derajat, warna in config.PLOT_PENANDA_PSI:
        for tanda in (1, -1):
            garis(tanda * math.radians(derajat), color=warna, linewidth=1.2, linestyle=":", zorder=1.3,
                  label=("|psi| = %g deg" % derajat) if tanda == 1 else None)


def _aktif(mask, nama):
    """(batas atas aktif, batas bawah aktif) barrier `nama` ("psi", "dpsi", "s", "dtheta") dari mask constraint
    (config.CBF_BARRIER_H). mask None = semua dianggap aktif (mode PD / file lama)."""
    atas, bawah = config.CBF_BARRIER_H[nama]
    if mask is None:
        return True, True
    return (atas - 1 < len(mask) and bool(mask[atas - 1])), (bawah - 1 < len(mask) and bool(mask[bawah - 1]))


def _gaya(aktif, tebal):
    """Barrier aktif: merah (tebal / putus-putus tipis). Tidak aktif: abu-abu tipis."""
    if aktif:
        return {"color": config.PLOT_SAFETY_WARNA, "linewidth": config.PLOT_SAFETY_TEBAL if tebal else
                config.PLOT_SAFETY_TIPIS * 1.2, "linestyle": "-" if tebal else "--", "zorder": 1.5 if tebal else 1.4}
    return {"color": config.PLOT_NONAKTIF_WARNA, "linewidth": config.PLOT_SAFETY_TIPIS,
            "linestyle": "--" if tebal else ":", "zorder": 1.2}


def _batas(ax, nilai, aktif=(True, True), tebal=False, sumbu="y"):
    """Garis barrier di +nilai (batas atas) dan -nilai (batas bawah)."""
    garis = ax.axhline if sumbu == "y" else ax.axvline
    garis(nilai, **_gaya(aktif[0], tebal))
    garis(-nilai, **_gaya(aktif[1], tebal))


def _kotak(ax, x_batas, y_batas, x_sumbu, y_sumbu, aktif_x=(True, True), aktif_y=(True, True)):
    """Safety set dua variabel: tiap sisi kotak barrier (tebal) + perpanjangan (tipis). Batas sumbu tetap."""
    sisi = (([x_batas, x_batas], [-y_batas, y_batas], aktif_x[0], "x", x_batas),
            ([-x_batas, -x_batas], [-y_batas, y_batas], aktif_x[1], "x", -x_batas),
            ([-x_batas, x_batas], [y_batas, y_batas], aktif_y[0], "y", y_batas),
            ([-x_batas, x_batas], [-y_batas, -y_batas], aktif_y[1], "y", -y_batas))
    for xs, ys, aktif, arah, nilai in sisi:
        ax.plot(xs, ys, **_gaya(aktif, True))
        (ax.axvline if arah == "x" else ax.axhline)(nilai, **_gaya(aktif, False))
    ax.set_xlim(-x_sumbu, x_sumbu)
    ax.set_ylim(-y_sumbu, y_sumbu)


def _legenda(ax, loc="upper right"):
    """Legenda: contoh barrier aktif / tidak aktif + garis berlabel (kurva, penanda, KOMET)."""
    from matplotlib.lines import Line2D
    proksi = [Line2D([], [], label="barrier CBF aktif", **{k: v for k, v in _gaya(True, True).items() if k != "zorder"}),
              Line2D([], [], label="barrier CBF tidak aktif",
                     **{k: v for k, v in _gaya(False, True).items() if k != "zorder"})]
    garis, label = ax.get_legend_handles_labels()
    ax.legend(proksi + garis, [g.get_label() for g in proksi] + label, fontsize=7, loc=loc)


def _rapikan(ax, judul, xlabel, ylabel, loc="upper right", barrier=True):
    ax.set_title(judul, fontsize=10)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.grid(True, color="0.85")
    if barrier:
        _legenda(ax, loc)
    else:
        ax.legend(fontsize=7, loc=loc)


def _grafik_waktu(ax, kumpulan, kolom, judul, satuan, batas, sumbu, aktif=(True, True)):
    """kumpulan: list (label, warna, data). batas: nilai barrier (None = tidak ada), sumbu: batas sumbu Y tetap."""
    for label, warna, data in kumpulan:
        ax.plot(data["t_s"], data[kolom], color=warna, linewidth=1.0, label=label, zorder=2)
    if batas is not None:
        _batas(ax, batas, aktif)
    if sumbu is not None:
        ax.set_ylim(-sumbu, sumbu)
    _rapikan(ax, judul, "t [s]", satuan, barrier=batas is not None)


def _grafik_psi(ax, kumpulan, psi_max_deg, judul, mask=None):
    """psi(t) [rad], sumbu +-PLOT_BATAS psi: barrier psi_max, penanda 5 / 10 deg, KOMET (hitam)."""
    for label, warna, data in kumpulan:
        ax.plot(data["t_s"], data["psi_rad"], color=warna, linewidth=1.0, label=label, zorder=2)
    _batas(ax, math.radians(psi_max_deg), _aktif(mask, "psi"))
    _penanda_psi(ax, "y")
    _komet(ax, math.radians(config.SAFE_TILT_DEG), "KOMET |psi| = %g deg" % config.SAFE_TILT_DEG)
    ax.set_ylim(-config.PLOT_BATAS["psi"], config.PLOT_BATAS["psi"])
    _rapikan(ax, "%s   (psi_max = %g deg)" % (judul, psi_max_deg), "t [s]", "psi [rad]")


def _grafik_s(ax, kumpulan, judul, mask=None):
    """s(t) = R*theta [m], sumbu +-PLOT_BATAS s: barrier s_max dan KOMET S_KOMET (hitam)."""
    _komet(ax, config.S_KOMET, "KOMET |s| = %g m" % config.S_KOMET)
    _grafik_waktu(ax, kumpulan, "s_m", judul, "s [m]", config.s_max, config.PLOT_BATAS["s"], _aktif(mask, "s"))


def _fase_psi(ax, kumpulan, psi_max_deg, mask=None):
    _kotak(ax, math.radians(psi_max_deg), config.dpsi_max, config.PLOT_BATAS["psi"], config.PLOT_BATAS["dpsi"],
           _aktif(mask, "psi"), _aktif(mask, "dpsi"))
    _penanda_psi(ax, "x")
    _komet(ax, math.radians(config.SAFE_TILT_DEG), "KOMET |psi| = %g deg" % config.SAFE_TILT_DEG, "x")
    for label, warna, data in kumpulan:
        ax.plot(data["psi_rad"], data["dpsi_rad_s"], color=warna, linewidth=0.8, label=label, zorder=2)
    _rapikan(ax, "Bidang fase psi - dpsi  (psi_max = %g deg)" % psi_max_deg, "psi [rad]", "dpsi [rad/s]")


def _fase_theta(ax, kumpulan, mask=None):
    """Bidang fase theta - dtheta: hanya barrier dtheta (theta tidak punya barrier; batas jarak ada di grafik s)."""
    _batas(ax, config.dtheta_max, _aktif(mask, "dtheta"), tebal=True)
    for label, warna, data in kumpulan:
        ax.plot(data["theta_rad"], data["dtheta_rad_s"], color=warna, linewidth=0.8, label=label, zorder=2)
    ax.set_xlim(-config.PLOT_BATAS["theta"], config.PLOT_BATAS["theta"])
    ax.set_ylim(-config.PLOT_BATAS["dtheta"], config.PLOT_BATAS["dtheta"])
    _rapikan(ax, "Bidang fase theta - dtheta", "theta [rad]   (s = R*theta)", "dtheta [rad/s]")


def teks_mask(mask):
    """'h1 h2 ...' constraint aktif, '' bila mask None."""
    if mask is None:
        return ""
    aktif = ["h%d" % (k + 1) for k in range(len(mask)) if mask[k]]
    return " ".join(aktif) if aktif else "-"


def _tabel(ax, kolom, isi, lebar):
    ax.axis("off")
    tabel = ax.table(cellText=isi, colLabels=kolom, loc="upper center", cellLoc="center", colWidths=lebar)
    tabel.auto_set_font_size(False)
    tabel.set_fontsize(8.5)
    tabel.scale(1.0, 1.25)


def gambar_percobaan(data, nama, path_png):
    """PNG + EPS satu percobaan: grafik terhadap waktu, bidang fase dengan safety set, dan tabel metrik."""
    mode = MODE_PD_CBF if " PD+CBF_" in nama else MODE_PD
    m = metrik(data)
    warna = WARNA_MODE[mode]
    satu = [(mode, warna, data)]
    t = data["t_s"]

    fig = plt.figure(figsize=(14, 19))
    kisi = fig.add_gridspec(5, 2, height_ratios=[1, 1, 1, 1.15, 1.25])
    mask = data["mask"]
    teks_c = ("\nconstraint CBF-QP aktif: " + teks_mask(mask)) if mask is not None else ""
    fig.suptitle("%s  |  durasi %.1f s  |  zeta %.1f deg  |  loop %.0f Hz\n%s%s"
                 % (nama, m["durasi"], data["zeta_deg"], m["loop_hz"], bungkus(teks_gain(mode, data["gain"])), teks_c),
                 fontsize=10)

    _grafik_psi(fig.add_subplot(kisi[0, 0]), satu, data["psi_max_deg"], "Sudut badan psi(t)", mask)
    _grafik_waktu(fig.add_subplot(kisi[0, 1]), satu, "dpsi_rad_s", "Kecepatan sudut badan dpsi(t)", "dpsi [rad/s]",
                  config.dpsi_max, config.PLOT_BATAS["dpsi"], _aktif(mask, "dpsi"))
    _grafik_waktu(fig.add_subplot(kisi[1, 0]), satu, "dtheta_rad_s", "Kecepatan roda dtheta(t)", "dtheta [rad/s]",
                  config.dtheta_max, config.PLOT_BATAS["dtheta"], _aktif(mask, "dtheta"))
    _grafik_s(fig.add_subplot(kisi[1, 1]), satu, "Jarak tempuh s(t) = R*theta", mask)

    ax = fig.add_subplot(kisi[2, 0])
    if mode == MODE_PD_CBF:
        koreksi = [koreksi_cbf(data, i) for i in range(len(t))]
        gagal = [st == "GAGAL" for st in data["status_cbf"]]
        ax.fill_between(t, -config.u_max, config.u_max, where=koreksi, color=WARNA_KOREKSI, step="mid", zorder=0.5,
                        linewidth=0, label="CBF-QP mengoreksi")
        ax.fill_between(t, -config.u_max, config.u_max, where=gagal, color=WARNA_GAGAL, step="mid", zorder=0.6,
                        linewidth=0, label="CBF-QP GAGAL")
    ax.plot(t, data["u_PD_Nm"], color="gray", linewidth=0.8, linestyle="--", label="u_PD", zorder=2)
    ax.plot(t, data["u_Nm"], color=warna, linewidth=1.0, label="u (ke motor)", zorder=2.1)
    ax.axhline(config.u_max, color="black", linewidth=0.8, linestyle=":", label="batas arus")
    ax.axhline(-config.u_max, color="black", linewidth=0.8, linestyle=":")
    ax.set_ylim(-1.4 * config.u_max, 1.4 * config.u_max)        # u_PD di luar ini terpotong di grafik
    _rapikan(ax, "Torsi satu roda u_PD dan u (u_PD di luar batas arus terpotong di grafik)", "t [s]", "u [N m]",
             barrier=False)

    ax = fig.add_subplot(kisi[2, 1])
    ax.plot(t, data["ISE_psi"], color=warna, label="ISE psi [rad^2 s]")
    ax.set_title("ISE kumulatif", fontsize=10)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("ISE psi [rad^2 s]")
    ax.grid(True, color="0.85")
    ax2 = ax.twinx()
    ax2.plot(t, data["ISE_dpsi"], color="tab:brown", linewidth=0.9, label="ISE dpsi [(rad/s)^2 s]")
    ax2.set_ylabel("ISE dpsi [(rad/s)^2 s]")
    garis = ax.get_lines() + ax2.get_lines()
    ax.legend(garis, [g.get_label() for g in garis], fontsize=7, loc="upper left")

    _fase_psi(fig.add_subplot(kisi[3, 0]), satu, data["psi_max_deg"], mask)
    _fase_theta(fig.add_subplot(kisi[3, 1]), satu, mask)

    baris = [(judul, _f(m[k], fmt)) for judul, k, fmt, _ in BARIS_METRIK]
    setengah = (len(baris) + 1) // 2
    kiri, kanan = baris[:setengah], baris[setengah:] + [("", "")] * (2 * setengah - len(baris))
    _tabel(fig.add_subplot(kisi[4, :]), ["Metrik", "Nilai", "Metrik", "Nilai"],
           [list(a) + list(b) for a, b in zip(kiri, kanan)], [0.3, 0.17, 0.36, 0.17])

    fig.tight_layout(rect=(0, 0, 1, 0.97))
    simpan_gambar(fig, path_png)
    plt.close(fig)


# =============================================================================
# Plot realtime (atera_main.py --plot), dijalankan di proses terpisah
# =============================================================================
def _gambar_barrier(gambar1, gambar2, gambar3, mask):
    """Semua garis batas plot realtime (barrier, penanda |psi|, KOMET). Mengembalikan garis yang dibuat, supaya bisa
    dihapus dan digambar ulang bila constraint aktif berubah (ganti mode kontrol / mode gerak)."""
    sebelum = {ax: set(ax.lines) for ax in (gambar1, gambar2, gambar3)}
    _kotak(gambar1, config.psi_max, config.dpsi_max, config.PLOT_BATAS["psi"], config.PLOT_BATAS["dpsi"],
           _aktif(mask, "psi"), _aktif(mask, "dpsi"))
    _penanda_psi(gambar1, "x")
    _komet(gambar1, math.radians(config.SAFE_TILT_DEG), "KOMET |psi| = %g deg" % config.SAFE_TILT_DEG, "x")
    _batas(gambar2, config.dtheta_max, _aktif(mask, "dtheta"), tebal=True)
    gambar2.set_xlim(-config.PLOT_BATAS["theta"], config.PLOT_BATAS["theta"])
    gambar2.set_ylim(-config.PLOT_BATAS["dtheta"], config.PLOT_BATAS["dtheta"])
    _batas(gambar3, config.s_max, _aktif(mask, "s"), tebal=True, sumbu="x")
    _komet(gambar3, config.S_KOMET, "KOMET |s| = %g m" % config.S_KOMET, "x")
    gambar3.set_xlim(-config.PLOT_BATAS["s"], config.PLOT_BATAS["s"])
    return [g for ax in sebelum for g in ax.lines if g not in sebelum[ax]]


def proses_plot(antrean, jawaban, berhenti, daftar_backend):
    """Isi proses jendela plot realtime robot fisik. Kiriman dari atera_main.py (list per kiriman):
        (t, theta, dtheta, psi, dpsi, arus)   satu titik data (NaN = garis diputus)
        ("RESET",)                            data dikosongkan (MULAI)
        ("MODE", mode_kontrol, mode_gerak)    warna kurva (PD ungu, PD+CBF biru) dan judul
        ("MASK", mask atau None)              constraint CBF aktif: barrier merah, lainnya abu-abu
    Yang digambar hanya data config.PLOT_RENTANG detik terakhir. Titik hitam = keadaan sekarang."""
    import time
    import numpy as np
    try:
        berhasil = ""
        for backend in daftar_backend:
            try:
                plt.switch_backend(backend)
                berhasil = backend
                break
            except Exception:
                pass
        if not berhasil:
            jawaban.put(("gagal", "matplotlib tidak punya backend jendela (pip install PyQt5 / sudo apt install "
                                  "python3-tk)"))
            return
        fig = plt.figure(figsize=(10.0, 9.0))
        kisi = fig.add_gridspec(3, 2, height_ratios=[1.6, 0.5, 0.9])
        gambar1 = fig.add_subplot(kisi[0, 0])
        gambar2 = fig.add_subplot(kisi[0, 1])
        gambar3 = fig.add_subplot(kisi[1, :])
        ax_arus = fig.add_subplot(kisi[2, :])
        warna = {MODE_PD: config.PLOT_WARNA_PD, MODE_PD_CBF: config.PLOT_WARNA_PD_CBF}
        keadaan = {"mode": MODE_PD, "gerak": "", "mask": None}

        garis_psi, = gambar1.plot([], [], color=warna[MODE_PD], linewidth=1.4, zorder=2)
        titik_psi, = gambar1.plot([], [], "o", color="black", markersize=6, label="sekarang", zorder=3)
        garis_theta, = gambar2.plot([], [], color=warna[MODE_PD], linewidth=1.4, zorder=2)
        titik_theta, = gambar2.plot([], [], "o", color="black", markersize=6, label="sekarang", zorder=3)
        garis_s, = gambar3.plot([], [], color=warna[MODE_PD], linewidth=2.0, zorder=2)
        titik_s, = gambar3.plot([], [], "o", color="black", markersize=6, label="sekarang", zorder=3)
        garis_arus, = ax_arus.plot([], [], color="tab:green")
        ax_arus.axhline(config.MAX_CURRENT_A, color="black", linewidth=1.0, linestyle=":")
        ax_arus.axhline(-config.MAX_CURRENT_A, color="black", linewidth=1.0, linestyle=":")
        ax_arus.set_ylim(-1.2 * config.MAX_CURRENT_A, 1.2 * config.MAX_CURRENT_A)
        ax_arus.set(xlabel="waktu [s]", ylabel="arus per motor [A]")
        gambar1.set(xlabel="psi [rad]", ylabel="dpsi [rad/s]")
        gambar2.set(xlabel="theta [rad]   (s = R*theta)", ylabel="dtheta [rad/s]")
        gambar2.set_title("Gambar 2", fontsize=10)
        gambar3.axhline(0.0, color="gray", linewidth=0.6, zorder=1.0)
        gambar3.set_ylim(-1.0, 1.0)
        gambar3.set_yticks([0.0])
        gambar3.set_xlabel("s = R*theta [m]   (dari titik nol Y / B)")
        gambar3.set_title("Gambar 3", fontsize=10)
        for ax in (gambar1, gambar2, ax_arus):
            ax.grid(True, color="0.85")
        gambar3.grid(True, axis="x", color="0.85")
        batas = _gambar_barrier(gambar1, gambar2, gambar3, None)

        def tulisan():
            gambar1.set_title("Gambar 1   (psi_max = %g deg)%s" % (config.psi_max_DEG, (
                "   CBF aktif: " + teks_mask(keadaan["mask"])) if keadaan["mask"] is not None else ""), fontsize=10)
            fig.suptitle("ATERA robot fisik | kontrol %s | %s | data %.0f s terakhir"
                         % (keadaan["mode"], keadaan["gerak"] or "-", config.PLOT_RENTANG), fontsize=9, color="gray")
            for ax in (gambar1, gambar2, gambar3):
                _legenda(ax)
            try:
                fig.canvas.manager.set_window_title("ATERA - plot realtime robot (%s)" % keadaan["mode"])
            except Exception:
                pass

        tulisan()
        fig.tight_layout()
        fig.show()
        jawaban.put(("siap", berhasil))

        data = {k: [] for k in ("t", "theta", "dtheta", "psi", "dpsi", "arus")}
        while not berhenti.is_set() and plt.fignum_exists(fig.number):
            ada_baru = False
            while True:
                try:
                    kiriman = antrean.get_nowait()
                except Exception:
                    break
                ada_baru = True
                for isi in kiriman:
                    if isi[0] == "RESET":
                        data = {k: [] for k in data}
                    elif isi[0] == "MODE":
                        keadaan["mode"], keadaan["gerak"] = isi[1], isi[2]
                        for garis in (garis_psi, garis_theta, garis_s):
                            garis.set_color(warna.get(isi[1], config.PLOT_WARNA_PD))
                        tulisan()
                    elif isi[0] == "MASK":
                        keadaan["mask"] = isi[1]
                        for garis in batas:
                            garis.remove()
                        batas = _gambar_barrier(gambar1, gambar2, gambar3, isi[1])
                        tulisan()
                    else:
                        for k, v in zip(("t", "theta", "dtheta", "psi", "dpsi", "arus"), isi):
                            data[k].append(v)
            if ada_baru:
                t = data["t"]
                buang = 0
                while buang < len(t) and t[buang] < t[-1] - config.PLOT_RENTANG:
                    buang += 1
                if buang:
                    data = {k: v[buang:] for k, v in data.items()}
                theta = np.array(data["theta"], dtype=float)
                garis_psi.set_data(data["psi"], data["dpsi"])
                garis_theta.set_data(theta, data["dtheta"])
                garis_s.set_data(config.R * theta, np.zeros(len(theta)))
                garis_arus.set_data(data["t"], data["arus"])
                if data["t"] and math.isfinite(data["psi"][-1]):
                    titik_psi.set_data([data["psi"][-1]], [data["dpsi"][-1]])
                    titik_theta.set_data([data["theta"][-1]], [data["dtheta"][-1]])
                    titik_s.set_data([config.R * data["theta"][-1]], [0.0])
                akhir = data["t"][-1] if data["t"] else 0.0
                ax_arus.set_xlim(max(0.0, akhir - config.PLOT_RENTANG), max(config.PLOT_RENTANG, akhir))
                fig.canvas.draw_idle()
            fig.canvas.flush_events()
            time.sleep(1.0 / config.PLOT_HZ)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        try:
            jawaban.put(("gagal", "%s: %s" % (type(exc).__name__, exc)))
        except Exception:
            pass


# =============================================================================
# Perintah terminal
# =============================================================================
def perintah_daftar(jenis):
    semua = daftar_percobaan(jenis)
    if not semua:
        print("Belum ada file %s di %s" % (jenis, folder_jenis(jenis)))
        return
    print("Folder:", folder_jenis(jenis))
    print("%-9s %-7s %-7s %-3s %7s %11s %8s %8s %8s %8s  %s" % (
        "tanggal", "jam", "mode", "KE", "durasi", "ISE psi", "|psi|mx", "luar psi", "pulih", "CBF kor", "gain"))
    for p in semua:
        data = baca_csv(p["path"])
        m = metrik(data)
        print("%-9s %-7s %-7s %02d %6.1fs %11.6f %7.2fd %7.2fs %8s %8s  %s" % (
            p["tanggal"], p["jam"], p["mode"], p["nomor"], m["durasi"], m["ise_psi"], m["psi_maks_deg"],
            m["luar_psi"], _f(m["pulih"], "%.2fs"), _f(m["cbf_koreksi"], "%.1f%%"), teks_gain(p["mode"], data["gain"])))


def pilih_percobaan(mode, nomor, tanggal, jenis):
    """Percobaan paling baru yang cocok (nomor dan tanggal boleh None)."""
    cocok = [p for p in daftar_percobaan(jenis) if p["mode"] == mode and (nomor is None or p["nomor"] == nomor)
             and (tanggal is None or p["tanggal"] == tanggal)]
    return cocok[-1] if cocok else None


def perintah_bandingkan(nomor_pd, nomor_cbf, tanggal, jenis):
    coba = {MODE_PD: pilih_percobaan(MODE_PD, nomor_pd, tanggal, jenis),
            MODE_PD_CBF: pilih_percobaan(MODE_PD_CBF, nomor_cbf, tanggal, jenis)}
    hilang = [mode for mode in coba if coba[mode] is None]
    if hilang:
        print("%s %s tidak ditemukan. Lihat:  python3 ise.py --daftar%s"
              % (jenis.capitalize(), " dan ".join(hilang), " --pengujian" if jenis == JENIS_PENGUJIAN else ""))
        return ""

    # ISE terus bertambah terhadap waktu, jadi dibandingkan pada lama waktu yang sama (durasi yang lebih pendek).
    penuh = {mode: baca_csv(coba[mode]["path"]) for mode in coba}
    t_banding = min(d["t_s"][-1] for d in penuh.values() if d["t_s"])
    data = {mode: potong(penuh[mode], t_banding) for mode in penuh}
    m = {mode: metrik(data[mode]) for mode in data}
    label = {mode: "%s KE-%02d (%s)" % (mode, coba[mode]["nomor"], coba[mode]["tanggal"]) for mode in coba}
    kumpulan = [(label[mode], WARNA_MODE[mode], data[mode]) for mode in data]

    catatan = []
    if data[MODE_PD]["gain"] and any(abs(data[MODE_PD]["gain"][k] - data[MODE_PD_CBF]["gain"][k]) > 1e-12
                                     for k in KOLOM_GAIN_PD):
        catatan.append("gain PD kedua percobaan berbeda")
    if data[MODE_PD]["zeta_deg"] != data[MODE_PD_CBF]["zeta_deg"]:
        catatan.append("kemiringan lantai (zeta) kedua percobaan berbeda")
    if data[MODE_PD]["psi_max_deg"] != data[MODE_PD_CBF]["psi_max_deg"]:
        catatan.append("psi_max kedua percobaan berbeda (garis / kotak psi = psi_max PD+CBF)")
    if m[MODE_PD_CBF]["cbf_aman"] is not None and m[MODE_PD_CBF]["cbf_aman"] < 99.95:
        catatan.append("CBF-QP tidak selalu AMAN (SLACK: constraint dilonggarkan, u paling sedikit melanggar; "
                       "GAGAL / ALPHA_NOL: u = u_PD yang dibatasi arus)")

    def selisih(a, b):
        return "-" if a in (None, 0.0) or b is None else "%+.1f %%" % (100.0 * (b - a) / a)

    isi = [[judul, _f(m[MODE_PD][k], fmt), _f(m[MODE_PD_CBF][k], fmt),
            selisih(m[MODE_PD][k], m[MODE_PD_CBF][k]) if banding else ""] for judul, k, fmt, banding in BARIS_METRIK]

    nama = "%s_BANDING %s PD KE-%02d vs PD+CBF KE-%02d" % (coba[MODE_PD_CBF]["tanggal"], jenis, coba[MODE_PD]["nomor"],
                                                          coba[MODE_PD_CBF]["nomor"])
    folder = os.path.join(folder_jenis(jenis), "BANDING")
    os.makedirs(folder, exist_ok=True)
    path_png = os.path.join(folder, nama + ".png")

    fig = plt.figure(figsize=(14, 21))
    kisi = fig.add_gridspec(5, 2, height_ratios=[1, 1, 1, 1.1, 1.9])
    mask = data[MODE_PD_CBF]["mask"]
    teks_c = ("\nconstraint CBF-QP aktif (PD+CBF): " + teks_mask(mask)) if mask is not None else ""
    fig.suptitle("%s\nPD: %s\nPD+CBF: %s\ndibandingkan pada t = 0 .. %.1f s (durasi yang lebih pendek)%s"
                 % (nama, bungkus(teks_gain(MODE_PD, data[MODE_PD]["gain"])),
                    bungkus(teks_gain(MODE_PD_CBF, data[MODE_PD_CBF]["gain"])), t_banding, teks_c), fontsize=10)
    psi_max_deg = data[MODE_PD_CBF]["psi_max_deg"]
    _grafik_psi(fig.add_subplot(kisi[0, 0]), kumpulan, psi_max_deg, "psi(t)", mask)
    _grafik_waktu(fig.add_subplot(kisi[0, 1]), kumpulan, "dpsi_rad_s", "dpsi(t)", "dpsi [rad/s]", config.dpsi_max,
                  config.PLOT_BATAS["dpsi"], _aktif(mask, "dpsi"))
    _grafik_waktu(fig.add_subplot(kisi[1, 0]), kumpulan, "dtheta_rad_s", "dtheta(t)", "dtheta [rad/s]",
                  config.dtheta_max, config.PLOT_BATAS["dtheta"], _aktif(mask, "dtheta"))
    _grafik_s(fig.add_subplot(kisi[1, 1]), kumpulan, "s(t) = R*theta", mask)
    ax = fig.add_subplot(kisi[2, 0])
    _grafik_waktu(ax, kumpulan, "u_Nm", "Torsi ke motor u(t)", "u [N m]", None, None)
    ax.axhline(config.u_max, color="black", linewidth=0.8, linestyle=":")
    ax.axhline(-config.u_max, color="black", linewidth=0.8, linestyle=":")
    ax.set_ylim(-1.4 * config.u_max, 1.4 * config.u_max)
    _grafik_waktu(fig.add_subplot(kisi[2, 1]), kumpulan, "ISE_psi", "ISE psi kumulatif", "ISE psi [rad^2 s]", None,
                  None)
    _fase_psi(fig.add_subplot(kisi[3, 0]), kumpulan, psi_max_deg, mask)
    _fase_theta(fig.add_subplot(kisi[3, 1]), kumpulan, mask)
    ax = fig.add_subplot(kisi[4, :])
    _tabel(ax, ["0 - %.1f s" % t_banding, label[MODE_PD], label[MODE_PD_CBF], "PD+CBF terhadap PD"], isi,
           [0.34, 0.22, 0.22, 0.18])
    fig.tight_layout(rect=(0, 0.02, 1, 0.95))
    if catatan:                                     # di bawah gambar, tidak menimpa tabel
        fig.text(0.5, 0.005, "PERHATIAN: " + "; ".join(catatan), ha="center", va="bottom", fontsize=9,
                 color="tab:red")
    simpan_gambar(fig, path_png)
    plt.close(fig)

    print("PD     :", coba[MODE_PD]["nama"])
    print("PD+CBF :", coba[MODE_PD_CBF]["nama"])
    print("Dibandingkan pada t = 0 .. %.1f s\n" % t_banding)
    print("%-36s %-22s %-22s %s" % ("", "PD", "PD+CBF", "PD+CBF terhadap PD"))
    for baris in isi:
        print("%-36s %-22s %-22s %s" % tuple(baris))
    for teks in catatan:
        print("PERHATIAN:", teks)
    print("Tersimpan:", path_png)
    return nama


def perintah_gambar_ulang(jenis):
    for p in daftar_percobaan(jenis):
        gambar_percobaan(baca_csv(p["path"]), p["nama"], p["path"][:-4] + ".png")
        print("Dibuat ulang:", p["nama"] + ".png / .eps")


def main():
    parser = argparse.ArgumentParser(description="Evaluasi ISE robot fisik ATERA")
    parser.add_argument("--daftar", action="store_true", help="daftar dan metrik semua percobaan")
    parser.add_argument("--bandingkan", action="store_true", help="PD vs PD+CBF")
    parser.add_argument("--gambar-ulang", action="store_true", help="buat ulang PNG + EPS semua CSV")
    parser.add_argument("--pengujian", action="store_true", help="pakai hasil PENGUJIAN (L3)")
    parser.add_argument("--pd", type=int, default=None, help="nomor KE percobaan PD")
    parser.add_argument("--cbf", type=int, default=None, help="nomor KE percobaan PD+CBF")
    parser.add_argument("--tanggal", type=str, default=None, help="YYYYMMDD")
    pilihan = parser.parse_args()
    jenis = JENIS_PENGUJIAN if pilihan.pengujian else JENIS_PERCOBAAN
    if pilihan.daftar:
        perintah_daftar(jenis)
    elif pilihan.bandingkan:
        perintah_bandingkan(pilihan.pd, pilihan.cbf, pilihan.tanggal, jenis)
    elif pilihan.gambar_ulang:
        perintah_gambar_ulang(jenis)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()