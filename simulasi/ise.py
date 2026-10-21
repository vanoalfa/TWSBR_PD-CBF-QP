"""ise.py - Evaluasi ISE (Integral Square Error) robot ATERA.

ISE = jumlah (error^2 * dt), dihitung untuk theta, dtheta, psi, dan dpsi.

Hasil disimpan di dua subfolder PLOT_EVALUASI (HHmmSS = jam, menit, detik saat disimpan):
    PLOT_EVALUASI/PERCOBAAN/YYYYMMDD_HHmmSS_COBA PD_PERCOBAAN KE-XX.csv / .png       (perekaman biasa)
    PLOT_EVALUASI/PENGUJIAN/YYYYMMDD_HHmmSS_COBA PD_PENGUJIAN KE-XX.csv / .png       (mode pengujian, tombol T)
    PLOT_EVALUASI/PERBANDINGAN/YYYYMMDD_HHmmSS_COBA PD_PERBANDINGAN KE-XX.csv / .png (perekam perbandingan, tombol 3)
    (PD diganti PD+CBF untuk mode PD + CBF-QP)
Nomor KE-XX naik otomatis per tanggal, per mode, dan per jenis (PERCOBAAN / PENGUJIAN).
File lama (langsung di PLOT_EVALUASI, atau tanpa HHmmSS) tetap terbaca sebagai PERCOBAAN.
Bila gain diubah saat sedang direkam, nama file diberi akhiran _TUNING.

Kolom c_h1, c_h2, ... (hanya mode PD + CBF-QP): nilai tiap constraint CBF-QP (urutan DAFTAR_C di cbf_qp.py) pada
torsi u yang dipakai siklus itu. c >= 0 = terpenuhi, c ~ 0 = constraint sedang aktif (membatasi u), c < 0 =
dilanggar (status GAGAL). Kosong = constraint tidak dipakai di mode gerak itu, atau mode PD.

Setiap gambar disimpan sebagai .png dan .eps (vektor, untuk LaTeX). Batas sumbu tetap (config.PLOT_BATAS):
psi +-1 rad, dpsi +-10 rad/s, theta +-60 rad, dtheta +-30 rad/s, s +-3 m.
Barrier CBF-QP: psi dan dtheta (config.CBF_BARRIER_H: batas atas / bawah = h berapa); dpsi dan s hanya garis acuan.
Warna garis:
    merah          : barrier CBF-QP yang AKTIF (constraint dicentang / dipakai). Kotak psi memakai psi_max
                     yang tercatat di CSV (kolom psi_max_DEG, bisa diubah di jendela Kontrol)
    abu-abu tipis  : barrier yang constraint-nya TIDAK aktif (dari kolom c_h kosong di CSV)
    hitam          : batas KOMET (|psi| = SAFE_TILT_DEG, |s| = S_KOMET)
    kuning, oranye : penanda |psi| = 5 deg dan 10 deg (config.PLOT_PENANDA_PSI), hanya gambar

Isi gambar:
    Gambar 1: sumbu X = psi,   sumbu Y = dpsi       kotak barrier psi dan dpsi, garis KOMET psi
    Gambar 2: sumbu X = theta, sumbu Y = dtheta     garis barrier dtheta (theta tidak punya barrier)
    Gambar 3: sumbu X = s = R*theta [m], sumbu Y = 0   barrier s_min dan s_max, garis KOMET s
Garis barrier aktif (merah):
    - bagian yang membentuk kotak barrier : garis tebal menyambung
    - perpanjangan garis di luar kotak    : garis tipis putus-putus
    - Gambar 3 (jarak s) hanya punya 2 batas, jadi keduanya garis tebal menyambung

Perintah (dijalankan dari terminal), tambahkan --pengujian / --perbandingan untuk folder PENGUJIAN / PERBANDINGAN:
    python3 ise.py --daftar                                          daftar semua percobaan
    python3 ise.py --bandingkan                                      PD terbaru vs PD+CBF terbaru
    python3 ise.py --bandingkan --pd 3 --cbf 5                       PD KE-03 vs PD+CBF KE-05
    python3 ise.py --bandingkan --pd 3 --cbf 5 --tanggal 20261007    sama, untuk tanggal tertentu
    python3 ise.py --bandingkan --pengujian --pd 1 --cbf 1           PENGUJIAN PD KE-01 vs PD+CBF KE-01
Hasil --bandingkan disimpan di subfolder yang sama:
    PERCOBAAN/YYYYMMDD_BANDING PD KE-03 vs PD+CBF KE-05.png
    PENGUJIAN/YYYYMMDD_BANDING PENGUJIAN PD KE-01 vs PD+CBF KE-01.png
"""

import argparse
import csv
import math
import os
import re
from datetime import datetime

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
plt.rcParams["legend.framealpha"] = 1.0     # EPS tidak mendukung transparansi
plt.rcParams["axes.axisbelow"] = True       # grid di bawah garis batas

import config

FOLDER_PLOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), config.ISE_PLOT_DIR)

JENIS_PERCOBAAN = "PERCOBAAN"       # perekaman biasa (E / Q / KOMET)
JENIS_PENGUJIAN = "PENGUJIAN"       # mode pengujian (tombol T): dari dorongan O / P selama beberapa detik
JENIS_PERBANDINGAN = "PERBANDINGAN"  # perekam perbandingan (tombol 3): PD vs PD+CBF dari dorongan O / P


def folder_jenis(jenis):
    """Subfolder PLOT_EVALUASI untuk jenis PERCOBAAN atau PENGUJIAN."""
    return os.path.join(FOLDER_PLOT, jenis)

MODE_PD = "PD"
MODE_PD_CBF = "PD+CBF"
STATUS_ALPHA_NOL = "ALPHA_NOL"      # sama dengan cbf_qp.STATUS_ALPHA (tidak di-import: mode PD tidak butuh casadi)

KOLOM_GAIN_PD = ["Kp_psi", "Kd_psi", "Kp_theta", "Kd_theta", "Kvel"]
KOLOM_GAIN_CBF = config.NAMA_ALPHA + ["psi_max_DEG"]
KOLOM_GAIN = KOLOM_GAIN_PD + KOLOM_GAIN_CBF

# Kolom tambahan (sesudah zeta_DEG, sebelum c_h..): arah (perintah maju / mundur: 0 lepas, +1 / -1 W / S), bebas
# (1 = mode BEBAS, tombol F), balancing (1 = mode BALANCING, tombol B), yaw dan referensi PD heading, u_belok, taksiran
# DOB, dan slack CBF-QP (0 di mode PD). File lama tanpa kolom ini tetap terbaca.
KOLOM_EKSTRA = ["arah", "bebas", "balancing", "yaw_rad", "dyaw_rad_s", "yaw_ref_rad", "u_belok_Nm",
                "dob_dtheta_rad_s2", "dob_dpsi_rad_s2", "slack"]
KOLOM_CSV = ["t_s", "theta_rad", "dtheta_rad_s", "psi_rad", "dpsi_rad_s",
             "error_theta", "error_dtheta", "error_psi", "error_dpsi",
             "u_PD_Nm", "u_Nm", "status_cbf", "dorong_N",
             "ISE_theta", "ISE_dtheta", "ISE_psi", "ISE_dpsi"] + KOLOM_GAIN + ["zeta_DEG"] + KOLOM_EKSTRA
STATUS_SLACK = "SLACK"              # sama dengan cbf_qp.STATUS_SLACK

# Contoh nama: 20261008_201530_COBA PD+CBF_PENGUJIAN KE-05_TUNING.csv  (bagian _HHmmSS boleh tidak ada: file lama)
POLA_NAMA = re.compile(r"^(\d{8})(?:_(\d{6}))?_COBA (PD\+CBF|PD)_(PERCOBAAN|PENGUJIAN|PERBANDINGAN) KE-(\d+)(_TUNING)?\.csv$")

BANTUAN = """
Perintah ise.py (tambahkan --pengujian untuk hasil mode pengujian di folder PLOT_EVALUASI/PENGUJIAN):
  python3 ise.py --daftar
        daftar semua percobaan di folder PLOT_EVALUASI/PERCOBAAN
  python3 ise.py --bandingkan
        bandingkan percobaan PD terbaru dengan percobaan PD+CBF terbaru (yang _TUNING dilewati)
  python3 ise.py --bandingkan --pd 3 --cbf 5
        bandingkan PD KE-03 dengan PD+CBF KE-05 (tanggal terbaru yang punya nomor itu)
  python3 ise.py --bandingkan --pd 3 --cbf 5 --tanggal 20261007
        sama, tetapi hanya percobaan tanggal 20261007
  python3 ise.py --daftar --pengujian
  python3 ise.py --bandingkan --pengujian --pd 1 --cbf 1
        sama, untuk hasil mode pengujian
  python3 ise.py --daftar --perbandingan
  python3 ise.py --bandingkan --perbandingan --pd 1 --cbf 1
        sama, untuk hasil perekam perbandingan (tombol 3); gambar BANDING-nya juga dibuat otomatis oleh atera_main.py
Hasil: PLOT_EVALUASI/PERCOBAAN/YYYYMMDD_BANDING PD KE-03 vs PD+CBF KE-05.png
       PLOT_EVALUASI/PENGUJIAN/YYYYMMDD_BANDING PENGUJIAN PD KE-01 vs PD+CBF KE-01.png
"""


# =============================================================================
# Nama file
# =============================================================================
def daftar_percobaan(jenis=JENIS_PERCOBAAN):
    """Semua file jenis PERCOBAAN atau PENGUJIAN, urut dari yang paling lama ke yang paling baru.
    Untuk PERCOBAAN, file lama yang masih langsung di folder PLOT_EVALUASI ikut dibaca."""
    folder_dicari = [folder_jenis(jenis)]
    if jenis == JENIS_PERCOBAAN:
        folder_dicari.append(FOLDER_PLOT)
    hasil = []
    for folder in folder_dicari:
        if not os.path.isdir(folder):
            continue
        for nama_file in os.listdir(folder):
            cocok = POLA_NAMA.match(nama_file)
            if cocok is None or cocok.group(4) != jenis:
                continue
            percobaan = {
                "tanggal": cocok.group(1),
                "jam": cocok.group(2) if cocok.group(2) is not None else "",
                "mode": cocok.group(3),
                "jenis": cocok.group(4),
                "nomor": int(cocok.group(5)),
                "tuning": cocok.group(6) is not None,
                "nama": nama_file[:-4],
                "path": os.path.join(folder, nama_file),
            }
            hasil.append(percobaan)
    hasil.sort(key=lambda percobaan: (percobaan["tanggal"], percobaan["nomor"], percobaan["jam"], percobaan["mode"]))
    return hasil


def nama_percobaan(mode, tuning, jenis=JENIS_PERCOBAAN):
    """Contoh: 20261008_201530_COBA PD+CBF_PENGUJIAN KE-03 (nomor naik otomatis per tanggal, mode, dan jenis)."""
    os.makedirs(folder_jenis(jenis), exist_ok=True)
    sekarang = datetime.now()
    tanggal = sekarang.strftime("%Y%m%d")
    jam = sekarang.strftime("%H%M%S")
    nomor = 1
    for percobaan in daftar_percobaan(jenis):
        if percobaan["tanggal"] == tanggal and percobaan["mode"] == mode and percobaan["nomor"] >= nomor:
            nomor = percobaan["nomor"] + 1
    nama = "%s_%s_COBA %s_%s KE-%02d" % (tanggal, jam, mode, jenis, nomor)
    if tuning:
        nama = nama + "_TUNING"
    return nama


def aktif_barrier(mask, nama):
    """(batas atas aktif, batas bawah aktif) barrier `nama` ("psi", "dpsi", "s", "dtheta") menurut mask constraint
    CBF-QP (list 0 / 1 urutan DAFTAR_C). mask None = semua dianggap aktif (contoh: mode PD, file lama).
    Nama yang bukan constraint CBF-QP (tidak ada di config.CBF_BARRIER_H, contoh dpsi, s) selalu tidak aktif."""
    if nama not in config.CBF_BARRIER_H:
        return False, False
    atas, bawah = config.CBF_BARRIER_H[nama]
    if mask is None:
        return True, True

    def ada(nomor):
        return nomor - 1 < len(mask) and bool(mask[nomor - 1])
    return ada(atas), ada(bawah)


def _gaya_batas(garis, aktif, tebal):
    """Aktif: merah (tebal untuk sisi kotak / garis batas, putus-putus tipis untuk perpanjangan).
    Tidak aktif: abu-abu tipis."""
    if aktif:
        garis.set_color(config.PLOT_SAFETY_WARNA)
        garis.set_linewidth(config.PLOT_SAFETY_TEBAL if tebal else config.PLOT_SAFETY_TIPIS)
        garis.set_linestyle("-" if tebal else "--")
        garis.set_zorder(1.5 if tebal else 1.4)
    else:
        garis.set_color(config.PLOT_NONAKTIF_WARNA)
        garis.set_linewidth(config.PLOT_SAFETY_TIPIS)
        garis.set_linestyle("--" if tebal else ":")
        garis.set_zorder(1.2)


def gambar_kotak_safety(ax, x_batas, y_batas, aktif_x=(True, True), aktif_y=(True, True)):
    """Batas safety set dua variabel. x_batas / y_batas = (min, max), aktif_x / aktif_y = (batas atas, batas bawah).
    Tiap sisi: garis kotak (tebal) + perpanjangan di luar kotak (tipis putus-putus). Mengembalikan dict garis
    (dipakai atur_kotak_safety untuk menggeser batas / mengganti aktif tanpa menggambar ulang)."""
    artis = {}
    for sisi in ("x+", "x-", "y+", "y-"):
        kotak, = ax.plot([], [])
        panjang = ax.axvline(0.0) if sisi[0] == "x" else ax.axhline(0.0)
        artis[sisi] = (kotak, panjang)
    atur_kotak_safety(artis, x_batas, y_batas, aktif_x, aktif_y)
    return artis


def atur_kotak_safety(artis, x_batas, y_batas, aktif_x, aktif_y):
    x_min, x_max = x_batas
    y_min, y_max = y_batas
    isi = {"x+": ([x_max, x_max], [y_min, y_max], x_max, aktif_x[0]),
           "x-": ([x_min, x_min], [y_min, y_max], x_min, aktif_x[1]),
           "y+": ([x_min, x_max], [y_max, y_max], y_max, aktif_y[0]),
           "y-": ([x_min, x_max], [y_min, y_min], y_min, aktif_y[1])}
    for sisi, (xs, ys, nilai, aktif) in isi.items():
        kotak, panjang = artis[sisi]
        kotak.set_data(xs, ys)
        if sisi[0] == "x":
            panjang.set_xdata([nilai, nilai])
        else:
            panjang.set_ydata([nilai, nilai])
        _gaya_batas(kotak, aktif, True)
        _gaya_batas(panjang, aktif, False)


def gambar_garis_barrier(ax, batas, aktif=(True, True), sumbu="y"):
    """Batas satu variabel (garis penuh, tebal): batas = (min, max), aktif = (batas atas, batas bawah).
    sumbu = "y" (garis mendatar) atau "x" (garis tegak). Mengembalikan dict garis (lihat atur_garis_barrier)."""
    garis = ax.axhline if sumbu == "y" else ax.axvline
    artis = {"+": garis(0.0), "-": garis(0.0), "sumbu": sumbu}
    atur_garis_barrier(artis, batas, aktif)
    return artis


def atur_garis_barrier(artis, batas, aktif):
    for kunci, nilai, a in (("+", batas[1], aktif[0]), ("-", batas[0], aktif[1])):
        if artis["sumbu"] == "y":
            artis[kunci].set_ydata([nilai, nilai])
        else:
            artis[kunci].set_xdata([nilai, nilai])
        _gaya_batas(artis[kunci], a, True)


def legenda(ax, loc="upper right", perpanjangan=True, fontsize=7):
    """Legenda: contoh garis barrier aktif / tidak aktif (proksi, karena warnanya bisa berubah) + garis berlabel."""
    from matplotlib.lines import Line2D
    proksi = [Line2D([], [], color=config.PLOT_SAFETY_WARNA, linewidth=config.PLOT_SAFETY_TEBAL,
                     label="barrier CBF aktif"),
              Line2D([], [], color=config.PLOT_NONAKTIF_WARNA, linewidth=config.PLOT_SAFETY_TIPIS, linestyle="--",
                     label="barrier CBF tidak aktif")]
    if perpanjangan:
        proksi.append(Line2D([], [], color=config.PLOT_SAFETY_WARNA, linewidth=config.PLOT_SAFETY_TIPIS,
                             linestyle="--", label="perpanjangan batas"))
    garis, label = ax.get_legend_handles_labels()
    ax.legend(proksi + garis, [g.get_label() for g in proksi] + label, fontsize=fontsize, loc=loc)


def teks_mask(mask):
    """'h1 h2 h7 h8' (constraint aktif), '-' bila tidak ada, '' bila mask None."""
    if mask is None:
        return ""
    aktif = ["h%d" % (k + 1) for k in range(len(mask)) if mask[k]]
    return " ".join(aktif) if aktif else "-"


def gambar_komet(ax, batas, label, sumbu="x"):
    """Garis hitam batas KOMET di +-batas. sumbu = "x" (garis tegak) atau "y" (garis mendatar)."""
    garis = ax.axvline if sumbu == "x" else ax.axhline
    for tanda in (1.0, -1.0):
        garis(tanda * batas, color=config.PLOT_KOMET_WARNA, linewidth=config.PLOT_KOMET_TEBAL, zorder=1.6,
              label=label if tanda > 0 else None)


def gambar_penanda_psi(ax, sumbu="x"):
    """Garis penanda |psi| (config.PLOT_PENANDA_PSI: 5 deg kuning, 10 deg oranye). sumbu = "x" atau "y" (psi di sumbu itu)."""
    garis = ax.axvline if sumbu == "x" else ax.axhline
    for derajat, warna in config.PLOT_PENANDA_PSI:
        for tanda in (1.0, -1.0):
            garis(tanda * math.radians(derajat), color=warna, linewidth=1.2, linestyle=":", zorder=1.3,
                  label=("|psi| = %g deg" % derajat) if tanda > 0 else None)


def gambar_safety_set(gambar1, gambar2, psi_max=None, mask=None):
    """Gambar 1: kotak barrier psi x dpsi + penanda |psi| + KOMET psi (hitam).
    Gambar 2: garis barrier dtheta saja (sumbu theta tanpa batas). Batas sumbu tetap.
    psi_max [rad]: None = config.psi_max. mask: constraint CBF aktif (None = semua aktif).
    Mengembalikan dict garis barrier (dipakai atur_safety_set bila psi_max / mask berubah)."""
    if psi_max is None:
        psi_max = config.psi_max
    artis = {"psi": gambar_kotak_safety(gambar1, (-psi_max, psi_max), (config.dpsi_min, config.dpsi_max),
                                        aktif_barrier(mask, "psi"), aktif_barrier(mask, "dpsi")),
             "dtheta": gambar_garis_barrier(gambar2, (config.dtheta_min, config.dtheta_max),
                                            aktif_barrier(mask, "dtheta"), "y")}
    gambar_penanda_psi(gambar1)
    gambar_komet(gambar1, math.radians(config.SAFE_TILT_DEG), "KOMET |psi| = %g deg" % config.SAFE_TILT_DEG)
    batas = config.PLOT_BATAS
    gambar1.set_xlim(-batas["psi"], batas["psi"])
    gambar1.set_ylim(-batas["dpsi"], batas["dpsi"])
    gambar2.set_xlim(-batas["theta"], batas["theta"])
    gambar2.set_ylim(-batas["dtheta"], batas["dtheta"])
    return artis


def atur_safety_set(artis, psi_max, mask):
    atur_kotak_safety(artis["psi"], (-psi_max, psi_max), (config.dpsi_min, config.dpsi_max),
                      aktif_barrier(mask, "psi"), aktif_barrier(mask, "dpsi"))
    atur_garis_barrier(artis["dtheta"], (config.dtheta_min, config.dtheta_max), aktif_barrier(mask, "dtheta"))


def gambar_batas_jarak(gambar3, mask=None):
    """Gambar 3 (jarak s): barrier s_min dan s_max dan KOMET +-S_KOMET (hitam). Mengembalikan dict garis barrier."""
    artis = gambar_garis_barrier(gambar3, (config.s_min, config.s_max), aktif_barrier(mask, "s"), "x")
    gambar_komet(gambar3, config.S_KOMET, "KOMET |s| = %g m" % config.S_KOMET)
    gambar3.axhline(0.0, color="gray", linewidth=0.6, zorder=1.0)
    gambar3.set_xlim(-config.PLOT_BATAS["s"], config.PLOT_BATAS["s"])
    gambar3.set_ylim(-1.0, 1.0)
    gambar3.set_yticks([0.0])
    gambar3.set_xlabel("s = R*theta [m]   (jarak tempuh dari posisi awal)")
    gambar3.set_title("Gambar 3", fontsize=10)
    gambar3.grid(True, axis="x", color="0.85")
    return artis


def atur_batas_jarak(artis, mask):
    atur_garis_barrier(artis, (config.s_min, config.s_max), aktif_barrier(mask, "s"))


def simpan_gambar(fig, path_png):
    """Simpan .png dan .eps (nama sama)."""
    fig.savefig(path_png, dpi=130)
    fig.savefig(path_png[:-4] + ".eps", format="eps")


def warna_mode(mode):
    """Warna kurva: PD ungu, PD+CBF biru (config.PLOT_WARNA_PD / PLOT_WARNA_PD_CBF)."""
    return config.PLOT_WARNA_PD_CBF if mode == MODE_PD_CBF else config.PLOT_WARNA_PD


def angka(nilai):
    """Angka pendek untuk tulisan gain: 15.0 -> '15', 0.0400 -> '0.04'."""
    return "%g" % nilai


def bungkus(teks, lebar=150):
    """Potong teks 'a | b | c' menjadi beberapa baris (judul gambar tidak terpotong)."""
    baris = [""]
    for bagian in teks.split(" | "):
        if baris[-1] != "" and len(baris[-1]) + len(bagian) + 3 > lebar:
            baris.append("")
        baris[-1] = bagian if baris[-1] == "" else baris[-1] + " | " + bagian
    return "\n".join(baris)


def teks_gain(mode, gain_awal, gain_akhir):
    """Contoh: 'Kp_psi 15 | Kd_psi 1.5 -> 2 | ...'. Alpha hanya ditulis untuk mode PD+CBF."""
    daftar = list(KOLOM_GAIN_PD)
    if mode == MODE_PD_CBF:
        # Alpha yang ada di data (jumlahnya bisa berbeda antar file)
        nomor = 1
        while ("Alpha_%d" % nomor) in gain_akhir:
            daftar.append("Alpha_%d" % nomor)
            nomor = nomor + 1
        if "psi_max_DEG" in gain_akhir:
            daftar.append("psi_max_DEG")
    bagian = []
    for nama in daftar:
        if abs(gain_awal[nama] - gain_akhir[nama]) > 1e-12:
            bagian.append("%s %s -> %s" % (nama, angka(gain_awal[nama]), angka(gain_akhir[nama])))
        else:
            bagian.append("%s %s" % (nama, angka(gain_akhir[nama])))
    return " | ".join(bagian)


# =============================================================================
# Perekam (dipakai atera_main.py)
# =============================================================================
class PerekamISE:
    def __init__(self):
        self.mode = ""
        self.jenis = JENIS_PERCOBAAN
        self.data = []
        self.t_terakhir = 0.0
        self.ise_theta = 0.0
        self.ise_dtheta = 0.0
        self.ise_psi = 0.0
        self.ise_dpsi = 0.0
        self.gain_awal = None
        self.gain_akhir = None
        self.gain_berubah = []         # nama gain yang diubah saat percobaan direkam
        self.jumlah_c = 0              # jumlah kolom constraint CBF-QP (c_h1, c_h2, ...) di CSV
        self.c_dipakai = []            # True untuk constraint yang pernah aktif selama perekaman (garis merah di gambar)
        self.zeta_deg = 0.0            # kemiringan lantai saat percobaan [deg]
        self.file_terakhir = ""
        self.path_terakhir = ""
        self.alasan = ""               # alasan bila simpan() tidak menyimpan

    def mulai(self, mode, jenis=JENIS_PERCOBAAN):
        """Mulai perekaman baru. mode = "PD" atau "PD+CBF", jenis = "PERCOBAAN" atau "PENGUJIAN"."""
        self.mode = mode
        self.jenis = jenis
        self.data = []
        self.t_terakhir = 0.0
        self.ise_theta = 0.0
        self.ise_dtheta = 0.0
        self.ise_psi = 0.0
        self.ise_dpsi = 0.0
        self.gain_awal = None
        self.gain_akhir = None
        self.gain_berubah = []
        self.jumlah_c = 0
        self.c_dipakai = []

    def tambah(self, t, x, error_theta, error_dtheta, error_psi, error_dpsi, u_PD, u, status_cbf, gain, dorong=0.0,
               nilai_c=None, ekstra=None):
        """Dipanggil tiap siklus kontrol.

        gain    : dict berisi Kp_psi, Kd_psi, Kp_theta, Kd_theta, Kvel, Alpha_1, Alpha_2, ... dan zeta_DEG
        dorong  : gaya dorongan yang sedang bekerja [N] (+ ke depan, - ke belakang, 0 tidak ada)
        nilai_c : list nilai constraint CBF-QP (None = constraint tidak dipakai), atau None (mode PD / motor mati)
        ekstra  : dict kolom KOLOM_EKSTRA (yang tidak ada ditulis 0)
        """
        dt = t - self.t_terakhir
        self.t_terakhir = t

        self.ise_theta = self.ise_theta + (error_theta ** 2) * dt
        self.ise_dtheta = self.ise_dtheta + (error_dtheta ** 2) * dt
        self.ise_psi = self.ise_psi + (error_psi ** 2) * dt
        self.ise_dpsi = self.ise_dpsi + (error_dpsi ** 2) * dt

        # Gain dicatat tiap baris supaya terlihat kapan gain diubah.
        gain_sekarang = {}
        for nama in KOLOM_GAIN:
            gain_sekarang[nama] = float(gain.get(nama, 0.0))
        if self.gain_awal is None:
            self.gain_awal = gain_sekarang
        self.gain_akhir = gain_sekarang

        # Alpha tidak dipakai di mode PD, jadi perubahan Alpha di mode PD bukan tuning.
        for nama in KOLOM_GAIN:
            if self.mode == MODE_PD and nama in KOLOM_GAIN_CBF:
                continue
            if abs(gain_sekarang[nama] - self.gain_awal[nama]) > 1e-12 and nama not in self.gain_berubah:
                self.gain_berubah.append(nama)

        baris = [t, x[0], x[1], x[2], x[3],
                 error_theta, error_dtheta, error_psi, error_dpsi,
                 u_PD, u, status_cbf, dorong,
                 self.ise_theta, self.ise_dtheta, self.ise_psi, self.ise_dpsi]
        for nama in KOLOM_GAIN:
            baris.append(gain_sekarang[nama])
        self.zeta_deg = float(gain.get("zeta_DEG", getattr(config, "zeta_DEG", 0.0)))
        baris.append(self.zeta_deg)
        ekstra = ekstra or {}
        for nama in KOLOM_EKSTRA:
            baris.append(float(ekstra.get(nama, 0.0)))
        # Kolom constraint CBF-QP di ujung baris (kolom lain tetap di posisinya).
        if nilai_c is not None:
            self.jumlah_c = max(self.jumlah_c, len(nilai_c))
            baris.extend("" if c is None else c for c in nilai_c)
            while len(self.c_dipakai) < len(nilai_c):
                self.c_dipakai.append(False)
            for k in range(len(nilai_c)):
                if nilai_c[k] is not None:
                    self.c_dipakai[k] = True
        self.data.append(baris)

    def simpan(self):
        """Simpan CSV dan PNG (selalu, bila ada data). Mengembalikan nama percobaan, atau "" bila belum ada data."""
        if len(self.data) == 0:
            self.alasan = "belum ada data (sudah disimpan, atau belum ada siklus yang direkam)"
            return ""
        self.alasan = ""

        tuning = len(self.gain_berubah) > 0
        nama = nama_percobaan(self.mode, tuning, self.jenis)
        folder = folder_jenis(self.jenis)
        path_csv = os.path.join(folder, nama + ".csv")
        path_png = os.path.join(folder, nama + ".png")

        # ----------------------------------------------------------------- CSV
        with open(path_csv, "w", newline="") as f:
            tulis = csv.writer(f)
            kolom_c = ["c_h%d" % (i + 1) for i in range(self.jumlah_c)]
            tulis.writerow(KOLOM_CSV + kolom_c)
            for baris in self.data:
                baris_teks = []
                for nilai in baris:
                    if isinstance(nilai, str):
                        baris_teks.append(nilai)
                    else:
                        baris_teks.append("%.8g" % nilai)
                # Baris tanpa nilai constraint (motor mati / mode PD) diberi kolom kosong.
                baris_teks.extend([""] * (len(KOLOM_CSV) + len(kolom_c) - len(baris_teks)))
                tulis.writerow(baris_teks)

        # ----------------------------------------------------------------- PNG
        theta = [baris[1] for baris in self.data]
        dtheta = [baris[2] for baris in self.data]
        psi = [baris[3] for baris in self.data]
        dpsi = [baris[4] for baris in self.data]
        durasi = self.data[-1][0]

        judul = "%s  |  mode %s  |  durasi %.1f s  |  zeta %.1f deg" % (nama, self.mode, durasi, self.zeta_deg)
        judul = judul + "\n" + bungkus(teks_gain(self.mode, self.gain_awal, self.gain_akhir))
        if self.mode == MODE_PD_CBF:
            status = [baris[11] for baris in self.data]
            cbf_ok = persen_cbf_ok(status)
            judul = judul + "  |  CBF-QP berstatus OK %.1f %% waktu" % cbf_ok
            slack = persen_status(status, STATUS_SLACK)
            if slack > 0.0:
                judul = judul + " (SLACK %.1f %%)" % slack
            if len(self.c_dipakai) > 0:
                judul = judul + "  |  constraint aktif: " + teks_mask(self.c_dipakai)
            alpha_nol = persen_status(status, STATUS_ALPHA_NOL)
            if alpha_nol > 0.0:
                peringatan = ("CBF-QP TIDAK AKTIF %.1f %% waktu (ALPHA_NOL: ada Alpha = 0 di config.py), "
                              "selama itu u = u_PD seperti mode PD" % alpha_nol)
                judul = judul + "\n" + peringatan
                print("PERHATIAN: " + peringatan)
        if tuning:
            judul = judul + "\nTUNING: gain diubah saat direkam (%s), ISE tidak untuk dibandingkan" \
                % ", ".join(self.gain_berubah)

        jarak = [config.R * nilai for nilai in theta]

        fig = plt.figure(figsize=(13, 8.4))
        kisi = fig.add_gridspec(2, 2, height_ratios=[1.0, 0.32])
        gambar1 = fig.add_subplot(kisi[0, 0])
        gambar2 = fig.add_subplot(kisi[0, 1])
        gambar3 = fig.add_subplot(kisi[1, :])
        fig.suptitle(judul, fontsize=10)
        psi_max = math.radians(self.gain_akhir.get("psi_max_DEG", config.psi_max_DEG))
        mask = self.c_dipakai if (self.mode == MODE_PD_CBF and len(self.c_dipakai) > 0) else None
        gambar_safety_set(gambar1, gambar2, psi_max, mask)
        gambar_batas_jarak(gambar3, mask)

        # Gambar 1: X = psi, Y = dpsi
        warna = warna_mode(self.mode)
        gambar1.plot(psi, dpsi, color=warna, linewidth=1.0)
        gambar1.plot(psi[0], dpsi[0], "o", color="tab:green", label="awal")
        gambar1.plot(psi[-1], dpsi[-1], "x", color="black", markersize=9, label="akhir")
        gambar1.set_xlabel("psi [rad]")
        gambar1.set_ylabel("dpsi [rad/s]")
        gambar1.set_title("Gambar 1\nISE psi = %.6f rad^2 s   |   ISE dpsi = %.6f (rad/s)^2 s"
                          % (self.ise_psi, self.ise_dpsi), fontsize=10)
        gambar1.grid(True, color="0.85")
        legenda(gambar1)

        # Gambar 2: X = theta, Y = dtheta
        gambar2.plot(theta, dtheta, color=warna, linewidth=1.0)
        gambar2.plot(theta[0], dtheta[0], "o", color="tab:green", label="awal")
        gambar2.plot(theta[-1], dtheta[-1], "x", color="black", markersize=9, label="akhir")
        gambar2.set_xlabel("theta [rad]   (s = R*theta)")
        gambar2.set_ylabel("dtheta [rad/s]")
        gambar2.set_title("Gambar 2\nISE theta = %.6f rad^2 s   |   ISE dtheta = %.6f (rad/s)^2 s"
                          % (self.ise_theta, self.ise_dtheta), fontsize=10)
        gambar2.grid(True, color="0.85")
        legenda(gambar2, perpanjangan=False)

        # Gambar 3: X = s, Y = 0
        gambar3.plot(jarak, [0.0] * len(jarak), color=warna, linewidth=2.0)
        gambar3.plot(jarak[0], 0.0, "o", color="tab:green", label="awal")
        gambar3.plot(jarak[-1], 0.0, "x", color="black", markersize=9, label="akhir")
        legenda(gambar3, perpanjangan=False, fontsize=8)

        fig.tight_layout()
        simpan_gambar(fig, path_png)
        plt.close(fig)

        self.file_terakhir = nama
        self.path_terakhir = os.path.join(folder, nama)
        self.data = []
        return nama


# =============================================================================
# Membaca CSV
# =============================================================================
def baca_csv(path):
    """Baca satu file CSV percobaan. Mengembalikan dict berisi list tiap kolom."""
    kolom_angka = ["t_s", "theta_rad", "dtheta_rad_s", "psi_rad", "dpsi_rad_s",
                   "error_theta", "error_dtheta", "error_psi", "error_dpsi", "u_PD_Nm", "u_Nm", "dorong_N"]
    kolom_angka = kolom_angka + KOLOM_GAIN_PD

    with open(path, newline="") as f:
        pembaca = csv.DictReader(f)
        # Kolom Alpha diambil dari judul kolom file (jumlah Alpha bisa berbeda antar percobaan).
        nama_alpha = []
        nomor = 1
        while ("Alpha_%d" % nomor) in (pembaca.fieldnames or []):
            nama_alpha.append("Alpha_%d" % nomor)
            nomor = nomor + 1
        kolom_angka = kolom_angka + nama_alpha

        data = {}
        for nama in kolom_angka:
            data[nama] = []
        data["status_cbf"] = []
        kolom_c = []
        nomor = 1
        while ("c_h%d" % nomor) in (pembaca.fieldnames or []):
            kolom_c.append("c_h%d" % nomor)
            nomor = nomor + 1
        c_dipakai = [False] * len(kolom_c)
        ada_gain = True
        zeta_deg = None                # None = file lama, zeta tidak tercatat
        psi_max_deg = None             # None = file lama, psi_max tidak tercatat (dipakai config.psi_max_DEG)
        for baris in pembaca:
            if zeta_deg is None and baris.get("zeta_DEG") not in (None, ""):
                zeta_deg = float(baris["zeta_DEG"])
            if baris.get("psi_max_DEG") not in (None, ""):
                psi_max_deg = float(baris["psi_max_DEG"])           # nilai terakhir
            for nama in kolom_angka:
                teks = baris.get(nama)
                if teks is None or teks == "":
                    # File lama (sebelum gain dan dorongan dicatat) tidak punya kolom ini.
                    if nama in KOLOM_GAIN_PD:
                        ada_gain = False
                    data[nama].append(0.0)
                else:
                    data[nama].append(float(teks))
            data["status_cbf"].append(baris.get("status_cbf", "-"))
            for k in range(len(kolom_c)):
                if baris.get(kolom_c[k]) not in (None, ""):
                    c_dipakai[k] = True
    data["ada_gain"] = ada_gain
    data["mask"] = c_dipakai if any(c_dipakai) else None      # None = mode PD / file lama: semua barrier merah
    data["nama_alpha"] = nama_alpha
    data["zeta_deg"] = zeta_deg
    data["psi_max_deg"] = psi_max_deg if psi_max_deg is not None else float(config.psi_max_DEG)
    return data


def potong(data, t_akhir):
    """Ambil data dari awal sampai waktu t_akhir saja."""
    jumlah = 0
    for t in data["t_s"]:
        if t <= t_akhir + 1e-9:
            jumlah = jumlah + 1
    hasil = {}
    for nama in data:
        if isinstance(data[nama], list):
            hasil[nama] = data[nama][:jumlah]
        else:
            hasil[nama] = data[nama]
    return hasil


def hitung_ise(t, error):
    """ISE = jumlah (error^2 * dt)."""
    ise = 0.0
    t_sebelumnya = 0.0
    for i in range(len(t)):
        dt = t[i] - t_sebelumnya
        t_sebelumnya = t[i]
        ise = ise + (error[i] ** 2) * dt
    return ise


def nilai_maks(daftar):
    """Nilai mutlak terbesar."""
    maks = 0.0
    for nilai in daftar:
        if abs(nilai) > maks:
            maks = abs(nilai)
    return maks


def hitung_dorongan(dorong):
    """Jumlah dorongan ke depan dan ke belakang (dihitung tiap gaya mulai bekerja)."""
    depan = 0
    belakang = 0
    sebelumnya = 0.0
    for nilai in dorong:
        if sebelumnya == 0.0 and nilai > 0.0:
            depan = depan + 1
        if sebelumnya == 0.0 and nilai < 0.0:
            belakang = belakang + 1
        sebelumnya = nilai
    return depan, belakang


def persen_status(status, nama):
    """Persentase waktu berstatus `nama` dari siklus mode PD + CBF-QP (status "-" = mode PD tidak dihitung)."""
    dipakai = [teks for teks in status if teks != "-"]
    if len(dipakai) == 0:
        return 0.0
    return 100.0 * sum(1 for teks in dipakai if teks == nama) / len(dipakai)


def persen_cbf_ok(status):
    """Persentase waktu CBF-QP berstatus AMAN. Selain AMAN: SLACK (constraint dilonggarkan),
    GAGAL / ALPHA_NOL (u_PD yang dibatasi arus saja). Mengembalikan None untuk mode PD (status "-")."""
    jumlah = 0
    jumlah_ok = 0
    for teks in status:
        if teks == "-":
            continue
        jumlah = jumlah + 1
        if teks == "AMAN":
            jumlah_ok = jumlah_ok + 1
    if jumlah == 0:
        return None
    return 100.0 * jumlah_ok / jumlah


def ringkasan(data):
    """Angka-angka yang ditulis di tabel perbandingan."""
    t = data["t_s"]
    depan, belakang = hitung_dorongan(data["dorong_N"])
    hasil = {
        "durasi": t[-1] if len(t) > 0 else 0.0,
        "ise_psi": hitung_ise(t, data["error_psi"]),
        "ise_dpsi": hitung_ise(t, data["error_dpsi"]),
        "ise_theta": hitung_ise(t, data["error_theta"]),
        "ise_dtheta": hitung_ise(t, data["error_dtheta"]),
        "psi_maks": nilai_maks(data["psi_rad"]),
        "dpsi_maks": nilai_maks(data["dpsi_rad_s"]),
        "dtheta_maks": nilai_maks(data["dtheta_rad_s"]),
        "s_maks": config.R * nilai_maks(data["theta_rad"]),
        "u_maks": nilai_maks(data["u_Nm"]),
        "dorong_depan": depan,
        "dorong_belakang": belakang,
        "dorong_maks": nilai_maks(data["dorong_N"]),
        "cbf_ok": persen_cbf_ok(data["status_cbf"]),
        "alpha_nol": persen_status(data["status_cbf"], STATUS_ALPHA_NOL),
        "slack": persen_status(data["status_cbf"], STATUS_SLACK),
    }
    return hasil


def teks_zeta(zeta_deg):
    if zeta_deg is None:
        return "?"
    return "%.1f deg" % zeta_deg


def gain_dari_data(data):
    """Gain di baris pertama dan baris terakhir file."""
    awal = {}
    akhir = {}
    for nama in KOLOM_GAIN_PD + data["nama_alpha"]:
        awal[nama] = data[nama][0] if len(data[nama]) > 0 else 0.0
        akhir[nama] = data[nama][-1] if len(data[nama]) > 0 else 0.0
    awal["psi_max_DEG"] = akhir["psi_max_DEG"] = data["psi_max_deg"]
    return awal, akhir


# =============================================================================
# python3 ise.py --daftar
# =============================================================================
def perintah_daftar(jenis):
    semua = daftar_percobaan(jenis)
    if len(semua) == 0:
        print("Belum ada file %s di folder %s" % (jenis, folder_jenis(jenis)))
        return
    print("Folder:", folder_jenis(jenis))
    print("%-9s %-7s %-7s %-4s %-7s %8s %7s %12s %12s  %s"
          % ("tanggal", "jam", "mode", "KE", "", "durasi", "zeta", "ISE psi", "ISE theta", "gain"))
    for percobaan in semua:
        data = baca_csv(percobaan["path"])
        hasil = ringkasan(data)
        if data["ada_gain"]:
            awal, akhir = gain_dari_data(data)
            gain = teks_gain(percobaan["mode"], awal, akhir)
        else:
            gain = "(file lama, gain tidak tercatat)"
        print("%-9s %-7s %-7s %02d   %-7s %6.1f s %7s %12.6f %12.6f  %s"
              % (percobaan["tanggal"], percobaan["jam"], percobaan["mode"], percobaan["nomor"],
                 "TUNING" if percobaan["tuning"] else "", hasil["durasi"], teks_zeta(data["zeta_deg"]),
                 hasil["ise_psi"], hasil["ise_theta"], gain))
    print("")
    tambahan = {JENIS_PENGUJIAN: " --pengujian", JENIS_PERBANDINGAN: " --perbandingan"}.get(jenis, "")
    print("Bandingkan:  python3 ise.py --bandingkan%s --pd NOMOR --cbf NOMOR [--tanggal YYYYMMDD]" % tambahan)


# =============================================================================
# python3 ise.py --bandingkan
# =============================================================================
def pilih_percobaan(mode, nomor, tanggal, jenis=JENIS_PERCOBAAN):
    """Percobaan paling baru yang cocok dengan mode, nomor (boleh None), dan tanggal (boleh None).

    Bila nomor tidak disebut, percobaan _TUNING dilewati (dipakai hanya bila tidak ada pilihan lain)."""
    terpilih = None
    terpilih_tuning = None
    for percobaan in daftar_percobaan(jenis):
        if percobaan["mode"] != mode:
            continue
        if nomor is not None and percobaan["nomor"] != nomor:
            continue
        if tanggal is not None and percobaan["tanggal"] != tanggal:
            continue
        # daftar sudah urut, jadi yang terakhir cocok = paling baru
        if nomor is None and percobaan["tuning"]:
            terpilih_tuning = percobaan
        else:
            terpilih = percobaan
    if terpilih is None:
        terpilih = terpilih_tuning
    return terpilih


def label_percobaan(percobaan):
    label = "%s KE-%02d" % (percobaan["mode"], percobaan["nomor"])
    if percobaan["tuning"]:
        label = label + " (TUNING)"
    return label


def cari_percobaan(nama, jenis=JENIS_PERCOBAAN):
    """Percobaan dengan nama file (tanpa .csv) persis sama, atau None."""
    for percobaan in daftar_percobaan(jenis):
        if percobaan["nama"] == nama:
            return percobaan
    return None


def perintah_bandingkan(nomor_pd, nomor_cbf, tanggal, jenis=JENIS_PERCOBAAN, nama_pd=None, nama_cbf=None):
    """nama_pd / nama_cbf (nama file tanpa .csv): langsung memilih file itu (dipakai pembanding otomatis
    atera_main.py di mode pengujian). Bila None, dipilih dari nomor dan tanggal."""
    if nama_pd is not None:
        coba_pd = cari_percobaan(nama_pd, jenis)
    else:
        coba_pd = pilih_percobaan(MODE_PD, nomor_pd, tanggal, jenis)
    if nama_cbf is not None:
        coba_cbf = cari_percobaan(nama_cbf, jenis)
    else:
        coba_cbf = pilih_percobaan(MODE_PD_CBF, nomor_cbf, tanggal, jenis)
    kata = jenis.capitalize()

    keterangan = ""
    if tanggal is not None:
        keterangan = " tanggal " + tanggal
    if coba_pd is None:
        print("%s PD%s%s tidak ditemukan."
              % (kata, "" if nomor_pd is None else " KE-%02d" % nomor_pd, keterangan))
    if coba_cbf is None:
        print("%s PD+CBF%s%s tidak ditemukan."
              % (kata, "" if nomor_cbf is None else " KE-%02d" % nomor_cbf, keterangan))
    if coba_pd is None or coba_cbf is None:
        print("Lihat yang tersedia dengan:  python3 ise.py --daftar%s"
              % {JENIS_PENGUJIAN: " --pengujian", JENIS_PERBANDINGAN: " --perbandingan"}.get(jenis, ""))
        return ""

    data_pd = baca_csv(coba_pd["path"])
    data_cbf = baca_csv(coba_cbf["path"])
    if len(data_pd["t_s"]) == 0 or len(data_cbf["t_s"]) == 0:
        print("Salah satu file CSV kosong.")
        return ""

    # ISE bertambah terus terhadap waktu, jadi dua percobaan hanya adil dibandingkan pada lama waktu
    # yang sama. Yang dipakai: dari t = 0 sampai durasi percobaan yang lebih pendek.
    durasi_pd = data_pd["t_s"][-1]
    durasi_cbf = data_cbf["t_s"][-1]
    t_banding = min(durasi_pd, durasi_cbf)
    potong_pd = potong(data_pd, t_banding)
    potong_cbf = potong(data_cbf, t_banding)
    hasil_pd = ringkasan(potong_pd)
    hasil_cbf = ringkasan(potong_cbf)

    label_pd = label_percobaan(coba_pd)
    label_cbf = label_percobaan(coba_cbf)

    # Catatan yang membuat perbandingan kurang adil
    catatan = []
    if coba_pd["tuning"] or coba_cbf["tuning"]:
        catatan.append("ada percobaan TUNING (gain berubah saat direkam)")
    gain_pd_awal, gain_pd_akhir = gain_dari_data(data_pd)
    gain_cbf_awal, gain_cbf_akhir = gain_dari_data(data_cbf)
    if data_pd["ada_gain"] and data_cbf["ada_gain"]:
        beda = []
        for nama in KOLOM_GAIN_PD:
            if abs(gain_pd_akhir[nama] - gain_cbf_akhir[nama]) > 1e-12:
                beda.append(nama)
        if len(beda) > 0:
            catatan.append("gain PD kedua percobaan berbeda (%s)" % ", ".join(beda))
    dorongan_pd = (hasil_pd["dorong_depan"], hasil_pd["dorong_belakang"], round(hasil_pd["dorong_maks"], 3))
    dorongan_cbf = (hasil_cbf["dorong_depan"], hasil_cbf["dorong_belakang"], round(hasil_cbf["dorong_maks"], 3))
    if data_pd["zeta_deg"] != data_cbf["zeta_deg"]:
        catatan.append("kemiringan lantai (zeta) kedua percobaan berbeda")
    if abs(data_pd["psi_max_deg"] - data_cbf["psi_max_deg"]) > 1e-9:
        catatan.append("psi_max kedua percobaan berbeda (kotak psi = psi_max PD+CBF)")
    if dorongan_pd != dorongan_cbf:
        catatan.append("dorongan kedua percobaan berbeda")
    if hasil_cbf["alpha_nol"] > 0.0:
        catatan.append("CBF-QP TIDAK AKTIF %.1f %% waktu (ALPHA_NOL: ada Alpha = 0), selama itu PD+CBF = PD"
                       % hasil_cbf["alpha_nol"])
    if hasil_cbf["cbf_ok"] is not None and hasil_cbf["cbf_ok"] < 99.95:
        catatan.append("CBF-QP tidak selalu OK (SLACK: constraint dilonggarkan, u paling sedikit melanggar; "
                       "GAGAL / ALPHA_NOL: u = u_PD yang dibatasi arus)")

    # ------------------------------------------------------------------ nama
    awalan = "BANDING" if jenis == JENIS_PERCOBAAN else "BANDING " + jenis
    nama = "%s_%s PD KE-%02d vs PD+CBF KE-%02d" % (coba_cbf["tanggal"], awalan, coba_pd["nomor"], coba_cbf["nomor"])
    if coba_pd["tanggal"] != coba_cbf["tanggal"]:
        nama = "%s_%s PD %s KE-%02d vs PD+CBF KE-%02d" \
            % (coba_cbf["tanggal"], awalan, coba_pd["tanggal"], coba_pd["nomor"], coba_cbf["nomor"])
    os.makedirs(folder_jenis(jenis), exist_ok=True)
    path_png = os.path.join(folder_jenis(jenis), nama + ".png")

    # ----------------------------------------------------------------- tabel
    def teks_dorongan(hasil):
        if hasil["dorong_depan"] == 0 and hasil["dorong_belakang"] == 0:
            return "tidak ada"
        return "%dx depan, %dx belakang (%.1f N)" % (hasil["dorong_depan"], hasil["dorong_belakang"],
                                                     hasil["dorong_maks"])

    def selisih(nilai_pd, nilai_cbf):
        """Perubahan PD+CBF terhadap PD dalam persen (negatif = PD+CBF lebih kecil)."""
        if nilai_pd == 0.0:
            return "-"
        return "%+.1f %%" % (100.0 * (nilai_cbf - nilai_pd) / nilai_pd)

    batas_psi = data_cbf["psi_max_deg"]
    tabel = [
        ["Durasi file [s]", "%.1f" % durasi_pd, "%.1f" % durasi_cbf, ""],
        ["Kemiringan lantai zeta", teks_zeta(data_pd["zeta_deg"]), teks_zeta(data_cbf["zeta_deg"]), ""],
        ["ISE psi [rad^2 s]", "%.6f" % hasil_pd["ise_psi"], "%.6f" % hasil_cbf["ise_psi"],
         selisih(hasil_pd["ise_psi"], hasil_cbf["ise_psi"])],
        ["ISE dpsi [(rad/s)^2 s]", "%.6f" % hasil_pd["ise_dpsi"], "%.6f" % hasil_cbf["ise_dpsi"],
         selisih(hasil_pd["ise_dpsi"], hasil_cbf["ise_dpsi"])],
        ["ISE theta [rad^2 s]", "%.6f" % hasil_pd["ise_theta"], "%.6f" % hasil_cbf["ise_theta"],
         selisih(hasil_pd["ise_theta"], hasil_cbf["ise_theta"])],
        ["ISE dtheta [(rad/s)^2 s]", "%.6f" % hasil_pd["ise_dtheta"], "%.6f" % hasil_cbf["ise_dtheta"],
         selisih(hasil_pd["ise_dtheta"], hasil_cbf["ise_dtheta"])],
        ["|psi| maks [deg]  (batas %.1f)" % batas_psi, "%.2f" % math.degrees(hasil_pd["psi_maks"]),
         "%.2f" % math.degrees(hasil_cbf["psi_maks"]), selisih(hasil_pd["psi_maks"], hasil_cbf["psi_maks"])],
        ["|dpsi| maks [rad/s]  (batas %.2f)" % config.dpsi_max, "%.3f" % hasil_pd["dpsi_maks"],
         "%.3f" % hasil_cbf["dpsi_maks"], selisih(hasil_pd["dpsi_maks"], hasil_cbf["dpsi_maks"])],
        ["|dtheta| maks [rad/s]  (batas %.2f)" % config.dtheta_max, "%.3f" % hasil_pd["dtheta_maks"],
         "%.3f" % hasil_cbf["dtheta_maks"], selisih(hasil_pd["dtheta_maks"], hasil_cbf["dtheta_maks"])],
        ["|s| maks [m]  (batas %s)" % (("%.2f" % config.s_max) if hasattr(config, "s_max") else "-"),
         "%.3f" % hasil_pd["s_maks"], "%.3f" % hasil_cbf["s_maks"], selisih(hasil_pd["s_maks"], hasil_cbf["s_maks"])],
        ["|u| maks [N m]  (batas %.2f)" % config.u_max, "%.3f" % hasil_pd["u_maks"],
         "%.3f" % hasil_cbf["u_maks"], selisih(hasil_pd["u_maks"], hasil_cbf["u_maks"])],
        ["Dorongan (O / P)", teks_dorongan(hasil_pd), teks_dorongan(hasil_cbf), ""],
        ["CBF-QP berstatus OK [% waktu]", "-",
         "-" if hasil_cbf["cbf_ok"] is None else "%.1f" % hasil_cbf["cbf_ok"], ""],
        ["CBF-QP berstatus SLACK [% waktu]", "-", "%.1f" % hasil_cbf["slack"], ""],
    ]
    judul_kolom = ["0 - %.1f s" % t_banding, label_pd, label_cbf, "PD+CBF terhadap PD"]

    # ---------------------------------------------------------------- gambar
    fig = plt.figure(figsize=(13, 13.8))
    kisi = fig.add_gridspec(3, 2, height_ratios=[1.25, 0.34, 1.2])
    gambar1 = fig.add_subplot(kisi[0, 0])
    gambar2 = fig.add_subplot(kisi[0, 1])
    gambar3 = fig.add_subplot(kisi[1, :])
    kotak_tabel = fig.add_subplot(kisi[2, :])

    judul = "%s\nISE dihitung dan data digambar dari t = 0 sampai %.1f s (durasi percobaan yang lebih pendek)" \
        % (nama, t_banding)
    if data_cbf["mask"] is not None:
        judul = judul + "\nconstraint CBF-QP aktif (PD+CBF): " + teks_mask(data_cbf["mask"])
    fig.suptitle(judul, fontsize=11)
    gambar_safety_set(gambar1, gambar2, math.radians(data_cbf["psi_max_deg"]), data_cbf["mask"])
    gambar_batas_jarak(gambar3, data_cbf["mask"])

    # Gambar 1: X = psi, Y = dpsi
    gambar1.plot(potong_pd["psi_rad"], potong_pd["dpsi_rad_s"], color=config.PLOT_WARNA_PD, linewidth=1.0, label=label_pd)
    gambar1.plot(potong_cbf["psi_rad"], potong_cbf["dpsi_rad_s"], color=config.PLOT_WARNA_PD_CBF, linewidth=1.0,
                 label=label_cbf)
    gambar1.set_xlabel("psi [rad]")
    gambar1.set_ylabel("dpsi [rad/s]")
    gambar1.set_title("Gambar 1", fontsize=10)
    gambar1.grid(True, color="0.85")
    legenda(gambar1)

    # Gambar 2: X = theta, Y = dtheta
    gambar2.plot(potong_pd["theta_rad"], potong_pd["dtheta_rad_s"], color=config.PLOT_WARNA_PD, linewidth=1.0, label=label_pd)
    gambar2.plot(potong_cbf["theta_rad"], potong_cbf["dtheta_rad_s"], color=config.PLOT_WARNA_PD_CBF, linewidth=1.0,
                 label=label_cbf)
    gambar2.set_xlabel("theta [rad]   (s = R*theta)")
    gambar2.set_ylabel("dtheta [rad/s]")
    gambar2.set_title("Gambar 2", fontsize=10)
    gambar2.grid(True, color="0.85")
    legenda(gambar2, perpanjangan=False)

    # Gambar 3: X = s, Y = 0 (kedua percobaan di garis yang sama; PD digambar lebih tebal di bawah)
    jarak_pd = [config.R * nilai for nilai in potong_pd["theta_rad"]]
    jarak_cbf = [config.R * nilai for nilai in potong_cbf["theta_rad"]]
    gambar3.plot(jarak_pd, [0.0] * len(jarak_pd), color=config.PLOT_WARNA_PD, linewidth=6.0, label=label_pd)
    gambar3.plot(jarak_cbf, [0.0] * len(jarak_cbf), color=config.PLOT_WARNA_PD_CBF, linewidth=2.0, label=label_cbf)
    legenda(gambar3, perpanjangan=False, fontsize=8)

    # Tabel ISE
    kotak_tabel.axis("off")
    gambar_tabel = kotak_tabel.table(cellText=tabel, colLabels=judul_kolom, loc="upper center", cellLoc="center",
                                     colWidths=[0.32, 0.24, 0.24, 0.20])
    gambar_tabel.auto_set_font_size(False)
    gambar_tabel.set_fontsize(9)
    gambar_tabel.scale(1.0, 1.35)

    keterangan_gain = []
    if data_pd["ada_gain"]:
        keterangan_gain.append("%s:  %s" % (label_pd, teks_gain(MODE_PD, gain_pd_awal, gain_pd_akhir)))
    if data_cbf["ada_gain"]:
        keterangan_gain.append("%s:  %s" % (label_cbf, teks_gain(MODE_PD_CBF, gain_cbf_awal, gain_cbf_akhir)))
    kotak_tabel.text(0.5, 0.08, "\n".join(keterangan_gain), ha="center", va="top", fontsize=8.5,
                     transform=kotak_tabel.transAxes)
    if len(catatan) > 0:
        kotak_tabel.text(0.5, -0.01, "PERHATIAN: " + ";\n".join(catatan), ha="center", va="top", fontsize=9,
                         color="tab:red", transform=kotak_tabel.transAxes)

    fig.tight_layout()
    simpan_gambar(fig, path_png)
    plt.close(fig)

    # -------------------------------------------------------------- terminal
    print("PD     :", coba_pd["nama"])
    print("PD+CBF :", coba_cbf["nama"])
    print("Dibandingkan pada t = 0 sampai %.1f s (durasi percobaan yang lebih pendek)." % t_banding)
    print("")
    print("%-34s %-28s %-28s %s" % (judul_kolom[0], judul_kolom[1], judul_kolom[2], judul_kolom[3]))
    for baris in tabel:
        print("%-34s %-28s %-28s %s" % (baris[0], baris[1], baris[2], baris[3]))
    print("")
    for teks in keterangan_gain:
        print(teks)
    for teks in catatan:
        print("PERHATIAN:", teks)
    print("Tersimpan:", path_png)
    return nama


def main():
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--daftar", action="store_true")
    parser.add_argument("--bandingkan", action="store_true")
    parser.add_argument("--pd", type=int, default=None)
    parser.add_argument("--cbf", type=int, default=None)
    parser.add_argument("--tanggal", type=str, default=None)
    parser.add_argument("--pengujian", action="store_true")
    parser.add_argument("--perbandingan", action="store_true")
    parser.add_argument("-h", "--help", action="store_true")
    pilihan = parser.parse_args()

    jenis = JENIS_PERCOBAAN
    if pilihan.pengujian:
        jenis = JENIS_PENGUJIAN
    if pilihan.perbandingan:
        jenis = JENIS_PERBANDINGAN
    if pilihan.daftar:
        perintah_daftar(jenis)
    elif pilihan.bandingkan:
        perintah_bandingkan(pilihan.pd, pilihan.cbf, pilihan.tanggal, jenis)
    else:
        print(BANTUAN)


if __name__ == "__main__":
    main()