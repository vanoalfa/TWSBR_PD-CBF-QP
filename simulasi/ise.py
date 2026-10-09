"""ise.py - Evaluasi ISE (Integral Square Error) robot ATERA.

ISE = jumlah (error^2 * dt), dihitung untuk theta, dtheta, psi, dan dpsi.

Hasil disimpan di dua subfolder PLOT_EVALUASI (HHmmSS = jam, menit, detik saat disimpan):
    PLOT_EVALUASI/PERCOBAAN/YYYYMMDD_HHmmSS_COBA PD_PERCOBAAN KE-XX.csv / .png       (perekaman biasa)
    PLOT_EVALUASI/PENGUJIAN/YYYYMMDD_HHmmSS_COBA PD_PENGUJIAN KE-XX.csv / .png       (mode pengujian, tombol T)
    (PD diganti PD+CBF untuk mode PD + CBF-QP)
Nomor KE-XX naik otomatis per tanggal, per mode, dan per jenis (PERCOBAAN / PENGUJIAN).
File lama (langsung di PLOT_EVALUASI, atau tanpa HHmmSS) tetap terbaca sebagai PERCOBAAN.
Bila gain diubah saat sedang direkam, nama file diberi akhiran _TUNING.

Isi gambar:
    Gambar 1: sumbu X = psi,   sumbu Y = dpsi       kotak batas psi dan dpsi
    Gambar 2: sumbu X = theta, sumbu Y = dtheta     kotak batas jarak (theta = s/R) dan dtheta
    Gambar 3: sumbu X = s = R*theta [m], sumbu Y = 0   batas s_min dan s_max
Garis batas safety set (merah):
    - bagian yang membentuk kotak barrier : garis tebal menyambung
    - perpanjangan garis di luar kotak    : garis tipis putus-putus
    - Gambar 3 (jarak s) hanya punya 2 batas, jadi keduanya garis tebal menyambung

Perintah (dijalankan dari terminal), tambahkan --pengujian untuk memakai folder PENGUJIAN:
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

import config

FOLDER_PLOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), config.ISE_PLOT_DIR)

JENIS_PERCOBAAN = "PERCOBAAN"       # perekaman biasa (E / Q / robot jatuh)
JENIS_PENGUJIAN = "PENGUJIAN"       # mode pengujian (tombol T): dari dorongan O / P selama beberapa detik


def folder_jenis(jenis):
    """Subfolder PLOT_EVALUASI untuk jenis PERCOBAAN atau PENGUJIAN."""
    return os.path.join(FOLDER_PLOT, jenis)

MODE_PD = "PD"
MODE_PD_CBF = "PD+CBF"

def cari_nama_alpha():
    """Nama Alpha di config.py: Alpha_1, Alpha_2, ... (berurutan, berhenti di nomor pertama yang tidak ada)."""
    hasil = []
    nomor = 1
    while hasattr(config, "Alpha_%d" % nomor):
        hasil.append("Alpha_%d" % nomor)
        nomor = nomor + 1
    return hasil


KOLOM_GAIN_PD = ["Kp_psi", "Kd_psi", "Kp_theta", "Kd_theta", "Kvel"]
KOLOM_GAIN_CBF = cari_nama_alpha()
KOLOM_GAIN = KOLOM_GAIN_PD + KOLOM_GAIN_CBF

KOLOM_CSV = ["t_s", "theta_rad", "dtheta_rad_s", "psi_rad", "dpsi_rad_s",
             "error_theta", "error_dtheta", "error_psi", "error_dpsi",
             "u_PD_Nm", "u_Nm", "status_cbf", "dorong_N",
             "ISE_theta", "ISE_dtheta", "ISE_psi", "ISE_dpsi"] + KOLOM_GAIN + ["zeta_DEG"]

# Contoh nama: 20261008_201530_COBA PD+CBF_PENGUJIAN KE-05_TUNING.csv  (bagian _HHmmSS boleh tidak ada: file lama)
POLA_NAMA = re.compile(r"^(\d{8})(?:_(\d{6}))?_COBA (PD\+CBF|PD)_(PERCOBAAN|PENGUJIAN) KE-(\d+)(_TUNING)?\.csv$")

# Garis batas safety set di semua gambar
SAFETY_WARNA = "red"
SAFETY_TEBAL = 2.2          # garis yang membentuk kotak barrier (menyambung)
SAFETY_TIPIS = 1.0          # perpanjangan garis di luar kotak (putus-putus)

WARNA_PD = "tab:blue"
WARNA_PD_CBF = "tab:orange"

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


def gambar_kotak_safety(ax, x_min, x_max, y_min, y_max):
    """Batas safety set dua variabel.
    Kotak barrier (x_min..x_max, y_min..y_max): merah tebal menyambung.
    Perpanjangan keempat garis di luar kotak: merah tipis putus-putus."""
    ax.axvline(x_min, color=SAFETY_WARNA, linewidth=SAFETY_TIPIS, linestyle="--", zorder=1.4,
               label="perpanjangan batas")
    ax.axvline(x_max, color=SAFETY_WARNA, linewidth=SAFETY_TIPIS, linestyle="--", zorder=1.4)
    ax.axhline(y_min, color=SAFETY_WARNA, linewidth=SAFETY_TIPIS, linestyle="--", zorder=1.4)
    ax.axhline(y_max, color=SAFETY_WARNA, linewidth=SAFETY_TIPIS, linestyle="--", zorder=1.4)
    ax.plot([x_min, x_max, x_max, x_min, x_min], [y_min, y_min, y_max, y_max, y_min],
            color=SAFETY_WARNA, linewidth=SAFETY_TEBAL, zorder=1.5, label="batas safety set (barrier)")


def ada_batas_jarak():
    return hasattr(config, "s_max") and hasattr(config, "s_min")


def gambar_safety_set(gambar1, gambar2):
    """Gambar 1: kotak psi x dpsi.  Gambar 2: kotak theta (= s/R) x dtheta."""
    gambar_kotak_safety(gambar1, config.psi_min, config.psi_max, config.dpsi_min, config.dpsi_max)
    if ada_batas_jarak():
        gambar_kotak_safety(gambar2, config.s_min / config.R, config.s_max / config.R,
                            config.dtheta_min, config.dtheta_max)
    else:
        # tanpa batas jarak tidak ada kotak: hanya 2 garis dtheta
        gambar2.axhline(config.dtheta_max, color=SAFETY_WARNA, linewidth=SAFETY_TEBAL, label="batas safety set")
        gambar2.axhline(config.dtheta_min, color=SAFETY_WARNA, linewidth=SAFETY_TEBAL)


def gambar_batas_jarak(gambar3):
    """Gambar 3 (jarak s): hanya 2 batas (s_min, s_max), keduanya garis tebal menyambung."""
    if ada_batas_jarak():
        gambar3.axvline(config.s_max, color=SAFETY_WARNA, linewidth=SAFETY_TEBAL, zorder=1.5,
                        label="batas safety set (barrier)")
        gambar3.axvline(config.s_min, color=SAFETY_WARNA, linewidth=SAFETY_TEBAL, zorder=1.5)
    gambar3.axhline(0.0, color="gray", linewidth=0.6, zorder=1.0)
    gambar3.set_ylim(-1.0, 1.0)
    gambar3.set_yticks([0.0])
    gambar3.set_xlabel("s = R*theta [m]   (jarak tempuh dari posisi awal)")
    gambar3.set_title("Gambar 3", fontsize=10)
    gambar3.grid(True, axis="x", alpha=0.4)


def angka(nilai):
    """Angka pendek untuk tulisan gain: 15.0 -> '15', 0.0400 -> '0.04'."""
    return "%g" % nilai


def teks_gain(mode, gain_awal, gain_akhir):
    """Contoh: 'Kp_psi 15 | Kd_psi 1.5 -> 2 | ...'. Alpha hanya ditulis untuk mode PD+CBF."""
    daftar = list(KOLOM_GAIN_PD)
    if mode == MODE_PD_CBF:
        # Alpha yang ada di data (jumlahnya bisa berbeda antar file)
        nomor = 1
        while ("Alpha_%d" % nomor) in gain_akhir:
            daftar.append("Alpha_%d" % nomor)
            nomor = nomor + 1
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

    def tambah(self, t, x, error_theta, error_dtheta, error_psi, error_dpsi, u_PD, u, status_cbf, gain, dorong=0.0):
        """Dipanggil tiap siklus kontrol.

        gain   : dict berisi Kp_psi, Kd_psi, Kp_theta, Kd_theta, Kvel, Alpha_1, Alpha_2, ... dan zeta_DEG
        dorong : gaya dorongan yang sedang bekerja [N] (+ ke depan, - ke belakang, 0 tidak ada)
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
            tulis.writerow(KOLOM_CSV)
            for baris in self.data:
                baris_teks = []
                for nilai in baris:
                    if isinstance(nilai, str):
                        baris_teks.append(nilai)
                    else:
                        baris_teks.append("%.8g" % nilai)
                tulis.writerow(baris_teks)

        # ----------------------------------------------------------------- PNG
        theta = [baris[1] for baris in self.data]
        dtheta = [baris[2] for baris in self.data]
        psi = [baris[3] for baris in self.data]
        dpsi = [baris[4] for baris in self.data]
        durasi = self.data[-1][0]

        judul = "%s  |  mode %s  |  durasi %.1f s  |  zeta %.1f deg" % (nama, self.mode, durasi, self.zeta_deg)
        judul = judul + "\n" + teks_gain(self.mode, self.gain_awal, self.gain_akhir)
        if self.mode == MODE_PD_CBF:
            cbf_ok = persen_cbf_ok([baris[11] for baris in self.data])
            judul = judul + "  |  CBF-QP berstatus OK %.1f %% waktu" % cbf_ok
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
        gambar_safety_set(gambar1, gambar2)
        gambar_batas_jarak(gambar3)

        # Gambar 1: X = psi, Y = dpsi
        gambar1.plot(psi, dpsi, color="tab:blue", linewidth=1.0)
        gambar1.plot(psi[0], dpsi[0], "o", color="tab:green", label="awal")
        gambar1.plot(psi[-1], dpsi[-1], "x", color="black", markersize=9, label="akhir")
        gambar1.set_xlabel("psi [rad]")
        gambar1.set_ylabel("dpsi [rad/s]")
        gambar1.set_title("Gambar 1\nISE psi = %.6f rad^2 s   |   ISE dpsi = %.6f (rad/s)^2 s"
                          % (self.ise_psi, self.ise_dpsi), fontsize=10)
        gambar1.grid(True, alpha=0.4)
        gambar1.legend(fontsize=8)

        # Gambar 2: X = theta, Y = dtheta
        gambar2.plot(theta, dtheta, color="tab:orange", linewidth=1.0)
        gambar2.plot(theta[0], dtheta[0], "o", color="tab:green", label="awal")
        gambar2.plot(theta[-1], dtheta[-1], "x", color="black", markersize=9, label="akhir")
        gambar2.set_xlabel("theta [rad]   (s = R*theta)")
        gambar2.set_ylabel("dtheta [rad/s]")
        gambar2.set_title("Gambar 2\nISE theta = %.6f rad^2 s   |   ISE dtheta = %.6f (rad/s)^2 s"
                          % (self.ise_theta, self.ise_dtheta), fontsize=10)
        gambar2.grid(True, alpha=0.4)
        gambar2.legend(fontsize=8)

        # Gambar 3: X = s, Y = 0
        gambar3.plot(jarak, [0.0] * len(jarak), color="tab:purple", linewidth=2.0)
        gambar3.plot(jarak[0], 0.0, "o", color="tab:green", label="awal")
        gambar3.plot(jarak[-1], 0.0, "x", color="black", markersize=9, label="akhir")
        gambar3.legend(fontsize=8, loc="upper right")

        fig.tight_layout()
        fig.savefig(path_png, dpi=130)
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
        ada_gain = True
        zeta_deg = None                # None = file lama, zeta tidak tercatat
        for baris in pembaca:
            if zeta_deg is None and baris.get("zeta_DEG") not in (None, ""):
                zeta_deg = float(baris["zeta_DEG"])
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
    data["ada_gain"] = ada_gain
    data["nama_alpha"] = nama_alpha
    data["zeta_deg"] = zeta_deg
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


def persen_cbf_ok(status):
    """Persentase waktu CBF-QP berstatus OK. Selain OK (INFEASIBLE, SOLVER_GAGAL, ALPHA_NOL), keluaran
    cbf_qp.py adalah u_PD yang dibatasi arus saja. Mengembalikan None untuk mode PD (status "-")."""
    jumlah = 0
    jumlah_ok = 0
    for teks in status:
        if teks == "-":
            continue
        jumlah = jumlah + 1
        if teks == "OK":
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
    tambahan = " --pengujian" if jenis == JENIS_PENGUJIAN else ""
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


def perintah_bandingkan(nomor_pd, nomor_cbf, tanggal, jenis=JENIS_PERCOBAAN):
    coba_pd = pilih_percobaan(MODE_PD, nomor_pd, tanggal, jenis)
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
              % (" --pengujian" if jenis == JENIS_PENGUJIAN else ""))
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
    if dorongan_pd != dorongan_cbf:
        catatan.append("dorongan kedua percobaan berbeda")
    if hasil_cbf["cbf_ok"] is not None and hasil_cbf["cbf_ok"] < 99.95:
        catatan.append("CBF-QP tidak selalu OK (saat tidak OK, u = u_PD yang dibatasi arus)")

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

    batas_psi = math.degrees(config.psi_max)
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
    ]
    judul_kolom = ["0 - %.1f s" % t_banding, label_pd, label_cbf, "PD+CBF terhadap PD"]

    # ---------------------------------------------------------------- gambar
    fig = plt.figure(figsize=(13, 13.0))
    kisi = fig.add_gridspec(3, 2, height_ratios=[1.25, 0.34, 1.05])
    gambar1 = fig.add_subplot(kisi[0, 0])
    gambar2 = fig.add_subplot(kisi[0, 1])
    gambar3 = fig.add_subplot(kisi[1, :])
    kotak_tabel = fig.add_subplot(kisi[2, :])

    judul = "%s\nISE dihitung dan data digambar dari t = 0 sampai %.1f s (durasi percobaan yang lebih pendek)" \
        % (nama, t_banding)
    fig.suptitle(judul, fontsize=11)
    gambar_safety_set(gambar1, gambar2)
    gambar_batas_jarak(gambar3)

    # Gambar 1: X = psi, Y = dpsi
    gambar1.plot(potong_pd["psi_rad"], potong_pd["dpsi_rad_s"], color=WARNA_PD, linewidth=1.0, label=label_pd)
    gambar1.plot(potong_cbf["psi_rad"], potong_cbf["dpsi_rad_s"], color=WARNA_PD_CBF, linewidth=1.0,
                 label=label_cbf)
    gambar1.set_xlabel("psi [rad]")
    gambar1.set_ylabel("dpsi [rad/s]")
    gambar1.set_title("Gambar 1", fontsize=10)
    gambar1.grid(True, alpha=0.4)
    gambar1.legend(fontsize=8)

    # Gambar 2: X = theta, Y = dtheta
    gambar2.plot(potong_pd["theta_rad"], potong_pd["dtheta_rad_s"], color=WARNA_PD, linewidth=1.0, label=label_pd)
    gambar2.plot(potong_cbf["theta_rad"], potong_cbf["dtheta_rad_s"], color=WARNA_PD_CBF, linewidth=1.0,
                 label=label_cbf)
    gambar2.set_xlabel("theta [rad]   (s = R*theta)")
    gambar2.set_ylabel("dtheta [rad/s]")
    gambar2.set_title("Gambar 2", fontsize=10)
    gambar2.grid(True, alpha=0.4)
    gambar2.legend(fontsize=8)

    # Gambar 3: X = s, Y = 0 (kedua percobaan di garis yang sama; PD digambar lebih tebal di bawah)
    jarak_pd = [config.R * nilai for nilai in potong_pd["theta_rad"]]
    jarak_cbf = [config.R * nilai for nilai in potong_cbf["theta_rad"]]
    gambar3.plot(jarak_pd, [0.0] * len(jarak_pd), color=WARNA_PD, linewidth=6.0, alpha=0.6, label=label_pd)
    gambar3.plot(jarak_cbf, [0.0] * len(jarak_cbf), color=WARNA_PD_CBF, linewidth=2.0, label=label_cbf)
    gambar3.legend(fontsize=8, loc="upper right")

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
    fig.savefig(path_png, dpi=130)
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
    parser.add_argument("-h", "--help", action="store_true")
    pilihan = parser.parse_args()

    jenis = JENIS_PENGUJIAN if pilihan.pengujian else JENIS_PERCOBAAN
    if pilihan.daftar:
        perintah_daftar(jenis)
    elif pilihan.bandingkan:
        perintah_bandingkan(pilihan.pd, pilihan.cbf, pilihan.tanggal, jenis)
    else:
        print(BANTUAN)


if __name__ == "__main__":
    main()