"""tuning_optuna.py - mencari gain PD dan Alpha CBF-QP terbaik secara bertahap dengan Optuna (simulasi PyBullet).

Dua profil (config.OPTUNA_PROFIL), masing-masing dengan study sendiri supaya hasilnya tidak tercampur:
    TERUKUR   : torsi roda 0.238 N m/A (hasil uji delay)
    DATASHEET : torsi roda 0.75 N m/A (datasheet Waveshare)   -> BELUM TERVERIFIKASI

Lima tahap (config.OPTUNA_TAHAP), dijalankan satu per satu. Satu tahap = satu study ATERA_<TAHAP>_<PROFIL>
di database OPTUNA/atera_optuna.db:
    PD            Kp_psi, Kd_psi, Kp_theta, Kd_theta   mode PD saja, cari gaya dorong tertinggi yang lolos
    CBFQP_PSI     Alpha_1, Alpha_2           PD + CBF-QP, barrier h1 h2
    CBFQP_DPSI    Alpha_3                    PD + CBF-QP, barrier h1 .. h4
    CBFQP_S       Alpha_4, Alpha_5           PD + CBF-QP, barrier h1 .. h6
    CBFQP_DTHETA  Alpha_6                    PD + CBF-QP, barrier h1 .. h8
Variabel yang tidak dituning di satu tahap SELALU diambil dari config.py, jadi salin hasil tiap tahap ke config.py
sebelum menjalankan tahap berikutnya. Batas pencarian: config.OPTUNA_BATAS (bawah, atas) per variabel.
Study tidak boleh dilanjutkan bila nilai yang dikunci atau skenario (dorongan, durasi, penalti, dst) berubah:
program berhenti dan menjelaskan bedanya, supaya skor lama dan baru tidak tercampur.

Satu PENGUJIAN (headless, lantai config.OPTUNA_ZETA_DEG, mode Balancing) = OPTUNA_ULANGAN ulangan skenario, robot
di-reset di awal tiap ulangan:
    tenang sampai stabil -> dorongan berurutan OPTUNA_ARAH_DORONG berjarak OPTUNA_DURASI_DORONGAN s (stabil atau
    belum) -> tunggu stabil setelah dorongan terakhir. Ulangan ke-k memakai seed noise OPTUNA_SEED + k dan waktu
    dorong yang digeser k x OPTUNA_GESER_DORONG s. Skor pengujian = jumlah skor semua ulangan.
Satu TRIAL:
    tahap PD     : bisection gaya dorong (profil gaya_dorong .. OPTUNA_GAYA_MAKS, langkah OPTUNA_GAYA_LANGKAH);
                   tiap gaya yang dicoba = satu pengujian. Skor = -OPTUNA_BOBOT_GAYA x gaya_lolos + skor pengujian
                   pada gaya_lolos (gaya lebih tinggi selalu lebih baik; ISE hanya membedakan gaya yang sama).
    tahap CBFQP_*: satu pengujian pada gaya gaya_pd_terbaik + OPTUNA_GAYA_TAMBAH_CBF.
Rumus skor pengujian: config.py bagian 13. Makin kecil makin baik.

Log terminal tiap trial: trial | status | skor | waktu nyata | terbaik | gaya | tanda/s | psimaks | gagal | detail
    gaya    = gaya lolos tertinggi [N] (tahap PD) atau gaya yang dipakai [N] (tahap CBFQP_*)
    tanda/s = pergantian tanda torsi u per detik (puluhan per detik = bang-bang), tidak masuk skor
    psimaks = puncak |psi| [deg],  gagal = lama status CBF bukan AMAN [s] (tahap CBFQP_*)
    status  = selesai (tanpa komet) | komet | pruned
    detail  = penyebab komet / gaya yang gagal (tahap PD), dan fase yang tidak mencapai stabil (tanpa penalti):
              awal (sebelum dorongan pertama) / akhir (setelah dorongan terakhir), U1 / U2 = ulangan ke-1 / ke-2
CSV memuat lebih banyak: puncak |psi|, |dpsi|, |s|, lama GAGAL, jumlah fase tidak stabil, puncak |psi|, |dpsi|,
|dtheta| pada OPTUNA_STABIL_LAMA detik terakhir fase awal dan fase terakhir, dan skor yang dipecah menjadi ISE dan
tiap penalti. Untuk tahap PD, diagnostik berasal dari pengujian pada gaya lolos tertinggi (atau gaya terendah bila
semua komet).

Jalankan (di folder simulasi):
    python3 tuning_optuna.py --profil TERUKUR --tahap PD            jumlah trial dari config.OPTUNA_TAHAP
    python3 tuning_optuna.py --profil TERUKUR --tahap CBFQP_PSI --trial 50   MENAMBAH 50 trial ke study yang ada
    python3 tuning_optuna.py --hasil                                cetak hasil terbaik semua profil dan tahap, buat ulang CSV + PNG
    python3 tuning_optuna.py --hasil --profil TERUKUR               hanya satu profil
    python3 tuning_optuna.py --profil TERUKUR --hapus PD            hapus study satu tahap (mulai dari nol)

Hasil (folder OPTUNA):
    atera_optuna.db                                      database semua trial (bisa dilanjutkan kapan saja)
    YYYYMMDD_HHmmSS_OPTUNA_PD_TERUKUR.csv / .png         riwayat trial + respon gain terbaik di zeta 0 dan 5.2 deg
    (untuk profil DATASHEET: ..._DATASHEET, diberi label BELUM TERVERIFIKASI)
Nilai terbaik dicetak di terminal dalam format config.py (termasuk gaya_pd_terbaik untuk tahap PD). Salin sendiri
ke config.py.

Butuh: pip install optuna
Nama file sengaja BUKAN optuna.py: file bernama optuna.py akan menutupi library optuna saat "import optuna".
"""

import argparse
import csv
import math
import os
import sys
import time
from collections import deque
from datetime import datetime

import numpy as np

try:
    import optuna
except ImportError:
    print("Library optuna belum terpasang:  pip install optuna")
    sys.exit(1)

import config
import model
import simulasi
from pd_control import MODE_BALANCING, PDControl

FOLDER_INI = os.path.dirname(os.path.abspath(__file__))
FOLDER_HASIL = os.path.join(FOLDER_INI, "OPTUNA")
DATABASE = "sqlite:///" + os.path.join(FOLDER_HASIL, "atera_optuna.db")
NAMA_GAIN_PD = ["Kp_psi", "Kd_psi", "Kp_theta", "Kd_theta"]
TEKS_ARAH = {1: "depan", -1: "belakang"}
STATUS_SELESAI = "selesai"
STATUS_KOMET = "komet"

# Diagnostik tiap trial (disimpan sebagai atribut trial dan kolom CSV).
# skor pengujian = ise + pen_jarak + pen_gagal + pen_komet
# gaya_N              = gaya lolos tertinggi (tahap PD, 0 bila tidak ada) atau gaya yang dipakai (tahap CBFQP_*)
# tanda_u_per_s       = pergantian tanda torsi u per detik bagian yang dinilai (bang-bang bila puluhan per detik)
# jumlah_tidak_stabil = jumlah fase (tenang awal, setelah dorongan terakhir, semua ulangan) yang tidak mencapai stabil
# awal_* / akhir_*    = puncak |psi|, |dpsi|, |dtheta| pada OPTUNA_STABIL_LAMA detik terakhir fase awal / fase
#                       terakhir, terbesar dari semua ulangan (satuan sama dengan OPTUNA_STABIL_*; 0 bila tidak
#                       tercapai karena komet)
KOLOM_STABIL = ["psi_rad", "dpsi_rad_s", "dtheta_rad_s"]
KOLOM_DIAG = (["gaya_N", "tanda_u_per_s", "psi_maks_deg", "dpsi_maks_rad_s", "s_maks_m", "lama_gagal_s",
               "jumlah_tidak_stabil"]
              + ["awal_" + nama for nama in KOLOM_STABIL] + ["akhir_" + nama for nama in KOLOM_STABIL]
              + ["ise", "pen_jarak", "pen_gagal", "pen_komet"])

# Baris log terminal (lihat docstring): header dan isi memakai lebar kolom yang sama.
JUDUL_KOLOM = ("trial", "status", "skor", "waktu", "terbaik", "gaya", "tanda/s", "psimaks", "gagal", "detail")
FORMAT_HEADER = "%5s | %-7s | %11s | %7s | %-18s | %6s | %7s | %7s | %6s | %s"
FORMAT_BARIS = "%5d | %-7s | %11s | %5.1f s | %-18s | %6s | %7.1f | %7.1f | %6s | %s"


def langkah(detik):
    """Jumlah langkah kontrol (config.DT) untuk durasi [s]."""
    return int(round(detik / config.DT))


def nama_dorongan():
    """Nama tiap dorongan sesuai urutan config.OPTUNA_ARAH_DORONG: depan_1, belakang_1, depan_2, belakang_2."""
    hitung = {1: 0, -1: 0}
    hasil = []
    for arah in config.OPTUNA_ARAH_DORONG:
        hitung[arah] = hitung[arah] + 1
        hasil.append("%s_%d" % (TEKS_ARAH[arah], hitung[arah]))
    return hasil


def jarak_dorongan(ulangan):
    """Jarak antar dorongan [s] pada ulangan ke-`ulangan` (0 = ulangan pertama)."""
    return config.OPTUNA_DURASI_DORONGAN + ulangan * config.OPTUNA_GESER_DORONG


def waktu_dinilai_ulangan_maks():
    """Lama maksimum bagian yang dinilai dalam satu ulangan [s] (dari dorongan pertama sampai batas tunggu setelah
    dorongan terakhir), untuk ulangan dengan jarak dorongan terpanjang."""
    jarak_terpanjang = jarak_dorongan(config.OPTUNA_ULANGAN - 1)
    return (len(config.OPTUNA_ARAH_DORONG) - 1) * jarak_terpanjang + config.OPTUNA_DURASI_STABIL


def waktu_dinilai_maks():
    """Lama maksimum bagian yang dinilai dalam satu pengujian (semua ulangan) [s]."""
    return config.OPTUNA_ULANGAN * waktu_dinilai_ulangan_maks()


def daftar_gaya(profil):
    """Kisi gaya dorong tahap PD [N]: gaya_dorong profil .. OPTUNA_GAYA_MAKS, berjarak OPTUNA_GAYA_LANGKAH."""
    awal = float(config.OPTUNA_PROFIL[profil]["gaya_dorong"])
    maks = float(config.OPTUNA_GAYA_MAKS)
    jarak = float(config.OPTUNA_GAYA_LANGKAH)
    if jarak <= 0.0 or maks < awal:
        sys.exit("Periksa config.py: perlu OPTUNA_GAYA_LANGKAH > 0 dan OPTUNA_GAYA_MAKS (%.2f) >= gaya_dorong "
                 "profil %s (%.2f)." % (maks, profil, awal))
    jumlah = int(math.floor((maks - awal) / jarak + 1e-9))
    return [round(awal + i * jarak, 6) for i in range(jumlah + 1)]


def gaya_cbf(profil):
    """Gaya dorong tahap CBFQP_* [N] = gaya_pd_terbaik + OPTUNA_GAYA_TAMBAH_CBF, atau None bila belum diisi."""
    terbaik = config.OPTUNA_PROFIL[profil].get("gaya_pd_terbaik")
    if terbaik is None:
        return None
    return float(terbaik) + float(config.OPTUNA_GAYA_TAMBAH_CBF)


def nama_study(tahap, profil):
    return "ATERA_%s_%s" % (tahap, profil)                # ATERA_PD_TERUKUR, ATERA_CBFQP_S_DATASHEET, ...


def nama_file(tahap, profil):
    return "OPTUNA_%s_%s" % (tahap, profil)


def semua_variabel():
    return NAMA_GAIN_PD + config.NAMA_ALPHA


def batas_pencarian(variabel):
    """(bawah, atas): dict nama -> batas, dari config.OPTUNA_BATAS. Berhenti dengan pesan bila batasnya salah."""
    bawah = {}
    atas = {}
    for nama in variabel:
        if nama not in config.OPTUNA_BATAS:
            sys.exit("%s belum ada di config.OPTUNA_BATAS. Isi dengan (batas bawah, batas atas)." % nama)
        bawah[nama] = float(config.OPTUNA_BATAS[nama][0])
        atas[nama] = float(config.OPTUNA_BATAS[nama][1])
        if atas[nama] <= bawah[nama]:
            sys.exit("Batas atas %s (%.4f) harus lebih besar dari batas bawah (%.4f). Periksa config.OPTUNA_BATAS."
                     % (nama, atas[nama], bawah[nama]))
        if nama in config.NAMA_ALPHA and bawah[nama] <= 0.0:
            sys.exit("Batas bawah %s harus > 0: Alpha = 0 mematikan CBF-QP. Periksa config.OPTUNA_BATAS." % nama)
    return bawah, atas


def teks_label(profil):
    """Label profil untuk terminal, judul PNG, dan atribut study."""
    data = config.OPTUNA_PROFIL[profil]
    teks = "profil %s (torsi roda %.3f N m/A)" % (profil, data["kt_efektif"])
    if not data["terverifikasi"]:
        teks = teks + " - BELUM TERVERIFIKASI"
    return teks


def teks_gaya(gaya):
    return "%.1f" % gaya if gaya > 0.0 else "-"


# =============================================================================
# PENGUJI: satu simulasi headless dipakai ulang untuk semua trial
# =============================================================================
class Penguji:
    def __init__(self, profil, pakai_cbf):
        """pakai_cbf: True bila ada tahap CBFQP_* yang akan dijalankan (membangun solver acados, butuh beberapa detik)."""
        # PyBullet hanya boleh satu dunia per program, jadi Penguji cukup dibuat SATU kali.
        self.sim = simulasi.Simulasi(gui=False)
        self.pd = PDControl()
        self.cbf = None
        if pakai_cbf:
            print("Menyiapkan CBF-QP (acados), tunggu beberapa detik ...")
            from cbf_qp import CBFQP
            self.cbf = CBFQP()
        # Nilai config.py saat program mulai: dasar semua variabel yang tidak dituning (nilai terkunci).
        self.dasar = {nama: float(getattr(config, nama)) for nama in semua_variabel()}
        self.tahap = None
        self.pakai_cbf = False              # tahap sekarang memakai CBF-QP
        self.paksa = {}                     # nilai yang dipaksa selama tahap sekarang (menggantikan config.py)
        self.skor_s = True                  # False: skor hanya dari psi
        self.penalti_jarak = True           # False: OPTUNA_PENALTI_JARAK tidak dihitung
        self.cari_gaya = False              # True: tiap trial mencari gaya dorong tertinggi yang lolos (tahap PD)
        self.gaya_tetap = None              # gaya dorong tahap CBFQP_* [N]
        self.diag = dict.fromkeys(KOLOM_DIAG, 0.0)       # diagnostik trial terakhir (lihat KOLOM_DIAG)
        self.zeta_deg = None
        self.profil = None
        self.atur_profil(profil)

    def atur_profil(self, profil):
        """Pasang torsi roda nyata sesuai config.OPTUNA_PROFIL (hanya untuk simulasi ini)."""
        data = config.OPTUNA_PROFIL[profil]
        self.profil = profil
        rasio_torsi = float(data["kt_efektif"]) / config.MOTOR_KT          # torsi nyata / torsi perintah
        self.sim.skala_torsi = rasio_torsi
        if self.cbf is not None:
            self.cbf.rasio_torsi = rasio_torsi                              # model CBF-QP sama dengan simulasi

    def atur_tahap(self, tahap):
        """Pasang pengaturan tahap: mode PD / PD + CBF-QP, barrier yang aktif, nilai paksa, jenis skor, dan gaya."""
        definisi = config.OPTUNA_TAHAP[tahap]
        self.tahap = tahap
        self.pakai_cbf = definisi["barrier"] is not None
        if self.pakai_cbf:
            if self.cbf is None:
                raise RuntimeError("Tahap %s butuh CBF-QP, tetapi Penguji dibuat tanpa CBF-QP." % tahap)
            self.cbf.pilih_barrier(definisi["barrier"])
        self.paksa = dict(definisi.get("paksa", {}))
        self.skor_s = bool(definisi.get("skor_s", True))
        self.penalti_jarak = self.skor_s and bool(definisi.get("penalti_jarak", True))
        self.cari_gaya = bool(definisi.get("cari_gaya", False))
        self.gaya_tetap = None if self.cari_gaya else gaya_cbf(self.profil)

    def atur_zeta(self, zeta_deg):
        if self.zeta_deg == zeta_deg:
            return
        self.zeta_deg = zeta_deg
        config.zeta_DEG = float(zeta_deg)
        config.zeta = math.radians(zeta_deg)           # dipakai model.py dan cbf_qp.py
        self.sim.ubah_zeta(config.zeta)

    def atur_gain(self, gain):
        """gain: dict nama -> nilai (variabel tahap ini, atau yang menimpa nilai terkunci).
        Urutan prioritas: gain > nilai paksa tahap > config.py. Gain PD ke PDControl, Alpha ke config
        (dibaca cbf_qp.py tiap siklus)."""
        nilai = dict(self.dasar)
        nilai.update(self.paksa)
        nilai.update(gain)
        for nama in nilai:
            if nama in NAMA_GAIN_PD:
                setattr(self.pd, nama, float(nilai[nama]))
            else:
                setattr(config, nama, float(nilai[nama]))

    def terkunci(self):
        """Nilai yang dikunci di tahap sekarang: dict nama -> nilai (PD yang tidak dituning, dan Alpha dari
        barrier yang aktif tetapi bukan milik tahap ini). Yang dipaksa tahap memakai nilai paksa."""
        nama_dikunci = list(NAMA_GAIN_PD)
        if self.pakai_cbf:
            nama_dikunci = nama_dikunci + [self.cbf.nama_alpha[i] for i in self.cbf.alpha_dipakai]
        variabel = config.OPTUNA_TAHAP[self.tahap]["variabel"]
        hasil = {}
        for nama in nama_dikunci:
            if nama not in variabel:
                hasil[nama] = float(self.paksa.get(nama, self.dasar[nama]))
        return hasil

    def skenario(self):
        """Semua yang mempengaruhi skor selain variabel yang dituning. Disimpan di study dan dicek tiap dijalankan."""
        data = config.OPTUNA_PROFIL[self.profil]
        hasil = {"kt_efektif": float(data["kt_efektif"]),
                 "ulangan": int(config.OPTUNA_ULANGAN), "geser_dorong": float(config.OPTUNA_GESER_DORONG),
                 "arah_dorong": list(config.OPTUNA_ARAH_DORONG),
                 "durasi_dorongan": float(config.OPTUNA_DURASI_DORONGAN),
                 "durasi_stabil": float(config.OPTUNA_DURASI_STABIL),
                 "stabil_psi": float(config.OPTUNA_STABIL_PSI), "stabil_dpsi": float(config.OPTUNA_STABIL_DPSI),
                 "stabil_dtheta": float(config.OPTUNA_STABIL_DTHETA),
                 "stabil_lama": float(config.OPTUNA_STABIL_LAMA), "zeta_deg": float(config.OPTUNA_ZETA_DEG),
                 "seed": int(config.OPTUNA_SEED), "penalti_komet": float(config.OPTUNA_PENALTI_KOMET),
                 "penalti_jarak": float(config.OPTUNA_PENALTI_JARAK) if self.penalti_jarak else 0.0,
                 "penalti_gagal": float(config.OPTUNA_PENALTI_GAGAL),
                 "barrier": config.OPTUNA_TAHAP[self.tahap]["barrier"], "skor_s": self.skor_s}
        if self.pakai_cbf:
            hasil["dtheta_max"] = float(config.dtheta_max)            # batas barrier h7, h8 (hanya CBF-QP)
        if self.cari_gaya:
            hasil.update({"gaya_awal": float(data["gaya_dorong"]), "gaya_maks": float(config.OPTUNA_GAYA_MAKS),
                          "gaya_langkah": float(config.OPTUNA_GAYA_LANGKAH),
                          "bobot_gaya": float(config.OPTUNA_BOBOT_GAYA)})
        else:
            hasil["gaya_dorong"] = self.gaya_tetap
        kunci = self.terkunci()
        for nama in kunci:
            hasil["kunci " + nama] = kunci[nama]
        return hasil

    # -------------------------------------------------------------------------
    # Satu pengujian = OPTUNA_ULANGAN ulangan pada satu gaya dorong
    # -------------------------------------------------------------------------
    def uji(self, gain, zeta_deg, gaya, trial=None, rekam=False):
        """Satu pengujian pada gaya dorong `gaya` [N].

        Mengembalikan (skor, status, detail, rekaman, diag):
            status  : STATUS_SELESAI atau STATUS_KOMET
            detail  : penyebab komet, dan/atau fase yang tidak mencapai stabil ("" bila tidak ada)
            rekaman : [dict] satu per ulangan (rekam=True): t [s sejak dorongan pertama ulangan itu], psi [deg],
                      s [m], waktu_dorong, nama_dorong, ulangan
            diag    : dict KOLOM_DIAG
        trial: bila diberikan, skor dilaporkan tiap 1 detik bagian yang dinilai (pruning); melempar
        optuna.TrialPruned bila trial pruned.
        """
        self.atur_zeta(zeta_deg)
        self.atur_gain(gain)
        d = dict.fromkeys(KOLOM_DIAG, 0.0)
        d["gaya_N"] = float(gaya)
        hitungan = {"langkah_dinilai": 0, "tanda": 0, "laporan_ke": 0}
        rekaman = []
        tidak_stabil = []
        status = STATUS_SELESAI
        detail_komet = ""
        for ulangan in range(config.OPTUNA_ULANGAN):
            komet = self._satu_ulangan(ulangan, gaya, d, hitungan, tidak_stabil, rekaman, trial, rekam)
            if komet:
                status = STATUS_KOMET
                detail_komet = komet
                break
        lama_dinilai = hitungan["langkah_dinilai"] * config.DT
        d["tanda_u_per_s"] = hitungan["tanda"] / lama_dinilai if lama_dinilai > 0.0 else 0.0
        d["jumlah_tidak_stabil"] = float(len(tidak_stabil))
        bagian = []
        if detail_komet:
            bagian.append(detail_komet)
        if tidak_stabil:
            bagian.append("tidak stabil: " + ", ".join(tidak_stabil))
        skor = d["ise"] + d["pen_jarak"] + d["pen_gagal"] + d["pen_komet"]
        return skor, status, "; ".join(bagian), rekaman, d

    def _satu_ulangan(self, ulangan, gaya, d, hitungan, tidak_stabil, rekaman, trial, rekam):
        """Satu ulangan skenario (reset, tenang, dorongan berurutan, tunggu stabil). Menambah ke d, hitungan,
        tidak_stabil, dan rekaman. Mengembalikan teks penyebab komet, atau "" bila tidak komet."""
        dt = config.DT
        arah = config.OPTUNA_ARAH_DORONG
        jumlah_dorongan = len(arah)
        nama = nama_dorongan()
        langkah_dorongan = langkah(jarak_dorongan(ulangan))
        langkah_tunda = langkah(ulangan * config.OPTUNA_GESER_DORONG)
        langkah_stabil_maks = langkah(config.OPTUNA_DURASI_STABIL)
        langkah_stabil_lama = langkah(config.OPTUNA_STABIL_LAMA)
        langkah_per_detik = langkah(1.0)
        waktu_total = waktu_dinilai_maks()
        waktu_sebelum = ulangan * waktu_dinilai_ulangan_maks()      # bagian dinilai milik ulangan sebelumnya
        batas_psi = math.radians(config.SAFE_TILT_DEG)
        label = " U%d" % (ulangan + 1) if config.OPTUNA_ULANGAN > 1 else ""

        # Reset: titik nol s = posisi awal ulangan ini. Noise IMU berbeda per ulangan, sama untuk semua trial.
        self.sim.reset()
        self.sim.acak = np.random.default_rng(config.OPTUNA_SEED + ulangan)
        self.sim.gaya_dorong = float(gaya)
        self.pd.set_command(0, MODE_BALANCING)
        data = {"t": [], "psi": [], "s": [], "waktu_dorong": [], "nama_dorong": nama, "ulangan": ulangan}
        if rekam:
            rekaman.append(data)

        def geser_waktu(k_awal):
            """Waktu rekaman dihitung dari langkah 0; ubah menjadi detik sejak dorongan pertama."""
            t0 = k_awal * dt if k_awal is not None else 0.0
            data["t"] = [t - t0 for t in data["t"]]
            data["waktu_dorong"] = [t - t0 for t in data["waktu_dorong"]]

        def catat_puncak(awalan):
            """Puncak |psi|, |dpsi|, |dtheta| pada OPTUNA_STABIL_LAMA detik terakhir fase (terbesar antar ulangan)."""
            for i, nama_kolom in enumerate(KOLOM_STABIL):
                puncak = max((nilai[i] for nilai in terakhir), default=0.0)
                d[awalan + nama_kolom] = max(d[awalan + nama_kolom], puncak)

        fase = -1                    # -1 = tenang awal, 0 .. jumlah_dorongan-1 = dorongan ke-(fase+1)
        k = 0                        # langkah sejak ulangan mulai
        k_fase = 0                   # langkah sejak awal fase (sejak diletakkan / sejak dorongan)
        k_dorong_pertama = None
        awal_selesai = False         # tenang awal sudah dinilai (stabil atau batas waktu); tinggal menunggu tunda
        k_mulai_dorong = 0           # k_fase saat dorongan pertama diberikan (setelah tunda ulangan ini)
        hitung_stabil = 0            # langkah berturut-turut yang memenuhi syarat stabil
        sudah_stabil = False         # syarat stabil (OPTUNA_STABIL_LAMA berturut-turut) tercapai di fase ini
        tanda_lama = 0.0             # tanda u terakhir yang bukan nol (pergantian tanda)
        terakhir = deque(maxlen=langkah_stabil_lama)     # (|psi|, |dpsi|, |dtheta|) OPTUNA_STABIL_LAMA detik terakhir

        while True:
            # Pergantian fase. Stabil hanya dicek di tenang awal dan setelah dorongan terakhir (hanya dicatat).
            if fase == -1:
                if not awal_selesai and (sudah_stabil or k_fase >= langkah_stabil_maks):
                    awal_selesai = True
                    catat_puncak("awal_")
                    if not sudah_stabil:
                        tidak_stabil.append("awal" + label)
                    k_mulai_dorong = k_fase + langkah_tunda
                akhir_fase = awal_selesai and k_fase >= k_mulai_dorong
            elif fase == jumlah_dorongan - 1:
                akhir_fase = sudah_stabil or k_fase >= langkah_stabil_maks
            else:
                akhir_fase = k_fase >= langkah_dorongan
            if akhir_fase:
                if fase == jumlah_dorongan - 1:
                    catat_puncak("akhir_")
                    if not sudah_stabil:
                        tidak_stabil.append("akhir" + label)
                    break
                fase = fase + 1
                k_fase = 0
                hitung_stabil = 0
                sudah_stabil = False
                terakhir.clear()
                if k_dorong_pertama is None:
                    k_dorong_pertama = k
                self.sim.dorong(arah[fase])
                data["waktu_dorong"].append(k * dt)

            x = self.sim.baca_state()
            u_PD = self.pd.compute_PD(x)
            if not self.pakai_cbf:
                u = max(config.u_min, min(config.u_max, u_PD))
                gagal = False
            else:
                u = self.cbf.compute(x, u_PD)
                gagal = not self.cbf.feasible              # status CBF-QP bukan AMAN (GAGAL atau ALPHA_NOL)
            self.sim.step(u)

            # Dinilai dari state yang dibaca kontrol (psi hasil Kalman).
            psi = float(x[2])
            s = model.hitung_s(x)
            t_uji = (k - k_dorong_pertama) * dt if k_dorong_pertama is not None else 0.0   # sejak dorongan pertama
            if rekam:
                data["t"].append(k * dt)
                data["psi"].append(math.degrees(psi))
                data["s"].append(s)
            d["psi_maks_deg"] = max(d["psi_maks_deg"], math.degrees(abs(psi)))
            d["dpsi_maks_rad_s"] = max(d["dpsi_maks_rad_s"], abs(float(x[3])))
            d["s_maks_m"] = max(d["s_maks_m"], abs(s))

            # KOMET: |psi| > SAFE_TILT_DEG, atau |s| > S_KOMET (hanya bila s ikut dinilai)
            if abs(psi) > batas_psi or (self.skor_s and abs(s) > config.S_KOMET):
                sisa = max(waktu_total - waktu_sebelum - t_uji, 0.0)
                d["pen_komet"] = config.OPTUNA_PENALTI_KOMET * (1.0 + sisa / waktu_total)
                posisi = "sebelum dorongan" if fase < 0 else "dorong %s, t %.1f s" % (nama[fase], k_fase * dt)
                if abs(psi) > batas_psi:
                    penyebab = "|psi| > %.0f deg" % config.SAFE_TILT_DEG
                else:
                    penyebab = "|s| > %.1f m" % config.S_KOMET
                geser_waktu(k_dorong_pertama)
                return "%s (%s%s)" % (penyebab, posisi, "," + label if label else "")

            # Syarat stabil: ketiganya terpenuhi terus-menerus selama OPTUNA_STABIL_LAMA.
            terakhir.append((abs(psi), abs(float(x[3])), abs(float(x[1]))))
            if (abs(psi) < config.OPTUNA_STABIL_PSI and abs(float(x[3])) < config.OPTUNA_STABIL_DPSI
                    and abs(float(x[1])) < config.OPTUNA_STABIL_DTHETA):
                hitung_stabil = hitung_stabil + 1
            else:
                hitung_stabil = 0
            if hitung_stabil >= langkah_stabil_lama:
                sudah_stabil = True

            k = k + 1
            k_fase = k_fase + 1
            if fase < 0:
                continue

            # Bagian yang dinilai: sejak dorongan pertama.
            suku = ((psi - config.psi_setpoint) / config.psi_max) ** 2
            if self.skor_s:
                suku = suku + ((s - config.R * config.theta_setpoint) / config.s_max) ** 2
                if self.penalti_jarak and (s > config.s_max or s < config.s_min):
                    d["pen_jarak"] = d["pen_jarak"] + config.OPTUNA_PENALTI_JARAK * dt
            d["ise"] = d["ise"] + suku * dt
            if gagal:
                d["pen_gagal"] = d["pen_gagal"] + config.OPTUNA_PENALTI_GAGAL * dt
                d["lama_gagal_s"] = d["lama_gagal_s"] + dt
            if u != 0.0:
                tanda = math.copysign(1.0, u)
                if tanda_lama != 0.0 and tanda != tanda_lama:
                    hitungan["tanda"] = hitungan["tanda"] + 1
                tanda_lama = tanda
            hitungan["langkah_dinilai"] = hitungan["langkah_dinilai"] + 1

            # Laporan tiap 1 detik bagian yang dinilai (berlanjut antar ulangan) untuk pruning.
            if trial is not None and hitungan["langkah_dinilai"] % langkah_per_detik == 0:
                trial.report(d["ise"] + d["pen_jarak"] + d["pen_gagal"], hitungan["laporan_ke"])
                hitungan["laporan_ke"] = hitungan["laporan_ke"] + 1
                if trial.should_prune():
                    raise optuna.TrialPruned()

        geser_waktu(k_dorong_pertama)
        return ""

    # -------------------------------------------------------------------------
    # Satu trial
    # -------------------------------------------------------------------------
    def cari_gaya_tertinggi(self, gain, zeta_deg):
        """Tahap PD: bisection gaya dorong tertinggi yang lolos satu pengujian (tanpa komet).

        ASUMSI: lolos pada gaya F berarti lolos pada gaya yang lebih kecil (bisection hanya mencoba sebagian kisi).
        Mengembalikan (skor, status, detail). Diagnostik (self.diag) dari pengujian pada gaya lolos tertinggi, atau
        pada gaya terendah bila semua komet (gaya_N = 0).
        """
        kisi = daftar_gaya(self.profil)
        skor, status, detail, _, diag = self.uji(gain, zeta_deg, kisi[0])
        if status == STATUS_KOMET:
            diag["gaya_N"] = 0.0
            self.diag = diag
            return skor, STATUS_KOMET, "%s pada %.1f N" % (detail, kisi[0])

        # kisi[lolos] lolos; kisi[gagal] komet (gagal = len(kisi): di atas OPTUNA_GAYA_MAKS, dianggap komet).
        lolos, gagal = 0, len(kisi)
        hasil_lolos = (skor, detail, diag)
        detail_gagal = ""
        while gagal - lolos > 1:
            tengah = (lolos + gagal) // 2
            skor, status, detail, _, diag = self.uji(gain, zeta_deg, kisi[tengah])
            if status == STATUS_KOMET:
                gagal = tengah
                detail_gagal = "komet pada %.1f N: %s" % (kisi[tengah], detail)
            else:
                lolos = tengah
                hasil_lolos = (skor, detail, diag)

        skor_lolos, detail_lolos, diag_lolos = hasil_lolos
        self.diag = diag_lolos
        bagian = [detail_gagal if detail_gagal else "lolos sampai batas %.1f N" % kisi[-1]]
        if detail_lolos:
            bagian.append(detail_lolos)
        skor_trial = -config.OPTUNA_BOBOT_GAYA * kisi[lolos] + skor_lolos
        return skor_trial, STATUS_SELESAI, "; ".join(bagian)

    def jalankan_trial(self, gain, trial=None):
        """Satu trial di lantai OPTUNA_ZETA_DEG: (skor, status, detail). Diagnostik di self.diag."""
        if self.cari_gaya:
            return self.cari_gaya_tertinggi(gain, config.OPTUNA_ZETA_DEG)
        skor, status, detail, _, diag = self.uji(gain, config.OPTUNA_ZETA_DEG, self.gaya_tetap, trial)
        self.diag = diag
        return skor, status, detail


# =============================================================================
# STUDY
# =============================================================================
def buat_study(tahap, profil):
    os.makedirs(FOLDER_HASIL, exist_ok=True)
    sampler = optuna.samplers.TPESampler(seed=config.OPTUNA_SEED)
    pruner = optuna.pruners.MedianPruner(n_startup_trials=config.OPTUNA_PRUNER_STARTUP,
                                         n_warmup_steps=config.OPTUNA_PRUNER_WARMUP_S)
    return optuna.create_study(study_name=nama_study(tahap, profil), storage=DATABASE, direction="minimize",
                               sampler=sampler, pruner=pruner, load_if_exists=True)


def sama(a, b):
    if isinstance(a, (int, float)) and isinstance(b, (int, float)) and not isinstance(a, bool):
        return abs(a - b) <= 1e-9
    return a == b


def beda_skenario(study, baru):
    """Daftar (nama, nilai di study, nilai sekarang) untuk skenario yang berbeda dari saat study dibuat."""
    lama = study.user_attrs.get("skenario", {})
    hasil = []
    for nama in sorted(set(lama) | set(baru)):
        if nama not in lama or nama not in baru or not sama(lama[nama], baru[nama]):
            hasil.append((nama, lama.get(nama, "-"), baru.get(nama, "-")))
    return hasil


def periksa_study(study, penguji):
    """Catat skenario di atribut study (tampil di Optuna dashboard). Bila study lama dibuat dengan skenario atau
    nilai terkunci yang berbeda dari sekarang, berhenti: trial lama dan baru tidak boleh dicampur."""
    profil = penguji.profil
    baru = penguji.skenario()
    if "skenario" in study.user_attrs:
        beda = beda_skenario(study, baru)
        if len(beda) > 0:
            print("SKENARIO study %s berubah sejak study dibuat:" % study.study_name)
            for nama, nilai_lama, nilai_baru in beda:
                print("    %-24s study: %-14s sekarang: %s" % (nama, nilai_lama, nilai_baru))
            print("Trial lama dan baru tidak boleh dicampur. Kembalikan nilai di config.py, atau hapus study:")
            print("    python3 tuning_optuna.py --profil %s --hapus %s" % (profil, penguji.tahap))
            sys.exit(1)
    elif len(study.trials) > 0:
        # Study lama tanpa catatan skenario (sebelum tuning bertahap): skenarionya tidak diketahui.
        print("Study %s sudah berisi %d trial tanpa catatan skenario (dibuat program lama)."
              % (study.study_name, len(study.trials)))
        print("Trial lama dan baru tidak boleh dicampur. Hapus study dulu:")
        print("    python3 tuning_optuna.py --profil %s --hapus %s" % (profil, penguji.tahap))
        sys.exit(1)
    data = config.OPTUNA_PROFIL[profil]
    study.set_user_attr("skenario", baru)
    study.set_user_attr("profil", profil)
    study.set_user_attr("terverifikasi", bool(data["terverifikasi"]))
    study.set_user_attr("catatan", data["catatan"])
    if not data["terverifikasi"]:
        study.set_user_attr("PERINGATAN", "BELUM TERVERIFIKASI")


def ada_study(tahap, profil):
    if not os.path.exists(os.path.join(FOLDER_HASIL, "atera_optuna.db")):
        return False
    for ringkasan in optuna.get_all_study_summaries(storage=DATABASE):
        if ringkasan.study_name == nama_study(tahap, profil):
            return True
    return False


def trial_terbaik(study):
    """Trial selesai terbaik, atau None bila belum ada yang selesai."""
    try:
        return study.best_trial
    except ValueError:
        return None


def jumlah_per_status(study):
    """(selesai, pruned): jumlah trial COMPLETE dan PRUNED."""
    selesai = 0
    pruned = 0
    for t in study.trials:
        if t.state == optuna.trial.TrialState.COMPLETE:
            selesai = selesai + 1
        elif t.state == optuna.trial.TrialState.PRUNED:
            pruned = pruned + 1
    return selesai, pruned


def cetak_kepala(study, tahap, profil, penguji, definisi, variabel, bawah, atas, kunci, jumlah_trial):
    """Ringkasan pengaturan di awal tuning, lalu judul kolom log."""
    garis = "=" * len(FORMAT_HEADER % JUDUL_KOLOM)
    print(garis)
    print("STUDY %s | TAHAP %s" % (study.study_name, tahap))
    print(teks_label(profil))
    if not config.OPTUNA_PROFIL[profil]["terverifikasi"]:
        print("!! " + config.OPTUNA_PROFIL[profil]["catatan"])
    if penguji.skor_s:
        jenis_skor = "ISE psi dan s" + ("" if penguji.penalti_jarak else ", tanpa penalti jarak")
    else:
        jenis_skor = "ISE psi saja"
    if penguji.pakai_cbf:
        mode = "PD + CBF-QP, barrier " + " ".join("h%d" % n for n in definisi["barrier"])
        jenis_skor = jenis_skor + ", penalti gagal"
    else:
        mode = "PD saja"
    print("Mode: %s | skor: %s, penalti komet" % (mode, jenis_skor))
    print("Dicari (langkah %.4f):" % config.OPTUNA_LANGKAH)
    for nama in variabel:
        print("    %-9s %10.4f .. %10.4f" % (nama, bawah[nama], atas[nama]))
    print("Dikunci (dari config.py, kecuali yang dipaksa):")
    for nama in kunci:
        print("    %-9s %10.4f%s" % (nama, kunci[nama], "   (dipaksa tahap ini)" if nama in penguji.paksa else ""))
    if penguji.cari_gaya:
        kisi = daftar_gaya(profil)
        print("Gaya dorong: cari tertinggi yang lolos, bisection %.1f .. %.1f N (langkah %.1f N, %d kisi), "
              "bobot %.0f per N" % (kisi[0], kisi[-1], config.OPTUNA_GAYA_LANGKAH, len(kisi),
                                    config.OPTUNA_BOBOT_GAYA))
    else:
        print("Gaya dorong: %.1f N (gaya_pd_terbaik %.1f + %.1f N)"
              % (penguji.gaya_tetap, penguji.gaya_tetap - config.OPTUNA_GAYA_TAMBAH_CBF,
                 config.OPTUNA_GAYA_TAMBAH_CBF))
    print("Lantai %.1f deg | %d trial (sudah ada %d)" % (config.OPTUNA_ZETA_DEG, jumlah_trial, len(study.trials)))
    print("Skenario: tenang sampai stabil -> %s (jarak %.1f s) -> sampai stabil"
          % (" -> ".join(nama_dorongan()), config.OPTUNA_DURASI_DORONGAN))
    print("          %d ulangan dengan reset (ulangan ke-k: seed %d + k, dorongan digeser k x %.2f s) | "
          "tunggu stabil maks %.0f s" % (config.OPTUNA_ULANGAN, config.OPTUNA_SEED, config.OPTUNA_GESER_DORONG,
                                         config.OPTUNA_DURASI_STABIL))
    print("Stabil: |psi| < %.4f rad, |dpsi| < %.4f rad/s, |dtheta| < %.4f rad/s selama %.1f s (tanpa penalti)"
          % (config.OPTUNA_STABIL_PSI, config.OPTUNA_STABIL_DPSI, config.OPTUNA_STABIL_DTHETA,
             config.OPTUNA_STABIL_LAMA))
    if config.OPTUNA_STABIL_LAMA >= config.OPTUNA_DURASI_STABIL:
        print("PERHATIAN: OPTUNA_STABIL_LAMA (%.1f s) >= OPTUNA_DURASI_STABIL (%.1f s): robot tidak pernah "
              "sempat dianggap stabil." % (config.OPTUNA_STABIL_LAMA, config.OPTUNA_DURASI_STABIL))
    print("gaya = gaya lolos [N] | tanda/s = pergantian tanda u per detik (bang-bang bila puluhan) | "
          "psimaks = puncak |psi| [deg] | gagal = lama status CBF bukan AMAN [s]")
    print("Ctrl+C untuk berhenti: trial yang sudah selesai tetap tersimpan dan bisa dilanjutkan.")
    print(garis)
    print(FORMAT_HEADER % JUDUL_KOLOM)
    print("-" * len(garis))


def jalankan_tahap(tahap, profil, jumlah_trial):
    definisi = config.OPTUNA_TAHAP[tahap]
    variabel = definisi["variabel"]
    bawah, atas = batas_pencarian(variabel)

    penguji = Penguji(profil, pakai_cbf=definisi["barrier"] is not None)
    penguji.atur_tahap(tahap)
    if penguji.cari_gaya:
        daftar_gaya(profil)                                 # berhenti dengan pesan bila batas gaya salah
    elif penguji.gaya_tetap is None:
        sys.exit("Tahap %s butuh gaya dorong dari hasil tahap PD. Isi OPTUNA_PROFIL['%s']['gaya_pd_terbaik'] di "
                 "config.py (dicetak di akhir tahap PD, atau: python3 tuning_optuna.py --hasil --profil %s)."
                 % (tahap, profil, profil))
    kunci = penguji.terkunci()
    for nama in kunci:
        if nama in config.NAMA_ALPHA and kunci[nama] <= 0.0:
            sys.exit("%s dikunci dari config.py tetapi nilainya %.4f (harus > 0). Isi hasil tahap sebelumnya ke config.py."
                     % (nama, kunci[nama]))
    study = buat_study(tahap, profil)
    periksa_study(study, penguji)
    cetak_kepala(study, tahap, profil, penguji, definisi, variabel, bawah, atas, kunci, jumlah_trial)

    def simpan_atribut(trial, keterangan, t0):
        trial.set_user_attr("keterangan", keterangan)
        trial.set_user_attr("lama_s", time.perf_counter() - t0)
        for nama in KOLOM_DIAG:
            trial.set_user_attr(nama, penguji.diag[nama])

    def objektif(trial):
        params = {}
        for nama in variabel:
            params[nama] = trial.suggest_float(nama, bawah[nama], atas[nama], step=config.OPTUNA_LANGKAH)
        t0 = time.perf_counter()
        try:
            skor, status, detail = penguji.jalankan_trial(params, trial)
        except optuna.TrialPruned:
            simpan_atribut(trial, "pruned", t0)
            raise
        simpan_atribut(trial, ("%s %s" % (status, detail)).strip(), t0)
        return skor

    def cetak_trial(study, trial):
        atribut = trial.user_attrs
        status, _, detail = atribut.get("keterangan", trial.state.name).partition(" ")
        teks_skor = "%11.3f" % trial.value if trial.value is not None else "%11s" % "-"
        terbaik = trial_terbaik(study)
        teks_terbaik = "%.3f (#%d)" % (terbaik.value, terbaik.number) if terbaik is not None else "-"
        teks_gagal = "%6.1f" % atribut.get("lama_gagal_s", 0.0) if penguji.pakai_cbf else "%6s" % "-"
        print(FORMAT_BARIS % (trial.number, status, teks_skor, atribut.get("lama_s", 0.0), teks_terbaik,
                              teks_gaya(atribut.get("gaya_N", 0.0)), atribut.get("tanda_u_per_s", 0.0),
                              atribut.get("psi_maks_deg", 0.0), teks_gagal, detail))

    optuna.logging.set_verbosity(optuna.logging.WARNING)
    try:
        study.optimize(objektif, n_trials=jumlah_trial, callbacks=[cetak_trial])
    except KeyboardInterrupt:
        print("\nDihentikan dengan Ctrl+C.")
    laporkan(tahap, profil, penguji)


# =============================================================================
# LAPORAN: CSV, PNG, nilai untuk config.py
# =============================================================================
def laporkan(tahap, profil, penguji):
    if not ada_study(tahap, profil):
        print("Tahap %s profil %s belum pernah dijalankan." % (tahap, profil))
        return
    study = buat_study(tahap, profil)
    terbaik = trial_terbaik(study)
    if terbaik is None:
        print("Tahap %s profil %s: belum ada trial yang selesai." % (tahap, profil))
        return
    penguji.atur_profil(profil)
    penguji.atur_tahap(tahap)

    # Skenario saat study dibuat dipakai lagi (nilai terkunci dan gaya memakai nilai study, bukan config.py sekarang).
    skenario = study.user_attrs.get("skenario", {})
    for nama, nilai_lama, nilai_baru in beda_skenario(study, penguji.skenario()):
        print("PERHATIAN: %s berbeda dari saat study dibuat (study %s, sekarang %s); uji ulang di bawah memakai "
              "nilai sekarang." % (nama, nilai_lama, nilai_baru))
    gain = {}
    for nama in skenario:
        if nama.startswith("kunci "):
            gain[nama[len("kunci "):]] = skenario[nama]
    gain.update(terbaik.params)

    # Gaya untuk uji ulang: gaya lolos trial terbaik (tahap PD), atau gaya tetap tahap CBFQP_*.
    if penguji.cari_gaya:
        gaya_lolos = float(terbaik.user_attrs.get("gaya_N", 0.0))
        gaya_uji = gaya_lolos if gaya_lolos > 0.0 else daftar_gaya(profil)[0]
    else:
        gaya_uji = skenario.get("gaya_dorong") or penguji.gaya_tetap
    if gaya_uji is None:
        print("Tahap %s profil %s: gaya dorong tidak diketahui (isi gaya_pd_terbaik di config.py)." % (tahap, profil))
        return

    selesai, pruned = jumlah_per_status(study)

    # Gain terbaik diuji lagi (direkam) di lantai optimasi dan lantai uji, pada gaya_uji.
    hasil_uji = []
    for zeta_deg in (config.OPTUNA_ZETA_DEG, config.OPTUNA_ZETA_UJI_DEG):
        skor, status, detail, rekaman, diag = penguji.uji(gain, zeta_deg, gaya_uji, rekam=True)
        hasil_uji.append((zeta_deg, skor, ("%s %s" % (status, detail)).strip(), rekaman, diag))

    print("\n================ HASIL TAHAP %s | %s ================" % (tahap, study.study_name))
    print(teks_label(profil))
    print("trial: %d total, %d selesai, %d pruned" % (len(study.trials), selesai, pruned))
    print("trial terbaik: %d, skor %.4f (%s)" % (terbaik.number, terbaik.value,
                                                terbaik.user_attrs.get("keterangan", "-")))
    print("uji ulang pada gaya %.1f N:" % gaya_uji)
    for zeta_deg, skor, keterangan, _, diag in hasil_uji:
        print("  lantai %4.1f deg: skor %10.4f | psimaks %5.1f deg | tanda/s %5.1f | gagal %4.1f s | %s"
              % (zeta_deg, skor, diag["psi_maks_deg"], diag["tanda_u_per_s"], diag["lama_gagal_s"], keterangan))
    print("\nSalin ke config.py:")
    for nama in terbaik.params:
        print("%s = %.4f" % (nama, terbaik.params[nama]))
    if penguji.cari_gaya:
        print("OPTUNA_PROFIL['%s']['gaya_pd_terbaik'] = %.1f      # tahap CBFQP_* memakai %.1f N"
              % (profil, gaya_lolos, gaya_lolos + config.OPTUNA_GAYA_TAMBAH_CBF))
    for nama in penguji.paksa:
        print("# selama tahap ini %s dipaksa %.4f (bukan nilai config.py)" % (nama, penguji.paksa[nama]))
    print("=================================================\n")

    awalan = os.path.join(FOLDER_HASIL, datetime.now().strftime("%Y%m%d_%H%M%S") + "_" + nama_file(tahap, profil))
    simpan_csv(awalan + ".csv", study, profil)
    simpan_png(awalan + ".png", tahap, profil, study, terbaik.params, hasil_uji, gaya_uji, penguji.cari_gaya)


def simpan_csv(path, study, profil):
    nama_param = []
    for t in study.trials:
        for nama in t.params:
            if nama not in nama_param:
                nama_param.append(nama)
    with open(path, "w", newline="") as f:
        penulis = csv.writer(f)
        penulis.writerow(["trial", "status", "skor", "keterangan", "lama_s", "profil", "terverifikasi"]
                         + nama_param + KOLOM_DIAG)
        for t in study.trials:
            skor = "%.6f" % t.value if t.value is not None else ""
            baris = [t.number, t.state.name, skor, t.user_attrs.get("keterangan", ""),
                     "%.1f" % t.user_attrs.get("lama_s", 0.0), profil,
                     "YA" if config.OPTUNA_PROFIL[profil]["terverifikasi"] else "BELUM"]
            for nama in nama_param:
                baris.append("%.4f" % t.params[nama] if nama in t.params else "")
            for nama in KOLOM_DIAG:
                baris.append("%.4f" % t.user_attrs[nama] if nama in t.user_attrs else "")
            penulis.writerow(baris)
    print("Tersimpan:", path)


def simpan_png(path, tahap, profil, study, gain, hasil_uji, gaya_uji, cari_gaya):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib tidak terpasang -> PNG tidak dibuat")
        return

    fig, ax = plt.subplots(3, 1, figsize=(11, 11))

    # 1. Riwayat tiap trial: gaya lolos (tahap PD) atau skor (tahap CBFQP_*), dan nilai trial terbaik sejauh ini.
    nomor_selesai = []
    nilai_selesai = []
    nomor_terbaik = []
    nilai_terbaik = []
    skor_terbaik = math.inf
    nilai_trial_terbaik = None
    nomor_pruned = []
    for t in study.trials:
        if t.state == optuna.trial.TrialState.COMPLETE:
            nilai = t.user_attrs.get("gaya_N", 0.0) if cari_gaya else t.value
            nomor_selesai.append(t.number)
            nilai_selesai.append(nilai)
            if t.value < skor_terbaik:
                skor_terbaik = t.value
                nilai_trial_terbaik = nilai
            nomor_terbaik.append(t.number)
            nilai_terbaik.append(nilai_trial_terbaik)
        elif t.state == optuna.trial.TrialState.PRUNED:
            nomor_pruned.append(t.number)
    ax[0].plot(nomor_selesai, nilai_selesai, "o", ms=3, color="tab:blue", label="trial selesai")
    ax[0].step(nomor_terbaik, nilai_terbaik, where="post", color="red", label="terbaik sejauh ini")
    for i in range(len(nomor_pruned)):
        ax[0].axvline(nomor_pruned[i], color="0.85", lw=0.8, zorder=0, label="pruned" if i == 0 else None)
    if cari_gaya:
        ax[0].set_ylabel("gaya lolos tertinggi [N]")
    else:
        ax[0].set_ylabel("skor (log)")
        if len(nilai_selesai) > 0 and min(nilai_selesai) > 0.0:
            ax[0].set_yscale("log")
    ax[0].set_xlabel("trial")
    ax[0].set_title("Riwayat optimasi tahap %s | %s" % (tahap, teks_label(profil)), fontsize=10,
                    color="black" if config.OPTUNA_PROFIL[profil]["terverifikasi"] else "red")
    ax[0].grid(True)
    ax[0].legend(fontsize=8)

    # 2 dan 3. Respon gain terbaik: warna = lantai, garis putus-putus = ulangan berikutnya.
    warna = ["tab:blue", "tab:orange"]
    gaya_garis = ["-", "--", ":", "-."]
    for i in range(len(hasil_uji)):
        zeta_deg, _, _, rekaman, _ = hasil_uji[i]
        for data in rekaman:
            label = "zeta %.1f deg, U%d" % (zeta_deg, data["ulangan"] + 1)
            ls = gaya_garis[data["ulangan"] % len(gaya_garis)]
            ax[1].plot(data["t"], data["psi"], color=warna[i], ls=ls, lw=1, label=label)
            ax[2].plot(data["t"], data["s"], color=warna[i], ls=ls, lw=1, label=label)
        if i == 0 and len(rekaman) > 0:
            # Penanda awal tiap dorongan ulangan pertama (ulangan berikutnya digeser OPTUNA_GESER_DORONG per ulangan).
            data = rekaman[0]
            for waktu, nama in zip(data["waktu_dorong"], data["nama_dorong"]):
                for a in (ax[1], ax[2]):
                    a.axvline(waktu, color="black", lw=0.8, ls=":")
                ax[1].annotate(nama, (waktu, 1.0), xycoords=("data", "axes fraction"), fontsize=8,
                               xytext=(3, -10), textcoords="offset points")
    batas_psi = math.degrees(config.psi_max)
    ax[1].axhline(batas_psi, color="red", lw=2.2)
    ax[1].axhline(-batas_psi, color="red", lw=2.2)
    ax[2].axhline(config.s_max, color="red", lw=2.2)
    ax[2].axhline(config.s_min, color="red", lw=2.2)
    ax[1].set_ylabel("psi Kalman [deg]")
    ax[2].set_ylabel("s [m]")
    for a in (ax[1], ax[2]):
        a.set_xlabel("t sejak dorongan pertama tiap ulangan [s]")
        a.grid(True)
        a.legend(fontsize=8)
    teks_gain = ", ".join("%s=%.4f" % (nama, gain[nama]) for nama in gain)
    ax[1].set_title("Respon gain terbaik (dorong %.1f N, %d ulangan): %s"
                    % (gaya_uji, config.OPTUNA_ULANGAN, teks_gain), fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)
    print("Tersimpan:", path)


def hapus_tahap(tahap, profil):
    if not ada_study(tahap, profil):
        print("Tahap %s profil %s belum ada di database." % (tahap, profil))
        return
    jawaban = input("Hapus SEMUA trial study %s dari database? ketik YA: " % nama_study(tahap, profil))
    if jawaban.strip() == "YA":
        optuna.delete_study(study_name=nama_study(tahap, profil), storage=DATABASE)
        print("Study %s dihapus." % nama_study(tahap, profil))
    else:
        print("Batal.")


def main():
    daftar_profil = list(config.OPTUNA_PROFIL.keys())
    daftar_tahap = list(config.OPTUNA_TAHAP.keys())
    parser = argparse.ArgumentParser(description="Tuning bertahap gain PD dan Alpha CBF-QP ATERA dengan Optuna")
    parser.add_argument("--profil", choices=daftar_profil,
                        help="profil torsi roda (config.OPTUNA_PROFIL); wajib untuk --tahap dan --hapus")
    parser.add_argument("--tahap", choices=daftar_tahap,
                        help="tahap yang dijalankan (config.OPTUNA_TAHAP); nilai tahap lain dibaca dari config.py")
    parser.add_argument("--trial", type=int, default=None, help="jumlah trial (default dari config.OPTUNA_TAHAP)")
    parser.add_argument("--hasil", action="store_true",
                        help="cetak hasil terbaik, buat ulang CSV + PNG (tanpa --profil: semua profil)")
    parser.add_argument("--hapus", choices=daftar_tahap, help="hapus study satu tahap dari satu profil")
    args = parser.parse_args()

    if args.hapus:
        if args.profil is None:
            parser.error("--hapus butuh --profil")
        hapus_tahap(args.hapus, args.profil)
    elif args.hasil:
        profil_dilaporkan = daftar_profil if args.profil is None else [args.profil]
        ada = [(t, p) for p in profil_dilaporkan for t in daftar_tahap if ada_study(t, p)]
        if len(ada) == 0:
            print("Belum ada study untuk profil yang diminta.")
            return
        perlu_cbf = any(config.OPTUNA_TAHAP[t]["barrier"] is not None for t, _ in ada)
        penguji = Penguji(ada[0][1], pakai_cbf=perlu_cbf)
        for tahap, profil in ada:
            laporkan(tahap, profil, penguji)
    elif args.tahap:
        if args.profil is None:
            parser.error("--tahap butuh --profil (%s)" % " / ".join(daftar_profil))
        jumlah = args.trial if args.trial is not None else config.OPTUNA_TAHAP[args.tahap]["trial"]
        jalankan_tahap(args.tahap, args.profil, jumlah)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()