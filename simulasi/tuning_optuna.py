"""tuning_optuna.py - mencari gain PD dan Alpha CBF-QP ATERA dengan Optuna (simulasi PyBullet), SATU TAHAP (SEMUA).

Semua variabel config.OPTUNA_VARIABEL (Kp_psi, Kd_psi, Kd_theta, Alpha_1 .. Alpha_3, Kp_theta) dicari BERSAMAAN dalam
satu study ATERA_SEMUA_<PROFIL> di database OPTUNA/atera_optuna.db. Profil (config.OPTUNA_PROFIL): TERUKUR
(0.238 N m/A), DATASHEET (0.75 N m/A, BELUM TERVERIFIKASI). Study tidak boleh dilanjutkan bila skenario, nilai
terkunci, atau fisika simulasi (FISIKA_SKENARIO) berubah. Study baru dimulai dari nilai config.py
(OPTUNA_MULAI_DARI_CONFIG).

Satu TRIAL = blok config.OPTUNA_BLOK BERURUTAN, tiap blok S1 -> S2 -> S3 (config.py bagian 12):
    1. PD_DIAM        mode DIAM (Kp_theta mati), PD saja             Kp_psi, Kd_psi, Kd_theta
    2. CBF_DIAM       mode DIAM, PD + CBF-QP h1 .. h4                + Alpha_1 .. Alpha_3   (syarat: >= PD_DIAM)
    3. PD_BALANCING   mode BALANCING (Kp_theta aktif), PD saja       + Kp_theta
    4. CBF_BALANCING  mode BALANCING, PD + CBF-QP                    (syarat: >= PD_BALANCING)
    S1  dorongan di lantai datar: bisection gaya dorong tertinggi yang lolos satu pengujian -> gaya_lolos
    S2  bidang miring tanpa dorongan: bisection kemiringan tertinggi yang berhasil -> miring_lolos
    S3  gaya_lolos di miring_lolos -> lolos / gagal
    skor blok  = -OPTUNA_BOBOT_GAYA x gaya_lolos + skor pengujian S1 - OPTUNA_BOBOT_MIRING x miring_lolos
                 - (OPTUNA_BOBOT_S3 bila S3 lolos);   skor trial = jumlah skor blok (makin kecil makin baik)
    SYARAT blok PD + CBF-QP (tidak lebih buruk dari blok PD mode yang sama DI TRIAL ITU): S1 langsung pada gaya lolos
        PD, S2 langsung pada kemiringan lolos PD, S3 harus lolos bila PD lolos S3.
    Gagal syarat, atau komet pada gaya terendah S1 -> trial berhenti (blok berikutnya tidak dijalankan),
        skor = skor yang sudah dihitung + OPTUNA_PENALTI_SYARAT.
    PEMANGKASAN AMAN (OPTUNA_PANGKAS_AMAN): setelah S1 tiap blok, bila skor terbaik yang masih mungkin tidak lebih
        baik dari trial terbaik, trial berhenti (status dipangkas, skor = skor terbaik yang masih mungkin).
    MULAI DARI TERBAIK (OPTUNA_MULAI_DARI_TERBAIK): S1 dan S2 blok PD dimulai dari hasil trial terbaik.
STAGNAN: tuning berhenti sendiri bila skor terbaik tidak membaik selama OPTUNA_STAGNANT trial selesai.

Log terminal tiap trial: trial | status | skor | waktu | terbaik | gaya | miring | ch/s | psimaks | gagal | detail
    gaya / miring / ch/s / psimaks / gagal = blok terakhir yang dijalankan, ch/s = chattering per detik,
    psimaks = puncak |psi| [deg], gagal = lama status CBF bukan AMAN [s], status = selesai | komet | dipangkas

Jalankan (di folder simulasi):
    python3 tuning_optuna.py --profil DATASHEET                  jumlah trial dari config.OPTUNA_TRIAL
    python3 tuning_optuna.py --profil DATASHEET --trial 50       MENAMBAH 50 trial ke study yang ada
    python3 tuning_optuna.py --profil DATASHEET --stagnant 500   batas stagnan 500 trial (0 = mati)
    python3 tuning_optuna.py --hasil [--profil DATASHEET]        cetak hasil terbaik, buat ulang CSV + PNG
    python3 tuning_optuna.py --profil DATASHEET --hapus          hapus study (mulai dari nol)
Hasil (folder OPTUNA): atera_optuna.db, YYYYMMDD_HHmmSS_OPTUNA_SEMUA_<PROFIL>.csv / .png.
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
from pd_control import MODE_BALANCING, MODE_DIAM, PDControl

FOLDER_INI = os.path.dirname(os.path.abspath(__file__))
FOLDER_HASIL = os.path.join(FOLDER_INI, "OPTUNA")
DATABASE = "sqlite:///" + os.path.join(FOLDER_HASIL, "atera_optuna.db")
NAMA_GAIN_PD = ["Kp_psi", "Kd_psi", "Kp_theta", "Kd_theta"]
KONTROL_PD = "PD"
KONTROL_CBF = "PD+CBF"
MODE_BLOK = {"DIAM": MODE_DIAM, "BALANCING": MODE_BALANCING}
TAHAP = "SEMUA"
# Nilai config.py yang menentukan fisika simulasi dan KOMET: ikut dicatat di skenario study, jadi study yang dibuat
# dengan simulasi berbeda tidak tercampur. Nama yang tidak ada dilewati.
FISIKA_SKENARIO = ["CONTROL_HZ", "SIM_JEDA_TORSI", "SIM_BATAS_KECEPATAN", "MOTOR_NO_LOAD_SPEED_RPM",
                   "MOTOR_STALL_CURRENT_A", "MOTOR_KT", "MAX_CURRENT_A", "SIM_MODEL_IMU", "SIM_GYRO_NOISE_DEG_S",
                   "SIM_ACC_NOISE_DEG", "LAMA_DORONG", "DORONG_SISI", "DORONG_GESER_SAMPING", "SAFE_TILT_DEG",
                   "S_KOMET", "psi_max", "s_max", "CBF_TAHU_ZETA", "CBF_QP_SOLVER", "DOB_AKTIF", "DOB_TAU_S",
                   "DOB_BATAS", "CBF_MARGIN_DEG"]
TEKS_ARAH = {1: "depan", -1: "belakang"}
STATUS_SELESAI = "selesai"
STATUS_KOMET = "komet"
STATUS_DIPANGKAS = "dipangkas"

# Diagnostik pengujian S1 blok terakhir yang dijalankan (atribut trial dan kolom CSV).
# skor pengujian = ise + pen_gagal + pen_komet + pen_s
# pen_s               = (hanya mode BALANCING) OPTUNA_BOBOT_S_MAKS x s_maks_m + OPTUNA_BOBOT_OVERSHOOT_S x overshoot_s
# gaya_N              = gaya lolos tertinggi S1 (0 bila gaya pertama yang diuji pun komet)
# chattering_per_s    = u berbalik dari >= +a ke <= -a (atau sebaliknya) per detik, a = OPTUNA_AMBANG_CHATTERING x u_max
# jumlah_tidak_stabil = jumlah fase (tenang awal, setelah dorongan terakhir) yang tidak mencapai stabil
# overshoot_*_persen  = overshoot terbesar psi / s dari semua dorongan [%] (syarat lolos)
# awal_* / akhir_*    = puncak |psi|, |dpsi|, |dtheta| pada OPTUNA_STABIL_LAMA detik terakhir fase awal / terakhir
KOLOM_STABIL = ["psi_rad", "dpsi_rad_s", "dtheta_rad_s"]
KOLOM_DIAG = (["gaya_N", "chattering_per_s", "psi_maks_deg", "dpsi_maks_rad_s", "s_maks_m", "lama_gagal_s",
               "jumlah_tidak_stabil"]
              + ["awal_" + nama for nama in KOLOM_STABIL] + ["akhir_" + nama for nama in KOLOM_STABIL]
              + ["ise", "pen_gagal", "pen_komet", "pen_s", "overshoot_psi_persen", "overshoot_s_persen"])
# Hasil S1 S2 S3 tiap blok (atribut trial dan kolom CSV), <blok> = nama blok huruf kecil (pd_diam, cbf_diam, ...).
# miring_deg = kemiringan lolos blok terakhir yang menjalankan S2 (log dan PNG). syarat_lolos: 1 = lolos, 0 = gagal.
KOLOM_BLOK = ["gaya_%s_N", "miring_%s_deg", "s3_%s_lolos", "miring_s_maks_%s_m", "skor_%s"]
KOLOM_HASIL = ([kolom % nama.lower() for nama, _, _ in config.OPTUNA_BLOK for kolom in KOLOM_BLOK]
               + ["miring_deg", "syarat_lolos", "dipangkas"])

# Baris log terminal: header dan isi memakai lebar kolom yang sama.
JUDUL_KOLOM = ("trial", "status", "skor", "waktu", "terbaik", "gaya", "miring", "ch/s", "psimaks", "gagal", "detail")
FORMAT_HEADER = "%5s | %-9s | %11s | %7s | %-18s | %6s | %6s | %7s | %7s | %6s | %s"
FORMAT_BARIS = "%5d | %-9s | %11s | %5.1f s | %-18s | %6s | %6s | %7.1f | %7.1f | %6s | %s"


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


def daftar_jarak(ulangan):
    """Jarak [s] antar dorongan berurutan pada ulangan ke-`ulangan` (0 = pertama): acak merata OPTUNA_JARAK_ACAK_MIN ..
    OPTUNA_JARAK_ACAK_MAKS (kelipatan DT), seed OPTUNA_SEED_JARAK + ulangan, jadi sama untuk semua trial dan gaya."""
    jumlah = max(0, len(config.OPTUNA_ARAH_DORONG) - 1)
    acak = np.random.default_rng(int(config.OPTUNA_SEED_JARAK) + ulangan)
    hasil = []
    for _ in range(jumlah):
        jarak = acak.uniform(float(config.OPTUNA_JARAK_ACAK_MIN), float(config.OPTUNA_JARAK_ACAK_MAKS))
        hasil.append(langkah(jarak) * config.DT)
    return hasil


def waktu_dinilai_ulangan_maks():
    """Lama maksimum bagian yang dinilai dalam satu ulangan [s] (dorongan pertama sampai batas tunggu terakhir)."""
    return (len(config.OPTUNA_ARAH_DORONG) - 1) * float(config.OPTUNA_JARAK_ACAK_MAKS) + config.OPTUNA_DURASI_STABIL


def waktu_dinilai_maks():
    """Lama maksimum bagian yang dinilai dalam satu pengujian (semua ulangan) [s]."""
    return config.OPTUNA_ULANGAN * waktu_dinilai_ulangan_maks()


def daftar_gaya(profil):
    """Kisi gaya dorong S1 [N]: gaya_dorong profil .. OPTUNA_GAYA_MAKS, berjarak OPTUNA_GAYA_LANGKAH."""
    awal = float(config.OPTUNA_PROFIL[profil]["gaya_dorong"])
    maks = float(config.OPTUNA_GAYA_MAKS)
    jarak = float(config.OPTUNA_GAYA_LANGKAH)
    if jarak <= 0.0 or maks < awal:
        sys.exit("Periksa config.py: perlu OPTUNA_GAYA_LANGKAH > 0 dan OPTUNA_GAYA_MAKS (%.2f) >= gaya_dorong "
                 "profil %s (%.2f)." % (maks, profil, awal))
    jumlah = int(math.floor((maks - awal) / jarak + 1e-9))
    return [round(awal + i * jarak, 6) for i in range(jumlah + 1)]


def daftar_miring():
    """Kisi kemiringan S2 [deg]: OPTUNA_MIRING_LANGKAH, 2 x langkah, ... sampai OPTUNA_MIRING_MAKS."""
    jarak = float(config.OPTUNA_MIRING_LANGKAH)
    maks = float(config.OPTUNA_MIRING_MAKS)
    if jarak <= 0.0 or maks < jarak:
        sys.exit("Periksa config.py: perlu OPTUNA_MIRING_LANGKAH > 0 dan OPTUNA_MIRING_MAKS (%.2f) >= "
                 "OPTUNA_MIRING_LANGKAH (%.2f)." % (maks, jarak))
    if float(config.OPTUNA_MIRING_LAMA) <= 0.0:
        sys.exit("Periksa config.py: perlu OPTUNA_MIRING_LAMA > 0.")
    jumlah = int(math.floor(maks / jarak + 1e-9))
    return [round(i * jarak, 6) for i in range(1, jumlah + 1)]


def nama_study(profil):
    return "ATERA_%s_%s" % (TAHAP, profil)                # ATERA_SEMUA_DATASHEET


def nama_file(profil):
    return "OPTUNA_%s_%s" % (TAHAP, profil)


def semua_variabel():
    return NAMA_GAIN_PD + config.NAMA_ALPHA


def tabel_batas(profil):
    """(dict nama -> (bawah, atas), nama tabel) dari config.OPTUNA_BATAS[profil]."""
    if profil not in config.OPTUNA_BATAS:
        sys.exit("config.OPTUNA_BATAS belum punya batas untuk profil %s." % profil)
    return config.OPTUNA_BATAS[profil], "config.OPTUNA_BATAS['%s']" % profil


def batas_pencarian(variabel, profil):
    """(bawah, atas): dict nama -> batas untuk profil ini. Berhenti dengan pesan bila batasnya salah."""
    tabel, sumber = tabel_batas(profil)
    bawah = {}
    atas = {}
    for nama in variabel:
        if nama not in tabel:
            sys.exit("%s belum ada di %s. Isi dengan (batas bawah, batas atas)." % (nama, sumber))
        bawah[nama] = float(tabel[nama][0])
        atas[nama] = float(tabel[nama][1])
        if atas[nama] <= bawah[nama]:
            sys.exit("Batas atas %s (%.4f) harus lebih besar dari batas bawah (%.4f). Periksa %s."
                     % (nama, atas[nama], bawah[nama], sumber))
        if nama in config.NAMA_ALPHA and bawah[nama] <= 0.0:
            sys.exit("Batas bawah %s harus > 0: Alpha = 0 mematikan CBF-QP. Periksa %s." % (nama, sumber))
    return bawah, atas


def teks_label(profil):
    """Label profil untuk terminal, judul PNG, dan atribut study."""
    data = config.OPTUNA_PROFIL[profil]
    teks = "profil %s (torsi roda %.3f N m/A)" % (profil, data["kt_efektif"])
    if not data["terverifikasi"]:
        teks = teks + " - BELUM TERVERIFIKASI"
    return teks


def teks_gaya(gaya):
    return "%.1f" % gaya if gaya is not None and gaya > 0.0 else "-"


def teks_miring(miring):
    """Kemiringan lolos S2 untuk log, "-" bila 0 atau tidak ada (None)."""
    return "%.1f" % miring if miring is not None and miring > 0.0 else "-"


def awal_dari_terbaik(study, blok):
    """Titik mulai S1 dan S2 blok PD (OPTUNA_MULAI_DARI_TERBAIK) dari trial terbaik sejauh ini: dict nama atribut
    -> nilai (> 0 saja). Kosong bila belum ada trial selesai atau OPTUNA_MULAI_DARI_TERBAIK = False."""
    if not config.OPTUNA_MULAI_DARI_TERBAIK:
        return {}
    terbaik = trial_terbaik(study)
    if terbaik is None:
        return {}
    awal = {}
    for nama, _, kontrol in blok:
        if kontrol != KONTROL_PD:
            continue
        for kolom in ("gaya_%s_N", "miring_%s_deg"):
            kunci = kolom % nama.lower()
            nilai = float(terbaik.user_attrs.get(kunci, 0.0))
            if nilai > 0.0:
                awal[kunci] = nilai
    return awal


def overshoot_persen(nilai, acuan, puncak_min, n_rata):
    """Overshoot respon SATU dorongan [%] (syarat lolos, lihat config.py blok SYARAT LOLOS):
        e(t)      = nilai(t) - acuan           (acuan = rata-rata sinyal sesaat sebelum dorongan)
        e diratakan rata-rata bergerak n_rata sampel (meredam noise sensor)
        puncak    = e dengan |e| terbesar
        overshoot = simpangan terbesar ke arah BERLAWANAN dengan puncak, setelah puncak, / |puncak| x 100
    0 % bila respon kembali tanpa melewati titik awal, atau puncak lebih kecil dari puncak_min."""
    e = np.asarray(nilai, dtype=float) - float(acuan)
    if n_rata > 1 and len(e) >= n_rata:
        e = np.convolve(e, np.ones(n_rata) / n_rata, mode="valid")
    if len(e) == 0:
        return 0.0
    i = int(np.argmax(np.abs(e)))
    puncak = float(e[i])
    if abs(puncak) < puncak_min:
        return 0.0
    berlawanan = -math.copysign(1.0, puncak) * e[i:]
    return 100.0 * max(0.0, float(np.max(berlawanan))) / abs(puncak)


def setelah_koreksi(nilai, koreksi):
    """O-b: bagian `nilai` sesudah sampel terakhir dengan koreksi True (CBF-QP mengoreksi atau psi di luar psi_max).
    Tanpa koreksi: seluruh `nilai`."""
    for j in range(len(koreksi) - 1, -1, -1):
        if koreksi[j]:
            return nilai[j + 1:]
    return nilai


# =============================================================================
# PENGUJI: satu simulasi headless dipakai ulang untuk semua trial
# =============================================================================
class Penguji:
    def __init__(self, profil):
        # PyBullet hanya boleh satu dunia per program, jadi Penguji cukup dibuat SATU kali.
        self.sim = simulasi.Simulasi(gui=False)
        self.pd = PDControl()
        self.blok = baca_blok()              # [(nama, mode gerak, kontrol)] urutan satu trial
        self.variabel = list(config.OPTUNA_VARIABEL)
        self.cbf = None
        if any(kontrol == KONTROL_CBF for _, _, kontrol in self.blok):
            print("Menyiapkan CBF-QP (solver %s) ..." % config.CBF_QP_SOLVER)
            from cbf_qp import CBFQP
            # CBF_TAHU_ZETA = True: model CBF-QP dan DOB memakai config.zeta (diubah atur_zeta saat bidang miring).
            self.cbf = CBFQP(zeta=None if config.CBF_TAHU_ZETA else 0.0)
        # Nilai config.py saat program mulai: dasar semua variabel yang tidak dituning (nilai terkunci).
        self.dasar = {nama: float(getattr(config, nama)) for nama in semua_variabel()}
        self.mode = MODE_DIAM                # mode gerak robot blok yang sedang diuji
        self.pakai_cbf = False               # kontrol yang sedang diuji memakai CBF-QP
        self.awal_cari = {}                  # titik mulai S1 / S2 blok PD (awal_dari_terbaik)
        self.diag = dict.fromkeys(KOLOM_DIAG, 0.0)   # diagnostik S1 blok terakhir (lihat KOLOM_DIAG)
        self.hasil = {}                      # KOLOM_HASIL trial terakhir
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

    def atur_mode(self, mode):
        """Mode gerak blok yang diuji (PDControl dan constraint CBF-QP mode itu)."""
        self.mode = mode
        self.pd.atur_mode(mode)
        if self.cbf is not None:
            self.cbf.mode_cbf(mode)

    def atur_zeta(self, zeta_deg):
        if self.zeta_deg == zeta_deg:
            return
        self.zeta_deg = zeta_deg
        config.zeta_DEG = float(zeta_deg)
        config.zeta = math.radians(zeta_deg)           # dipakai model.py dan cbf_qp.py
        self.sim.ubah_zeta(config.zeta)

    def atur_gain(self, gain):
        """gain: dict nama -> nilai (variabel yang dituning, menimpa config.py). Gain PD ke PDControl, Alpha ke config
        (dibaca cbf_qp.py tiap siklus)."""
        nilai = dict(self.dasar)
        nilai.update(gain)
        for nama in nilai:
            if nama in NAMA_GAIN_PD:
                setattr(self.pd, nama, float(nilai[nama]))
            else:
                setattr(config, nama, float(nilai[nama]))

    def dipakai(self):
        """Nama gain yang bekerja (Kp_theta hanya bila ada blok BALANCING, Alpha hanya bila ada blok PD + CBF-QP)."""
        nama = ["Kp_psi", "Kd_psi", "Kd_theta"]
        if any(mode == MODE_BALANCING for _, mode, _ in self.blok):
            nama = nama + ["Kp_theta"]
        if self.cbf is not None:
            nama = nama + [self.cbf.nama_alpha[i] for i in self.cbf.alpha_dipakai]
        return nama

    def terkunci(self):
        """Nilai yang dipakai tetapi tidak dituning: dict nama -> nilai config.py."""
        return {nama: float(self.dasar[nama]) for nama in self.dipakai() if nama not in self.variabel}

    def skenario(self):
        """Semua yang mempengaruhi skor selain variabel yang dituning. Disimpan di study dan dicek tiap dijalankan."""
        data = config.OPTUNA_PROFIL[self.profil]
        hasil = {"kt_efektif": float(data["kt_efektif"]), "blok": [list(b) for b in self.blok],
                 "ulangan": int(config.OPTUNA_ULANGAN), "geser_dorong": float(config.OPTUNA_GESER_DORONG),
                 "arah_dorong": list(config.OPTUNA_ARAH_DORONG),
                 "jarak_dorongan": [daftar_jarak(k) for k in range(int(config.OPTUNA_ULANGAN))],
                 "durasi_stabil": float(config.OPTUNA_DURASI_STABIL),
                 "stabil_psi": float(config.OPTUNA_STABIL_PSI), "stabil_dpsi": float(config.OPTUNA_STABIL_DPSI),
                 "stabil_dtheta": float(config.OPTUNA_STABIL_DTHETA),
                 "stabil_lama": float(config.OPTUNA_STABIL_LAMA), "zeta_deg": float(config.OPTUNA_ZETA_DEG),
                 "seed": int(config.OPTUNA_SEED), "penalti_komet": float(config.OPTUNA_PENALTI_KOMET),
                 "penalti_gagal": float(config.OPTUNA_PENALTI_GAGAL),
                 "batas_chattering": float(config.OPTUNA_BATAS_CHATTERING),
                 "ambang_chattering": float(config.OPTUNA_AMBANG_CHATTERING),
                 "penalti_syarat": float(config.OPTUNA_PENALTI_SYARAT),
                 "miring_langkah": float(config.OPTUNA_MIRING_LANGKAH),
                 "miring_maks": float(config.OPTUNA_MIRING_MAKS), "miring_lama": float(config.OPTUNA_MIRING_LAMA),
                 "bobot_miring": float(config.OPTUNA_BOBOT_MIRING), "bobot_s3": float(config.OPTUNA_BOBOT_S3),
                 "gaya_awal": float(data["gaya_dorong"]), "gaya_maks": float(config.OPTUNA_GAYA_MAKS),
                 "gaya_langkah": float(config.OPTUNA_GAYA_LANGKAH), "bobot_gaya": float(config.OPTUNA_BOBOT_GAYA),
                 "bobot_s_maks": float(config.OPTUNA_BOBOT_S_MAKS),
                 "bobot_overshoot_s": float(config.OPTUNA_BOBOT_OVERSHOOT_S)}
        for nama in ("OPTUNA_SYARAT_OVERSHOOT_PSI", "OPTUNA_SYARAT_OVERSHOOT_S", "OPTUNA_OVERSHOOT_ACUAN_S",
                     "OPTUNA_OVERSHOOT_RATA_S", "OPTUNA_OVERSHOOT_PUNCAK_PSI_DEG", "OPTUNA_OVERSHOOT_PUNCAK_S_M"):
            hasil[nama] = float(getattr(config, nama))
        for nama in FISIKA_SKENARIO:
            if hasattr(config, nama):
                nilai = getattr(config, nama)
                hasil["fisika " + nama] = nilai if isinstance(nilai, (bool, str)) else float(nilai)
        if self.cbf is not None:
            hasil["dtheta_max"] = float(config.dtheta_max)            # batas barrier h3, h4
        kunci = self.terkunci()
        for nama in kunci:
            hasil["kunci " + nama] = kunci[nama]
        return hasil

    # -------------------------------------------------------------------------
    # Satu pengujian = OPTUNA_ULANGAN ulangan pada satu gaya dorong
    # -------------------------------------------------------------------------
    def uji(self, gain, zeta_deg, gaya, rekam=False):
        """Satu pengujian pada gaya dorong `gaya` [N], kontrol sekarang (self.pakai_cbf), mode gerak blok.

        Mengembalikan (skor, status, detail, rekaman, diag):
            status  : STATUS_SELESAI atau STATUS_KOMET
            detail  : penyebab komet, dan/atau fase yang tidak mencapai stabil ("" bila tidak ada)
            rekaman : [dict] satu per ulangan (rekam=True): t [s sejak dorongan pertama], psi [deg], s [m],
                      waktu_dorong, nama_dorong, ulangan
            diag    : dict KOLOM_DIAG
        """
        self.atur_zeta(zeta_deg)
        self.atur_gain(gain)
        d = dict.fromkeys(KOLOM_DIAG, 0.0)
        d["gaya_N"] = float(gaya)
        hitungan = {"langkah_dinilai": 0, "balik": 0}
        rekaman = []
        tidak_stabil = []
        status = STATUS_SELESAI
        detail_komet = ""
        for ulangan in range(config.OPTUNA_ULANGAN):
            komet = self._satu_ulangan(ulangan, gaya, d, hitungan, tidak_stabil, rekaman, rekam)
            if komet:
                status = STATUS_KOMET
                detail_komet = komet
                break
        lama_dinilai = hitungan["langkah_dinilai"] * config.DT
        d["chattering_per_s"] = hitungan["balik"] / lama_dinilai if lama_dinilai > 0.0 else 0.0
        d["jumlah_tidak_stabil"] = float(len(tidak_stabil))
        batas_ch = float(config.OPTUNA_BATAS_CHATTERING)
        if status == STATUS_SELESAI and batas_ch > 0.0 and d["chattering_per_s"] > batas_ch:
            # Torsi bergetar bolak-balik (chattering) tidak boleh dianggap lolos: dihitung KOMET di akhir pengujian.
            status = STATUS_KOMET
            detail_komet = "chattering %.2f/s > %g/s" % (d["chattering_per_s"], batas_ch)
            d["pen_komet"] = d["pen_komet"] + config.OPTUNA_PENALTI_KOMET
        if status == STATUS_SELESAI:
            # Syarat tidak berayun (config.py blok SYARAT LOLOS): overshoot di atas batas dihitung KOMET.
            berayun = []
            dinilai = (("psi", "overshoot_psi_persen"), ("s", "overshoot_s_persen"))
            for nama, kunci in dinilai if self.mode == MODE_BALANCING else dinilai[:1]:
                batas = float(getattr(config, "OPTUNA_SYARAT_OVERSHOOT_" + nama.upper()))
                if batas > 0.0 and d[kunci] > batas:
                    berayun.append("overshoot %s %.1f %% > %g %%" % (nama, d[kunci], batas))
            if berayun:
                status = STATUS_KOMET
                detail_komet = "berayun: " + ", ".join(berayun)
                d["pen_komet"] = d["pen_komet"] + config.OPTUNA_PENALTI_KOMET
        bagian = []
        if detail_komet:
            bagian.append(detail_komet)
        if tidak_stabil:
            bagian.append("tidak stabil: " + ", ".join(tidak_stabil))
        if self.mode == MODE_BALANCING:
            # Kp_theta: jarak sekecil mungkin dan tanpa overshoot s (critically damped)
            d["pen_s"] = (config.OPTUNA_BOBOT_S_MAKS * d["s_maks_m"]
                          + config.OPTUNA_BOBOT_OVERSHOOT_S * d["overshoot_s_persen"])
        skor = d["ise"] + d["pen_gagal"] + d["pen_komet"] + d["pen_s"]
        return skor, status, "; ".join(bagian), rekaman, d

    def _satu_ulangan(self, ulangan, gaya, d, hitungan, tidak_stabil, rekaman, rekam):
        """Satu ulangan skenario (reset, tenang, dorongan berurutan, tunggu stabil). Menambah ke d, hitungan,
        tidak_stabil, dan rekaman. Mengembalikan teks penyebab komet, atau "" bila tidak komet."""
        dt = config.DT
        arah = config.OPTUNA_ARAH_DORONG
        jumlah_dorongan = len(arah)
        nama = nama_dorongan()
        langkah_jarak = [langkah(jarak) for jarak in daftar_jarak(ulangan)]   # jarak setelah dorongan ke-i
        langkah_tunda = langkah(ulangan * config.OPTUNA_GESER_DORONG)
        langkah_stabil_maks = langkah(config.OPTUNA_DURASI_STABIL)
        langkah_stabil_lama = langkah(config.OPTUNA_STABIL_LAMA)
        waktu_total = waktu_dinilai_maks()
        waktu_sebelum = ulangan * waktu_dinilai_ulangan_maks()      # bagian dinilai milik ulangan sebelumnya
        batas_psi = math.radians(config.SAFE_TILT_DEG)
        cek_s = self.mode == MODE_BALANCING                         # KOMET s hanya mode BALANCING
        label = " U%d" % (ulangan + 1) if config.OPTUNA_ULANGAN > 1 else ""

        # Reset: titik nol s = posisi awal ulangan ini. Noise IMU berbeda per ulangan, sama untuk semua trial.
        self.sim.reset()
        self.sim.acak = np.random.default_rng(config.OPTUNA_SEED + ulangan)
        self.sim.gaya_dorong = float(gaya)
        if self.cbf is not None:
            self.cbf.reset_dob()                  # robot fisik: DOB dikosongkan saat MULAI
        self.pd.set_command(0)
        self.pd.atur_mode(self.mode)
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
        ambang_u = config.OPTUNA_AMBANG_CHATTERING * config.u_max   # |u| di atas ini = torsi "besar" (chattering)
        tanda_besar = 0.0            # tanda u besar terakhir (+1 / -1, 0 = belum ada)
        terakhir = deque(maxlen=langkah_stabil_lama)     # (|psi|, |dpsi|, |dtheta|) OPTUNA_STABIL_LAMA detik terakhir
        # Overshoot (syarat lolos): respon tiap dorongan dicatat sampai dorongan berikutnya / akhir ulangan.
        n_rata = max(1, langkah(config.OPTUNA_OVERSHOOT_RATA_S))
        sebelum = deque(maxlen=max(1, langkah(config.OPTUNA_OVERSHOOT_ACUAN_S)))   # (psi [deg], s [m]) terakhir
        respon = {"psi": [], "s": [], "koreksi": [], "acuan_psi": 0.0, "acuan_s": 0.0}

        def tutup_respon():
            """Overshoot respon dorongan sekarang -> d (yang terbesar dari semua dorongan dan ulangan). PD + CBF-QP:
            overshoot psi hanya sesudah CBF-QP selesai mengoreksi (O-b, config.py blok SYARAT LOLOS)."""
            if not respon["psi"]:
                return
            psi_dinilai = respon["psi"]
            if self.pakai_cbf:
                psi_dinilai = setelah_koreksi(psi_dinilai, respon["koreksi"])
            nilai = overshoot_persen(psi_dinilai, respon["acuan_psi"], config.OPTUNA_OVERSHOOT_PUNCAK_PSI_DEG, n_rata)
            d["overshoot_psi_persen"] = max(d["overshoot_psi_persen"], nilai)
            nilai = overshoot_persen(respon["s"], respon["acuan_s"], config.OPTUNA_OVERSHOOT_PUNCAK_S_M, n_rata)
            d["overshoot_s_persen"] = max(d["overshoot_s_persen"], nilai)

        def buka_respon():
            """Dorongan baru: acuan = rata-rata OPTUNA_OVERSHOOT_ACUAN_S detik sebelum dorongan."""
            respon["psi"] = []
            respon["s"] = []
            respon["koreksi"] = []
            respon["acuan_psi"] = float(np.mean([v[0] for v in sebelum])) if sebelum else 0.0
            respon["acuan_s"] = float(np.mean([v[1] for v in sebelum])) if sebelum else 0.0

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
                akhir_fase = k_fase >= langkah_jarak[fase]
            if akhir_fase:
                tutup_respon()                       # respon dorongan sebelumnya selesai (tidak ada bila fase awal)
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
                buka_respon()
                self.sim.dorong(arah[fase])
                data["waktu_dorong"].append(k * dt)

            x = self.sim.baca_state()
            # urutan sama dengan robot fisik: DOB (awal siklus) -> PD -> CBF-QP -> DOB (catat u)
            if self.pakai_cbf:
                self.cbf.dob_perbarui(x, config.DT)
            u_PD = self.pd.compute(x)
            if not self.pakai_cbf:
                u = max(config.u_min, min(config.u_max, u_PD))
                gagal = False
                koreksi = False
            else:
                u = self.cbf.compute(x, u_PD)
                gagal = not self.cbf.feasible              # status CBF-QP bukan AMAN (SLACK, GAGAL, atau ALPHA_NOL)
                koreksi = self.cbf.active or gagal
                self.cbf.dob_catat(x, u)
            self.sim.step(u)

            # Dinilai dari state yang dibaca kontrol (psi hasil Kalman).
            psi = float(x[2])
            s = model.hitung_s(x)
            t_uji = (k - k_dorong_pertama) * dt if k_dorong_pertama is not None else 0.0   # sejak dorongan pertama
            sebelum.append((math.degrees(psi), s))
            if fase >= 0:
                respon["psi"].append(math.degrees(psi))
                respon["s"].append(s)
                respon["koreksi"].append(koreksi or abs(psi) > config.psi_max)
            if rekam:
                data["t"].append(k * dt)
                data["psi"].append(math.degrees(psi))
                data["s"].append(s)
            d["psi_maks_deg"] = max(d["psi_maks_deg"], math.degrees(abs(psi)))
            d["dpsi_maks_rad_s"] = max(d["dpsi_maks_rad_s"], abs(float(x[3])))
            d["s_maks_m"] = max(d["s_maks_m"], abs(s))

            # KOMET: |psi| > SAFE_TILT_DEG, atau |s| > S_KOMET (hanya mode BALANCING)
            if abs(psi) > batas_psi or (cek_s and abs(s) > config.S_KOMET):
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
            suku = suku + ((s - config.R * config.theta_setpoint) / config.s_max) ** 2
            d["ise"] = d["ise"] + suku * dt
            if gagal:
                d["pen_gagal"] = d["pen_gagal"] + config.OPTUNA_PENALTI_GAGAL * dt
                d["lama_gagal_s"] = d["lama_gagal_s"] + dt
            if abs(u) >= ambang_u:
                tanda = math.copysign(1.0, u)
                if tanda_besar != 0.0 and tanda != tanda_besar:
                    hitungan["balik"] = hitungan["balik"] + 1
                tanda_besar = tanda
            hitungan["langkah_dinilai"] = hitungan["langkah_dinilai"] + 1

        geser_waktu(k_dorong_pertama)
        return ""

    # -------------------------------------------------------------------------
    # S1, S2, S3
    # -------------------------------------------------------------------------
    def cari_gaya_tertinggi(self, gain, zeta_deg, mulai=None, syarat=False):
        """S1: bisection gaya dorong tertinggi yang lolos satu pengujian (tanpa komet), kontrol sekarang.

        ASUMSI: lolos pada gaya F berarti lolos pada gaya yang lebih kecil.
        mulai [N] (opsional, blok PD = gaya lolos trial terbaik): setelah gaya terendah lolos, pencarian dimulai dari
        kisi terdekat gaya itu, langkah naik / turun dilipatgandakan (1, 2, 4, ... kisi) sampai batas lolos / komet
        ditemukan, lalu bisection. Hasil sama dengan bisection biasa, pengujian lebih sedikit.
        syarat True (PD + CBF-QP, `mulai` = gaya acuan): gaya terendah tidak diuji, pengujian pertama langsung pada
        `mulai`. Komet -> berhenti (gaya_N = 0); lolos -> naik sampai komet, lalu bisection.
        Mengembalikan (skor, status, detail). Diagnostik (self.diag) dari pengujian pada gaya lolos tertinggi, atau
        pada gaya pertama yang diuji bila komet (gaya_N = 0).
        """
        kisi = daftar_gaya(self.profil)
        detail_gagal = ""
        jarak = 1
        if syarat:
            i = min(range(len(kisi)), key=lambda k: abs(kisi[k] - float(mulai)))
            skor, status, detail, _, diag = self.uji(gain, zeta_deg, kisi[i])
            if status == STATUS_KOMET:
                diag["gaya_N"] = 0.0
                self.diag = diag
                return skor, STATUS_KOMET, "gagal syarat: komet pada %.1f N: %s" % (
                    kisi[i], detail.split("; tidak stabil:")[0])
            lolos, gagal = i, len(kisi)
            hasil_lolos = (skor, detail, diag)
        else:
            # Gaya terendah SELALU diuji lebih dulu (juga dengan `mulai`): robot harus lolos dari gaya rendah.
            skor, status, detail, _, diag = self.uji(gain, zeta_deg, kisi[0])
            if status == STATUS_KOMET:
                diag["gaya_N"] = 0.0
                self.diag = diag
                return skor, STATUS_KOMET, "%s pada %.1f N" % (detail, kisi[0])
            # kisi[lolos] lolos; kisi[gagal] komet (gagal = len(kisi): di atas OPTUNA_GAYA_MAKS, dianggap komet).
            lolos, gagal = 0, len(kisi)
            hasil_lolos = (skor, detail, diag)
        if mulai is not None and not syarat:
            i = min(range(len(kisi)), key=lambda k: abs(kisi[k] - float(mulai)))
            if i > 0:
                skor, status, detail, _, diag = self.uji(gain, zeta_deg, kisi[i])
                if status == STATUS_KOMET:
                    gagal = i
                    detail_gagal = "komet pada %.1f N: %s" % (kisi[i], detail.split("; tidak stabil:")[0])
                    while gagal - lolos > 1:                 # turun sampai lolos (kisi[0] sudah lolos)
                        j = max(lolos + 1, gagal - jarak)
                        skor, status, detail, _, diag = self.uji(gain, zeta_deg, kisi[j])
                        if status != STATUS_KOMET:
                            lolos = j
                            hasil_lolos = (skor, detail, diag)
                            break
                        gagal = j
                        detail_gagal = "komet pada %.1f N: %s" % (kisi[j], detail.split("; tidak stabil:")[0])
                        jarak = 2 * jarak
                else:
                    lolos = i
                    hasil_lolos = (skor, detail, diag)
        if mulai is not None and gagal == len(kisi):
            while lolos < len(kisi) - 1:                     # naik sampai komet (atau batas atas kisi)
                j = min(len(kisi) - 1, lolos + jarak)
                skor, status, detail, _, diag = self.uji(gain, zeta_deg, kisi[j])
                if status == STATUS_KOMET:
                    gagal = j
                    detail_gagal = "komet pada %.1f N: %s" % (kisi[j], detail.split("; tidak stabil:")[0])
                    break
                lolos = j
                hasil_lolos = (skor, detail, diag)
                jarak = 2 * jarak
        while gagal - lolos > 1:
            tengah = (lolos + gagal) // 2
            skor, status, detail, _, diag = self.uji(gain, zeta_deg, kisi[tengah])
            if status == STATUS_KOMET:
                gagal = tengah
                # hanya penyebab komet (fase tidak stabil dilaporkan dari pengujian yang lolos)
                detail_gagal = "komet pada %.1f N: %s" % (kisi[tengah], detail.split("; tidak stabil:")[0])
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

    def uji_miring(self, gain, zeta_deg):
        """Satu pengujian S2 di bidang miring zeta_deg [deg], TANPA dorongan, kontrol sekarang: OPTUNA_ULANGAN ulangan
        (reset, seed noise OPTUNA_SEED + k), robot diletakkan tegak lalu dibiarkan OPTUNA_MIRING_LAMA detik.
        BERHASIL bila sepanjang waktu |psi| <= SAFE_TILT_DEG, (hanya mode BALANCING) |s| <= s_max, dan ch/s seluruh
        pengujian <= OPTUNA_BATAS_CHATTERING (0 = tidak dicek).
        Mengembalikan (berhasil, detail, info): detail = penyebab gagal ("" bila berhasil), info = dict s_maks_m,
        psi_maks_deg, chattering_per_s."""
        self.atur_zeta(zeta_deg)
        self.atur_gain(gain)
        dt = config.DT
        batas_psi = math.radians(config.SAFE_TILT_DEG)
        cek_s = self.mode == MODE_BALANCING
        ambang_u = config.OPTUNA_AMBANG_CHATTERING * config.u_max
        jumlah_langkah = langkah(config.OPTUNA_MIRING_LAMA)
        info = {"s_maks_m": 0.0, "psi_maks_deg": 0.0, "chattering_per_s": 0.0}
        balik = 0
        langkah_total = 0
        gagal = ""
        for ulangan in range(config.OPTUNA_ULANGAN):
            label = ", U%d" % (ulangan + 1) if config.OPTUNA_ULANGAN > 1 else ""
            self.sim.reset()
            self.sim.acak = np.random.default_rng(config.OPTUNA_SEED + ulangan)
            self.sim.gaya_dorong = 0.0
            self.pd.set_command(0)
            self.pd.atur_mode(self.mode)
            if self.cbf is not None:
                self.cbf.reset_dob()                  # robot fisik: DOB dikosongkan saat MULAI
            tanda_besar = 0.0
            for k in range(jumlah_langkah):
                x = self.sim.baca_state()
                # urutan sama dengan robot fisik: DOB (awal siklus) -> PD -> CBF-QP -> DOB (catat u)
                if self.pakai_cbf:
                    self.cbf.dob_perbarui(x, config.DT)
                u_PD = self.pd.compute(x)
                if not self.pakai_cbf:
                    u = max(config.u_min, min(config.u_max, u_PD))
                else:
                    u = self.cbf.compute(x, u_PD)
                    self.cbf.dob_catat(x, u)
                self.sim.step(u)
                psi = float(x[2])
                s = model.hitung_s(x)
                info["psi_maks_deg"] = max(info["psi_maks_deg"], math.degrees(abs(psi)))
                info["s_maks_m"] = max(info["s_maks_m"], abs(s))
                if abs(u) >= ambang_u:
                    tanda = math.copysign(1.0, u)
                    if tanda_besar != 0.0 and tanda != tanda_besar:
                        balik = balik + 1
                    tanda_besar = tanda
                langkah_total = langkah_total + 1
                if abs(psi) > batas_psi:
                    gagal = "|psi| > %.0f deg (t %.1f s%s)" % (config.SAFE_TILT_DEG, k * dt, label)
                elif cek_s and abs(s) > config.s_max:
                    gagal = "|s| > s_max %.2f m (t %.1f s%s)" % (config.s_max, k * dt, label)
                if gagal:
                    break
            if gagal:
                break
        lama = langkah_total * dt
        info["chattering_per_s"] = balik / lama if lama > 0.0 else 0.0
        batas_ch = float(config.OPTUNA_BATAS_CHATTERING)
        if not gagal and batas_ch > 0.0 and info["chattering_per_s"] > batas_ch:
            gagal = "chattering %.2f/s > %g/s" % (info["chattering_per_s"], batas_ch)
        return not gagal, gagal, info

    def cari_miring_tertinggi(self, gain, mulai=None, syarat=False):
        """S2: bisection kemiringan tertinggi (kisi daftar_miring) yang berhasil uji_miring, kontrol sekarang.
        ASUMSI: berhasil pada kemiringan z berarti berhasil pada kemiringan yang lebih kecil.
        mulai [deg] (opsional, > 0): pengujian pertama pada kisi terdekat, langkah naik / turun dilipatgandakan
        (1, 2, 4, ... kisi) sampai batas berhasil / gagal ditemukan, lalu bisection.
        syarat True (PD + CBF-QP, `mulai` = kemiringan acuan): gagal pada `mulai` -> berhenti (miring_lolos = 0).
        Mengembalikan (miring_lolos [deg], 0 bila gagal; detail; info uji_miring yang berhasil pada miring_lolos, atau
        pengujian terakhir yang gagal)."""
        kisi = daftar_miring()
        detail_gagal = ""
        if mulai is None or mulai <= 0.0:
            berhasil, detail, info = self.uji_miring(gain, kisi[0])
            if not berhasil:
                return 0.0, "gagal pada %.1f deg: %s" % (kisi[0], detail), info
            # kisi[lolos] berhasil; kisi[gagal] gagal (gagal = len(kisi): di atas OPTUNA_MIRING_MAKS, dianggap gagal).
            lolos, gagal = 0, len(kisi)
            info_lolos = info
        else:
            i = min(range(len(kisi)), key=lambda k: abs(kisi[k] - float(mulai)))
            jarak = 1
            berhasil, detail, info = self.uji_miring(gain, kisi[i])
            if berhasil:
                lolos, gagal = i, len(kisi)
                info_lolos = info
                while lolos < len(kisi) - 1:                 # naik sampai gagal (atau batas atas kisi)
                    j = min(len(kisi) - 1, lolos + jarak)
                    berhasil, detail, info = self.uji_miring(gain, kisi[j])
                    if not berhasil:
                        gagal = j
                        detail_gagal = "gagal pada %.1f deg: %s" % (kisi[j], detail)
                        break
                    lolos = j
                    info_lolos = info
                    jarak = 2 * jarak
            elif syarat:
                return 0.0, "gagal syarat: gagal pada %.1f deg: %s" % (kisi[i], detail), info
            else:
                # lolos = -1: di bawah kisi pertama (0 deg), dianggap berhasil
                lolos, gagal = -1, i
                info_lolos = info                            # info kisi[gagal] sampai ada yang berhasil
                detail_gagal = "gagal pada %.1f deg: %s" % (kisi[i], detail)
                while gagal - lolos > 1:                     # turun sampai berhasil (atau kisi pertama gagal)
                    j = max(lolos + 1, gagal - jarak)
                    berhasil, detail, info = self.uji_miring(gain, kisi[j])
                    if berhasil:
                        lolos = j
                        info_lolos = info
                        break
                    gagal = j
                    info_lolos = info
                    detail_gagal = "gagal pada %.1f deg: %s" % (kisi[j], detail)
                    jarak = 2 * jarak
                if lolos < 0:                                # kisi pertama pun gagal (sama dengan tanpa `mulai`)
                    return 0.0, detail_gagal, info_lolos
        while gagal - lolos > 1:
            tengah = (lolos + gagal) // 2
            berhasil, detail, info = self.uji_miring(gain, kisi[tengah])
            if berhasil:
                lolos = tengah
                info_lolos = info
            else:
                gagal = tengah
                detail_gagal = "gagal pada %.1f deg: %s" % (kisi[tengah], detail)
        teks = detail_gagal if detail_gagal else "lolos sampai batas %.1f deg" % kisi[-1]
        return kisi[lolos], teks, info_lolos

    def uji_s3(self, gain, gaya, miring):
        """S3: gaya [N] di kemiringan miring [deg], kontrol sekarang. Mengembalikan (lolos, teks).
        Tidak dijalankan (= gagal) bila gaya atau miring = 0."""
        if gaya <= 0.0 or miring <= 0.0:
            return False, "S3 tidak dijalankan (gaya / kemiringan = 0)"
        _, status_s3, detail_s3, _, _ = self.uji(gain, miring, gaya)
        s3_lolos = status_s3 == STATUS_SELESAI
        teks_s3 = "lolos" if s3_lolos else "komet " + detail_s3.split("; tidak stabil:")[0]
        return s3_lolos, "S3 %.1f N di %.1f deg: %s" % (gaya, miring, teks_s3)

    def dalam_mode(self, pakai_cbf, fungsi, *args, **kwargs):
        """fungsi(*args, **kwargs) dengan kontrol tertentu (pakai_cbf False = PD saja), lalu kontrol dikembalikan."""
        if pakai_cbf and self.cbf is None:
            raise RuntimeError("Kontrol PD + CBF-QP butuh Penguji dengan CBF-QP.")
        sebelumnya = self.pakai_cbf
        self.pakai_cbf = bool(pakai_cbf)
        try:
            return fungsi(*args, **kwargs)
        finally:
            self.pakai_cbf = sebelumnya

    # -------------------------------------------------------------------------
    # Satu trial
    # -------------------------------------------------------------------------
    def acuan_untuk(self, i, hasil):
        """Acuan syarat blok ke-i bila PD + CBF-QP: hasil blok PD terakhir dengan mode yang sama di trial ini.
        None untuk blok PD, atau bila belum ada blok PD dengan mode itu."""
        _, mode, kontrol = self.blok[i]
        if kontrol != KONTROL_CBF:
            return None
        acuan = None
        for nama, mode_lain, kontrol_lain in self.blok[:i]:
            if mode_lain == mode and kontrol_lain == KONTROL_PD and nama in hasil:
                acuan = nama
        return None if acuan is None else dict(hasil[acuan], nama=acuan)

    def jalankan_trial(self, gain, skor_terbaik=None):
        """Satu trial: blok config.OPTUNA_BLOK berurutan, tiap blok S1 -> S2 -> S3. Lihat docstring modul.
        skor_terbaik: skor trial terbaik sejauh ini (pemangkasan aman), None = belum ada.
        Mengembalikan (skor, status, detail). self.diag = diagnostik S1 blok terakhir yang dijalankan,
        self.hasil = kolom KOLOM_HASIL."""
        zeta = config.OPTUNA_ZETA_DEG
        awal = self.awal_cari
        hasil = {}                               # nama blok -> dict skor (S1), gaya, miring, s3, s_maks, detail
        self.hasil = {}
        bonus_maks = config.OPTUNA_BOBOT_MIRING * daftar_miring()[-1] + config.OPTUNA_BOBOT_S3
        # skor blok terbaik yang mungkin: lolos sampai gaya maks (skor pengujian >= 0), S2 sampai miring maks, S3 lolos
        blok_maks = config.OPTUNA_BOBOT_GAYA * daftar_gaya(self.profil)[-1] + bonus_maks

        def bonus(r):
            return config.OPTUNA_BOBOT_MIRING * r.get("miring", 0.0) + (config.OPTUNA_BOBOT_S3 if r.get("s3") else 0.0)

        def jumlah():
            return sum(r["skor"] - bonus(r) for r in hasil.values())

        def selesai(status, tambahan=0.0, teks=""):
            for nama, r in hasil.items():
                b = nama.lower()
                self.hasil["gaya_%s_N" % b] = r["gaya"]
                self.hasil["skor_%s" % b] = r["skor"] - bonus(r)
                if "miring" in r:
                    self.hasil["miring_%s_deg" % b] = r["miring"]
                    self.hasil["miring_s_maks_%s_m" % b] = r["s_maks"]
                    self.hasil["miring_deg"] = r["miring"]
                if "s3" in r:
                    self.hasil["s3_%s_lolos" % b] = 1.0 if r["s3"] else 0.0
            self.hasil["syarat_lolos"] = 0.0 if tambahan > 0.0 else 1.0
            detail = " || ".join("%s %s" % (nama, r["detail"]) for nama, r in hasil.items())
            return jumlah() + tambahan, status, detail + (" || " + teks if teks else "")

        for i, (nama, mode, kontrol) in enumerate(self.blok):
            self.atur_mode(mode)
            cbf = kontrol == KONTROL_CBF
            acuan = self.acuan_untuk(i, hasil)
            b = nama.lower()

            # S1
            if acuan is not None:
                mulai, syarat = acuan["gaya"], True
            else:
                mulai, syarat = awal.get("gaya_%s_N" % b), False
            skor, _, detail = self.dalam_mode(cbf, self.cari_gaya_tertinggi, gain, zeta, mulai, syarat)
            r = hasil[nama] = {"skor": skor, "gaya": float(self.diag["gaya_N"]), "detail": "S1 " + detail}
            if r["gaya"] <= 0.0:
                return selesai(STATUS_KOMET, config.OPTUNA_PENALTI_SYARAT,
                               "%s gagal S1, blok berikutnya tidak dijalankan" % nama)

            # Pemangkasan aman: skor terbaik yang masih mungkin (blok ini S2 sampai miring maks dan S3 lolos, blok
            # berikutnya skor terbaik yang mungkin)
            if config.OPTUNA_PANGKAS_AMAN and skor_terbaik is not None:
                batas = jumlah() - bonus_maks - (len(self.blok) - i - 1) * blok_maks
                if batas >= skor_terbaik:
                    _, _, detail = selesai(STATUS_DIPANGKAS)
                    self.hasil.pop("syarat_lolos", None)
                    self.hasil["dipangkas"] = 1.0
                    return batas, STATUS_DIPANGKAS, detail + " || terbaik yang masih mungkin %.1f >= %.1f" % (
                        batas, skor_terbaik)

            # S2
            syarat = acuan is not None and acuan.get("miring", 0.0) > 0.0
            if acuan is not None:
                mulai = acuan["miring"] if syarat else None
            else:
                mulai = awal.get("miring_%s_deg" % b)
            miring, detail, info = self.dalam_mode(cbf, self.cari_miring_tertinggi, gain, mulai, syarat)
            r.update({"miring": miring, "s_maks": info["s_maks_m"]})
            r["detail"] += "; S2 %s deg: %s" % (teks_miring(miring), detail)
            if syarat and miring <= 0.0:
                return selesai(STATUS_KOMET, config.OPTUNA_PENALTI_SYARAT, "%s gagal syarat S2" % nama)

            # S3
            s3, teks = self.dalam_mode(cbf, self.uji_s3, gain, r["gaya"], r["miring"])
            r["s3"] = s3
            r["detail"] += "; " + teks
            if acuan is not None and acuan.get("s3") and not s3:
                # tidak lebih buruk dari acuan bila lolos di gaya dan kemiringan acuan
                lolos, teks = self.dalam_mode(True, self.uji_s3, gain, acuan["gaya"], acuan["miring"])
                r["detail"] += "; di acuan %s: %s" % (acuan["nama"], teks)
                if not lolos:
                    return selesai(STATUS_KOMET, config.OPTUNA_PENALTI_SYARAT, "%s gagal syarat S3" % nama)
        return selesai(STATUS_SELESAI)


# =============================================================================
# STUDY
# =============================================================================
def baca_blok():
    """config.OPTUNA_BLOK diperiksa: [(nama, mode gerak pd_control, kontrol)]."""
    blok = []
    for nama, mode, kontrol in config.OPTUNA_BLOK:
        if mode not in MODE_BLOK or kontrol not in (KONTROL_PD, KONTROL_CBF):
            sys.exit("Periksa config.OPTUNA_BLOK %s: mode harus %s, kontrol harus \"PD\" atau \"PD+CBF\"."
                     % (nama, " / ".join(MODE_BLOK)))
        blok.append((nama, MODE_BLOK[mode], kontrol))
    if len(set(nama for nama, _, _ in blok)) != len(blok):
        sys.exit("Periksa config.OPTUNA_BLOK: nama blok harus berbeda.")
    return blok


def buat_study(profil):
    os.makedirs(FOLDER_HASIL, exist_ok=True)
    # multivariate=True: TPE mempelajari semua variabel BERSAMAAN (pasangan yang saling terkait seperti Kp_psi-Kd_psi,
    # Alpha_1-Alpha_2). Optuna menampilkan ExperimentalWarning, itu normal.
    sampler = optuna.samplers.TPESampler(seed=config.OPTUNA_SEED, multivariate=True)
    return optuna.create_study(study_name=nama_study(profil), storage=DATABASE, direction="minimize",
                               sampler=sampler, pruner=optuna.pruners.NopPruner(), load_if_exists=True)


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
    """Catat skenario di atribut study. Bila study lama dibuat dengan skenario atau nilai terkunci yang berbeda dari
    sekarang, berhenti: trial lama dan baru tidak boleh dicampur."""
    profil = penguji.profil
    baru = penguji.skenario()
    if "skenario" in study.user_attrs:
        beda = beda_skenario(study, baru)
        if len(beda) > 0:
            print("SKENARIO study %s berubah sejak study dibuat:" % study.study_name)
            for nama, nilai_lama, nilai_baru in beda:
                print("    %-24s study: %-14s sekarang: %s" % (nama, nilai_lama, nilai_baru))
            print("Trial lama dan baru tidak boleh dicampur. Kembalikan nilai di config.py, atau hapus study:")
            print("    python3 tuning_optuna.py --profil %s --hapus" % profil)
            sys.exit(1)
    elif len(study.trials) > 0:
        print("Study %s sudah berisi %d trial tanpa catatan skenario. Hapus study dulu:"
              % (study.study_name, len(study.trials)))
        print("    python3 tuning_optuna.py --profil %s --hapus" % profil)
        sys.exit(1)
    data = config.OPTUNA_PROFIL[profil]
    study.set_user_attr("skenario", baru)
    study.set_user_attr("profil", profil)
    study.set_user_attr("terverifikasi", bool(data["terverifikasi"]))
    study.set_user_attr("catatan", data["catatan"])
    if not data["terverifikasi"]:
        study.set_user_attr("PERINGATAN", "BELUM TERVERIFIKASI")


def ada_study(profil):
    if not os.path.exists(os.path.join(FOLDER_HASIL, "atera_optuna.db")):
        return False
    for ringkasan in optuna.get_all_study_summaries(storage=DATABASE):
        if ringkasan.study_name == nama_study(profil):
            return True
    return False


def trial_terbaik(study):
    """Trial selesai terbaik, atau None bila belum ada yang selesai."""
    try:
        return study.best_trial
    except ValueError:
        return None


class PenjagaStagnan:
    """Auto-exit (early stopping seluruh study) bila skor terbaik tidak membaik selama `stagnant` trial SELESAI
    berturut-turut. Dipakai sebagai callback study.optimize(); memanggil study.stop() saat stagnan.

    Membaik = skor < acuan - ambang * max(1, |acuan|), acuan = skor terbaik saat perbaikan terakhir. Trial gagal tidak
    dihitung. stagnant <= 0 = mati. Riwayat study (trial selesai di database) dihitung lebih dulu."""

    def __init__(self, study, stagnant, ambang):
        self.stagnant = int(stagnant)
        self.ambang = float(ambang)
        self.acuan = math.inf
        self.tanpa_perbaikan = 0
        self.berhenti = False
        for t in sorted(study.trials, key=lambda t: t.number):
            if t.state == optuna.trial.TrialState.COMPLETE and t.value is not None:
                self._catat(float(t.value))

    @property
    def aktif(self):
        return self.stagnant > 0

    def _catat(self, nilai):
        if not math.isfinite(nilai):
            self.tanpa_perbaikan += 1
        elif not math.isfinite(self.acuan) or nilai < self.acuan - self.ambang * max(1.0, abs(self.acuan)):
            self.acuan = nilai
            self.tanpa_perbaikan = 0
        else:
            self.tanpa_perbaikan += 1

    def stagnan(self):
        return self.aktif and self.tanpa_perbaikan >= self.stagnant

    def teks(self):
        if not self.aktif:
            return "auto-exit stagnan MATI (stagnant 0)"
        return ("auto-exit bila %d trial selesai berturut-turut tanpa perbaikan > %g %% (sekarang %d / %d)"
                % (self.stagnant, 100.0 * self.ambang, self.tanpa_perbaikan, self.stagnant))

    def __call__(self, study, trial):
        if trial.state != optuna.trial.TrialState.COMPLETE or trial.value is None:
            return
        self._catat(float(trial.value))
        if self.stagnan() and not self.berhenti:
            self.berhenti = True
            print("STAGNAN: %d trial selesai berturut-turut tanpa perbaikan > %g %% (skor acuan %.4f). "
                  "Tuning berhenti, lanjut ke laporan." % (self.tanpa_perbaikan, 100.0 * self.ambang, self.acuan))
            study.stop()


def cetak_kepala(study, penguji, variabel, bawah, atas, jumlah_trial, penjaga, awal=({}, {})):
    """Ringkasan pengaturan di awal tuning, lalu judul kolom log. awal = mulai_dari_config()."""
    profil = penguji.profil
    garis = "=" * len(FORMAT_HEADER % JUDUL_KOLOM)
    print(garis)
    print("STUDY %s | TAHAP %s | blok: %s" % (study.study_name, TAHAP, " -> ".join(
        "%s (%s, %s)" % b for b in penguji.blok)))
    print(teks_label(profil))
    if not config.OPTUNA_PROFIL[profil]["terverifikasi"]:
        print("!! " + config.OPTUNA_PROFIL[profil]["catatan"])
    print("Dicari (langkah %.4f, batas dari %s):" % (config.OPTUNA_LANGKAH, tabel_batas(profil)[1]))
    for nama in variabel:
        print("    %-9s %10.4f .. %10.4f" % (nama, bawah[nama], atas[nama]))
    params_awal, dilewati = awal
    if params_awal:
        print("Trial pertama = nilai config.py: %s" % ", ".join("%s %.4f" % (k, v) for k, v in params_awal.items()))
    if dilewati:
        print("    (di luar batas, dipilih Optuna: %s)" % ", ".join("%s %.4f" % (k, v) for k, v in dilewati.items()))
    kunci = penguji.terkunci()
    print("Dikunci (dari config.py): " + (", ".join("%s %.4f" % (k, v) for k, v in kunci.items()) or "-"))
    kisi = daftar_gaya(profil)
    kisi_miring = daftar_miring()
    print("S1 lantai %.1f deg: gaya %.1f .. %.1f N (langkah %.1f N), bobot %.0f per N"
          % (config.OPTUNA_ZETA_DEG, kisi[0], kisi[-1], config.OPTUNA_GAYA_LANGKAH, config.OPTUNA_BOBOT_GAYA))
    print("S2 bidang miring: %.1f .. %.1f deg (langkah %.1f deg), %.0f s x %d ulangan, bobot %.0f per deg | "
          "S3 gaya S1 di kemiringan S2, bonus %.0f"
          % (kisi_miring[0], kisi_miring[-1], config.OPTUNA_MIRING_LANGKAH, config.OPTUNA_MIRING_LAMA,
             config.OPTUNA_ULANGAN, config.OPTUNA_BOBOT_MIRING, config.OPTUNA_BOBOT_S3))
    if penguji.cbf is not None:
        print("SYARAT PD+CBF (acuan = blok PD mode yang sama di trial itu): S1 langsung pada gaya acuan, S2 pada "
              "kemiringan acuan, S3 lolos bila acuan lolos; gagal = trial berhenti + %.0f"
              % config.OPTUNA_PENALTI_SYARAT)
    print("Dorongan: %s, jarak acak %.1f .. %.1f s (seed %d + k), %d ulangan, tunggu stabil maks %.0f s"
          % (" -> ".join(nama_dorongan()), config.OPTUNA_JARAK_ACAK_MIN, config.OPTUNA_JARAK_ACAK_MAKS,
             config.OPTUNA_SEED_JARAK, config.OPTUNA_ULANGAN, config.OPTUNA_DURASI_STABIL))
    print("SYARAT LOLOS: tidak jatuh, ch/s <= %g, overshoot psi <= %g %%; blok BALANCING juga overshoot s <= %g %%, "
          "|s| <= s_max (S2) dan KOMET s" % (config.OPTUNA_BATAS_CHATTERING, config.OPTUNA_SYARAT_OVERSHOOT_PSI,
                                             config.OPTUNA_SYARAT_OVERSHOOT_S))
    print("Pemangkasan aman: %s | mulai dari trial terbaik: %s | %s."
          % ("YA" if config.OPTUNA_PANGKAS_AMAN else "TIDAK", "YA" if config.OPTUNA_MULAI_DARI_TERBAIK else "TIDAK",
             penjaga.teks()))
    selesai = len([t for t in study.trials if t.state != optuna.trial.TrialState.WAITING])
    print("%d trial (sudah ada %d). Ctrl+C untuk berhenti: trial yang selesai tetap tersimpan." % (jumlah_trial, selesai))
    print(garis)
    print(FORMAT_HEADER % JUDUL_KOLOM)
    print("-" * len(garis))


def mulai_dari_config(study, variabel, bawah, atas):
    """Study BARU (belum ada trial): trial pertama = nilai config.py (dibulatkan ke kisi Optuna). Nilai di luar batas
    pencarian tidak diisi. Mengembalikan (params yang dimasukkan, dict nilai di luar batas)."""
    params = {}
    dilewati = {}
    if not config.OPTUNA_MULAI_DARI_CONFIG or len(study.trials) > 0:
        return params, dilewati
    langkah_cari = float(config.OPTUNA_LANGKAH)
    for nama in variabel:
        nilai = float(getattr(config, nama))
        if not bawah[nama] <= nilai <= atas[nama]:
            dilewati[nama] = nilai
            continue
        k_maks = math.floor((atas[nama] - bawah[nama]) / langkah_cari + 1e-9)
        k = min(round((nilai - bawah[nama]) / langkah_cari), k_maks)
        params[nama] = round(bawah[nama] + k * langkah_cari, 10)
    if params:
        study.enqueue_trial(params)
    return params, dilewati


def jalankan(profil, jumlah_trial, stagnant=None):
    """stagnant: batas stagnan (trial); None = config.OPTUNA_STAGNANT."""
    variabel = list(config.OPTUNA_VARIABEL)
    bawah, atas = batas_pencarian(variabel, profil)

    penguji = Penguji(profil)
    daftar_gaya(profil)                                     # berhenti dengan pesan bila batas gaya salah
    daftar_miring()                                         # berhenti dengan pesan bila kisi kemiringan salah
    kunci = penguji.terkunci()
    for nama in kunci:
        if nama in config.NAMA_ALPHA and kunci[nama] <= 0.0:
            sys.exit("%s dikunci dari config.py tetapi nilainya %.4f (harus > 0)." % (nama, kunci[nama]))
    study = buat_study(profil)
    periksa_study(study, penguji)
    if stagnant is None:
        stagnant = config.OPTUNA_STAGNANT
    penjaga = PenjagaStagnan(study, stagnant, config.OPTUNA_STAGNANT_MIN)
    awal = mulai_dari_config(study, variabel, bawah, atas)
    cetak_kepala(study, penguji, variabel, bawah, atas, jumlah_trial, penjaga, awal)

    def objektif(trial):
        params = {}
        for nama in variabel:
            params[nama] = trial.suggest_float(nama, bawah[nama], atas[nama], step=config.OPTUNA_LANGKAH)
        t0 = time.perf_counter()
        penguji.awal_cari = awal_dari_terbaik(study, penguji.blok)
        terbaik = trial_terbaik(study)
        skor, status, detail = penguji.jalankan_trial(params, terbaik.value if terbaik is not None else None)
        trial.set_user_attr("keterangan", ("%s %s" % (status, detail)).strip())
        trial.set_user_attr("lama_s", time.perf_counter() - t0)
        for nama in KOLOM_DIAG:
            trial.set_user_attr(nama, penguji.diag[nama])
        for nama, nilai in penguji.hasil.items():
            trial.set_user_attr(nama, nilai)
        return skor

    def cetak_trial(study, trial):
        atribut = trial.user_attrs
        status, _, detail = atribut.get("keterangan", trial.state.name).partition(" ")
        teks_skor = "%11.3f" % trial.value if trial.value is not None else "%11s" % "-"
        terbaik = trial_terbaik(study)
        teks_terbaik = "%.3f (#%d)" % (terbaik.value, terbaik.number) if terbaik is not None else "-"
        teks_gagal = "%6.1f" % atribut.get("lama_gagal_s", 0.0) if penguji.cbf is not None else "%6s" % "-"
        print(FORMAT_BARIS % (trial.number, status, teks_skor, atribut.get("lama_s", 0.0), teks_terbaik,
                              teks_gaya(atribut.get("gaya_N", 0.0)), teks_miring(atribut.get("miring_deg")),
                              atribut.get("chattering_per_s", 0.0), atribut.get("psi_maks_deg", 0.0), teks_gagal,
                              detail))

    optuna.logging.set_verbosity(optuna.logging.WARNING)
    if penjaga.stagnan():
        print("STAGNAN sejak sebelumnya: %d trial selesai terakhir tanpa perbaikan > %g %% (batas %d). Tidak ada trial "
              "baru; untuk tetap melanjutkan pakai --stagnant dengan angka lebih besar, atau --stagnant 0."
              % (penjaga.tanpa_perbaikan, 100.0 * penjaga.ambang, penjaga.stagnant))
    else:
        try:
            study.optimize(objektif, n_trials=jumlah_trial, callbacks=[cetak_trial, penjaga])
        except KeyboardInterrupt:
            print("\nDihentikan dengan Ctrl+C.")
    laporkan(profil, penguji)


# =============================================================================
# LAPORAN: CSV, PNG, nilai untuk config.py
# =============================================================================
def laporkan(profil, penguji):
    if not ada_study(profil):
        print("Study %s belum pernah dijalankan." % nama_study(profil))
        return
    study = buat_study(profil)
    terbaik = trial_terbaik(study)
    if terbaik is None:
        print("Study %s: belum ada trial yang selesai." % nama_study(profil))
        return
    penguji.atur_profil(profil)

    # Nilai terkunci memakai nilai study (bukan config.py sekarang).
    skenario = study.user_attrs.get("skenario", {})
    for nama, nilai_lama, nilai_baru in beda_skenario(study, penguji.skenario()):
        print("PERHATIAN: %s berbeda dari saat study dibuat (study %s, sekarang %s); uji ulang di bawah memakai "
              "nilai %s." % (nama, nilai_lama, nilai_baru, "study" if nama.startswith("kunci ") else "sekarang"))
    gain = {nama[len("kunci "):]: nilai for nama, nilai in skenario.items() if nama.startswith("kunci ")}
    gain.update(terbaik.params)

    # Uji ulang gain terbaik, blok terakhir: gaya lolos S1 di lantai datar dan di kemiringan lolos S2 (S3).
    a = terbaik.user_attrs
    nama_blok, mode, kontrol = penguji.blok[-1]
    penguji.atur_mode(mode)
    b = nama_blok.lower()
    gaya_uji = float(a.get("gaya_%s_N" % b, 0.0)) or daftar_gaya(profil)[0]
    miring_terbaik = float(a.get("miring_%s_deg" % b, 0.0))
    zeta_miring = miring_terbaik if miring_terbaik > 0.0 else config.OPTUNA_ZETA_UJI_DEG
    hasil_uji = []
    for zeta_deg in (config.OPTUNA_ZETA_DEG, zeta_miring):
        skor, status, detail, rekaman, diag = penguji.dalam_mode(kontrol == KONTROL_CBF, penguji.uji, gain, zeta_deg,
                                                                 gaya_uji, rekam=True)
        hasil_uji.append((zeta_deg, skor, ("%s %s" % (status, detail)).strip(), rekaman, diag))

    selesai = len([t for t in study.trials if t.state == optuna.trial.TrialState.COMPLETE])
    print("\n================ HASIL TAHAP %s | %s ================" % (TAHAP, study.study_name))
    print(teks_label(profil))
    print("trial: %d total, %d selesai" % (len(study.trials), selesai))
    print("trial terbaik: %d, skor %.4f (%s)" % (terbaik.number, terbaik.value, a.get("keterangan", "-")))
    for nama, _, _ in penguji.blok:
        k = nama.lower()
        if "gaya_%s_N" % k in a:
            print("    %-14s gaya %s N | miring %s deg | S3 %s" % (
                nama, teks_gaya(a["gaya_%s_N" % k]), teks_miring(a.get("miring_%s_deg" % k)),
                "-" if "s3_%s_lolos" % k not in a else ("lolos" if a["s3_%s_lolos" % k] else "gagal")))
    for zeta_deg, skor, keterangan, _, diag in hasil_uji:
        print("uji ulang %s, lantai %.1f deg, %.1f N: skor %.4f | psimaks %.1f deg | ch/s %.1f | gagal %.1f s | %s"
              % (nama_blok, zeta_deg, gaya_uji, skor, diag["psi_maks_deg"], diag["chattering_per_s"],
                 diag["lama_gagal_s"], keterangan))
    print("\nSalin ke config.py:")
    for nama in terbaik.params:
        print("%s = %.4f" % (nama, terbaik.params[nama]))
    print("=================================================\n")

    awalan = os.path.join(FOLDER_HASIL, datetime.now().strftime("%Y%m%d_%H%M%S") + "_" + nama_file(profil))
    simpan_csv(awalan + ".csv", study, profil)
    simpan_png(awalan + ".png", profil, study, terbaik.params, hasil_uji, gaya_uji, nama_blok)


def simpan_csv(path, study, profil):
    nama_param = []
    for t in study.trials:
        for nama in t.params:
            if nama not in nama_param:
                nama_param.append(nama)
    kolom_hasil = [nama for nama in KOLOM_HASIL if any(nama in t.user_attrs for t in study.trials)]
    with open(path, "w", newline="") as f:
        penulis = csv.writer(f)
        penulis.writerow(["trial", "status", "skor", "keterangan", "lama_s", "profil", "terverifikasi"]
                         + nama_param + kolom_hasil + KOLOM_DIAG)
        for t in study.trials:
            skor = "%.6f" % t.value if t.value is not None else ""
            baris = [t.number, t.state.name, skor, t.user_attrs.get("keterangan", ""),
                     "%.1f" % t.user_attrs.get("lama_s", 0.0), profil,
                     "YA" if config.OPTUNA_PROFIL[profil]["terverifikasi"] else "BELUM"]
            for nama in nama_param:
                baris.append("%.4f" % t.params[nama] if nama in t.params else "")
            for nama in kolom_hasil + KOLOM_DIAG:
                baris.append("%.4f" % t.user_attrs[nama] if nama in t.user_attrs else "")
            penulis.writerow(baris)
    print("Tersimpan:", path)


def simpan_png(path, profil, study, gain, hasil_uji, gaya_uji, nama_blok):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib tidak terpasang -> PNG tidak dibuat")
        return

    fig, ax = plt.subplots(3, 1, figsize=(11, 11))

    # 1. Riwayat tiap trial: gaya lolos dan gaya trial terbaik sejauh ini, kemiringan lolos di sumbu kanan.
    nomor_selesai, nilai_selesai, nomor_terbaik, nilai_terbaik = [], [], [], []
    skor_terbaik = math.inf
    nilai_trial_terbaik = None
    selesai = [t for t in study.trials if t.state == optuna.trial.TrialState.COMPLETE]
    for t in selesai:
        nilai = t.user_attrs.get("gaya_N", 0.0)
        nomor_selesai.append(t.number)
        nilai_selesai.append(nilai)
        if t.value < skor_terbaik:
            skor_terbaik = t.value
            nilai_trial_terbaik = nilai
        nomor_terbaik.append(t.number)
        nilai_terbaik.append(nilai_trial_terbaik)
    ax[0].plot(nomor_selesai, nilai_selesai, "o", ms=3, color="tab:blue", label="trial selesai")
    ax[0].step(nomor_terbaik, nilai_terbaik, where="post", color="red", label="terbaik sejauh ini")
    ax[0].set_ylabel("gaya lolos tertinggi blok terakhir [N]")
    trial_miring = [t for t in selesai if "miring_deg" in t.user_attrs]
    if trial_miring:
        kanan = ax[0].twinx()
        kanan.plot([t.number for t in trial_miring], [t.user_attrs["miring_deg"] for t in trial_miring], "x", ms=4,
                   color="tab:green", label="kemiringan lolos S2")
        kanan.set_ylabel("kemiringan lolos S2 [deg]", color="tab:green")
        kanan.legend(fontsize=8, loc="lower right")
    ax[0].set_xlabel("trial")
    ax[0].set_title("Riwayat optimasi tahap %s | %s" % (TAHAP, teks_label(profil)), fontsize=10,
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
            data = rekaman[0]
            for waktu, nama in zip(data["waktu_dorong"], data["nama_dorong"]):
                for a in (ax[1], ax[2]):
                    a.axvline(waktu, color="black", lw=0.8, ls=":")
                ax[1].annotate(nama, (waktu, 1.0), xycoords=("data", "axes fraction"), fontsize=8,
                               xytext=(3, -10), textcoords="offset points")
    batas_psi = math.degrees(config.psi_max)
    ax[1].axhline(batas_psi, color="red", lw=2.2)
    ax[1].axhline(-batas_psi, color="red", lw=2.2)
    ax[2].axhline(config.s_max, color="0.6", lw=1.0)
    ax[2].axhline(config.s_min, color="0.6", lw=1.0)
    ax[1].set_ylabel("psi Kalman [deg]")
    ax[2].set_ylabel("s [m]")
    for a in (ax[1], ax[2]):
        a.set_xlabel("t sejak dorongan pertama tiap ulangan [s]")
        a.grid(True)
        a.legend(fontsize=8)
    teks_gain = ", ".join("%s=%.4f" % (nama, gain[nama]) for nama in gain)
    ax[1].set_title("Respon gain terbaik, blok %s (dorong %.1f N, %d ulangan): %s"
                    % (nama_blok, gaya_uji, config.OPTUNA_ULANGAN, teks_gain), fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)
    print("Tersimpan:", path)


def hapus_study(profil):
    if not ada_study(profil):
        print("Study %s belum ada di database." % nama_study(profil))
        return
    jawaban = input("Hapus SEMUA trial study %s dari database? ketik YA: " % nama_study(profil))
    if jawaban.strip() == "YA":
        optuna.delete_study(study_name=nama_study(profil), storage=DATABASE)
        print("Study %s dihapus." % nama_study(profil))
    else:
        print("Batal.")


def main():
    daftar_profil = list(config.OPTUNA_PROFIL.keys())
    parser = argparse.ArgumentParser(description="Tuning satu tahap (SEMUA) gain PD dan Alpha CBF-QP ATERA")
    parser.add_argument("--profil", choices=daftar_profil, help="profil torsi roda (config.OPTUNA_PROFIL)")
    parser.add_argument("--trial", type=int, default=None, help="jumlah trial (default config.OPTUNA_TRIAL)")
    parser.add_argument("--stagnant", type=int, default=None,
                        help="auto-exit bila sekian trial selesai tanpa perbaikan "
                             "(default config.OPTUNA_STAGNANT, 0 = mati)")
    parser.add_argument("--hasil", action="store_true",
                        help="cetak hasil terbaik, buat ulang CSV + PNG (tanpa --profil: semua profil)")
    parser.add_argument("--hapus", action="store_true", help="hapus study profil --profil dari database")
    args = parser.parse_args()

    if args.hapus:
        if args.profil is None:
            parser.error("--hapus butuh --profil")
        hapus_study(args.profil)
    elif args.hasil:
        ada = [p for p in (daftar_profil if args.profil is None else [args.profil]) if ada_study(p)]
        if len(ada) == 0:
            print("Belum ada study untuk profil yang diminta.")
            return
        penguji = Penguji(ada[0])
        for profil in ada:
            laporkan(profil, penguji)
    elif args.profil:
        jalankan(args.profil, args.trial if args.trial is not None else config.OPTUNA_TRIAL, args.stagnant)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()