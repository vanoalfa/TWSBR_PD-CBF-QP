"""tuning_optuna.py - mencari gain PD dan Alpha CBF-QP terbaik secara bertahap dengan Optuna (simulasi PyBullet).

Dua profil (config.OPTUNA_PROFIL), masing-masing dengan study sendiri supaya hasilnya tidak tercampur:
    TERUKUR   : torsi roda 0.238 N m/A (hasil uji delay)
    DATASHEET : torsi roda 0.75 N m/A (datasheet Waveshare)   -> BELUM TERVERIFIKASI

Lima tahap BERTAHAP + satu tahap GABUNGAN (config.OPTUNA_TAHAP), dijalankan satu per satu. Satu tahap = satu study
ATERA_<TAHAP>_<PROFIL> di database OPTUNA/atera_optuna.db:
    PD            Kp_psi, Kd_psi, Kp_theta, Kd_theta   mode PD saja, cari gaya dorong tertinggi yang lolos
    CBFQP_PSI     Alpha_1, Alpha_2           PD + CBF-QP, barrier h1 h2
    CBFQP_DPSI    Alpha_3                    PD + CBF-QP, barrier h1 .. h4
    CBFQP_S       Alpha_4, Alpha_5           PD + CBF-QP, barrier h1 .. h6
    CBFQP_DTHETA  Alpha_6                    PD + CBF-QP, barrier h1 .. h8
    SEMUA         ke-10 variabel di atas     SEKALIGUS (tuning gabungan, terpisah dari urutan bertahap). Tiap trial
                                             menguji gain yang SAMA dua kali: PD saja, lalu PD + CBF-QP h1 .. h8
                                             (lihat TAHAP SEMUA DUA MODE). Bandingkan hasilnya: --banding
Variabel yang tidak dituning di satu tahap SELALU diambil dari config.py, jadi salin hasil tiap tahap ke config.py
sebelum menjalankan tahap berikutnya. Batas pencarian: config.OPTUNA_BATAS[profil] (bawah, atas) per variabel, berbeda
untuk tiap profil karena torsi roda yang lebih kuat membutuhkan gain yang lebih kecil.
Study tidak boleh dilanjutkan bila nilai yang dikunci atau skenario (dorongan, durasi, penalti, fisika simulasi
FISIKA_SKENARIO, dst) berubah: program berhenti dan menjelaskan bedanya, supaya skor lama dan baru tidak tercampur.
Study baru (belum ada trial) dimulai dari nilai config.py sebagai trial pertama bila OPTUNA_MULAI_DARI_CONFIG.

Satu PENGUJIAN (headless, lantai config.OPTUNA_ZETA_DEG, mode DIAM) = OPTUNA_ULANGAN ulangan skenario, robot
di-reset di awal tiap ulangan:
    tenang sampai stabil -> dorongan berurutan OPTUNA_ARAH_DORONG berjarak OPTUNA_DURASI_DORONGAN s (stabil atau
    belum) -> tunggu stabil setelah dorongan terakhir. Ulangan ke-k memakai seed noise OPTUNA_SEED + k dan waktu
    dorong yang digeser k x OPTUNA_GESER_DORONG s. Skor pengujian = jumlah skor semua ulangan.
Satu TRIAL:
    tahap PD     : bisection gaya dorong (profil gaya_dorong .. OPTUNA_GAYA_MAKS, langkah OPTUNA_GAYA_LANGKAH);
                   tiap gaya yang dicoba = satu pengujian. Skor = -OPTUNA_BOBOT_GAYA x gaya_lolos + skor pengujian
                   pada gaya_lolos (gaya lebih tinggi selalu lebih baik; ISE hanya membedakan gaya yang sama).
    tahap SEMUA  : sama dengan tahap PD, tetapi CBF-QP h1 .. h8 aktif (penalti GAGAL ikut dihitung).
    tahap CBFQP_*: pengujian berurutan dari gaya terkecil OPTUNA_GAYA_CBF_FRAKSI x gaya_pd_terbaik. Pada gaya
                   pertama yang komet, trial BERHENTI: gaya yang lebih besar tidak diuji (dilewati) dan tiap gaya
                   yang dilewati diberi penalti komet maksimum 2 x OPTUNA_PENALTI_KOMET (seperti komet di detik 0).
                   ASUMSI: komet pada gaya F berarti komet juga pada gaya yang lebih besar (sama dengan tahap PD).
                   Skor trial = jumlah skor gaya yang diuji + penalti gaya yang dilewati, jadi trial yang komet
                   pada gaya lebih kecil hampir selalu lebih buruk (selama skor satu gaya yang lolos jauh di bawah
                   OPTUNA_PENALTI_KOMET). Alpha dinilai dari dorongan kecil sampai besar.
Rumus skor pengujian: config.py bagian 13. Makin kecil makin baik.

SYARAT LOLOS satu pengujian (config.py bagian 13, blok "SYARAT LOLOS"): tidak jatuh (KOMET), tidak chattering
(ch/s <= OPTUNA_BATAS_CHATTERING), dan tidak berayun (overshoot psi <= OPTUNA_SYARAT_OVERSHOOT_PSI, overshoot s <=
OPTUNA_SYARAT_OVERSHOOT_S). Yang tidak memenuhi dihitung KOMET (penalti OPTUNA_PENALTI_KOMET). Overshoot dihitung
tiap dorongan (fungsi overshoot_persen), yang terbesar dipakai: 0 % = kembali tanpa melewati titik awal (critically
damped / overdamped), makin besar makin berayun (underdamped).

TAHAP SEMUA DUA MODE: gain PD dan Alpha yang SAMA diuji PD saja (CBF-QP mati) lalu PD + CBF-QP:
    skor trial = skor PD saja + skor PD + CBF-QP + OPTUNA_BOBOT_CBF_BURUK x max(0, skor PD + CBF-QP - skor PD saja)
    (skor tiap mode seperti tahap PD: -OPTUNA_BOBOT_GAYA x gaya lolos + skor pengujian pada gaya itu). Jadi PD harus
    bagus sendirian, dan trial yang PD + CBF-QP-nya lebih buruk dari PD saja dihukum, sehingga Optuna mencari Alpha
    yang membuat CBF-QP tidak memperburuk PD.

SKENARIO S1 S2 S3 (tahap dengan "miring": True, yaitu SEMUA; config.py blok "SKENARIO S1 S2 S3"). Tiap mode:
    S1 dorongan di lantai datar   : gaya dorong dinaikkan sampai gagal (bisection, seperti tahap PD) -> gaya_lolos
    S2 bidang miring tanpa dorongan: kemiringan dinaikkan OPTUNA_MIRING_LANGKAH tiap berhasil (bisection) ->
                                    miring_lolos. Berhasil = tidak jatuh, |s| <= s_max, ch/s <= batas, selama
                                    OPTUNA_MIRING_LAMA detik tiap ulangan (uji_miring)
    S3 dorongan di bidang miring  : gaya_lolos S1 di kemiringan miring_lolos S2 (skenario dan syarat lolos S1)
    skor mode = skor S1 - OPTUNA_BOBOT_MIRING x miring_lolos - (OPTUNA_BOBOT_S3 bila S3 lolos)
    S2 dan S3 baru dijalankan bila S1 punya gaya lolos (> 0); S3 butuh miring_lolos > 0.

Dua jenis penghentian otomatis:
    pruning   : satu TRIAL dihentikan bila skor sementaranya (dilaporkan tiap 1 detik simulasi) lebih buruk dari
                median trial sebelumnya pada detik yang sama (MedianPruner, hanya tahap CBFQP_*).
    stagnan   : seluruh TAHAP berhenti sendiri bila skor terbaik tidak membaik selama OPTUNA_STAGNANT trial selesai
                berturut-turut (PenjagaStagnan), lalu laporan dibuat seperti biasa. Lihat config.py bagian 13.

Log terminal tiap trial: trial | status | skor | waktu nyata | terbaik | gaya | miring | ch/s | psimaks | gagal | detail
    gaya    = gaya lolos tertinggi [N] (tanpa komet), "-" bila gaya terendah pun komet
    miring  = kemiringan lolos S2 [deg] (tahap "miring"; dua mode: mode PD + CBF-QP), "-" bila tidak ada
    ch/s    = chattering per detik: berapa kali u berbalik dari >= +a ke <= -a (atau sebaliknya),
              a = OPTUNA_AMBANG_CHATTERING x u_max. Getaran kecil di sekitar 0 (noise) tidak dihitung.
              ch/s > OPTUNA_BATAS_CHATTERING = pengujian dianggap KOMET (kontrol bergetar tidak boleh dianggap lolos).
    psimaks = puncak |psi| [deg],  gagal = lama status CBF bukan AMAN [s] (tahap CBFQP_* dan SEMUA)
    status  = selesai (tanpa komet) | komet | pruned
    detail  = penyebab komet / gaya yang gagal (tahap PD, SEMUA), dan fase yang tidak mencapai stabil (tanpa penalti):
              awal (sebelum dorongan pertama) / akhir (setelah dorongan terakhir), U1 / U2 = ulangan ke-1 / ke-2
CSV memuat lebih banyak: puncak |psi|, |dpsi|, |s|, lama GAGAL, jumlah fase tidak stabil, puncak |psi|, |dpsi|,
|dtheta| pada OPTUNA_STABIL_LAMA detik terakhir fase awal dan fase terakhir, dan skor yang dipecah menjadi ISE dan
tiap penalti. Untuk tahap PD, diagnostik berasal dari pengujian pada gaya lolos tertinggi (atau gaya terendah bila
semua komet).

Jalankan (di folder simulasi):
    python3 tuning_optuna.py --profil TERUKUR --tahap PD            jumlah trial dari config.OPTUNA_TAHAP
    python3 tuning_optuna.py --profil TERUKUR --tahap CBFQP_PSI --trial 50   MENAMBAH 50 trial ke study yang ada
    python3 tuning_optuna.py --profil TERUKUR --tahap PD --stagnant 500      batas stagnan 500 trial (0 = mati)
    python3 tuning_optuna.py --hasil                                cetak hasil terbaik semua profil dan tahap, buat ulang CSV + PNG
    python3 tuning_optuna.py --hasil --profil TERUKUR               hanya satu profil
    python3 tuning_optuna.py --profil TERUKUR --hapus PD            hapus study satu tahap (mulai dari nol)
    python3 tuning_optuna.py --profil TERUKUR --banding             BERTAHAP vs SEMUA, skenario sama

Hasil (folder OPTUNA):
    atera_optuna.db                                      database semua trial (bisa dilanjutkan kapan saja)
    YYYYMMDD_HHmmSS_OPTUNA_PD_TERUKUR.csv / .png         riwayat trial + respon gain terbaik di zeta 0 dan 5.2 deg
    YYYYMMDD_HHmmSS_OPTUNA_BANDING_TERUKUR.csv / .png    hasil --banding (BERTAHAP vs SEMUA)
    (untuk profil DATASHEET: ..._DATASHEET, diberi label BELUM TERVERIFIKASI)
Nilai terbaik dicetak di terminal dalam format config.py (termasuk gaya_pd_terbaik untuk tahap PD). Salin sendiri
ke config.py.

SAMA DENGAN ROBOT FISIK (supaya hasil tuning langsung disalin ke program/atera_program/config.py):
    - CBF-QP tidak diberi tahu kemiringan lantai (config.CBF_TAHU_ZETA = False, model CBF zeta = 0, seperti robot
      dengan zeta_DEG = 0); gangguan kemiringan, dorongan, dan selisih model ditaksir DOB.
    - DOB dijalankan tiap siklus dengan urutan robot: DOB perbarui -> PD -> CBF-QP -> DOB catat u, dan dikosongkan
      di awal tiap ulangan (robot: saat MULAI).
    - safety set, Alpha, gain, slack, margin, dan DOB dibaca dari config.py simulasi yang nilainya = config.py robot.

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
from pd_control import PDControl

FOLDER_INI = os.path.dirname(os.path.abspath(__file__))
FOLDER_HASIL = os.path.join(FOLDER_INI, "OPTUNA")
DATABASE = "sqlite:///" + os.path.join(FOLDER_HASIL, "atera_optuna.db")
NAMA_GAIN_PD = ["Kp_psi", "Kd_psi", "Kp_theta", "Kd_theta"]
TAHAP_GABUNGAN = "SEMUA"
# Nilai config.py yang menentukan fisika simulasi dan KOMET: ikut dicatat di skenario study, jadi study yang dibuat
# dengan simulasi berbeda (contoh: sebelum ada SIM_BATAS_KECEPATAN) tidak tercampur. Nama yang tidak ada dilewati.
FISIKA_SKENARIO = ["CONTROL_HZ", "SIM_JEDA_TORSI", "SIM_BATAS_KECEPATAN", "MOTOR_NO_LOAD_SPEED_RPM",
                   "MOTOR_STALL_CURRENT_A", "MOTOR_KT", "MAX_CURRENT_A", "SIM_MODEL_IMU", "SIM_GYRO_NOISE_DEG_S",
                   "SIM_ACC_NOISE_DEG", "LAMA_DORONG", "DORONG_SISI", "DORONG_GESER_SAMPING", "SAFE_TILT_DEG",
                   "S_KOMET", "psi_max", "dpsi_max", "s_max",
                   "CBF_TAHU_ZETA", "DOB_AKTIF", "DOB_TAU_S", "DOB_BATAS", "CBF_MARGIN_DEG"]
TEKS_ARAH = {1: "depan", -1: "belakang"}
STATUS_SELESAI = "selesai"
STATUS_KOMET = "komet"

# Diagnostik tiap trial (disimpan sebagai atribut trial dan kolom CSV).
# skor pengujian = ise + pen_jarak + pen_gagal + pen_komet
# gaya_N              = gaya lolos tertinggi (tahap PD / SEMUA, 0 bila tidak ada) atau gaya yang dipakai
#                       (tahap CBFQP_*)
# chattering_per_s    = u berbalik dari >= +a ke <= -a (atau sebaliknya) per detik bagian yang dinilai,
#                       a = OPTUNA_AMBANG_CHATTERING x u_max (getaran kecil akibat noise tidak dihitung)
# jumlah_tidak_stabil = jumlah fase (tenang awal, setelah dorongan terakhir, semua ulangan) yang tidak mencapai stabil
# overshoot_*_persen  = overshoot terbesar psi / s dari semua dorongan [%] (overshoot_persen, syarat lolos)
# awal_* / akhir_*    = puncak |psi|, |dpsi|, |dtheta| pada OPTUNA_STABIL_LAMA detik terakhir fase awal / fase
#                       terakhir, terbesar dari semua ulangan (satuan sama dengan OPTUNA_STABIL_*; 0 bila tidak
#                       tercapai karena komet)
KOLOM_STABIL = ["psi_rad", "dpsi_rad_s", "dtheta_rad_s"]
KOLOM_DIAG = (["gaya_N", "chattering_per_s", "psi_maks_deg", "dpsi_maks_rad_s", "s_maks_m", "lama_gagal_s",
               "jumlah_tidak_stabil"]
              + ["awal_" + nama for nama in KOLOM_STABIL] + ["akhir_" + nama for nama in KOLOM_STABIL]
              + ["ise", "pen_jarak", "pen_gagal", "pen_komet", "overshoot_psi_persen", "overshoot_s_persen"])
# Tahap SEMUA dua mode (atribut trial dan kolom CSV): gaya lolos dan skor PD saja, skor PD + CBF-QP, dan penalti
# bila PD + CBF-QP lebih buruk dari PD saja (selisih skor, sebelum dikali OPTUNA_BOBOT_CBF_BURUK).
KOLOM_DUA = ["gaya_pd_saja_N", "skor_pd_saja", "skor_pd_cbf", "cbf_lebih_buruk"]
# Skenario S1 S2 S3 (tahap "miring", atribut trial dan kolom CSV). Dua mode: tanpa akhiran = mode PD + CBF-QP,
# akhiran _pd_saja = mode PD saja. miring_deg = kemiringan lolos S2 [deg], s3_lolos = 1 (lolos) / 0 (gagal atau tidak
# dijalankan), miring_s_maks_m = puncak |s| pengujian S2 yang lolos pada miring_deg.
KOLOM_MIRING = ["miring_deg", "s3_lolos", "miring_s_maks_m", "miring_pd_saja_deg", "s3_pd_saja_lolos",
                "miring_s_maks_pd_saja_m"]

# Baris log terminal (lihat docstring): header dan isi memakai lebar kolom yang sama.
JUDUL_KOLOM = ("trial", "status", "skor", "waktu", "terbaik", "gaya", "miring", "ch/s", "psimaks", "gagal", "detail")
FORMAT_HEADER = "%5s | %-7s | %11s | %7s | %-18s | %6s | %6s | %7s | %7s | %6s | %s"
FORMAT_BARIS = "%5d | %-7s | %11s | %5.1f s | %-18s | %6s | %6s | %7.1f | %7.1f | %6s | %s"


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


def daftar_gaya_cbf(profil):
    """Gaya dorong tahap CBFQP_* [N], urut dari kecil ke besar tanpa duplikat (diuji berurutan dan berhenti pada
    gaya pertama yang komet): OPTUNA_GAYA_CBF_FRAKSI x gaya_pd_terbaik, atau None bila belum diisi."""
    terbaik = config.OPTUNA_PROFIL[profil].get("gaya_pd_terbaik")
    if terbaik is None:
        return None
    fraksi = list(config.OPTUNA_GAYA_CBF_FRAKSI)
    if len(fraksi) == 0 or min(fraksi) <= 0.0:
        sys.exit("Periksa config.py: OPTUNA_GAYA_CBF_FRAKSI harus berisi minimal satu angka > 0.")
    return sorted(set(round(f * float(terbaik), 2) for f in fraksi))


def teks_daftar_gaya(daftar):
    return ", ".join("%.2f" % g for g in daftar)


def penalti_dilewati():
    """Penalti tiap gaya tahap CBFQP_* yang tidak diuji karena gaya sebelumnya komet: sama dengan komet di detik 0
    (penalti komet maksimum, OPTUNA_PENALTI_KOMET x (1 + 1))."""
    return 2.0 * config.OPTUNA_PENALTI_KOMET


def nama_study(tahap, profil):
    return "ATERA_%s_%s" % (tahap, profil)                # ATERA_PD_TERUKUR, ATERA_CBFQP_S_DATASHEET, ...


def nama_file(tahap, profil):
    return "OPTUNA_%s_%s" % (tahap, profil)


def semua_variabel():
    return NAMA_GAIN_PD + config.NAMA_ALPHA


def nama_kolom_gaya(gaya):
    """Nama atribut / kolom CSV skor pengujian pada satu gaya (tahap CBFQP_*), contoh skor_5.75N."""
    return "skor_%gN" % gaya


def tabel_batas(profil):
    """(dict nama -> (bawah, atas), nama tabel). config.OPTUNA_BATAS boleh per profil {profil: {nama: (b, a)}}
    atau satu tabel untuk semua profil {nama: (b, a)} (format lama)."""
    tabel = config.OPTUNA_BATAS
    if profil in tabel:
        return tabel[profil], "config.OPTUNA_BATAS['%s']" % profil
    if any(isinstance(nilai, dict) for nilai in tabel.values()):
        sys.exit("config.OPTUNA_BATAS belum punya batas untuk profil %s." % profil)
    return tabel, "config.OPTUNA_BATAS"


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
    return "%.1f" % gaya if gaya > 0.0 else "-"


def teks_miring(miring):
    """Kemiringan lolos S2 untuk log, "-" bila 0 atau tidak ada (None)."""
    return "%.1f" % miring if miring is not None and miring > 0.0 else "-"


def overshoot_persen(nilai, acuan, puncak_min, n_rata):
    """Overshoot respon SATU dorongan [%] (syarat lolos, lihat config.py blok SYARAT LOLOS):
        e(t)      = nilai(t) - acuan           (acuan = rata-rata sinyal sesaat sebelum dorongan)
        e diratakan rata-rata bergerak n_rata sampel (meredam noise sensor)
        puncak    = e dengan |e| terbesar sejak dorongan sampai dorongan berikutnya
        overshoot = simpangan terbesar ke arah BERLAWANAN dengan puncak, setelah puncak, / |puncak| x 100
    0 % bila respon kembali tanpa melewati titik awal (critically damped / overdamped). Puncak lebih kecil dari
    puncak_min tidak dinilai (0 %): dorongan terlalu kecil untuk membedakan ayunan dari noise."""
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


def gabung_diag(daftar):
    """Diagnostik beberapa pengujian (tahap CBFQP_*, beberapa gaya) menjadi satu:
    jumlah untuk ISE, penalti, lama GAGAL, dan fase tidak stabil; nilai terbesar untuk puncak, gaya, dan ch/s."""
    hasil = dict.fromkeys(KOLOM_DIAG, 0.0)
    dijumlah = ("ise", "pen_jarak", "pen_gagal", "pen_komet", "lama_gagal_s", "jumlah_tidak_stabil")
    for d in daftar:
        for nama in KOLOM_DIAG:
            hasil[nama] = hasil[nama] + d[nama] if nama in dijumlah else max(hasil[nama], d[nama])
    return hasil


# =============================================================================
# PENGUJI: satu simulasi headless dipakai ulang untuk semua trial
# =============================================================================
class Penguji:
    def __init__(self, profil, pakai_cbf):
        """pakai_cbf: True bila ada tahap ber-CBF-QP (CBFQP_*, SEMUA) yang akan dijalankan (menyiapkan QP solver)."""
        # PyBullet hanya boleh satu dunia per program, jadi Penguji cukup dibuat SATU kali.
        self.sim = simulasi.Simulasi(gui=False)
        self.pd = PDControl()
        self.cbf = None
        if pakai_cbf:
            print("Menyiapkan CBF-QP (CasADi) ...")
            from cbf_qp import CBFQP
            # Sama dengan robot fisik (config.CBF_TAHU_ZETA = False): model CBF-QP memakai zeta = 0 walau lantai
            # simulasi miring; gangguan kemiringan ditaksir DOB (dijalankan tiap siklus, lihat _satu_ulangan).
            self.cbf = CBFQP(zeta=None if config.CBF_TAHU_ZETA else 0.0)
        # Nilai config.py saat program mulai: dasar semua variabel yang tidak dituning (nilai terkunci).
        self.dasar = {nama: float(getattr(config, nama)) for nama in semua_variabel()}
        self.tahap = None
        self.pakai_cbf = False              # tahap sekarang memakai CBF-QP
        self.paksa = {}                     # nilai yang dipaksa selama tahap sekarang (menggantikan config.py)
        self.skor_s = True                  # False: skor hanya dari psi
        self.penalti_jarak = True           # False: OPTUNA_PENALTI_JARAK tidak dihitung
        self.cari_gaya = False              # True: tiap trial mencari gaya dorong tertinggi yang lolos (tahap PD)
        self.dua_mode = False               # True: tiap trial PD saja + PD + CBF-QP dengan gain sama (tahap SEMUA)
        self.diag_dua = dict.fromkeys(KOLOM_DUA, 0.0)   # hasil dua mode trial terakhir (lihat KOLOM_DUA)
        self.miring = False                 # True: tiap trial (tiap mode) S1 -> S2 -> S3 (tahap SEMUA)
        self.diag_miring = {}               # hasil S2 / S3 trial terakhir (lihat KOLOM_MIRING)
        self.gaya_cbf = None                # daftar gaya dorong tahap CBFQP_* [N]
        self.skor_per_gaya = {}             # tahap CBFQP_*: gaya [N] -> skor pengujian trial terakhir
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
        self.dua_mode = bool(definisi.get("dua_mode", False))
        if self.dua_mode and not (self.pakai_cbf and self.cari_gaya):
            raise RuntimeError("Tahap %s: dua_mode butuh barrier (CBF-QP) dan cari_gaya = True." % tahap)
        self.miring = bool(definisi.get("miring", False))
        if self.miring and not self.cari_gaya:
            raise RuntimeError("Tahap %s: miring (S1 S2 S3) butuh cari_gaya = True." % tahap)
        self.diag_miring = {}
        self.gaya_cbf = None if self.cari_gaya else daftar_gaya_cbf(self.profil)

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
        hasil["batas_chattering"] = float(config.OPTUNA_BATAS_CHATTERING)
        hasil["ambang_chattering"] = float(config.OPTUNA_AMBANG_CHATTERING)
        for nama in ("OPTUNA_SYARAT_OVERSHOOT_PSI", "OPTUNA_SYARAT_OVERSHOOT_S", "OPTUNA_OVERSHOOT_ACUAN_S",
                     "OPTUNA_OVERSHOOT_RATA_S", "OPTUNA_OVERSHOOT_PUNCAK_PSI_DEG", "OPTUNA_OVERSHOOT_PUNCAK_S_M"):
            hasil[nama] = float(getattr(config, nama))
        hasil["dua_mode"] = self.dua_mode
        if self.dua_mode:
            hasil["bobot_cbf_buruk"] = float(config.OPTUNA_BOBOT_CBF_BURUK)
        if self.miring:
            # hanya dicatat bila aktif, supaya study tahap lain yang sudah ada tidak dianggap berubah skenarionya
            hasil.update({"miring": True, "miring_langkah": float(config.OPTUNA_MIRING_LANGKAH),
                          "miring_maks": float(config.OPTUNA_MIRING_MAKS),
                          "miring_lama": float(config.OPTUNA_MIRING_LAMA),
                          "bobot_miring": float(config.OPTUNA_BOBOT_MIRING), "bobot_s3": float(config.OPTUNA_BOBOT_S3)})
        for nama in FISIKA_SKENARIO:
            if hasattr(config, nama):
                nilai = getattr(config, nama)
                hasil["fisika " + nama] = nilai if isinstance(nilai, bool) else float(nilai)
        if self.pakai_cbf:
            hasil["dtheta_max"] = float(config.dtheta_max)            # batas barrier h7, h8 (hanya CBF-QP)
        if self.cari_gaya:
            hasil.update({"gaya_awal": float(data["gaya_dorong"]), "gaya_maks": float(config.OPTUNA_GAYA_MAKS),
                          "gaya_langkah": float(config.OPTUNA_GAYA_LANGKAH),
                          "bobot_gaya": float(config.OPTUNA_BOBOT_GAYA)})
        else:
            hasil["gaya_dorong"] = self.gaya_cbf
            hasil["berhenti_saat_komet"] = True       # cara menilai: gaya setelah komet dilewati (dengan penalti)
        kunci = self.terkunci()
        for nama in kunci:
            hasil["kunci " + nama] = kunci[nama]
        return hasil

    # -------------------------------------------------------------------------
    # Satu pengujian = OPTUNA_ULANGAN ulangan pada satu gaya dorong
    # -------------------------------------------------------------------------
    def uji(self, gain, zeta_deg, gaya, trial=None, rekam=False, laporan_awal=0, skor_awal=0.0):
        """Satu pengujian pada gaya dorong `gaya` [N].

        Mengembalikan (skor, status, detail, rekaman, diag):
            status  : STATUS_SELESAI atau STATUS_KOMET
            detail  : penyebab komet, dan/atau fase yang tidak mencapai stabil ("" bila tidak ada)
            rekaman : [dict] satu per ulangan (rekam=True): t [s sejak dorongan pertama ulangan itu], psi [deg],
                      s [m], waktu_dorong, nama_dorong, ulangan
            diag    : dict KOLOM_DIAG
        trial: bila diberikan, skor dilaporkan tiap 1 detik bagian yang dinilai (pruning); melempar
        optuna.TrialPruned bila trial pruned. laporan_awal = nomor laporan pertama dan skor_awal = skor gaya
        sebelumnya dalam trial yang sama (lihat uji_beberapa_gaya).
        """
        self.atur_zeta(zeta_deg)
        self.atur_gain(gain)
        d = dict.fromkeys(KOLOM_DIAG, 0.0)
        d["gaya_N"] = float(gaya)
        hitungan = {"langkah_dinilai": 0, "balik": 0, "laporan_ke": laporan_awal, "skor_awal": skor_awal}
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
            for nama, kunci in (("psi", "overshoot_psi_persen"), ("s", "overshoot_s_persen")):
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
        if self.cbf is not None:
            self.cbf.reset_dob()                  # robot fisik: DOB dikosongkan saat MULAI
        self.pd.set_command(0)                    # mode DIAM (tanpa perintah gerak)
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
        respon = {"psi": [], "s": [], "acuan_psi": 0.0, "acuan_s": 0.0}          # respon dorongan sekarang

        def tutup_respon():
            """Overshoot respon dorongan sekarang -> d (yang terbesar dari semua dorongan dan ulangan)."""
            if not respon["psi"]:
                return
            for nama, kunci, puncak_min in (("psi", "overshoot_psi_persen", config.OPTUNA_OVERSHOOT_PUNCAK_PSI_DEG),
                                            ("s", "overshoot_s_persen", config.OPTUNA_OVERSHOOT_PUNCAK_S_M)):
                nilai = overshoot_persen(respon[nama], respon["acuan_" + nama], puncak_min, n_rata)
                d[kunci] = max(d[kunci], nilai)

        def buka_respon():
            """Dorongan baru: acuan = rata-rata OPTUNA_OVERSHOOT_ACUAN_S detik sebelum dorongan."""
            respon["psi"] = []
            respon["s"] = []
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
                akhir_fase = k_fase >= langkah_dorongan
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
            else:
                u = self.cbf.compute(x, u_PD)
                gagal = not self.cbf.feasible              # status CBF-QP bukan AMAN (SLACK, GAGAL, atau ALPHA_NOL)
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
            if abs(u) >= ambang_u:
                tanda = math.copysign(1.0, u)
                if tanda_besar != 0.0 and tanda != tanda_besar:
                    hitungan["balik"] = hitungan["balik"] + 1
                tanda_besar = tanda
            hitungan["langkah_dinilai"] = hitungan["langkah_dinilai"] + 1

            # Laporan tiap 1 detik bagian yang dinilai (berlanjut antar ulangan) untuk pruning.
            if trial is not None and hitungan["langkah_dinilai"] % langkah_per_detik == 0:
                trial.report(hitungan["skor_awal"] + d["ise"] + d["pen_jarak"] + d["pen_gagal"],
                             hitungan["laporan_ke"])
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
        """Satu pengujian S2 di bidang miring zeta_deg [deg], TANPA dorongan, mode kontrol sekarang (self.pakai_cbf):
        OPTUNA_ULANGAN ulangan (reset, seed noise OPTUNA_SEED + k seperti uji), robot diletakkan tegak lalu dibiarkan
        OPTUNA_MIRING_LAMA detik dalam mode DIAM.
        BERHASIL bila sepanjang waktu |psi| <= SAFE_TILT_DEG (tidak jatuh) dan |s| <= s_max (tidak meluncur melewati
        barrier s), dan ch/s seluruh pengujian <= OPTUNA_BATAS_CHATTERING (0 = tidak dicek).
        Mengembalikan (berhasil, detail, info): detail = penyebab gagal ("" bila berhasil), info = dict s_maks_m,
        psi_maks_deg, chattering_per_s."""
        self.atur_zeta(zeta_deg)
        self.atur_gain(gain)
        dt = config.DT
        batas_psi = math.radians(config.SAFE_TILT_DEG)
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
            self.pd.set_command(0)                    # mode DIAM (tanpa perintah gerak)
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
                elif abs(s) > config.s_max:
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

    def cari_miring_tertinggi(self, gain):
        """S2: bisection kemiringan tertinggi (kisi daftar_miring) yang berhasil uji_miring, mode kontrol sekarang.
        ASUMSI: berhasil pada kemiringan z berarti berhasil pada kemiringan yang lebih kecil.
        Mengembalikan (miring_lolos [deg], 0 bila kisi pertama pun gagal; detail; info uji_miring yang berhasil pada
        miring_lolos, atau pada kisi pertama bila gagal)."""
        kisi = daftar_miring()
        berhasil, detail, info = self.uji_miring(gain, kisi[0])
        if not berhasil:
            return 0.0, "gagal pada %.1f deg: %s" % (kisi[0], detail), info
        # kisi[lolos] berhasil; kisi[gagal] gagal (gagal = len(kisi): di atas OPTUNA_MIRING_MAKS, dianggap gagal).
        lolos, gagal = 0, len(kisi)
        info_lolos = info
        detail_gagal = ""
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

    def nilai_mode(self, gain, pakai_cbf):
        """Satu mode kontrol (pakai_cbf False = PD saja), lalu mode tahap dikembalikan. Mask barrier CBF-QP tetap dari
        tahap (atur_tahap).
            tanpa "miring": S1 saja (cari_gaya_tertinggi di lantai OPTUNA_ZETA_DEG), sama persis dengan sebelumnya
            dengan "miring": S1 -> S2 (cari_miring_tertinggi) -> S3 (uji gaya_lolos S1 di miring_lolos S2)
                             skor = skor S1 - OPTUNA_BOBOT_MIRING x miring_lolos - (OPTUNA_BOBOT_S3 bila S3 lolos)
                             S2 dan S3 tidak dijalankan bila S1 gagal pada gaya terendah (miring_lolos = 0)
        Mengembalikan (skor, status, detail, hasil_miring). status dari S1 (komet bila gaya terendah pun komet).
        self.diag = diagnostik S1. hasil_miring = {} tanpa "miring", atau dict miring_deg, s3_lolos, miring_s_maks_m."""
        if pakai_cbf and self.cbf is None:
            raise RuntimeError("Mode PD + CBF-QP butuh Penguji dengan CBF-QP.")
        mode_tahap = self.pakai_cbf
        self.pakai_cbf = bool(pakai_cbf)
        try:
            skor, status, detail = self.cari_gaya_tertinggi(gain, config.OPTUNA_ZETA_DEG)
            if not self.miring:
                return skor, status, detail, {}
            diag_s1 = self.diag
            gaya = float(diag_s1["gaya_N"])
            if gaya <= 0.0:
                # S2 dan S3 baru dijalankan setelah S1 punya hasil (gaya lolos > 0)
                hasil = {"miring_deg": 0.0, "s3_lolos": 0.0, "miring_s_maks_m": 0.0}
                return skor, status, "S1 %s; S2 dan S3 tidak dijalankan (S1 gagal pada gaya terendah)" % detail, hasil
            miring, detail_s2, info_s2 = self.cari_miring_tertinggi(gain)
            if miring > 0.0:
                _, status_s3, detail_s3, _, _ = self.uji(gain, miring, gaya)
                s3_lolos = status_s3 == STATUS_SELESAI
                teks_s3 = "lolos" if s3_lolos else "komet " + detail_s3.split("; tidak stabil:")[0]
                teks_s3 = "S3 %.1f N di %.1f deg: %s" % (gaya, miring, teks_s3)
            else:
                s3_lolos = False
                teks_s3 = "S3 tidak dijalankan (kemiringan S2 = 0)"
            self.diag = diag_s1
            skor = skor - config.OPTUNA_BOBOT_MIRING * miring - (config.OPTUNA_BOBOT_S3 if s3_lolos else 0.0)
            detail = "S1 %s; S2 %s deg: %s; %s" % (detail, teks_miring(miring), detail_s2, teks_s3)
            hasil = {"miring_deg": miring, "s3_lolos": 1.0 if s3_lolos else 0.0, "miring_s_maks_m": info_s2["s_maks_m"]}
            return skor, status, detail, hasil
        finally:
            self.pakai_cbf = mode_tahap

    def cari_gaya_mode(self, gain, zeta_deg, pakai_cbf):
        """cari_gaya_tertinggi dengan mode kontrol tertentu (pakai_cbf False = PD saja), lalu mode tahap dikembalikan.
        Mask barrier CBF-QP tetap dari tahap (atur_tahap)."""
        if pakai_cbf and self.cbf is None:
            raise RuntimeError("Mode PD + CBF-QP butuh Penguji dengan CBF-QP.")
        mode_tahap = self.pakai_cbf
        self.pakai_cbf = bool(pakai_cbf)
        try:
            return self.cari_gaya_tertinggi(gain, zeta_deg)
        finally:
            self.pakai_cbf = mode_tahap

    def trial_dua_mode(self, gain):
        """Tahap SEMUA: gain yang SAMA diuji PD saja lalu PD + CBF-QP (nilai_mode: S1, dan S2 S3 bila "miring").
            skor = skor PD saja + skor PD + CBF-QP + OPTUNA_BOBOT_CBF_BURUK x max(0, skor PD + CBF-QP - skor PD saja)
        Mengembalikan (skor, status, detail). self.diag = diagnostik S1 PD + CBF-QP, self.diag_dua = KOLOM_DUA,
        self.diag_miring = KOLOM_MIRING (bila "miring")."""
        skor_pd, _, detail_pd, miring_pd = self.nilai_mode(gain, pakai_cbf=False)
        gaya_pd = self.diag["gaya_N"]
        skor_cbf, _, detail_cbf, miring_cbf = self.nilai_mode(gain, pakai_cbf=True)
        gaya_cbf = self.diag["gaya_N"]
        if self.miring:
            self.diag_miring = {"miring_deg": miring_cbf["miring_deg"], "s3_lolos": miring_cbf["s3_lolos"],
                                "miring_s_maks_m": miring_cbf["miring_s_maks_m"],
                                "miring_pd_saja_deg": miring_pd["miring_deg"], "s3_pd_saja_lolos": miring_pd["s3_lolos"],
                                "miring_s_maks_pd_saja_m": miring_pd["miring_s_maks_m"]}
        lebih_buruk = skor_cbf - skor_pd
        if lebih_buruk < 1.0e-3:            # selisih numerik sangat kecil (CBF-QP hampir tidak aktif): tidak dihitung
            lebih_buruk = 0.0
        skor = skor_pd + skor_cbf + config.OPTUNA_BOBOT_CBF_BURUK * lebih_buruk
        self.diag_dua = {"gaya_pd_saja_N": gaya_pd, "skor_pd_saja": skor_pd, "skor_pd_cbf": skor_cbf,
                         "cbf_lebih_buruk": lebih_buruk}
        status = STATUS_KOMET if gaya_pd <= 0.0 or gaya_cbf <= 0.0 else STATUS_SELESAI
        detail = "PD saja %s N: %s || PD+CBF %s N: %s" % (teks_gaya(gaya_pd), detail_pd,
                                                           teks_gaya(gaya_cbf), detail_cbf)
        if lebih_buruk > 0.0:
            detail = detail + " || PD+CBF lebih buruk dari PD saja (+%.1f)" % lebih_buruk
        return skor, status, detail

    def uji_beberapa_gaya(self, gain, zeta_deg, daftar, trial=None):
        """Tahap CBFQP_*: pengujian berurutan pada gaya di `daftar` (kecil ke besar), BERHENTI pada gaya pertama
        yang komet. Gaya sesudahnya dilewati, masing-masing dengan penalti penalti_dilewati().
        Skor = jumlah skor gaya yang diuji + penalti gaya yang dilewati.

        Mengembalikan (skor, status, detail). Diagnostik gabungan di self.diag (lihat gabung_diag; gaya_N = gaya
        lolos tertinggi, 0 bila gaya terendah pun komet; penalti dilewati masuk pen_komet), skor tiap gaya yang
        diuji di self.skor_per_gaya. Pruning (bila trial diberikan) memakai skor kumulatif gaya yang sudah diuji.
        Laporan gaya ke-i selalu mulai dari nomor i x (lama dinilai maksimum + 1), jadi nomor laporan yang sama di
        semua trial = gaya yang sama dan detik yang sama sejak dorongan pertama (MedianPruner membandingkan per nomor).
        """
        laporan_per_gaya = int(math.ceil(waktu_dinilai_maks())) + 1
        total = 0.0
        komet = False
        gaya_lolos = 0.0
        dilewati = []
        bagian = []
        semua_diag = []
        self.skor_per_gaya = {}
        for indeks, gaya in enumerate(daftar):
            skor, status, detail, _, diag = self.uji(gain, zeta_deg, gaya, trial,
                                                      laporan_awal=indeks * laporan_per_gaya, skor_awal=total)
            total = total + skor
            self.skor_per_gaya[gaya] = skor
            semua_diag.append(diag)
            if status == STATUS_KOMET:
                komet = True
                bagian.append("%.2f N: komet %s" % (gaya, detail))
                dilewati = list(daftar[indeks + 1:])
                break
            gaya_lolos = gaya
            if detail:
                bagian.append("%.2f N: %s" % (gaya, detail))
        self.diag = gabung_diag(semua_diag)
        self.diag["gaya_N"] = gaya_lolos
        if dilewati:
            penalti = len(dilewati) * penalti_dilewati()
            total = total + penalti
            self.diag["pen_komet"] = self.diag["pen_komet"] + penalti
            bagian.append("dilewati %s N" % teks_daftar_gaya(dilewati))
        return total, STATUS_KOMET if komet else STATUS_SELESAI, "; ".join(bagian)

    def jalankan_trial(self, gain, trial=None):
        """Satu trial: (skor, status, detail). Lantai OPTUNA_ZETA_DEG; tahap "miring" juga S2 dan S3 di bidang
        miring. Diagnostik di self.diag (dan self.diag_dua, self.diag_miring)."""
        if self.dua_mode:
            return self.trial_dua_mode(gain)
        if self.cari_gaya:
            skor, status, detail, hasil_miring = self.nilai_mode(gain, self.pakai_cbf)
            if self.miring:
                self.diag_miring = hasil_miring
            return skor, status, detail
        return self.uji_beberapa_gaya(gain, config.OPTUNA_ZETA_DEG, self.gaya_cbf, trial)


# =============================================================================
# STUDY
# =============================================================================
def buat_study(tahap, profil):
    os.makedirs(FOLDER_HASIL, exist_ok=True)
    # multivariate=True: TPE mempelajari semua variabel BERSAMAAN (pasangan yang saling terkait seperti Kp_psi-Kd_psi,
    # Alpha_1-Alpha_2, Alpha_4-Alpha_5), bukan satu per satu. Optuna menampilkan ExperimentalWarning, itu normal.
    sampler = optuna.samplers.TPESampler(seed=config.OPTUNA_SEED, multivariate=True)
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


class PenjagaStagnan:
    """Auto-exit (early stopping seluruh study) bila skor terbaik tidak membaik selama `stagnant` trial SELESAI
    berturut-turut. Dipakai sebagai callback study.optimize(); memanggil study.stop() saat stagnan.

    Membaik = skor < acuan - ambang * max(1, |acuan|), acuan = skor terbaik saat perbaikan terakhir. Perbaikan yang
    lebih kecil dari ambang tidak mereset hitungan, tetapi tetap terkumpul: begitu totalnya melewati ambang terhadap
    acuan, itu dihitung perbaikan. Trial pruned / gagal tidak dihitung. stagnant <= 0 = mati.
    Riwayat study (trial selesai yang sudah ada di database) dihitung lebih dulu, jadi study yang dilanjutkan tidak
    mulai dari nol."""

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


def cetak_kepala(study, tahap, profil, penguji, definisi, variabel, bawah, atas, kunci, jumlah_trial, penjaga,
                 awal=({}, {})):
    """Ringkasan pengaturan di awal tuning, lalu judul kolom log. awal = mulai_dari_config(): (params trial pertama
    dari config.py, nilai config.py yang di luar batas)."""
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
        if penguji.dua_mode:
            mode = "PD saja DAN " + mode + " (gain sama)"
        jenis_skor = jenis_skor + ", penalti gagal"
    else:
        mode = "PD saja"
    print("Mode: %s | skor: %s, penalti komet" % (mode, jenis_skor))
    print("Dicari (langkah %.4f, batas dari %s):" % (config.OPTUNA_LANGKAH, tabel_batas(profil)[1]))
    for nama in variabel:
        print("    %-9s %10.4f .. %10.4f" % (nama, bawah[nama], atas[nama]))
    params_awal, dilewati = awal
    if params_awal:
        print("Trial pertama = nilai config.py: %s" % ", ".join("%s %.4f" % (k, v) for k, v in params_awal.items()))
    if dilewati:
        print("    (di luar batas, dipilih Optuna: %s)" % ", ".join("%s %.4f" % (k, v) for k, v in dilewati.items()))
    print("Dikunci (dari config.py, kecuali yang dipaksa):")
    if not kunci:
        print("    (tidak ada)")
    for nama in kunci:
        print("    %-9s %10.4f%s" % (nama, kunci[nama], "   (dipaksa tahap ini)" if nama in penguji.paksa else ""))
    if penguji.cari_gaya:
        kisi = daftar_gaya(profil)
        print("Gaya dorong: cari tertinggi yang lolos, bisection %.1f .. %.1f N (langkah %.1f N, %d kisi), "
              "bobot %.0f per N" % (kisi[0], kisi[-1], config.OPTUNA_GAYA_LANGKAH, len(kisi),
                                    config.OPTUNA_BOBOT_GAYA))
    else:
        print("Gaya dorong: %s N (OPTUNA_GAYA_CBF_FRAKSI %s x gaya_pd_terbaik %.2f N), diuji berurutan"
              % (teks_daftar_gaya(penguji.gaya_cbf), list(config.OPTUNA_GAYA_CBF_FRAKSI),
                 float(config.OPTUNA_PROFIL[profil]["gaya_pd_terbaik"])))
        print("Berhenti pada gaya pertama yang komet; tiap gaya yang dilewati diberi penalti %.0f. "
              "Skor = jumlah skor gaya yang diuji + penalti." % penalti_dilewati())
    selesai = len([t for t in study.trials if t.state != optuna.trial.TrialState.WAITING])
    print("Lantai %.1f deg | %d trial (sudah ada %d)" % (config.OPTUNA_ZETA_DEG, jumlah_trial, selesai))
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
    print("gaya = gaya lolos [N] | miring = kemiringan lolos S2 [deg] | ch/s = u berbalik antara >= +%.2f dan "
          "<= -%.2f N m per detik (chattering) | psimaks = puncak |psi| [deg] | gagal = lama status CBF bukan AMAN [s]"
          % (config.OPTUNA_AMBANG_CHATTERING * config.u_max, config.OPTUNA_AMBANG_CHATTERING * config.u_max))
    print("SYARAT LOLOS (config.py): tidak jatuh, ch/s <= %g, overshoot psi <= %g %%, overshoot s <= %g %% "
          "(0 = syarat mati); tidak memenuhi = KOMET"
          % (config.OPTUNA_BATAS_CHATTERING, config.OPTUNA_SYARAT_OVERSHOOT_PSI, config.OPTUNA_SYARAT_OVERSHOOT_S))
    if penguji.miring:
        kisi_miring = daftar_miring()
        print("S1 S2 S3 tiap mode (gain sama):")
        print("    S1 dorongan, lantai %.1f deg: gaya dinaikkan sampai gagal (di atas)" % config.OPTUNA_ZETA_DEG)
        print("    S2 bidang miring tanpa dorongan: bisection %.1f .. %.1f deg (langkah %.1f deg, %d kisi), %.0f s x %d "
              "ulangan; berhasil = tidak jatuh, |s| <= s_max %.2f m, ch/s <= %g"
              % (kisi_miring[0], kisi_miring[-1], config.OPTUNA_MIRING_LANGKAH, len(kisi_miring),
                 config.OPTUNA_MIRING_LAMA, config.OPTUNA_ULANGAN, config.s_max, config.OPTUNA_BATAS_CHATTERING))
        print("    S3 gaya lolos S1 di kemiringan lolos S2 (skenario dan syarat lolos S1)")
        print("    skor mode = skor S1 - %.0f x kemiringan lolos [deg] - %.0f bila S3 lolos"
              % (config.OPTUNA_BOBOT_MIRING, config.OPTUNA_BOBOT_S3))
    if penguji.dua_mode:
        print("DUA MODE: skor = skor PD saja + skor PD+CBF + %g x max(0, skor PD+CBF - skor PD saja)"
              % config.OPTUNA_BOBOT_CBF_BURUK)
    print("Berhenti otomatis: %s." % penjaga.teks())
    print("Ctrl+C untuk berhenti: trial yang sudah selesai tetap tersimpan dan bisa dilanjutkan.")
    print(garis)
    print(FORMAT_HEADER % JUDUL_KOLOM)
    print("-" * len(garis))


def mulai_dari_config(study, variabel, bawah, atas):
    """Study BARU (belum ada trial): trial pertama = nilai config.py, supaya
    pencarian dimulai dari titik yang sudah diketahui. Nilai di luar batas pencarian tidak diisi (dipilih Optuna).
    Mengembalikan (params yang dimasukkan, dict nama -> nilai config.py yang di luar batas); keduanya kosong bila
    study sudah berisi trial atau OPTUNA_MULAI_DARI_CONFIG = False."""
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
        # dibulatkan ke kisi Optuna (bawah + k x OPTUNA_LANGKAH), tidak melewati batas atas
        k_maks = math.floor((atas[nama] - bawah[nama]) / langkah_cari + 1e-9)
        k = min(round((nilai - bawah[nama]) / langkah_cari), k_maks)
        params[nama] = round(bawah[nama] + k * langkah_cari, 10)
    if params:
        study.enqueue_trial(params)
    return params, dilewati


def jalankan_tahap(tahap, profil, jumlah_trial, stagnant=None):
    """stagnant: batas stagnan (trial); None = kunci "stagnant" tahap di config.OPTUNA_TAHAP, atau
    config.OPTUNA_STAGNANT."""
    definisi = config.OPTUNA_TAHAP[tahap]
    variabel = definisi["variabel"]
    bawah, atas = batas_pencarian(variabel, profil)

    penguji = Penguji(profil, pakai_cbf=definisi["barrier"] is not None)
    penguji.atur_tahap(tahap)
    if penguji.cari_gaya:
        daftar_gaya(profil)                                 # berhenti dengan pesan bila batas gaya salah
    elif penguji.gaya_cbf is None:
        sys.exit("Tahap %s butuh gaya dorong dari hasil tahap PD. Isi OPTUNA_PROFIL['%s']['gaya_pd_terbaik'] di "
                 "config.py (dicetak di akhir tahap PD, atau: python3 tuning_optuna.py --hasil --profil %s)."
                 % (tahap, profil, profil))
    if penguji.miring:
        daftar_miring()                                     # berhenti dengan pesan bila kisi kemiringan salah
    kunci = penguji.terkunci()
    for nama in kunci:
        if nama in config.NAMA_ALPHA and kunci[nama] <= 0.0:
            sys.exit("%s dikunci dari config.py tetapi nilainya %.4f (harus > 0). Isi hasil tahap sebelumnya ke config.py."
                     % (nama, kunci[nama]))
    study = buat_study(tahap, profil)
    periksa_study(study, penguji)
    if stagnant is None:
        stagnant = definisi.get("stagnant", config.OPTUNA_STAGNANT)
    penjaga = PenjagaStagnan(study, stagnant, config.OPTUNA_STAGNANT_MIN)
    awal = mulai_dari_config(study, variabel, bawah, atas)
    cetak_kepala(study, tahap, profil, penguji, definisi, variabel, bawah, atas, kunci, jumlah_trial, penjaga, awal)

    def simpan_atribut(trial, keterangan, t0):
        trial.set_user_attr("keterangan", keterangan)
        trial.set_user_attr("lama_s", time.perf_counter() - t0)
        for nama in KOLOM_DIAG:
            trial.set_user_attr(nama, penguji.diag[nama])
        if penguji.dua_mode:
            for nama in KOLOM_DUA:
                trial.set_user_attr(nama, penguji.diag_dua[nama])
        if penguji.miring:
            for nama, nilai in penguji.diag_miring.items():
                trial.set_user_attr(nama, nilai)
        if not penguji.cari_gaya and keterangan != "pruned":
            for gaya, skor in penguji.skor_per_gaya.items():
                trial.set_user_attr(nama_kolom_gaya(gaya), skor)

    def objektif(trial):
        params = {}
        for nama in variabel:
            params[nama] = trial.suggest_float(nama, bawah[nama], atas[nama], step=config.OPTUNA_LANGKAH)
        t0 = time.perf_counter()
        penguji.diag_miring = {}
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
                              teks_gaya(atribut.get("gaya_N", 0.0)), teks_miring(atribut.get("miring_deg")),
                              atribut.get("chattering_per_s", 0.0), atribut.get("psi_maks_deg", 0.0), teks_gagal,
                              detail))

    optuna.logging.set_verbosity(optuna.logging.WARNING)
    if penjaga.stagnan():
        # Study yang dilanjutkan sudah stagnan sejak sebelumnya: tidak ada trial baru (tambah --stagnant untuk memaksa)
        print("STAGNAN sejak sebelumnya: %d trial selesai terakhir tanpa perbaikan > %g %% (batas %d). Tidak ada trial "
              "baru; untuk tetap melanjutkan pakai --stagnant dengan angka lebih besar, atau --stagnant 0."
              % (penjaga.tanpa_perbaikan, 100.0 * penjaga.ambang, penjaga.stagnant))
    else:
        try:
            study.optimize(objektif, n_trials=jumlah_trial, callbacks=[cetak_trial, penjaga])
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
        sumber = "study" if nama.startswith("kunci ") or nama == "gaya_dorong" else "sekarang"
        print("PERHATIAN: %s berbeda dari saat study dibuat (study %s, sekarang %s); uji ulang di bawah memakai "
              "nilai %s." % (nama, nilai_lama, nilai_baru, sumber))
    gain = {}
    for nama in skenario:
        if nama.startswith("kunci "):
            gain[nama[len("kunci "):]] = skenario[nama]
    gain.update(terbaik.params)

    # Gaya untuk uji ulang: gaya lolos trial terbaik (tahap PD), atau semua gaya tahap CBFQP_* (dari study).
    if penguji.cari_gaya:
        gaya_lolos = float(terbaik.user_attrs.get("gaya_N", 0.0))
        daftar_uji = [gaya_lolos if gaya_lolos > 0.0 else daftar_gaya(profil)[0]]
    else:
        daftar_uji = skenario.get("gaya_dorong") or penguji.gaya_cbf
        if daftar_uji is not None and not isinstance(daftar_uji, list):
            daftar_uji = [float(daftar_uji)]                  # study lama: satu gaya
    if daftar_uji is None:
        print("Tahap %s profil %s: gaya dorong tidak diketahui (isi gaya_pd_terbaik di config.py)." % (tahap, profil))
        return
    daftar_uji = sorted(daftar_uji)

    selesai, pruned = jumlah_per_status(study)

    # Gain terbaik diuji lagi di lantai optimasi: berurutan dari gaya terkecil, berhenti pada gaya pertama yang komet
    # (sama dengan saat tuning). Gaya lolos tertinggi (atau gaya terendah bila semua komet) juga diuji di lantai uji;
    # keduanya direkam untuk PNG.
    hasil_gaya = []
    rekaman_pilihan = None
    for gaya in daftar_uji:
        skor, status, detail, rekaman, diag = penguji.uji(gain, config.OPTUNA_ZETA_DEG, gaya, rekam=True)
        hasil_gaya.append((gaya, skor, ("%s %s" % (status, detail)).strip(), rekaman, diag))
        if status == STATUS_KOMET:
            break
        rekaman_pilihan = hasil_gaya[-1]
    if rekaman_pilihan is None:
        rekaman_pilihan = hasil_gaya[0]
    gaya_uji = rekaman_pilihan[0]
    dilewati = daftar_uji[len(hasil_gaya):]
    hasil_uji = [(config.OPTUNA_ZETA_DEG,) + rekaman_pilihan[1:]]
    # Lantai miring: tahap "miring" = S3 (kemiringan lolos S2 trial terbaik), lainnya OPTUNA_ZETA_UJI_DEG.
    miring_terbaik = float(terbaik.user_attrs.get("miring_deg", 0.0)) if penguji.miring else 0.0
    zeta_miring = miring_terbaik if miring_terbaik > 0.0 else config.OPTUNA_ZETA_UJI_DEG
    teks_miring_uji = ("S3: kemiringan lolos S2 trial terbaik" if miring_terbaik > 0.0
                       else "tidak ikut dioptimasi")
    skor, status, detail, rekaman, diag = penguji.uji(gain, zeta_miring, gaya_uji, rekam=True)
    hasil_uji.append((zeta_miring, skor, ("%s %s" % (status, detail)).strip(), rekaman, diag))

    print("\n================ HASIL TAHAP %s | %s ================" % (tahap, study.study_name))
    print(teks_label(profil))
    print("trial: %d total, %d selesai, %d pruned" % (len(study.trials), selesai, pruned))
    print("trial terbaik: %d, skor %.4f (%s)" % (terbaik.number, terbaik.value,
                                                terbaik.user_attrs.get("keterangan", "-")))
    print("uji ulang lantai %.1f deg:" % config.OPTUNA_ZETA_DEG)
    for gaya, skor, keterangan, _, diag in hasil_gaya:
        print("  gaya %6.2f N: skor %10.4f | psimaks %5.1f deg | ch/s %5.1f | gagal %4.1f s | %s"
              % (gaya, skor, diag["psi_maks_deg"], diag["chattering_per_s"], diag["lama_gagal_s"], keterangan))
    for gaya in dilewati:
        print("  gaya %6.2f N: dilewati (komet pada gaya sebelumnya), penalti %.0f" % (gaya, penalti_dilewati()))
    if not penguji.cari_gaya:
        lolos = [h[0] for h in hasil_gaya if not h[2].startswith(STATUS_KOMET)]
        print("  gaya lolos tertinggi: %s" % ("%.2f N" % max(lolos) if lolos else "- (gaya terendah pun komet)"))
    zeta_deg, skor, keterangan, _, diag = hasil_uji[1]
    print("uji ulang lantai %.1f deg (%s):" % (zeta_deg, teks_miring_uji))
    print("  gaya %6.2f N: skor %10.4f | psimaks %5.1f deg | ch/s %5.1f | gagal %4.1f s | %s"
          % (gaya_uji, skor, diag["psi_maks_deg"], diag["chattering_per_s"], diag["lama_gagal_s"], keterangan))
    if penguji.dua_mode:
        a = terbaik.user_attrs
        print("dua mode (trial terbaik, gain sama): PD saja lolos %s N (skor %.3f) | PD + CBF-QP lolos %s N (skor %.3f)"
              % (teks_gaya(a.get("gaya_pd_saja_N", 0.0)), a.get("skor_pd_saja", 0.0), teks_gaya(a.get("gaya_N", 0.0)),
                 a.get("skor_pd_cbf", 0.0)))
        if a.get("cbf_lebih_buruk", 0.0) > 0.0:
            print("  PERHATIAN: PD + CBF-QP lebih buruk dari PD saja (selisih skor %.3f)" % a["cbf_lebih_buruk"])
    if penguji.miring:
        a = terbaik.user_attrs
        teks_s3 = {1.0: "lolos", 0.0: "gagal / tidak dijalankan"}
        mode_miring = [("PD + CBF-QP" if penguji.pakai_cbf else "PD saja", "", "_m")]
        if penguji.dua_mode:
            mode_miring = [("PD saja", "_pd_saja", "_pd_saja_m"), ("PD + CBF-QP", "", "_m")]
        for nama_mode, akhiran, akhiran_s in mode_miring:
            print("S2 / S3 %s (trial terbaik): kemiringan lolos %s deg (|s| maks %.2f m) | S3 %s"
                  % (nama_mode, teks_miring(a.get("miring" + akhiran + "_deg")),
                     a.get("miring_s_maks" + akhiran_s, 0.0), teks_s3.get(a.get("s3" + akhiran + "_lolos"), "-")))
    print("\nSalin ke config.py:")
    for nama in terbaik.params:
        print("%s = %.4f" % (nama, terbaik.params[nama]))
    if penguji.cari_gaya and penguji.pakai_cbf:
        # tahap SEMUA (gabungan): bukan pengganti otomatis hasil bertahap, bandingkan dulu
        print("# tahap %s: gaya lolos tertinggi %.1f N. Bandingkan dengan hasil bertahap sebelum disalin:"
              % (tahap, gaya_lolos))
        print("#     python3 tuning_optuna.py --profil %s --banding" % profil)
    elif penguji.cari_gaya and gaya_lolos > 0.0:
        print("OPTUNA_PROFIL['%s']['gaya_pd_terbaik'] = %.1f      # tahap CBFQP_* memakai %s N"
              % (profil, gaya_lolos, teks_daftar_gaya(sorted(set(round(f * gaya_lolos, 2)
                                                                  for f in config.OPTUNA_GAYA_CBF_FRAKSI)))))
    elif penguji.cari_gaya:
        print("# gaya_pd_terbaik belum bisa diisi: trial terbaik pun komet pada gaya terendah %.1f N"
              % daftar_gaya(profil)[0])
    for nama in penguji.paksa:
        print("# selama tahap ini %s dipaksa %.4f (bukan nilai config.py)" % (nama, penguji.paksa[nama]))
    print("=================================================\n")

    awalan = os.path.join(FOLDER_HASIL, datetime.now().strftime("%Y%m%d_%H%M%S") + "_" + nama_file(tahap, profil))
    simpan_csv(awalan + ".csv", study, profil)
    simpan_png(awalan + ".png", tahap, profil, study, terbaik.params, hasil_uji, gaya_uji, penguji.cari_gaya)


def simpan_csv(path, study, profil):
    nama_param = []
    kolom_gaya = set()
    for t in study.trials:
        for nama in t.params:
            if nama not in nama_param:
                nama_param.append(nama)
        kolom_gaya.update(k for k in t.user_attrs if k.startswith("skor_") and k.endswith("N"))
    kolom_gaya = sorted(kolom_gaya, key=lambda k: float(k[len("skor_"):-1]))
    # tahap SEMUA dua mode: hasil PD saja dan PD + CBF-QP (atribut trial)
    kolom_dua = [nama for nama in KOLOM_DUA if any(nama in t.user_attrs for t in study.trials)]
    # tahap "miring": hasil S2 dan S3 (atribut trial)
    kolom_dua = kolom_dua + [nama for nama in KOLOM_MIRING if any(nama in t.user_attrs for t in study.trials)]
    with open(path, "w", newline="") as f:
        penulis = csv.writer(f)
        penulis.writerow(["trial", "status", "skor", "keterangan", "lama_s", "profil", "terverifikasi"]
                         + nama_param + kolom_dua + KOLOM_DIAG + kolom_gaya)
        for t in study.trials:
            skor = "%.6f" % t.value if t.value is not None else ""
            baris = [t.number, t.state.name, skor, t.user_attrs.get("keterangan", ""),
                     "%.1f" % t.user_attrs.get("lama_s", 0.0), profil,
                     "YA" if config.OPTUNA_PROFIL[profil]["terverifikasi"] else "BELUM"]
            for nama in nama_param:
                baris.append("%.4f" % t.params[nama] if nama in t.params else "")
            for nama in kolom_dua + KOLOM_DIAG + kolom_gaya:
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
        # tahap "miring": kemiringan lolos S2 tiap trial di sumbu kanan
        trial_miring = [t for t in study.trials
                        if t.state == optuna.trial.TrialState.COMPLETE and "miring_deg" in t.user_attrs]
        if trial_miring:
            kanan = ax[0].twinx()
            kanan.plot([t.number for t in trial_miring], [t.user_attrs["miring_deg"] for t in trial_miring], "x", ms=4,
                       color="tab:green", label="kemiringan lolos S2")
            kanan.set_ylabel("kemiringan lolos S2 [deg]", color="tab:green")
            kanan.legend(fontsize=8, loc="lower right")
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


# =============================================================================
# BANDING: hasil tuning BERTAHAP vs SEMUA pada skenario yang sama
# =============================================================================
def params_terbaik(tahap, profil):
    """Parameter trial terbaik study satu tahap (dict), atau None bila study / trial selesai belum ada."""
    if not ada_study(tahap, profil):
        return None
    terbaik = trial_terbaik(buat_study(tahap, profil))
    return dict(terbaik.params) if terbaik is not None else None


def gain_bertahap(profil):
    """Gain hasil tuning BERTAHAP: tiap variabel dari trial terbaik study tahap yang menuningnya (PD, CBFQP_*);
    bila study tahap itu belum ada, nilai config.py. Mengembalikan (gain, sumber): dict nama -> nilai / asal nilai."""
    gain = {nama: float(getattr(config, nama)) for nama in semua_variabel()}
    sumber = dict.fromkeys(gain, "config.py")
    for tahap, definisi in config.OPTUNA_TAHAP.items():
        if tahap == TAHAP_GABUNGAN:
            continue
        params = params_terbaik(tahap, profil) or {}
        for nama in definisi["variabel"]:
            if nama in params:
                gain[nama] = float(params[nama])
                sumber[nama] = "study " + tahap
    return gain, sumber


def gaya_banding(profil, gaya_tertinggi, jumlah=5):
    """Gaya dorong uji per-gaya --banding [N]: `jumlah` titik kisi tahap PD (daftar_gaya) dari gaya terendah sampai
    gaya lolos tertinggi di antara semua set (tanpa gaya_pd_terbaik)."""
    kisi = daftar_gaya(profil)
    if gaya_tertinggi <= kisi[0]:
        return [kisi[0]]
    target = [kisi[0] + i * (gaya_tertinggi - kisi[0]) / (jumlah - 1) for i in range(jumlah)]
    return sorted(set(min(kisi, key=lambda g, t=t: abs(g - t)) for t in target))


def jalankan_banding(profil):
    """Uji hasil tuning BERTAHAP dan SEMUA dengan penilaian yang sama persis (PD + CBF-QP h1 .. h8):
        1. skor PD + CBF-QP (objektif tahap SEMUA tanpa bagian PD saja) dan gaya lolos tertinggi
        2. gaya lolos tertinggi PD SAJA dengan gain yang sama (untuk perbandingan PD vs PD + CBF-QP)
        3. skor tiap gaya gaya_banding() (semua gaya diuji, tidak berhenti saat komet)
        4. respon psi dan s pada gaya terbesar yang lolos untuk semua set, di lantai OPTUNA_ZETA_DEG, dan uji
           di lantai miring OPTUNA_ZETA_UJI_DEG (tidak ikut dioptimasi)
    Hasil di terminal, CSV, dan PNG (folder OPTUNA)."""
    bertahap, sumber = gain_bertahap(profil)
    kumpulan = {"BERTAHAP": bertahap}
    keterangan = {"BERTAHAP": "per variabel: study tahap PD / CBFQP_*, atau config.py"}
    params_semua = params_terbaik(TAHAP_GABUNGAN, profil)
    if params_semua is None:
        perintah = "python3 tuning_optuna.py --profil %s --tahap %s" % (profil, TAHAP_GABUNGAN)
        sys.exit("Belum ada hasil tahap %s profil %s. Jalankan dulu:\n    %s" % (TAHAP_GABUNGAN, profil, perintah))
    gain = dict(bertahap)
    gain.update({nama: float(nilai) for nama, nilai in params_semua.items()})
    kumpulan["SEMUA"] = gain
    keterangan["SEMUA"] = "trial terbaik study %s" % nama_study(TAHAP_GABUNGAN, profil)
    label = list(kumpulan)

    penguji = Penguji(profil, pakai_cbf=True)
    penguji.atur_tahap(TAHAP_GABUNGAN)
    zeta = config.OPTUNA_ZETA_DEG
    zeta_uji = config.OPTUNA_ZETA_UJI_DEG

    print("\n================ BANDING %s | %s ================" % (profil, " vs ".join(label)))
    print(teks_label(profil))
    print("Penilaian sama untuk semua: PD + CBF-QP h1 .. h8, skenario dan syarat lolos config.py bagian 13.")
    for k in label:
        print("    %-9s %s" % (k, keterangan[k]))
    print("\n%-10s" % "variabel" + "".join("%12s" % k for k in label) + "   asal nilai BERTAHAP")
    for nama in semua_variabel():
        print("%-10s" % nama + "".join("%12.4f" % kumpulan[k][nama] for k in label) + "   " + sumber[nama])
    dari_config = [nama for nama in semua_variabel() if sumber[nama] == "config.py"]
    if dari_config:
        print("PERHATIAN: %s belum punya study bertahap, BERTAHAP memakai nilai config.py." % ", ".join(dari_config))

    hasil = {}
    for k in label:
        print("Menguji %s ..." % k)
        skor_cbf, _, _ = penguji.cari_gaya_mode(kumpulan[k], zeta, pakai_cbf=True)
        gaya_cbf = penguji.diag["gaya_N"]
        _, _, _ = penguji.cari_gaya_mode(kumpulan[k], zeta, pakai_cbf=False)
        hasil[k] = {"skor_semua": skor_cbf, "gaya_lolos": gaya_cbf, "gaya_pd_saja": penguji.diag["gaya_N"]}
    daftar = gaya_banding(profil, max(hasil[k]["gaya_lolos"] for k in label))
    for k in label:
        per_gaya = []
        for gaya in daftar:
            skor, status, detail, _, diag = penguji.uji(kumpulan[k], zeta, gaya)
            per_gaya.append({"skor": skor, "status": status, "detail": detail, "diag": diag})
        hasil[k]["per_gaya"] = per_gaya

    # Respon pada gaya terbesar yang lolos untuk semua set (gaya terkecil bila tidak ada), lantai datar dan miring.
    lolos_bersama = [gaya for i, gaya in enumerate(daftar)
                     if all(hasil[k]["per_gaya"][i]["status"] == STATUS_SELESAI for k in label)]
    gaya_respon = max(lolos_bersama) if lolos_bersama else daftar[0]
    for k in label:
        hasil[k]["rekaman"] = penguji.uji(kumpulan[k], zeta, gaya_respon, rekam=True)[3]
        skor, status, detail, _, diag = penguji.uji(kumpulan[k], zeta_uji, gaya_respon)
        hasil[k]["miring"] = {"skor": skor, "status": status, "detail": detail, "diag": diag}

    def baris(judul, isi):
        print("%-32s" % judul + "".join(" | %-26s" % teks for teks in isi))

    def teks_uji(r):
        return "%-7s %8.2f %5.1f deg" % (r["status"], r["skor"], r["diag"]["psi_maks_deg"])

    print()
    baris("", label)
    baris("skor PD + CBF-QP", ["%.3f" % hasil[k]["skor_semua"] for k in label])
    baris("gaya lolos PD + CBF-QP [N]", [teks_gaya(hasil[k]["gaya_lolos"]) for k in label])
    baris("gaya lolos PD saja [N]", [teks_gaya(hasil[k]["gaya_pd_saja"]) for k in label])
    baris("lantai %.1f deg, gaya:" % zeta, ["status      skor  psimaks"] * len(label))
    for i, gaya in enumerate(daftar):
        baris("  %.2f N" % gaya, [teks_uji(hasil[k]["per_gaya"][i]) for k in label])
    terakhir = [hasil[k]["per_gaya"][-1]["diag"] for k in label]
    baris("  ch/s pada %.2f N" % daftar[-1], ["%.2f" % d["chattering_per_s"] for d in terakhir])
    baris("  overshoot psi / s pada %.2f N" % daftar[-1],
          ["%.1f %% / %.1f %%" % (d["overshoot_psi_persen"], d["overshoot_s_persen"]) for d in terakhir])
    baris("lantai %.1f deg, %.2f N" % (zeta_uji, gaya_respon), [teks_uji(hasil[k]["miring"]) for k in label])
    terbaik = min(label, key=lambda k: hasil[k]["skor_semua"])
    print("Skor makin kecil makin baik. Terbaik menurut skor PD + CBF-QP (gaya lolos, lalu ISE): %s" % terbaik)
    for k in label:
        semua_uji = [(zeta, gaya, hasil[k]["per_gaya"][i]) for i, gaya in enumerate(daftar)]
        semua_uji.append((zeta_uji, gaya_respon, hasil[k]["miring"]))
        for zeta_deg, gaya, r in semua_uji:
            if r["status"] == STATUS_KOMET:
                print("  %s lantai %.1f deg, %.2f N: %s" % (k, zeta_deg, gaya, r["detail"]))

    awalan = os.path.join(FOLDER_HASIL, datetime.now().strftime("%Y%m%d_%H%M%S") + "_OPTUNA_BANDING_" + profil)
    simpan_csv_banding(awalan + ".csv", kumpulan, sumber, hasil, daftar, gaya_respon)
    simpan_png_banding(awalan + ".png", profil, kumpulan, hasil, daftar, gaya_respon)


def simpan_csv_banding(path, kumpulan, sumber, hasil, daftar, gaya_respon):
    """CSV --banding: satu baris per nilai, satu kolom per set (BERTAHAP, SEMUA) berdampingan."""
    label = list(kumpulan)

    def tulis_uji(penulis, bagian, daftar_hasil):
        penulis.writerow([bagian, "status"] + [r["status"] for r in daftar_hasil] + [""])
        penulis.writerow([bagian, "skor"] + ["%.4f" % r["skor"] for r in daftar_hasil] + [""])
        for nama in KOLOM_DIAG:
            penulis.writerow([bagian, nama] + ["%.4f" % r["diag"][nama] for r in daftar_hasil] + [""])

    os.makedirs(FOLDER_HASIL, exist_ok=True)
    with open(path, "w", newline="") as f:
        penulis = csv.writer(f)
        penulis.writerow(["bagian", "nama"] + label + ["keterangan"])
        for nama in semua_variabel():
            penulis.writerow(["gain", nama] + ["%.4f" % kumpulan[k][nama] for k in label]
                             + ["BERTAHAP dari " + sumber[nama]])
        penulis.writerow(["PD + CBF-QP", "skor"] + ["%.4f" % hasil[k]["skor_semua"] for k in label]
                         + ["skor PD + CBF-QP seperti tahap SEMUA (makin kecil makin baik)"])
        penulis.writerow(["PD + CBF-QP", "gaya_lolos_N"] + ["%.2f" % hasil[k]["gaya_lolos"] for k in label]
                         + ["gaya dorong lolos tertinggi"])
        penulis.writerow(["PD saja", "gaya_lolos_N"] + ["%.2f" % hasil[k]["gaya_pd_saja"] for k in label]
                         + ["gaya dorong lolos tertinggi PD saja, gain sama"])
        for i, gaya in enumerate(daftar):
            tulis_uji(penulis, "lantai %.1f deg gaya %.2f N" % (config.OPTUNA_ZETA_DEG, gaya),
                      [hasil[k]["per_gaya"][i] for k in label])
        tulis_uji(penulis, "lantai %.1f deg gaya %.2f N" % (config.OPTUNA_ZETA_UJI_DEG, gaya_respon),
                  [hasil[k]["miring"] for k in label])
    print("Tersimpan:", path)


def simpan_png_banding(path, profil, kumpulan, hasil, daftar, gaya_respon):
    """PNG --banding: gaya lolos tertinggi (PD saja dan PD + CBF-QP), skor tiap gaya, respon psi dan s."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib tidak terpasang -> PNG tidak dibuat")
        return
    warna = {"BERTAHAP": "tab:purple", "SEMUA": "tab:blue"}
    penanda = {"BERTAHAP": "o", "SEMUA": "s"}
    label = list(kumpulan)
    fig, ax = plt.subplots(2, 2, figsize=(13, 9))
    fig.suptitle("BANDING tuning %s | %s\npenilaian sama: PD + CBF-QP h1 .. h8, lantai %.1f deg"
                 % (" vs ".join(label), teks_label(profil), config.OPTUNA_ZETA_DEG), fontsize=11)

    # 1. Gaya lolos tertinggi: PD saja (pudar, garis miring) dan PD + CBF-QP, gain sama
    lebar = 0.38
    for i, k in enumerate(label):
        for geser, kunci, nama_mode, pola in ((-lebar / 2, "gaya_pd_saja", "PD saja", "//"),
                                              (lebar / 2, "gaya_lolos", "PD + CBF-QP", "")):
            nilai = hasil[k][kunci]
            ax[0, 0].bar(i + geser, nilai, width=lebar, color=warna[k], alpha=0.45 if pola else 1.0, hatch=pola,
                         edgecolor="white")
            ax[0, 0].text(i + geser, nilai, "%.1f N\n%s" % (nilai, nama_mode), ha="center", va="bottom", fontsize=8)
    ax[0, 0].set_xticks(range(len(label)))
    ax[0, 0].set_xticklabels(label)
    ax[0, 0].set_ylim(0.0, 1.3 * max([1.0] + [max(hasil[k]["gaya_lolos"], hasil[k]["gaya_pd_saja"]) for k in label]))
    ax[0, 0].set_ylabel("gaya dorong lolos tertinggi [N]")
    ax[0, 0].set_title("Gaya lolos tertinggi: PD saja vs PD + CBF-QP (gain sama)", fontsize=10)
    ax[0, 0].grid(True, axis="y", color="0.88")

    # 2. Skor tiap gaya (x merah = komet)
    for k in label:
        ax[0, 1].plot(daftar, [r["skor"] for r in hasil[k]["per_gaya"]], "-", marker=penanda[k], color=warna[k],
                      label=k)
        for gaya, r in zip(daftar, hasil[k]["per_gaya"]):
            if r["status"] != STATUS_SELESAI:
                ax[0, 1].plot([gaya], [r["skor"]], "x", color="red", ms=10)
    if min(r["skor"] for k in label for r in hasil[k]["per_gaya"]) > 0.0:
        ax[0, 1].set_yscale("log")
    ax[0, 1].set_xlabel("gaya dorong [N]")
    ax[0, 1].set_ylabel("skor pengujian PD + CBF-QP (makin kecil makin baik)")
    ax[0, 1].set_title("Skor tiap gaya (x merah = komet)", fontsize=10)
    ax[0, 1].grid(True, color="0.88")
    ax[0, 1].legend(fontsize=8)

    # 3 dan 4. Respon psi dan s (garis putus-putus = ulangan berikutnya)
    for k in label:
        for data in hasil[k]["rekaman"]:
            garis = "-" if data["ulangan"] == 0 else "--"
            nama = "%s U%d" % (k, data["ulangan"] + 1)
            ax[1, 0].plot(data["t"], data["psi"], garis, color=warna[k], lw=1, label=nama)
            ax[1, 1].plot(data["t"], data["s"], garis, color=warna[k], lw=1, label=nama)
    batas_psi = math.degrees(config.psi_max)
    ax[1, 0].axhline(batas_psi, color="red", lw=2)
    ax[1, 0].axhline(-batas_psi, color="red", lw=2)
    ax[1, 1].axhline(config.s_max, color="red", lw=2)
    ax[1, 1].axhline(config.s_min, color="red", lw=2)
    for a in (ax[1, 0], ax[1, 1]):
        a.set_xlabel("t sejak dorongan pertama tiap ulangan [s]")
        a.grid(True, color="0.88")
        a.legend(fontsize=7)
    ax[1, 0].set_ylabel("psi Kalman [deg]")
    ax[1, 1].set_ylabel("s [m]")
    ax[1, 0].set_title("Respon psi PD + CBF-QP pada gaya %.2f N (terbesar yang lolos untuk semua)" % gaya_respon,
                       fontsize=10)
    ax[1, 1].set_title("Respon s PD + CBF-QP pada gaya %.2f N" % gaya_respon, fontsize=10)
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


# Nilai config.py yang dibutuhkan program ini dan baru ditambahkan belakangan: nama -> baris contoh.
CONFIG_WAJIB = {
    "OPTUNA_AMBANG_CHATTERING": "OPTUNA_AMBANG_CHATTERING = 0.5    # fraksi u_max: |u| di atas ini = torsi besar",
    "OPTUNA_GAYA_CBF_FRAKSI": "OPTUNA_GAYA_CBF_FRAKSI = [0.5, 0.75, 1.0]   # tahap CBFQP_*: gaya = fraksi x gaya_pd_terbaik",
    "OPTUNA_STAGNANT": "OPTUNA_STAGNANT = 300           # auto-exit: trial selesai tanpa perbaikan (0 = mati)",
    "OPTUNA_STAGNANT_MIN": "OPTUNA_STAGNANT_MIN = 1.0e-3    # perbaikan minimum (relatif) yang dihitung",
    "OPTUNA_BATAS_CHATTERING": "OPTUNA_BATAS_CHATTERING = 2.0     # ch/s di atas ini = KOMET (0 = mati)",
    "OPTUNA_MULAI_DARI_CONFIG": "OPTUNA_MULAI_DARI_CONFIG = True  # study baru: trial pertama = nilai config.py",
    "OPTUNA_SYARAT_OVERSHOOT_PSI": "OPTUNA_SYARAT_OVERSHOOT_PSI = 10.0   # [%] syarat lolos: overshoot psi maksimum",
    "OPTUNA_SYARAT_OVERSHOOT_S": "OPTUNA_SYARAT_OVERSHOOT_S = 5.0       # [%] syarat lolos: overshoot s maksimum",
    "OPTUNA_OVERSHOOT_ACUAN_S": "OPTUNA_OVERSHOOT_ACUAN_S = 1.0        # [s] acuan = rata-rata sebelum dorongan",
    "OPTUNA_OVERSHOOT_RATA_S": "OPTUNA_OVERSHOOT_RATA_S = 0.05        # [s] perataan sinyal sebelum dihitung",
    "OPTUNA_OVERSHOOT_PUNCAK_PSI_DEG": "OPTUNA_OVERSHOOT_PUNCAK_PSI_DEG = 0.5  # [deg] puncak lebih kecil: abaikan",
    "OPTUNA_OVERSHOOT_PUNCAK_S_M": "OPTUNA_OVERSHOOT_PUNCAK_S_M = 0.01     # [m] puncak lebih kecil: tidak dinilai",
    "OPTUNA_BOBOT_CBF_BURUK": "OPTUNA_BOBOT_CBF_BURUK = 1.0          # tahap SEMUA: bobot penalti PD+CBF lebih buruk",
    "OPTUNA_MIRING_LANGKAH": "OPTUNA_MIRING_LANGKAH = 0.1   # [deg] S2: kenaikan kemiringan tiap berhasil",
    "OPTUNA_MIRING_MAKS": "OPTUNA_MIRING_MAKS = 30.0      # [deg] S2: kemiringan tertinggi yang dicoba",
    "OPTUNA_MIRING_LAMA": "OPTUNA_MIRING_LAMA = 15.0      # [s] S2: lama robot di bidang miring tiap ulangan",
    "OPTUNA_BOBOT_MIRING": "OPTUNA_BOBOT_MIRING = 5000.0   # [per deg] S2: bobot kemiringan lolos di skor",
    "OPTUNA_BOBOT_S3": "OPTUNA_BOBOT_S3 = 5000.0       # S3: bonus skor bila lolos",
    "CBF_TAHU_ZETA": "CBF_TAHU_ZETA = False          # False = seperti robot: CBF tidak tahu kemiringan, DOB menaksir",
}


def periksa_config():
    """Berhenti dengan pesan jelas bila config.py belum memuat nilai yang dibutuhkan (config.py versi lama)."""
    kurang = [nama for nama in CONFIG_WAJIB if not hasattr(config, nama)]
    if kurang:
        print("config.py belum memuat nilai berikut. Tambahkan di bagian 13 (TUNING OPTUNA):")
        for nama in kurang:
            print("    " + CONFIG_WAJIB[nama])
        sys.exit(1)


def main():
    daftar_profil = list(config.OPTUNA_PROFIL.keys())
    daftar_tahap = list(config.OPTUNA_TAHAP.keys())
    parser = argparse.ArgumentParser(description="Tuning bertahap gain PD dan Alpha CBF-QP ATERA dengan Optuna")
    parser.add_argument("--profil", choices=daftar_profil,
                        help="profil torsi roda (config.OPTUNA_PROFIL); wajib untuk --tahap, --hapus, --banding")
    parser.add_argument("--tahap", choices=daftar_tahap,
                        help="tahap yang dijalankan (config.OPTUNA_TAHAP); nilai tahap lain dibaca dari config.py")
    parser.add_argument("--trial", type=int, default=None, help="jumlah trial (default dari config.OPTUNA_TAHAP)")
    parser.add_argument("--stagnant", type=int, default=None,
                        help="auto-exit bila sekian trial selesai tanpa perbaikan "
                             "(default config.OPTUNA_STAGNANT, 0 = mati)")
    parser.add_argument("--hasil", action="store_true",
                        help="cetak hasil terbaik, buat ulang CSV + PNG (tanpa --profil: semua profil)")
    parser.add_argument("--hapus", choices=daftar_tahap, help="hapus study satu tahap dari satu profil")
    parser.add_argument("--banding", action="store_true",
                        help="bandingkan hasil tuning BERTAHAP vs %s, skenario sama" % TAHAP_GABUNGAN)
    args = parser.parse_args()
    periksa_config()

    if args.hapus:
        if args.profil is None:
            parser.error("--hapus butuh --profil")
        hapus_tahap(args.hapus, args.profil)
    elif args.banding:
        if args.profil is None:
            parser.error("--banding butuh --profil (%s)" % " / ".join(daftar_profil))
        jalankan_banding(args.profil)
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
        jalankan_tahap(args.tahap, args.profil, jumlah, args.stagnant)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()