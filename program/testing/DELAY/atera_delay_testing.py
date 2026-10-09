#!/usr/bin/env python3
"""
atera_delay_testing.py - mengukur jeda (delay) hardware ATERA, untuk dimasukkan ke simulasi.

Ada 3 uji dan 1 mode analisis:

  motor    : jeda motor DDSM115. RODA HARUS TERANGKAT. Tiap run:
                 diam (arus 0) -> naik (arus I) -> lepas (arus 0)
             yang diukur:
               - waktu komunikasi RS485 (perintah dikirim -> balasan diterima)
               - respon arus umpan balik (mencapai 20% dan 63%, lalu turun ke 37%)
               - td_naik  : berapa lama setelah perintah dikirim torsi mulai memutar roda
               - td_lepas : berapa lama torsi masih bekerja setelah arus dinolkan
             td_naik dan td_lepas dicari dari POSISI encoder (rpm umpan balik tertinggal).
  imu      : jeda sudut psi hasil Kalman. Motor mati, badan robot DIGOYANG dengan tangan.
             psi Kalman dibandingkan dengan sudut hasil integral gyro (integral gyro tidak
             punya jeda, hanya drift; drift-nya dibuang dengan garis lurus).
  loop     : lama satu siklus hardware: baca IMU + kirim/baca 2 motor (arus 0, roda diam).
  analisis : hitung ulang dari file CSV yang sudah tersimpan (tanpa hardware).

Pemakaian (di Raspberry Pi, dari folder program/testing/DELAY):
  python3 atera_delay_testing.py motor
  python3 atera_delay_testing.py motor --roda kanan --arus 0.2 0.4 --ulang 3
  python3 atera_delay_testing.py imu --durasi 15
  python3 atera_delay_testing.py loop
  python3 atera_delay_testing.py analisis DATA_DELAY/delay_motor_kanan_20261008_101500.csv

Driver yang dipakai SAMA dengan program utama (program/atera_program: config.py,
ddsm115.py, mpu6050.py, kalman.py), jadi jeda yang terukur adalah jeda program yang sebenarnya.
Data disimpan di folder DATA_DELAY (di sebelah file ini) sebagai CSV + PNG:
  delay_<motor_kiri|motor_kanan|imu|loop>_<YYYYMMDD_HHmmSS>.csv / .png

Butuh: numpy, pyserial, smbus2, matplotlib (mode analisis hanya butuh numpy + matplotlib).
"""

import argparse
import csv
import math
import os
import sys
import time
from datetime import datetime

import numpy as np

FOLDER_INI = os.path.dirname(os.path.abspath(__file__))
FOLDER_ATERA = os.path.normpath(os.path.join(FOLDER_INI, "..", "..", "atera_program"))
FOLDER_DATA = os.path.join(FOLDER_INI, "DATA_DELAY")

# =============================================================================
# PENGATURAN UJI (hanya untuk program uji ini; nilai robot tetap dari config.py)
# =============================================================================
# --- uji motor (roda terangkat) ---
PERIODE_MOTOR = 0.002       # [s] ASUMSI: DDSM115 maksimum 500 Hz (wiki Waveshare, lihat ddsm115.py)
LAMA_DIAM = 0.3             # [s] arus 0 sebelum langkah arus
LAMA_NAIK_MAKS = 0.5        # [s] batas waktu fase naik
BATAS_KECEPATAN = 11.0      # [rad/s] fase naik berhenti di sini. ASUMSI: aman (~105 rpm, uji inersia 110 rpm)
LAMA_LEPAS = 0.6            # [s] lama perekaman setelah arus dinolkan
ARUS_UJI_MAKS = 0.8         # [A] ASUMSI: batas aman uji ini (juga dibatasi config.MAX_CURRENT_A)
JENDELA_SEBELUM = 0.05      # [s] data sebelum langkah yang ikut dipakai analisis
JENDELA_NAIK = 0.10         # [s] data setelah perintah I yang dipakai mencari td_naik
JENDELA_LEPAS = 0.10        # [s] data setelah perintah 0 A yang dipakai mencari td_lepas
JEDA_CARI_MAKS = 0.08       # [s] rentang pencarian td_naik dan td_lepas
JEDA_CARI_LANGKAH = 0.0005  # [s]
# Percepatan untuk G = K/J (cara yang sama dengan uji inersia):
AWAL_NAIK_DIBUANG = 0.05    # [s] awal fase naik yang tidak dipakai (arus belum tunak)
BEBAS_MULAI = 0.15          # [s] setelah perintah 0 A: awal jendela perlambatan bebas (gesekan)
BEBAS_AKHIR = 0.50          # [s] akhir jendela perlambatan bebas

# --- uji imu ---
JEDA_IMU_MAKS = 0.15        # [s] rentang pencarian jeda psi Kalman
GOYANGAN_MINIMUM = 8.0      # [deg] puncak-ke-puncak psi minimum agar hasil bisa dipercaya

KOLOM_MOTOR = ["run", "fase", "t_kirim_s", "t_balas_s", "arus_perintah_A", "arus_umpan_balik_A",
               "rpm_umpan_balik", "theta_rad", "error"]
KOLOM_IMU = ["t_mulai_s", "t_selesai_s", "psi_kalman_deg", "dpsi_gyro_deg_s",
             "psi_akselerometer_deg", "gyro_jenuh"]
KOLOM_LOOP = ["t_mulai_s", "t_imu_selesai_s", "t_perintah_terkirim_s", "t_selesai_s",
              "motor_gagal", "periode_target_s"]


# =============================================================================
# BANTUAN UMUM
# =============================================================================
def siapkan_path_atera():
    """Supaya config.py, ddsm115.py, mpu6050.py dari program/atera_program bisa di-import."""
    if not os.path.isdir(FOLDER_ATERA):
        print("Folder", FOLDER_ATERA, "tidak ditemukan.")
        print("File ini harus berada di program/testing/DELAY/.")
        return False
    if FOLDER_ATERA not in sys.path:
        sys.path.insert(0, FOLDER_ATERA)
    return True


def tanya_mulai(pesan):
    print(pesan)
    jawaban = input("Ketik S lalu Enter untuk mulai (Enter saja = batal): ")
    return jawaban.strip().lower() == "s"


def nama_file_baru(jenis):
    """DATA_DELAY/delay_<jenis>_<YYYYMMDD_HHmmSS> (tanpa ekstensi)."""
    os.makedirs(FOLDER_DATA, exist_ok=True)
    waktu = datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.join(FOLDER_DATA, "delay_" + jenis + "_" + waktu)


def simpan_csv(path, kolom, daftar_baris):
    with open(path, "w", newline="") as f:
        penulis = csv.writer(f)
        penulis.writerow(kolom)
        for baris in daftar_baris:
            penulis.writerow(baris)
    print("Data disimpan  :", path)


def baca_csv(path):
    """Kembalikan dict: nama kolom -> np.array (kolom 'fase' tetap teks)."""
    with open(path) as f:
        pembaca = csv.reader(f)
        kolom = next(pembaca)
        isi = []
        for baris in pembaca:
            if baris:
                isi.append(baris)
    data = {}
    for j in range(len(kolom)):
        nilai = []
        for baris in isi:
            nilai.append(baris[j])
        if kolom[j] == "fase":
            data[kolom[j]] = np.array(nilai)
        else:
            data[kolom[j]] = np.array(nilai, dtype=float)
    return data


def ms(detik):
    return detik * 1000.0


def cetak_statistik(nama, nilai_detik):
    """Cetak rata-rata, persentil 99, dan maksimum (dalam ms)."""
    nilai = nilai_detik[np.isfinite(nilai_detik)]
    if len(nilai) == 0:
        print(f"  {nama:<36}: tidak ada data")
        return
    print(f"  {nama:<36}: rata-rata {ms(np.mean(nilai)):6.2f} ms | 99% {ms(np.percentile(nilai, 99)):6.2f} ms"
          f" | maks {ms(np.max(nilai)):6.2f} ms")


def indeks_pertama(syarat):
    """Indeks pertama yang bernilai True, atau None."""
    for k in range(len(syarat)):
        if syarat[k]:
            return k
    return None


def kecepatan_dari_theta(t, theta, m=3):
    """Turunan tengah (theta[k+m] - theta[k-m]) / (t[k+m] - t[k-m]). Tidak menggeser waktu."""
    n = len(t)
    kecepatan = np.full(n, np.nan)
    for k in range(m, n - m):
        dt = t[k + m] - t[k - m]
        if dt > 0:
            kecepatan[k] = (theta[k + m] - theta[k - m]) / dt
    return kecepatan


def ambil_plt():
    """matplotlib tanpa jendela. None bila matplotlib tidak terpasang."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        return plt
    except ImportError:
        print("matplotlib tidak terpasang -> PNG tidak dibuat (pip install matplotlib)")
        return None


# =============================================================================
# UJI MOTOR
# =============================================================================
def rekam_satu_run(motor, arus, nomor_run, daftar_baris):
    """Satu run: diam (0 A) -> naik (arus) -> lepas (0 A). Baris CSV ditambahkan ke daftar_baris."""
    fase = "diam"
    arus_perintah = 0.0
    t_mulai_fase = 0.0
    kecepatan = 0.0
    riwayat_t = []
    riwayat_theta = []
    gagal_berturut = 0

    t0 = time.perf_counter()
    t_berikut = t0
    while True:
        t = time.perf_counter() - t0

        # 1. tentukan fase dan arus untuk sampel ini
        if fase == "diam" and t >= LAMA_DIAM:
            fase = "naik"
            arus_perintah = arus
            t_mulai_fase = t
        elif fase == "naik":
            if abs(kecepatan) >= BATAS_KECEPATAN or t - t_mulai_fase >= LAMA_NAIK_MAKS:
                fase = "lepas"
                arus_perintah = 0.0
                t_mulai_fase = t
        elif fase == "lepas" and t - t_mulai_fase >= LAMA_LEPAS:
            break

        # 2. kirim perintah lalu tunggu balasan
        t_kirim = time.perf_counter() - t0
        motor.write_current(arus_perintah)
        try:
            fb = motor.read_feedback()
            gagal_berturut = 0
        except Exception as e:
            fb = None
            gagal_berturut = gagal_berturut + 1
            if gagal_berturut > 20:
                raise RuntimeError("motor tidak membalas 20 kali berturut-turut: " + str(e))
        t_balas = time.perf_counter() - t0

        # 3. simpan (sampel gagal tetap dicatat supaya waktu kirim perintah tidak hilang)
        if fb is None:
            daftar_baris.append([nomor_run, fase, t_kirim, t_balas, arus_perintah,
                                 math.nan, math.nan, math.nan, -1])
        else:
            daftar_baris.append([nomor_run, fase, t_kirim, t_balas, arus_perintah,
                                 fb.current_a, fb.speed_rpm, motor.theta, fb.error_code])
            if fb.error_code != 0:
                raise RuntimeError(f"motor melaporkan error 0x{fb.error_code:02X}")
            # 4. kecepatan kasar dari 10 sampel terakhir, hanya untuk batas aman
            riwayat_t.append(t_balas)
            riwayat_theta.append(motor.theta)
            if len(riwayat_t) > 10:
                riwayat_t.pop(0)
                riwayat_theta.pop(0)
                kecepatan = (riwayat_theta[-1] - riwayat_theta[0]) / (riwayat_t[-1] - riwayat_t[0])

        # 5. tunggu sampai periode berikutnya
        t_berikut = t_berikut + PERIODE_MOTOR
        sisa = t_berikut - time.perf_counter()
        if sisa > 0:
            time.sleep(sisa)
        else:
            t_berikut = time.perf_counter()


def tunggu_roda_berhenti(motor, batas_waktu=8.0):
    """Kirim arus 0 sampai roda diam selama 0.3 s. False bila batas waktu habis."""
    t_awal = time.perf_counter()
    theta_lama = None
    t_lama = None
    lama_diam = 0.0
    while time.perf_counter() - t_awal < batas_waktu:
        try:
            motor.exchange(0.0)
        except Exception:
            time.sleep(0.01)
            continue
        t = time.perf_counter()
        if theta_lama is not None:
            kecepatan = abs(motor.theta - theta_lama) / (t - t_lama)
            if kecepatan < 0.3:
                lama_diam = lama_diam + (t - t_lama)
            else:
                lama_diam = 0.0
            if lama_diam >= 0.3:
                return True
        theta_lama = motor.theta
        t_lama = t
        time.sleep(0.01)
    return False


def matikan_motor(motor, mode_kecepatan):
    """Arus 0, lalu kembali ke mode kecepatan dengan target 0 rpm (sama seperti uji inersia)."""
    for _ in range(3):
        try:
            motor.exchange(0.0)
            break
        except Exception:
            pass
    try:
        motor.set_mode(mode_kecepatan)
        motor.write_current(0.0)        # di mode kecepatan, nilai 0 = target 0 rpm
    except Exception:
        pass
    motor.close()
    print("  Motor: arus 0, kembali ke mode kecepatan (0 rpm), port ditutup.")


def uji_motor(args):
    if not siapkan_path_atera():
        return
    import config
    import ddsm115

    for arus in args.arus:
        if abs(arus) > ARUS_UJI_MAKS or abs(arus) > config.MAX_CURRENT_A:
            print(f"Arus {arus} A melebihi batas aman uji ({ARUS_UJI_MAKS} A) / config.MAX_CURRENT_A.")
            return

    daftar_roda = []
    if "kiri" in args.roda:
        daftar_roda.append(("kiri", config.LEFT_MOTOR_PORT, config.LEFT_MOTOR_ID, config.LEFT_MOTOR_SIGN))
    if "kanan" in args.roda:
        daftar_roda.append(("kanan", config.RIGHT_MOTOR_PORT, config.RIGHT_MOTOR_ID, config.RIGHT_MOTOR_SIGN))

    teks_roda = ""
    for roda in daftar_roda:
        teks_roda = teks_roda + f" {roda[0]} ({roda[1]})"
    print("UJI MOTOR. Roda diuji satu per satu:" + teks_roda)
    print(f"Arus uji: {args.arus} A, dua arah, diulang {args.ulang} kali.")
    if not tanya_mulai("PASTIKAN RODA TERANGKAT dan bebas berputar. Jangan sentuh roda selama uji."):
        print("Dibatalkan.")
        return

    for nama, port, id_motor, tanda in daftar_roda:
        print(f"\n=== Roda {nama} ({port}) ===")
        motor = ddsm115.DDSM115Motor(port, id_motor, tanda, "motor " + nama)
        if not motor.initialize():
            print("  GAGAL:", motor.problem)
            continue
        daftar_baris = []
        nomor_run = 0
        try:
            tunggu_roda_berhenti(motor, 3.0)
            for _ in range(args.ulang):
                for arus in args.arus:
                    for arah in (1.0, -1.0):
                        nomor_run = nomor_run + 1
                        print(f"  run {nomor_run}: arus {arah * arus:+.2f} A")
                        rekam_satu_run(motor, arah * arus, nomor_run, daftar_baris)
                        if not tunggu_roda_berhenti(motor):
                            print("  PERINGATAN: roda belum berhenti setelah 8 s")
        except KeyboardInterrupt:
            print("\n  Dihentikan dengan Ctrl+C")
        except Exception as e:
            print("  GAGAL:", e)
        finally:
            matikan_motor(motor, ddsm115.MODE_SPEED)

        if len(daftar_baris) > 0:
            path = nama_file_baru("motor_" + nama)
            simpan_csv(path + ".csv", KOLOM_MOTOR, daftar_baris)
            analisis_file_motor(path + ".csv")


# ----------------------------------------------------------- analisis motor --
def percepatan_kuadrat(t, theta):
    """Percepatan konstan terbaik untuk data sudut: theta = c0 + c1*t + 0.5*a*t^2."""
    koef = np.polyfit(t - np.mean(t), theta, 2)
    return 2.0 * koef[0]


def cari_td_naik(t, theta):
    """Model: theta = c + 0.5*a*max(0, t - td)^2   (t = 0 saat perintah arus dikirim).

    Sebelum td roda diam, sesudah td roda dipercepat a. Semua td dicoba, dipilih yang
    galat kuadratnya paling kecil. Kembalikan (td, a).
    """
    td_terbaik = math.nan
    a_terbaik = math.nan
    galat_terbaik = math.inf
    td = 0.0
    while td <= JEDA_CARI_MAKS:
        sesudah = np.maximum(0.0, t - td)
        A = np.column_stack([np.ones(len(t)), 0.5 * sesudah ** 2])
        koef = np.linalg.lstsq(A, theta, rcond=None)[0]
        galat = theta - A @ koef
        jumlah_kuadrat = float(np.dot(galat, galat))
        if jumlah_kuadrat < galat_terbaik:
            galat_terbaik = jumlah_kuadrat
            td_terbaik = td
            a_terbaik = koef[1]
        td = td + JEDA_CARI_LANGKAH
    return td_terbaik, a_terbaik


def cari_td_lepas(t, theta):
    """Model (t = 0 saat arus 0 dikirim): sampai td roda masih dipercepat a, sesudah td melambat d.

        theta = c0 + c1*t + a*(0.5*min(t,td)^2 + td*max(0,t-td)) + d*0.5*max(0,t-td)^2

    Kembalikan (td, a, d).
    """
    td_terbaik = math.nan
    koef_terbaik = [math.nan, math.nan, math.nan, math.nan]
    galat_terbaik = math.inf
    td = 0.0
    while td <= JEDA_CARI_MAKS:
        sebelum = np.minimum(t, td)
        sesudah = np.maximum(0.0, t - td)
        kolom_a = 0.5 * sebelum ** 2 + td * sesudah
        kolom_d = 0.5 * sesudah ** 2
        A = np.column_stack([np.ones(len(t)), t, kolom_a, kolom_d])
        koef = np.linalg.lstsq(A, theta, rcond=None)[0]
        galat = theta - A @ koef
        jumlah_kuadrat = float(np.dot(galat, galat))
        if jumlah_kuadrat < galat_terbaik:
            galat_terbaik = jumlah_kuadrat
            td_terbaik = td
            koef_terbaik = koef
        td = td + JEDA_CARI_LANGKAH
    return td_terbaik, koef_terbaik[2], koef_terbaik[3]


def analisis_satu_run(d):
    """d: dict kolom -> array untuk SATU run. Kembalikan dict hasil, atau teks alasan gagal."""
    fase = d["fase"]
    n = len(fase)
    indeks = np.arange(n)
    k_naik = indeks_pertama(fase == "naik")
    k_lepas = indeks_pertama(fase == "lepas")
    if k_naik is None or k_lepas is None or n - k_lepas < 20:
        return "run tidak lengkap (tidak ada fase naik dan lepas)"

    arus = d["arus_perintah_A"][k_naik]
    arah = 1.0 if arus > 0 else -1.0
    t_langkah = d["t_kirim_s"][k_naik]      # perintah arus I dikirim
    t_nol = d["t_kirim_s"][k_lepas]         # perintah arus 0 dikirim
    # ASUMSI: motor mengukur arus/posisi di tengah antara perintah dikirim dan balasan diterima
    t_ukur = 0.5 * (d["t_kirim_s"] + d["t_balas_s"])
    ada = np.isfinite(d["theta_rad"])
    theta = d["theta_rad"]

    # arah gerak roda (posisi encoder) dibandingkan arah arus
    k_awal = indeks_pertama(ada & (indeks < k_naik))
    k_akhir = None
    for k in range(k_lepas - 1, -1, -1):
        if ada[k]:
            k_akhir = k
            break
    if k_awal is None or k_akhir is None:
        return "data posisi kosong"
    gerak = theta[k_akhir] - theta[k_awal]
    if abs(gerak) < 0.2:
        return f"roda hampir tidak bergerak ({gerak:.3f} rad) -> roda terangkat? arus terlalu kecil?"
    tanda_gerak = 1.0 if gerak > 0 else -1.0
    theta_maju = (theta - theta[k_awal]) * tanda_gerak     # mulai dari 0, positif searah gerak roda
    arus_maju = d["arus_umpan_balik_A"] * arah              # positif searah arus perintah

    hasil = {"arus": arus, "encoder_searah": gerak * arah > 0}
    hasil["komunikasi"] = d["t_balas_s"] - d["t_kirim_s"]

    # --- respon arus umpan balik ---
    naik = ada & (indeks >= k_naik) & (indeks < k_lepas)
    tunak = naik & (t_ukur - t_langkah >= 0.05)
    if np.count_nonzero(tunak) < 5:
        tunak = naik
    i_tunak = float(np.median(arus_maju[tunak]))
    hasil["i_tunak"] = i_tunak
    hasil["arus_20"] = math.nan
    hasil["arus_63"] = math.nan
    hasil["arus_turun_37"] = math.nan
    if i_tunak > 0:
        k = indeks_pertama(naik & (arus_maju >= 0.20 * i_tunak))
        if k is not None:
            hasil["arus_20"] = t_ukur[k] - t_langkah
        k = indeks_pertama(naik & (arus_maju >= 0.63 * i_tunak))
        if k is not None:
            hasil["arus_63"] = t_ukur[k] - t_langkah
        k = indeks_pertama(ada & (indeks >= k_lepas) & (arus_maju <= 0.37 * i_tunak))
        if k is not None:
            hasil["arus_turun_37"] = t_ukur[k] - t_nol

    # --- td_naik dari posisi ---
    jendela = (ada & (indeks < k_lepas) & (t_ukur >= t_langkah - JENDELA_SEBELUM)
               & (t_ukur <= t_langkah + JENDELA_NAIK))
    hasil["td_naik"] = cari_td_naik(t_ukur[jendela] - t_langkah, theta_maju[jendela])[0]

    # --- td_lepas dari posisi ---
    jendela = ada & (t_ukur >= t_nol - JENDELA_SEBELUM) & (t_ukur <= t_nol + JENDELA_LEPAS)
    hasil["td_lepas"] = cari_td_lepas(t_ukur[jendela] - t_nol, theta_maju[jendela])[0]

    # --- G = K/J = (a_naik - a_bebas) / I, sama seperti uji inersia ---
    jendela = ada & (indeks < k_lepas) & (t_ukur >= t_langkah + AWAL_NAIK_DIBUANG)
    jendela_bebas = ada & (t_ukur >= t_nol + BEBAS_MULAI) & (t_ukur <= t_nol + BEBAS_AKHIR)
    hasil["a_naik"] = math.nan
    hasil["a_bebas"] = math.nan
    hasil["G"] = math.nan
    if np.count_nonzero(jendela) >= 10 and np.count_nonzero(jendela_bebas) >= 10:
        hasil["a_naik"] = percepatan_kuadrat(t_ukur[jendela], theta_maju[jendela])
        hasil["a_bebas"] = percepatan_kuadrat(t_ukur[jendela_bebas], theta_maju[jendela_bebas])
        hasil["G"] = (hasil["a_naik"] - hasil["a_bebas"]) / abs(arus)

    # untuk grafik
    hasil["_t_ukur"] = t_ukur[ada]
    hasil["_theta"] = theta_maju[ada]
    hasil["_arus"] = arus_maju[ada] / abs(arus)
    hasil["_t_langkah"] = t_langkah
    hasil["_t_nol"] = t_nol
    return hasil


def median_hasil(daftar_hasil, kunci):
    nilai = []
    for h in daftar_hasil:
        if math.isfinite(h[kunci]):
            nilai.append(h[kunci])
    if len(nilai) == 0:
        return math.nan
    return float(np.median(nilai))


def analisis_file_motor(path):
    nama_roda = os.path.basename(path).split("_")[2]
    data = baca_csv(path)
    print(f"\nANALISIS MOTOR (roda {nama_roda}) : {os.path.basename(path)}")

    daftar_hasil = []
    nomor_run_semua = np.unique(data["run"])
    print(f"  {'run':>3} {'arus':>6} {'arus20%':>8} {'arus63%':>8} {'turun37%':>9} {'td_naik':>8}"
          f" {'td_lepas':>9} {'a_naik':>8} {'a_bebas':>8} {'G':>9}")
    print(f"  {'':>3} {'[A]':>6} {'[ms]':>8} {'[ms]':>8} {'[ms]':>9} {'[ms]':>8} {'[ms]':>9}"
          f" {'[r/s2]':>8} {'[r/s2]':>8} {'[r/s2/A]':>9}")
    for nomor in nomor_run_semua:
        pilih = data["run"] == nomor
        d = {}
        for kolom in data:
            d[kolom] = data[kolom][pilih]
        hasil = analisis_satu_run(d)
        if isinstance(hasil, str):
            print(f"  {int(nomor):>3} GAGAL - {hasil}")
            continue
        daftar_hasil.append(hasil)
        print(f"  {int(nomor):>3} {hasil['arus']:>+6.2f} {ms(hasil['arus_20']):>8.1f} {ms(hasil['arus_63']):>8.1f}"
              f" {ms(hasil['arus_turun_37']):>9.1f} {ms(hasil['td_naik']):>8.1f} {ms(hasil['td_lepas']):>9.1f}"
              f" {hasil['a_naik']:>8.1f} {hasil['a_bebas']:>8.1f} {hasil['G']:>9.0f}")

    if len(daftar_hasil) == 0:
        print("  Tidak ada run yang berhasil dianalisis.")
        return None

    semua_komunikasi = np.concatenate([h["komunikasi"] for h in daftar_hasil])
    cetak_statistik("komunikasi (kirim -> balasan)", semua_komunikasi)

    ringkasan = {}
    for kunci in ("arus_20", "arus_63", "arus_turun_37", "td_naik", "td_lepas", "G", "a_bebas"):
        ringkasan[kunci] = median_hasil(daftar_hasil, kunci)
    ringkasan["komunikasi"] = float(np.median(semua_komunikasi[np.isfinite(semua_komunikasi)]))
    rasio_arus = []
    for h in daftar_hasil:
        rasio_arus.append(h["i_tunak"] / abs(h["arus"]))

    print(f"\n  NILAI UNTUK SIMULASI (median {len(daftar_hasil)} run, roda {nama_roda}):")
    print(f"    jeda torsi mulai bekerja  td_naik  = {ms(ringkasan['td_naik']):6.1f} ms")
    print(f"    jeda torsi hilang         td_lepas = {ms(ringkasan['td_lepas']):6.1f} ms")
    print(f"    arus umpan balik: 20% dalam {ms(ringkasan['arus_20']):.1f} ms, 63% dalam "
          f"{ms(ringkasan['arus_63']):.1f} ms, turun ke 37% dalam {ms(ringkasan['arus_turun_37']):.1f} ms")
    print(f"    arus umpan balik tunak = {float(np.median(rasio_arus)):.2f} x arus perintah")
    print(f"    G = K/J = (a_naik - a_bebas)/I = {ringkasan['G']:.0f} rad/s^2 per A"
          f" (perlambatan bebas {ringkasan['a_bebas']:.1f} rad/s^2)")
    try:
        siapkan_path_atera()
        import config
        print(f"    K efektif = G x j_theta = {ringkasan['G'] * config.j_theta:.3f} N m/A"
              f"  (config: j_theta = {config.j_theta}, MOTOR_KT = {config.MOTOR_KT})")
    except (ImportError, AttributeError):
        print("    K efektif = G x j_theta (config.py tidak terbaca)")

    jumlah_berlawanan = 0
    for h in daftar_hasil:
        if not h["encoder_searah"]:
            jumlah_berlawanan = jumlah_berlawanan + 1
    if jumlah_berlawanan > 0:
        print(f"  PERINGATAN: di {jumlah_berlawanan} run theta bergerak BERLAWANAN dengan arah arus"
              f" -> cek LEFT/RIGHT_MOTOR_SIGN dan ENCODER_DIRECTION di config.py")

    gambar_motor(path.replace(".csv", ".png"), daftar_hasil, ringkasan, nama_roda)
    return ringkasan


def gambar_motor(path_png, daftar_hasil, ringkasan, nama_roda):
    plt = ambil_plt()
    if plt is None:
        return
    fig, ax = plt.subplots(2, 2, figsize=(12, 7.5))
    for h in daftar_hasil:
        label = f"{h['arus']:+.2f} A"
        kecepatan = kecepatan_dari_theta(h["_t_ukur"], h["_theta"])
        t_naik = h["_t_ukur"] - h["_t_langkah"]
        t_lepas = h["_t_ukur"] - h["_t_nol"]
        ax[0][0].plot(ms(t_naik), h["_arus"], lw=1, label=label)
        ax[0][1].plot(ms(t_naik), kecepatan, lw=1, label=label)
        ax[1][0].plot(ms(t_lepas), h["_arus"], lw=1, label=label)
        ax[1][1].plot(ms(t_lepas), kecepatan, lw=1, label=label)

    ax[0][0].set_xlim(-ms(JENDELA_SEBELUM), 150)
    ax[0][1].set_xlim(-ms(JENDELA_SEBELUM), 250)
    ax[1][0].set_xlim(-ms(JENDELA_SEBELUM), 150)
    ax[1][1].set_xlim(-ms(JENDELA_SEBELUM), 300)
    ax[0][1].axvline(ms(ringkasan["td_naik"]), color="red", ls="--",
                     label=f"td_naik {ms(ringkasan['td_naik']):.1f} ms")
    ax[1][1].axvline(ms(ringkasan["td_lepas"]), color="red", ls="--",
                     label=f"td_lepas {ms(ringkasan['td_lepas']):.1f} ms")
    ax[0][0].set_title("arus umpan balik / arus perintah (perintah I dikirim di t = 0)", fontsize=9)
    ax[0][1].set_title("kecepatan roda dari posisi [rad/s] (perintah I dikirim di t = 0)", fontsize=9)
    ax[1][0].set_title("arus umpan balik / arus perintah (perintah 0 A dikirim di t = 0)", fontsize=9)
    ax[1][1].set_title("kecepatan roda dari posisi [rad/s] (perintah 0 A dikirim di t = 0)", fontsize=9)
    for baris in ax:
        for a in baris:
            a.axvline(0.0, color="black", lw=0.8)
            a.set_xlabel("t [ms]")
            a.grid(True)
    ax[0][1].legend(fontsize=7)
    ax[1][1].legend(fontsize=7)
    fig.suptitle(f"Uji jeda motor DDSM115 - roda {nama_roda}")
    fig.tight_layout()
    fig.savefig(path_png, dpi=120)
    plt.close(fig)
    print("Grafik disimpan:", path_png)


# =============================================================================
# UJI IMU
# =============================================================================
def uji_imu(args):
    if not siapkan_path_atera():
        return
    import config
    import mpu6050

    imu = mpu6050.MPU6050()
    if not imu.open():
        print("IMU GAGAL dibuka:", imu.problem)
        return
    print(f"IMU terbaca: {imu.chip_name} (WHO_AM_I 0x{imu.who_am_i & 0xFF:02X}), "
          f"DLPF_CFG = {config.MPU6050_DLPF_CFG}")
    daftar_baris = []
    try:
        if not tanya_mulai("Langkah 1: MOTOR MATI. Letakkan robot DIAM dan TEGAK untuk kalibrasi (~3 s)."):
            print("Dibatalkan.")
            return
        hasil = imu.calibrate()
        print(f"  kalibrasi selesai: zero offset {hasil['zero_offset_deg']:.2f} deg, "
              f"simpangan gyro saat diam {hasil['gyro_std']:.2f} deg/s")
        if not tanya_mulai(f"Langkah 2: setelah mulai, GOYANGKAN badan robot maju-mundur dengan tangan\n"
                           f"(sekitar +-10..20 deg, 1-2 kali per detik) selama {args.durasi:.0f} detik."):
            print("Dibatalkan.")
            return

        periode = 1.0 / config.CONTROL_HZ      # sama dengan loop program utama
        t0 = time.perf_counter()
        t_berikut = t0
        detik_tampil = 0
        while time.perf_counter() - t0 < args.durasi:
            t_mulai = time.perf_counter() - t0
            sampel = imu.read()
            t_selesai = time.perf_counter() - t0
            psi_akselerometer = (sampel.acc_angle_deg - imu.zero_offset_deg) * imu.sign
            daftar_baris.append([t_mulai, t_selesai, sampel.psi_deg, sampel.dpsi_deg,
                                 psi_akselerometer, int(imu.gyro_saturated)])
            if t_selesai >= detik_tampil + 1:
                detik_tampil = detik_tampil + 1
                print(f"  {detik_tampil:3d} s  psi = {sampel.psi_deg:+6.1f} deg")
            t_berikut = t_berikut + periode
            sisa = t_berikut - time.perf_counter()
            if sisa > 0:
                time.sleep(sisa)
            else:
                t_berikut = time.perf_counter()
    except KeyboardInterrupt:
        print("\nDihentikan dengan Ctrl+C")
    except OSError as e:
        print("IMU GAGAL:", e)
    finally:
        imu.close()

    if len(daftar_baris) > 0:
        path = nama_file_baru("imu")
        simpan_csv(path + ".csv", KOLOM_IMU, daftar_baris)
        analisis_file_imu(path + ".csv")


def integral_gyro(t, dpsi, psi_awal):
    """Sudut dari integral gyro (trapesium). Tidak punya jeda, tetapi drift."""
    psi_gyro = np.zeros(len(t))
    psi_gyro[0] = psi_awal
    for k in range(1, len(t)):
        psi_gyro[k] = psi_gyro[k - 1] + 0.5 * (dpsi[k] + dpsi[k - 1]) * (t[k] - t[k - 1])
    return psi_gyro


def galat_setelah_geser(t, psi_uji, psi_gyro, jeda):
    """Bandingkan psi_uji(t) dengan psi_gyro(t - jeda), drift gyro dibuang (garis lurus).

    Kembalikan (rms galat, waktu yang dipakai, acuan yang sudah dikoreksi).
    """
    pakai = t - jeda >= t[0]
    t_pakai = t[pakai]
    acuan = np.interp(t_pakai - jeda, t, psi_gyro)
    selisih = psi_uji[pakai] - acuan
    garis = np.polyfit(t_pakai, selisih, 1)
    acuan_koreksi = acuan + np.polyval(garis, t_pakai)
    sisa = psi_uji[pakai] - acuan_koreksi
    rms = math.sqrt(float(np.mean(sisa ** 2)))
    return rms, t_pakai, acuan_koreksi


def analisis_file_imu(path):
    data = baca_csv(path)
    print(f"\nANALISIS IMU : {os.path.basename(path)}")
    t = data["t_selesai_s"]
    psi_kalman = data["psi_kalman_deg"]
    psi_akselerometer = data["psi_akselerometer_deg"]
    psi_gyro = integral_gyro(t, data["dpsi_gyro_deg_s"], psi_kalman[0])

    cetak_statistik("lama baca IMU (I2C + Kalman)", data["t_selesai_s"] - data["t_mulai_s"])
    cetak_statistik("periode baca", np.diff(data["t_mulai_s"]))

    jeda_terbaik = 0.0
    rms_terbaik = math.inf
    jeda = 0.0
    while jeda <= JEDA_IMU_MAKS:
        rms = galat_setelah_geser(t, psi_kalman, psi_gyro, jeda)[0]
        if rms < rms_terbaik:
            rms_terbaik = rms
            jeda_terbaik = jeda
        jeda = jeda + 0.0005
    rms_tanpa_geser = galat_setelah_geser(t, psi_kalman, psi_gyro, 0.0)[0]
    rms_akselerometer = galat_setelah_geser(t, psi_akselerometer, psi_gyro, 0.0)[0]
    goyangan = float(np.max(psi_kalman) - np.min(psi_kalman))

    print(f"  goyangan psi (puncak ke puncak)     : {goyangan:.1f} deg")
    print(f"  galat RMS psi akselerometer saja    : {rms_akselerometer:.2f} deg")
    print(f"  galat RMS psi Kalman (tanpa geser)  : {rms_tanpa_geser:.2f} deg")
    print(f"\n  NILAI UNTUK SIMULASI:")
    print(f"    jeda psi (Kalman) = {ms(jeda_terbaik):.1f} ms, galat RMS sisa = {rms_terbaik:.2f} deg")
    print(f"    dpsi (gyro) tidak lewat Kalman -> jedanya hanya dari DLPF internal MPU6050,")
    print(f"    tidak terukur di sini: lihat tabel register 26 (CONFIG) di MPU-6000/6050 Register Map.")
    print(f"    periode baca nyata = {ms(float(np.mean(np.diff(data['t_mulai_s'])))):.2f} ms")
    if goyangan < GOYANGAN_MINIMUM:
        print(f"  PERINGATAN: goyangan kurang dari {GOYANGAN_MINIMUM:.0f} deg -> ulangi dengan goyangan lebih besar")
    if np.max(data["gyro_jenuh"]) > 0:
        print("  PERINGATAN: gyro sempat jenuh (> 250 deg/s) -> goyang lebih pelan")
    if jeda_terbaik >= JEDA_IMU_MAKS - 0.001:
        print("  PERINGATAN: jeda mentok di batas pencarian -> hasil tidak bisa dipercaya")

    gambar_imu(path.replace(".csv", ".png"), t, psi_kalman, psi_akselerometer, psi_gyro, jeda_terbaik)
    return {"jeda_psi": jeda_terbaik, "rms": rms_terbaik}


def gambar_imu(path_png, t, psi_kalman, psi_akselerometer, psi_gyro, jeda):
    plt = ambil_plt()
    if plt is None:
        return
    _, t0_acuan, acuan0 = galat_setelah_geser(t, psi_kalman, psi_gyro, 0.0)
    _, tj_acuan, acuanj = galat_setelah_geser(t, psi_kalman, psi_gyro, jeda)
    fig, ax = plt.subplots(2, 1, figsize=(11, 7))
    tengah = 0.5 * (t[0] + t[-1])
    for i in range(2):
        ax[i].plot(t, psi_akselerometer, color="0.7", lw=0.8, label="psi akselerometer")
        ax[i].plot(t0_acuan, acuan0, color="black", lw=1.2, label="integral gyro (acuan, tanpa jeda)")
        ax[i].plot(tj_acuan, acuanj, color="red", lw=1, ls="--",
                   label=f"integral gyro digeser {ms(jeda):.1f} ms")
        ax[i].plot(t, psi_kalman, color="tab:blue", lw=1.2, label="psi Kalman")
        ax[i].set_xlabel("t [s]")
        ax[i].set_ylabel("psi [deg]")
        ax[i].grid(True)
    ax[1].set_xlim(tengah - 1.0, tengah + 1.0)
    ax[0].set_title("seluruh rekaman", fontsize=9)
    ax[1].set_title("perbesaran 2 detik di tengah", fontsize=9)
    ax[0].legend(fontsize=8)
    fig.suptitle(f"Uji jeda IMU - jeda psi Kalman = {ms(jeda):.1f} ms")
    fig.tight_layout()
    fig.savefig(path_png, dpi=120)
    plt.close(fig)
    print("Grafik disimpan:", path_png)


# =============================================================================
# UJI LOOP
# =============================================================================
def uji_loop(args):
    if not siapkan_path_atera():
        return
    import config
    import ddsm115
    import mpu6050

    imu = mpu6050.MPU6050()
    if not imu.open():
        print("IMU GAGAL dibuka:", imu.problem)
        return
    roda = ddsm115.DualDDSM115()
    if not roda.initialize():
        for masalah in roda.problems():
            print("Motor GAGAL:", masalah)
        roda.close()
        imu.close()
        return

    periode = 1.0 / config.CONTROL_HZ
    daftar_baris = []
    try:
        if not tanya_mulai(f"UJI LOOP {args.durasi:.0f} s pada {config.CONTROL_HZ:.0f} Hz. Arus 0 A "
                           f"(roda tidak diputar), robot boleh di lantai."):
            print("Dibatalkan.")
            return
        t0 = time.perf_counter()
        t_berikut = t0
        while time.perf_counter() - t0 < args.durasi:
            t_mulai = time.perf_counter() - t0
            imu.read()
            t_imu = time.perf_counter() - t0
            # sama seperti DualDDSM115.command_currents: tulis kedua motor dulu, baru baca
            for motor in roda.motors:
                motor.write_current(0.0)
            t_terkirim = time.perf_counter() - t0
            gagal = 0
            for motor in roda.motors:
                try:
                    motor.read_feedback()
                except Exception:
                    gagal = gagal + 1
            t_selesai = time.perf_counter() - t0
            daftar_baris.append([t_mulai, t_imu, t_terkirim, t_selesai, gagal, periode])

            t_berikut = t_berikut + periode
            sisa = t_berikut - time.perf_counter()
            if sisa > 0:
                time.sleep(sisa)
            else:
                t_berikut = time.perf_counter()
    except KeyboardInterrupt:
        print("\nDihentikan dengan Ctrl+C")
    except Exception as e:
        print("GAGAL:", e)
    finally:
        roda.stop_all()
        roda.close()
        imu.close()

    if len(daftar_baris) > 0:
        path = nama_file_baru("loop")
        simpan_csv(path + ".csv", KOLOM_LOOP, daftar_baris)
        analisis_file_loop(path + ".csv")


def analisis_file_loop(path):
    data = baca_csv(path)
    print(f"\nANALISIS LOOP : {os.path.basename(path)}")
    periode_target = float(data["periode_target_s"][0])
    lama_imu = data["t_imu_selesai_s"] - data["t_mulai_s"]
    lama_motor = data["t_selesai_s"] - data["t_imu_selesai_s"]
    jeda_perintah = data["t_perintah_terkirim_s"] - data["t_mulai_s"]
    lama_siklus = data["t_selesai_s"] - data["t_mulai_s"]
    periode = np.diff(data["t_mulai_s"])

    cetak_statistik("baca IMU", lama_imu)
    cetak_statistik("kirim + baca 2 motor", lama_motor)
    cetak_statistik("mulai baca IMU -> perintah terkirim", jeda_perintah)
    cetak_statistik("satu siklus hardware", lama_siklus)
    cetak_statistik("periode loop", periode)

    jumlah_terlambat = 0
    for p in periode:
        if p > 1.2 * periode_target:
            jumlah_terlambat = jumlah_terlambat + 1
    jumlah_gagal = int(np.count_nonzero(data["motor_gagal"]))

    print(f"\n  NILAI UNTUK SIMULASI:")
    print(f"    periode kontrol nyata = {ms(float(np.mean(periode))):.2f} ms (target {ms(periode_target):.2f} ms),"
          f" terlambat > 20%: {jumlah_terlambat} dari {len(periode)} siklus")
    print(f"    jeda baca IMU -> perintah motor terkirim = {ms(float(np.median(jeda_perintah))):.2f} ms"
          f" (belum termasuk hitungan PD + CBF-QP)")
    if jumlah_gagal > 0:
        print(f"  PERINGATAN: {jumlah_gagal} siklus dengan balasan motor gagal")

    gambar_loop(path.replace(".csv", ".png"), data, lama_siklus, periode, periode_target)


def gambar_loop(path_png, data, lama_siklus, periode, periode_target):
    plt = ambil_plt()
    if plt is None:
        return
    fig, ax = plt.subplots(2, 1, figsize=(11, 7))
    ax[0].plot(data["t_mulai_s"][1:], ms(periode), lw=0.8, label="periode loop")
    ax[0].plot(data["t_mulai_s"], ms(lama_siklus), lw=0.8, label="satu siklus hardware")
    ax[0].axhline(ms(periode_target), color="red", ls="--", label="periode target")
    ax[0].set_xlabel("t [s]")
    ax[0].set_ylabel("[ms]")
    ax[0].legend(fontsize=8)
    ax[0].grid(True)
    ax[1].hist(ms(lama_siklus), bins=60, color="tab:orange", alpha=0.8, label="satu siklus hardware")
    ax[1].axvline(ms(periode_target), color="red", ls="--", label="periode target")
    ax[1].set_xlabel("[ms]")
    ax[1].set_ylabel("jumlah siklus")
    ax[1].legend(fontsize=8)
    ax[1].grid(True)
    fig.suptitle("Uji waktu loop hardware (IMU + 2 motor)")
    fig.tight_layout()
    fig.savefig(path_png, dpi=120)
    plt.close(fig)
    print("Grafik disimpan:", path_png)


# =============================================================================
# MAIN
# =============================================================================
def main():
    parser = argparse.ArgumentParser(description="Uji jeda hardware ATERA (motor DDSM115, IMU MPU6050, loop)")
    parser.add_argument("uji", choices=["motor", "imu", "loop", "analisis"])
    parser.add_argument("file", nargs="*", help="file CSV untuk mode analisis")
    parser.add_argument("--roda", nargs="+", choices=["kiri", "kanan"], default=["kiri", "kanan"],
                        help="roda yang diuji (uji motor)")
    parser.add_argument("--arus", nargs="+", type=float, default=[0.2, 0.4],
                        help="arus uji [A], tiap nilai dipakai dua arah (uji motor)")
    parser.add_argument("--ulang", type=int, default=2, help="jumlah pengulangan (uji motor)")
    parser.add_argument("--durasi", type=float, default=10.0, help="lama uji imu / loop [s]")
    args = parser.parse_args()

    if args.uji == "motor":
        uji_motor(args)
    elif args.uji == "imu":
        uji_imu(args)
    elif args.uji == "loop":
        uji_loop(args)
    else:
        if len(args.file) == 0:
            parser.error("mode analisis butuh file CSV, mis. DATA_DELAY/delay_motor_kanan_*.csv")
        for path in args.file:
            nama = os.path.basename(path)
            if nama.startswith("delay_motor_"):
                analisis_file_motor(path)
            elif nama.startswith("delay_imu_"):
                analisis_file_imu(path)
            elif nama.startswith("delay_loop_"):
                analisis_file_loop(path)
            else:
                print("Lewati (nama file bukan delay_motor_/delay_imu_/delay_loop_):", path)


if __name__ == "__main__":
    main()