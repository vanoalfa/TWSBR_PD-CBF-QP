#!/usr/bin/env python3
"""
ddsm115_torsi_statis_testing.py - mengukur konstanta torsi K DDSM115 secara langsung (uji torsi statis).

Tujuan: K = torsi / arus perintah, TANPA bergantung pada momen inersia roda j_theta.
(Uji inersia / uji DELAY hanya bisa memberi G = K / J.)

Susunan alat (lihat gambar susunan_uji_torsi_statis.png):
  - Badan robot DIJEPIT / ditahan di meja, roda yang diuji TIDAK menyentuh lantai.
  - Sebuah lengan kaku (penggaris besi / bilah kayu) diikat kuat ke roda, mendatar.
  - Ujung lengan menekan timbangan digital (timbangan dapur, resolusi 1 g).
  - r = jarak dari PUSAT POROS roda ke titik sentuh lengan di timbangan [m].

Cara kerja tiap titik uji:
  1. arus 0      -> baca timbangan (tara: berat lengan sendiri)
  2. arus I      -> motor menekan lengan ke timbangan, roda TIDAK berputar
  3. ketik angka timbangan (gram) lalu Enter. Selama menunggu, arus I tetap dikirim
     (paling lama BATAS_TAHAN detik, lalu otomatis 0 A).
  torsi = (m_I - m_0) [kg] x 9.81 x r ,   K = kemiringan garis torsi terhadap arus

Pemakaian (di Raspberry Pi, dari folder program/testing/DDSM115):
  python3 ddsm115_torsi_statis_testing.py --roda kanan --lengan 0.10
  python3 ddsm115_torsi_statis_testing.py --roda kiri --lengan 0.10 --arus 0.2 0.3 0.4 0.5 --G 255
  python3 ddsm115_torsi_statis_testing.py --analisis DATA_TORSI/torsi_kanan_20261009_150000.csv --G 264
Bila timbangan tidak bertambah (lengan malah terangkat), jalankan lagi dengan --arah -1.

Data: DATA_TORSI/torsi_<roda>_<YYYYMMDD_HHmmSS>.csv dan .png (di sebelah file ini).
Butuh: numpy, pyserial, matplotlib.
"""

import argparse
import csv
import math
import os
import sys
import threading
import time
from datetime import datetime

import numpy as np

FOLDER_INI = os.path.dirname(os.path.abspath(__file__))
FOLDER_ATERA = os.path.normpath(os.path.join(FOLDER_INI, "..", "..", "atera_program"))
FOLDER_DATA = os.path.join(FOLDER_INI, "DATA_TORSI")

G_BUMI = 9.81
ARUS_UJI_MAKS = 0.8          # [A] ASUMSI: batas aman uji ini (motor tidak berputar -> jangan lama-lama)
BATAS_TAHAN = 10.0           # [s] arus otomatis 0 bila angka timbangan belum diketik
LAMA_NAIK = 0.3              # [s] arus dinaikkan perlahan supaya lengan tidak memukul timbangan
BATAS_GERAK = 0.20           # [rad] roda bergerak lebih dari ini saat ditahan -> lengan selip, uji dihentikan
PERIODE = 0.005              # [s] perintah dikirim tiap 5 ms (sama dengan loop kontrol)
KT_DATASHEET = 0.75          # [N m/A] wiki Waveshare DDSM115 (untuk pembanding di grafik)

KOLOM = ["arus_perintah_A", "timbangan_g", "tara_g", "gaya_N", "torsi_Nm",
         "arus_umpan_balik_A", "gerak_roda_rad"]


class PenahanArus:
    """Mengirim arus yang sama terus-menerus di thread terpisah, selama program menunggu input."""

    def __init__(self, motor):
        self.motor = motor
        self.arus = 0.0
        self.jalan = False
        self.kesalahan = ""
        self.arus_umpan_balik = []
        self.theta_awal = None
        self.gerak_maks = 0.0
        self.thread = None

    def mulai(self, arus):
        self.arus = 0.0
        self.jalan = True
        self.kesalahan = ""
        self.arus_umpan_balik = []
        self.theta_awal = None
        self.gerak_maks = 0.0
        self.thread = threading.Thread(target=self.putar, args=(arus,), daemon=True)
        self.thread.start()

    def putar(self, arus_target):
        t0 = time.perf_counter()
        while self.jalan:
            t = time.perf_counter() - t0
            if t < LAMA_NAIK:
                self.arus = arus_target * t / LAMA_NAIK
            else:
                self.arus = arus_target
            if t > BATAS_TAHAN:
                self.kesalahan = "batas waktu %.0f s habis, arus dinolkan" % BATAS_TAHAN
                break
            try:
                fb = self.motor.exchange(self.arus)
                if self.theta_awal is None:
                    self.theta_awal = self.motor.theta
                gerak = abs(self.motor.theta - self.theta_awal)
                if gerak > self.gerak_maks:
                    self.gerak_maks = gerak
                if gerak > BATAS_GERAK:
                    self.kesalahan = "roda berputar %.2f rad -> lengan selip / tidak menekan timbangan" % gerak
                    break
                if t >= LAMA_NAIK + 0.2:
                    self.arus_umpan_balik.append(fb.current_a)
            except Exception as e:
                self.kesalahan = "komunikasi motor: " + str(e)
                break
            time.sleep(PERIODE)
        self.jalan = False
        nolkan(self.motor)

    def berhenti(self):
        self.jalan = False
        if self.thread is not None:
            self.thread.join(timeout=1.0)
        nolkan(self.motor)


def nolkan(motor):
    for _ in range(3):
        try:
            motor.exchange(0.0)
            return
        except Exception:
            time.sleep(0.01)


def baca_angka(pesan):
    """Minta angka dari keyboard. Kosong / bukan angka -> None."""
    teks = input(pesan).strip().replace(",", ".")
    if teks == "":
        return None
    try:
        return float(teks)
    except ValueError:
        print("  bukan angka, titik ini dilewati")
        return None


def nama_file_baru(roda):
    os.makedirs(FOLDER_DATA, exist_ok=True)
    waktu = datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.join(FOLDER_DATA, "torsi_" + roda + "_" + waktu)


# =============================================================================
# UJI
# =============================================================================
def uji(args):
    if not os.path.isdir(FOLDER_ATERA):
        print("Folder", FOLDER_ATERA, "tidak ditemukan. File ini harus di program/testing/DDSM115/.")
        return
    sys.path.insert(0, FOLDER_ATERA)
    import config
    import ddsm115

    for arus in args.arus:
        if arus <= 0 or arus > ARUS_UJI_MAKS or arus > config.MAX_CURRENT_A:
            print("Arus %.2f A tidak boleh (harus 0 < arus <= %.2f A)." % (arus, ARUS_UJI_MAKS))
            return

    if args.roda == "kiri":
        motor = ddsm115.DDSM115Motor(config.LEFT_MOTOR_PORT, config.LEFT_MOTOR_ID, config.LEFT_MOTOR_SIGN,
                                     "motor kiri")
    else:
        motor = ddsm115.DDSM115Motor(config.RIGHT_MOTOR_PORT, config.RIGHT_MOTOR_ID, config.RIGHT_MOTOR_SIGN,
                                     "motor kanan")

    print("UJI TORSI STATIS - roda %s, lengan r = %.3f m, arus %s A, arah %+d"
          % (args.roda, args.lengan, args.arus, args.arah))
    print("Pastikan: badan robot dijepit, roda tidak menyentuh lantai, lengan terikat kuat ke roda,")
    print("ujung lengan menyentuh timbangan, timbangan sudah menyala.")
    if not motor.initialize():
        print("Motor GAGAL:", motor.problem)
        return

    penahan = PenahanArus(motor)
    daftar_baris = []
    try:
        for ulang in range(args.ulang):
            for arus in args.arus:
                print("\n--- arus %.2f A (ulangan %d) ---" % (arus, ulang + 1))
                nolkan(motor)
                tara = baca_angka("  [arus 0 A] ketik angka timbangan (gram), Enter (kosong = lewati): ")
                if tara is None:
                    continue
                input("  tekan Enter untuk memberi arus %.2f A ..." % arus)
                penahan.mulai(args.arah * arus)
                time.sleep(LAMA_NAIK + 0.3)
                if not penahan.jalan:
                    print("  DIHENTIKAN:", penahan.kesalahan)
                    continue
                angka = baca_angka("  [arus %.2f A] ketik angka timbangan (gram), Enter: " % arus)
                tidak_sempat = not penahan.jalan
                penahan.berhenti()
                if tidak_sempat:
                    print("  DIHENTIKAN sebelum angka diketik:", penahan.kesalahan)
                    continue
                if angka is None:
                    continue
                gaya = (angka - tara) / 1000.0 * G_BUMI
                torsi = gaya * args.lengan
                if len(penahan.arus_umpan_balik) > 0:
                    i_fb = abs(float(np.median(penahan.arus_umpan_balik)))
                else:
                    i_fb = math.nan
                daftar_baris.append([arus, angka, tara, gaya, torsi, i_fb, penahan.gerak_maks])
                print("  gaya %.3f N  torsi %.4f N m  ->  torsi/arus = %.3f N m/A  (arus umpan balik %.3f A)"
                      % (gaya, torsi, torsi / arus, i_fb))
                if gaya <= 0:
                    print("  PERINGATAN: gaya tidak bertambah -> lengan terangkat? coba --arah %+d" % (-args.arah))
    except KeyboardInterrupt:
        print("\nDihentikan dengan Ctrl+C")
    finally:
        penahan.berhenti()
        motor.close()
        print("Motor: arus 0, port ditutup.")

    if len(daftar_baris) == 0:
        print("Tidak ada data.")
        return
    path = nama_file_baru(args.roda)
    with open(path + ".csv", "w", newline="") as f:
        penulis = csv.writer(f)
        penulis.writerow(KOLOM)
        for baris in daftar_baris:
            penulis.writerow(baris)
    print("Data disimpan  :", path + ".csv")
    analisis(path + ".csv", args.G)


# =============================================================================
# ANALISIS
# =============================================================================
def analisis(path, G):
    arus = []
    torsi = []
    arus_fb = []
    with open(path) as f:
        pembaca = csv.DictReader(f)
        for baris in pembaca:
            arus.append(float(baris["arus_perintah_A"]))
            torsi.append(float(baris["torsi_Nm"]))
            arus_fb.append(float(baris["arus_umpan_balik_A"]))
    arus = np.array(arus)
    torsi = np.array(torsi)
    arus_fb = np.array(arus_fb)

    print("\nANALISIS :", os.path.basename(path), "(%d titik)" % len(arus))
    if len(np.unique(arus)) >= 2:
        # garis torsi = K*arus + c  (c menampung gesekan / tara yang kurang tepat)
        K, c = np.polyfit(arus, torsi, 1)
    else:
        K = float(np.mean(torsi / arus))
        c = 0.0
    rasio_fb = float(np.nanmedian(arus_fb / arus))
    print("  K (torsi per A arus PERINTAH)     = %.3f N m/A   (datasheet %.2f)" % (K, KT_DATASHEET))
    print("  titik potong garis                = %+.4f N m" % c)
    print("  arus umpan balik / arus perintah  = %.2f" % rasio_fb)
    if rasio_fb > 0:
        print("  K terhadap arus UMPAN BALIK       = %.3f N m/A" % (K / rasio_fb))
    if G is not None and G > 0:
        print("  j_theta = K / G = %.3f / %.0f       = %.2e kg m^2  (config: 9.2e-4)" % (K, G, K / G))

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib tidak terpasang -> PNG tidak dibuat")
        return
    fig, ax = plt.subplots(figsize=(8, 5.5))
    x = np.linspace(0.0, max(arus) * 1.1, 50)
    ax.plot(arus, torsi, "o", color="tab:blue", label="hasil ukur")
    ax.plot(x, K * x + c, color="tab:blue", label="garis ukur: K = %.3f N m/A" % K)
    ax.plot(x, KT_DATASHEET * x, "--", color="red", label="datasheet: 0.75 N m/A")
    ax.plot(x, 0.238 * x, ":", color="gray", label="dari uji DELAY (j_theta 9.2e-4): 0.238 N m/A")
    ax.set_xlabel("arus perintah [A]")
    ax.set_ylabel("torsi roda [N m]")
    ax.set_title("Uji torsi statis DDSM115 - %s" % os.path.basename(path))
    ax.grid(True)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path.replace(".csv", ".png"), dpi=120)
    plt.close(fig)
    print("Grafik disimpan:", path.replace(".csv", ".png"))


def main():
    parser = argparse.ArgumentParser(description="Uji torsi statis DDSM115 (konstanta torsi K)")
    parser.add_argument("--roda", choices=["kiri", "kanan"], default="kanan")
    parser.add_argument("--lengan", type=float, default=0.10, help="r: pusat poros -> titik sentuh timbangan [m]")
    parser.add_argument("--arus", nargs="+", type=float, default=[0.2, 0.3, 0.4, 0.5], help="arus uji [A]")
    parser.add_argument("--ulang", type=int, default=2, help="jumlah pengulangan")
    parser.add_argument("--arah", type=int, choices=[1, -1], default=1, help="-1 bila lengan malah terangkat")
    parser.add_argument("--G", type=float, default=None, help="G = K/J dari uji DELAY (kiri 255, kanan 264)")
    parser.add_argument("--analisis", metavar="CSV", help="analisis ulang file CSV, tanpa motor")
    args = parser.parse_args()
    if args.analisis:
        analisis(args.analisis, args.G)
    else:
        uji(args)


if __name__ == "__main__":
    main()