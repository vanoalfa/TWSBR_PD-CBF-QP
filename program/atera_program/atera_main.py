"""atera_main.py - program utama ATERA (robot fisik) + GUI terminal.

Jalankan di Raspberry Pi 5, dari folder atera_program:   python3 atera_main.py
    tambahkan --plot untuk jendela plot realtime (sama dengan simulasi). Butuh layar di Pi: monitor, VNC, atau
    ssh -X. Tanpa layar, plot dilewati dan robot tetap jalan.

Kontrolnya sama dengan simulasi (simulasi/atera_main.py), hanya sumber state dan tujuan torsinya yang berbeda:
    IMU (psi, dpsi) + encoder DDSM115 -> x = [theta, dtheta, psi, dpsi]
    -> u_PD = pd.compute(x)                      (pd_control.py, torsi SATU roda)
    -> mode PD     : u = u_PD dibatasi arus
       mode PD+CBF : u = cbf.compute(x, u_PD)    (cbf_qp.py, QP solver CasADi)
    -> roda kiri u + u_belok, roda kanan u - u_belok, arus = torsi / MOTOR_KT  (ddsm115.py)
    -> ise.py merekam (kolom CSV sama dengan simulasi) ke PLOT_EVALUASI/ATERA_RASPI

State: DIAM -(Y)-> SIAP -(MULAI)-> BALANCING -(X)-> SIAP
       BALANCING -> KOMET    : |psi| > SAFE_TILT_DEG, |s| > S_KOMET (mode Balancing), atau torsi jenuh terlalu lama
       BALANCING -> GANGGUAN : IMU / motor gagal dibaca saat balancing
       Di KOMET / GANGGUAN roda mati dan data tersimpan; tegakkan robot lalu tekan MULAI lagi.
Titik nol s (s = R*theta = 0) dipasang oleh Tombol Y (bersama kalibrasi IMU) dan Tombol B (kapan saja), bukan oleh
MULAI. MULAI ditolak bila robot lebih dari s_max dari titik nol: kembalikan robot ke dekatnya, atau tekan B di sini.
Tombol: joystick, atau keyboard di terminal ini (juga lewat SSH). Lihat joystick_mapping.py / halaman BANTUAN di GUI.
Log: config.LOG_FILE.
"""

import logging
import math
import multiprocessing
import os
import shutil
import signal
import sys
import textwrap
import time

import numpy as np

import config
import ise
import joystick_mapping as jm
import model
from ddsm115 import MODE_CURRENT, MODE_NAMES, DDSM115Error, DualDDSM115, decode_error
from mpu6050 import MPU6050
from pd_control import MODE_BALANCING, MODE_NAVIGATE, PDControl

LOGGER = logging.getLogger("atera")
DEG = 180.0 / math.pi
DIAM, KALIBRASI, SIAP, BALANCING, KOMET, GANGGUAN = "DIAM", "KALIBRASI", "SIAP", "BALANCING", "KOMET", "GANGGUAN"
HALAMAN = ("STATUS", "KONTROL", "HARDWARE", "BANTUAN")
ERROR, AWAS, INFO = "ERROR", "AWAS", "INFO"
NAMA_GAIN = ["Kp_psi", "Kd_psi", "Kp_theta", "Kd_theta", "Kvel"] + config.NAMA_ALPHA


def batang(nilai, batas, lebar=31):
    """Batang teks posisi nilai di antara -batas..+batas, batas ditandai '|'."""
    rentang = 1.25 * batas
    sel = [" "] * lebar
    tengah = lebar // 2
    tepi = int(round(tengah * batas / rentang))
    sel[tengah] = ":"
    sel[tengah - tepi] = sel[tengah + tepi] = "|"
    sel[tengah + int(round(tengah * max(-rentang, min(rentang, nilai)) / rentang))] = "#"
    return "[" + "".join(sel) + "]"


def jepit(u):
    return max(config.u_min, min(config.u_max, u))


class Atera:
    def __init__(self, plot=False):
        self.motors = DualDDSM115()
        self.imu = MPU6050()
        self.joystick = jm.Joystick()
        self.keyboard = jm.KeyboardTerminal()          # keyboard terminal (cadangan / pengganti joystick)
        self.pd = PDControl()
        self.cbf = None
        self.cbf_alasan = "belum disiapkan"
        self.perekam = ise.PerekamISE()            # PERCOBAAN: MULAI sampai X / R3 / KOMET
        self.perekam_uji = ise.PerekamISE()        # PENGUJIAN: L3, DURASI_PENGUJIAN detik
        self.gain = {nama: float(getattr(config, nama)) for nama in NAMA_GAIN}
        self.gain["psi_max_DEG"] = float(config.psi_max_DEG)
        self.gain["zeta_DEG"] = float(config.zeta_DEG)

        self.state = DIAM
        self.mode_kontrol = ise.MODE_PD
        self.mode_gerak = MODE_BALANCING
        self.kalibrasi_hasil = {}
        self.halaman = 0
        self.pesan = "Menyiapkan hardware ..."
        self.keluar = False

        self.sample = None
        self.x = np.zeros(4)
        self.theta_nol = 0.0
        self.arah = self.belok = 0
        self.u_PD = self.u = self.u_belok = 0.0
        self.status_cbf = "-"
        self.t0 = self.t_uji0 = 0.0
        self.uji_berjalan = False
        self.lama_jenuh = 0.0
        self.jumlah_cbf = {}

        # Plot realtime (--plot): proses terpisah (ise.proses_plot), data dikirim lewat antrean
        self.pakai_plot = plot
        self.plot = None
        self.plot_antrean = self.plot_berhenti = None
        self.plot_kiriman = []
        self.plot_status = "mati (jalankan dengan --plot)" if not plot else "belum dibuka"
        self.plot_t0 = time.monotonic()
        self.plot_siklus = 0
        self.plot_mask = "belum"

        self.loop_hz = float(config.CONTROL_HZ)
        self.kerja_ms = 0.0
        self.overrun = 0
        self.coba_imu = self.coba_motor = 0.0

    # ------------------------------------------------------------------ setup
    def setup(self):
        self.render()
        self.imu.open()
        self.motors.initialize()
        self.joystick.open()
        self.keyboard.open()
        self.atur_pesan("Menyiapkan CBF-QP (CasADi) ...")
        self.render()
        try:
            from cbf_qp import CBFQP
            self.cbf = CBFQP()
            self.cbf_alasan = "siap (%d constraint, Alpha: %s)" % (
                self.cbf.NH, ", ".join(self.cbf.nama_alpha[i] for i in self.cbf.alpha_dipakai))
        except Exception as exc:
            LOGGER.exception("CBF-QP")
            self.cbf_alasan = "%s: %s" % (type(exc).__name__, exc)
        if self.pakai_plot:
            self.atur_pesan("Membuka plot realtime ...")
            self.render()
            self.buat_plot()
        error = [m for m in self.masalah() if m[0] == ERROR]
        self.atur_pesan("Ada %d masalah, lihat baris Masalah." % len(error) if error
                        else "Hardware siap. Tekan Tombol Y untuk kalibrasi.")

    def tutup(self):
        self.motors.stop_all()
        if self.state == BALANCING:
            self.simpan_semua("program keluar")
        ise.tunggu_simpan()
        for alat in (self.motors, self.joystick, self.keyboard, self.imu):
            alat.close()
        if self.plot is not None:
            self.plot_berhenti.set()
            self.plot.join(timeout=2.0)

    # --------------------------------------------------------- plot realtime
    def buat_plot(self):
        """Nyalakan proses jendela plot realtime (ise.proses_plot). Gagal -> robot tetap jalan tanpa plot."""
        if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
            self.plot_status = "tidak ada layar (DISPLAY kosong): pakai monitor / VNC / ssh -X"
            LOGGER.warning("plot realtime: %s", self.plot_status)
            return
        konteks = multiprocessing.get_context("spawn")
        self.plot_antrean = konteks.Queue()
        jawaban = konteks.Queue()
        self.plot_berhenti = konteks.Event()
        self.plot = konteks.Process(target=ise.proses_plot, daemon=True,
                                    args=(self.plot_antrean, jawaban, self.plot_berhenti, config.PLOT_BACKEND))
        self.plot.start()
        hasil = ("gagal", "proses plot tidak menjawab")
        batas_waktu = time.monotonic() + 30.0
        while time.monotonic() < batas_waktu:
            try:
                hasil = jawaban.get(timeout=0.3)
                break
            except Exception:
                if not self.plot.is_alive():
                    hasil = ("gagal", "proses plot berhenti (exit code %s)" % self.plot.exitcode)
                    break
        if hasil[0] != "siap":
            self.plot_berhenti.set()
            self.plot.join(timeout=1.0)
            self.plot = None
            self.plot_status = "gagal: %s" % hasil[1]
            LOGGER.warning("plot realtime %s", self.plot_status)
            return
        self.plot_status = "aktif (%s)" % hasil[1]
        self.kirim_plot(("MODE", self.mode_kontrol, self.mode_gerak))
        self.perbarui_mask_plot()

    def kirim_plot(self, isi):
        if self.plot is not None:
            self.plot_kiriman.append(isi)

    def putus_plot(self):
        """Garis plot diputus (titik nol s pindah), supaya tidak ada garis lurus yang melompat."""
        nan = float("nan")
        self.kirim_plot((time.monotonic() - self.plot_t0, nan, nan, nan, nan, nan))

    def perbarui_mask_plot(self):
        """Barrier merah di plot = constraint CBF-QP mode gerak sekarang (C_MODE di WORKPLACE cbf_qp.py), juga di
        mode PD supaya safety set yang sama terlihat untuk dibandingkan (sama seperti simulasi)."""
        if self.plot is None:
            return
        mask = None if self.cbf is None else [bool(m) for m in self.cbf.mask_mode[self.mode_gerak]]
        if mask != self.plot_mask:
            self.plot_mask = mask
            self.kirim_plot(("MASK", mask))

    def gambar_plot(self):
        """Kirim data yang terkumpul ke proses plot (dipanggil dengan kecepatan GUI)."""
        if self.plot is None or not self.plot_kiriman:
            return
        if not self.plot.is_alive():
            self.plot = None
            self.plot_status = "jendela plot ditutup"
            return
        self.plot_antrean.put(self.plot_kiriman)
        self.plot_kiriman = []

    def atur_pesan(self, teks):
        self.pesan = teks
        LOGGER.info(teks)

    # ----------------------------------------------------------------- tombol
    def proses_aksi(self, aksi):
        if aksi == "KELUAR":
            self.keluar = True
        elif aksi == "KALIBRASI":
            self.kalibrasi()
        elif aksi == "MULAI":
            self.mulai()
        elif aksi == "RODA_MATI":
            if self.state == BALANCING:
                self.berhenti(SIAP, "Roda dimatikan.")
            else:
                self.atur_pesan("Roda sudah mati.")
        elif aksi in ("MODE_PD", "MODE_PD_CBF"):
            self.ganti_mode_kontrol(ise.MODE_PD if aksi == "MODE_PD" else ise.MODE_PD_CBF)
        elif aksi in ("BALANCING", "NAVIGATE"):
            self.ganti_mode_gerak(MODE_BALANCING if aksi == "BALANCING" else MODE_NAVIGATE)
        elif aksi == "PENGUJIAN":
            self.mulai_pengujian()
        elif aksi == "SIMPAN":
            if self.state != BALANCING:
                self.atur_pesan("R3 hanya saat balancing (data sudah tersimpan saat roda dimatikan).")
            else:
                self.atur_pesan("PERCOBAAN disimpan: %s. Rekaman baru dimulai." % self.simpan(self.perekam))
                self.perekam.mulai(self.mode_kontrol, ise.JENIS_PERCOBAAN)
                self.t0 = time.monotonic()
        elif aksi in ("HALAMAN_KIRI", "HALAMAN_KANAN"):
            self.halaman = (self.halaman + (1 if aksi == "HALAMAN_KANAN" else -1)) % len(HALAMAN)

    def kalibrasi(self):
        if self.state == BALANCING:
            self.atur_pesan("Kalibrasi ditolak: matikan roda dulu (Tombol X).")
            return
        error = [teks for tingkat, teks in self.imu.problems() if tingkat == ERROR]
        if error:
            self.atur_pesan("Kalibrasi ditolak, IMU bermasalah: " + error[0])
            return
        self.state = KALIBRASI
        self.atur_pesan("Kalibrasi IMU ... jaga robot DIAM dan TEGAK di titik seimbang.")
        self.render()
        try:
            self.kalibrasi_hasil = self.imu.calibrate()
        except OSError as exc:
            self.state = DIAM
            self.atur_pesan("Kalibrasi GAGAL: %s" % exc)
            return
        self.state = SIAP
        try:
            self.sample = self.imu.read()                  # psi dengan sudut nol baru, untuk titik nol s
        except OSError as exc:
            LOGGER.error("%s", exc)
        self.nolkan_s()
        hasil = self.kalibrasi_hasil
        if hasil["gyro_std"] > config.IMU_CAL_MAX_GYRO_STD:
            self.atur_pesan("Robot bergerak saat kalibrasi (simpangan gyro %.1f deg/s). Ulangi Tombol Y."
                            % hasil["gyro_std"])
        else:
            self.atur_pesan("Kalibrasi selesai (sudut nol %+.2f deg, s = 0 di sini). Tekan MULAI."
                            % hasil["zero_offset_deg"])

    def nolkan_s(self):
        """Titik nol s (theta = 0) = posisi robot sekarang."""
        self.motors.reset_wheel_angles()
        self.theta_nol = 0.0
        self.theta_nol = self.baca_state()[0]
        self.putus_plot()

    def mulai(self):
        if self.state == BALANCING:
            return
        alasan = ""
        error_imu = [teks for tingkat, teks in self.imu.problems() if tingkat == ERROR]
        if error_imu:
            alasan = "IMU bermasalah: " + error_imu[0]
        elif not self.motors.ready:
            alasan = "; ".join(self.motors.problems())
        elif self.state == DIAM or not self.kalibrasi_hasil:
            alasan = "belum dikalibrasi, tekan Tombol Y dulu"
        elif self.sample is None or abs(self.sample.psi_deg) > config.START_TILT_DEG:
            alasan = "tegakkan robot dulu (|psi| harus < %g deg)" % config.START_TILT_DEG
        elif self.mode_kontrol == ise.MODE_PD_CBF and self.cbf is None:
            alasan = "CBF-QP tidak tersedia (%s), pilih mode PD (LB)" % self.cbf_alasan
        elif abs(model.hitung_s(self.baca_state())) > config.s_max:
            alasan = ("robot %.2f m dari titik nol s (batas %g m): kembalikan ke dekat titik nol, atau tekan B "
                      "untuk memasang titik nol di sini" % (model.hitung_s(self.x), config.s_max))
        if alasan:
            self.atur_pesan("MULAI ditolak: " + alasan)
            return
        self.mode_gerak = MODE_BALANCING
        self.lama_jenuh = 0.0
        self.jumlah_cbf = {}
        self.uji_berjalan = False
        self.perekam.mulai(self.mode_kontrol, ise.JENIS_PERCOBAAN)
        self.t0 = time.monotonic()
        self.state = BALANCING
        self.plot_t0 = self.t0                            # plot realtime mulai dari 0 setiap MULAI
        self.kirim_plot(("RESET",))
        self.kirim_plot(("MODE", self.mode_kontrol, self.mode_gerak))
        self.perbarui_mask_plot()
        self.atur_pesan("Balancing, mode %s. Tombol A = Navigate, L3 = PENGUJIAN, X = roda mati." % self.mode_kontrol)

    def simpan(self, perekam):
        nama = perekam.simpan(latar=True)
        return nama if nama else "(tidak ada data)"

    def simpan_semua(self, sebab):
        teks = "PERCOBAAN " + self.simpan(self.perekam)
        if self.uji_berjalan:
            teks += ", PENGUJIAN (belum %.0f s) %s" % (config.DURASI_PENGUJIAN, self.simpan(self.perekam_uji))
            self.uji_berjalan = False
        LOGGER.info("%s -> tersimpan %s", sebab, teks)
        return teks

    def berhenti(self, state, sebab):
        """Roda dimatikan, data disimpan (di latar)."""
        self.motors.stop_all()
        self.u_PD = self.u = self.u_belok = 0.0
        self.state = state
        self.atur_pesan("%s%s | tersimpan: %s" % ("" if state == SIAP else state + ": ", sebab,
                                                   self.simpan_semua(sebab)))

    def ganti_mode_kontrol(self, mode):
        if mode == ise.MODE_PD_CBF and self.cbf is None:
            self.atur_pesan("Mode PD+CBF ditolak, CBF-QP tidak tersedia: " + self.cbf_alasan)
            return
        if mode == self.mode_kontrol:
            return
        self.mode_kontrol = mode
        self.kirim_plot(("MODE", self.mode_kontrol, self.mode_gerak))
        self.perbarui_mask_plot()
        if self.state == BALANCING:            # satu rekaman = satu mode kontrol
            teks = self.simpan_semua("ganti mode")
            self.perekam.mulai(mode, ise.JENIS_PERCOBAAN)
            self.t0 = time.monotonic()
            self.atur_pesan("Mode %s. Rekaman lama tersimpan: %s" % (mode, teks))
        else:
            self.atur_pesan("Mode kontrol: %s" % mode)

    def ganti_mode_gerak(self, mode):
        """B: titik nol s pindah ke posisi sekarang (kapan saja), saat balancing juga masuk mode Balancing.
        A: mode Navigate (hanya saat balancing)."""
        if mode == MODE_BALANCING:
            self.nolkan_s()
            if self.state == BALANCING:
                self.mode_gerak = MODE_BALANCING
                self.atur_pesan("Mode Balancing: robot ditahan di sini (s = 0 di sini).")
            else:
                self.atur_pesan("Titik nol s dipasang di sini (s = 0). Tekan MULAI.")
        elif self.state != BALANCING:
            self.atur_pesan("Tombol A (Navigate) berlaku saat balancing. MULAI selalu dimulai di mode Balancing.")
        else:
            self.mode_gerak = MODE_NAVIGATE
            self.atur_pesan("Mode Navigate: D-Pad / analog kiri / keyboard W A S D untuk maju-mundur dan belok.")
        self.kirim_plot(("MODE", self.mode_kontrol, self.mode_gerak))
        self.perbarui_mask_plot()

    def mulai_pengujian(self):
        if self.state != BALANCING:
            self.atur_pesan("PENGUJIAN hanya bisa saat balancing.")
        elif self.uji_berjalan:
            self.atur_pesan("PENGUJIAN masih berjalan (%.1f / %.0f s)."
                            % (time.monotonic() - self.t_uji0, config.DURASI_PENGUJIAN))
        else:
            self.perekam_uji.mulai(self.mode_kontrol, ise.JENIS_PENGUJIAN)
            self.t_uji0 = time.monotonic()
            self.uji_berjalan = True
            self.atur_pesan("PENGUJIAN dimulai (%.0f s): dorong robot sekarang." % config.DURASI_PENGUJIAN)

    # --------------------------------------------------------- loop kontrol
    def baca_state(self):
        """x = [theta, dtheta, psi, dpsi]; theta = encoder + psi (model.hitung_theta) - titik nol s."""
        psi = self.sample.psi if self.sample else 0.0
        dpsi = self.sample.dpsi if self.sample else 0.0
        theta = model.hitung_theta(self.motors.theta, psi) - self.theta_nol
        dtheta = model.hitung_theta(self.motors.dtheta, dpsi)
        self.x = np.array([theta, dtheta, psi, dpsi])
        return self.x

    def rawat_hardware(self):
        """Saat roda mati: arus nol (encoder tetap terbaca), coba ulang hardware yang bermasalah."""
        sekarang = time.monotonic()
        if not self.imu.ready and sekarang >= self.coba_imu:
            self.coba_imu = sekarang + config.IMU_RETRY_S
            if self.imu.open():
                self.atur_pesan("IMU terhubung kembali.")
        if not self.motors.ready and sekarang >= self.coba_motor:
            self.coba_motor = sekarang + config.MOTOR_RETRY_S
            if self.motors.initialize():
                self.atur_pesan("Kedua motor siap.")
        self.motors.command_currents(0.0, 0.0)

    def perintah_gerak(self):
        """(arah, belok): keyboard (W A S D) didahulukan bila sedang ditekan, selain itu joystick."""
        arah_k, belok_k = self.keyboard.perintah()
        arah_j, belok_j = self.joystick.perintah()
        return (arah_k if arah_k != 0 else arah_j), (belok_k if belok_k != 0 else belok_j)

    def langkah_kontrol(self, dt):
        self.langkah_kontrol_inti(dt)
        if self.plot is not None:                          # satu titik plot tiap PLOT_KIRIM_SETIAP siklus
            self.plot_siklus += 1
            if self.plot_siklus >= config.PLOT_KIRIM_SETIAP:
                self.plot_siklus = 0
                x = self.x
                self.kirim_plot((time.monotonic() - self.plot_t0, float(x[0]), float(x[1]), float(x[2]),
                                 float(x[3]), self.u / config.MOTOR_KT))

    def langkah_kontrol_inti(self, dt):
        imu_ok = False
        if self.imu.ready:
            try:
                self.sample = self.imu.read()
                imu_ok = True
            except OSError as exc:
                LOGGER.error("%s", exc)
        if self.state != BALANCING:
            if self.state != KALIBRASI:
                self.rawat_hardware()
            self.baca_state()
            return
        if not imu_ok:
            self.berhenti(GANGGUAN, "IMU gagal dibaca saat balancing: " + self.imu.problem)
            return

        x = self.baca_state()
        s = model.hitung_s(x)
        if abs(x[2]) > math.radians(config.SAFE_TILT_DEG):
            self.berhenti(KOMET, "psi = %.1f deg (> %g deg)" % (x[2] * DEG, config.SAFE_TILT_DEG))
            return
        if self.mode_gerak == MODE_BALANCING and abs(s) > config.S_KOMET:
            self.berhenti(KOMET, "s = %.2f m (> %g m, mode Balancing)" % (s, config.S_KOMET))
            return

        self.arah, self.belok = self.perintah_gerak() if self.mode_gerak == MODE_NAVIGATE else (0, 0)
        self.pd.set_command(self.arah, self.mode_gerak)
        self.u_PD = self.pd.compute(x)
        nilai_c = None
        if self.mode_kontrol == ise.MODE_PD:
            self.u = jepit(self.u_PD)
            self.status_cbf = "-"
        else:
            self.cbf.mode_cbf(self.mode_gerak)
            self.u = self.cbf.compute(x, self.u_PD)
            self.status_cbf = self.cbf.status
            self.jumlah_cbf[self.status_cbf] = self.jumlah_cbf.get(self.status_cbf, 0) + 1
            nilai_c = [float(c) if m > 0.0 and math.isfinite(c) else None for c, m in zip(self.cbf.c, self.cbf.mask)]
        self.u_belok = config.TORSI_BELOK * self.belok
        self.motors.command_currents(jepit(self.u + self.u_belok) / config.MOTOR_KT,
                                     jepit(self.u - self.u_belok) / config.MOTOR_KT, strict=True)

        sekarang = time.monotonic()
        pd = self.pd
        error = (pd.error_theta, pd.error_dtheta, pd.error_psi, pd.error_dpsi)
        self.perekam.tambah(sekarang - self.t0, x, *error, self.u_PD, self.u, self.status_cbf, self.gain, 0.0, nilai_c)
        if self.uji_berjalan:
            t_uji = sekarang - self.t_uji0
            self.perekam_uji.tambah(t_uji, x, *error, self.u_PD, self.u, self.status_cbf, self.gain, 0.0, nilai_c)
            if t_uji >= config.DURASI_PENGUJIAN:
                self.uji_berjalan = False
                self.atur_pesan("PENGUJIAN selesai, tersimpan: " + self.simpan(self.perekam_uji))

        self.lama_jenuh = self.lama_jenuh + dt if abs(self.u) >= 0.999 * config.u_max else 0.0
        if 0.0 < config.SATURASI_MAKS_S < self.lama_jenuh:
            self.berhenti(KOMET, "torsi jenuh lebih dari %g s (robot diangkat / tertahan?)" % config.SATURASI_MAKS_S)

    def run(self):
        periode = 1.0 / config.CONTROL_HZ
        berikut = time.monotonic()
        mulai_lalu = berikut - periode
        ui_lalu = 0.0
        while not self.keluar:
            mulai = time.monotonic()
            dt = max(1e-4, mulai - mulai_lalu)
            mulai_lalu = mulai
            self.loop_hz += 0.05 * (1.0 / dt - self.loop_hz)

            try:
                for aksi in self.joystick.poll() + self.keyboard.poll():
                    self.proses_aksi(aksi)
                self.langkah_kontrol(dt)
            except DDSM115Error as exc:
                self.berhenti(GANGGUAN, "motor: %s" % exc)
            except Exception as exc:
                LOGGER.exception("loop kontrol")
                if self.state == BALANCING:
                    self.berhenti(GANGGUAN, "%s: %s" % (type(exc).__name__, exc))
                else:
                    self.atur_pesan("Error: %s: %s" % (type(exc).__name__, exc))

            sekarang = time.monotonic()
            self.kerja_ms += 0.05 * ((sekarang - mulai) * 1000.0 - self.kerja_ms)
            if sekarang - ui_lalu >= 1.0 / config.UI_HZ:
                self.render()
                self.gambar_plot()
                ui_lalu = sekarang
            berikut += periode
            jeda = berikut - time.monotonic()
            if jeda > 0.0:
                time.sleep(jeda)
            else:
                if self.state == BALANCING:
                    self.overrun += 1
                berikut = time.monotonic()

    # --------------------------------------------------------------- diagnosa
    def masalah(self):
        """Daftar (tingkat, sumber, pesan), urut dari yang paling parah."""
        hasil = [(tingkat, "IMU", teks) for tingkat, teks in self.imu.problems()]
        for motor in self.motors.motors:
            nama = motor.name.capitalize()
            fb = motor.feedback
            if not motor.ready:
                hasil.append((ERROR, nama, motor.problem))
            elif fb is not None and fb.mode != MODE_CURRENT:
                hasil.append((ERROR, nama, "mode %s, bukan current loop" % MODE_NAMES.get(fb.mode, fb.mode)))
            elif fb is not None and fb.error_code:
                hasil.append((AWAS, nama, "motor melaporkan error: %s -> matikan-nyalakan catu daya motor"
                              % decode_error(fb.error_code)))
            elif motor.comm_errors > 0:
                hasil.append((AWAS, nama, "%d balasan terakhir gagal: %s" % (motor.comm_errors, motor.last_error)))
        if not self.joystick.connected:
            hasil.append((AWAS if not self.keyboard.aktif else INFO, "Joystick",
                          self.joystick.problem + (" (keyboard terminal tetap bisa dipakai)" if self.keyboard.aktif
                                                   else "")))
        if not self.keyboard.aktif:
            hasil.append((INFO, "Keyboard", self.keyboard.problem))
        if self.imu.ready and not self.kalibrasi_hasil:
            hasil.append((INFO, "Kalibrasi", "belum dikalibrasi -> Tombol Y (wajib sebelum MULAI)"))
        if self.cbf is None:
            hasil.append((INFO, "CBF-QP", "mode PD+CBF tidak bisa dipakai: " + self.cbf_alasan))
        else:
            nol = [self.cbf.nama_alpha[i] for i in self.cbf.alpha_dipakai if getattr(config, self.cbf.nama_alpha[i]) <= 0]
            if nol:
                hasil.append((AWAS, "CBF-QP", "%s = 0 -> CBF-QP tidak aktif (ALPHA_NOL)" % ", ".join(nol)))
        if self.state == BALANCING and self.loop_hz < config.LOOP_MIN_RASIO * config.CONTROL_HZ:
            hasil.append((AWAS, "Loop", "loop %.0f Hz dari target %g Hz (kerja %.1f ms)"
                          % (self.loop_hz, config.CONTROL_HZ, self.kerja_ms)))
        urutan = {ERROR: 0, AWAS: 1, INFO: 2}
        return sorted(hasil, key=lambda m: urutan[m[0]])

    # -------------------------------------------------------------------- GUI
    def render(self):
        try:
            self._render()
        except Exception:
            LOGGER.exception("render")

    def bungkus(self, awalan, teks, maks=3):
        return (textwrap.wrap(teks, width=self.lebar, initial_indent=awalan,
                              subsequent_indent=" " * len(awalan)) or [awalan])[:maks]

    def _render(self):
        self.lebar = max(70, min(120, shutil.get_terminal_size((100, 40)).columns - 1))
        tab = " ".join("[%d:%s]" % (i + 1, n) if i == self.halaman else " %d:%s " % (i + 1, n)
                       for i, n in enumerate(HALAMAN))
        baris = ["ATERA robot fisik | state %s | kontrol %s | gerak %s" % (self.state, self.mode_kontrol, self.mode_gerak),
                 "=" * self.lebar,
                 "Loop %.1f Hz (target %g) | kerja %.1f ms | overrun %d | simpan latar %d | joystick %s | keyboard %s"
                 % (self.loop_hz, config.CONTROL_HZ, self.kerja_ms, self.overrun, ise.simpan_berjalan(),
                    self.joystick.name if self.joystick.connected else "TIDAK ADA",
                    "aktif" if self.keyboard.aktif else "tidak")]
        baris += self.bungkus("Pesan: ", self.pesan)
        baris += self.bungkus("Plot realtime: ", self.plot_status, 1)
        masalah = [m for m in self.masalah() if m[0] != INFO]
        if masalah:
            for tingkat, sumber, teks in masalah[:3]:
                baris += self.bungkus("[%s] %s: " % (tingkat, sumber), teks, 2)
        else:
            baris.append("Masalah: tidak ada")
        baris += ["-" * self.lebar, tab, "-" * self.lebar]
        baris += (self.hal_status, self.hal_kontrol, self.hal_hardware, self.hal_bantuan)[self.halaman]()
        baris += ["-" * self.lebar,
                  "Joystick: Y kalibrasi | MULAI | X mati | LB PD | RB PD+CBF | B diam | A gerak | L3 uji | R3 simpan",
                  "Keyboard: Y kalibrasi | M/Enter MULAI | X/Spasi mati | 1 PD | 2 PD+CBF | B diam | N gerak (WASD) | "
                  "T uji | E simpan | Q keluar | <- -> halaman"]
        sys.stdout.write("\x1b[H" + "".join(b[:self.lebar] + "\x1b[K\n" for b in baris) + "\x1b[J")
        sys.stdout.flush()

    def hal_status(self):
        x = self.x
        s = model.hitung_s(x)
        kiri, kanan = self.motors.motors
        baris = ["  psi    %+8.2f deg   %s  +-%.0f deg" % (x[2] * DEG, batang(x[2], config.psi_max), config.psi_max * DEG),
                 "  dpsi   %+8.3f rad/s %s  +-%.2f rad/s" % (x[3], batang(x[3], config.dpsi_max), config.dpsi_max),
                 "  dtheta %+8.3f rad/s %s  +-%.1f rad/s" % (x[1], batang(x[1], config.dtheta_max), config.dtheta_max),
                 "  s      %+8.3f m     %s  +-%.1f m dari titik nol Y/B (theta %+.2f rad)" % (s, batang(s, config.s_max), config.s_max, x[0]),
                 "  u      %+8.3f N m   %s  +-%.2f N m" % (self.u, batang(self.u, config.u_max), config.u_max),
                 "",
                 "  u_PD %+.3f N m -> u %+.3f N m | belok %+.3f | arus kiri %+.2f A, kanan %+.2f A (batas %.1f A)"
                 % (self.u_PD, self.u, self.u_belok, kiri.command_a, kanan.command_a, config.MAX_CURRENT_A),
                 "  perintah gerak (joystick / keyboard): arah %+d, belok %+d (hanya mode Navigate) | status CBF: %s"
                 % (self.arah, self.belok, self.status_cbf)]
        if self.state == BALANCING:
            baris.append("  PERCOBAAN %.1f s | ISE psi %.6f rad^2 s" % (time.monotonic() - self.t0, self.perekam.ise_psi))
            if self.uji_berjalan:
                baris.append("  PENGUJIAN %.1f / %.0f s | ISE psi %.6f rad^2 s" % (
                    time.monotonic() - self.t_uji0, config.DURASI_PENGUJIAN, self.perekam_uji.ise_psi))
        baris.append("  terakhir disimpan: %s" % (self.perekam.file_terakhir or "-"))
        return baris

    def hal_kontrol(self):
        pd = self.pd
        balancing = self.mode_gerak == MODE_BALANCING
        x = self.x
        suku = (("Kp_psi*psi", pd.Kp_psi, x[2]), ("Kd_psi*dpsi", pd.Kd_psi, x[3]),
                ("Kp_theta*theta", pd.Kp_theta if balancing else 0.0, x[0]), ("Kd_theta*dtheta", pd.Kd_theta, x[1]),
                ("Kvel*arah", pd.Kvel, 0 if balancing else pd.arah))
        baris = ["PD (torsi satu roda): u_PD = %+.4f N m" % self.u_PD]
        baris += ["  %-16s %8.3f x %+8.4f = %+8.4f N m" % (n, k, v, k * v) for n, k, v in suku]
        baris.append("")
        if self.cbf is None:
            return baris + self.bungkus("CBF-QP: tidak tersedia, ", self.cbf_alasan)
        cbf = self.cbf
        lo, hi = cbf.u_interval
        baris.append("CBF-QP (%s) | status %s | %s %s | solve %.3f ms | u diizinkan [%+.3f, %+.3f] N m"
                     % ("dipakai" if self.mode_kontrol == ise.MODE_PD_CBF else "tidak dipakai, mode PD",
                        cbf.status, cbf.plugin, "OK" if cbf.solver_ok else "gagal", cbf.solve_ms, lo, hi))
        baris.append("  Alpha: " + "  ".join("%s=%g" % (n, getattr(config, n)) for n in cbf.nama_alpha)
                     + "  | psi_max=%g deg" % config.psi_max_DEG)
        baris.append("  sejak MULAI: " + (", ".join("%s %d" % kv for kv in self.jumlah_cbf.items()) or "-"))
        if self.mode_kontrol == ise.MODE_PD:
            return baris + ["  nilai constraint c_h hanya dihitung di mode PD+CBF (RB)"]
        for k in range(cbf.NH):
            aktif = cbf.mask[k] > 0.0
            c = cbf.c[k]
            keadaan = "tidak dipakai di mode ini" if not aktif else (
                "-" if not math.isfinite(c) else ("MELANGGAR" if c < 0 else ("membatasi u" if c < 1e-6 else "aman")))
            baris.append("  c_h%d = %s  %s" % (k + 1, "%+12.4f" % c if math.isfinite(c) else "%12s" % "-", keadaan))
        return baris

    def hal_hardware(self):
        s = self.sample
        baris = ["IMU %s (WHO_AM_I 0x%02X) | %d sampel, %d gagal | |a| %.2f g"
                 % (self.imu.chip_name, self.imu.who_am_i & 0xFF, self.imu.read_count, self.imu.error_count,
                    self.imu.acc_norm)]
        if s is not None:
            baris.append("  psi %+8.3f deg | dpsi %+8.2f deg/s | sudut accel %+8.3f deg | dt %.2f ms"
                         % (s.psi_deg, s.dpsi_deg, s.acc_angle_deg, s.dt * 1000.0))
        hasil = self.kalibrasi_hasil
        baris.append("  kalibrasi: " + ("sudut nol %+.3f deg, bias gyro %+.3f / %+.3f deg/s, simpangan %.2f deg/s"
                                        % (hasil["zero_offset_deg"], hasil["gyro_bias_x"], hasil["gyro_bias_y"],
                                           hasil["gyro_std"]) if hasil else "BELUM"))
        baris += ["", "MOTOR        perintah   arus fb   rpm fb  posisi    theta   dtheta  balasan ok/timeout/rusak"]
        for m in self.motors.motors:
            fb = m.feedback
            if not m.ready or fb is None:
                baris += self.bungkus("  %-10s " % m.name, m.problem or "belum ada umpan balik", 2)
                continue
            baris.append("  %-10s %+7.3f A %+7.3f A %+6.0f %7d %+8.3f %+8.3f  %d/%d/%d %s"
                         % (m.name, m.command_a, fb.current_a, fb.speed_rpm, fb.position_raw, m.theta, m.dtheta,
                            m.count_ok, m.count_timeout, m.count_bad_frame, decode_error(fb.error_code)))
        baris += ["", "Joystick: " + (self.joystick.name if self.joystick.connected else self.joystick.problem),
                  "  event terakhir: " + self.joystick.last_event,
                  "Keyboard terminal: " + ("aktif" if self.keyboard.aktif else self.keyboard.problem),
                  "  tombol terakhir: " + self.keyboard.last_event,
                  "", "Cek arah sebelum balancing (roda mati):",
                  "  badan miring ke DEPAN -> psi POSITIF (salah: IMU_SIGN)",
                  "  dorong robot ke DEPAN di lantai -> theta / s NAIK (salah: ENCODER_DIRECTION)",
                  "  robot diangkat, MULAI, badan dimiringkan ke DEPAN -> kedua roda berputar ke DEPAN "
                  "(salah: MOTOR_SIGN roda itu)"]
        return baris

    def hal_bantuan(self):
        return (["Joystick:"] + ["  " + teks for teks in jm.BANTUAN] + [""] + list(jm.BANTUAN_KEYBOARD)
                + ["", "Hasil: %s/<PERCOBAAN atau PENGUJIAN>/<PD atau CBFQP>" % config.ISE_PLOT_DIR,
                   "  python3 ise.py --daftar                     daftar dan metrik percobaan",
                   "  python3 ise.py --bandingkan --pd 3 --cbf 5   PD KE-03 vs PD+CBF KE-05 (folder BANDING)",
                   "  tambahkan --pengujian untuk hasil PENGUJIAN (L3)",
                   "", "Plot realtime: python3 atera_main.py --plot  (butuh layar Pi: monitor / VNC / ssh -X)",
                   "  merah = barrier CBF mode gerak ini, abu-abu = tidak dipakai, hitam = KOMET, "
                   "kuning / oranye = |psi| 5 / 10 deg"])


def _sigterm(*_):
    raise KeyboardInterrupt


def main():
    path_log = os.path.join(os.path.dirname(os.path.abspath(__file__)), config.LOG_FILE)
    logging.basicConfig(filename=path_log, level=logging.INFO,
                        format="%(asctime)s | %(levelname)s | %(name)s | %(message)s")
    signal.signal(signal.SIGTERM, _sigterm)
    app = Atera(plot="--plot" in sys.argv[1:])
    sys.stdout.write("\x1b[?1049h\x1b[?25l\x1b[2J")          # layar alternatif, kursor disembunyikan
    error = ""
    try:
        app.setup()
        app.run()
    except KeyboardInterrupt:
        LOGGER.info("Dihentikan dari keyboard")
    except Exception as exc:
        LOGGER.exception("Error fatal")
        error = "%s: %s" % (type(exc).__name__, exc)
    finally:
        try:
            app.tutup()
        finally:
            sys.stdout.write("\x1b[?25h\x1b[?1049l")
            sys.stdout.flush()
    print("ATERA berhenti" + (" karena error: %s (detail: %s)" % (error, path_log) if error else "."))
    if app.perekam.file_terakhir:
        print("Percobaan terakhir: %s" % app.perekam.path_terakhir)
    return 1 if error else 0


if __name__ == "__main__":
    raise SystemExit(main())