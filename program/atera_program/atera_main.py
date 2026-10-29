"""atera_main.py - program utama ATERA (robot fisik) + GUI terminal.

Jalankan di Raspberry Pi 5, dari folder atera_program:   python3 atera_main.py
    tambahkan --plot untuk plot realtime DI LAPTOP: Raspi membuka port config.PLOT_PORT (TCP) dan mengirim data plot
    ke laptop yang tersambung. Di laptop (satu WiFi / hotspot dengan Raspi), dari folder atera_program:
        python3 ise.py --plot-realtime <IP Raspi>
    IP Raspi tampil di baris "Plot realtime" GUI ini. Laptop belum / tidak tersambung -> robot tetap jalan.

Kontrolnya sama dengan simulasi (simulasi/atera_main.py), hanya sumber state dan tujuan torsinya yang berbeda.
Tiap siklus (langkah_kontrol_inti):
    perintah D-Pad / analog / W A S D -> pd.set_command(arah, belok) (Kvel*arah dan belok, semua mode gerak)
        mode gerak (pd_control.py): DIAM (default, tanpa Kp_theta), Tombol B = BALANCING (toggle, Kp_theta aktif,
        setpoint theta dipasang saat masuk dan saat D-Pad atas / bawah dilepas, nolkan_s), Tombol A = BEBAS (tanpa
        PD heading dan Kp_theta, bisa ditarik tali; A lagi -> DIAM; B saat BEBAS -> BALANCING)
    -> IMU (psi, dpsi) + encoder DDSM115 -> x = [theta, dtheta, psi, dpsi], yaw dari encoder kiri - kanan
    -> cek KOMET
    -> cbf.dob_perbarui(x, dt)                   DOB (semua mode kontrol, supaya taksirannya selalu siap)
    -> u_PD = pd.compute(x)                      (pd_control.py, torsi SATU roda)
    -> mode PD     : u = u_PD dibatasi arus
       mode PD+CBF : u = cbf.compute(x, u_PD)    (cbf_qp.py, h1 .. h4, slack + DOB)
    -> u_belok = pd.compute_belok(...)           PD heading: kedua roda bergerak bersama (BEBAS: selisih torsi tetap)
    -> roda kiri u + u_belok, roda kanan u - u_belok, arus = torsi / MOTOR_KT  (ddsm115.py)
    -> cbf.dob_catat(x, u)
    -> ise.py merekam (kolom CSV sama dengan simulasi) ke PLOT_EVALUASI/ATERA_RASPI

State: AWAL -(Y)-> SIAP -(MULAI)-> BALANCING -(X)-> SIAP
       BALANCING -> KOMET    : |psi| > SAFE_TILT_DEG, |s| > S_KOMET (hanya mode gerak BALANCING tanpa D-Pad atas /
                               bawah), torsi jenuh terlalu lama,
                               atau |yaw - yaw_ref| > YAW_KOMET_DEG (berputar, bukan mode BEBAS).
                               Saat KOMET roda DIREM aktif (KOMET_REM_*), bukan dibiarkan bebas 0 A.
       BALANCING -> GANGGUAN : IMU / motor gagal dibaca saat balancing
       Di KOMET / GANGGUAN roda mati dan data tersimpan; tegakkan robot lalu tekan MULAI lagi.
Setpoint theta (titik nol s, s = R*theta = 0) dipasang oleh Tombol Y (bersama kalibrasi IMU), saat masuk mode gerak
BALANCING (juga arah / yaw_ref), dan saat D-Pad atas / bawah dilepas di mode BALANCING; bukan oleh MULAI.
Tombol: joystick, atau keyboard di terminal ini (juga lewat SSH). Lihat joystick_mapping.py / halaman BANTUAN di GUI.
Log: config.LOG_FILE.
"""

import json
import logging
import math
import os
import shutil
import signal
import socket
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
from pd_control import MODE_BALANCING, MODE_BEBAS, MODE_DIAM, PDControl, hitung_yaw

LOGGER = logging.getLogger("atera")
DEG = 180.0 / math.pi
AWAL, KALIBRASI, SIAP, BALANCING, KOMET, GANGGUAN = "AWAL", "KALIBRASI", "SIAP", "BALANCING", "KOMET", "GANGGUAN"
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


def ip_raspi():
    """IP Raspi di jaringan WiFi / hotspot (untuk ditampilkan di GUI). Tidak mengirim paket apa pun: connect() UDP
    hanya memilih kartu jaringan yang dipakai menuju alamat itu."""
    uji = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        uji.connect(("10.255.255.255", 1))
        return uji.getsockname()[0]
    except OSError:
        return "<IP Raspi: cek dengan hostname -I>"
    finally:
        uji.close()


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
        self.perekam_uji = ise.PerekamISE()        # PENGUJIAN: L3 mulai, L3 lagi selesai
        self.uji_pd_terakhir = ""                  # PENGUJIAN PD terakhir sesi ini (pasangan BANDING otomatis)
        self.gain = {nama: float(getattr(config, nama)) for nama in NAMA_GAIN}
        self.gain["psi_max_DEG"] = float(config.psi_max_DEG)
        self.gain["zeta_DEG"] = float(config.zeta_DEG)

        self.state = AWAL
        self.mode_kontrol = ise.MODE_PD
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
        self.yaw = self.dyaw = 0.0                   # dari encoder kiri - kanan
        self.t_komet = -math.inf                     # waktu KOMET terakhir (pengereman aktif)
        self.rem_status = "-"                        # teks pengereman untuk GUI

        # Plot realtime (--plot): data dikirim lewat TCP ke laptop (ise.py --plot-realtime), lihat buat_plot()
        self.pakai_plot = plot
        self.plot_server = None                      # socket yang menunggu laptop di config.PLOT_PORT
        self.plot_klien = []                         # laptop tersambung: [socket, IP, data yang belum terkirim]
        self.plot_ip = "<IP Raspi>"
        self.plot_kiriman = []
        self.plot_status = "mati (jalankan dengan --plot)" if not plot else "belum dibuka"
        self.plot_t0 = time.monotonic()
        self.plot_siklus = 0
        self.plot_mode = None                        # (mode kontrol, mode gerak) terakhir yang dikirim ke plot
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
        self.atur_pesan("Menyiapkan CBF-QP (solver %s) ..." % config.CBF_QP_SOLVER)
        self.render()
        try:
            from cbf_qp import CBFQP
            self.cbf = CBFQP()
            self.cbf_alasan = "siap (solver %s, %d constraint, Alpha: %s)" % (
                self.cbf.plugin, self.cbf.NH, ", ".join(self.cbf.nama_alpha[i] for i in self.cbf.alpha_dipakai))
        except Exception as exc:
            LOGGER.exception("CBF-QP")
            self.cbf_alasan = "%s: %s" % (type(exc).__name__, exc)
        if self.pakai_plot:
            self.atur_pesan("Menyiapkan plot realtime untuk laptop ...")
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
        for klien in list(self.plot_klien):
            self.putus_laptop(klien, None)
        if self.plot_server is not None:
            self.plot_server.close()
            self.plot_server = None

    # --------------------------------------------------------- plot realtime (di laptop)
    def buat_plot(self):
        """Buka port config.PLOT_PORT (TCP) untuk laptop. Plot digambar DI LAPTOP oleh ise.py --plot-realtime, Raspi
        hanya mengirim data. Gagal membuka port -> robot tetap jalan tanpa plot."""
        self.plot_ip = ip_raspi()
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind(("", config.PLOT_PORT))
            server.listen(1)
            server.setblocking(False)              # accept() tidak pernah menahan loop kontrol
        except OSError as exc:
            server.close()
            self.plot_status = "gagal membuka port %d: %s" % (config.PLOT_PORT, exc)
            LOGGER.warning("plot realtime %s", self.plot_status)
            return
        self.plot_server = server
        self.plot_status = self.teks_tunggu_laptop()
        LOGGER.info("plot realtime: menunggu laptop di %s:%d", self.plot_ip, config.PLOT_PORT)

    def teks_tunggu_laptop(self):
        return ("menunggu laptop | di laptop: python3 ise.py --plot-realtime %s  (port %d)"
                % (self.plot_ip, config.PLOT_PORT))

    def kirim_plot(self, isi):
        if self.plot_klien:                                # tanpa laptop, data plot tidak dikumpulkan
            self.plot_kiriman.append(isi)

    def putus_plot(self):
        """Garis plot diputus (titik nol s pindah), supaya tidak ada garis lurus yang melompat."""
        nan = float("nan")
        self.kirim_plot((time.monotonic() - self.plot_t0, nan, nan, nan, nan, nan))

    def perbarui_plot_mode(self):
        """Judul / warna plot (mode kontrol, mode gerak) dan barrier merah = constraint CBF-QP mode gerak sekarang
        (C_MODE di WORKPLACE cbf_qp.py), juga di mode PD supaya safety set yang sama terlihat untuk dibandingkan.
        Hanya dikirim bila berubah, jadi aman dipanggil tiap siklus."""
        if not self.plot_klien:
            return
        mode = (self.mode_kontrol, self.pd.mode_gerak)
        if mode != self.plot_mode:
            self.plot_mode = mode
            self.kirim_plot(("MODE",) + mode)
        mask = None if self.cbf is None else [bool(m) for m in self.cbf.mask_mode[self.pd.mode_gerak]]
        if mask != self.plot_mask:
            self.plot_mask = mask
            self.kirim_plot(("MASK", mask))

    def sambung_laptop(self, sok, alamat):
        """Laptop baru tersambung. Laptop itu dikirimi dulu nilai config Raspi yang dipakai gambar (supaya garis batas
        di laptop sama dengan robot walau config.py laptop berbeda), lalu mode dan barrier sekarang. Boleh lebih dari
        satu laptop; koneksi lama yang mati diputus sendiri oleh gambar_plot()."""
        sok.setblocking(False)
        sok.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.plot_mode = (self.mode_kontrol, self.pd.mode_gerak)
        self.plot_mask = None if self.cbf is None else [bool(m) for m in self.cbf.mask_mode[self.pd.mode_gerak]]
        awal = [("KONFIG", {nama: float(getattr(config, nama)) for nama in ise.KONFIG_PLOT}),
                ("MODE",) + self.plot_mode, ("MASK", self.plot_mask)]
        self.plot_klien.append([sok, alamat[0], (json.dumps(awal) + "\n").encode()])
        self.plot_status = "terhubung ke laptop %s" % ", ".join(k[1] for k in self.plot_klien)
        LOGGER.info("plot realtime: laptop %s tersambung", alamat[0])

    def putus_laptop(self, klien, sebab):
        try:
            klien[0].close()
        except OSError:
            pass
        self.plot_klien.remove(klien)
        if not self.plot_klien:
            self.plot_kiriman = []
        if sebab is None:
            return
        LOGGER.info("plot realtime: laptop %s terputus (%s)", klien[1], sebab)
        if self.plot_klien:
            self.plot_status = "terhubung ke laptop %s" % ", ".join(k[1] for k in self.plot_klien)
        else:
            self.plot_status = "laptop %s terputus (%s) | %s" % (klien[1], sebab, self.teks_tunggu_laptop())

    def gambar_plot(self):
        """Dipanggil dengan kecepatan GUI (config.UI_HZ). Terima laptop baru, lalu kirim data yang terkumpul.
        Semua socket non-blocking: jaringan lambat / putus tidak pernah menahan loop kontrol."""
        if self.plot_server is None:
            return
        while True:
            try:
                sok, alamat = self.plot_server.accept()
            except (BlockingIOError, InterruptedError):
                break
            except OSError as exc:
                LOGGER.warning("plot realtime accept: %s", exc)
                break
            self.sambung_laptop(sok, alamat)
        baris = b""
        if self.plot_kiriman:
            baris = (json.dumps(self.plot_kiriman) + "\n").encode()
            self.plot_kiriman = []
        for klien in list(self.plot_klien):
            sok = klien[0]
            try:
                if sok.recv(1024) == b"":                  # laptop tidak pernah mengirim; b"" = koneksi ditutup
                    self.putus_laptop(klien, "jendela plot ditutup")
                    continue
            except (BlockingIOError, InterruptedError):
                pass
            except OSError as exc:
                self.putus_laptop(klien, type(exc).__name__)
                continue
            klien[2] += baris
            if len(klien[2]) > config.PLOT_TUNGGU_MAKS:     # koneksi mati diam-diam atau jaringan terlalu lambat
                self.putus_laptop(klien, "jaringan terlalu lambat / putus")
                continue
            try:
                if klien[2]:
                    klien[2] = klien[2][sok.send(klien[2]):]
            except (BlockingIOError, InterruptedError):
                pass
            except OSError as exc:
                self.putus_laptop(klien, type(exc).__name__)

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
        elif aksi == "BALANCING":
            self.ganti_balancing()
        elif aksi == "BEBAS":
            self.ganti_bebas()
        elif aksi == "PENGUJIAN":
            self.ganti_pengujian()
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
        self.atur_pesan("Kalibrasi IMU ... jaga robot tidak bergerak dan TEGAK di titik seimbang.")
        self.render()
        try:
            self.kalibrasi_hasil = self.imu.calibrate()
        except OSError as exc:
            self.state = AWAL
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
        """Setpoint theta (titik nol s, theta = 0) = posisi robot sekarang. Hanya offset theta_nol yang digeser,
        encoder tidak direset, jadi yaw dari encoder kiri - kanan tetap menyambung (tidak melompat ke 0)."""
        self.theta_nol += self.baca_state()[0]
        self.putus_plot()

    def ganti_balancing(self):
        """Tombol B (hanya saat balancing): DIAM / BEBAS -> BALANCING (setpoint roda = posisi dan arah sekarang),
        BALANCING -> DIAM."""
        if self.state != BALANCING:
            self.atur_pesan("Tombol B (mode BALANCING) berlaku saat balancing.")
            return
        if self.pd.mode_gerak == MODE_BALANCING:
            self.pd.atur_mode(MODE_DIAM)
            self.atur_pesan("Mode DIAM: Kp_theta mati.")
        else:
            self.pd.atur_mode(MODE_BALANCING)
            self.nolkan_s()
            self.pd.pasang_yaw_ref(self.baca_yaw()[0])
            self.atur_pesan("Mode BALANCING: Kp_theta aktif, setpoint roda di sini (s = 0, arah sekarang). B = DIAM.")
        self.perbarui_plot_mode()

    def ganti_bebas(self):
        """Tombol A (hanya saat balancing): DIAM / BALANCING -> BEBAS, BEBAS -> DIAM (arah sekarang = yaw_ref)."""
        if self.state != BALANCING:
            self.atur_pesan("Tombol A (mode BEBAS) berlaku saat balancing.")
            return
        if self.pd.mode_gerak == MODE_BEBAS:
            self.pd.atur_mode(MODE_DIAM)
            self.pd.pasang_yaw_ref(self.baca_yaw()[0])
            self.atur_pesan("Mode BEBAS mati: mode DIAM, arah sekarang = setpoint heading.")
        else:
            self.pd.atur_mode(MODE_BEBAS)
            self.atur_pesan("Mode BEBAS: PD heading dan Kp_theta mati; robot bisa ditarik tali. A = DIAM.")
        self.perbarui_plot_mode()

    def mulai(self):
        if self.state == BALANCING:
            return
        alasan = ""
        error_imu = [teks for tingkat, teks in self.imu.problems() if tingkat == ERROR]
        if error_imu:
            alasan = "IMU bermasalah: " + error_imu[0]
        elif not self.motors.ready:
            alasan = "; ".join(self.motors.problems())
        elif self.state == AWAL or not self.kalibrasi_hasil:
            alasan = "belum dikalibrasi, tekan Tombol Y dulu"
        elif self.sample is None or abs(self.sample.psi_deg) > config.START_TILT_DEG:
            alasan = "tegakkan robot dulu (|psi| harus < %g deg)" % config.START_TILT_DEG
        elif self.mode_kontrol == ise.MODE_PD_CBF and self.cbf is None:
            alasan = "CBF-QP tidak tersedia (%s), pilih mode PD (LB)" % self.cbf_alasan
        if alasan:
            self.atur_pesan("MULAI ditolak: " + alasan)
            return
        # mulai mode gerak DIAM, arah sekarang = setpoint heading, DOB dikosongkan, pengereman KOMET berhenti
        self.pd.atur_mode(MODE_DIAM)
        self.pd.set_command(0, 0)
        self.pd.pasang_yaw_ref(self.baca_yaw()[0])
        if self.cbf is not None:
            self.cbf.reset_dob()
        self.t_komet = -math.inf
        self.lama_jenuh = 0.0
        self.jumlah_cbf = {}
        self.uji_berjalan = False
        self.perekam.mulai(self.mode_kontrol, ise.JENIS_PERCOBAAN)
        self.t0 = time.monotonic()
        self.state = BALANCING
        self.plot_t0 = self.t0                            # plot realtime mulai dari 0 setiap MULAI
        self.kirim_plot(("RESET",))
        self.perbarui_plot_mode()
        self.atur_pesan("Balancing, mode %s, gerak DIAM. D-Pad = gerak / belok, B = BALANCING, A = BEBAS, "
                        "L3 = PENGUJIAN mulai / selesai, X = roda mati." % self.mode_kontrol)

    def simpan(self, perekam):
        """Simpan di latar. PENGUJIAN PD+CBF: BANDING otomatis dengan PENGUJIAN PD terakhir hari itu
        (config.EVAL_BANDING_OTOMATIS)."""
        pasangan = ""
        uji = perekam is self.perekam_uji
        if uji and perekam.mode == ise.MODE_PD_CBF and config.EVAL_BANDING_OTOMATIS:
            pasangan = self.uji_pd_terakhir or ise.pengujian_pd_terakhir()
        nama = perekam.simpan(latar=True, banding_pd=pasangan)
        if not nama:
            return "(tidak ada data)"
        if uji and perekam.mode == ise.MODE_PD:
            self.uji_pd_terakhir = nama
        if pasangan:
            cocok = ise.POLA_NAMA.match(pasangan + ".csv")
            nama += " + BANDING dengan PD KE-%s" % (cocok.group(5) if cocok else pasangan)
        elif uji and perekam.mode == ise.MODE_PD_CBF and config.EVAL_BANDING_OTOMATIS:
            nama += " (belum ada PENGUJIAN PD hari ini untuk BANDING)"
        return nama

    def simpan_semua(self, sebab):
        teks = "PERCOBAAN " + self.simpan(self.perekam)
        if self.uji_berjalan:
            teks += ", PENGUJIAN %s" % self.simpan(self.perekam_uji)
            self.uji_berjalan = False
        LOGGER.info("%s -> tersimpan %s", sebab, teks)
        return teks

    def berhenti(self, state, sebab):
        """Roda dimatikan, data disimpan (di latar). KOMET: roda lalu direm aktif (rawat_hardware)."""
        self.motors.stop_all()
        self.u_PD = self.u = self.u_belok = 0.0
        self.state = state
        if state == KOMET:
            self.t_komet = time.monotonic()
        if self.cbf is not None:
            self.cbf.reset_dob()
        self.atur_pesan("%s%s | tersimpan: %s" % ("" if state == SIAP else state + ": ", sebab,
                                                   self.simpan_semua(sebab)))

    def ganti_mode_kontrol(self, mode):
        if mode == ise.MODE_PD_CBF and self.cbf is None:
            self.atur_pesan("Mode PD+CBF ditolak, CBF-QP tidak tersedia: " + self.cbf_alasan)
            return
        if mode == self.mode_kontrol:
            return
        self.mode_kontrol = mode
        self.perbarui_plot_mode()
        if self.state == BALANCING:            # satu rekaman = satu mode kontrol
            teks = self.simpan_semua("ganti mode")
            self.perekam.mulai(mode, ise.JENIS_PERCOBAAN)
            self.t0 = time.monotonic()
            self.atur_pesan("Mode %s. Rekaman lama tersimpan: %s" % (mode, teks))
        else:
            self.atur_pesan("Mode kontrol: %s" % mode)

    def ganti_pengujian(self):
        """L3: PENGUJIAN mulai; L3 lagi: PENGUJIAN selesai dan disimpan (tanpa batas waktu). X, KOMET, ganti mode
        kontrol, dan keluar program juga menyimpan PENGUJIAN yang sedang berjalan (simpan_semua)."""
        if self.uji_berjalan:
            self.uji_berjalan = False
            self.atur_pesan("PENGUJIAN selesai (%.1f s), tersimpan: %s"
                            % (time.monotonic() - self.t_uji0, self.simpan(self.perekam_uji)))
        elif self.state != BALANCING:
            self.atur_pesan("PENGUJIAN hanya bisa saat balancing.")
        else:
            self.perekam_uji.mulai(self.mode_kontrol, ise.JENIS_PENGUJIAN)
            self.t_uji0 = time.monotonic()
            self.uji_berjalan = True
            self.atur_pesan("PENGUJIAN dimulai: dorong robot sekarang. Tekan L3 lagi untuk selesai dan simpan.")

    # --------------------------------------------------------- loop kontrol
    def baca_state(self):
        """x = [theta, dtheta, psi, dpsi]; theta = encoder + psi (model.hitung_theta) - titik nol s."""
        psi = self.sample.psi if self.sample else 0.0
        dpsi = self.sample.dpsi if self.sample else 0.0
        theta = model.hitung_theta(self.motors.theta, psi) - self.theta_nol
        dtheta = model.hitung_theta(self.motors.dtheta, dpsi)
        self.x = np.array([theta, dtheta, psi, dpsi])
        return self.x

    def baca_yaw(self):
        """(yaw, dyaw) dari encoder roda kiri dan kanan (relatif badan, + = roda ke depan):
        yaw = R*(theta_kanan - theta_kiri)/W_RODA, + = robot berputar ke KIRI dilihat dari atas."""
        kiri, kanan = self.motors.motors
        self.yaw = hitung_yaw(kiri.theta, kanan.theta)
        self.dyaw = hitung_yaw(kiri.dtheta, kanan.dtheta)
        return self.yaw, self.dyaw

    def rawat_hardware(self):
        """Saat roda mati: arus nol (encoder tetap terbaca), coba ulang hardware yang bermasalah.
        Setelah KOMET: roda direm aktif dulu (arus_rem_komet), baru 0 A."""
        sekarang = time.monotonic()
        if not self.imu.ready and sekarang >= self.coba_imu:
            self.coba_imu = sekarang + config.IMU_RETRY_S
            if self.imu.open():
                self.atur_pesan("IMU terhubung kembali.")
        if not self.motors.ready and sekarang >= self.coba_motor:
            self.coba_motor = sekarang + config.MOTOR_RETRY_S
            if self.motors.initialize():
                self.atur_pesan("Kedua motor siap.")
        # Hanya bila KEDUA motor siap: motor yang putus dicoba ulang di atas tiap MOTOR_RETRY_S (tanpa ini
        # DualDDSM115 menulis "gagal berturut-turut" ke atera.log tiap siklus).
        if self.motors.ready:
            kiri, kanan = self.arus_rem_komet(sekarang)
            self.motors.command_currents(kiri, kanan)

    def arus_rem_komet(self, sekarang):
        """Arus pengereman aktif setelah KOMET (A, kerangka robot: + = ke depan).

        DDSM115 current loop dengan 0 A = roda BEBAS: roda yang berputar ~20 rad/s saat KOMET (data 20261016 KE-01,
        KE-06, KE-07) tetap berputar lama. Selama KOMET_REM_LAMA_S detik setelah KOMET tiap roda diberi arus yang
        melawan putarannya sendiri:  i = -KOMET_REM_GAIN_A * dtheta_motor  (dijepit +-KOMET_REM_ARUS_MAKS),
        berhenti (0 A) bila kedua roda sudah < KOMET_REM_SELESAI rad/s. Tombol X (roda mati biasa) tetap 0 A."""
        if not (config.KOMET_REM_AKTIF and self.state == KOMET
                and sekarang - self.t_komet < config.KOMET_REM_LAMA_S):
            self.rem_status = "-"
            return 0.0, 0.0
        kecepatan = [m.dtheta for m in self.motors.motors]       # relatif badan = putaran motor [rad/s]
        if all(abs(w) < config.KOMET_REM_SELESAI for w in kecepatan):
            self.t_komet = -math.inf                              # selesai: roda sudah berhenti
            self.rem_status = "selesai"
            return 0.0, 0.0
        batas = config.KOMET_REM_ARUS_MAKS
        arus = [max(-batas, min(batas, -config.KOMET_REM_GAIN_A * w)) for w in kecepatan]
        self.rem_status = "mengerem (%.1f / %.1f rad/s)" % (kecepatan[0], kecepatan[1])
        return arus[0], arus[1]

    def perintah_gerak(self):
        """(arah, belok): keyboard (W A S D) didahulukan bila sedang ditekan, selain itu joystick."""
        arah_k, belok_k = self.keyboard.perintah()
        arah_j, belok_j = self.joystick.perintah()
        return (arah_k if arah_k != 0 else arah_j), (belok_k if belok_k != 0 else belok_j)

    def langkah_kontrol(self, dt):
        self.langkah_kontrol_inti(dt)
        if self.plot_klien:                                # satu titik plot tiap PLOT_KIRIM_SETIAP siklus
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

        # Perintah gerak. D-Pad atas / bawah baru DILEPAS di mode BALANCING: setpoint theta dipasang tepat di posisi
        # ini (sebelum state dibaca, jadi siklus ini sudah memakai theta = 0 di sini).
        self.arah, self.belok = self.perintah_gerak()
        arah_sebelumnya = self.pd.arah
        self.pd.set_command(self.arah, self.belok)
        if self.pd.mode_gerak == MODE_BALANCING and arah_sebelumnya != 0 and self.pd.arah == 0:
            self.nolkan_s()
        self.perbarui_plot_mode()

        x = self.baca_state()
        yaw, dyaw = self.baca_yaw()
        s = model.hitung_s(x)
        if abs(x[2]) > math.radians(config.SAFE_TILT_DEG):
            self.berhenti(KOMET, "psi = %.1f deg (> %g deg)" % (x[2] * DEG, config.SAFE_TILT_DEG))
            return
        if self.pd.tahan_posisi and abs(s) > config.S_KOMET:
            self.berhenti(KOMET, "s = %.2f m (> %g m, mode BALANCING)" % (s, config.S_KOMET))
            return
        if self.pd.mode_gerak != MODE_BEBAS and abs(yaw - self.pd.yaw_ref) > math.radians(config.YAW_KOMET_DEG):
            self.berhenti(KOMET, "berputar: yaw - yaw_ref = %.0f deg (> %g deg)"
                          % ((yaw - self.pd.yaw_ref) * DEG, config.YAW_KOMET_DEG))
            return

        # urutan: DOB (awal siklus) -> PD -> CBF-QP -> heading -> kirim -> DOB (catat u)
        if self.cbf is not None:
            self.cbf.dob_perbarui(x, dt)
        self.u_PD = self.pd.compute(x)
        nilai_c = None
        if self.mode_kontrol == ise.MODE_PD:
            self.u = jepit(self.u_PD)
            self.status_cbf = "-"
        else:
            self.cbf.mode_cbf(self.pd.mode_gerak)
            self.u = self.cbf.compute(x, self.u_PD)
            self.status_cbf = self.cbf.status
            self.jumlah_cbf[self.status_cbf] = self.jumlah_cbf.get(self.status_cbf, 0) + 1
            nilai_c = [float(c) if m > 0.0 and math.isfinite(c) else None for c, m in zip(self.cbf.c, self.cbf.mask)]
        # PD heading: u_belok hanya memakai sisa torsi u_max - |u|, jadi jepit() di bawah tidak pernah mengubah
        # rata-rata kedua roda (balancing didahulukan).
        self.u_belok = self.pd.compute_belok(yaw, dyaw, self.u, config.u_max, dt)
        self.motors.command_currents(jepit(self.u + self.u_belok) / config.MOTOR_KT,
                                     jepit(self.u - self.u_belok) / config.MOTOR_KT, strict=True)
        if self.cbf is not None:
            self.cbf.dob_catat(x, self.u)

        sekarang = time.monotonic()
        pd = self.pd
        error = (pd.error_theta, pd.error_dtheta, pd.error_psi, pd.error_dpsi)
        ekstra = self.data_ekstra()
        self.perekam.tambah(sekarang - self.t0, x, *error, self.u_PD, self.u, self.status_cbf, self.gain, 0.0, nilai_c,
                            ekstra)
        if self.uji_berjalan:
            t_uji = sekarang - self.t_uji0
            self.perekam_uji.tambah(t_uji, x, *error, self.u_PD, self.u, self.status_cbf, self.gain, 0.0, nilai_c,
                                    ekstra)

        self.lama_jenuh = self.lama_jenuh + dt if abs(self.u) >= 0.999 * config.u_max else 0.0
        if 0.0 < config.SATURASI_MAKS_S < self.lama_jenuh:
            self.berhenti(KOMET, "torsi jenuh lebih dari %g s (robot diangkat / tertahan?)" % config.SATURASI_MAKS_S)

    def data_ekstra(self):
        """Kolom CSV tambahan (ise.KOLOM_EKSTRA)."""
        pd = self.pd
        d_dtheta, d_dpsi = self.cbf.d_hat if self.cbf is not None else (0.0, 0.0)
        return {"arah": pd.arah, "bebas": float(pd.mode_gerak == MODE_BEBAS),
                "balancing": float(pd.mode_gerak == MODE_BALANCING),
                "yaw_rad": self.yaw, "dyaw_rad_s": self.dyaw, "yaw_ref_rad": pd.yaw_ref, "u_belok_Nm": self.u_belok,
                "dob_dtheta_rad_s2": d_dtheta, "dob_dpsi_rad_s2": d_dpsi,
                "slack": self.cbf.slack if (self.cbf is not None and self.mode_kontrol == ise.MODE_PD_CBF) else 0.0}

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
        baris = ["ATERA robot fisik | state %s | kontrol %s | gerak %s" % (self.state, self.mode_kontrol,
                                                                              self.pd.mode_gerak),
                 "=" * self.lebar,
                 "Loop %.1f Hz (target %g) | kerja %.1f ms | overrun %d | simpan latar %d | joystick %s | keyboard %s"
                 % (self.loop_hz, config.CONTROL_HZ, self.kerja_ms, self.overrun, ise.simpan_berjalan(),
                    self.joystick.name if self.joystick.connected else "TIDAK ADA",
                    "aktif" if self.keyboard.aktif else "tidak")]
        baris += self.bungkus("Pesan: ", self.pesan)
        baris += self.bungkus("Plot realtime: ", self.plot_status, 2)    # 2 baris: perintah laptop + IP selalu utuh
        masalah = [m for m in self.masalah() if m[0] != INFO]
        if masalah:
            for tingkat, sumber, teks in masalah[:3]:
                baris += self.bungkus("[%s] %s: " % (tingkat, sumber), teks, 2)
        else:
            baris.append("Masalah: tidak ada")
        baris += ["-" * self.lebar, tab, "-" * self.lebar]
        baris += (self.hal_status, self.hal_kontrol, self.hal_hardware, self.hal_bantuan)[self.halaman]()
        baris += ["-" * self.lebar,
                  "Joystick: Y kalibrasi | MULAI | X mati | LB PD | RB PD+CBF | B balancing | A bebas | D-Pad gerak | "
                  "L3 uji | R3 simpan",
                  "Keyboard: Y kalibrasi | M/Enter MULAI | X/Spasi mati | 1 PD | 2 PD+CBF | B balancing | F bebas | "
                  "WASD gerak | T uji | E simpan | Q keluar | <- -> halaman"]
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
                 "  yaw    %+8.1f deg   (ref %+.1f deg, KOMET bila selisih > %g deg) | dyaw %+.2f rad/s"
                 % (self.yaw * DEG, self.pd.yaw_ref * DEG, config.YAW_KOMET_DEG, self.dyaw),
                 "  u_PD %+.3f N m -> u %+.3f N m | u_belok %+.3f | arus kiri %+.2f A, kanan %+.2f A (batas %.1f A)"
                 % (self.u_PD, self.u, self.u_belok, kiri.command_a, kanan.command_a, config.MAX_CURRENT_A),
                 "  perintah gerak (joystick / keyboard): arah %+d, belok %+d -> mode %s | status CBF: %s"
                 % (self.arah, self.belok, self.pd.mode_gerak, self.status_cbf)]
        if self.state == BALANCING:
            baris.append("  PERCOBAAN %.1f s | ISE psi %.6f rad^2 s" % (time.monotonic() - self.t0, self.perekam.ise_psi))
            if self.uji_berjalan:
                baris.append("  PENGUJIAN %.1f s (L3 lagi = selesai dan simpan) | ISE psi %.6f rad^2 s" % (
                    time.monotonic() - self.t_uji0, self.perekam_uji.ise_psi))
        if self.state == KOMET:
            baris.append("  pengereman KOMET: %s" % self.rem_status)
        baris.append("  terakhir disimpan: %s" % (self.perekam.file_terakhir or "-"))
        return baris

    def hal_kontrol(self):
        pd = self.pd
        x = self.x
        # suku PD: gain x (state - setpoint), sama dengan u_PD karena BALANCE_DIRECTION_SIGN = -1
        suku = (("Kp_psi*psi", pd.Kp_psi, x[2] - pd.psi_setpoint),
                ("Kd_psi*dpsi", pd.Kd_psi, x[3] - pd.dpsi_setpoint),
                ("Kp_theta*theta", pd.Kp_theta if pd.tahan_posisi else 0.0, x[0] - pd.theta_setpoint),
                ("Kd_theta*dtheta", pd.Kd_theta, x[1] - pd.dtheta_setpoint),
                ("Kvel*arah", pd.Kvel, pd.arah))
        baris = ["PD (torsi satu roda): u_PD = %+.4f N m" % self.u_PD]
        baris += ["  %-19s %8.3f x %+8.4f = %+8.4f N m" % (n, k, v, k * v) for n, k, v in suku]
        v_tunak = -config.R * pd.Kvel * pd.arah / pd.Kd_theta if pd.Kd_theta > 0.0 else float("nan")
        if pd.tahan_posisi:
            baris.append("  BALANCING: ditahan di setpoint theta, s %+.3f m | Kp_theta aktif" % model.hitung_s(x))
        else:
            baris.append("  %s: Kp_theta tidak dipakai | v tunak -R*Kvel*arah/Kd_theta %+.2f m/s | v %+.2f m/s"
                         % (pd.mode_gerak, v_tunak, config.R * x[1]))
        if pd.mode_gerak == MODE_BEBAS:
            baris.append("  BEBAS: PD heading mati (yaw bebas, bisa ditarik tali) | u_belok = belok x TORSI_BELOK "
                         "%g = %+.3f N m" % (config.TORSI_BELOK, self.u_belok))
        else:
            baris.append("  heading: yaw %+.1f deg, ref %+.1f deg, dyaw_ref %+.2f rad/s -> u_belok %+.3f N m (%s)"
                         % (self.yaw * DEG, pd.yaw_ref * DEG, pd.dyaw_ref, self.u_belok,
                            "Kp_yaw %g, Kd_yaw %g" % (pd.Kp_yaw, pd.Kd_yaw) if config.YAW_AKTIF
                            else "YAW_AKTIF = False"))
        baris.append("")
        if self.cbf is None:
            return baris + self.bungkus("CBF-QP: tidak tersedia, ", self.cbf_alasan)
        cbf = self.cbf
        lo, hi = cbf.u_interval
        baris.append("CBF-QP (%s) | status %s | %s %s | solve %.3f ms | u diizinkan [%+.3f, %+.3f] N m"
                     % ("dipakai" if self.mode_kontrol == ise.MODE_PD_CBF else "tidak dipakai, mode PD",
                        cbf.status, cbf.plugin, "OK" if cbf.solver_ok else "gagal", cbf.solve_ms, lo, hi))
        baris.append("  Alpha: " + "  ".join("%s=%g" % (n, getattr(config, n)) for n in cbf.nama_alpha)
                     + "  | psi_max=%g deg (barrier %g deg)" % (config.psi_max_DEG, config.psi_max_DEG - config.CBF_MARGIN_DEG))
        d_dtheta, d_dpsi = cbf.d_hat
        baris.append("  DOB %s: d_ddtheta %+8.2f, d_ddpsi %+8.2f rad/s^2 | slack %s: d = %.3g"
                     % ("dipakai" if config.DOB_AKTIF else "hanya dicatat", d_dtheta, d_dpsi,
                        "aktif" if config.CBF_SLACK_AKTIF else "mati", cbf.slack))
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
                   "  PENGUJIAN PD (LB, L3 .. L3) lalu PD+CBF (RB, L3 .. L3): BANDING otomatis di PENGUJIAN/BANDING",
                   "  panel terpilih (config.EVAL_PANEL) tersimpan sendiri di subfolder PANEL/ (.eps tanpa judul)",
                   "", "Plot realtime di laptop: Raspi  python3 atera_main.py --plot",
                   "  laptop (satu WiFi, folder atera_program)  python3 ise.py --plot-realtime %s" % self.plot_ip,
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