"""simulasi.py - Dunia simulasi PyBullet untuk robot ATERA.

Jalankan:  python3 atera_main.py --PD   atau   python3 atera_main.py --CBFQP
           (python3 simulasi.py --PD / --CBFQP hasilnya sama)

File ini hanya mengurus simulasinya:
    - dunia (plane.urdf bawaan PyBullet), robot, sensor, motor, dorongan
    - realisme dari uji hardware (nilai di config.py, bagian 11; semua pengaturan simulasi di bagian 12):
        a. torsi roda = arus perintah x MOTOR_KT_EFEKTIF (bukan MOTOR_KT)
        b. torsi baru bekerja SIM_JEDA_TORSI detik setelah perintah dihitung
        c. psi dibaca lewat model MPU6050 (akselerometer + gyro + noise) dan Kalman yang sama dengan robot
    - jendela "ATERA - Kontrol" (Qt): tombol W A S D B N O P + kolom angka parameter (input utama)
      (B = mode Balancing, N = mode Navigate)
    - keyboard di jendela PyBullet (input cadangan, tombolnya sama)
    - tampilan di jendela PyBullet: teks parameter dan bidang merah safety set
    - jendela plot realtime (hanya bila dijalankan dengan --plot)
Kontrolnya ada di atera_main.py.

Arah di simulasi:
    DEPAN robot = sumbu +Y dunia.  Sumbu roda = sumbu X.  Atas = sumbu Z.
    Roda kiri (Lwheel_link) ada di sisi -X dunia, roda kanan (Rwheel_link) di sisi +X.
    psi   > 0 : badan miring ke depan (diukur dari garis vertikal)
    theta > 0 : roda menggelinding ke depan (sudut roda absolut, definisi di model.hitung_theta)
    u     > 0 : torsi mendorong roda ke depan (torsi SATU roda, kedua roda diberi u yang sama)
"""

import importlib.util
import math
import multiprocessing
import os
import time

import numpy as np
import pybullet as p
import pybullet_data

import config
import model
from pd_control import MODE_BALANCING


# Kalman dipakai langsung dari kalman.py milik robot (tanpa salinan). Dimuat lewat path karena folder robot juga
# punya config.py dan model.py yang namanya sama dengan milik simulasi. Susunan folder repo: simulasi/ dan program/atera_program/.
_FOLDER = os.path.dirname(os.path.abspath(__file__))
_PATH_KALMAN = [os.path.join(_FOLDER, "..", "program", "atera_program", "kalman.py"),
                os.path.join(_FOLDER, "..", "atera_program", "kalman.py")]
_ADA = [q for q in _PATH_KALMAN if os.path.isfile(q)]
if not _ADA:
    raise FileNotFoundError("kalman.py tidak ditemukan. Dicari di:\n  " + "\n  ".join(_PATH_KALMAN))
_spec = importlib.util.spec_from_file_location("kalman", _ADA[0])
kalman = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(kalman)


NAMA_ALPHA = config.NAMA_ALPHA          # semua Alpha di config.py -> otomatis muncul di jendela Kontrol
NAMA_GAIN = ["Kp_psi", "Kd_psi", "Kp_theta", "Kd_theta", "Kvel"] + NAMA_ALPHA

# Parameter di jendela Kontrol: (nama, jumlah desimal). Isinya dari config.DESIMAL dan config.DESIMAL_ALPHA.
DAFTAR_PARAMS = [(_nama, config.DESIMAL.get(_nama, config.DESIMAL_ALPHA))
                 for _nama in NAMA_GAIN + ["Gaya_dorong", "zeta_DEG"]]

FOLDER_INI = os.path.dirname(os.path.abspath(__file__))
PILIHAN_URDF = [os.path.join(FOLDER_INI, path) for path in config.URDF_PILIHAN]


def cari_urdf():
    for path in PILIHAN_URDF:
        if os.path.exists(path):
            return os.path.abspath(path)
    raise FileNotFoundError("robot.urdf tidak ditemukan. Lokasi yang dicoba:\n  " + "\n  ".join(PILIHAN_URDF))


def quaternion_dari_matriks(m):
    """Matriks rotasi 3x3 -> quaternion [x, y, z, w] (urutan PyBullet)."""
    jejak = m[0, 0] + m[1, 1] + m[2, 2]
    if jejak > 0.0:
        s = math.sqrt(jejak + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return [x, y, z, w]


def nilai_awal_params():
    """Nilai awal tiap parameter: gain dari config.py, gaya dorong dari config.GAYA_DORONG_AWAL."""
    nilai = {}
    for nama, _ in DAFTAR_PARAMS:
        if nama == "Gaya_dorong":
            nilai[nama] = float(config.GAYA_DORONG_AWAL)
        else:
            nilai[nama] = float(getattr(config, nama))
    return nilai


def tulis_angka(nama, nilai):
    desimal = 2
    for nama_params, jumlah_desimal in DAFTAR_PARAMS:
        if nama_params == nama:
            desimal = jumlah_desimal
    teks = "%.*f" % (desimal, nilai)
    if abs(float(teks) - nilai) > 1e-12:
        teks = "%.10g" % nilai          # angka yang diketik lebih teliti daripada jumlah desimal bawaan
    return teks


def cetak_nilai(nilai, awal):
    """Cetak nilai parameter dalam bentuk baris config.py."""
    print("")
    print("--- nilai parameter sekarang (salin ke config.py) ---")
    for nama, _ in DAFTAR_PARAMS:
        teks = "%s = %s" % (nama, tulis_angka(nama, nilai[nama]))
        if nama == "Gaya_dorong":
            teks = "# %s N (GAYA_DORONG_AWAL di config.py)" % teks
        if abs(nilai[nama] - awal[nama]) > 1e-9:
            teks = "%-28s # berubah, sebelumnya %s" % (teks, tulis_angka(nama, awal[nama]))
        print(teks, flush=True)
    print("---------------------------------------------------", flush=True)


# =============================================================================
# Jendela "ATERA - Kontrol"
#
# Jendela Kontrol (Qt) dan jendela plot (matplotlib) dijalankan di PROSES TERPISAH dari PyBullet, jadi bila
# jendelanya gagal / crash, simulasi tetap jalan. Data dikirim antar proses lewat antrean:
#     jendela -> simulasi : ("nilai", dict parameter), ("tombol", huruf baru ditekan), ("tahan", dict tombol ditahan)
#     simulasi -> jendela : ("status", dict) untuk menyalakan tombol dan menulis mode / dorongan
# Pasang Qt:  pip install PyQt5
# =============================================================================
def pilihan_platform_qt():
    """Urutan "platform" Qt yang dicoba. Di WSL (WSLg) Wayland dicoba dulu, karena jendela X11 (xcb)
    sering gagal di WSL (butuh pustaka libxcb tambahan, atau crash "[xcb] Unknown sequence number")."""
    pilihan = []
    if os.environ.get("QT_QPA_PLATFORM"):
        pilihan.append(os.environ["QT_QPA_PLATFORM"])     # pilihan pengguna sendiri didahulukan
    if os.environ.get("WAYLAND_DISPLAY") and "wayland" not in pilihan:
        pilihan.append("wayland")
    if "xcb" not in pilihan:
        pilihan.append("xcb")
    return pilihan


def tunggu_jawaban(proses, antrean, batas_waktu):
    """Tunggu pesan pertama ("siap" / "gagal") dari proses jendela. Bila prosesnya mati duluan
    (Qt bisa langsung menghentikan proses bila platform-nya gagal), hasilnya ("mati", ...)."""
    mulai = time.time()
    while time.time() - mulai < batas_waktu:
        try:
            return antrean.get(timeout=0.3)
        except Exception:
            pass
        if not proses.is_alive():
            try:
                return antrean.get(timeout=0.3)
            except Exception:
                return ("mati", "proses jendela berhenti (exit code %s)" % proses.exitcode)
    return ("gagal", "proses jendela tidak menjawab")


def proses_kontrol(antrean, antrean_masuk, berhenti, platform):
    """Isi proses jendela Kontrol."""
    os.environ["QT_QPA_PLATFORM"] = platform
    try:
        jendela = JendelaKontrol(antrean)
        if jendela.root is None:
            antrean.put(("gagal", jendela.pesan))
            return
        antrean.put(("siap", ""))
        terakhir = dict(jendela.nilai)
        while jendela.root is not None and not berhenti.is_set():
            # Status dari simulasi (ambil yang paling baru saja)
            status = None
            while True:
                try:
                    jenis, isi = antrean_masuk.get_nowait()
                except Exception:
                    break
                if jenis == "status":
                    status = isi
            if status is not None:
                jendela.tampilkan_status(status)

            jendela.perbarui()
            if jendela.nilai != terakhir:
                terakhir = dict(jendela.nilai)
                antrean.put(("nilai", terakhir))
            time.sleep(0.01)
        jendela.tutup()
    except KeyboardInterrupt:
        pass


class PenghubungKontrol:
    """Dipakai Simulasi: menyalakan proses jendela Kontrol dan menerima nilai parameter serta tombolnya."""

    def __init__(self):
        self.awal = nilai_awal_params()
        self.nilai = dict(self.awal)
        self.tombol_baru = []          # tombol B N O P yang baru ditekan dan belum diambil
        self.tahan = {"w": False, "a": False, "s": False, "d": False, "o": False, "p": False}
        self.status_terakhir = None
        self.aktif = False
        self.pesan = ""
        self.platform = None
        konteks = multiprocessing.get_context("spawn")
        catatan = []
        # Coba tiap platform Qt sampai ada yang berhasil membuka jendela.
        for platform in pilihan_platform_qt():
            self.antrean = konteks.Queue()
            self.antrean_keluar = konteks.Queue()
            self.berhenti = konteks.Event()
            self.proses = konteks.Process(target=proses_kontrol,
                                          args=(self.antrean, self.antrean_keluar, self.berhenti, platform),
                                          daemon=True)
            self.proses.start()
            jenis, isi = tunggu_jawaban(self.proses, self.antrean, 20.0)
            if jenis == "siap":
                self.aktif = True
                self.platform = platform
                print("Jendela Kontrol aktif (Qt platform: %s)." % platform)
                return
            catatan.append("%s: %s" % (platform, isi))
            self.berhenti.set()
            self.proses.join(timeout=1.0)
            if jenis == "gagal":
                break                   # misalnya PyQt5 belum terpasang: platform lain juga akan gagal
        self.pesan = " | ".join(catatan)

    def perbarui(self):
        """Ambil semua kiriman baru dari jendela Kontrol."""
        if not self.aktif:
            return
        while True:
            try:
                jenis, isi = self.antrean.get_nowait()
            except Exception:
                break
            if jenis == "nilai":
                self.nilai = isi
            if jenis == "tombol":
                self.tombol_baru.append(isi)
            if jenis == "tahan":
                self.tahan = isi
        if not self.proses.is_alive():
            # Jendela ditutup: nilai parameter terakhir tetap dipakai, tombolnya dianggap lepas.
            self.aktif = False
            for huruf in self.tahan:
                self.tahan[huruf] = False

    def ambil_tombol(self):
        hasil = self.tombol_baru
        self.tombol_baru = []
        return hasil

    def kirim_status(self, status):
        """Kirim status ke jendela Kontrol (hanya bila berubah)."""
        if self.aktif and status != self.status_terakhir:
            self.status_terakhir = status
            self.antrean_keluar.put(("status", status))

    def cetak(self):
        cetak_nilai(self.nilai, self.awal)

    def tutup(self):
        self.berhenti.set()
        if self.proses.is_alive():
            self.proses.join(timeout=2.0)


def proses_plot(antrean, jawaban, berhenti, judul, daftar_backend, platform):
    """Isi proses jendela plot realtime (bentuk dan garis batasnya sama dengan gambar di ise.py):
        Gambar 1 : sumbu X = psi,   sumbu Y = dpsi
        Gambar 2 : sumbu X = theta, sumbu Y = dtheta
        Gambar 3 : sumbu X = s = R*theta, sumbu Y = 0
        bawah    : arus tiap motor terhadap waktu
    Yang digambar hanya data config.PLOT_RENTANG detik terakhir. Titik hitam = keadaan sekarang."""
    os.environ["QT_QPA_PLATFORM"] = platform
    try:
        import numpy as np
        import ise                       # garis batas safety set digambar dengan fungsi yang sama
        import matplotlib.pyplot as plt
        berhasil = False
        for backend in daftar_backend:
            try:
                plt.switch_backend(backend)
                berhasil = True
                break
            except Exception:
                pass
        if not berhasil:
            jawaban.put(("gagal", "matplotlib tidak punya backend jendela (pip install PyQt5)"))
            return

        fig = plt.figure(figsize=(10.0, 9.0))
        kisi = fig.add_gridspec(3, 2, height_ratios=[1.6, 0.5, 0.9])
        gambar1 = fig.add_subplot(kisi[0, 0])
        gambar2 = fig.add_subplot(kisi[0, 1])
        gambar3 = fig.add_subplot(kisi[1, :])
        ax_arus = fig.add_subplot(kisi[2, :])
        try:
            fig.canvas.manager.set_window_title("ATERA - plot realtime " + judul)
        except Exception:
            pass
        ise.gambar_safety_set(gambar1, gambar2)
        ise.gambar_batas_jarak(gambar3)

        # Gambar 1: X = psi, Y = dpsi
        garis_psi, = gambar1.plot([], [], color="tab:blue", linewidth=1.0)
        titik_psi, = gambar1.plot([], [], "o", color="black", markersize=6, label="sekarang")
        gambar1.set_xlabel("psi [rad]")
        gambar1.set_ylabel("dpsi [rad/s]")
        gambar1.set_title("Gambar 1", fontsize=10)
        gambar1.legend(fontsize=7, loc="upper right")

        # Gambar 2: X = theta, Y = dtheta
        garis_theta, = gambar2.plot([], [], color="tab:orange", linewidth=1.0)
        titik_theta, = gambar2.plot([], [], "o", color="black", markersize=6, label="sekarang")
        gambar2.set_xlabel("theta [rad]   (s = R*theta)")
        gambar2.set_ylabel("dtheta [rad/s]")
        gambar2.set_title("Gambar 2", fontsize=10)
        gambar2.legend(fontsize=7, loc="upper right")

        # Gambar 3: X = s, Y = 0
        garis_s, = gambar3.plot([], [], color="tab:purple", linewidth=2.0)
        titik_s, = gambar3.plot([], [], "o", color="black", markersize=6, label="sekarang")
        gambar3.legend(fontsize=7, loc="upper right")

        # Arus terhadap waktu (hanya 2 batas, garis tebal menyambung)
        garis_arus, = ax_arus.plot([], [], color="tab:green")
        ax_arus.axhline(config.MAX_CURRENT_A, color=config.PLOT_SAFETY_WARNA, linewidth=config.PLOT_SAFETY_TEBAL)
        ax_arus.axhline(config.MIN_CURRENT_A, color=config.PLOT_SAFETY_WARNA, linewidth=config.PLOT_SAFETY_TEBAL)
        ax_arus.set_ylim(1.2 * config.MIN_CURRENT_A, 1.2 * config.MAX_CURRENT_A)
        ax_arus.set_ylabel("arus per motor [A]")
        ax_arus.set_xlabel("waktu [s]")

        for ax in (gambar1, gambar2, ax_arus):
            ax.grid(True, alpha=0.4)
        fig.suptitle("data %.0f s terakhir" % config.PLOT_RENTANG, fontsize=9, color="gray")
        fig.tight_layout()
        fig.show()
        jawaban.put(("siap", ""))

        # Batas sumbu minimal: sedikit lebih lebar dari kotak safety set
        ada_jarak = ise.ada_batas_jarak()
        if ada_jarak:
            theta_bawah = config.s_min / config.R
            theta_atas = config.s_max / config.R

        data_t = []
        data_psi = []
        data_dpsi = []
        data_theta = []
        data_dtheta = []
        data_arus = []
        while not berhenti.is_set() and plt.fignum_exists(fig.number):
            # Ambil semua data baru yang dikirim simulasi.
            ada_baru = False
            while True:
                try:
                    kiriman = antrean.get_nowait()
                except Exception:
                    break
                for isi in kiriman:
                    if isi[0] == "RESET":
                        # Robot diletakkan ulang (R, KOMET lalu R, atau zeta diubah): plot mulai dari awal.
                        data_t = []
                        data_psi = []
                        data_dpsi = []
                        data_theta = []
                        data_dtheta = []
                        data_arus = []
                        continue
                    t, psi, dpsi, theta, dtheta, arus = isi
                    data_t.append(t)
                    data_psi.append(psi)
                    data_dpsi.append(dpsi)
                    data_theta.append(theta)
                    data_dtheta.append(dtheta)
                    data_arus.append(arus)
                ada_baru = True
            if ada_baru and len(data_t) == 0:
                for garis in (garis_psi, garis_theta, garis_s, garis_arus, titik_psi, titik_theta, titik_s):
                    garis.set_data([], [])
                ax_arus.set_xlim(0.0, config.PLOT_RENTANG)
                fig.canvas.draw_idle()
            if ada_baru and len(data_t) > 0:
                while len(data_t) > 0 and data_t[0] < data_t[-1] - config.PLOT_RENTANG:
                    data_t.pop(0)
                    data_psi.pop(0)
                    data_dpsi.pop(0)
                    data_theta.pop(0)
                    data_dtheta.pop(0)
                    data_arus.pop(0)
                psi = np.array(data_psi)
                dpsi = np.array(data_dpsi)
                theta = np.array(data_theta)
                dtheta = np.array(data_dtheta)
                jarak = config.R * theta

                garis_psi.set_data(psi, dpsi)
                garis_theta.set_data(theta, dtheta)
                garis_s.set_data(jarak, np.zeros(len(jarak)))
                garis_arus.set_data(data_t, data_arus)
                if not math.isnan(data_psi[-1]):
                    titik_psi.set_data([data_psi[-1]], [data_dpsi[-1]])
                    titik_theta.set_data([data_theta[-1]], [data_dtheta[-1]])
                    titik_s.set_data([config.R * data_theta[-1]], [0.0])

                # Batas sumbu: minimal sedikit lebih lebar dari safety set, melebar bila data keluar.
                if np.any(~np.isnan(psi)):
                    batas_x = max(1.2 * config.psi_max, 1.1 * np.nanmax(np.abs(psi)))
                    batas_y = max(1.2 * config.dpsi_max, 1.1 * np.nanmax(np.abs(dpsi)))
                    gambar1.set_xlim(-batas_x, batas_x)
                    gambar1.set_ylim(-batas_y, batas_y)

                    bawah = np.nanmin(theta)
                    atas = np.nanmax(theta)
                    if ada_jarak:                       # kotak batas jarak selalu terlihat
                        bawah = min(bawah, theta_bawah)
                        atas = max(atas, theta_atas)
                    lebar = max(1.0, atas - bawah)
                    gambar2.set_xlim(bawah - 0.1 * lebar, atas + 0.1 * lebar)
                    batas_y = max(1.2 * config.dtheta_max, 1.1 * np.nanmax(np.abs(dtheta)))
                    gambar2.set_ylim(-batas_y, batas_y)

                    bawah = config.R * (bawah - 0.1 * lebar)
                    atas = config.R * (atas + 0.1 * lebar)
                    gambar3.set_xlim(bawah, atas)
                ax_arus.set_xlim(max(0.0, data_t[-1] - config.PLOT_RENTANG), max(config.PLOT_RENTANG, data_t[-1]))
                fig.canvas.draw_idle()
            fig.canvas.flush_events()
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass


class JendelaKontrol:
    """Jendela Qt "ATERA - Kontrol":
        - bagian atas : tombol W A S D, B N, O P (bisa diklik / ditahan dengan mouse, atau ditekan di keyboard)
        - bagian bawah: kolom angka tiap parameter (dua blok: gain + lingkungan, dan Alpha). Kolom berwarna = beda dari config.py

    Nilai awal parameter dari config.py. Nilai di jendela ini TIDAK ditulis ke config.py; salin sendiri
    dari hasil tombol "Cetak nilai ke terminal".
    """

    def __init__(self, antrean=None):
        self.antrean = antrean         # jalur kirim ke simulasi (None saat diuji sendiri)
        self.awal = nilai_awal_params()
        self.nilai = dict(self.awal)

        # Tombol ditahan, dipisah menurut sumbernya. Ditahan = keyboard ATAU mouse.
        self.tahan_keyboard = {}
        self.tahan_mouse = {}
        for huruf in ("w", "a", "s", "d", "o", "p", "b", "n", "q", "r", "e", "t"):
            self.tahan_keyboard[huruf] = False
            self.tahan_mouse[huruf] = False
        self.tahan_terkirim = None
        self.status = {"judul": "", "mode_balancing": True, "tahan": {}, "dorong": "dorong: -", "arah_dorong": 0,
                       "komet": False, "mode_pengujian": False, "pengujian": ""}

        self.root = None               # jendela Qt; None bila gagal dibuat atau sudah ditutup
        self.pesan = ""
        self.isian = {}
        self.tombol = {}

        try:
            from PyQt5 import QtCore, QtWidgets
            self.QtCore = QtCore
            self.QtWidgets = QtWidgets
            self.app = QtWidgets.QApplication.instance()
            if self.app is None:
                self.app = QtWidgets.QApplication(["ATERA - Kontrol"])
        except Exception as kesalahan:
            self.pesan = "%s: %s" % (type(kesalahan).__name__, kesalahan)
            return

        jendela = QtWidgets.QWidget()
        jendela.setWindowTitle("ATERA - Kontrol")
        susunan = QtWidgets.QVBoxLayout(jendela)

        # ------------------------------------------------ bagian tombol
        self.label_mode = QtWidgets.QLabel("")
        self.label_mode.setStyleSheet("font-weight: bold")
        susunan.addWidget(self.label_mode)

        kisi_tombol = QtWidgets.QGridLayout()
        susunan.addLayout(kisi_tombol)
        letak = {"w": (0, 1), "a": (1, 0), "s": (1, 1), "d": (1, 2),
                 "b": (0, 4), "n": (0, 5), "o": (1, 4), "p": (1, 5), "q": (0, 7), "r": (1, 7), "e": (0, 8), "t": (1, 8)}
        keterangan = {"w": "W\nmaju", "a": "A\nkiri", "s": "S\nmundur", "d": "D\nkanan",
                      "b": "B\nBalancing", "n": "N\nNavigate", "o": "O\ndorong depan", "p": "P\ndorong belakang",
                      "q": "Q\nkeluar + simpan", "r": "R\nreset robot", "e": "E\nsimpan data",
                      "t": "T\nmode pengujian"}
        for huruf in ("w", "a", "s", "d", "b", "n", "o", "p", "q", "r", "e", "t"):
            tombol = QtWidgets.QPushButton(keterangan[huruf])
            tombol.setFixedSize(92, 46)
            tombol.setFocusPolicy(QtCore.Qt.NoFocus)        # tombol tidak merebut keyboard
            tombol.pressed.connect(lambda h=huruf: self.atur_tombol(h, True, "mouse"))
            tombol.released.connect(lambda h=huruf: self.atur_tombol(h, False, "mouse"))
            kisi_tombol.addWidget(tombol, letak[huruf][0], letak[huruf][1])
            self.tombol[huruf] = tombol
        kisi_tombol.setColumnMinimumWidth(3, 24)
        kisi_tombol.setColumnMinimumWidth(6, 24)
        self.tombol["q"].setStyleSheet("color: #b3261e")
        self.tombol["r"].setStyleSheet("color: #b3261e")

        self.label_gerak = QtWidgets.QLabel("")
        self.label_dorong = QtWidgets.QLabel("")
        baris_label = QtWidgets.QHBoxLayout()
        baris_label.addWidget(self.label_gerak)
        baris_label.addStretch(1)
        baris_label.addWidget(self.label_dorong)
        susunan.addLayout(baris_label)

        petunjuk = QtWidgets.QLabel("Keyboard dibaca saat jendela ini aktif. Setelah mengetik angka, tekan Enter "
                                    "supaya keyboard kembali dipakai untuk tombol.")
        petunjuk.setStyleSheet("color: gray")
        susunan.addWidget(petunjuk)

        garis = QtWidgets.QFrame()
        garis.setFrameShape(QtWidgets.QFrame.HLine)
        susunan.addWidget(garis)

        # ------------------------------------------------ bagian parameter (dua blok berdampingan)
        # Blok kiri: gain PD, Kvel, gaya dorong, zeta. Blok kanan: semua Alpha.
        kisi = QtWidgets.QGridLayout()
        kisi.setHorizontalSpacing(6)
        kisi.setVerticalSpacing(3)
        susunan.addLayout(kisi)

        blok = [[nama for nama, _ in DAFTAR_PARAMS if nama not in NAMA_ALPHA], list(NAMA_ALPHA)]
        for b in range(2):
            kolom = b * 5                       # per blok: Parameter | Nilai | config.py | reset | jarak
            for k, judul in enumerate(("Parameter", "Nilai", "config.py")):
                label = QtWidgets.QLabel(judul)
                label.setStyleSheet("color: gray")
                label.setAlignment(QtCore.Qt.AlignRight if k > 0 else QtCore.Qt.AlignLeft)
                kisi.addWidget(label, 0, kolom + k)
            for i, nama in enumerate(blok[b]):
                baris = i + 1
                kisi.addWidget(QtWidgets.QLabel(nama), baris, kolom)

                isian = QtWidgets.QLineEdit()
                isian.setFixedWidth(84)
                isian.setAlignment(QtCore.Qt.AlignRight)
                isian.editingFinished.connect(lambda n=nama: self.dari_isian(n))
                isian.returnPressed.connect(lambda: self.lepas_fokus())
                kisi.addWidget(isian, baris, kolom + 1)
                self.isian[nama] = isian

                label_config = QtWidgets.QLabel(self.format(nama, self.awal[nama]))
                label_config.setStyleSheet("color: gray")
                label_config.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
                kisi.addWidget(label_config, baris, kolom + 2)

                # Tombol reset per parameter: kembali ke nilai config.py
                tombol_reset = QtWidgets.QPushButton("reset")
                tombol_reset.setFixedWidth(46)
                tombol_reset.setFocusPolicy(QtCore.Qt.NoFocus)
                tombol_reset.setToolTip("Kembalikan %s ke nilai config.py" % nama)
                tombol_reset.clicked.connect(lambda dicentang=False, n=nama: self.atur(n, self.awal[n]))
                kisi.addWidget(tombol_reset, baris, kolom + 3)
        kisi.setColumnMinimumWidth(4, 28)
        baris = max(len(blok[0]), len(blok[1])) + 1

        tombol_cetak = QtWidgets.QPushButton("Cetak nilai ke terminal")
        tombol_cetak.setFocusPolicy(QtCore.Qt.NoFocus)
        tombol_cetak.clicked.connect(lambda: self.cetak())
        tombol_kembali = QtWidgets.QPushButton("Reset SEMUA ke config.py")
        tombol_kembali.setFocusPolicy(QtCore.Qt.NoFocus)
        tombol_kembali.clicked.connect(lambda: self.kembalikan())
        kisi.addWidget(tombol_cetak, baris, 0, 1, 4)
        kisi.addWidget(tombol_kembali, baris, 5, 1, 4)

        # Keyboard: semua tombol keyboard di jendela ini disaring dulu oleh saring_keyboard().
        class Saringan(QtCore.QObject):
            def __init__(self, pemilik):
                super().__init__()
                self.pemilik = pemilik

            def eventFilter(self, objek, kejadian):
                return self.pemilik.saring_keyboard(kejadian)

        self.saringan = Saringan(self)
        self.app.installEventFilter(self.saringan)
        self.kode_tombol = {QtCore.Qt.Key_W: "w", QtCore.Qt.Key_A: "a", QtCore.Qt.Key_S: "s",
                            QtCore.Qt.Key_D: "d", QtCore.Qt.Key_B: "b", QtCore.Qt.Key_N: "n",
                            QtCore.Qt.Key_O: "o", QtCore.Qt.Key_P: "p", QtCore.Qt.Key_Q: "q",
                            QtCore.Qt.Key_R: "r", QtCore.Qt.Key_E: "e", QtCore.Qt.Key_T: "t"}

        self.root = jendela
        for nama in self.nilai:
            self.tampilkan_nilai(nama)
        self.tampilkan_status(self.status)
        jendela.show()
        self.perbarui()

    # ------------------------------------------------ tombol
    def kirim(self, pesan):
        if self.antrean is not None:
            self.antrean.put(pesan)

    def saring_keyboard(self, kejadian):
        """Mengembalikan True bila tombol keyboard dipakai di sini (tidak diteruskan ke widget)."""
        QEvent = self.QtCore.QEvent
        jenis = kejadian.type()
        if jenis != QEvent.KeyPress and jenis != QEvent.KeyRelease:
            return False
        # Saat mengetik angka di kolom parameter, huruf tidak dipakai sebagai tombol.
        if isinstance(self.app.focusWidget(), self.QtWidgets.QLineEdit):
            return False
        huruf = self.kode_tombol.get(kejadian.key())
        if huruf is None:
            return False
        if kejadian.isAutoRepeat():
            return True                 # pengulangan otomatis keyboard saat tombol ditahan: abaikan
        self.atur_tombol(huruf, jenis == QEvent.KeyPress, "keyboard")
        return True

    def atur_tombol(self, huruf, ditekan, sumber):
        if sumber == "keyboard":
            sebelumnya = self.tahan_keyboard[huruf] or self.tahan_mouse[huruf]
            self.tahan_keyboard[huruf] = ditekan
        else:
            sebelumnya = self.tahan_keyboard[huruf] or self.tahan_mouse[huruf]
            self.tahan_mouse[huruf] = ditekan
        # B N O P Q R E T: dikirim sekali saat mulai ditekan
        if ditekan and not sebelumnya and huruf in ("b", "n", "o", "p", "q", "r", "e", "t"):
            self.kirim(("tombol", huruf))
        self.kirim_tahan()
        self.tampilkan_status(self.status)

    def tahan_sekarang(self):
        hasil = {}
        for huruf in ("w", "a", "s", "d", "o", "p"):
            hasil[huruf] = self.tahan_keyboard[huruf] or self.tahan_mouse[huruf]
        return hasil

    def kirim_tahan(self):
        tahan = self.tahan_sekarang()
        if tahan != self.tahan_terkirim:
            self.tahan_terkirim = tahan
            self.kirim(("tahan", tahan))

    def lepas_semua_keyboard(self):
        """Dipanggil saat jendela tidak aktif: tombol keyboard yang 'dilepas' di jendela lain tidak terbaca,
        jadi semua dianggap lepas supaya robot tidak terus berjalan."""
        ada = False
        for huruf in self.tahan_keyboard:
            if self.tahan_keyboard[huruf]:
                self.tahan_keyboard[huruf] = False
                ada = True
        if ada:
            self.kirim_tahan()
            self.tampilkan_status(self.status)

    def lepas_fokus(self):
        """Setelah Enter di kolom angka, fokus dilepas supaya huruf kembali dipakai untuk tombol."""
        if self.root is not None:
            self.root.setFocus()

    def gaya_tombol(self, latar, tulisan):
        if latar == "":
            return "color: %s" % tulisan
        return "background-color: %s; color: %s; font-weight: bold" % (latar, tulisan)

    def tampilkan_status(self, status):
        """Nyalakan tombol dan tulis mode / gerak / dorongan.
        status dari simulasi: judul, mode_balancing, tahan (gabungan Qt + keyboard PyBullet), dorong, arah_dorong"""
        self.status = status
        if self.root is None and self.tombol == {}:
            return
        mode_balancing = status["mode_balancing"]
        tahan = dict(status["tahan"])
        sendiri = self.tahan_sekarang()
        for huruf in sendiri:
            tahan[huruf] = tahan.get(huruf, False) or sendiri[huruf]

        for huruf in ("w", "a", "s", "d"):
            if mode_balancing:
                self.tombol[huruf].setStyleSheet(self.gaya_tombol("#dddddd" if tahan[huruf] else "", config.WARNA_QT_ABAI))
            elif tahan[huruf]:
                self.tombol[huruf].setStyleSheet(self.gaya_tombol(config.WARNA_QT_TEKAN, "white"))
            else:
                self.tombol[huruf].setStyleSheet("")
        self.tombol["b"].setStyleSheet(self.gaya_tombol(config.WARNA_QT_MODE, "white") if mode_balancing else "")
        self.tombol["n"].setStyleSheet("" if mode_balancing else self.gaya_tombol(config.WARNA_QT_MODE, "white"))
        menyala_o = tahan["o"] or status["arah_dorong"] > 0
        menyala_p = tahan["p"] or status["arah_dorong"] < 0
        self.tombol["o"].setStyleSheet(self.gaya_tombol(config.WARNA_QT_TEKAN, "white") if menyala_o else "")
        self.tombol["p"].setStyleSheet(self.gaya_tombol(config.WARNA_QT_TEKAN, "white") if menyala_p else "")

        self.tombol["t"].setStyleSheet(self.gaya_tombol(config.WARNA_QT_UJI, "white")
                                       if status.get("mode_pengujian", False) else "")
        if status.get("komet", False):
            self.label_mode.setText("komet, motor mati  ->  R: ulang   E: simpan data   Q: keluar + simpan")
            self.label_mode.setStyleSheet("font-weight: bold; color: #b3261e")
        elif status.get("mode_pengujian", False):
            self.label_mode.setText(status.get("pengujian", "MODE PENGUJIAN"))
            self.label_mode.setStyleSheet("font-weight: bold; color: %s" % config.WARNA_QT_UJI)
        else:
            self.label_mode.setText("Mode kontrol: %s   |   %s" % (status["judul"],
                                    "Mode Balancing" if mode_balancing else "Mode Navigate"))
            self.label_mode.setStyleSheet("font-weight: bold")
        if mode_balancing:
            ada = tahan["w"] or tahan["a"] or tahan["s"] or tahan["d"]
            self.label_gerak.setText("W A S D diabaikan (Mode Balancing)" if ada else "W A S D tidak aktif")
        else:
            gerak = []
            if tahan["w"] and not tahan["s"]:
                gerak.append("maju")
            if tahan["s"] and not tahan["w"]:
                gerak.append("mundur")
            if tahan["a"] and not tahan["d"]:
                gerak.append("kiri")
            if tahan["d"] and not tahan["a"]:
                gerak.append("kanan")
            self.label_gerak.setText("gerak: " + (" + ".join(gerak) if len(gerak) > 0 else "diam"))
        self.label_dorong.setText("%s   (%.1f N x %.2f s)" % (status["dorong"], self.nilai["Gaya_dorong"],
                                                              config.LAMA_DORONG))

    # ------------------------------------------------ parameter
    def format(self, nama, nilai):
        return tulis_angka(nama, nilai)

    def tampilkan_nilai(self, nama):
        """Samakan kolom angka dengan self.nilai[nama]; kolom diberi warna bila beda dari config.py."""
        self.isian[nama].setText(self.format(nama, self.nilai[nama]))
        berubah = abs(self.nilai[nama] - self.awal[nama]) > 1e-9
        self.isian[nama].setStyleSheet("background-color: #ffe3b3; color: black" if berubah else "")

    def dari_isian(self, nama):
        teks = self.isian[nama].text().strip().replace(",", ".")
        try:
            nilai = float(teks)
        except ValueError:
            self.tampilkan_nilai(nama)  # bukan angka: kembalikan tulisan ke nilai terakhir
            return
        self.atur(nama, nilai)

    def atur(self, nama, nilai):
        self.nilai[nama] = nilai
        self.tampilkan_nilai(nama)
        if nama == "Gaya_dorong":
            self.tampilkan_status(self.status)

    def kembalikan(self):
        for nama in self.nilai:
            self.atur(nama, self.awal[nama])
        print("Parameter dikembalikan ke nilai config.py.", flush=True)

    def daftar_berubah(self):
        hasil = []
        for nama in self.nilai:
            if abs(self.nilai[nama] - self.awal[nama]) > 1e-9:
                hasil.append(nama)
        return hasil

    def cetak(self):
        cetak_nilai(self.nilai, self.awal)

    def perbarui(self):
        """Proses klik / ketikan di jendela Kontrol."""
        if self.root is None:
            return
        self.app.processEvents()
        if not self.root.isVisible():
            self.root = None            # jendela ditutup: nilai terakhir tetap dipakai
            return
        if not self.root.isActiveWindow():
            self.lepas_semua_keyboard()

    def tutup(self):
        if self.root is not None:
            self.root.close()
            self.app.processEvents()
            self.root = None


# =============================================================================
# Simulasi
# =============================================================================
class Simulasi:
    def __init__(self, gui=True, plot=False, judul=""):
        self.gui = gui
        self.dt_fisika = config.DT / config.SUBSTEP
        self.zeta = config.zeta
        self.judul = judul

        p.connect(p.GUI if gui else p.DIRECT)
        p.setGravity(0.0, 0.0, -config.g)
        p.setPhysicsEngineParameter(fixedTimeStep=self.dt_fisika, numSolverIterations=50)
        if gui:
            # Shortcut bawaan PyBullet dimatikan supaya tombol W A S D B N O P tidak bentrok,
            # dan panel kiri / kanan bawaan PyBullet disembunyikan.
            p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

        # Dunia default PyBullet: plane.urdf. Untuk bidang miring, plane diputar sebesar zeta terhadap
        # titik (0, 0, 0) sehingga menanjak ke arah depan (+Y). zeta bisa diubah saat berjalan (ubah_zeta).
        self.normal = np.array([0.0, -math.sin(self.zeta), math.cos(self.zeta)])
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.lantai = p.loadURDF("plane.urdf", basePosition=[0.0, 0.0, 0.0],
                                 baseOrientation=p.getQuaternionFromEuler([self.zeta, 0.0, 0.0]))
        p.changeDynamics(self.lantai, -1, lateralFriction=config.GESEKAN_RODA)

        # Robot
        self.path_urdf = cari_urdf()
        self.robot = p.loadURDF(self.path_urdf, flags=p.URDF_USE_INERTIA_FROM_FILE)

        self.joint_kiri = None
        self.joint_kanan = None
        for j in range(p.getNumJoints(self.robot)):
            nama_link = p.getJointInfo(self.robot, j)[12].decode()
            if "Lwheel" in nama_link:
                self.joint_kiri = j
            if "Rwheel" in nama_link:
                self.joint_kanan = j
        if self.joint_kiri is None or self.joint_kanan is None:
            raise RuntimeError("Joint roda kiri/kanan tidak ditemukan di URDF")

        # PyBullet menyimpan posisi badan di frame PUSAT MASSA (frame inersia), yang letak dan arahnya
        # berbeda dari frame body_link di URDF. Simpan selisihnya supaya bisa diubah bolak-balik.
        info_badan = p.getDynamicsInfo(self.robot, -1)
        massa_badan = info_badan[0]
        self.inersia_pos = info_badan[3]
        self.inersia_orn = info_badan[4]
        self.inersia_balik = p.invertTransform(self.inersia_pos, self.inersia_orn)

        # Titik tengah poros roda di frame body_link (dari posisi kedua joint roda).
        titik = []
        for joint in (self.joint_kiri, self.joint_kanan):
            pos_joint = p.getJointInfo(self.robot, joint)[14]          # di frame inersia badan
            pos_link, _ = p.multiplyTransforms(self.inersia_pos, self.inersia_orn, pos_joint, [0, 0, 0, 1])
            titik.append(pos_link)
        self.poros_badan = 0.5 * (np.array(titik[0]) + np.array(titik[1]))

        # psi di model.py adalah sudut garis poros -> pusat massa badan terhadap vertikal. Di URDF pusat massa
        # badan bergeser sedikit ke depan/belakang (sumbu z body_link), jadi garis itu tidak tepat sejajar
        # sumbu y body_link. Selisih sudutnya disimpan di sini (sama seperti kalibrasi nol IMU di robot asli).
        garis = np.array(self.inersia_pos) - self.poros_badan
        self.sudut_com = math.atan2(garis[2], garis[1])

        # Massa di robot.urdf harus sama dengan config.py, karena CBF-QP memakai model dari config.py.
        massa_roda = p.getDynamicsInfo(self.robot, self.joint_kiri)[0]
        if abs(massa_badan - config.m_b) > 0.001 or abs(massa_roda - config.m_r) > 0.001:
            print("PERHATIAN: massa di robot.urdf (badan %.3f kg, roda %.3f kg) berbeda dengan config.py."
                  % (massa_badan, massa_roda))

        for link in (-1, self.joint_kiri, self.joint_kanan):
            p.changeDynamics(self.robot, link, linearDamping=0.0, angularDamping=0.0, jointDamping=0.0)
        for joint in (self.joint_kiri, self.joint_kanan):
            p.changeDynamics(self.robot, joint, lateralFriction=config.GESEKAN_RODA, spinningFriction=config.GESEKAN_PUTAR)
            # Matikan motor bawaan PyBullet supaya roda hanya digerakkan torsi kita.
            p.setJointMotorControl2(self.robot, joint, p.VELOCITY_CONTROL, force=0.0)

        # Dorongan
        self.gaya_dorong = float(config.GAYA_DORONG_AWAL)
        self.sisa_dorong = 0.0
        self.arah_dorong = 0

        # Keyboard
        self.arah = 0                  # +1 maju (W), -1 mundur (S)
        self.belok = 0                 # -1 kiri (A), +1 kanan (D)
        self.ditahan = {"w": False, "a": False, "s": False, "d": False, "o": False, "p": False}

        # Tampilan di jendela PyBullet
        self.id_teks = {}
        self.isi_teks = {}
        self.hitung_tampil = 0
        self.jangkar = None
        self.setengah_lebar = config.JARAK_TEKS
        self.setengah_tinggi = config.JARAK_TEKS
        self.giliran_baris = 0
        self.bidang_depan = None
        self.bidang_belakang = None
        self.bidang_langgar = {"depan": False, "belakang": False}
        self.kubus_jarak = None
        self.kubus_langgar = False

        # Jendela Kontrol
        self.kontrol = None
        self.nilai_params = nilai_awal_params()

        # Data untuk plot realtime
        self.waktu = 0.0
        self.x_terakhir = np.zeros(4)
        self.plot = None               # proses jendela plot (hanya dengan --plot)
        self.plot_antrean = None       # jalur kirim data ke proses plot
        self.plot_berhenti = None
        self.plot_kiriman = []         # data yang belum dikirim

        # Realisme dari uji hardware (config.py bagian 11)
        self.skala_torsi = config.MOTOR_KT_EFEKTIF / config.MOTOR_KT     # torsi nyata / torsi perintah
        self.jeda_substep = int(round(config.SIM_JEDA_TORSI / self.dt_fisika))
        self.antrean_torsi = []        # torsi yang sudah diperintah tetapi belum bekerja (satu per langkah fisika)
        self.kalman = kalman.KalmanAngle()
        self.psi_imu = 0.0             # [rad]   psi hasil Kalman (yang dibaca kontrol)
        self.dpsi_imu = 0.0            # [rad/s] dpsi dari gyro
        self.v_imu_lama = np.zeros(3)  # kecepatan titik IMU siklus lalu (untuk percepatan)
        self.acak = np.random.default_rng()

        self.reset()

        # Tinggi pusat bidang dorong dari poros roda: permukaan teratas badan - setengah sisi bidang.
        bawah_aabb, atas_aabb = p.getAABB(self.robot, -1)
        posisi_link, rot = self.pose_badan()
        poros = posisi_link + rot @ self.poros_badan
        self.tinggi_atas = atas_aabb[2] - poros[2]
        self.tinggi_dorong = self.tinggi_atas - 0.5 * config.DORONG_SISI
        self.bidang_dorong_visual = {}
        self.dorong_menyala = {1: None, -1: None}

        if gui:
            print("Realisme: torsi roda x%.2f (K efektif %.3f N m/A), jeda torsi %.1f ms, model IMU + Kalman %s."
                  % (self.skala_torsi, config.MOTOR_KT_EFEKTIF, self.jeda_substep * self.dt_fisika * 1000,
                     "AKTIF" if config.SIM_MODEL_IMU else "MATI (psi sempurna)"))
            print("Titik dorong O / P: bidang %.0f x %.0f mm, pusatnya %.1f mm di atas poros roda "
                  "(puncak badan %.1f mm), %.1f mm dari garis tengah badan."
                  % (config.DORONG_SISI * 1000, config.DORONG_SISI * 1000, self.tinggi_dorong * 1000, self.tinggi_atas * 1000,
                     config.JARAK_IMU_KE_SISI_LUAR * 1000))
            posisi, _ = p.getBasePositionAndOrientation(self.robot)
            p.resetDebugVisualizerCamera(cameraDistance=config.KAMERA_JARAK, cameraYaw=config.KAMERA_YAW,
                                         cameraPitch=config.KAMERA_PITCH, cameraTargetPosition=posisi)
            self.buat_bidang_safety_set()
            self.buat_kubus_jarak()
            self.buat_bidang_dorong()
            self.buat_jangkar_teks()
            self.kontrol = PenghubungKontrol()
            if not self.kontrol.aktif:
                print("Jendela Kontrol tidak aktif (%s)." % self.kontrol.pesan)
                print("Cek: pip install PyQt5  dan untuk xcb:  sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 "
                      "libxcb-icccm4 libxcb-image0 libxcb-keysyms1 libxcb-randr0 libxcb-render-util0 "
                      "libxcb-shape0 libxcb-xfixes0")
                print("Simulasi memakai gain dari config.py. Tombol: keyboard di jendela PyBullet.")
            if plot:
                self.buat_plot()

    # ------------------------------------------------------------------ robot
    def reset(self):
        """Letakkan robot tegak di atas lantai dengan roda diam."""
        psi0 = math.radians(config.PSI_AWAL_DEG)
        poros = (config.R + 0.001) * self.normal
        # URDF memakai sumbu y sebagai atas, PyBullet memakai sumbu z -> putar 90 derajat di sumbu x.
        # Lalu diputar 180 derajat di sumbu z supaya depan robot (+Z body_link) menghadap +Y dunia.
        orientasi_link = p.getQuaternionFromEuler([math.pi / 2.0 + psi0 - self.sudut_com, 0.0, math.pi])
        rot = np.array(p.getMatrixFromQuaternion(orientasi_link)).reshape(3, 3)
        posisi_link = poros - rot @ self.poros_badan
        # Ubah dari frame body_link ke frame pusat massa yang dipakai PyBullet.
        posisi, orientasi = p.multiplyTransforms(posisi_link.tolist(), orientasi_link,
                                                 self.inersia_pos, self.inersia_orn)
        p.resetBasePositionAndOrientation(self.robot, posisi, orientasi)
        p.resetBaseVelocity(self.robot, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
        p.resetJointState(self.robot, self.joint_kiri, 0.0, 0.0)
        p.resetJointState(self.robot, self.joint_kanan, 0.0, 0.0)
        self.sisa_dorong = 0.0
        self.reset_plot()

        # Tanda tiap joint: +1 bila putaran positif joint membuat roda maju.
        self.tanda_kiri = self.tanda_joint(self.joint_kiri)
        self.tanda_kanan = self.tanda_joint(self.joint_kanan)

        # Belum ada torsi yang menunggu, IMU mulai dari sudut sebenarnya (seperti kalibrasi di robot nyata).
        self.antrean_torsi = []
        for _ in range(self.jeda_substep):
            self.antrean_torsi.append((0.0, 0.0))
        self.mulai_imu()

        # Titik nol s (dan theta_setpoint = 0) = posisi robot diletakkan.
        self.theta_nol = 0.0
        self.theta_nol = self.baca_state()[0]

    def ubah_zeta(self, zeta):
        """Ganti kemiringan lantai [rad] lalu letakkan ulang robot di posisi awal."""
        self.zeta = float(zeta)
        self.normal = np.array([0.0, -math.sin(self.zeta), math.cos(self.zeta)])
        p.resetBasePositionAndOrientation(self.lantai, [0.0, 0.0, 0.0],
                                          p.getQuaternionFromEuler([self.zeta, 0.0, 0.0]))
        self.atur_kubus_jarak()
        self.reset()

    def tinggi_lantai(self, titik):
        """Tinggi (z) permukaan lantai di bawah titik (x, y). Lantai melewati titik (0, 0, 0)."""
        return -(self.normal[0] * titik[0] + self.normal[1] * titik[1]) / self.normal[2]

    def pose_badan(self):
        """Posisi titik nol body_link dan matriks rotasinya (koordinat dunia)."""
        posisi, orientasi = p.getBasePositionAndOrientation(self.robot)       # frame pusat massa
        posisi_link, orientasi_link = p.multiplyTransforms(posisi, orientasi,
                                                           self.inersia_balik[0], self.inersia_balik[1])
        rot = np.array(p.getMatrixFromQuaternion(orientasi_link)).reshape(3, 3)
        return np.array(posisi_link), rot

    def sumbu_badan(self):
        """Arah sumbu roda, arah atas badan, dan arah depan (semua di koordinat dunia)."""
        _, rot = self.pose_badan()
        sumbu_roda = rot[:, 0]             # sumbu x body_link, mengarah ke roda kiri
        atas_badan = rot[:, 1]
        depan = np.cross(sumbu_roda, [0.0, 0.0, 1.0])
        depan = depan / np.linalg.norm(depan)
        return sumbu_roda, atas_badan, depan

    def tanda_joint(self, joint):
        sumbu_roda, _, _ = self.sumbu_badan()
        sumbu_lokal = p.getJointInfo(self.robot, joint)[13]
        orientasi_link = p.getLinkState(self.robot, joint)[5]
        rot = np.array(p.getMatrixFromQuaternion(orientasi_link)).reshape(3, 3)
        sumbu_dunia = rot @ np.array(sumbu_lokal)
        # Roda maju = berputar terhadap arah +sumbu_roda (kaidah tangan kanan).
        return 1.0 if np.dot(sumbu_dunia, sumbu_roda) > 0.0 else -1.0

    def baca_psi_benar(self):
        """psi dan dpsi SEBENARNYA dari PyBullet (tanpa kesalahan sensor)."""
        sumbu_roda, atas_badan, depan = self.sumbu_badan()
        _, kecepatan_sudut = p.getBaseVelocity(self.robot)
        psi = math.atan2(np.dot(atas_badan, depan), atas_badan[2]) + self.sudut_com
        dpsi = float(np.dot(kecepatan_sudut, sumbu_roda))
        return psi, dpsi

    def baca_state(self):
        """State x = [theta, dtheta, psi, dpsi] seperti yang dibaca robot nyata (IMU + encoder).

        Dengan config.SIM_MODEL_IMU = True, psi dari Kalman dan dpsi dari gyro (lihat perbarui_imu).
        theta dari model.hitung_theta (encoder + psi Kalman), dikurangi theta_nol (titik nol s).
        """
        psi_benar, dpsi_benar = self.baca_psi_benar()
        if config.SIM_MODEL_IMU:
            psi = self.psi_imu
            dpsi = self.dpsi_imu
        else:
            psi = psi_benar
            dpsi = dpsi_benar

        kiri = p.getJointState(self.robot, self.joint_kiri)
        kanan = p.getJointState(self.robot, self.joint_kanan)
        # Joint (seperti encoder DDSM115) mengukur roda relatif terhadap badan.
        theta_encoder = 0.5 * (self.tanda_kiri * kiri[0] + self.tanda_kanan * kanan[0])
        dtheta_encoder = 0.5 * (self.tanda_kiri * kiri[1] + self.tanda_kanan * kanan[1])

        theta = model.hitung_theta(theta_encoder, psi) - self.theta_nol
        dtheta = model.hitung_theta(dtheta_encoder, dpsi)
        self.x_terakhir = np.array([theta, dtheta, psi, dpsi])
        return self.x_terakhir

    # -------------------------------------------------------------- model IMU
    def titik_imu(self):
        """Letak IMU (dunia). ASUMSI: IMU di garis tengah badan, sejauh L_IMU dari poros searah sumbu atas badan."""
        posisi_link, rot = self.pose_badan()
        poros = posisi_link + rot @ self.poros_badan
        return poros + config.L_IMU * rot[:, 1]

    def kecepatan_imu(self):
        """Kecepatan titik IMU (dunia) = v pusat massa badan + omega x (r_IMU - r_pusat_massa)."""
        pusat, _ = p.getBasePositionAndOrientation(self.robot)
        v, omega = p.getBaseVelocity(self.robot)
        lengan = self.titik_imu() - np.array(pusat)
        return np.array(v) + np.cross(np.array(omega), lengan)

    def mulai_imu(self):
        """Kalman baru, sudut awalnya = psi sebenarnya (seperti setelah kalibrasi di robot nyata)."""
        psi_benar, dpsi_benar = self.baca_psi_benar()
        self.kalman = kalman.KalmanAngle()
        self.kalman.setAngle(math.degrees(psi_benar))
        self.psi_imu = psi_benar
        self.dpsi_imu = dpsi_benar
        self.v_imu_lama = self.kecepatan_imu()

    def perbarui_imu(self):
        """Satu sampel MPU6050 + Kalman per siklus kontrol, sama seperti mpu6050.read() di robot nyata.

        Akselerometer mengukur (percepatan titik IMU - gravitasi), jadi ikut terganggu saat robot
        berakselerasi. Gyro mengukur kecepatan sudut badan. Keduanya diberi noise hasil uji.
        """
        v_imu = self.kecepatan_imu()
        percepatan = (v_imu - self.v_imu_lama) / config.DT
        self.v_imu_lama = v_imu
        gaya_spesifik = percepatan + np.array([0.0, 0.0, config.g])

        sumbu_roda, atas_badan, depan = self.sumbu_badan()
        maju_badan = np.cross(sumbu_roda, atas_badan)
        maju_badan = maju_badan / np.linalg.norm(maju_badan)
        if np.dot(maju_badan, depan) < 0.0:
            maju_badan = -maju_badan
        # Sumbu sensor: ax ke depan badan, ay searah poros roda, az ke atas badan (IMU_AXIS = "pitch").
        ax = float(np.dot(gaya_spesifik, maju_badan))
        ay = float(np.dot(gaya_spesifik, sumbu_roda))
        az = float(np.dot(gaya_spesifik, atas_badan))
        psi_akselerometer = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)) + self.sudut_com)
        psi_akselerometer = psi_akselerometer + self.acak.normal(0.0, config.SIM_ACC_NOISE_DEG)

        _, dpsi_benar = self.baca_psi_benar()
        gyro = math.degrees(dpsi_benar) + self.acak.normal(0.0, config.SIM_GYRO_NOISE_DEG_S)

        psi_kalman = self.kalman.getAngle(psi_akselerometer, gyro, config.DT)
        self.psi_imu = math.radians(psi_kalman)
        self.dpsi_imu = math.radians(gyro)

    def nolkan_theta(self):
        """Titik nol s dipindah ke posisi sekarang (dipanggil setiap robot masuk mode Balancing)."""
        self.theta_nol = self.theta_nol + self.baca_state()[0]
        self.putus_garis_plot()

    def reset_plot(self):
        """Plot realtime dikosongkan dan waktunya mulai lagi dari 0 (dipanggil setiap robot diletakkan ulang)."""
        self.waktu = 0.0
        if self.plot is not None:
            self.plot_kiriman = [("RESET",)]

    def putus_garis_plot(self):
        """Garis di plot realtime diputus (robot diletakkan ulang atau theta dinolkan), supaya tidak ada
        garis lurus yang melompat dari keadaan lama ke keadaan baru."""
        if self.plot is not None:
            kosong = float("nan")
            self.plot_kiriman.append((self.waktu, kosong, kosong, kosong, kosong, kosong))

    def dorong(self, arah):
        """arah = +1 didorong ke depan (O), -1 didorong ke belakang (P)."""
        self.arah_dorong = arah
        self.sisa_dorong = config.LAMA_DORONG

    def bidang_dorong(self, sisi):
        """Letak bidang dorong. sisi = +1 bidang di sisi depan badan, -1 di sisi belakang.
        Mengembalikan pusat bidang, sumbu roda, sumbu atas badan, dan arah depan badan (dunia)."""
        posisi_link, rot = self.pose_badan()
        poros = posisi_link + rot @ self.poros_badan
        sumbu_roda, atas_badan, depan = self.sumbu_badan()
        maju_badan = np.cross(sumbu_roda, atas_badan)       # tegak lurus permukaan depan badan
        maju_badan = maju_badan / np.linalg.norm(maju_badan)
        if np.dot(maju_badan, depan) < 0.0:
            maju_badan = -maju_badan
        pusat = poros + self.tinggi_dorong * atas_badan + sisi * config.JARAK_IMU_KE_SISI_LUAR * maju_badan \
            + config.DORONG_GESER_SAMPING * sumbu_roda
        return pusat, sumbu_roda, atas_badan, maju_badan

    def beri_gaya_dorong(self):
        """Gaya dorong dibagi rata ke config.DORONG_TITIK x config.DORONG_TITIK titik di bidang 40 x 40 mm."""
        sisi = -self.arah_dorong               # dorong ke depan = kena sisi belakang badan
        pusat, sumbu_roda, atas_badan, maju_badan = self.bidang_dorong(sisi)
        jumlah = config.DORONG_TITIK * config.DORONG_TITIK
        gaya_titik = (self.arah_dorong * self.gaya_dorong / jumlah) * maju_badan
        for i in range(config.DORONG_TITIK):
            for j in range(config.DORONG_TITIK):
                if config.DORONG_TITIK > 1:
                    geser_a = (i / (config.DORONG_TITIK - 1) - 0.5) * config.DORONG_SISI
                    geser_b = (j / (config.DORONG_TITIK - 1) - 0.5) * config.DORONG_SISI
                else:
                    geser_a = 0.0
                    geser_b = 0.0
                titik = pusat + geser_a * sumbu_roda + geser_b * atas_badan
                p.applyExternalForce(self.robot, -1, gaya_titik.tolist(), titik.tolist(), p.WORLD_FRAME)

    def step(self, u, u_belok=0.0):
        """Jalankan simulasi selama satu siklus kontrol.

        u       : torsi (N m) untuk tiap roda
        u_belok : torsi tambahan untuk belok; positif = belok ke kanan
        """
        # Belok kanan: roda kiri lebih cepat, roda kanan lebih lambat.
        u_kiri = u + u_belok
        u_kanan = u - u_belok
        # Batas arus motor berlaku untuk tiap roda (u = MOTOR_KT x arus perintah).
        u_kiri = max(config.u_min, min(config.u_max, u_kiri))
        u_kanan = max(config.u_min, min(config.u_max, u_kanan))
        # (a) Torsi yang benar-benar keluar dari roda = arus perintah x MOTOR_KT_EFEKTIF.
        torsi_kiri = self.skala_torsi * u_kiri
        torsi_kanan = self.skala_torsi * u_kanan

        for _ in range(config.SUBSTEP):
            # (b) Torsi baru bekerja jeda_substep langkah fisika (SIM_JEDA_TORSI detik) setelah diperintah.
            self.antrean_torsi.append((torsi_kiri, torsi_kanan))
            kiri_bekerja, kanan_bekerja = self.antrean_torsi.pop(0)
            p.setJointMotorControl2(self.robot, self.joint_kiri, p.TORQUE_CONTROL,
                                    force=self.tanda_kiri * kiri_bekerja)
            p.setJointMotorControl2(self.robot, self.joint_kanan, p.TORQUE_CONTROL,
                                    force=self.tanda_kanan * kanan_bekerja)
            if self.sisa_dorong > 0.0:
                self.beri_gaya_dorong()
                self.sisa_dorong = self.sisa_dorong - self.dt_fisika
            p.stepSimulation()

        # (c) Sampel IMU baru untuk siklus berikutnya.
        self.perbarui_imu()

        # Catat data plot: psi, theta, dan arus tiap motor (i = u / K_t).
        self.waktu = self.waktu + config.DT
        if self.plot is not None:
            self.plot_kiriman.append((self.waktu, float(self.x_terakhir[2]), float(self.x_terakhir[3]),
                                      float(self.x_terakhir[0]), float(self.x_terakhir[1]),
                                      u / config.MOTOR_KT))

    # ----------------------------------------------------------------- params
    def baca_params(self):
        """Nilai parameter sekarang (dari jendela Kontrol; dari config.py bila jendelanya tidak ada)."""
        if self.kontrol is not None:
            self.nilai_params = dict(self.kontrol.nilai)
        self.gaya_dorong = self.nilai_params["Gaya_dorong"]
        return self.nilai_params

    def cetak_params(self):
        if self.kontrol is not None:
            self.kontrol.cetak()

    # --------------------------------------------------------------- keyboard
    def baca_keyboard(self):
        """Baca tombol dari jendela Kontrol (utama) dan dari keyboard di jendela PyBullet (cadangan).

        Mengembalikan daftar tombol yang baru ditekan: 'b', 'n', 'o', 'p', 'q', 'r', 'e', 't'.
        Tombol gerak (ditahan) disimpan di:
            self.arah  : +1 maju (W), -1 mundur (S), 0 lepas
            self.belok : -1 kiri (A), +1 kanan (D), 0 lepas
        """
        hasil = []
        if not self.gui:
            return hasil
        tombol = p.getKeyboardEvents()

        def ditahan(huruf):
            for kode in (ord(huruf), ord(huruf.upper())):
                if kode in tombol and (tombol[kode] & p.KEY_IS_DOWN):
                    return True
            return False

        def baru_ditekan(huruf):
            for kode in (ord(huruf), ord(huruf.upper())):
                if kode in tombol and (tombol[kode] & p.KEY_WAS_TRIGGERED):
                    return True
            return False

        # Jendela Kontrol
        tahan_kontrol = {}
        if self.kontrol is not None:
            self.kontrol.perbarui()
            hasil = hasil + self.kontrol.ambil_tombol()
            tahan_kontrol = self.kontrol.tahan

        # Ditahan = ditahan di jendela Kontrol ATAU di keyboard PyBullet
        for huruf in self.ditahan:
            self.ditahan[huruf] = tahan_kontrol.get(huruf, False) or ditahan(huruf)

        # Gerak seperti D-Pad: W / S maju / mundur, A / D belok kiri / kanan.
        self.arah = int(self.ditahan["w"]) - int(self.ditahan["s"])
        self.belok = int(self.ditahan["d"]) - int(self.ditahan["a"])

        for huruf in ("b", "n", "o", "p", "q", "r", "e", "t"):
            if baru_ditekan(huruf) and huruf not in hasil:
                hasil.append(huruf)
        return hasil

    # ----------------------------------------------------- bidang hijau dorong
    def buat_bidang_dorong(self):
        """Dua kotak hijau 40 x 40 mm di sisi depan dan belakang puncak badan (titik dorong O / P).
        Hanya gambar, tanpa collision. Menyala terang saat dorongan bekerja."""
        setengah = [config.DORONG_SISI / 2.0, 0.0015, config.DORONG_SISI / 2.0]
        for sisi in (1, -1):
            bentuk = p.createVisualShape(p.GEOM_BOX, halfExtents=setengah, rgbaColor=config.DORONG_WARNA)
            self.bidang_dorong_visual[sisi] = p.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=-1,
                                                                 baseVisualShapeIndex=bentuk,
                                                                 basePosition=[0.0, 0.0, -5.0])
            self.dorong_menyala[sisi] = False
        self.gambar_bidang_dorong()

    def gambar_bidang_dorong(self):
        if len(self.bidang_dorong_visual) == 0:
            return
        for sisi in (1, -1):
            pusat, _, atas_badan, maju_badan = self.bidang_dorong(sisi)
            pusat = pusat + sisi * 0.002 * maju_badan        # sedikit di luar permukaan supaya terlihat
            sumbu_x = np.cross(maju_badan, atas_badan)
            matriks = np.column_stack([sumbu_x, maju_badan, atas_badan])
            bidang = self.bidang_dorong_visual[sisi]
            p.resetBasePositionAndOrientation(bidang, pusat.tolist(), quaternion_dari_matriks(matriks))

            menyala = self.sisa_dorong > 0.0 and sisi == -self.arah_dorong
            if menyala != self.dorong_menyala[sisi]:
                self.dorong_menyala[sisi] = menyala
                p.changeVisualShape(bidang, -1, rgbaColor=config.DORONG_WARNA_AKTIF if menyala else config.DORONG_WARNA)

    # ------------------------------------------------- bidang merah safety set
    def buat_bidang_safety_set(self):
        """Dua bidang merah transparan berbentuk | | di depan dan di belakang robot (lihat config.JARAK_IMU_KE_SISI_LUAR).
        Hanya gambar (tanpa collision), jadi tidak memengaruhi fisika."""
        setengah = [config.BIDANG_LEBAR / 2.0, 0.001, config.BIDANG_TINGGI / 2.0]
        hasil = []
        for _ in range(2):
            bentuk = p.createVisualShape(p.GEOM_BOX, halfExtents=setengah, rgbaColor=config.BIDANG_WARNA)
            hasil.append(p.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=-1, baseVisualShapeIndex=bentuk,
                                           basePosition=[0.0, 0.0, -5.0]))
        self.bidang_depan = hasil[0]
        self.bidang_belakang = hasil[1]
        self.gambar_bidang_safety_set()

    def gambar_bidang_safety_set(self):
        if self.bidang_depan is None:
            return
        posisi_link, rot = self.pose_badan()
        poros = posisi_link + rot @ self.poros_badan
        _, _, depan = self.sumbu_badan()                    # arah depan robot, mendatar
        atas = np.array([0.0, 0.0, 1.0])
        sumbu_datar = np.cross(depan, atas)                 # arah poros roda, mendatar
        matriks = np.column_stack([sumbu_datar, depan, atas])
        orientasi = quaternion_dari_matriks(matriks)
        psi = self.x_terakhir[2]          # psi hasil Kalman, sama dengan yang dibaca kontrol

        # Jarak mendatar bidang dari poros roda
        jarak = config.L_IMU * math.sin(config.psi_max) + config.JARAK_IMU_KE_SISI_LUAR * math.cos(config.psi_max)

        for nama, bidang, tanda in (("depan", self.bidang_depan, 1.0), ("belakang", self.bidang_belakang, -1.0)):
            pusat = poros + tanda * jarak * depan
            pusat[2] = self.tinggi_lantai(pusat) + 0.5 * config.BIDANG_TINGGI      # bidang berdiri di lantai
            p.resetBasePositionAndOrientation(bidang, pusat.tolist(), orientasi)

            langgar = psi > config.psi_max if nama == "depan" else psi < config.psi_min
            if langgar != self.bidang_langgar[nama]:
                self.bidang_langgar[nama] = langgar
                p.changeVisualShape(bidang, -1, rgbaColor=config.BIDANG_WARNA_LANGGAR if langgar else config.BIDANG_WARNA)

    # ----------------------------------------------- kotak biru safety set jarak s
    def buat_kubus_jarak(self):
        """Kotak biru transparan = daerah s_min <= s <= s_max. Hanya gambar, tanpa collision.
        Tidak dibuat bila config.py belum punya s_max / s_min."""
        if not hasattr(config, "s_max") or not hasattr(config, "s_min"):
            print("Kotak jarak tidak digambar: s_max / s_min belum ada di config.py.")
            return
        panjang = config.s_max - config.s_min
        setengah = [config.KUBUS_LEBAR / 2.0, panjang / 2.0, config.KUBUS_TINGGI / 2.0]
        bentuk = p.createVisualShape(p.GEOM_BOX, halfExtents=setengah, rgbaColor=config.KUBUS_WARNA)
        self.kubus_jarak = p.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=-1, baseVisualShapeIndex=bentuk,
                                             basePosition=[0.0, 0.0, -5.0])
        self.atur_kubus_jarak()

    def atur_kubus_jarak(self):
        """Letakkan kotak di posisi awal robot (s = 0 di titik (0, 0, 0)), searah lintasan dan mengikuti lantai."""
        if self.kubus_jarak is None:
            return
        arah_lintasan = np.array([0.0, math.cos(self.zeta), math.sin(self.zeta)])   # +Y dunia, sepanjang lantai
        tengah_s = 0.5 * (config.s_max + config.s_min)
        pusat = tengah_s * arah_lintasan + 0.5 * config.KUBUS_TINGGI * self.normal
        p.resetBasePositionAndOrientation(self.kubus_jarak, pusat.tolist(),
                                          p.getQuaternionFromEuler([self.zeta, 0.0, 0.0]))

    def warnai_kubus_jarak(self):
        """Kotak lebih gelap bila s = R*theta di luar batas."""
        if self.kubus_jarak is None:
            return
        jarak = model.hitung_s(self.x_terakhir)
        langgar = jarak > config.s_max or jarak < config.s_min
        if langgar != self.kubus_langgar:
            self.kubus_langgar = langgar
            p.changeVisualShape(self.kubus_jarak, -1, rgbaColor=config.KUBUS_WARNA_LANGGAR if langgar else config.KUBUS_WARNA)

    # ------------------------------------------------- teks di jendela PyBullet
    # PyBullet hanya bisa menulis teks di ruang 3D. Supaya teks tetap di tempat yang sama di layar,
    # semua teks ditempelkan ke satu benda kecil tak terlihat ("jangkar") yang selalu diletakkan
    # tepat di depan kamera. Memindahkan jangkar itu murah; menulis ulang teks yang mahal.
    def buat_jangkar_teks(self):
        bentuk = p.createVisualShape(p.GEOM_SPHERE, radius=0.0005, rgbaColor=[0.0, 0.0, 0.0, 0.0])
        self.jangkar = p.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=-1, baseVisualShapeIndex=bentuk,
                                         basePosition=[0.0, 0.0, 1.0])

    def atur_jangkar_teks(self, kamera, tengah):
        """Letakkan jangkar di depan kamera, menghadap kamera. tengah = titik yang dilihat kamera."""
        proyeksi = kamera[3]
        maju = np.array(kamera[5])
        ke_kanan = np.array(kamera[6])
        ke_atas = np.array(kamera[7])
        maju = maju / np.linalg.norm(maju)
        ke_kanan = ke_kanan / np.linalg.norm(ke_kanan)
        ke_atas = ke_atas / np.linalg.norm(ke_atas)

        mata = np.array(tengah) - kamera[10] * maju
        pusat = mata + config.JARAK_TEKS * maju
        matriks = np.column_stack([ke_kanan, ke_atas, -maju])
        p.resetBasePositionAndOrientation(self.jangkar, pusat.tolist(), quaternion_dari_matriks(matriks))

        # Setengah lebar dan tinggi layar pada jarak config.JARAK_TEKS dari kamera.
        self.setengah_lebar = config.JARAK_TEKS / proyeksi[0]
        self.setengah_tinggi = config.JARAK_TEKS / proyeksi[5]

    def tulis(self, kunci, teks, sx, sy, warna, ukuran=1.2):
        """Tulis / perbarui satu teks di posisi layar (sx, sy): -1 kiri/bawah sampai +1 kanan/atas.
        Mengembalikan True bila teks benar-benar dikirim ke PyBullet (isi, posisi, atau warnanya berubah)."""
        posisi = (round(sx * self.setengah_lebar, 5), round(sy * self.setengah_tinggi, 5), 0.0)
        isi = (teks, posisi, tuple(warna), ukuran)
        if self.isi_teks.get(kunci) == isi:
            return False
        self.isi_teks[kunci] = isi
        id_lama = self.id_teks.get(kunci, -1)
        if id_lama >= 0:
            self.id_teks[kunci] = p.addUserDebugText(teks, list(posisi), textColorRGB=warna, textSize=ukuran,
                                                     parentObjectUniqueId=self.jangkar,
                                                     replaceItemUniqueId=id_lama)
        else:
            self.id_teks[kunci] = p.addUserDebugText(teks, list(posisi), textColorRGB=warna, textSize=ukuran,
                                                     parentObjectUniqueId=self.jangkar)
        return True

    def gambar_teks(self, baris):
        """Blok parameter di kiri atas jendela PyBullet. (Visual tombol ada di jendela Kontrol.)"""
        # Blok parameter. Baris yang isinya berubah ditulis bergiliran, config.BARIS_PER_TAMPIL baris tiap panggilan.
        isi_baris = list(baris[:config.JUMLAH_BARIS_MAKS])
        while len(isi_baris) < config.JUMLAH_BARIS_MAKS:
            isi_baris.append(" ")
        ditulis = 0
        for _ in range(config.JUMLAH_BARIS_MAKS):
            i = self.giliran_baris
            self.giliran_baris = (self.giliran_baris + 1) % config.JUMLAH_BARIS_MAKS
            if isi_baris[i] == " " and ("baris%d" % i) not in self.id_teks:
                continue                # baris kosong yang belum pernah dipakai
            if isi_baris[i].startswith(("TUNING", "komet", "MODE PENGUJIAN")):
                warna = config.WARNA_PERINGATAN
            else:
                warna = config.WARNA_BIASA
            if self.tulis("baris%d" % i, isi_baris[i], -0.96, 0.90 - config.JARAK_BARIS * i, warna):
                ditulis = ditulis + 1
                if ditulis >= config.BARIS_PER_TAMPIL:
                    break

    # ------------------------------------------------------------------- plot
    def buat_plot(self):
        """Nyalakan proses jendela plot realtime."""
        konteks = multiprocessing.get_context("spawn")
        # Pakai platform Qt yang sudah terbukti jalan untuk jendela Kontrol, bila ada.
        if self.kontrol is not None and self.kontrol.platform is not None:
            daftar_platform = [self.kontrol.platform]
        else:
            daftar_platform = pilihan_platform_qt()
        catatan = []
        for platform in daftar_platform:
            self.plot_antrean = konteks.Queue()
            jawaban = konteks.Queue()
            self.plot_berhenti = konteks.Event()
            self.plot = konteks.Process(target=proses_plot,
                                        args=(self.plot_antrean, jawaban, self.plot_berhenti, self.judul,
                                              config.PLOT_BACKEND, platform),
                                        daemon=True)
            self.plot.start()
            jenis, isi = tunggu_jawaban(self.plot, jawaban, 30.0)
            if jenis == "siap":
                return
            catatan.append("%s: %s" % (platform, isi))
            self.plot_berhenti.set()
            self.plot.join(timeout=1.0)
            if jenis == "gagal":
                break
        print("Plot realtime tidak aktif:", " | ".join(catatan))
        self.plot = None

    def gambar_plot(self):
        """Kirim data baru ke proses plot."""
        if self.plot is None or len(self.plot_kiriman) == 0:
            return
        if not self.plot.is_alive():
            self.plot = None            # jendela plot ditutup: simulasi tetap jalan tanpa plot
            return
        self.plot_antrean.put(self.plot_kiriman)
        self.plot_kiriman = []

    # ---------------------------------------------------------------- tampilan
    def tampilkan(self, baris, mode_gerak, komet=False, pengujian=""):
        """Perbarui semua tampilan. Dipanggil dari atera_main.py beberapa kali per detik.

        baris          : daftar teks parameter (ditulis di kiri atas jendela PyBullet)
        mode_gerak     : MODE_BALANCING atau MODE_NAVIGATE
        komet          : True = KOMET, motor dimatikan (ditampilkan di jendela Kontrol)
        pengujian      : teks status mode pengujian ("" = tidak dalam mode pengujian)
        """
        if not self.gui:
            return
        self.hitung_tampil = self.hitung_tampil + 1

        # Kamera mengikuti robot, tetapi baru digeser bila robot sudah jauh dari tengah layar.
        # Jarak dan sudut pandang tetap bisa diatur dengan mouse.
        posisi, _ = p.getBasePositionAndOrientation(self.robot)
        kamera = p.getDebugVisualizerCamera()
        tengah = kamera[11]
        if np.linalg.norm(np.array(posisi) - np.array(tengah)) > config.KAMERA_BATAS_GESER:
            tengah = posisi
            p.resetDebugVisualizerCamera(cameraDistance=kamera[10], cameraYaw=kamera[8], cameraPitch=kamera[9],
                                         cameraTargetPosition=tengah)

        self.gambar_bidang_safety_set()
        self.warnai_kubus_jarak()
        self.gambar_bidang_dorong()
        self.atur_jangkar_teks(kamera, tengah)
        self.gambar_teks(baris)

        # Status untuk jendela Kontrol: tombol mana yang menyala, mode, dan dorongan.
        if self.kontrol is not None:
            if self.sisa_dorong > 0.0:
                ke = "KE DEPAN" if self.arah_dorong > 0 else "KE BELAKANG"
                teks_dorong = "dorong: %s %.1f N" % (ke, self.gaya_dorong)
                arah_dorong = self.arah_dorong
            else:
                teks_dorong = "dorong: -"
                arah_dorong = 0
            self.kontrol.kirim_status({"judul": self.judul, "mode_balancing": mode_gerak == MODE_BALANCING,
                                       "tahan": dict(self.ditahan), "dorong": teks_dorong,
                                       "arah_dorong": arah_dorong, "komet": bool(komet),
                                       "mode_pengujian": pengujian != "", "pengujian": pengujian})
        if self.hitung_tampil % config.PLOT_SETIAP == 0:
            self.gambar_plot()

    def masih_terbuka(self):
        return bool(p.isConnected())

    def tutup(self):
        if self.kontrol is not None:
            self.kontrol.tutup()
        if self.plot is not None:
            self.plot_berhenti.set()
            self.plot.join(timeout=2.0)
        if p.isConnected():
            p.disconnect()


if __name__ == "__main__":
    import atera_main
    atera_main.main()