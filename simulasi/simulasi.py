"""simulasi.py - Dunia simulasi PyBullet untuk robot ATERA.

Jalankan:  python3 atera_main.py --plot     (mode kontrol PD / PD + CBF-QP dipilih di jendela Kontrol)
           (python3 simulasi.py --plot hasilnya sama)

File ini hanya mengurus simulasinya:
    - dunia (plane.urdf bawaan PyBullet), robot, sensor, motor, dorongan
    - realisme dari uji hardware (nilai di config.py, bagian 10; semua pengaturan simulasi di bagian 11):
        a. torsi roda = arus perintah x MOTOR_KT_EFEKTIF (bukan MOTOR_KT)
        b. torsi baru bekerja SIM_JEDA_TORSI detik setelah perintah dihitung
        c. psi dibaca lewat model MPU6050 (akselerometer + gyro + noise) dan Kalman yang sama dengan robot
        d. batas arus bergantung kecepatan roda (back-EMF, SIM_BATAS_KECEPATAN): arus yang bisa masuk
           turun linear dari batas arus sampai 0 di kecepatan no-load DDSM115 (200 rpm = 20.9 rad/s)
    - jendela "ATERA - Kontrol" (Qt): tombol mode kontrol PD / PD + CBF-QP, tombol W A S D B F O P,
      dan kolom angka parameter (input utama). B = mode BALANCING, F = mode BEBAS, U = PENGUJIAN OTOMATIS
    - lintasan UJI 2 pengujian otomatis (buat_lintasan / hapus_lintasan, dipakai uji_otomatis.py)
    - FOTO PENGUJIAN (tombol 6 / 7, alurnya di atera_main.FotoPengujian): bandul, robot bayangan, lintasan berwarna,
      kamera foto, getCameraImage + PNG BERSIH (tanpa teks) dan ANOTASI (garis, busur, teks). Bagian "FOTO" di bawah.
    - catatan pengujian otomatis: blok tulisan di jendela PyBullet dan panel catatan di jendela plot
    - plot realtime: kurva sekarang + kurva PEMBANDING (pengujian terakhir tiap mode: PD ungu, PD+CBF biru)
    - keyboard di jendela PyBullet (input cadangan, tombolnya sama)
    - tampilan di jendela PyBullet: teks parameter, kamera DIAM tampak sisi kanan robot (tidak ikut robot, supaya
      arah gerak robot terlihat), dan bidang merah barrier psi (ikut robot, hanya di mode PD + CBF-QP, jaraknya
      ikut psi_max_DEG, warnanya makin pekat saat psi mendekati psi_max)
    - jendela plot realtime (hanya bila dijalankan dengan --plot)
Kontrolnya ada di atera_main.py.

Arah di simulasi:
    DEPAN robot = sumbu +Y dunia.  Sumbu roda = sumbu X.  Atas = sumbu Z.
    Roda kiri (Lwheel_link) ada di sisi -X dunia, roda kanan (Rwheel_link) di sisi +X.
    psi   > 0 : badan miring ke depan (diukur dari garis vertikal)
    theta > 0 : roda menggelinding ke depan (sudut roda absolut, definisi di model.hitung_theta)
    u     > 0 : torsi mendorong roda ke depan (torsi SATU roda; roda kiri u + u_belok, roda kanan u - u_belok)
    yaw   > 0 : robot berputar ke kiri dilihat dari atas (baca_yaw, dari encoder kedua roda)
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
from pd_control import MODE_BALANCING, MODE_BEBAS, MODE_DIAM, hitung_yaw


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
                 for _nama in NAMA_GAIN + ["Gaya_dorong", "zeta_DEG", "psi_max_DEG"]]
DAFTAR_PARAMS.append(("chi_DEG", 1))    # sudut lepas bandul FOTO tombol 6 [deg] (bukan nilai config.py)

FOLDER_INI = os.path.dirname(os.path.abspath(__file__))
PILIHAN_URDF = [os.path.join(FOLDER_INI, path) for path in config.URDF_PILIHAN]

# Nama mode kontrol PD + CBF-QP (= ise.MODE_PD_CBF). ise tidak di-import di sini karena ise memasang backend
# matplotlib "Agg" saat di-import.
MODE_PD_CBF = "PD+CBF"
# Tempat bidang merah disembunyikan (jauh di bawah lantai): mode PD, atau barrier sisi itu tidak dicentang.
TEMPAT_SEMBUNYI = [0.0, 0.0, -5.0]


def warna_hex(teks, alpha=1.0):
    """'#FF9191' -> [1.0, 0.569, 0.569, alpha] (rgba PyBullet)."""
    teks = teks.lstrip("#")
    return [int(teks[i:i + 2], 16) / 255.0 for i in (0, 2, 4)] + [float(alpha)]


# =============================================================================
# FOTO PENGUJIAN (tombol 6 = dipukul bandul di bidang datar, 7 = naik bidang miring; alur di atera_main.py)
# Semua nilai FOTO_* hanya untuk GAMBAR (benda visual tanpa collision, kamera, warna): fisika dan kontrol tidak
# berubah. ASUMSI: ditaruh di sini (bukan config.py) karena tugas ini hanya mengubah atera_main.py dan simulasi.py.
# Hasil: PLOT_EVALUASI/FOTO/<PD | PDCBFQP10 | ...>/YYYYMMDD_HHmmSS_<kontrol>_<uji>_Simulasi_FOTO KE-XX_BERSIH.png
#                                                                                         ..._ANOTASI.png
#        <uji> = sudut bandul chi (mis. 60) untuk tombol 6, 7-5 (UJI2_ZETA_DEG) untuk tombol 7.
# =============================================================================
FOTO_FOLDER = os.path.join(FOLDER_INI, "PLOT_EVALUASI", "FOTO")
FOTO_UKURAN_PUKUL = (1920, 1080)   # [px] (lebar, tinggi) PNG tombol 6
FOTO_UKURAN_MIRING = (2560, 900)    # [px] (lebar, tinggi) PNG tombol 7 (lebar, karena lintasan panjang)
# Bagian gambar tempat benda-benda harus muat (kiri, kanan, bawah, atas; 0 .. 1 dari lebar / tinggi gambar). Sisanya
# untuk blok info (atas) dan grafik kecil (kanan atas, tombol 6) di PNG ANOTASI, supaya tidak menutupi benda.
FOTO_KOTAK_PUKUL = (0.03, 0.69, 0.04, 0.78)
FOTO_KOTAK_MIRING = (0.02, 0.98, 0.06, 0.74)
FOTO_FOV_DEG = 20.0                 # [deg] sudut pandang vertikal kamera foto. Kecil = hampir tanpa perspektif
FOTO_PITCH_DEG = -4.0               # [deg] kamera hampir tepat dari samping (yaw = config.KAMERA_YAW, sisi kanan)
FOTO_CHI_AWAL_DEG = 60.0            # [deg] chi awal di jendela Kontrol (kolom chi_DEG)
FOTO_JENDELA_PUNCAK_S = 2.5         # [s] tombol 6: |psi| maksimum dicari selama sekian detik setelah pukulan
FOTO_BOLA_JARI_M = 0.023            # [m] ASUMSI: bandul bola baja 0.404 kg -> r = (3 m / (4 pi 7850))^(1/3) = 0.023 m
FOTO_TALI_JARI_M = 0.0012           # [m] jari-jari tali (gambar)
FOTO_WARNA_BOLA = [0.20, 0.27, 0.42, 1.0]
FOTO_WARNA_TALI = [0.12, 0.12, 0.12, 1.0]
FOTO_WARNA_GANTUNGAN = [0.25, 0.25, 0.25, 1.0]
FOTO_WARNA_BAYANGAN = [0.72, 0.72, 0.78, 0.30]     # robot / bandul bayangan (transparan)
FOTO_TITIK_BUSUR = 16               # jumlah titik kecil lintasan ayunan bandul (chi -> 0)
FOTO_WARNA_TANJAKAN = warna_hex("#FF9191")           # papan bidang miring
FOTO_WARNA_DATAR = warna_hex("#FFE55C")              # bidang datar lintasan (bawah dan atas)
FOTO_JALUR_BAWAH_BELAKANG_M = 0.5   # [m] ASUMSI: jalur kuning datar bawah mulai sekian m di belakang posisi awal robot
FOTO_BAYANGAN_TUJUAN = True         # tombol 7: robot bayangan tegak di bidang atas (penanda tujuan, BUKAN hasil uji)
FOTO_SISI_ANOTASI_M = 0.20          # [m] garis anotasi robot digeser ke sisi kamera (di luar roda) supaya terlihat


def tingkat_bidang(psi, psi_max, sisi):
    """Tingkat kepekatan warna bidang merah barrier psi (bilangan bulat):
        0 .. BIDANG_TINGKAT : naik linear dengan psi / psi_max ke arah bidang itu (0 = psi 0 atau miring menjauh,
                              BIDANG_TINGKAT = tepat di psi_max)
        BIDANG_TINGKAT + 1  : psi melewati barrier sisi itu
    sisi = +1 bidang depan (h1, psi <= psi_max), -1 bidang belakang (h2, psi >= -psi_max)."""
    if psi_max <= 0.0 or not math.isfinite(psi):
        return 0
    rasio = sisi * psi / psi_max
    if rasio > 1.0:
        return config.BIDANG_TINGKAT + 1
    return int(round(max(0.0, rasio) * config.BIDANG_TINGKAT))


def warna_bidang(tingkat):
    """rgba bidang merah untuk satu tingkat kepekatan (lihat tingkat_bidang). Opasitas = kanal a pada rgba PyBullet
    (0 = tembus pandang, 1 = pekat), bukan Alpha CBF."""
    if tingkat > config.BIDANG_TINGKAT:
        opasitas = config.BIDANG_OPASITAS_LANGGAR
    else:
        opasitas = config.BIDANG_OPASITAS_MIN + (config.BIDANG_OPASITAS_MAKS - config.BIDANG_OPASITAS_MIN) \
            * tingkat / config.BIDANG_TINGKAT
    return list(config.BIDANG_RGB) + [opasitas]


# ----------------------------------------------------------------------------- FOTO: proyeksi dan gambar anotasi
def proyeksi_layar(titik, view, proj, lebar, tinggi):
    """Titik dunia (N x 3) -> piksel (N x 2) pada gambar getCameraImage dengan matriks view / proj yang sama
    (matriks PyBullet = 16 angka urutan kolom). Piksel (0, 0) = kiri atas."""
    titik = np.atleast_2d(np.asarray(titik, dtype=float))
    matriks = np.array(proj, dtype=float).reshape(4, 4).T @ np.array(view, dtype=float).reshape(4, 4).T
    klip = np.c_[titik, np.ones(len(titik))] @ matriks.T
    ndc = klip[:, :3] / klip[:, 3:4]
    return np.c_[(ndc[:, 0] + 1.0) * 0.5 * lebar, (1.0 - ndc[:, 1]) * 0.5 * tinggi]


def foto_simpan_bersih(rgb, path):
    """PNG BERSIH: hasil getCameraImage apa adanya (tanpa teks / garis anotasi)."""
    from matplotlib import image as mpimg
    mpimg.imsave(path, rgb)


def foto_simpan_anotasi(rgb, view, proj, anotasi, path):
    """PNG ANOTASI: hasil getCameraImage + garis, busur, panah, teks (titik dunia diproyeksikan dengan kamera yang
    sama, jadi tepat di tempatnya) + blok info kiri atas + grafik kecil (opsional). Tanpa pyplot (aman dengan backend
    apa pun). anotasi (dict, semua kunci opsional):
        garis : list {"titik": N x 3, "warna": rgb, "tebal": px, "putus": bool}
        panah : list {"dari": 3, "ke": 3, "warna": rgb, "tebal": px}
        teks  : list {"posisi": 3, "teks": str, "warna": rgb, "ukuran": pt, "geser": (dx, dy) px, "ha": "left"}
        info  : list (teks, rgb, tebal)  -> blok kiri atas
        grafik: {"t", "y", "t_tanda", "y_tanda", "batas", "judul", "label_y", "t_pukul"}
    """
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure

    tinggi, lebar = rgb.shape[:2]
    skala = lebar / 1920.0                      # ukuran huruf ikut lebar gambar
    fig = Figure(figsize=(lebar / 100.0, tinggi / 100.0), dpi=100)
    FigureCanvasAgg(fig)
    ax = fig.add_axes([0.0, 0.0, 1.0, 1.0])
    ax.imshow(rgb, interpolation="none")
    ax.set_xlim(0, lebar)
    ax.set_ylim(tinggi, 0)
    ax.axis("off")

    def piksel(titik):
        return proyeksi_layar(titik, view, proj, lebar, tinggi)

    for g in anotasi.get("garis", []):
        xy = piksel(g["titik"])
        ax.plot(xy[:, 0], xy[:, 1], color=g.get("warna", (0, 0, 0)), lw=g.get("tebal", 2.0),
                ls="--" if g.get("putus", False) else "-", solid_capstyle="round", zorder=3)
    for a in anotasi.get("panah", []):
        xy = piksel([a["dari"], a["ke"]])
        ax.annotate("", xy=tuple(xy[1]), xytext=tuple(xy[0]), zorder=4,
                    arrowprops=dict(arrowstyle="-|>", color=a.get("warna", (0, 0, 0)), lw=a.get("tebal", 3.0),
                                    mutation_scale=28, shrinkA=0, shrinkB=0))
    for t in anotasi.get("teks", []):
        xy = piksel([t["posisi"]])[0]
        dx, dy = t.get("geser", (0, 0))
        ax.text(xy[0] + dx, xy[1] + dy, t["teks"], color=t.get("warna", (0, 0, 0)), fontsize=t.get("ukuran", 15) * skala,
                fontweight="bold", ha=t.get("ha", "left"), va=t.get("va", "center"), zorder=5,
                bbox=dict(boxstyle="round,pad=0.25", fc="white", ec="none", alpha=0.75))
    for i, (teks, warna, tebal) in enumerate(anotasi.get("info", [])):
        fig.text(0.010, 1.0 - (14.0 + 38.0 * i) * skala / tinggi, teks, color=warna, fontsize=15 * skala,
                 family="monospace",
                 fontweight="bold" if tebal else "normal", va="top", ha="left",
                 bbox=dict(boxstyle="square,pad=0.2", fc="white", ec="none", alpha=0.70))

    grafik = anotasi.get("grafik")
    if grafik is not None and len(grafik["t"]) > 1:
        kecil = fig.add_axes([0.715, 0.56, 0.27, 0.30])
        kecil.patch.set_alpha(0.88)
        kecil.plot(grafik["t"], grafik["y"], color=(0.0, 0.31, 0.69), lw=1.8)
        if grafik.get("batas") is not None:
            kecil.axhline(grafik["batas"], color=(0.85, 0.0, 0.0), ls=":", lw=1.5)
        if grafik.get("t_pukul") is not None:
            kecil.axvline(grafik["t_pukul"], color=(0.04, 0.54, 0.16), ls="--", lw=1.2)
        kecil.plot([grafik["t_tanda"]], [grafik["y_tanda"]], "o", ms=12, mfc="none", mec=(0.8, 0.0, 0.0), mew=2.5)
        kecil.set_title(grafik.get("judul", ""), fontsize=13)
        kecil.set_xlabel("t (s)", fontsize=12)
        kecil.set_ylabel(grafik.get("label_y", ""), fontsize=12)
        kecil.tick_params(labelsize=11)
        kecil.grid(alpha=0.3)
    fig.savefig(path, dpi=100)


def batas_arus_kecepatan(arus, w):
    """Arus [A] yang benar-benar bisa masuk motor DDSM115 pada kecepatan roda w [rad/s] (relatif badan,
    + = ke depan), untuk arus perintah `arus` (sudah dibatasi MAX_CURRENT_A). Model motor DC: tegangan baterai habis
    dipakai back-EMF saat roda cepat, jadi arus SEARAH putaran turun linear sampai 0 di kecepatan no-load:
        i_maju_maks   = min(MAX_CURRENT_A, MOTOR_STALL_CURRENT_A * (1 - w / w_noload)),  dijepit >= 0
        i_mundur_maks = min(MAX_CURRENT_A, MOTOR_STALL_CURRENT_A * (1 + w / w_noload)),  dijepit >= 0
    Arus yang MELAWAN putaran (mengerem) tidak dikurangi (sampai MAX_CURRENT_A).
    ASUMSI: kurva linear; w_noload = MOTOR_NO_LOAD_SPEED_RPM (cocok dengan plafon ~21 rad/s data robot 20261015)."""
    if not config.SIM_BATAS_KECEPATAN:
        return arus
    w_noload = config.MOTOR_NO_LOAD_SPEED_RPM * 2.0 * math.pi / 60.0
    maju = min(config.MAX_CURRENT_A, max(0.0, config.MOTOR_STALL_CURRENT_A * (1.0 - w / w_noload)))
    mundur = min(config.MAX_CURRENT_A, max(0.0, config.MOTOR_STALL_CURRENT_A * (1.0 + w / w_noload)))
    return max(-mundur, min(maju, arus))


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


CENTANG_PER_BARIS = 4                   # kotak centang constraint CBF-QP per baris di jendela Kontrol


def label_constraint():
    """Label kotak centang constraint CBF-QP: {nomor h: teks}, dari config.CBF_BARRIER_H."""
    label = {}
    for nama, (atas, bawah) in config.CBF_BARRIER_H.items():
        label[atas] = "h%d  %s maks" % (atas, nama)
        label[bawah] = "h%d  %s min" % (bawah, nama)
    return label


def nilai_awal_params():
    """Nilai awal tiap parameter: gain dari config.py, gaya dorong dari config.GAYA_DORONG_AWAL."""
    nilai = {}
    for nama, _ in DAFTAR_PARAMS:
        if nama == "Gaya_dorong":
            nilai[nama] = float(config.GAYA_DORONG_AWAL)
        elif nama == "chi_DEG":
            nilai[nama] = float(FOTO_CHI_AWAL_DEG)
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
        if nama == "chi_DEG":
            teks = "# %s deg (FOTO_CHI_AWAL_DEG di simulasi.py, tombol 6)" % teks
        if abs(nilai[nama] - awal[nama]) > 1e-9:
            teks = "%-28s # berubah, sebelumnya %s" % (teks, tulis_angka(nama, awal[nama]))
        print(teks, flush=True)
    print("---------------------------------------------------", flush=True)


# =============================================================================
# Jendela "ATERA - Kontrol"
#
# Jendela Kontrol (Qt) dan jendela plot (matplotlib) dijalankan di PROSES TERPISAH dari PyBullet, jadi bila
# jendelanya gagal / crash, simulasi tetap jalan. Data dikirim antar proses lewat antrean:
#     jendela -> simulasi : ("nilai", dict parameter), ("tombol", huruf baru ditekan / "mode_pd" / "mode_cbf" /
#                           "banding"), ("tahan", dict tombol ditahan), ("centang", list constraint CBF dicentang)
#     simulasi -> jendela : ("centang", list) sekali di awal: centang awal constraint CBF
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
                if jenis == "centang":
                    jendela.atur_centang(isi)
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
        self.centang = None            # constraint CBF yang dicentang (list bool), None = belum ada kiriman
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
            if jenis == "centang":
                self.centang = list(isi)
        if not self.proses.is_alive():
            # Jendela ditutup: nilai parameter terakhir tetap dipakai, tombolnya dianggap lepas.
            self.aktif = False
            for huruf in self.tahan:
                self.tahan[huruf] = False

    def ambil_tombol(self):
        hasil = self.tombol_baru
        self.tombol_baru = []
        return hasil

    def kirim_centang(self, centang):
        """Centang awal constraint CBF ke jendela Kontrol."""
        self.centang = list(centang)
        if self.aktif:
            self.antrean_keluar.put(("centang", list(centang)))

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
        Gambar 2 : dtheta terhadap waktu (theta tidak digambar: posisi roda terus bertambah saat robot berjalan)
        Gambar 3 : sumbu X = s = R*theta, sumbu Y = 0
        bawah    : arus tiap motor terhadap waktu
        catatan  : catatan pengujian otomatis (kontrol, uji, bandul, gain, status), kiriman ("CATATAN", list teks)
    Yang digambar hanya data config.PLOT_RENTANG detik terakhir. Titik hitam = keadaan sekarang.
    Batas sumbu tetap (config.PLOT_BATAS). Barrier hanya psi, dpsi, dtheta, s: merah = constraint CBF yang dicentang
    (kotak psi ikut psi_max_DEG di Kontrol), abu-abu = tidak dicentang. Hitam = batas KOMET, kuning / oranye =
    penanda |psi| 5 / 10 deg.
    Warna kurva = mode kontrol (PD ungu, PD+CBF biru). Kurva PEMBANDING (pengujian terakhir tiap mode) digambar
    pudar di belakang kurva sekarang; di Gambar 3 pembanding PD di atas garis 0, pembanding PD+CBF di bawahnya.
    judul = mode kontrol awal. Kiriman khusus: ("RESET",), ("MODE", mode), ("MASK", mask constraint CBF),
    ("PEMBANDING", mode, label, psi, dpsi, theta, dtheta, t)."""
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

        fig = plt.figure(figsize=(10.0, 11.0))
        kisi = fig.add_gridspec(4, 2, height_ratios=[1.6, 0.5, 0.9, 0.95])
        gambar1 = fig.add_subplot(kisi[0, 0])
        gambar2 = fig.add_subplot(kisi[0, 1])
        gambar3 = fig.add_subplot(kisi[1, :])
        ax_arus = fig.add_subplot(kisi[2, :])
        # Panel catatan pengujian otomatis (tulisan saja, tanpa sumbu)
        ax_catatan = fig.add_subplot(kisi[3, :])
        ax_catatan.axis("off")
        teks_catatan = ax_catatan.text(0.0, 1.0, "Catatan PENGUJIAN OTOMATIS tampil di sini (tekan U).",
                                       transform=ax_catatan.transAxes, ha="left", va="top", family="monospace",
                                       fontsize=7.5)
        warna = {ise.MODE_PD: config.PLOT_WARNA_PD, ise.MODE_PD_CBF: config.PLOT_WARNA_PD_CBF}
        mode_plot = judul if judul in warna else ise.MODE_PD

        def judul_jendela():
            try:
                fig.canvas.manager.set_window_title("ATERA - plot realtime (%s)" % mode_plot)
            except Exception:
                pass
            fig.suptitle("mode kontrol %s   |   data %.0f s terakhir" % (mode_plot, config.PLOT_RENTANG),
                         fontsize=9, color="gray")

        psi_max_plot = config.psi_max
        mask_plot = None                        # constraint CBF aktif (dikirim simulasi, lihat ("MASK", mask))
        garis_aman = ise.gambar_safety_set(gambar1, gambar2, psi_max_plot, mask_plot)
        batas_s = ise.gambar_batas_jarak(gambar3, mask_plot)

        def judul_gambar1():
            teks = "Gambar 1   (psi_max = %g deg)" % math.degrees(psi_max_plot)
            if mask_plot is not None:
                teks = teks + "   CBF aktif: " + ise.teks_mask(mask_plot)
            gambar1.set_title(teks, fontsize=10)

        def legenda_semua():
            ise.legenda(gambar1)
            ise.legenda(gambar2, perpanjangan=False)
            ise.legenda(gambar3, perpanjangan=False)

        # Gambar 1: X = psi, Y = dpsi
        # Kurva pembanding (pudar, di belakang kurva sekarang). Gambar 3: PD di y = +0.5, PD+CBF di y = -0.5.
        pembanding = {}
        tinggi_s = {ise.MODE_PD: 0.5, ise.MODE_PD_CBF: -0.5}
        for m in (ise.MODE_PD, ise.MODE_PD_CBF):
            pembanding[m] = (gambar1.plot([], [], color=warna[m], linewidth=1.0, alpha=0.45, zorder=1.8)[0],
                             gambar2.plot([], [], color=warna[m], linewidth=1.0, alpha=0.45, zorder=1.8)[0],
                             gambar3.plot([], [], color=warna[m], linewidth=3.0, alpha=0.45, zorder=1.8)[0])

        garis_psi, = gambar1.plot([], [], color=warna[mode_plot], linewidth=1.4)
        titik_psi, = gambar1.plot([], [], "o", color="black", markersize=6, label="sekarang")
        gambar1.set_xlabel("psi [rad]")
        gambar1.set_ylabel("dpsi [rad/s]")
        judul_gambar1()

        # Gambar 2: X = waktu, Y = dtheta
        garis_dtheta, = gambar2.plot([], [], color=warna[mode_plot], linewidth=1.4)
        titik_dtheta, = gambar2.plot([], [], "o", color="black", markersize=6, label="sekarang")
        gambar2.set_xlabel("waktu [s]")
        gambar2.set_ylabel("dtheta [rad/s]")
        gambar2.set_title("Gambar 2   dtheta(t)", fontsize=10)
        gambar2.set_xlim(0.0, config.PLOT_RENTANG)

        # Gambar 3: X = s, Y = 0
        garis_s, = gambar3.plot([], [], color=warna[mode_plot], linewidth=2.0)
        titik_s, = gambar3.plot([], [], "o", color="black", markersize=6, label="sekarang")
        legenda_semua()

        # Arus terhadap waktu (hanya 2 batas, garis tebal menyambung)
        garis_arus, = ax_arus.plot([], [], color="tab:green")
        ax_arus.axhline(config.MAX_CURRENT_A, color="black", linewidth=1.0, linestyle=":")    # batas arus (bukan CBF)
        ax_arus.axhline(config.MIN_CURRENT_A, color="black", linewidth=1.0, linestyle=":")
        ax_arus.set_ylim(1.2 * config.MIN_CURRENT_A, 1.2 * config.MAX_CURRENT_A)
        ax_arus.set_ylabel("arus per motor [A]")
        ax_arus.set_xlabel("waktu [s]")

        for ax in (gambar1, gambar2, ax_arus):
            ax.grid(True, color="0.85")
        judul_jendela()
        fig.tight_layout()
        fig.show()
        jawaban.put(("siap", ""))

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
                    if isi[0] == "CATATAN":
                        # Catatan pengujian otomatis (list baris teks)
                        teks_catatan.set_text("\n".join(isi[1]))
                        ada_baru = True
                        continue
                    if isi[0] == "MODE":
                        # Mode kontrol diganti: warna kurva sekarang ikut mode baru.
                        mode_plot = isi[1] if isi[1] in warna else ise.MODE_PD
                        for garis in (garis_psi, garis_dtheta, garis_s):
                            garis.set_color(warna[mode_plot])
                        judul_jendela()
                        ada_baru = True
                        continue
                    if isi[0] == "MASK":
                        # Centang constraint CBF berubah: barrier aktif merah, tidak aktif abu-abu.
                        mask_plot = isi[1]
                        ise.atur_safety_set(garis_aman, psi_max_plot, mask_plot)
                        ise.atur_batas_jarak(batas_s, mask_plot)
                        judul_gambar1()
                        ada_baru = True
                        continue
                    if isi[0] == "PEMBANDING":
                        _, m, label, p_psi, p_dpsi, p_theta, p_dtheta, p_t = isi
                        g1, g2, g3 = pembanding[m]
                        g1.set_data(p_psi, p_dpsi)
                        g2.set_data(p_t, p_dtheta)
                        g3.set_data([config.R * v for v in p_theta], [tinggi_s[m]] * len(p_theta))
                        for g in (g1, g2, g3):
                            g.set_label("pembanding " + label)
                        legenda_semua()
                        ada_baru = True
                        continue
                    if isi[0] == "RESET":
                        # Robot diletakkan ulang (R, KOMET lalu R, atau zeta diubah): plot mulai dari awal.
                        data_t = []
                        data_psi = []
                        data_dpsi = []
                        data_theta = []
                        data_dtheta = []
                        data_arus = []
                        continue
                    t, psi, dpsi, theta, dtheta, arus, psi_max_baru = isi
                    if psi_max_baru != psi_max_plot:
                        # psi_max diubah di jendela Kontrol: kotak psi digeser
                        psi_max_plot = psi_max_baru
                        ise.atur_safety_set(garis_aman, psi_max_plot, mask_plot)
                        judul_gambar1()
                        fig.canvas.draw_idle()
                    data_t.append(t)
                    data_psi.append(psi)
                    data_dpsi.append(dpsi)
                    data_theta.append(theta)
                    data_dtheta.append(dtheta)
                    data_arus.append(arus)
                ada_baru = True
            if ada_baru and len(data_t) == 0:
                for garis in (garis_psi, garis_dtheta, garis_s, garis_arus, titik_psi, titik_dtheta, titik_s):
                    garis.set_data([], [])
                for ax in (gambar2, ax_arus):
                    ax.set_xlim(0.0, config.PLOT_RENTANG)
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
                garis_dtheta.set_data(data_t, dtheta)
                garis_s.set_data(jarak, np.zeros(len(jarak)))
                garis_arus.set_data(data_t, data_arus)
                if not math.isnan(data_psi[-1]):
                    titik_psi.set_data([data_psi[-1]], [data_dpsi[-1]])
                    titik_dtheta.set_data([data_t[-1]], [data_dtheta[-1]])
                    titik_s.set_data([config.R * data_theta[-1]], [0.0])
                for ax in (gambar2, ax_arus):
                    ax.set_xlim(max(0.0, data_t[-1] - config.PLOT_RENTANG), max(config.PLOT_RENTANG, data_t[-1]))
                fig.canvas.draw_idle()
            fig.canvas.flush_events()
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass


class JendelaKontrol:
    """Jendela Qt "ATERA - Kontrol":
        - paling atas : tombol mode kontrol PD (ungu) / PD + CBF-QP (biru) (keyboard 1 / 2), perekam perbandingan
                        (keyboard 3), dan kotak centang constraint CBF-QP yang dipakai (h1 .. h4)
        - bagian atas : tombol W A S D, B F, O P, Q R E T, U (bisa diklik / ditahan dengan mouse, atau ditekan di
                        keyboard). U = PENGUJIAN OTOMATIS (uji_otomatis.py)
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
        for huruf in ("w", "a", "s", "d", "o", "p", "b", "f", "q", "r", "e", "t", "u"):
            self.tahan_keyboard[huruf] = False
            self.tahan_mouse[huruf] = False
        self.tahan_terkirim = None
        self.status = {"judul": "", "mode_gerak": MODE_DIAM, "tahan": {}, "dorong": "dorong: -", "arah_dorong": 0,
                       "komet": False, "mode_pengujian": False, "pengujian": "", "cbf_ada": True,
                       "pembanding": "", "mode_banding": False, "uji_otomatis": False}

        self.root = None               # jendela Qt; None bila gagal dibuat atau sudah ditutup
        self.sedang_atur_centang = False
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

        # ------------------------------------------------ mode kontrol (PD / PD + CBF-QP)
        baris_mode = QtWidgets.QHBoxLayout()
        label = QtWidgets.QLabel("Mode kontrol:")
        label.setStyleSheet("font-weight: bold")
        baris_mode.addWidget(label)
        self.tombol_mode = {}
        for kunci, teks in (("mode_pd", "1  |  PD"), ("mode_cbf", "2  |  PD + CBF-QP"),
                            ("banding", "3  |  rekam perbandingan"), ("foto_pukul", "6  |  foto dipukul"),
                            ("foto_miring", "7  |  foto bidang miring")):
            tombol = QtWidgets.QPushButton(teks)
            tombol.setFixedSize(150 if kunci in ("mode_pd", "mode_cbf") else 170, 34)
            tombol.setFocusPolicy(QtCore.Qt.NoFocus)
            if kunci == "banding":
                tombol.setToolTip("Nyala / mati perekam perbandingan: R lalu O / P = mulai rekam, disimpan otomatis "
                                  "dan jadi pembanding mode itu")
            elif kunci == "foto_pukul":
                tombol.setToolTip("FOTO: bidang datar, dipukul bandul chi_DEG, freeze + PNG di |psi| maksimum "
                                  "(mode kontrol sekarang)")
            elif kunci == "foto_miring":
                tombol.setToolTip("FOTO: naik bidang miring %g deg, freeze + PNG saat poros roda di tengah tanjakan "
                                  "(mode kontrol sekarang)" % config.UJI2_ZETA_DEG)
            else:
                tombol.setToolTip("Ganti mode kontrol: data sekarang disimpan, lalu robot diletakkan ulang")
            tombol.clicked.connect(lambda dicentang=False, k=kunci: self.kirim(("tombol", k)))
            baris_mode.addWidget(tombol)
            self.tombol_mode[kunci] = tombol
        baris_mode.addStretch(1)
        susunan.addLayout(baris_mode)

        # Kotak centang constraint CBF-QP (h1 .. h4): yang dicentang = dipakai CBF-QP. Centang awal dikirim simulasi
        # (C_MODE[MODE_DIAM] di WORKPLACE cbf_qp.py). CENTANG_PER_BARIS kotak per baris.
        kisi_centang = QtWidgets.QGridLayout()
        kisi_centang.setHorizontalSpacing(14)
        label = QtWidgets.QLabel("Constraint CBF-QP:")
        label.setStyleSheet("font-weight: bold")
        kisi_centang.addWidget(label, 0, 0)
        self.centang = {}
        for urutan, (nomor, teks) in enumerate(sorted(label_constraint().items())):
            kotak = QtWidgets.QCheckBox(teks)
            kotak.setFocusPolicy(QtCore.Qt.NoFocus)
            kotak.toggled.connect(lambda dicentang=False: self.kirim_centang())
            kisi_centang.addWidget(kotak, urutan // CENTANG_PER_BARIS, 1 + urutan % CENTANG_PER_BARIS)
            self.centang[nomor] = kotak
        kisi_centang.setColumnStretch(1 + CENTANG_PER_BARIS, 1)
        susunan.addLayout(kisi_centang)
        self.label_pembanding = QtWidgets.QLabel("")
        self.label_pembanding.setStyleSheet("color: gray")
        susunan.addWidget(self.label_pembanding)

        # ------------------------------------------------ bagian tombol
        self.label_mode = QtWidgets.QLabel("")
        self.label_mode.setStyleSheet("font-weight: bold")
        susunan.addWidget(self.label_mode)

        kisi_tombol = QtWidgets.QGridLayout()
        susunan.addLayout(kisi_tombol)
        letak = {"w": (0, 1), "a": (1, 0), "s": (1, 1), "d": (1, 2),
                 "b": (0, 4), "f": (0, 5), "o": (1, 4), "p": (1, 5), "q": (0, 7), "r": (1, 7), "e": (0, 8), "t": (1, 8),
                 "u": (0, 10)}
        keterangan = {"w": "W\nmaju", "a": "A\nkiri", "s": "S\nmundur", "d": "D\nkanan",
                      "b": "B\nmode BALANCING", "f": "F\nmode BEBAS", "o": "O\ndorong depan", "p": "P\ndorong belakang",
                      "q": "Q\nkeluar + simpan", "r": "R\nreset robot", "e": "E\nsimpan data",
                      "t": "T\npengujian", "u": "U\nuji otomatis"}
        for huruf in ("w", "a", "s", "d", "b", "f", "o", "p", "q", "r", "e", "t", "u"):
            tombol = QtWidgets.QPushButton(keterangan[huruf])
            tombol.setFixedSize(92, 46)
            tombol.setFocusPolicy(QtCore.Qt.NoFocus)        # tombol tidak merebut keyboard
            tombol.pressed.connect(lambda h=huruf: self.atur_tombol(h, True, "mouse"))
            tombol.released.connect(lambda h=huruf: self.atur_tombol(h, False, "mouse"))
            kisi_tombol.addWidget(tombol, letak[huruf][0], letak[huruf][1])
            self.tombol[huruf] = tombol
        kisi_tombol.setColumnMinimumWidth(3, 24)
        kisi_tombol.setColumnMinimumWidth(6, 24)
        kisi_tombol.setColumnMinimumWidth(9, 24)
        self.tombol["u"].setToolTip("PENGUJIAN OTOMATIS: UJI 1 (pukulan bandul) dan UJI 2 (tanjakan) untuk PD dan "
                                    "PD + CBF-QP 5 / 10 / 15 deg. Tekan lagi untuk membatalkan.")
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
        self.kode_mode = {QtCore.Qt.Key_1: "mode_pd", QtCore.Qt.Key_2: "mode_cbf", QtCore.Qt.Key_3: "banding",
                          QtCore.Qt.Key_6: "foto_pukul", QtCore.Qt.Key_7: "foto_miring"}
        self.kode_tombol = {QtCore.Qt.Key_W: "w", QtCore.Qt.Key_A: "a", QtCore.Qt.Key_S: "s",
                            QtCore.Qt.Key_D: "d", QtCore.Qt.Key_B: "b", QtCore.Qt.Key_F: "f", QtCore.Qt.Key_O: "o",
                            QtCore.Qt.Key_P: "p", QtCore.Qt.Key_Q: "q",
                            QtCore.Qt.Key_R: "r", QtCore.Qt.Key_E: "e", QtCore.Qt.Key_T: "t",
                            QtCore.Qt.Key_U: "u"}

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

    def daftar_centang(self):
        """list bool constraint 1 .. N (N = nomor h terbesar di kotak centang)."""
        jumlah = max(self.centang) if self.centang else 0
        return [self.centang[n].isChecked() if n in self.centang else False for n in range(1, jumlah + 1)]

    def kirim_centang(self):
        if not self.sedang_atur_centang:
            self.kirim(("centang", self.daftar_centang()))

    def atur_centang(self, centang):
        """Centang dari simulasi (awal). Dikirim balik sekali supaya simulasi dan jendela sama."""
        self.sedang_atur_centang = True
        for nomor, kotak in self.centang.items():
            kotak.setChecked(nomor - 1 < len(centang) and bool(centang[nomor - 1]))
        self.sedang_atur_centang = False
        self.kirim_centang()

    def saring_keyboard(self, kejadian):
        """Mengembalikan True bila tombol keyboard dipakai di sini (tidak diteruskan ke widget)."""
        QEvent = self.QtCore.QEvent
        jenis = kejadian.type()
        if jenis != QEvent.KeyPress and jenis != QEvent.KeyRelease:
            return False
        # Saat mengetik angka di kolom parameter, huruf tidak dipakai sebagai tombol.
        if isinstance(self.app.focusWidget(), self.QtWidgets.QLineEdit):
            return False
        if kejadian.key() in self.kode_mode:
            if jenis == QEvent.KeyPress and not kejadian.isAutoRepeat():
                self.kirim(("tombol", self.kode_mode[kejadian.key()]))
            return True
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
        # B F O P Q R E T U: dikirim sekali saat mulai ditekan
        if ditekan and not sebelumnya and huruf in ("b", "f", "o", "p", "q", "r", "e", "t", "u"):
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
        status dari simulasi: judul, mode_gerak (MODE_DIAM / MODE_BALANCING / MODE_BEBAS), tahan (gabungan Qt + keyboard
        PyBullet), dorong, arah_dorong"""
        self.status = status
        if self.root is None and self.tombol == {}:
            return
        mode_gerak = status["mode_gerak"]
        tahan = dict(status["tahan"])
        sendiri = self.tahan_sekarang()
        for huruf in sendiri:
            tahan[huruf] = tahan.get(huruf, False) or sendiri[huruf]

        for huruf in ("w", "a", "s", "d"):
            self.tombol[huruf].setStyleSheet(self.gaya_tombol(config.WARNA_QT_TEKAN, "white") if tahan[huruf] else "")
        self.tombol["f"].setStyleSheet(self.gaya_tombol(config.WARNA_QT_MODE, "white") if mode_gerak == MODE_BEBAS else "")
        self.tombol["b"].setStyleSheet(self.gaya_tombol(config.WARNA_QT_MODE, "white")
                                       if mode_gerak == MODE_BALANCING else "")
        menyala_o = tahan["o"] or status["arah_dorong"] > 0
        menyala_p = tahan["p"] or status["arah_dorong"] < 0
        self.tombol["o"].setStyleSheet(self.gaya_tombol(config.WARNA_QT_TEKAN, "white") if menyala_o else "")
        self.tombol["p"].setStyleSheet(self.gaya_tombol(config.WARNA_QT_TEKAN, "white") if menyala_p else "")

        mode_banding = status.get("mode_banding", False)
        uji_otomatis = status.get("uji_otomatis", False)
        self.tombol["t"].setStyleSheet(self.gaya_tombol(config.WARNA_QT_UJI, "white")
                                       if status.get("mode_pengujian", False) and not mode_banding
                                       and not uji_otomatis else "")
        self.tombol["u"].setStyleSheet(self.gaya_tombol(config.WARNA_QT_UJI, "white") if uji_otomatis else "")
        self.tombol_mode["banding"].setStyleSheet(self.gaya_tombol(config.WARNA_QT_BANDING, "white")
                                                  if mode_banding else "")

        # Mode kontrol: tombol mode yang aktif diberi warna kurvanya (PD ungu, PD + CBF-QP biru).
        cbf_ada = status.get("cbf_ada", True)
        self.tombol_mode["mode_pd"].setStyleSheet(self.gaya_tombol(config.WARNA_QT_PD, "white")
                                                  if status["judul"] == "PD" else "")
        self.tombol_mode["mode_cbf"].setStyleSheet(self.gaya_tombol(config.WARNA_QT_PD_CBF, "white")
                                                   if status["judul"] == "PD+CBF" else "")
        self.tombol_mode["mode_cbf"].setEnabled(cbf_ada)
        self.tombol_mode["mode_cbf"].setToolTip("" if cbf_ada else "CBF-QP gagal disiapkan (lihat terminal)")
        self.label_pembanding.setText(status.get("pembanding", ""))
        if status.get("komet", False):
            self.label_mode.setText("komet, motor mati  ->  R: ulang   E: simpan data   Q: keluar + simpan")
            self.label_mode.setStyleSheet("font-weight: bold; color: #b3261e")
        elif status.get("mode_pengujian", False):
            self.label_mode.setText(status.get("pengujian", "MODE PENGUJIAN"))
            self.label_mode.setStyleSheet("font-weight: bold; color: %s"
                                          % (config.WARNA_QT_BANDING if mode_banding else config.WARNA_QT_UJI))
        else:
            teks_gerak = {MODE_DIAM: "DIAM (tanpa Kp_theta)", MODE_BALANCING: "BALANCING (Kp_theta, B untuk DIAM)",
                          MODE_BEBAS: "BEBAS (tanpa PD heading, F untuk DIAM)"}.get(mode_gerak, mode_gerak)
            self.label_mode.setText("Mode kontrol: %s   |   %s" % (status["judul"], teks_gerak))
            self.label_mode.setStyleSheet("font-weight: bold")
        perintah = []
        if tahan["w"] and not tahan["s"]:
            perintah.append("maju")
        if tahan["s"] and not tahan["w"]:
            perintah.append("mundur")
        if tahan["a"] and not tahan["d"]:
            perintah.append("kiri")
        if tahan["d"] and not tahan["a"]:
            perintah.append("kanan")
        self.label_gerak.setText("gerak: " + (" + ".join(perintah) if len(perintah) > 0 else "diam"))
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
        self.judul = judul             # mode kontrol sekarang (ise.MODE_PD / ise.MODE_PD_CBF), lihat atur_mode()
        self.cbf_ada = True            # False = CBF-QP gagal disiapkan, tombol PD + CBF-QP dimatikan
        self.teks_pembanding = ""      # ditulis di jendela Kontrol
        self.plot_tahan = False        # True = data tidak dikirim ke plot realtime (perekam perbandingan menunggu O / P)
        self.centang_awal = []         # constraint CBF dicentang bila jendela Kontrol tidak ada

        p.connect(p.GUI if gui else p.DIRECT)
        p.setGravity(0.0, 0.0, -config.g)
        p.setPhysicsEngineParameter(fixedTimeStep=self.dt_fisika, numSolverIterations=50)
        if gui:
            # Shortcut bawaan PyBullet dimatikan supaya tombol W A S D B F O P tidak bentrok,
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
        # Bidang merah barrier psi (lihat gambar_bidang_safety_set)
        self.bidang = {}               # {+1: bidang depan (h1), -1: bidang belakang (h2)}: id benda PyBullet
        self.bidang_tingkat = {}       # tingkat kepekatan warna yang sedang tampil tiap bidang
        self.bidang_letak = {}         # (posisi, orientasi) yang sedang dipakai tiap bidang
        self.mask_cbf = None           # constraint CBF-QP yang dipakai (list bool h1 ..), dari atur_mask_plot

        # Lintasan UJI 2 pengujian otomatis (buat_lintasan): None = tidak ada
        self.lintasan = None           # dict ukuran lintasan (dunia)
        self.lintasan_benda = []       # id benda PyBullet lintasan
        self.catatan_terkirim = None   # catatan pengujian otomatis terakhir yang dikirim ke plot
        self.posisi_awal_badan = None  # posisi pusat massa badan saat pertama diletakkan (target kamera awal)
        self.foto_benda = []           # id benda visual FOTO (bandul, jalur, robot bayangan), dihapus foto_bersihkan
        self.foto_bandul = None        # dict id dan ukuran bandul FOTO (foto_buat_bandul)

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

        # Realisme dari uji hardware (config.py bagian 10)
        self.skala_torsi = config.RASIO_TORSI                           # torsi nyata / torsi perintah
        self.jeda_substep = int(round(config.SIM_JEDA_TORSI / self.dt_fisika))
        self.antrean_torsi = []        # ARUS yang sudah diperintah tetapi belum bekerja (satu per langkah fisika)
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
            print("Realisme: torsi roda x%.2f (K efektif %.3f N m/A), jeda torsi %.1f ms, model IMU + Kalman %s, "
                  "batas arus-kecepatan motor %s."
                  % (self.skala_torsi, config.MOTOR_KT_EFEKTIF, self.jeda_substep * self.dt_fisika * 1000,
                     "AKTIF" if config.SIM_MODEL_IMU else "MATI (psi sempurna)",
                     ("AKTIF (0 A di %.0f rpm)" % config.MOTOR_NO_LOAD_SPEED_RPM) if config.SIM_BATAS_KECEPATAN
                     else "MATI (motor ideal)"))
            print("Titik dorong O / P: bidang %.0f x %.0f mm, pusatnya %.1f mm di atas poros roda "
                  "(puncak badan %.1f mm), %.1f mm dari garis tengah badan."
                  % (config.DORONG_SISI * 1000, config.DORONG_SISI * 1000, self.tinggi_dorong * 1000, self.tinggi_atas * 1000,
                     config.JARAK_IMU_KE_SISI_LUAR * 1000))
            # Kamera DIAM tampak sisi kanan robot, diarahkan ke posisi awal robot. Hanya diatur sekali di sini
            # (tidak ikut robot), supaya arah gerak robot terlihat. Mouse tetap bisa memutar / zoom.
            posisi, _ = p.getBasePositionAndOrientation(self.robot)
            self.posisi_awal_badan = [float(v) for v in posisi]
            self.kamera_awal()
            self.buat_bidang_safety_set()
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

    def baca_yaw(self):
        """(yaw, dyaw) dari 'encoder' kedua roda, sama seperti robot fisik (pd_control.hitung_yaw).
        Joint PyBullet mengukur roda relatif badan (seperti DDSM115); psi saling meniadakan di selisih kiri-kanan."""
        kiri = p.getJointState(self.robot, self.joint_kiri)
        kanan = p.getJointState(self.robot, self.joint_kanan)
        yaw = hitung_yaw(self.tanda_kiri * kiri[0], self.tanda_kanan * kanan[0])
        dyaw = hitung_yaw(self.tanda_kiri * kiri[1], self.tanda_kanan * kanan[1])
        return yaw, dyaw

    def nolkan_theta(self):
        """Setpoint theta (titik nol s) dipindah ke posisi sekarang (tombol B, dan saat W / S dilepas). Hanya offset
        theta_nol yang digeser, jadi yaw dari joint roda tetap menyambung."""
        self.theta_nol = self.theta_nol + self.baca_state()[0]
        self.putus_garis_plot()

    def reset_plot(self):
        """Plot realtime dikosongkan dan waktunya mulai lagi dari 0 (dipanggil setiap robot diletakkan ulang)."""
        self.waktu = 0.0
        if self.plot is not None:
            # data lama dibuang, kiriman khusus (MODE, MASK, PEMBANDING) yang belum terkirim tetap dikirim
            self.plot_kiriman = [k for k in self.plot_kiriman if isinstance(k[0], str)] + [("RESET",)]

    def putus_garis_plot(self):
        """Garis di plot realtime diputus (robot diletakkan ulang atau theta dinolkan), supaya tidak ada
        garis lurus yang melompat dari keadaan lama ke keadaan baru."""
        if self.plot is not None and not self.plot_tahan:
            kosong = float("nan")
            self.plot_kiriman.append((self.waktu, kosong, kosong, kosong, kosong, kosong, config.psi_max))

    def atur_centang_awal(self, centang):
        """Centang awal constraint CBF (dari atera_main.py), dikirim ke jendela Kontrol."""
        self.centang_awal = [bool(c) for c in centang]
        if self.kontrol is not None:
            self.kontrol.kirim_centang(self.centang_awal)

    def baca_centang(self):
        """Constraint CBF yang dicentang sekarang (list bool, urutan DAFTAR_C)."""
        if self.kontrol is not None and self.kontrol.centang is not None:
            return list(self.kontrol.centang)
        return list(self.centang_awal)

    def atur_mask_plot(self, mask):
        """Constraint CBF aktif (list bool h1 ..):
            - ke plot realtime (barrier aktif merah, tidak aktif abu-abu)
            - ke bidang merah barrier psi di jendela PyBullet (bidang depan tampil bila h1 dipakai, belakang bila h2)"""
        self.mask_cbf = [bool(m) for m in mask]
        if self.plot is not None:
            self.plot_kiriman.append(("MASK", list(self.mask_cbf)))

    def atur_mode(self, mode):
        """Mode kontrol diganti (tombol PD / PD + CBF-QP): jendela Kontrol dan warna kurva plot ikut."""
        self.judul = mode
        if self.plot is not None:
            self.plot_kiriman.append(("MODE", mode))

    def kirim_pembanding(self, mode, label, data):
        """Kurva pembanding satu mode (data = hasil ise.baca_csv) dikirim ke plot realtime."""
        if self.plot is not None:
            self.plot_kiriman.append(("PEMBANDING", mode, label, list(data["psi_rad"]), list(data["dpsi_rad_s"]),
                                      list(data["theta_rad"]), list(data["dtheta_rad_s"]), list(data["t_s"])))

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
        # Yang diantrekan adalah ARUS perintah (i = u / MOTOR_KT), supaya batas arus yang bergantung
        # kecepatan roda (batas_arus_kecepatan) dihitung saat arus itu benar-benar bekerja.
        arus_kiri = u_kiri / config.MOTOR_KT
        arus_kanan = u_kanan / config.MOTOR_KT

        for _ in range(config.SUBSTEP):
            # (b) Arus baru bekerja jeda_substep langkah fisika (SIM_JEDA_TORSI detik) setelah diperintah.
            self.antrean_torsi.append((arus_kiri, arus_kanan))
            kiri_bekerja, kanan_bekerja = self.antrean_torsi.pop(0)
            # (c) Batas arus bergantung kecepatan motor (back-EMF), lalu
            # (a) torsi yang benar-benar keluar dari roda = arus x MOTOR_KT_EFEKTIF (= skala_torsi x MOTOR_KT).
            w_kiri = self.tanda_kiri * p.getJointState(self.robot, self.joint_kiri)[1]
            w_kanan = self.tanda_kanan * p.getJointState(self.robot, self.joint_kanan)[1]
            torsi_kiri = self.skala_torsi * config.MOTOR_KT * batas_arus_kecepatan(kiri_bekerja, w_kiri)
            torsi_kanan = self.skala_torsi * config.MOTOR_KT * batas_arus_kecepatan(kanan_bekerja, w_kanan)
            p.setJointMotorControl2(self.robot, self.joint_kiri, p.TORQUE_CONTROL,
                                    force=self.tanda_kiri * torsi_kiri)
            p.setJointMotorControl2(self.robot, self.joint_kanan, p.TORQUE_CONTROL,
                                    force=self.tanda_kanan * torsi_kanan)
            if self.sisa_dorong > 0.0:
                self.beri_gaya_dorong()
                self.sisa_dorong = self.sisa_dorong - self.dt_fisika
            p.stepSimulation()

        # (c) Sampel IMU baru untuk siklus berikutnya.
        self.perbarui_imu()

        # Catat data plot: psi, theta, dan arus tiap motor (i = u / K_t).
        self.waktu = self.waktu + config.DT
        if self.plot is not None and not self.plot_tahan:
            self.plot_kiriman.append((self.waktu, float(self.x_terakhir[2]), float(self.x_terakhir[3]),
                                      float(self.x_terakhir[0]), float(self.x_terakhir[1]),
                                      u / config.MOTOR_KT, config.psi_max))

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

        Mengembalikan daftar tombol yang baru ditekan: 'b', 'f', 'o', 'p', 'q', 'r', 'e', 't', 'u',
        'mode_pd' (tombol PD / keyboard 1), 'mode_cbf' (tombol PD + CBF-QP / keyboard 2),
        'banding' (perekam perbandingan / keyboard 3), 'foto_pukul' (keyboard 6), 'foto_miring' (keyboard 7).
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

        for huruf in ("b", "f", "o", "p", "q", "r", "e", "t", "u"):
            if baru_ditekan(huruf) and huruf not in hasil:
                hasil.append(huruf)
        for angka, kunci in (("1", "mode_pd"), ("2", "mode_cbf"), ("3", "banding"), ("6", "foto_pukul"),
                             ("7", "foto_miring")):
            if baru_ditekan(angka) and kunci not in hasil:
                hasil.append(kunci)
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

    # ------------------------------------------------- bidang merah barrier psi
    def buat_bidang_safety_set(self):
        """Dua bidang merah transparan berbentuk | | : di depan robot (h1, psi <= psi_max) dan di belakang robot
        (h2, psi >= -psi_max). Hanya gambar (tanpa collision), jadi tidak memengaruhi fisika."""
        setengah = [config.BIDANG_LEBAR / 2.0, 0.001, config.BIDANG_TINGGI / 2.0]
        for sisi in (1, -1):
            bentuk = p.createVisualShape(p.GEOM_BOX, halfExtents=setengah, rgbaColor=warna_bidang(0))
            self.bidang[sisi] = p.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=-1,
                                                  baseVisualShapeIndex=bentuk, basePosition=TEMPAT_SEMBUNYI)
            self.bidang_tingkat[sisi] = 0
            self.bidang_letak[sisi] = None
        self.gambar_bidang_safety_set()

    def barrier_psi_dipakai(self, sisi):
        """True bila barrier psi sisi itu dipakai CBF-QP: sisi +1 = h1, -1 = h2 (nomor dari config.CBF_BARRIER_H).
        Sebelum atera_main.py mengirim mask (atur_mask_plot), keduanya dianggap dipakai."""
        if self.mask_cbf is None:
            return True
        atas, bawah = config.CBF_BARRIER_H["psi"]
        indeks = (atas if sisi > 0 else bawah) - 1
        return indeks < len(self.mask_cbf) and self.mask_cbf[indeks]

    def gambar_bidang_safety_set(self):
        """Letak dan warna bidang merah barrier psi (dipanggil dari tampilkan()):
            - IKUT robot: di depan dan di belakang poros roda, searah depan robot (mendatar), berdiri di lantai
              (atau di permukaan lintasan UJI 2 bila bidangnya berada di atas lintasan)
            - jarak mendatar dari poros = L_IMU*sin(psi_max) + JARAK_IMU_KE_SISI_LUAR*cos(psi_max), jadi ikut berubah
              saat psi_max_DEG diubah di jendela Kontrol (sisi terluar badan di ketinggian IMU tepat menyentuh bidang
              saat psi = +-psi_max)
            - hanya tampil di mode PD + CBF-QP: bidang depan bila h1 dipakai, bidang belakang bila h2 dipakai
            - warna makin pekat saat psi mendekati barrier sisi itu (tingkat_bidang), paling pekat saat melewatinya
        PyBullet hanya dipanggil bila letak atau tingkat warnanya berubah."""
        if len(self.bidang) == 0:
            return
        psi_max = float(config.psi_max)
        jarak = config.L_IMU * math.sin(psi_max) + config.JARAK_IMU_KE_SISI_LUAR * math.cos(psi_max)
        poros = self.posisi_poros()
        _, _, depan = self.sumbu_badan()                    # arah depan robot, mendatar
        atas = np.array([0.0, 0.0, 1.0])
        sumbu_datar = np.cross(depan, atas)                 # arah poros roda, mendatar
        orientasi = quaternion_dari_matriks(np.column_stack([sumbu_datar, depan, atas]))
        psi = float(self.x_terakhir[2])   # psi hasil Kalman, sama dengan yang dibaca kontrol

        for sisi, bidang in self.bidang.items():
            tampil = self.judul == MODE_PD_CBF and self.barrier_psi_dipakai(sisi)
            if tampil:
                pusat = poros + sisi * jarak * depan
                pusat[2] = self.tinggi_tanah(pusat) + 0.5 * config.BIDANG_TINGGI       # berdiri di lantai / lintasan
                letak = ([float(v) for v in pusat], [float(v) for v in orientasi])
            else:
                letak = (list(TEMPAT_SEMBUNYI), [0.0, 0.0, 0.0, 1.0])
            if letak != self.bidang_letak.get(sisi):
                self.bidang_letak[sisi] = letak
                p.resetBasePositionAndOrientation(bidang, letak[0], letak[1])

            tingkat = tingkat_bidang(psi, psi_max, sisi) if tampil else 0
            if tingkat != self.bidang_tingkat.get(sisi):
                self.bidang_tingkat[sisi] = tingkat
                p.changeVisualShape(bidang, -1, rgbaColor=warna_bidang(tingkat))

    # ------------------------------------------------ lintasan UJI 2 dan kamera
    def posisi_poros(self):
        """Titik tengah poros roda (koordinat dunia) [m]."""
        posisi_link, rot = self.pose_badan()
        return posisi_link + rot @ self.poros_badan

    def tinggi_lintasan(self, titik):
        """Tinggi (z) permukaan lintasan UJI 2 di bawah titik (x, y) [m], None bila titik tidak di atas lintasan."""
        lin = self.lintasan
        if lin is None or abs(titik[0]) > 0.5 * lin["lebar"]:
            return None
        y = titik[1]
        if lin["y_mulai"] <= y <= lin["y_puncak"]:
            return (y - lin["y_mulai"]) * math.tan(lin["zeta"])
        if lin["y_puncak"] < y <= lin["y_akhir"]:
            return lin["tinggi"]
        return None

    def tinggi_tanah(self, titik):
        """Tinggi permukaan yang diinjak di bawah titik (x, y): lantai, atau lintasan UJI 2 bila lebih tinggi [m]."""
        z = self.tinggi_lantai(titik)
        z_lintasan = self.tinggi_lintasan(titik)
        return z if z_lintasan is None else max(z, z_lintasan)

    def buat_lintasan(self, y_mulai, zeta_deg, panjang_miring, datar_atas, lebar, tebal, warna_papan, warna_panggung):
        """Lintasan UJI 2 searah +Y dunia (depan robot saat diletakkan), di atas lantai datar:
            y < y_mulai                 : lantai PyBullet (bidang datar bawah)
            y_mulai .. y_puncak         : papan tanjakan zeta_deg, panjang_miring [m] di sepanjang permukaannya
            y_puncak .. y_akhir         : panggung (bidang datar atas) setinggi puncak tanjakan, panjang datar_atas [m]
        Papan = kotak tipis (tebal [m]) yang diputar zeta terhadap sumbu X: tepi bawah permukaannya tepat di lantai
        (y_mulai, z = 0) dan tepi atasnya tepat di sudut depan panggung (tanpa celah atau tangga)."""
        self.hapus_lintasan()
        zeta = math.radians(zeta_deg)
        dy = panjang_miring * math.cos(zeta)
        tinggi = panjang_miring * math.sin(zeta)
        y_puncak = y_mulai + dy
        y_akhir = y_puncak + datar_atas
        self.lintasan = {"y_mulai": float(y_mulai), "y_puncak": float(y_puncak), "y_akhir": float(y_akhir),
                         "tinggi": float(tinggi), "zeta": float(zeta), "lebar": float(lebar),
                         "panjang_miring": float(panjang_miring)}
        normal = np.array([0.0, -math.sin(zeta), math.cos(zeta)])           # normal permukaan papan
        tengah_permukaan = np.array([0.0, y_mulai + 0.5 * dy, 0.5 * tinggi])
        benda = [
            # papan tanjakan
            ([0.5 * lebar, 0.5 * panjang_miring, 0.5 * tebal], tengah_permukaan - 0.5 * tebal * normal,
             p.getQuaternionFromEuler([zeta, 0.0, 0.0]), warna_papan),
            # panggung bidang datar atas (berdiri di lantai)
            ([0.5 * lebar, 0.5 * datar_atas, 0.5 * tinggi], np.array([0.0, 0.5 * (y_puncak + y_akhir), 0.5 * tinggi]),
             [0.0, 0.0, 0.0, 1.0], warna_panggung),
        ]
        for setengah, pusat, orientasi, warna in benda:
            bentuk = p.createCollisionShape(p.GEOM_BOX, halfExtents=setengah)
            tampak = p.createVisualShape(p.GEOM_BOX, halfExtents=setengah, rgbaColor=warna)
            nomor = p.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=bentuk, baseVisualShapeIndex=tampak,
                                      basePosition=[float(v) for v in pusat], baseOrientation=list(orientasi))
            p.changeDynamics(nomor, -1, lateralFriction=config.GESEKAN_RODA)
            self.lintasan_benda.append(nomor)
        return self.lintasan

    def hapus_lintasan(self):
        """Hapus lintasan UJI 2 (bila ada)."""
        for nomor in self.lintasan_benda:
            try:
                p.removeBody(nomor)
            except Exception:
                pass
        self.lintasan_benda = []
        self.lintasan = None

    def atur_kamera(self, jarak, yaw, pitch, target):
        """Pasang kamera (diam sampai diatur lagi; mouse tetap bisa memutar / zoom)."""
        if self.gui:
            p.resetDebugVisualizerCamera(cameraDistance=float(jarak), cameraYaw=float(yaw), cameraPitch=float(pitch),
                                         cameraTargetPosition=[float(v) for v in target])

    def kamera_awal(self):
        """Kamera bawaan config.py (KAMERA_*), diarahkan ke posisi awal robot."""
        target = self.posisi_awal_badan if self.posisi_awal_badan is not None else [0.0, 0.0, 0.15]
        self.atur_kamera(config.KAMERA_JARAK, config.KAMERA_YAW, config.KAMERA_PITCH, target)

    # ----------------------------------------------- FOTO PENGUJIAN (tombol 6 / 7, alur di atera_main.FotoPengujian)
    # Semua benda FOTO hanya VISUAL (tanpa collision, massa 0): fisika robot tidak berubah.
    def foto_visual(self, bentuk, warna, posisi, orientasi=(0.0, 0.0, 0.0, 1.0), **ukuran):
        """Buat satu benda visual FOTO (dihapus foto_bersihkan). Mengembalikan id benda."""
        tampak = p.createVisualShape(bentuk, rgbaColor=list(warna), **ukuran)
        nomor = p.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=-1, baseVisualShapeIndex=tampak,
                                  basePosition=[float(v) for v in posisi], baseOrientation=list(orientasi))
        self.foto_benda.append(nomor)
        return nomor

    def foto_pasang_segmen(self, nomor, a, b):
        """Letakkan silinder FOTO (sumbu z lokalnya) di antara titik a dan b."""
        a = np.asarray(a, dtype=float)
        b = np.asarray(b, dtype=float)
        arah = (b - a) / np.linalg.norm(b - a)
        bantu = np.array([1.0, 0.0, 0.0]) if abs(arah[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        sumbu_x = np.cross(bantu, arah)
        sumbu_x = sumbu_x / np.linalg.norm(sumbu_x)
        sumbu_y = np.cross(arah, sumbu_x)
        p.resetBasePositionAndOrientation(nomor, (0.5 * (a + b)).tolist(),
                                          quaternion_dari_matriks(np.column_stack([sumbu_x, sumbu_y, arah])))

    def foto_buat_bandul(self, gantungan, maju, sumbu_roda, chi):
        """Bandul FOTO (gambar saja; gaya pukulan tetap applyExternalForce = beri_gaya_dorong):
            - gantungan (batang kecil searah sumbu roda) di titik gantung
            - bandul bayangan (transparan) di posisi lepas chi + titik-titik lintasan ayunannya
            - tali + bola yang bergerak (foto_atur_bandul)
        gantungan [m] dunia, maju = arah depan robot (mendatar), chi = sudut lepas [rad] ke BELAKANG robot."""
        panjang = config.BANDUL_PANJANG_TALI_M
        jari = FOTO_BOLA_JARI_M
        gantungan = np.asarray(gantungan, dtype=float)
        maju = np.asarray(maju, dtype=float) / np.linalg.norm(maju)
        sumbu = np.asarray(sumbu_roda, dtype=float) / np.linalg.norm(sumbu_roda)
        atas = np.array([0.0, 0.0, 1.0])

        def titik_bola(phi):
            return gantungan + panjang * (-math.sin(phi) * maju - math.cos(phi) * atas)

        self.foto_visual(p.GEOM_BOX, FOTO_WARNA_GANTUNGAN, gantungan + 0.008 * atas,
                         quaternion_dari_matriks(np.column_stack([sumbu, maju, atas])), halfExtents=[0.06, 0.008, 0.008])
        lepas = titik_bola(chi)
        self.foto_visual(p.GEOM_SPHERE, FOTO_WARNA_BAYANGAN, lepas, radius=jari)
        tali_lepas = self.foto_visual(p.GEOM_CYLINDER, FOTO_WARNA_BAYANGAN, gantungan, radius=FOTO_TALI_JARI_M,
                                      length=panjang - jari)
        self.foto_pasang_segmen(tali_lepas, gantungan, gantungan + (panjang - jari) / panjang * (lepas - gantungan))
        for k in range(1, FOTO_TITIK_BUSUR):
            self.foto_visual(p.GEOM_SPHERE, [0.30, 0.30, 0.30, 0.55], titik_bola(chi * k / FOTO_TITIK_BUSUR),
                             radius=0.0035)
        tali = self.foto_visual(p.GEOM_CYLINDER, FOTO_WARNA_TALI, gantungan, radius=FOTO_TALI_JARI_M,
                                length=panjang - jari)
        bola = self.foto_visual(p.GEOM_SPHERE, FOTO_WARNA_BOLA, lepas, radius=jari)
        self.foto_bandul = {"gantungan": gantungan, "maju": maju, "panjang": panjang, "jari": jari, "tali": tali,
                            "bola": bola, "titik_bola": titik_bola, "chi": float(chi)}
        self.foto_atur_bandul(chi)

    def foto_atur_bandul(self, phi):
        """Sudut bandul FOTO [rad] dari garis vertikal (+ = ke belakang robot)."""
        b = self.foto_bandul
        if b is None:
            return
        bola = b["titik_bola"](phi)
        p.resetBasePositionAndOrientation(b["bola"], bola.tolist(), [0.0, 0.0, 0.0, 1.0])
        self.foto_pasang_segmen(b["tali"], b["gantungan"],
                                b["gantungan"] + (b["panjang"] - b["jari"]) / b["panjang"] * (bola - b["gantungan"]))

    def foto_jalur_bawah(self, lin):
        """Jalur kuning (FOTO_WARNA_DATAR) di bidang datar bawah lintasan UJI 2, dari FOTO_JALUR_BAWAH_BELAKANG_M di
        belakang posisi awal robot sampai kaki tanjakan. Hanya gambar, tebal 0.8 mm di atas lantai."""
        y_awal = -FOTO_JALUR_BAWAH_BELAKANG_M
        panjang = lin["y_mulai"] - y_awal
        self.foto_visual(p.GEOM_BOX, FOTO_WARNA_DATAR, [0.0, y_awal + 0.5 * panjang, 0.0004],
                         halfExtents=[0.5 * lin["lebar"], 0.5 * panjang, 0.0004])

    def ambil_pose(self):
        """Pose robot sekarang (frame pusat massa PyBullet + sudut kedua roda + state terakhir)."""
        posisi, orientasi = p.getBasePositionAndOrientation(self.robot)
        roda = [p.getJointState(self.robot, j)[0] for j in (self.joint_kiri, self.joint_kanan)]
        return {"posisi": list(posisi), "orientasi": list(orientasi), "roda": roda, "x": np.array(self.x_terakhir)}

    def pasang_pose(self, pose):
        """FREEZE: robot dipindah ke pose tersimpan dan didiamkan (fisika TIDAK dilanjutkan setelahnya). Bidang merah
        dan bidang dorong ikut digambar ulang."""
        p.resetBasePositionAndOrientation(self.robot, pose["posisi"], pose["orientasi"])
        p.resetBaseVelocity(self.robot, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
        for joint, sudut in zip((self.joint_kiri, self.joint_kanan), pose["roda"]):
            p.resetJointState(self.robot, joint, sudut, 0.0)
        self.x_terakhir = np.array(pose["x"])
        self.sisa_dorong = 0.0
        if self.gui:
            self.gambar_bidang_safety_set()
            self.gambar_bidang_dorong()

    def pose_tegak(self, poros, psi=0.0):
        """Pose (frame pusat massa) robot dengan titik tengah poros roda di `poros` [m] dan sudut badan psi [rad],
        menghadap +Y dunia (rumus sama dengan reset)."""
        orientasi_link = p.getQuaternionFromEuler([math.pi / 2.0 + psi - self.sudut_com, 0.0, math.pi])
        rot = np.array(p.getMatrixFromQuaternion(orientasi_link)).reshape(3, 3)
        posisi_link = np.asarray(poros, dtype=float) - rot @ self.poros_badan
        posisi, orientasi = p.multiplyTransforms(posisi_link.tolist(), orientasi_link,
                                                 self.inersia_pos, self.inersia_orn)
        return list(posisi), list(orientasi)

    def foto_bayangan(self, posisi, orientasi, roda=(0.0, 0.0)):
        """Robot bayangan FOTO: salinan robot.urdf, diam (fixed base), tanpa collision, transparan
        (FOTO_WARNA_BAYANGAN). posisi / orientasi = frame pusat massa (ambil_pose / pose_tegak)."""
        nomor = p.loadURDF(self.path_urdf, useFixedBase=True, flags=p.URDF_USE_INERTIA_FROM_FILE)
        self.foto_benda.append(nomor)
        p.resetBasePositionAndOrientation(nomor, list(posisi), list(orientasi))
        for link in [-1] + list(range(p.getNumJoints(nomor))):
            p.setCollisionFilterGroupMask(nomor, link, 0, 0)
            p.changeVisualShape(nomor, link, rgbaColor=FOTO_WARNA_BAYANGAN)
        for joint, sudut in zip((self.joint_kiri, self.joint_kanan), roda):
            p.resetJointState(nomor, joint, sudut, 0.0)
        return nomor

    def foto_kamera(self, titik, ukuran, kotak):
        """Kamera FOTO: yaw config.KAMERA_YAW (sisi kanan robot, sama dengan kamera biasa), pitch FOTO_PITCH_DEG,
        sudut pandang FOTO_FOV_DEG. Jarak dan target dihitung supaya SEMUA titik (dunia) masuk ke bagian gambar
        kotak = (kiri, kanan, bawah, atas) [0 .. 1], ukuran = (lebar, tinggi) [px]. Kamera jendela PyBullet ikut
        diarahkan ke tempat yang sama. Mengembalikan (view, proj, mata): matriks getCameraImage dan letak kamera."""
        titik = np.asarray(titik, dtype=float)
        yaw = float(config.KAMERA_YAW)
        pitch = float(FOTO_PITCH_DEG)
        aspek = ukuran[0] / float(ukuran[1])
        kiri, kanan_k, bawah, atas_k = kotak
        tan_v = math.tan(0.5 * math.radians(FOTO_FOV_DEG))
        target = 0.5 * (titik.min(axis=0) + titik.max(axis=0))
        for _ in range(4):                       # geser target ke tengah titik-titik (dilihat dari kamera)
            V = np.array(p.computeViewMatrixFromYawPitchRoll(target.tolist(), 1.0, yaw, pitch, 0.0, 2)).reshape(4, 4).T
            kanan, atas, belakang = V[0, :3], V[1, :3], V[2, :3]
            relatif = titik - target
            xs = relatif @ kanan
            ys = relatif @ atas
            target = target + 0.5 * (xs.min() + xs.max()) * kanan + 0.5 * (ys.min() + ys.max()) * atas
        relatif = titik - target
        xs = relatif @ kanan
        ys = relatif @ atas
        dekat = max(0.0, float((relatif @ belakang).max()))          # titik yang lebih dekat ke kamera dari target
        # Lebar / tinggi gambar penuh di jarak d = 2 d tan_v aspek / 2 d tan_v; benda harus muat di bagian kotak.
        jarak = max((ys.max() - ys.min()) / (2.0 * tan_v * (atas_k - bawah)),
                    (xs.max() - xs.min()) / (2.0 * tan_v * aspek * (kanan_k - kiri))) + dekat
        # Tengah benda digeser ke tengah kotak (target = tengah gambar).
        target = target - (0.5 * (kiri + kanan_k) - 0.5) * 2.0 * jarak * tan_v * aspek * kanan \
            - (0.5 * (bawah + atas_k) - 0.5) * 2.0 * jarak * tan_v * atas
        view = p.computeViewMatrixFromYawPitchRoll(target.tolist(), jarak, yaw, pitch, 0.0, 2)
        proj = p.computeProjectionMatrixFOV(FOTO_FOV_DEG, aspek, 0.05, 4.0 * jarak + 5.0)
        mata = target + jarak * belakang
        if self.gui:
            kamera = p.getDebugVisualizerCamera()
            tan_gui = 1.0 / kamera[3][5] if kamera[3][5] != 0.0 else tan_v
            p.resetDebugVisualizerCamera(cameraDistance=jarak * tan_v / tan_gui, cameraYaw=yaw, cameraPitch=pitch,
                                         cameraTargetPosition=target.tolist())
        return list(view), list(proj), mata

    def foto_hapus_debug(self):
        """Hapus semua teks / garis debug PyBullet (teks parameter ditulis ulang otomatis oleh tampilkan)."""
        p.removeAllUserDebugItems()
        self.id_teks = {}
        self.isi_teks = {}

    def foto_ambil(self, view, proj, ukuran):
        """getCameraImage ukuran = (lebar, tinggi) dari kamera FOTO -> (rgb uint8 tinggi x lebar x 3, nama renderer).
        Teks / garis debug dihapus dulu supaya tidak ikut. Renderer OpenGL (sama dengan jendela PyBullet, ada bayangan);
        bila gagal, TinyRenderer (lebih lambat, tampilan sedikit berbeda)."""
        self.foto_hapus_debug()
        kesalahan = None
        for renderer, nama in ((p.ER_BULLET_HARDWARE_OPENGL, "OpenGL"), (p.ER_TINY_RENDERER, "TinyRenderer")):
            try:
                hasil = p.getCameraImage(int(ukuran[0]), int(ukuran[1]), viewMatrix=view, projectionMatrix=proj, shadow=1,
                                         renderer=renderer)
                rgb = np.reshape(np.asarray(hasil[2], dtype=np.uint8), (hasil[1], hasil[0], 4))[:, :, :3]
                if rgb.max() > 0:                # gambar hitam semua = renderer gagal diam-diam
                    return np.ascontiguousarray(rgb), nama
                kesalahan = RuntimeError("gambar %s hitam semua" % nama)
            except Exception as galat:
                kesalahan = galat
        raise RuntimeError("getCameraImage gagal: %s" % kesalahan)

    def foto_tampilkan_anotasi(self, anotasi):
        """Garis dan teks anotasi di jendela PyBullet (saat FREEZE), titik dunia sama dengan PNG ANOTASI."""
        if not self.gui:
            return
        for g in anotasi.get("garis", []):
            titik = np.asarray(g["titik"], dtype=float)
            warna = list(g.get("warna", (0, 0, 0)))[:3]
            for i in range(len(titik) - 1):
                if g.get("putus", False) and i % 2 == 1:
                    continue
                p.addUserDebugLine(titik[i].tolist(), titik[i + 1].tolist(), lineColorRGB=warna,
                                   lineWidth=max(1.0, g.get("tebal", 2.0)))
        for a in anotasi.get("panah", []):
            p.addUserDebugLine(list(a["dari"]), list(a["ke"]), lineColorRGB=list(a.get("warna", (0, 0, 0)))[:3],
                               lineWidth=max(1.0, a.get("tebal", 3.0)))
        for t in anotasi.get("teks", []):
            p.addUserDebugText(t["teks"].replace("\n", "  "), list(t["posisi"]),
                               textColorRGB=list(t.get("warna", (0, 0, 0)))[:3], textSize=1.3)

    def foto_bersihkan(self):
        """Hapus semua benda FOTO (bandul, jalur, robot bayangan) dan anotasinya."""
        for nomor in self.foto_benda:
            try:
                p.removeBody(nomor)
            except Exception:
                pass
        self.foto_benda = []
        self.foto_bandul = None
        if self.gui and p.isConnected():
            self.foto_hapus_debug()

    def kirim_catatan(self, baris):
        """Catatan pengujian otomatis (list teks) ke panel catatan jendela plot (hanya bila berubah)."""
        baris = [str(b) for b in baris]
        if self.plot is not None and baris != self.catatan_terkirim:
            self.catatan_terkirim = baris
            self.plot_kiriman.append(("CATATAN", baris))

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

    def gambar_teks(self, baris, baris_uji=None):
        """Blok parameter di kiri atas jendela PyBullet (baris: list teks), dan blok PENGUJIAN OTOMATIS di kiri bawah
        (baris_uji: list (teks, warna rgb), None / [] = tidak ada). (Visual tombol ada di jendela Kontrol.)
        Baris yang isinya berubah ditulis bergiliran (kedua blok satu giliran), config.BARIS_PER_TAMPIL baris tiap
        panggilan. Baris blok uji jarang berubah, jadi hampir tidak menambah beban."""
        isi = []
        for i in range(config.JUMLAH_BARIS_MAKS):
            teks = baris[i] if i < len(baris) else " "
            if teks.startswith(("TUNING", "komet", "PENGUJIAN")):
                warna = config.WARNA_PERINGATAN
            else:
                warna = config.WARNA_BIASA
            isi.append(("baris%d" % i, teks, 0.90 - config.JARAK_BARIS * i, warna, 1.2))
        baris_uji = baris_uji or []
        for i in range(config.JUMLAH_BARIS_UJI_MAKS):
            teks, warna = baris_uji[i] if i < len(baris_uji) else (" ", config.WARNA_BIASA)
            isi.append(("uji%d" % i, teks, config.TEKS_UJI_ATAS - config.JARAK_BARIS_UJI * i, list(warna),
                        config.UKURAN_TEKS_UJI))
        ditulis = 0
        for _ in range(len(isi)):
            i = self.giliran_baris % len(isi)
            self.giliran_baris = (i + 1) % len(isi)
            kunci, teks, sy, warna, ukuran = isi[i]
            if teks == " " and kunci not in self.id_teks:
                continue                # baris kosong yang belum pernah dipakai
            if self.tulis(kunci, teks, -0.96, sy, warna, ukuran):
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
    def tampilkan(self, baris, mode_gerak, komet=False, pengujian="", banding=False, baris_uji=None,
                  uji_otomatis=False):
        """Perbarui semua tampilan. Dipanggil dari atera_main.py (dan uji_otomatis.py) beberapa kali per detik.

        baris          : daftar teks parameter (ditulis di kiri atas jendela PyBullet)
        mode_gerak     : MODE_DIAM, MODE_BALANCING, atau MODE_BEBAS (pd_control.py)
        komet          : True = KOMET, motor dimatikan (ditampilkan di jendela Kontrol)
        pengujian      : teks status mode pengujian / perekam perbandingan ("" = tidak dalam mode itu)
        banding        : True = perekam perbandingan (tombol 3) menyala
        baris_uji      : blok tulisan PENGUJIAN OTOMATIS di kiri bawah jendela PyBullet, list (teks, warna rgb)
        uji_otomatis   : True = PENGUJIAN OTOMATIS (tombol U) sedang berjalan (tombol U menyala di jendela Kontrol)
        """
        if not self.gui:
            return
        self.hitung_tampil = self.hitung_tampil + 1

        # Kamera DIAM (tidak ikut robot, diatur sekali di __init__): di sini hanya dibaca untuk meletakkan teks.
        # Jarak dan sudut pandang tetap bisa diatur dengan mouse.
        kamera = p.getDebugVisualizerCamera()
        tengah = kamera[11]

        self.gambar_bidang_safety_set()
        self.gambar_bidang_dorong()
        self.atur_jangkar_teks(kamera, tengah)
        self.gambar_teks(baris, baris_uji)

        # Status untuk jendela Kontrol: tombol mana yang menyala, mode, dan dorongan.
        if self.kontrol is not None:
            if self.sisa_dorong > 0.0:
                ke = "KE DEPAN" if self.arah_dorong > 0 else "KE BELAKANG"
                teks_dorong = "dorong: %s %.1f N" % (ke, self.gaya_dorong)
                arah_dorong = self.arah_dorong
            else:
                teks_dorong = "dorong: -"
                arah_dorong = 0
            self.kontrol.kirim_status({"judul": self.judul, "mode_gerak": mode_gerak,
                                       "tahan": dict(self.ditahan), "dorong": teks_dorong,
                                       "arah_dorong": arah_dorong, "komet": bool(komet),
                                       "mode_pengujian": pengujian != "", "pengujian": pengujian,
                                       "cbf_ada": self.cbf_ada, "pembanding": self.teks_pembanding,
                                       "mode_banding": bool(banding), "uji_otomatis": bool(uji_otomatis)})
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