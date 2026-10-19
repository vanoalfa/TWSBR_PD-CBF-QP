"""atera_main.py - Program utama ATERA (SEMENTARA, untuk simulasi PyBullet dengan keyboard).

Jalankan:
    python3 atera_main.py            tanpa plot realtime
    python3 atera_main.py --plot     dengan jendela plot realtime

Mode kontrol PD / PD + CBF-QP dipilih dengan tombol di jendela Kontrol (atau keyboard 1 / 2). Mode awal = PD.
Mengganti mode: data perekaman biasa disimpan, lalu robot diletakkan ulang (seperti R) dengan mode baru.
Semua perintah dan tombol dicetak di terminal saat program mulai.

MODE GERAK (pd_control.py), sama dengan robot fisik (D-Pad, Tombol A):
    DIAM  : W / S tidak ditahan. Robot ditahan di setpoint theta (Kp_theta aktif), constraint s (h5, h6) dan
            KOMET s aktif.
    GERAK : W / S ditahan. Kp_theta dan constraint s diabaikan, Kvel*arah menggerakkan robot.
    Saat W / S DILEPAS, setpoint theta (titik nol s) dipasang tepat di posisi itu. Tombol B memasang setpoint roda
    (titik nol s + arah / yaw_ref) kapan saja. A / D = belok (tanpa W / S: berputar di tempat).
    BEBAS : tombol F (toggle). Seperti program lama: tanpa PD heading, Kp_theta dan constraint s diabaikan, belok A / D
            = selisih torsi TORSI_BELOK. Tekan F lagi: kembali DIAM / GERAK, setpoint dipasang di posisi itu.

CONSTRAINT CBF-QP: dipilih dengan kotak centang h1 .. h8 di jendela Kontrol (awal = C_MODE[MODE_DIAM] di WORKPLACE
cbf_qp.py). Saat GERAK / BEBAS, constraint yang tidak ada di C_MODE mode itu (h5, h6) ikut dilepas walau dicentang.
Plot: barrier yang sedang dipakai merah, lainnya abu-abu.

PEREKAM PERBANDINGAN (tombol 3): PD vs PD + CBF-QP dengan dorongan yang sama. Setiap rekaman yang selesai menjadi
PEMBANDING mode-nya (plot realtime: PD ungu, PD+CBF biru, pudar). Bila keduanya ada, gambar BANDING PERBANDINGAN
dibuat otomatis (sama dengan python3 ise.py --bandingkan --perbandingan). Disimpan di PLOT_EVALUASI/PERBANDINGAN.

Tiap siklus: perintah W A S D -> pd.set_command -> state -> KOMET -> DOB -> PD -> CBF-QP -> PD heading
(pd.compute_belok: roda kiri u + u_belok, roda kanan u - u_belok, supaya kedua roda bergerak bersama; BEBAS: selisih
torsi tetap) -> step.
DOB CBF-QP (cbf.dob_perbarui / cbf.dob_catat) juga jalan di mode PD supaya taksirannya tercatat di CSV.

Jenis perekaman ISE:
    PERCOBAAN : dari robot diletakkan (atau sejak E terakhir) sampai E ditekan, KOMET, atau keluar.
                Disimpan di PLOT_EVALUASI/PERCOBAAN.
    PENGUJIAN : mode pengujian (tombol T). Perekaman dimulai saat dorongan O / P diberikan dan berhenti
                otomatis setelah config.DURASI_PENGUJIAN detik. Disimpan di PLOT_EVALUASI/PENGUJIAN.
    PERBANDINGAN : perekam perbandingan (tombol 3), sama seperti PENGUJIAN, ditambah pembanding dan BANDING otomatis.
"""

import argparse
import math
import os
import time

import pybullet

import config
import ise
import model
from pd_control import MODE_BEBAS, MODE_DIAM, MODE_GERAK, PDControl
from simulasi import NAMA_ALPHA, Simulasi

BANTUAN = """
====================== ATERA - simulasi PyBullet ======================
MENJALANKAN
  python3 atera_main.py           tanpa plot realtime
  python3 atera_main.py --plot    dengan jendela plot realtime

TOMBOL (keyboard di jendela "ATERA - Kontrol", atau klik / tahan tombolnya dengan mouse;
        keyboard di jendela PyBullet juga tetap bisa sebagai cadangan)
  1 / 2    mode kontrol PD / PD + CBF-QP (tombol di atas jendela Kontrol). Data perekaman biasa disimpan,
           lalu robot diletakkan ulang dengan mode baru (pengujian yang sedang berjalan dibatalkan).
  3        nyala / mati PEREKAM PERBANDINGAN. Plot realtime kosong sampai O / P. Cara memakai:
             1 (PD) -> 3 -> R -> O (atau P) -> tunggu %.0f s (disimpan otomatis, jadi pembanding PD)
             2 (PD + CBF-QP) -> R -> O (dorongan sama) -> tunggu -> disimpan + gambar BANDING otomatis
           3 tidak perlu dimatikan di antaranya. Gaya dorong (Gaya_dorong) jangan diubah di antara keduanya.
  h1..h8   kotak centang constraint CBF-QP yang dipakai (di jendela Kontrol). Saat GERAK / BEBAS h5 / h6 dilepas.
  W / S    maju / mundur (ditahan) = mode GERAK: Kp_theta dan constraint s diabaikan, torsi Kvel*arah.
           Kecepatan tunak v = -R*Kvel*arah/Kd_theta = %+.2f m/s untuk W (Kvel di jendela Kontrol).
           Dilepas = mode DIAM: setpoint theta (titik nol s) dipasang di posisi itu, Kp_theta aktif lagi.
  A / D    belok kiri / kanan (ditahan), YAW_RATE_BELOK = %.2f rad/s (tanpa W / S: berputar di tempat)
  B        pasang setpoint roda di posisi sekarang: titik nol s (s = R*theta) dan arah (yaw_ref)
  F        mode BEBAS nyala / mati (robot fisik: Tombol A): tanpa PD heading, Kp_theta, constraint s; A / D = selisih
           torsi TORSI_BELOK. Saat dimatikan, setpoint roda dipasang di posisi itu.
  O        dorong robot ke depan
  P        dorong robot ke belakang
  E        simpan data dan plot ISE sekarang; robot tetap berjalan, perekaman baru dimulai
  Q        keluar dan simpan data dan plot ISE
  R        reset: robot diletakkan ulang di posisi awal, ISE dan plot realtime mulai dari 0 (data yang
           belum disimpan DIBUANG; tekan E dulu bila ingin disimpan). Nilai parameter tetap.
  T        masuk / keluar MODE PENGUJIAN. Di mode ini perekaman biasa berhenti; setiap O / P memulai
           satu PENGUJIAN: direkam %.0f s sejak dorongan, lalu disimpan otomatis (_PENGUJIAN KE-XX).
  KOMET    |psi| > SAFE_TILT_DEG, |s| > S_KOMET (hanya mode DIAM), atau |yaw - yaw_ref| > YAW_KOMET_DEG
           (berputar, bukan mode BEBAS): motor dimatikan,
           perekaman berhenti (pengujian yang sedang berjalan langsung disimpan). Lanjutkan dengan R, E, atau Q.
  Berhenti juga bisa: tutup jendela PyBullet, atau Ctrl+C di terminal (hasil ISE tetap tersimpan)

JENDELA "ATERA - Kontrol"
  Butuh Qt (sekali saja, di venv):  pip install PyQt5
  Parameter: ketik angka lalu Enter. Nilai awal dari config.py.
  Setelah mengetik angka tekan Enter, supaya huruf kembali dipakai sebagai tombol.
  Nilai TIDAK ditulis ke config.py: tekan "Cetak nilai ke terminal", lalu salin sendiri.
  Gain diubah saat direkam -> nama file ISE diberi akhiran _TUNING.
  zeta_DEG = kemiringan lantai [deg] (+ = menanjak ke depan). Bila diubah: lantai ikut miring, robot
           diletakkan ulang, data yang belum disimpan dibuang (sama seperti R).
  psi_max_DEG = batas |psi| yang dijaga CBF (h1, h2) [deg], langsung dipakai (bidang merah ikut pindah).
           Plot: merah = barrier CBF dicentang, abu-abu = tidak dicentang, hitam = batas KOMET (psi 40 deg,
           s 2 m), kuning / oranye = penanda 5 / 10 deg.

HASIL ISE (folder PLOT_EVALUASI/PERCOBAAN, /PENGUJIAN, /PERBANDINGAN, file .csv + .png + .eps)
  python3 ise.py --daftar                                         daftar semua percobaan
  python3 ise.py --bandingkan                                     PD terbaru vs PD+CBF terbaru
  python3 ise.py --bandingkan --pd 3 --cbf 5                      PD KE-03 vs PD+CBF KE-05
  python3 ise.py --bandingkan --pd 3 --cbf 5 --tanggal 20261007   sama, untuk tanggal tertentu
  tambahkan --pengujian / --perbandingan untuk hasil tombol T / 3, contoh:  python3 ise.py --daftar --perbandingan
=======================================================================
""" % (config.DURASI_PENGUJIAN, -config.R * config.Kvel / config.Kd_theta if config.Kd_theta > 0.0 else float("nan"),
       config.YAW_RATE_BELOK, config.DURASI_PENGUJIAN)


def simpan_ise(perekam):
    """Simpan perekaman ISE dan tulis hasilnya di terminal. Mengembalikan nama file ("" = tidak tersimpan)."""
    try:
        nama = perekam.simpan()
    except Exception as kesalahan:
        print("GAGAL menyimpan ISE -> %s: %s" % (type(kesalahan).__name__, kesalahan))
        return ""
    if nama != "":
        print("Tersimpan: %s.csv, .png, .eps" % perekam.path_terakhir)
    else:
        print("ISE tidak disimpan:", perekam.alasan)
    return nama


def simpan_pembanding(sim, perekam_uji, pembanding):
    """Simpan rekaman PERBANDINGAN (tombol 3) dan jadikan pembanding mode-nya (pembanding: dict mode -> nama file).
    Bila pembanding PD dan PD+CBF sudah ada, gambar BANDING PERBANDINGAN dibuat otomatis."""
    mode = perekam_uji.mode
    nama = simpan_ise(perekam_uji)
    if nama == "":
        return
    pembanding[mode] = nama
    try:
        data = ise.baca_csv(perekam_uji.path_terakhir + ".csv")
        cocok = ise.POLA_NAMA.match(nama + ".csv")
        label = "%s KE-%s" % (mode, cocok.group(5)) if cocok else mode
        sim.kirim_pembanding(mode, label, data)
    except Exception as kesalahan:
        print("Pembanding tidak bisa digambar -> %s: %s" % (type(kesalahan).__name__, kesalahan))
    sim.teks_pembanding = teks_pembanding(pembanding)
    print("PEMBANDING %s = %s" % (mode, nama))
    if pembanding[ise.MODE_PD] and pembanding[ise.MODE_PD_CBF]:
        print("Pembanding PD dan PD+CBF ada: membuat gambar BANDING PERBANDINGAN ...")
        try:
            ise.perintah_bandingkan(None, None, None, ise.JENIS_PERBANDINGAN,
                                    nama_pd=pembanding[ise.MODE_PD], nama_cbf=pembanding[ise.MODE_PD_CBF])
        except Exception as kesalahan:
            print("GAGAL membuat gambar BANDING -> %s: %s" % (type(kesalahan).__name__, kesalahan))


def teks_pembanding(pembanding):
    """Teks untuk jendela Kontrol, contoh: 'pembanding  PD: ..._PENGUJIAN KE-03  |  PD+CBF: -'."""
    bagian = []
    for mode in (ise.MODE_PD, ise.MODE_PD_CBF):
        nama = pembanding[mode]
        cocok = ise.POLA_NAMA.match(nama + ".csv") if nama else None
        bagian.append("%s: %s" % (mode, ("KE-%s (%s)" % (cocok.group(5), cocok.group(2))) if cocok else "-"))
    return "pembanding (tombol 3)   " + "   |   ".join(bagian)


def letakkan_ulang(sim, perekam, mode, pd, cbf):
    """Robot diletakkan ulang di posisi awal (plot realtime ikut direset, titik nol s = posisi awal), perekaman
    percobaan baru dimulai. PD mulai DIAM (BEBAS mati) dengan arah sekarang sebagai setpoint heading,
    DOB dikosongkan."""
    sim.reset()
    perekam.mulai(mode, ise.JENIS_PERCOBAAN)
    pd.atur_bebas(False)
    pd.set_command(0, 0)
    pd.pasang_yaw_ref(sim.baca_yaw()[0])
    if cbf is not None:
        cbf.reset_dob()


def data_ekstra(pd, cbf, mode):
    """Kolom CSV tambahan (ise.KOLOM_EKSTRA): arah, bebas, yaw, u_belok, DOB, slack."""
    d_dtheta, d_dpsi = cbf.d_hat if cbf is not None else (0.0, 0.0)
    return {"arah": pd.arah, "bebas": float(pd.bebas),
            "yaw_rad": pd.yaw, "dyaw_rad_s": pd.dyaw, "yaw_ref_rad": pd.yaw_ref, "u_belok_Nm": pd.u_belok,
            "dob_dtheta_rad_s2": d_dtheta, "dob_dpsi_rad_s2": d_dpsi,
            "slack": cbf.slack if (cbf is not None and mode == ise.MODE_PD_CBF) else 0.0}


def main():
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--plot", action="store_true")
    parser.add_argument("-h", "--help", action="store_true")
    pilihan, lainnya = parser.parse_known_args()

    print(BANTUAN)
    if pilihan.help:
        return
    if lainnya:
        print("Argumen diabaikan: %s (mode kontrol sekarang dipilih di jendela Kontrol)" % " ".join(lainnya))

    # ASUMSI: mode kontrol awal = PD (bisa diganti kapan saja dengan tombol / keyboard 1 / 2).
    mode = ise.MODE_PD

    pd = PDControl()

    # CBF-QP memakai CasADi, selalu disiapkan sebelum jendela simulasi dibuka supaya tombol PD + CBF-QP bisa
    # langsung dipakai. Bila gagal, simulasi tetap jalan dengan mode PD saja (tombol PD + CBF-QP dimatikan).
    # config.CBF_TAHU_ZETA = False (= robot fisik): model CBF-QP memakai zeta = 0 walau lantai dimiringkan, gangguan
    # kemiringan ditaksir DOB. True: CBF-QP membaca config.zeta tiap siklus (ikut berubah bila zeta_DEG diubah).
    cbf = None
    print("Menyiapkan CBF-QP (CasADi) ...")
    try:
        from cbf_qp import CBFQP
        cbf = CBFQP(zeta=None if config.CBF_TAHU_ZETA else 0.0)
    except Exception as kesalahan:
        print("CBF-QP TIDAK BISA DIPAKAI -> %s: %s" % (type(kesalahan).__name__, kesalahan))
        print("Simulasi tetap jalan, hanya mode PD.")

    sim = Simulasi(gui=True, plot=pilihan.plot, judul=mode)
    sim.cbf_ada = cbf is not None
    pembanding = {ise.MODE_PD: "", ise.MODE_PD_CBF: ""}       # nama file PERBANDINGAN pembanding tiap mode
    sim.teks_pembanding = teks_pembanding(pembanding)

    # Constraint CBF-QP yang dipakai = kotak centang h1 .. h8 di jendela Kontrol (awal: C_MODE[MODE_DIAM]), dan saat
    # GERAK hanya yang juga ada di C_MODE[MODE_GERAK].
    if cbf is not None:
        jumlah_h = cbf.NH
        centang_awal = [bool(m) for m in cbf.mask_mode[MODE_DIAM]]
    else:
        jumlah_h = max(max(pasangan) for pasangan in config.CBF_BARRIER_H.values())
        centang_awal = [False] * jumlah_h
    sim.atur_centang_awal(centang_awal)
    centang_dipakai = None                 # centang yang sedang dipakai (None = belum diterapkan)
    mask_dipakai = None                    # constraint yang sedang dipakai CBF-QP = centang dan mode gerak

    if pd.Kp_psi == 0.0 and pd.Kd_psi == 0.0:
        print("PERHATIAN: Kp_psi dan Kd_psi di config.py masih 0, robot akan komet. Isi di jendela Kontrol.")
    if cbf is not None:
        print("CBF-QP: %d constraint di DAFTAR_C | Alpha dipakai: %s"
              % (cbf.NH, ", ".join([cbf.nama_alpha[i] for i in cbf.alpha_dipakai])))
        for i in cbf.alpha_dipakai:
            if getattr(config, cbf.nama_alpha[i]) <= 0.0:
                print("PERHATIAN: %s di config.py masih 0, CBF-QP tidak aktif (status ALPHA_NOL) sampai diisi."
                      % cbf.nama_alpha[i])

    komet = False                  # True = KOMET, motor mati, menunggu R / E / Q

    # Mode pengujian (tombol T)
    mode_pengujian = False         # True = perekaman biasa berhenti, O / P memulai pengujian
    pengujian_berjalan = False     # True = sedang merekam satu pengujian
    t_uji = 0.0                    # waktu sejak dorongan pengujian [s]

    # Perekam perbandingan (tombol 3)
    mode_banding = False           # True = perekaman biasa berhenti, plot kosong sampai O / P
    banding_berjalan = False       # True = sedang merekam satu perbandingan
    t_banding = 0.0                # waktu sejak dorongan [s]

    # Kemiringan lantai yang sedang dipakai, dan nilai baru yang sedang ditunggu (lihat ZETA_TUNGGU)
    zeta_dipakai = float(config.zeta_DEG)
    zeta_tunggu = zeta_dipakai
    waktu_zeta = time.time()

    perekam = ise.PerekamISE()             # perekaman biasa (PERCOBAAN)
    perekam.mulai(mode, ise.JENIS_PERCOBAAN)
    pd.pasang_yaw_ref(sim.baca_yaw()[0])
    perekam_uji = ise.PerekamISE()         # mode pengujian (PENGUJIAN)
    perekam_banding = ise.PerekamISE()     # perekam perbandingan (PERBANDINGAN)
    print("Hasil ISE disimpan di folder:")
    print("   ", os.path.abspath(ise.folder_jenis(ise.JENIS_PERCOBAAN)))
    print("   ", os.path.abspath(ise.folder_jenis(ise.JENIS_PENGUJIAN)))
    print("   ", os.path.abspath(ise.folder_jenis(ise.JENIS_PERBANDINGAN)))
    print("Mulai. Mode kontrol: %s | mode DIAM | zeta %.1f deg" % (mode, zeta_dipakai))

    t = 0.0
    siklus = 0
    waktu_berikutnya = time.time()
    # Kecepatan simulasi dibanding waktu nyata (x1.00 = sama cepat). Bila komputer lambat, nilainya < 1:
    # t simulasi berjalan lebih lambat dari jam dinding.
    waktu_ukur = time.time()
    siklus_ukur = 0
    kecepatan = 1.0

    try:
        while sim.masih_terbuka():
            # ----------------------------------------------------------- keyboard
            tombol_baru = sim.baca_keyboard()
            # Parameter dibaca sesudah keyboard dan sebelum tombol diproses, supaya gaya dorong yang
            # dicetak saat O / P ditekan adalah nilai terbaru dari jendela Kontrol.
            params = sim.baca_params()

            if "q" in tombol_baru:
                print("Tombol Q: keluar dan simpan.")
                break

            if "banding" in tombol_baru:
                if not mode_banding:
                    if mode_pengujian:
                        if pengujian_berjalan:
                            print("Pengujian yang sedang berjalan dibatalkan (tidak disimpan).")
                        mode_pengujian = False
                        pengujian_berjalan = False
                        print("MODE PENGUJIAN (T) dimatikan.")
                    mode_banding = True
                    banding_berjalan = False
                    perekam.mulai(mode, ise.JENIS_PERCOBAAN)     # perekaman biasa berhenti (data dibuang)
                    sim.plot_tahan = True
                    sim.reset_plot()
                    print("PEREKAM PERBANDINGAN nyala (mode %s): tekan R (robot diletakkan ulang), lalu O / P untuk "
                          "mulai merekam %.0f s. Tekan 3 lagi untuk mematikan." % (mode, config.DURASI_PENGUJIAN))
                else:
                    if banding_berjalan:
                        print("Rekaman perbandingan yang sedang berjalan dibatalkan (tidak disimpan).")
                    mode_banding = False
                    banding_berjalan = False
                    sim.plot_tahan = False
                    perekam.mulai(mode, ise.JENIS_PERCOBAAN)
                    t = 0.0
                    print("PEREKAM PERBANDINGAN mati: perekaman biasa dimulai lagi.")

            if "t" in tombol_baru:
                if mode_banding:
                    if banding_berjalan:
                        print("Rekaman perbandingan yang sedang berjalan dibatalkan (tidak disimpan).")
                    mode_banding = False
                    banding_berjalan = False
                    sim.plot_tahan = False
                    print("PEREKAM PERBANDINGAN (3) dimatikan.")
                if not mode_pengujian:
                    mode_pengujian = True
                    pengujian_berjalan = False
                    perekam.mulai(mode, ise.JENIS_PERCOBAAN)     # perekaman biasa berhenti (data dibuang)
                    print("MODE PENGUJIAN: perekaman biasa berhenti (data yang belum disimpan dibuang). "
                          "Tekan O / P untuk memulai satu pengujian (%.0f s). Tekan T lagi untuk keluar."
                          % config.DURASI_PENGUJIAN)
                else:
                    if pengujian_berjalan:
                        print("Pengujian yang sedang berjalan dibatalkan (tidak disimpan).")
                    mode_pengujian = False
                    pengujian_berjalan = False
                    perekam.mulai(mode, ise.JENIS_PERCOBAAN)
                    t = 0.0
                    print("Keluar dari MODE PENGUJIAN: perekaman biasa dimulai lagi.")

            if "e" in tombol_baru:
                if mode_banding:
                    if banding_berjalan:
                        print("Tombol E: rekaman perbandingan disimpan sebelum %.0f s." % config.DURASI_PENGUJIAN)
                        simpan_pembanding(sim, perekam_banding, pembanding)
                        banding_berjalan = False
                        sim.plot_tahan = True
                        sim.reset_plot()
                    else:
                        print("Tombol E: tidak ada rekaman perbandingan yang sedang berjalan.")
                elif mode_pengujian:
                    if pengujian_berjalan:
                        print("Tombol E: pengujian disimpan sebelum %.0f s." % config.DURASI_PENGUJIAN)
                        simpan_ise(perekam_uji)
                        pengujian_berjalan = False
                    else:
                        print("Tombol E: tidak ada pengujian yang sedang berjalan.")
                else:
                    print("Tombol E: simpan data dan plot.")
                    simpan_ise(perekam)
                    perekam.mulai(mode, ise.JENIS_PERCOBAAN)     # percobaan baru dari keadaan sekarang
                    t = 0.0

            # ------------------------------------------- mode kontrol PD / PD + CBF-QP
            mode_baru = None
            if "mode_pd" in tombol_baru:
                mode_baru = ise.MODE_PD
            if "mode_cbf" in tombol_baru:
                mode_baru = ise.MODE_PD_CBF
            if mode_baru is not None and mode_baru != mode:
                if mode_baru == ise.MODE_PD_CBF and cbf is None:
                    print("PD + CBF-QP tidak bisa dipilih: CBF-QP gagal disiapkan (lihat pesan saat program mulai).")
                else:
                    if pengujian_berjalan:
                        print("Pengujian yang sedang berjalan dibatalkan (tidak disimpan).")
                        pengujian_berjalan = False
                    if banding_berjalan:
                        print("Rekaman perbandingan yang sedang berjalan dibatalkan (tidak disimpan).")
                        banding_berjalan = False
                        sim.plot_tahan = True
                    if not mode_pengujian and not mode_banding:
                        print("Ganti mode kontrol: data %s disimpan dulu." % mode)
                        simpan_ise(perekam)
                    mode = mode_baru
                    sim.atur_mode(mode)
                    letakkan_ulang(sim, perekam, mode, pd, cbf)
                    komet = False
                    t = 0.0
                    waktu_berikutnya = time.time()
                    print("Mode kontrol: %s. Robot diletakkan ulang, ISE mulai dari 0." % mode)
                    continue

            if "r" in tombol_baru:
                if pengujian_berjalan:
                    print("Pengujian yang sedang berjalan dibatalkan (tidak disimpan).")
                    pengujian_berjalan = False
                if banding_berjalan:
                    print("Rekaman perbandingan yang sedang berjalan dibatalkan (tidak disimpan).")
                    banding_berjalan = False
                    sim.plot_tahan = True
                print("Tombol R: robot diletakkan ulang, ISE dan plot mulai dari 0 (data yang belum disimpan dibuang).")
                letakkan_ulang(sim, perekam, mode, pd, cbf)
                komet = False
                t = 0.0
                waktu_berikutnya = time.time()
                continue

            # ------------------------------------------- kemiringan lantai zeta
            zeta_baru = float(params["zeta_DEG"])
            if zeta_baru != zeta_tunggu:
                zeta_tunggu = zeta_baru
                waktu_zeta = time.time()
            if zeta_tunggu != zeta_dipakai and time.time() - waktu_zeta >= config.ZETA_TUNGGU:
                zeta_dipakai = zeta_tunggu
                config.zeta_DEG = zeta_dipakai
                config.zeta = math.radians(zeta_dipakai)       # dipakai model.py dan cbf_qp.py
                sim.ubah_zeta(config.zeta)
                if pengujian_berjalan:
                    print("Pengujian yang sedang berjalan dibatalkan (tidak disimpan).")
                    pengujian_berjalan = False
                if banding_berjalan:
                    print("Rekaman perbandingan yang sedang berjalan dibatalkan (tidak disimpan).")
                    banding_berjalan = False
                    sim.plot_tahan = True
                letakkan_ulang(sim, perekam, mode, pd, cbf)
                komet = False
                t = 0.0
                waktu_berikutnya = time.time()
                print("Lantai diubah: zeta = %.1f deg. Robot diletakkan ulang, ISE mulai dari 0." % zeta_dipakai)
                continue

            for tombol in tombol_baru:
                if tombol == "b":
                    # Setpoint roda = posisi dan arah sekarang (titik nol s dipakai PD, barrier CBF, KOMET, dan ISE).
                    sim.nolkan_theta()
                    pd.pasang_yaw_ref(sim.baca_yaw()[0])
                    print("Tombol B: setpoint roda dipasang di sini (s = 0, arah sekarang).")
                if tombol == "f":
                    pd.atur_bebas(not pd.bebas)
                    if pd.bebas:
                        print("Mode BEBAS: PD heading, Kp_theta, constraint s mati. Tekan F lagi untuk kembali.")
                    else:
                        sim.nolkan_theta()                   # setpoint roda = posisi dan arah sekarang
                        pd.pasang_yaw_ref(sim.baca_yaw()[0])
                        print("Mode BEBAS mati: setpoint dipasang di sini (s = 0, arah sekarang).")
                if tombol == "o" or tombol == "p":
                    arah_dorong = 1 if tombol == "o" else -1
                    teks_arah = "depan" if arah_dorong > 0 else "belakang"
                    if mode_banding:
                        if komet:
                            print("komet: tekan R dulu sebelum memulai rekaman perbandingan.")
                        elif banding_berjalan:
                            print("Rekaman perbandingan masih berjalan (%.1f / %.0f s), dorongan diabaikan."
                                  % (t_banding, config.DURASI_PENGUJIAN))
                        else:
                            sim.dorong(arah_dorong)
                            perekam_banding.mulai(mode, ise.JENIS_PERBANDINGAN)
                            t_banding = 0.0
                            banding_berjalan = True
                            sim.plot_tahan = False
                            sim.reset_plot()                  # plot realtime mulai dari dorongan ini
                            print("PERBANDINGAN %s dimulai: didorong ke %s %.1f N, direkam %.0f s."
                                  % (mode, teks_arah, sim.gaya_dorong, config.DURASI_PENGUJIAN))
                    elif not mode_pengujian:
                        sim.dorong(arah_dorong)
                        print("Didorong ke %s %.1f N" % (teks_arah, sim.gaya_dorong))
                    elif komet:
                        print("komet: tekan R dulu sebelum memulai pengujian.")
                    elif pengujian_berjalan:
                        print("Pengujian masih berjalan (%.1f / %.0f s), dorongan diabaikan."
                              % (t_uji, config.DURASI_PENGUJIAN))
                    else:
                        sim.dorong(arah_dorong)
                        perekam_uji.mulai(mode, ise.JENIS_PENGUJIAN)
                        t_uji = 0.0
                        pengujian_berjalan = True
                        print("PENGUJIAN dimulai: didorong ke %s %.1f N, direkam %.0f s."
                              % (teks_arah, sim.gaya_dorong, config.DURASI_PENGUJIAN))

            # ------------------------------------------------ gain dari Kontrol
            # Nilai di jendela Kontrol langsung dipakai. Bila jendelanya tidak ada, isinya nilai config.py.
            pd.Kp_psi = params["Kp_psi"]
            pd.Kd_psi = params["Kd_psi"]
            pd.Kp_theta = params["Kp_theta"]
            pd.Kd_theta = params["Kd_theta"]
            pd.Kvel = params["Kvel"]
            for nama in NAMA_ALPHA:                # cbf_qp.py membaca Alpha dari config tiap siklus
                setattr(config, nama, params[nama])
            if params["psi_max_DEG"] > 0.0:        # batas psi CBF (h1, h2), dibaca cbf_qp.py tiap siklus
                config.psi_max_DEG = params["psi_max_DEG"]
                config.psi_max = math.radians(config.psi_max_DEG)
                config.psi_min = -config.psi_max

            # Perintah gerak -> mode gerak otomatis. W / S baru DILEPAS (GERAK -> DIAM): setpoint theta dipasang
            # tepat di posisi ini (sebelum state dibaca, jadi siklus ini sudah memakai theta = 0 di sini).
            gerak_sebelumnya = pd.mode_gerak
            pd.set_command(sim.arah, sim.belok)
            if gerak_sebelumnya == MODE_GERAK and pd.mode_gerak == MODE_DIAM:
                sim.nolkan_theta()

            # Constraint CBF-QP = kotak centang di jendela Kontrol DAN yang ada di C_MODE mode gerak sekarang.
            centang = sim.baca_centang()
            centang = (centang + [False] * jumlah_h)[:jumlah_h]
            if centang != centang_dipakai:
                centang_dipakai = centang
                print("Constraint CBF-QP dicentang: %s" % (ise.teks_mask(centang)))
            mask = list(centang)
            if cbf is not None:
                mask = [centang[k] and cbf.mask_mode[pd.mode_gerak][k] > 0.0 for k in range(jumlah_h)]
            if mask != mask_dipakai:
                mask_dipakai = mask
                if cbf is not None:
                    cbf.pilih_barrier([k + 1 for k in range(jumlah_h) if mask[k]])
                sim.atur_mask_plot(mask)

            # ------------------------------------------------------------ kontrol
            x = sim.baca_state()
            yaw, dyaw = sim.baca_yaw()                      # dari encoder kedua roda
            psi = x[2]
            s = model.hitung_s(x)

            # KOMET: |psi| > SAFE_TILT_DEG, |s| > S_KOMET (hanya mode DIAM), atau berputar (bukan mode BEBAS)
            sebab = ""
            if abs(psi) > math.radians(config.SAFE_TILT_DEG):
                sebab = "psi = %.1f deg" % math.degrees(psi)
            elif pd.mode_gerak == MODE_DIAM and abs(s) > config.S_KOMET:
                sebab = "s = %.2f m" % s
            elif pd.mode_gerak != MODE_BEBAS and abs(yaw - pd.yaw_ref) > math.radians(config.YAW_KOMET_DEG):
                sebab = "berputar, yaw - yaw_ref = %.0f deg" % math.degrees(yaw - pd.yaw_ref)
            if not komet and sebab != "":
                komet = True
                if cbf is not None:
                    cbf.reset_dob()
                print("komet (%s): motor dimatikan, perekaman berhenti. "
                      "Tekan R (ulang), E (simpan data), atau Q (keluar + simpan)." % sebab)
                if pengujian_berjalan:
                    print("komet saat pengujian: pengujian disimpan sampai saat komet.")
                    simpan_ise(perekam_uji)
                    pengujian_berjalan = False
                if banding_berjalan:
                    print("komet saat rekaman perbandingan: disimpan sampai saat komet.")
                    simpan_pembanding(sim, perekam_banding, pembanding)
                    banding_berjalan = False
                    sim.plot_tahan = True

            nilai_c = None                  # nilai constraint CBF-QP (dicatat di CSV), hanya mode PD + CBF-QP
            if komet:
                # Motor mati: tidak ada torsi, badan dibiarkan jatuh ke lantai.
                u_PD = 0.0
                u = 0.0
                u_belok = 0.0
                pd.u_belok = 0.0
                status_cbf = "MATI"
            else:
                # urutan: DOB (awal siklus) -> PD -> CBF-QP -> heading -> DOB (catat u)
                if cbf is not None:
                    cbf.dob_perbarui(x, config.DT)
                u_PD = pd.compute(x)
                if mode == ise.MODE_PD:
                    # Batas arus motor: u_min <= u <= u_max
                    u = max(config.u_min, min(config.u_max, u_PD))
                    status_cbf = "-"
                else:
                    u = cbf.compute(x, u_PD)
                    status_cbf = cbf.status
                    # c_k pada u terpakai. Dicatat kosong bila constraint tidak dipakai (mask 0) atau CBF-QP tidak
                    # dijalankan (status ALPHA_NOL, nilai c = NaN).
                    nilai_c = [float(c) if m > 0.0 and math.isfinite(c) else None for c, m in zip(cbf.c, cbf.mask)]
                # PD heading: kedua roda bergerak bersama (sisa torsi u_max - |u| saja)
                u_belok = pd.compute_belok(yaw, dyaw, u, config.u_max, config.DT)
                if cbf is not None:
                    cbf.dob_catat(x, u)

            # Gaya dorongan yang bekerja di siklus ini (dicatat di CSV).
            if sim.sisa_dorong > 0.0:
                dorong = sim.arah_dorong * sim.gaya_dorong
            else:
                dorong = 0.0

            sim.step(u, u_belok)
            siklus = siklus + 1
            siklus_ukur = siklus_ukur + 1
            if siklus_ukur >= 200:
                sekarang = time.time()
                kecepatan = siklus_ukur * config.DT / max(sekarang - waktu_ukur, 1e-6)
                waktu_ukur = sekarang
                siklus_ukur = 0

            # ------------------------------------------------------- perekaman ISE
            # Data hanya direkam selama belum KOMET.
            if not komet:
                if mode_banding:
                    if banding_berjalan:
                        t_banding = t_banding + config.DT
                        perekam_banding.tambah(t_banding, x, pd.error_theta, pd.error_dtheta, pd.error_psi,
                                               pd.error_dpsi, u_PD, u, status_cbf, params, dorong, nilai_c,
                                               data_ekstra(pd, cbf, mode))
                        if t_banding >= config.DURASI_PENGUJIAN - 0.5 * config.DT:
                            print("PERBANDINGAN %s selesai (%.0f s)." % (mode, config.DURASI_PENGUJIAN))
                            simpan_pembanding(sim, perekam_banding, pembanding)
                            banding_berjalan = False
                            sim.plot_tahan = True
                            sim.reset_plot()              # kurva sekarang dihapus, tinggal pembanding (pudar)
                            lain = ise.MODE_PD_CBF if mode == ise.MODE_PD else ise.MODE_PD
                            if not pembanding[lain]:
                                print("Berikutnya: tekan %s (mode %s), R, lalu O / P dengan dorongan yang sama."
                                      % ("2" if lain == ise.MODE_PD_CBF else "1", lain))
                elif not mode_pengujian:
                    t = t + config.DT
                    perekam.tambah(t, x, pd.error_theta, pd.error_dtheta, pd.error_psi, pd.error_dpsi,
                                   u_PD, u, status_cbf, params, dorong, nilai_c, data_ekstra(pd, cbf, mode))
                elif pengujian_berjalan:
                    t_uji = t_uji + config.DT
                    perekam_uji.tambah(t_uji, x, pd.error_theta, pd.error_dtheta, pd.error_psi, pd.error_dpsi,
                                       u_PD, u, status_cbf, params, dorong, nilai_c, data_ekstra(pd, cbf, mode))
                    if t_uji >= config.DURASI_PENGUJIAN - 0.5 * config.DT:
                        print("PENGUJIAN selesai (%.0f s)." % config.DURASI_PENGUJIAN)
                        simpan_ise(perekam_uji)
                        pengujian_berjalan = False

            # ----------------------------------------------------------- tampilan
            gerak = "mode " + pd.mode_gerak
            if mode_banding:
                perekam_aktif = perekam_banding
                t_tampil = t_banding
                if banding_berjalan:
                    teks_uji = "PEREKAM PERBANDINGAN | %s | merekam t %.1f / %.0f s" \
                        % (mode, t_banding, config.DURASI_PENGUJIAN)
                else:
                    teks_uji = "PEREKAM PERBANDINGAN | %s | tekan R lalu O / P (rekam %.0f s), 3 untuk mematikan" \
                        % (mode, config.DURASI_PENGUJIAN)
            elif mode_pengujian:
                perekam_aktif = perekam_uji
                t_tampil = t_uji
                if pengujian_berjalan:
                    teks_uji = "MODE PENGUJIAN | %s | pengujian berjalan t %.1f / %.0f s" \
                        % (mode, t_uji, config.DURASI_PENGUJIAN)
                else:
                    teks_uji = "MODE PENGUJIAN | %s | tekan O / P untuk mulai (%.0f s), T untuk keluar" \
                        % (mode, config.DURASI_PENGUJIAN)
            else:
                perekam_aktif = perekam
                t_tampil = t
                teks_uji = ""

            if siklus % 10 == 0:
                baris = []
                if komet:
                    baris.append("komet, motor mati  ->  R: ulang   E: simpan data   Q: keluar + simpan")
                elif mode_pengujian or mode_banding:
                    baris.append(teks_uji)
                elif mode == ise.MODE_PD:
                    baris.append("%s | %s | zeta %.1f deg" % (mode, gerak, zeta_dipakai))
                else:
                    baris.append("%s | %s | CBF: %s | zeta %.1f deg" % (mode, gerak, status_cbf, zeta_dipakai))
                baris.append("psi %+.1f / %.1f deg    dpsi %+.2f / %.2f rad/s"
                             % (math.degrees(psi), math.degrees(config.psi_max), x[3], config.dpsi_max))
                baris.append("theta %+.2f rad (s %+.2f m)    dtheta %+.2f / %.2f rad/s"
                             % (x[0], s, x[1], config.dtheta_max))
                baris.append("u_PD %+.3f   u %+.3f N m    arus %+.2f / %.2f A"
                             % (u_PD, u, u / config.MOTOR_KT, config.MAX_CURRENT_A))
                # heading, perintah gerak, DOB, slack
                d_dtheta, d_dpsi = cbf.d_hat if cbf is not None else (0.0, 0.0)
                baris.append("yaw %+.1f (ref %+.1f) deg  u_belok %+.3f  |  arah %+d belok %+d  |  DOB ddpsi %+.1f  "
                             "slack %.2g" % (math.degrees(yaw), math.degrees(pd.yaw_ref), pd.u_belok, pd.arah,
                                             pd.belok, d_dpsi,
                                             cbf.slack if (cbf is not None and mode == ise.MODE_PD_CBF) else 0.0))
                baris.append("ISE psi %.6f    t %.1f s  (kecepatan x%.2f waktu nyata)"
                             % (perekam_aktif.ise_psi, t_tampil, kecepatan))
                if len(perekam_aktif.gain_berubah) > 0:
                    baris.append("TUNING: " + " ".join(perekam_aktif.gain_berubah) + " diubah saat direkam")
                sim.tampilkan(baris, pd.mode_gerak, komet, teks_uji, mode_banding)

            if siklus % 200 == 0 and not komet:
                print("t %6.1f s (x%.2f) | %-6s | %-10s | psi %+6.2f deg | s %+5.2f m | dtheta %+6.2f rad/s | "
                      "u_PD %+6.3f | u %+6.3f | %s" % (t_tampil, kecepatan, mode, gerak, math.degrees(psi),
                                                       s, x[1], u_PD, u, status_cbf))

            # Jalankan simulasi dengan kecepatan waktu nyata.
            waktu_berikutnya = waktu_berikutnya + config.DT
            jeda = waktu_berikutnya - time.time()
            if jeda > 0.0:
                time.sleep(jeda)
            else:
                waktu_berikutnya = time.time()

    except KeyboardInterrupt:
        print("Dihentikan dari keyboard.")
    except pybullet.error:
        print("Jendela PyBullet ditutup.")
    finally:
        # Selalu dijalankan, juga bila program berhenti karena error: simpan dulu, baru tutup jendela.
        if banding_berjalan:
            print("Rekaman perbandingan belum selesai: disimpan sampai saat ini.")
            simpan_pembanding(sim, perekam_banding, pembanding)
        elif pengujian_berjalan:
            print("Pengujian belum selesai: disimpan sampai saat ini.")
            simpan_ise(perekam_uji)
        elif not mode_pengujian and not mode_banding:
            simpan_ise(perekam)
        else:
            print("Tidak ada pengujian / rekaman perbandingan yang belum disimpan.")
        sim.cetak_params()
        sim.tutup()


if __name__ == "__main__":
    main()