"""atera_main.py - Program utama ATERA (SEMENTARA, untuk simulasi PyBullet dengan keyboard).

Jalankan:
    python3 atera_main.py --PD            kontrol PD saja
    python3 atera_main.py --CBFQP         kontrol PD + CBF-QP
    tambahkan --plot untuk membuka jendela plot realtime (psi, theta, arus)

Satu kali jalan = satu mode kontrol. Semua perintah dan tombol dicetak di terminal saat program mulai.

Satu percobaan ISE = dari robot diletakkan (atau sejak E terakhir) sampai E ditekan, robot jatuh, atau keluar.
"""

import argparse
import math
import os
import time

import pybullet

import config
import ise
from pd_control import PDControl
from simulasi import NAMA_ALPHA, Simulasi

ZETA_TUNGGU = 0.5   # [s] zeta baru dipakai bila nilainya di jendela Kontrol tidak berubah selama ini
                    # (supaya robot tidak diletakkan ulang berkali-kali saat slider sedang digeser)

MODE_PD = "PD"
MODE_PD_CBF = "PD+CBF"

BANTUAN = """
====================== ATERA - simulasi PyBullet ======================
MENJALANKAN (satu kali jalan = satu mode kontrol)
  python3 atera_main.py --PD             kontrol PD saja
  python3 atera_main.py --CBFQP          kontrol PD + CBF-QP
  python3 atera_main.py --CBFQP --plot   --plot = tambah jendela plot realtime (psi, theta, arus)

TOMBOL (keyboard di jendela "ATERA - Kontrol", atau klik / tahan tombolnya dengan mouse;
        keyboard di jendela PyBullet juga tetap bisa sebagai cadangan)
  B        Mode Balancing : robot diam di tempat, W A S D diabaikan
  N        Mode Jalan     : W A S D dipakai
  W / S    maju / mundur (ditahan)
  A / D    belok kiri / kanan (ditahan)
  O        dorong robot ke depan
  P        dorong robot ke belakang
  E        simpan data dan plot ISE percobaan sekarang; robot tetap berjalan, percobaan baru dimulai
  Q        keluar dan simpan data dan plot ISE
  R        reset: robot diletakkan ulang di posisi awal, ISE mulai dari 0 (percobaan yang belum
           disimpan DIBUANG; tekan E dulu bila ingin disimpan). Nilai parameter di jendela Kontrol tetap.
  ROBOT JATUH (|psi| > SAFE_TILT_DEG): motor dimatikan, badan jatuh ke lantai, perekaman berhenti.
           Lanjutkan dengan R (ulang), E (simpan), atau Q (keluar + simpan).
  Berhenti juga bisa: tutup jendela PyBullet, atau Ctrl+C di terminal (hasil ISE tetap tersimpan)

JENDELA "ATERA - Kontrol"
  Butuh Qt (sekali saja, di venv):  pip install PyQt5
  Parameter: geser slider, atau ketik angka lalu Enter. Nilai awal dari config.py.
  Setelah mengetik angka tekan Enter, supaya huruf kembali dipakai sebagai tombol.
  Nilai TIDAK ditulis ke config.py: tekan "Cetak nilai ke terminal", lalu salin sendiri.
  Gain diubah saat direkam -> nama file ISE diberi akhiran _TUNING.
  zeta_DEG = kemiringan lantai [deg] (+ = menanjak ke depan). Bila diubah: lantai ikut miring, robot
           diletakkan ulang, percobaan yang belum disimpan dibuang (sama seperti R).

HASIL ISE (folder PLOT_EVALUASI)
  python3 ise.py --daftar                                         daftar semua percobaan
  python3 ise.py --bandingkan                                     PD terbaru vs PD+CBF terbaru
  python3 ise.py --bandingkan --pd 3 --cbf 5                      PD KE-03 vs PD+CBF KE-05
  python3 ise.py --bandingkan --pd 3 --cbf 5 --tanggal 20261007   sama, untuk tanggal tertentu
=======================================================================
"""


def simpan_percobaan(perekam):
    """Simpan percobaan ISE dan tulis hasilnya di terminal (berhasil, tidak ada data, atau gagal)."""
    try:
        nama = perekam.simpan()
    except Exception as kesalahan:
        print("GAGAL menyimpan ISE -> %s: %s" % (type(kesalahan).__name__, kesalahan))
        return
    if nama != "":
        print("Tersimpan: %s.csv dan .png" % perekam.path_terakhir)
    else:
        print("ISE tidak disimpan:", perekam.alasan)


def letakkan_ulang(sim, pd, perekam, mode):
    """Robot diletakkan ulang di posisi awal, posisi tahan roda kembali ke awal, percobaan baru dimulai."""
    sim.reset()
    pd.theta_setpoint = float(config.theta_setpoint)
    perekam.mulai(mode)


def main():
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--PD", action="store_true")
    parser.add_argument("--CBFQP", action="store_true")
    parser.add_argument("--plot", action="store_true")
    parser.add_argument("-h", "--help", action="store_true")
    pilihan = parser.parse_args()

    print(BANTUAN)
    if pilihan.help:
        return
    if pilihan.PD == pilihan.CBFQP:
        print("Pilih SATU mode kontrol:  python3 atera_main.py --PD   atau   python3 atera_main.py --CBFQP")
        return

    mode = MODE_PD if pilihan.PD else MODE_PD_CBF

    pd = PDControl()

    # CBF-QP memakai acados. Disiapkan sebelum jendela simulasi dibuka.
    # zeta tidak dikunci: CBF-QP membaca config.zeta tiap siklus, jadi ikut berubah bila zeta_DEG diubah.
    cbf = None
    if mode == MODE_PD_CBF:
        print("Menyiapkan CBF-QP (acados), tunggu beberapa detik ...")
        try:
            from cbf_qp import CBFQP
            cbf = CBFQP()
        except Exception as kesalahan:
            print("CBF-QP TIDAK BISA DIPAKAI -> %s: %s" % (type(kesalahan).__name__, kesalahan))
            print("Perbaiki dulu, atau jalankan dengan:  python3 atera_main.py --PD")
            return

    sim = Simulasi(gui=True, plot=pilihan.plot, judul=mode)

    if pd.Kp_psi == 0.0 and pd.Kd_psi == 0.0:
        print("PERHATIAN: Kp_psi dan Kd_psi di config.py masih 0, robot akan jatuh. Isi di jendela Kontrol.")
    if pd.Kvel == 0.0:
        print("PERHATIAN: Kvel di config.py masih 0, tombol W / S tidak berpengaruh.")
    if cbf is not None:
        print("CBF-QP: %d constraint di DAFTAR_C | Alpha dipakai: %s"
              % (cbf.NH, ", ".join([cbf.nama_alpha[i] for i in cbf.alpha_dipakai])))
        for i in cbf.alpha_dipakai:
            if getattr(config, cbf.nama_alpha[i]) <= 0.0:
                print("PERHATIAN: %s di config.py masih 0, CBF-QP tidak aktif (status ALPHA_NOL) sampai diisi."
                      % cbf.nama_alpha[i])

    mode_balancing = True          # True = Mode Balancing (B), False = Mode Jalan (N)
    jatuh = False                  # True = robot jatuh, motor mati, menunggu R / E / Q

    # Kemiringan lantai yang sedang dipakai, dan nilai baru yang sedang ditunggu (lihat ZETA_TUNGGU)
    zeta_dipakai = float(config.zeta_DEG)
    zeta_tunggu = zeta_dipakai
    waktu_zeta = time.time()

    perekam = ise.PerekamISE()
    perekam.mulai(mode)
    print("Hasil ISE disimpan di folder:", os.path.abspath(ise.FOLDER_PLOT))
    print("Mulai. Mode kontrol: %s | Mode Balancing | zeta %.1f deg" % (mode, zeta_dipakai))

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

            if "e" in tombol_baru:
                print("Tombol E: simpan data dan plot.")
                simpan_percobaan(perekam)
                perekam.mulai(mode)            # percobaan baru dimulai dari keadaan sekarang
                t = 0.0

            if "r" in tombol_baru:
                print("Tombol R: robot diletakkan ulang, ISE mulai dari 0 (data yang belum disimpan dibuang).")
                letakkan_ulang(sim, pd, perekam, mode)
                mode_balancing = True
                jatuh = False
                t = 0.0
                waktu_berikutnya = time.time()
                continue

            # ------------------------------------------- kemiringan lantai zeta
            zeta_baru = float(params["zeta_DEG"])
            if zeta_baru != zeta_tunggu:
                zeta_tunggu = zeta_baru
                waktu_zeta = time.time()
            if zeta_tunggu != zeta_dipakai and time.time() - waktu_zeta >= ZETA_TUNGGU:
                zeta_dipakai = zeta_tunggu
                config.zeta_DEG = zeta_dipakai
                config.zeta = math.radians(zeta_dipakai)       # dipakai model.py dan cbf_qp.py
                sim.ubah_zeta(config.zeta)
                letakkan_ulang(sim, pd, perekam, mode)
                mode_balancing = True
                jatuh = False
                t = 0.0
                waktu_berikutnya = time.time()
                print("Lantai diubah: zeta = %.1f deg. Robot diletakkan ulang, ISE mulai dari 0." % zeta_dipakai)
                continue

            for tombol in tombol_baru:
                if tombol == "b":
                    mode_balancing = True
                    # Robot ditahan di posisi sekarang: setpoint theta = theta sekarang.
                    # (theta sendiri tidak dinolkan, supaya jarak tempuh s = R*theta tetap dari posisi awal)
                    pd.theta_setpoint = float(sim.x_terakhir[0])
                    print("Mode Balancing: robot ditahan di s = %.2f m, W A S D diabaikan"
                          % (config.R * pd.theta_setpoint))
                if tombol == "n":
                    mode_balancing = False
                    print("Mode Jalan: W A S D dipakai")
                if tombol == "o":
                    sim.dorong(+1)
                    print("Didorong ke depan %.1f N" % sim.gaya_dorong)
                if tombol == "p":
                    sim.dorong(-1)
                    print("Didorong ke belakang %.1f N" % sim.gaya_dorong)

            # ------------------------------------------------ gain dari Kontrol
            # Nilai di jendela Kontrol langsung dipakai. Bila jendelanya tidak ada, isinya nilai config.py.
            pd.Kp_psi = params["Kp_psi"]
            pd.Kd_psi = params["Kd_psi"]
            pd.Kp_theta = params["Kp_theta"]
            pd.Kd_theta = params["Kd_theta"]
            pd.Kvel = params["Kvel"]
            for nama in NAMA_ALPHA:                # cbf_qp.py membaca Alpha dari config tiap siklus
                setattr(config, nama, params[nama])

            # Perintah gerak. Di Mode Balancing arah diabaikan (diatur di dalam PDControl).
            pd.set_command(sim.arah, mode_balancing)
            if mode_balancing:
                u_belok = 0.0
            else:
                u_belok = config.TORSI_BELOK * sim.belok

            # ------------------------------------------------------------ kontrol
            x = sim.baca_state()
            psi = x[2]

            if not jatuh and abs(psi) > math.radians(config.SAFE_TILT_DEG):
                jatuh = True
                print("ROBOT JATUH (psi = %.1f deg): motor dimatikan, perekaman berhenti. "
                      "Tekan R (ulang), E (simpan data), atau Q (keluar + simpan)." % math.degrees(psi))

            if jatuh:
                # Motor mati: tidak ada torsi, badan dibiarkan jatuh ke lantai.
                u_PD = 0.0
                u = 0.0
                u_belok = 0.0
                status_cbf = "MATI"
            else:
                u_PD = pd.compute_PD(x)
                if mode == MODE_PD:
                    # Batas arus motor: u_min <= u <= u_max
                    u = max(config.u_min, min(config.u_max, u_PD))
                    status_cbf = "-"
                else:
                    u = cbf.compute(x, u_PD)
                    status_cbf = cbf.status

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

            # Data ISE hanya direkam selama robot belum jatuh.
            if not jatuh:
                t = t + config.DT
                perekam.tambah(t, x, pd.error_theta, pd.error_dtheta, pd.error_psi, pd.error_dpsi,
                               u_PD, u, status_cbf, params, dorong)

            # ----------------------------------------------------------- tampilan
            if mode_balancing:
                gerak = "Mode Balancing"
            else:
                gerak = "Mode Jalan"
            if siklus % 10 == 0:
                baris = []
                if jatuh:
                    baris.append("ROBOT JATUH, motor mati  ->  R: ulang   E: simpan data   Q: keluar + simpan")
                elif mode == MODE_PD:
                    baris.append("%s | %s | zeta %.1f deg" % (mode, gerak, zeta_dipakai))
                else:
                    baris.append("%s | %s | CBF: %s | zeta %.1f deg" % (mode, gerak, status_cbf, zeta_dipakai))
                baris.append("psi %+.1f / %.1f deg    dpsi %+.2f / %.2f rad/s"
                             % (math.degrees(psi), math.degrees(config.psi_max), x[3], config.dpsi_max))
                baris.append("theta %+.2f rad (s %+.2f m)    dtheta %+.2f / %.2f rad/s"
                             % (x[0], config.R * x[0], x[1], config.dtheta_max))
                baris.append("u_PD %+.3f   u %+.3f N m    arus %+.2f / %.2f A"
                             % (u_PD, u, u / config.MOTOR_KT, config.MAX_CURRENT_A))
                baris.append("ISE psi %.6f    t %.1f s  (kecepatan x%.2f waktu nyata)" % (perekam.ise_psi, t, kecepatan))
                if len(perekam.gain_berubah) > 0:
                    baris.append("TUNING: " + " ".join(perekam.gain_berubah) + " diubah saat direkam")
                sim.tampilkan(baris, mode_balancing, jatuh)

            if siklus % 200 == 0 and not jatuh:
                print("t %6.1f s (x%.2f) | %-6s | %-14s | psi %+6.2f deg | s %+5.2f m | dtheta %+6.2f rad/s | "
                      "u_PD %+6.3f | u %+6.3f | %s" % (t, kecepatan, mode, gerak, math.degrees(psi), config.R * x[0],
                                                       x[1], u_PD, u, status_cbf))

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
        simpan_percobaan(perekam)
        sim.cetak_params()
        sim.tutup()


if __name__ == "__main__":
    main()