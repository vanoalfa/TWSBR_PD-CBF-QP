"""atera_main.py - Program utama ATERA (SEMENTARA, untuk simulasi PyBullet dengan keyboard).

Jalankan:
    python3 atera_main.py                 jendela PyBullet + jendela Kontrol + jendela plot realtime
    python3 atera_main.py --tanpa-plot    tanpa jendela plot realtime (--plot tetap diterima, sama dengan bawaan)

PENGUJIAN OTOMATIS (tombol U, uji_otomatis.py): UJI 1 pukulan bandul 30 .. 90 deg di bidang datar dan UJI 2 tanjakan
7.5 deg, untuk PD dan PD + CBF-QP psi_max 5 / 10 / 15 deg, berurutan tanpa disentuh. U lagi = batal. Hasil di
PLOT_EVALUASI/PENGUJIAN/<PD | PDCBFQP5 | PDCBFQP10 | PDCBFQP15>/ (config.py bagian 13).

FOTO PENGUJIAN (tombol 6 / 7, kelas FotoPengujian di bawah, mode kontrol = yang sedang dipilih):
    6  bidang datar, dipukul bandul chi_DEG (jendela Kontrol) -> FREEZE + PNG di |psi| maksimum
    7  naik bidang miring UJI2_ZETA_DEG (tanjakan merah muda, bidang datar kuning) -> FREEZE + PNG di tengah tanjakan
    PNG BERSIH + ANOTASI di PLOT_EVALUASI/FOTO/<PD | PDCBFQP..>/ (FOTO_* di simulasi.py).

Mode kontrol PD / PD + CBF-QP dipilih dengan tombol di jendela Kontrol (atau keyboard 1 / 2). Mode awal = PD.
Mengganti mode: data perekaman biasa disimpan, lalu robot diletakkan ulang (seperti R) dengan mode baru.
Semua perintah dan tombol dicetak di terminal saat program mulai.

MODE GERAK (pd_control.py), sama dengan robot fisik (Tombol B, Tombol A):
    DIAM      : default. Tanpa Kp_theta (posisi tidak ditahan).
    BALANCING : tombol B (toggle dengan DIAM; dari BEBAS langsung BALANCING). Kp_theta dan KOMET s aktif, setpoint
                theta (titik nol s) + arah (yaw_ref) dipasang saat masuk BALANCING dan saat W / S DILEPAS.
    BEBAS     : tombol F. Tanpa PD heading dan Kp_theta, belok A / D = selisih torsi TORSI_BELOK. F lagi -> DIAM.
    W / S (maju / mundur, Kvel*arah) dan A / D (belok) berlaku di semua mode.

CONSTRAINT CBF-QP: dipilih dengan kotak centang h1 .. h4 di jendela Kontrol (awal = C_MODE[MODE_DIAM] di WORKPLACE
cbf_qp.py). Plot: barrier yang sedang dipakai merah, lainnya abu-abu.

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
    PENGUJIAN : tombol T = mulai merekam sekarang (perekaman biasa tetap berjalan, dorong dengan O / P), T lagi =
                selesai dan disimpan (tanpa batas waktu, sama dengan L3 robot fisik). Disimpan di PLOT_EVALUASI/PENGUJIAN.
    PERBANDINGAN : perekam perbandingan (tombol 3): direkam config.DURASI_PERBANDINGAN detik sejak dorongan O / P,
                ditambah pembanding dan BANDING otomatis.
"""

import argparse
import math
import os
import re
import time
from datetime import datetime

import numpy as np
import pybullet

import config
import ise
import model
import simulasi
import uji_otomatis
from pd_control import MODE_BALANCING, MODE_BEBAS, MODE_DIAM, PDControl
from simulasi import NAMA_ALPHA, Simulasi

BANTUAN = """
====================== ATERA - simulasi PyBullet ======================
MENJALANKAN
  python3 atera_main.py                jendela PyBullet, Kontrol, dan plot realtime
  python3 atera_main.py --tanpa-plot   tanpa jendela plot realtime

TOMBOL (keyboard di jendela "ATERA - Kontrol", atau klik / tahan tombolnya dengan mouse;
        keyboard di jendela PyBullet juga tetap bisa sebagai cadangan)
  1 / 2    mode kontrol PD / PD + CBF-QP (tombol di atas jendela Kontrol). Data perekaman biasa disimpan,
           lalu robot diletakkan ulang dengan mode baru (pengujian yang sedang berjalan dibatalkan).
  3        nyala / mati PEREKAM PERBANDINGAN. Plot realtime kosong sampai O / P. Cara memakai:
             1 (PD) -> 3 -> R -> O (atau P) -> tunggu %.0f s (disimpan otomatis, jadi pembanding PD)
             2 (PD + CBF-QP) -> R -> O (dorongan sama) -> tunggu -> disimpan + gambar BANDING otomatis
           3 tidak perlu dimatikan di antaranya. Gaya dorong (Gaya_dorong) jangan diubah di antara keduanya.
  h1..h4   kotak centang constraint CBF-QP yang dipakai (di jendela Kontrol): h1 h2 psi, h3 h4 dtheta.
  W / S    maju / mundur (ditahan), torsi Kvel*arah, semua mode (di BALANCING Kp_theta diabaikan selama ditahan).
           Kecepatan tunak v = -R*Kvel*arah/Kd_theta = %+.2f m/s untuk W (Kvel di jendela Kontrol).
           Dilepas di mode BALANCING: setpoint theta (titik nol s) dipasang di posisi itu.
  A / D    belok kiri / kanan (ditahan), YAW_RATE_BELOK = %.2f rad/s (tanpa W / S: berputar di tempat)
  B        mode BALANCING nyala / mati (robot fisik: Tombol B). Nyala: Kp_theta aktif, setpoint roda (titik nol s
           dan arah) dipasang di posisi sekarang. Mati: kembali DIAM. Saat BEBAS: langsung BALANCING.
  F        mode BEBAS nyala / mati (robot fisik: Tombol A): tanpa PD heading dan Kp_theta, A / D = selisih torsi
           TORSI_BELOK. Mati: kembali DIAM.
  U        PENGUJIAN OTOMATIS: semua pengujian berurutan (UJI 1 bandul %s deg di bidang datar, lalu UJI 2
           tanjakan %g deg %g m) untuk PD, PD+CBF-QP %s deg. Gain = nilai jendela Kontrol saat U ditekan.
           Selama berjalan tombol lain diabaikan; U lagi = batal, Q = batal lalu keluar. Hasil:
           PLOT_EVALUASI/PENGUJIAN/<PD | PDCBFQP..>/YYYYMMDD_HHmmSS_PD_30_Simulasi_PENGUJIAN KE-XX (.csv .png .eps)
  6        FOTO dipukul (mode kontrol sekarang): bidang datar, bandul chi_DEG (jendela Kontrol, awal %g deg) dilepas
           dan memukul dari belakang. FREEZE di |psi| MAKSIMUM, PNG BERSIH + ANOTASI disimpan.
  7        FOTO bidang miring (mode kontrol sekarang): naik tanjakan %g deg dengan Kvel. FREEZE saat poros roda di
           TENGAH tanjakan, PNG BERSIH + ANOTASI disimpan. Hasil 6 / 7: PLOT_EVALUASI/FOTO/<PD | PDCBFQP..>/
           Saat FREEZE: 6 / 7 foto lagi, 1 / 2 ganti mode, R / U kembali, Q keluar. Selama berjalan: U batal.
  O        dorong robot ke depan
  P        dorong robot ke belakang
  E        simpan data dan plot ISE sekarang; robot tetap berjalan, perekaman baru dimulai
  Q        keluar dan simpan data dan plot ISE
  R        reset: robot diletakkan ulang di posisi awal, ISE dan plot realtime mulai dari 0 (data yang
           belum disimpan DIBUANG; tekan E dulu bila ingin disimpan). Nilai parameter tetap.
  T        PENGUJIAN mulai (perekaman biasa tetap berjalan; dorong dengan O / P). T lagi: PENGUJIAN selesai dan
           disimpan (_PENGUJIAN KE-XX). Tanpa batas waktu (robot fisik: L3). Urutan: 1 (PD), T .. T, lalu
           2 (PD + CBF-QP), T .. T: PENGUJIAN PD+CBF otomatis dibandingkan dengan PENGUJIAN PD terakhir (BANDING).
  KOMET    |psi| > SAFE_TILT_DEG, |s| > S_KOMET (hanya mode BALANCING tanpa W / S), atau |yaw - yaw_ref| > YAW_KOMET_DEG
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
""" % (config.DURASI_PERBANDINGAN, -config.R * config.Kvel / config.Kd_theta if config.Kd_theta > 0.0 else float("nan"),
       config.YAW_RATE_BELOK, " ".join("%g" % s for s in config.UJI1_SUDUT_BANDUL_DEG), config.UJI2_ZETA_DEG,
       config.UJI2_PANJANG_MIRING_M, " / ".join("%g" % s for s in config.UJI_PSI_MAX_DEG),
       simulasi.FOTO_CHI_AWAL_DEG, config.UJI2_ZETA_DEG)


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


_UJI_PD_TERAKHIR = [""]       # PENGUJIAN PD terakhir sesi ini (pasangan BANDING otomatis)


def simpan_uji(perekam_uji):
    """Simpan PENGUJIAN (tombol T). PENGUJIAN PD+CBF: gambar BANDING otomatis dengan PENGUJIAN PD terakhir hari itu
    (config.EVAL_BANDING_OTOMATIS). Mengembalikan nama file ("" = tidak tersimpan)."""
    mode = perekam_uji.mode
    nama = simpan_ise(perekam_uji)
    if nama == "":
        return ""
    if mode == ise.MODE_PD:
        _UJI_PD_TERAKHIR[0] = nama
    elif config.EVAL_BANDING_OTOMATIS:
        pasangan = _UJI_PD_TERAKHIR[0] or ise.pengujian_pd_terakhir()
        if pasangan == "":
            print("BANDING otomatis dilewati: belum ada PENGUJIAN PD hari ini (tekan 1, T, dorong, T dulu).")
        else:
            print("BANDING otomatis: %s vs %s" % (pasangan, nama))
            try:
                ise.perintah_bandingkan(None, None, None, ise.JENIS_PENGUJIAN, nama_pd=pasangan, nama_cbf=nama)
            except Exception as kesalahan:
                print("GAGAL membuat gambar BANDING -> %s: %s" % (type(kesalahan).__name__, kesalahan))
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
    percobaan baru dimulai. PD mulai DIAM dengan arah sekarang sebagai setpoint heading, DOB dikosongkan."""
    sim.reset()
    perekam.mulai(mode, ise.JENIS_PERCOBAAN)
    pd.atur_mode(MODE_DIAM)
    pd.set_command(0, 0)
    pd.pasang_yaw_ref(sim.baca_yaw()[0])
    if cbf is not None:
        cbf.reset_dob()


def data_ekstra(pd, cbf, mode):
    """Kolom CSV tambahan (ise.KOLOM_EKSTRA): arah, bebas, balancing, yaw, u_belok, DOB, slack."""
    d_dtheta, d_dpsi = cbf.d_hat if cbf is not None else (0.0, 0.0)
    return {"arah": pd.arah, "bebas": float(pd.mode_gerak == MODE_BEBAS),
            "balancing": float(pd.mode_gerak == MODE_BALANCING),
            "yaw_rad": pd.yaw, "dyaw_rad_s": pd.dyaw, "yaw_ref_rad": pd.yaw_ref, "u_belok_Nm": pd.u_belok,
            "dob_dtheta_rad_s2": d_dtheta, "dob_dpsi_rad_s2": d_dpsi,
            "slack": cbf.slack if (cbf is not None and mode == ise.MODE_PD_CBF) else 0.0}


# =============================================================================
# FOTO PENGUJIAN (tombol 6 / 7)
# =============================================================================
FOTO_PUKUL = "foto_pukul"          # tombol 6: Pengujian 1, bidang datar, dipukul bandul
FOTO_MIRING = "foto_miring"        # tombol 7: Pengujian 2, naik bidang miring
POLA_FOTO = re.compile(r"_FOTO KE-(\d+)_(?:BERSIH|ANOTASI)\.png$")
BIRU = (0.0, 0.31, 0.69)
JINGGA = (0.80, 0.42, 0.0)
MERAH = (0.80, 0.0, 0.0)
HITAM = (0.0, 0.0, 0.0)


def nama_foto(folder, kode_kontrol, kode_uji):
    """YYYYMMDD_HHmmSS_<kode_kontrol>_<kode_uji>_Simulasi_FOTO KE-XX (KE-XX = nomor terbesar di folder + 1)."""
    os.makedirs(folder, exist_ok=True)
    nomor = [int(c.group(1)) for c in (POLA_FOTO.search(f) for f in os.listdir(folder)) if c]
    sekarang = datetime.now()
    return "%s_%s_%s_Simulasi_FOTO KE-%02d" % (sekarang.strftime("%Y%m%d_%H%M%S"), kode_kontrol, kode_uji,
                                               max(nomor, default=0) + 1)


def busur(pusat, jari, sudut_awal, sudut_akhir, maju, turun=False, jumlah=40):
    """Titik busur (dunia) di bidang (maju, atas): sudut [rad] diukur dari garis vertikal KE ATAS (turun = True:
    dari garis vertikal KE BAWAH, seperti tali bandul) ke arah `maju`."""
    atas = np.array([0.0, 0.0, -1.0 if turun else 1.0])
    return np.array([np.asarray(pusat) + jari * (math.sin(a) * maju + math.cos(a) * atas)
                     for a in np.linspace(sudut_awal, sudut_akhir, jumlah)])


class FotoPengujian(uji_otomatis.UjiOtomatis):
    """FOTO PENGUJIAN dari simulasi PyBullet (tombol 6 / 7), dengan mode kontrol yang SEDANG dipilih (1 / 2):
        6  Pengujian 1: bidang datar, bandul chi_DEG (jendela Kontrol) dilepas dan memukul badan dari belakang
           (gaya = uji_otomatis.gaya_bandul, sama dengan tombol U). |psi| dicari selama FOTO_JENDELA_PUNCAK_S detik
           setelah pukulan, lalu FREEZE di pose |psi| MAKSIMUM (robot dikembalikan ke pose itu).
        7  Pengujian 2: lintasan datar (kuning) -> tanjakan UJI2_ZETA_DEG (merah muda) -> datar atas (kuning), naik
           dengan Kvel (arah maju), FREEZE saat poros roda tepat di TENGAH tanjakan (panjang miring / 2).
    Saat FREEZE: PNG BERSIH (getCameraImage) dan ANOTASI disimpan di PLOT_EVALUASI/FOTO/<kontrol>/, garis dan teks
    anotasi tampil di jendela PyBullet, simulasi berhenti sampai tombol ditekan:
        6 / 7 = foto lagi, 1 / 2 = ganti mode kontrol, R / U = kembali ke simulasi biasa, Q = keluar.
    Loop kontrol (siklus, letakkan, KOMET, gain, constraint h1 .. h4 C_MODE DIAM) memakai UjiOtomatis tanpa diubah;
    data tidak direkam ke CSV (hanya gambar). Selama berjalan: U = batal, Q = batal lalu keluar."""

    def __init__(self, sim, pd, cbf, params, mode, jenis):
        super().__init__(sim, pd, cbf, params)
        if mode == ise.MODE_PD:
            kontrol = {"label": "PD", "mode": ise.MODE_PD, "psi_max_deg": None, "folder": "PD", "kode": "PD"}
        else:
            derajat = float(params["psi_max_DEG"])
            kode = uji_otomatis.angka_kode(derajat)
            kontrol = {"label": "PD+CBF-QP %g deg" % derajat, "mode": ise.MODE_PD_CBF, "psi_max_deg": derajat,
                       "folder": "PDCBFQP" + kode, "kode": "PD+CBFQP" + kode}
        self.daftar_kontrol = [kontrol]
        self.kontrol = kontrol
        self.jenis = jenis
        self.chi_deg = float(params.get("chi_DEG", simulasi.FOTO_CHI_AWAL_DEG))
        self.percobaan_ke = 1
        self.judul_foto = "FOTO 6 (dipukul)" if jenis == FOTO_PUKUL else "FOTO 7 (bidang miring)"

    # ------------------------------------------------------------------ urutan
    def jalankan(self):
        """Mengembalikan (SELESAI / BATAL / KELUAR, tombol yang ditekan saat FREEZE untuk diproses atera_main)."""
        sim = self.sim
        zeta_simpan = (config.zeta_DEG, config.zeta)
        lantai_simpan = sim.zeta
        hasil = uji_otomatis.SELESAI
        lanjut = []
        print("")
        print("[%s] kontrol %s | %s" % (self.judul_foto, self.kontrol["label"],
                                         ("bandul chi %g deg -> F %.2f N" % (self.chi_deg,
                                                                             uji_otomatis.gaya_bandul(self.chi_deg)))
                                         if self.jenis == FOTO_PUKUL else
                                         ("tanjakan %g deg, %g m, Kvel %g" % (config.UJI2_ZETA_DEG,
                                                                              config.UJI2_PANJANG_MIRING_M,
                                                                              self.params["Kvel"]))))
        try:
            if abs(sim.zeta) > 1e-12:
                print("Lantai dibuat datar untuk FOTO (zeta lantai %.1f deg -> 0)." % math.degrees(sim.zeta))
                sim.ubah_zeta(0.0)
            sim.hapus_lintasan()
            sim.foto_bersihkan()
            foto = self.foto_pukul() if self.jenis == FOTO_PUKUL else self.foto_miring()
            self.simpan(foto)
            lanjut = self.tunggu_freeze(foto)
            if "q" in lanjut:
                hasil = uji_otomatis.KELUAR
        except uji_otomatis.Batal as alasan:
            hasil = uji_otomatis.KELUAR if str(alasan) == "q" else uji_otomatis.BATAL
            print("%s dibatalkan (%s)." % (self.judul_foto, "Q: keluar" if hasil == uji_otomatis.KELUAR else "U"))
        finally:
            config.zeta_DEG, config.zeta = zeta_simpan
            try:
                sim.foto_bersihkan()
                sim.hapus_lintasan()
                if abs(sim.zeta - lantai_simpan) > 1e-12:
                    sim.ubah_zeta(lantai_simpan)
                sim.kamera_awal()
            except Exception:
                pass                     # jendela PyBullet sudah ditutup
        return hasil, [t for t in lanjut if t != "q"]

    # ------------------------------------------------------- tombol 6: dipukul
    def foto_pukul(self):
        sim = self.sim
        dt = config.DT
        self.uji = uji_otomatis.UJI1
        self.sudut = self.chi_deg
        self.gaya = uji_otomatis.gaya_bandul(self.chi_deg)
        self.zeta_cbf_deg = 0.0
        self.letakkan()
        self.fase = "robot diletakkan, tenang %g s" % config.UJI_TENANG_S
        self.jalan_selama(config.UJI_TENANG_S)

        # Bandul: bola menyentuh bidang dorong sisi BELAKANG badan saat tali tegak (phi = 0).
        pusat_pukul, sumbu_roda, _, maju_badan = sim.bidang_dorong(-1)
        maju = np.array([maju_badan[0], maju_badan[1], 0.0])
        maju = maju / np.linalg.norm(maju)
        atas = np.array([0.0, 0.0, 1.0])
        chi = math.radians(self.chi_deg)
        panjang = config.BANDUL_PANJANG_TALI_M
        jari = simulasi.FOTO_BOLA_JARI_M
        bola_kena = pusat_pukul - (jari + 0.002) * maju
        gantungan = bola_kena + panjang * atas
        sim.foto_buat_bandul(gantungan, maju, sumbu_roda, chi)
        lepas = gantungan + panjang * (-math.sin(chi) * maju - math.cos(chi) * atas)
        poros = sim.posisi_poros()
        jarak_bidang = config.L_IMU * math.sin(config.psi_max) + config.JARAK_IMU_KE_SISI_LUAR * math.cos(config.psi_max)
        titik = [gantungan + 0.03 * atas, lepas - (jari + 0.03) * maju, poros - config.R * atas,
                 poros + (jarak_bidang + 0.12) * maju + 0.45 * atas, poros + (sim.tinggi_atas + 0.06) * atas,
                 poros + 0.30 * maju - (config.R + 0.17) * atas, poros - 0.45 * maju - (config.R + 0.17) * atas]
        ukuran = simulasi.FOTO_UKURAN_PUKUL
        view, proj, mata = sim.foto_kamera(titik, ukuran, simulasi.FOTO_KOTAK_PUKUL)
        geser = np.array([math.copysign(simulasi.FOTO_SISI_ANOTASI_M, mata[0] - poros[0]), 0.0, 0.0])

        # Ayunan bandul (kinematika bandul sederhana, hanya gambar): dari chi sampai tegak, lalu memukul.
        self.fase = "bandul dilepas dari chi %g deg" % self.chi_deg
        phi, omega, h = chi, 0.0, dt / 10.0
        riwayat = []                                     # (siklus ke, psi PyBullet [deg])
        while phi > 0.0:
            for _ in range(10):
                omega = omega - config.g / panjang * math.sin(phi) * h
                phi = phi + omega * h
                if phi <= 0.0:
                    break
            sim.foto_atur_bandul(max(phi, 0.0))
            self.siklus(0)
            riwayat.append((self.siklus_ke, math.degrees(sim.baca_psi_benar()[0])))
        sim.foto_atur_bandul(0.0)                        # ASUMSI (tabel Excel): bandul berhenti setelah memukul
        pose_pukul = sim.ambil_pose()
        siklus_pukul = self.siklus_ke
        sim.gaya_dorong = self.gaya
        sim.dorong(1)                                    # dari belakang ke depan (seperti tombol O / tombol U)
        print("  dipukul: chi %g deg -> F %.2f N selama %.3f s (bola %.2f m/s)"
              % (self.chi_deg, self.gaya, config.LAMA_DORONG, abs(omega) * panjang))

        # Cari |psi| maksimum (psi SEBENARNYA PyBullet = kemiringan yang terlihat di gambar)
        puncak, pose_puncak, t_puncak, komet = -1.0, None, 0.0, ""
        t = 0.0
        while t < simulasi.FOTO_JENDELA_PUNCAK_S - 0.5 * dt:
            self.fase = "dipukul, mencari |psi| maksimum %.1f / %g s (puncak sementara %.2f deg)" \
                % (t, simulasi.FOTO_JENDELA_PUNCAK_S, math.degrees(max(puncak, 0.0)))
            try:
                self.siklus(0)
            except uji_otomatis.Komet as sebab:
                komet = str(sebab)
            t = t + dt
            psi_benar = sim.baca_psi_benar()[0]
            riwayat.append((self.siklus_ke, math.degrees(psi_benar)))
            if abs(psi_benar) > puncak:
                puncak, pose_puncak, t_puncak = abs(psi_benar), sim.ambil_pose(), t
                psi_puncak = psi_benar
            if komet:
                break

        # FREEZE di pose |psi| maksimum + robot bayangan saat dipukul
        sim.pasang_pose(pose_puncak)
        sim.foto_bayangan(pose_pukul["posisi"], pose_pukul["orientasi"], pose_pukul["roda"])
        poros = sim.posisi_poros()
        psi_imu = float(pose_puncak["x"][2])
        print("  FREEZE: |psi| maksimum %.2f deg (IMU + Kalman %.2f deg) di t = %.3f s setelah dipukul%s"
              % (math.degrees(puncak), math.degrees(psi_imu), t_puncak, ("  [KOMET: %s]" % komet) if komet else ""))

        # Anotasi
        arah_badan = math.sin(psi_puncak) * maju + math.cos(psi_puncak) * atas
        poros_a = poros + geser
        poros_pukul = sim_poros_dari_pose(sim, pose_pukul) + geser
        anotasi = {
            "garis": [
                {"titik": [poros_a, poros_a + 0.40 * atas], "warna": HITAM, "tebal": 1.8, "putus": True},
                {"titik": [poros_a, poros_a + 0.38 * arah_badan], "warna": BIRU, "tebal": 3.0},
                {"titik": busur(poros_a, 0.24, 0.0, psi_puncak, maju), "warna": BIRU, "tebal": 3.0},
                # busur chi: di bidang ayunan bandul (tidak digeser), dari tali tegak ke tali saat dilepas
                {"titik": busur(gantungan, 0.15, 0.0, -chi, maju, turun=True), "warna": JINGGA, "tebal": 3.0},
            ],
            "teks": [
                {"posisi": poros_a + 0.42 * atas, "teks": "ψ maks = %.1f°" % math.degrees(puncak), "warna": BIRU,
                 "ukuran": 20, "ha": "center", "va": "bottom"},
                {"posisi": busur(gantungan, 0.19, -0.5 * chi, -0.5 * chi, maju, turun=True, jumlah=1)[0],
                 "teks": "χ", "warna": JINGGA, "ukuran": 22, "ha": "center"},
                {"posisi": poros_pukul - 0.09 * atas - 0.04 * maju, "teks": "t = 0 s (dipukul)", "warna": (0.3, 0.3, 0.3),
                 "ukuran": 14, "ha": "center"},
                {"posisi": poros_a - 0.15 * atas, "teks": "t = %.3f s (ψ maks)" % t_puncak, "warna": BIRU,
                 "ukuran": 14, "ha": "center"},
            ],
            "info": [
                ("PENGUJIAN 1 - DIPUKUL BANDUL | simulasi PyBullet | bidang datar ζ = 0°", HITAM, True),
                ("Kontrol %s | χ = %g° -> F = %.2f N selama %.3f s (m_bd %g kg, l_bd %g m)"
                 % (self.kontrol["label"], self.chi_deg, self.gaya, config.LAMA_DORONG, config.BANDUL_MASSA_KG,
                    config.BANDUL_PANJANG_TALI_M), HITAM, False),
                ("FREEZE |ψ| maksimum: t = %.3f s setelah dipukul | ψ PyBullet %.2f° | ψ IMU+Kalman %.2f°"
                 % (t_puncak, math.degrees(puncak), math.degrees(abs(psi_imu))), BIRU, True),
                ("Robot transparan = saat dipukul (t = 0 s), robot solid = saat |ψ| maksimum", (0.3, 0.3, 0.3), False),
            ],
            "grafik": {"t": [(k - siklus_pukul) * config.DT for k, _ in riwayat],
                       "y": [abs(v) for _, v in riwayat], "t_tanda": t_puncak, "y_tanda": math.degrees(puncak),
                       "batas": self.kontrol["psi_max_deg"], "t_pukul": 0.0,
                       "judul": "|ψ| PyBullet (°), lingkaran = FREEZE", "label_y": "|ψ| (°)"},
        }
        if komet:
            anotasi["info"].append(("KOMET (robot jatuh): %s -> freeze di |ψ| terbesar sebelum KOMET" % komet,
                                    MERAH, True))
        return {"view": view, "proj": proj, "ukuran": ukuran, "anotasi": anotasi,
                "kode_uji": uji_otomatis.angka_kode(self.chi_deg),
                "ringkas": "|psi| maks %.2f deg di t = %.3f s" % (math.degrees(puncak), t_puncak)}

    # ------------------------------------------------- tombol 7: bidang miring
    def foto_miring(self):
        sim = self.sim
        dt = config.DT
        self.uji = uji_otomatis.UJI2
        self.sudut = None
        self.gaya = 0.0
        self.zeta_cbf_deg = float(config.UJI2_CBF_ZETA_DEG)
        lin = sim.buat_lintasan(config.UJI2_DATAR_BAWAH_M, config.UJI2_ZETA_DEG, config.UJI2_PANJANG_MIRING_M,
                                config.UJI2_DATAR_ATAS_M, config.UJI2_LEBAR_M, config.UJI2_TEBAL_PAPAN_M,
                                simulasi.FOTO_WARNA_TANJAKAN, simulasi.FOTO_WARNA_DATAR)
        sim.foto_jalur_bawah(lin)
        self.letakkan()
        self.fase = "robot diletakkan, tenang %g s" % config.UJI_TENANG_S
        self.jalan_selama(config.UJI_TENANG_S)
        pose_awal = sim.ambil_pose()

        atas = np.array([0.0, 0.0, 1.0])
        maju = np.array([0.0, 1.0, 0.0])
        zeta = lin["zeta"]
        y_tengah = lin["y_mulai"] + 0.5 * (lin["y_puncak"] - lin["y_mulai"])
        y_awal = -simulasi.FOTO_JALUR_BAWAH_BELAKANG_M
        titik = [[0.0, y_awal, -0.24], [0.0, lin["y_akhir"], -0.24], [0.0, lin["y_akhir"], lin["tinggi"] + 0.42],
                 [0.0, y_awal, 0.42]]
        ukuran = simulasi.FOTO_UKURAN_MIRING
        view, proj, mata = sim.foto_kamera(titik, ukuran, simulasi.FOTO_KOTAK_MIRING)
        sisi = math.copysign(1.0, mata[0])
        geser_robot = np.array([sisi * simulasi.FOTO_SISI_ANOTASI_M, 0.0, 0.0])
        geser_jalur = np.array([sisi * (0.5 * lin["lebar"] + 0.01), 0.0, 0.0])

        # NAIK dengan Kvel (arah maju) sampai poros roda di tengah tanjakan
        print("  NAIK dengan Kvel %g (arah maju), freeze di y = %.3f m (tengah tanjakan)" % (self.params["Kvel"], y_tengah))
        t, gagal = 0.0, ""
        y_terjauh, t_terjauh = self.y_poros(), 0.0
        while self.y_poros() < y_tengah:
            self.fase = "NAIK: poros %.2f m dari kaki tanjakan, freeze di %.3f m" \
                % (self.y_poros() - lin["y_mulai"], y_tengah - lin["y_mulai"])
            try:
                self.siklus(1)
            except uji_otomatis.Komet as sebab:
                gagal = "KOMET (%s)" % sebab
                break
            t = t + dt
            if self.y_poros() > y_terjauh + config.UJI2_NAIK_MACET_M:
                y_terjauh, t_terjauh = self.y_poros(), t
            elif t - t_terjauh >= config.UJI2_NAIK_MACET_S:
                gagal = "tidak kuat naik (tidak maju %g m dalam %g s)" % (config.UJI2_NAIK_MACET_M,
                                                                          config.UJI2_NAIK_MACET_S)
                break

        # FREEZE di sini + robot bayangan di posisi awal (dan tujuan, bila FOTO_BAYANGAN_TUJUAN)
        pose = sim.ambil_pose()
        sim.pasang_pose(pose)
        sim.foto_bayangan(pose_awal["posisi"], pose_awal["orientasi"], pose_awal["roda"])
        poros = sim.posisi_poros()
        if simulasi.FOTO_BAYANGAN_TUJUAN:
            posisi, orientasi = sim.pose_tegak([poros[0], lin["y_puncak"] + config.UJI2_LEPAS_ATAS_M,
                                                lin["tinggi"] + config.R + 0.001])
            sim.foto_bayangan(posisi, orientasi)
        psi_benar = sim.baca_psi_benar()[0]
        psi_imu = float(pose["x"][2])
        dari_kaki = (poros[1] - lin["y_mulai"]) / math.cos(zeta)
        print("  FREEZE: poros %.3f m dari kaki tanjakan (di permukaan), psi %.2f deg (IMU + Kalman %.2f deg), "
              "t = %.2f s sejak naik%s" % (dari_kaki, math.degrees(psi_benar), math.degrees(psi_imu), t,
                                           ("  [GAGAL: %s]" % gagal) if gagal else ""))

        # Anotasi
        kaki = np.array([0.0, lin["y_mulai"], 0.0]) + geser_jalur
        poros_a = poros + geser_robot
        arah_badan = math.sin(psi_benar) * maju + math.cos(psi_benar) * atas
        anotasi = {
            "garis": [
                # Pengujian 2: hanya penanda biru (garis badan + busur psi), garis merah / hitam dihapus
                {"titik": [poros_a, poros_a + 0.38 * arah_badan], "warna": BIRU, "tebal": 3.0},
                {"titik": busur(poros_a, 0.22, 0.0, psi_benar, maju), "warna": BIRU, "tebal": 3.0},
            ],
            "teks": [
                {"posisi": kaki + 0.95 * (math.cos(0.5 * zeta) * maju + math.sin(0.5 * zeta) * atas), "teks": "ζ",
                 "warna": HITAM, "ukuran": 22, "ha": "left"},
                {"posisi": poros_a + 0.42 * atas, "teks": "ψ = %.1f°" % math.degrees(psi_benar), "warna": BIRU,
                 "ukuran": 20, "ha": "center", "va": "bottom"},
            ],
            "info": [
                ("PENGUJIAN 2 - BIDANG MIRING | simulasi PyBullet | ζ = %g°, panjang %g m"
                 % (config.UJI2_ZETA_DEG, config.UJI2_PANJANG_MIRING_M), HITAM, True),
                ("Kontrol %s | naik dengan Kvel %g (arah maju) | zeta model CBF-QP %g°"
                 % (self.kontrol["label"], self.params["Kvel"], self.zeta_cbf_deg), HITAM, False),
                ("FREEZE di tengah tanjakan: %.3f m dari kaki tanjakan, t = %.2f s sejak naik"
                 % (dari_kaki, t), BIRU, True),
                ("ψ PyBullet %.2f° | ψ IMU+Kalman %.2f° (terhadap vertikal) | robot transparan = posisi awal%s"
                 % (math.degrees(psi_benar), math.degrees(psi_imu),
                    " dan tujuan" if simulasi.FOTO_BAYANGAN_TUJUAN else ""), (0.3, 0.3, 0.3), False),
            ],
        }
        if gagal:
            anotasi["info"].append(("GAGAL sebelum tengah tanjakan: %s -> freeze di sini" % gagal, MERAH, True))
        return {"view": view, "proj": proj, "ukuran": ukuran, "anotasi": anotasi,
                "kode_uji": uji_otomatis.angka_kode(config.UJI2_ZETA_DEG),
                "ringkas": "%.3f m dari kaki tanjakan, psi %.2f deg" % (dari_kaki, math.degrees(psi_benar))}

    # ------------------------------------------------------- simpan + FREEZE
    def simpan(self, foto):
        folder = os.path.join(simulasi.FOTO_FOLDER, self.kontrol["folder"])
        nama = nama_foto(folder, self.kontrol["kode"], foto["kode_uji"])
        path = os.path.join(folder, nama)
        try:
            rgb, renderer = self.sim.foto_ambil(foto["view"], foto["proj"], foto["ukuran"])
            simulasi.foto_simpan_bersih(rgb, path + "_BERSIH.png")
            simulasi.foto_simpan_anotasi(rgb, foto["view"], foto["proj"], foto["anotasi"], path + "_ANOTASI.png")
            print("  FOTO tersimpan (%dx%d, %s): %s_BERSIH.png dan _ANOTASI.png"
                  % (rgb.shape[1], rgb.shape[0], renderer, path))
            foto["tersimpan"] = nama
        except Exception as kesalahan:
            print("  FOTO GAGAL disimpan -> %s: %s" % (type(kesalahan).__name__, kesalahan))
            foto["tersimpan"] = ""

    def tunggu_freeze(self, foto):
        """FREEZE: simulasi berhenti (tidak ada step), anotasi tampil di jendela PyBullet (mouse tetap bisa memutar
        kamera). Mengembalikan tombol yang ditekan: [] (R / U), ["q"], atau [6 / 7 / 1 / 2] untuk atera_main."""
        sim = self.sim
        sim.foto_tampilkan_anotasi(foto["anotasi"])
        petunjuk = "6 / 7 foto lagi | 1 / 2 ganti mode | R / U kembali | Q keluar"
        print("  FREEZE (%s). Tombol: %s" % (foto["ringkas"], petunjuk))
        teks_kontrol = "FREEZE %s | %s | %s" % (self.judul_foto, self.kontrol["label"], petunjuk)
        baris = ["FREEZE %s | %s | %s" % (self.judul_foto, self.kontrol["label"], foto["ringkas"]),
                 "PNG: %s" % (foto["tersimpan"] or "GAGAL (lihat terminal)"), petunjuk]
        while sim.masih_terbuka():
            tombol = sim.baca_keyboard()
            sim.baca_params()
            for kunci in ("q", FOTO_PUKUL, FOTO_MIRING, "mode_pd", "mode_cbf"):
                if kunci in tombol:
                    return [kunci]
            if "r" in tombol or "u" in tombol:
                return []
            sim.tampilkan(baris, MODE_DIAM, False, teks_kontrol, False, None, False)
            time.sleep(0.05)
        return ["q"]

    def tampil(self):
        """Tampilan selama FOTO berjalan (dipanggil UjiOtomatis.siklus tiap 10 siklus)."""
        x = self.x
        baris = ["%s | %s | mode DIAM | CBF: %s" % (self.judul_foto, self.kontrol["label"], self.status_cbf),
                 "psi %+.1f / %.1f deg    dpsi %+.2f rad/s" % (math.degrees(x[2]), config.psi_max_DEG, x[3]),
                 "theta %+.2f rad    dtheta %+.2f rad/s    u %+.3f N m" % (x[0], x[1], self.u),
                 "STATUS: " + self.fase, "U = batal, Q = keluar"]
        self.sim.tampilkan(baris, MODE_DIAM, False, "%s | %s | U = batal" % (self.judul_foto, self.kontrol["label"]),
                           False, None, True)


def sim_poros_dari_pose(sim, pose):
    """Titik tengah poros roda (dunia) untuk pose tersimpan (frame pusat massa) [m]."""
    posisi_link, orientasi_link = pybullet.multiplyTransforms(pose["posisi"], pose["orientasi"],
                                                              sim.inersia_balik[0], sim.inersia_balik[1])
    rot = np.array(pybullet.getMatrixFromQuaternion(orientasi_link)).reshape(3, 3)
    return np.array(posisi_link) + rot @ sim.poros_badan


def main():
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--plot", action="store_true")              # bawaan sekarang: plot selalu dibuka
    parser.add_argument("--tanpa-plot", action="store_true")
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

    # CBF-QP (solver config.CBF_QP_SOLVER) selalu disiapkan sebelum jendela simulasi dibuka supaya tombol PD + CBF-QP
    # bisa langsung dipakai. Bila gagal, simulasi tetap jalan dengan mode PD saja (tombol PD + CBF-QP dimatikan).
    # config.CBF_TAHU_ZETA = False (= robot fisik): model CBF-QP memakai zeta = 0 walau lantai dimiringkan, gangguan
    # kemiringan ditaksir DOB. True: CBF-QP membaca config.zeta tiap siklus (ikut berubah bila zeta_DEG diubah).
    cbf = None
    print("Menyiapkan CBF-QP (solver %s) ..." % config.CBF_QP_SOLVER)
    try:
        from cbf_qp import CBFQP
        cbf = CBFQP(zeta=None if config.CBF_TAHU_ZETA else 0.0)
    except Exception as kesalahan:
        print("CBF-QP TIDAK BISA DIPAKAI -> %s: %s" % (type(kesalahan).__name__, kesalahan))
        print("Simulasi tetap jalan, hanya mode PD.")

    sim = Simulasi(gui=True, plot=not pilihan.tanpa_plot, judul=mode)
    sim.cbf_ada = cbf is not None
    pembanding = {ise.MODE_PD: "", ise.MODE_PD_CBF: ""}       # nama file PERBANDINGAN pembanding tiap mode
    sim.teks_pembanding = teks_pembanding(pembanding)

    # Constraint CBF-QP yang dipakai = kotak centang h1 .. h4 di jendela Kontrol (awal: C_MODE[MODE_DIAM]) yang juga
    # ada di C_MODE mode gerak sekarang.
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

    # PENGUJIAN (tombol T mulai, T lagi selesai)
    pengujian_berjalan = False     # True = sedang merekam satu pengujian (bersamaan dengan perekaman biasa)
    t_uji = 0.0                    # waktu sejak T ditekan [s]

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
    tombol_tertunda = []           # tombol yang ditekan saat FREEZE FOTO, diproses di siklus berikutnya

    try:
        while sim.masih_terbuka():
            # ----------------------------------------------------------- keyboard
            tombol_baru = sim.baca_keyboard() + tombol_tertunda
            tombol_tertunda = []
            # Parameter dibaca sesudah keyboard dan sebelum tombol diproses, supaya gaya dorong yang
            # dicetak saat O / P ditekan adalah nilai terbaru dari jendela Kontrol.
            params = sim.baca_params()

            if "q" in tombol_baru:
                print("Tombol Q: keluar dan simpan.")
                break

            # ------------------------------- PENGUJIAN OTOMATIS (tombol U) / FOTO PENGUJIAN (tombol 6 / 7)
            jenis_foto = None
            for kunci in (FOTO_PUKUL, FOTO_MIRING):
                if kunci in tombol_baru:
                    jenis_foto = kunci
            if "u" in tombol_baru or jenis_foto is not None:
                nama_uji = "PENGUJIAN OTOMATIS" if jenis_foto is None else "FOTO PENGUJIAN"
                if jenis_foto is None:
                    alasan = uji_otomatis.cek_siap(cbf)
                else:
                    alasan = "" if cbf is not None else "CBF-QP gagal disiapkan (lihat pesan saat program mulai)."
                if alasan != "":
                    print("%s tidak bisa dimulai: %s" % (nama_uji, alasan))
                else:
                    # Yang sedang berjalan disimpan / dibatalkan dulu (sama seperti mengganti mode kontrol).
                    if pengujian_berjalan:
                        print("PENGUJIAN (T) disimpan dulu sebelum %s." % nama_uji)
                        simpan_uji(perekam_uji)
                        pengujian_berjalan = False
                    if banding_berjalan:
                        print("Rekaman perbandingan yang sedang berjalan dibatalkan (tidak disimpan).")
                        banding_berjalan = False
                    if mode_banding:
                        print("PEREKAM PERBANDINGAN dimatikan.")
                        mode_banding = False
                    else:
                        simpan_ise(perekam)
                    sim.plot_tahan = False
                    if jenis_foto is None:
                        hasil_uji = uji_otomatis.UjiOtomatis(sim, pd, cbf, params).jalankan()
                    else:
                        hasil_uji, tombol_tertunda = FotoPengujian(sim, pd, cbf, params, mode, jenis_foto).jalankan()
                    # Kembali ke keadaan biasa: mode kontrol sebelum U, robot diletakkan ulang, constraint dari
                    # kotak centang, psi_max / Alpha / gain dari jendela Kontrol (siklus berikutnya).
                    sim.atur_mode(mode)
                    letakkan_ulang(sim, perekam, mode, pd, cbf)
                    komet = False
                    t = 0.0
                    centang_dipakai = None
                    mask_dipakai = None
                    waktu_berikutnya = time.time()
                    if hasil_uji == uji_otomatis.KELUAR:
                        print("Tombol Q saat %s: keluar." % nama_uji)
                        break
                    print("Kembali ke mode biasa: %s, robot diletakkan ulang." % mode)
                    continue

            if "banding" in tombol_baru:
                if not mode_banding:
                    if pengujian_berjalan:
                        print("PENGUJIAN (T) disimpan dulu sebelum perekam perbandingan dinyalakan.")
                        simpan_uji(perekam_uji)
                        pengujian_berjalan = False
                    mode_banding = True
                    banding_berjalan = False
                    perekam.mulai(mode, ise.JENIS_PERCOBAAN)     # perekaman biasa berhenti (data dibuang)
                    sim.plot_tahan = True
                    sim.reset_plot()
                    print("PEREKAM PERBANDINGAN nyala (mode %s): tekan R (robot diletakkan ulang), lalu O / P untuk "
                          "mulai merekam %.0f s. Tekan 3 lagi untuk mematikan." % (mode, config.DURASI_PERBANDINGAN))
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
                    print("PEREKAM PERBANDINGAN (3) masih menyala: matikan dulu dengan 3 sebelum PENGUJIAN (T).")
                elif pengujian_berjalan:
                    print("PENGUJIAN selesai (%.1f s)." % t_uji)
                    simpan_uji(perekam_uji)
                    pengujian_berjalan = False
                elif komet:
                    print("komet: tekan R dulu sebelum memulai pengujian.")
                else:
                    perekam_uji.mulai(mode, ise.JENIS_PENGUJIAN)
                    t_uji = 0.0
                    pengujian_berjalan = True
                    print("PENGUJIAN dimulai: dorong robot dengan O / P. Tekan T lagi untuk selesai dan simpan.")

            if "e" in tombol_baru:
                if mode_banding:
                    if banding_berjalan:
                        print("Tombol E: rekaman perbandingan disimpan sebelum %.0f s." % config.DURASI_PERBANDINGAN)
                        simpan_pembanding(sim, perekam_banding, pembanding)
                        banding_berjalan = False
                        sim.plot_tahan = True
                        sim.reset_plot()
                    else:
                        print("Tombol E: tidak ada rekaman perbandingan yang sedang berjalan.")
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
                        print("Ganti mode kontrol: PENGUJIAN %s disimpan dulu." % mode)
                        simpan_uji(perekam_uji)
                        pengujian_berjalan = False
                    if banding_berjalan:
                        print("Rekaman perbandingan yang sedang berjalan dibatalkan (tidak disimpan).")
                        banding_berjalan = False
                        sim.plot_tahan = True
                    if not mode_banding:
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
                    if pd.mode_gerak == MODE_BALANCING:
                        pd.atur_mode(MODE_DIAM)
                        print("Mode DIAM: Kp_theta mati.")
                    else:
                        pd.atur_mode(MODE_BALANCING)
                        sim.nolkan_theta()                   # setpoint roda = posisi dan arah sekarang
                        pd.pasang_yaw_ref(sim.baca_yaw()[0])
                        print("Mode BALANCING: Kp_theta aktif, setpoint roda di sini (s = 0, arah sekarang).")
                if tombol == "f":
                    if pd.mode_gerak == MODE_BEBAS:
                        pd.atur_mode(MODE_DIAM)
                        pd.pasang_yaw_ref(sim.baca_yaw()[0])
                        print("Mode BEBAS mati: mode DIAM, arah sekarang = setpoint heading.")
                    else:
                        pd.atur_mode(MODE_BEBAS)
                        print("Mode BEBAS: PD heading dan Kp_theta mati. Tekan F lagi untuk DIAM.")
                if tombol == "o" or tombol == "p":
                    arah_dorong = 1 if tombol == "o" else -1
                    teks_arah = "depan" if arah_dorong > 0 else "belakang"
                    if mode_banding:
                        if komet:
                            print("komet: tekan R dulu sebelum memulai rekaman perbandingan.")
                        elif banding_berjalan:
                            print("Rekaman perbandingan masih berjalan (%.1f / %.0f s), dorongan diabaikan."
                                  % (t_banding, config.DURASI_PERBANDINGAN))
                        else:
                            sim.dorong(arah_dorong)
                            perekam_banding.mulai(mode, ise.JENIS_PERBANDINGAN)
                            t_banding = 0.0
                            banding_berjalan = True
                            sim.plot_tahan = False
                            sim.reset_plot()                  # plot realtime mulai dari dorongan ini
                            print("PERBANDINGAN %s dimulai: didorong ke %s %.1f N, direkam %.0f s."
                                  % (mode, teks_arah, sim.gaya_dorong, config.DURASI_PERBANDINGAN))
                    else:
                        sim.dorong(arah_dorong)
                        print("Didorong ke %s %.1f N%s" % (teks_arah, sim.gaya_dorong,
                                                            " (PENGUJIAN t %.1f s)" % t_uji if pengujian_berjalan else ""))

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

            # Perintah gerak. W / S baru DILEPAS di mode BALANCING: setpoint theta dipasang tepat di posisi ini
            # (sebelum state dibaca, jadi siklus ini sudah memakai theta = 0 di sini).
            arah_sebelumnya = pd.arah
            pd.set_command(sim.arah, sim.belok)
            if pd.mode_gerak == MODE_BALANCING and arah_sebelumnya != 0 and pd.arah == 0:
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

            # KOMET: |psi| > SAFE_TILT_DEG, |s| > S_KOMET (hanya BALANCING tanpa W / S), atau berputar (bukan BEBAS)
            sebab = ""
            if abs(psi) > math.radians(config.SAFE_TILT_DEG):
                sebab = "psi = %.1f deg" % math.degrees(psi)
            elif pd.tahan_posisi and abs(s) > config.S_KOMET:
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
                    simpan_uji(perekam_uji)
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
                        if t_banding >= config.DURASI_PERBANDINGAN - 0.5 * config.DT:
                            print("PERBANDINGAN %s selesai (%.0f s)." % (mode, config.DURASI_PERBANDINGAN))
                            simpan_pembanding(sim, perekam_banding, pembanding)
                            banding_berjalan = False
                            sim.plot_tahan = True
                            sim.reset_plot()              # kurva sekarang dihapus, tinggal pembanding (pudar)
                            lain = ise.MODE_PD_CBF if mode == ise.MODE_PD else ise.MODE_PD
                            if not pembanding[lain]:
                                print("Berikutnya: tekan %s (mode %s), R, lalu O / P dengan dorongan yang sama."
                                      % ("2" if lain == ise.MODE_PD_CBF else "1", lain))
                else:
                    ekstra = data_ekstra(pd, cbf, mode)
                    t = t + config.DT
                    perekam.tambah(t, x, pd.error_theta, pd.error_dtheta, pd.error_psi, pd.error_dpsi,
                                   u_PD, u, status_cbf, params, dorong, nilai_c, ekstra)
                    if pengujian_berjalan:                  # PENGUJIAN (T) direkam bersamaan dengan perekaman biasa
                        t_uji = t_uji + config.DT
                        perekam_uji.tambah(t_uji, x, pd.error_theta, pd.error_dtheta, pd.error_psi, pd.error_dpsi,
                                           u_PD, u, status_cbf, params, dorong, nilai_c, ekstra)

            # ----------------------------------------------------------- tampilan
            gerak = "mode " + pd.mode_gerak
            if mode_banding:
                perekam_aktif = perekam_banding
                t_tampil = t_banding
                if banding_berjalan:
                    teks_uji = "PEREKAM PERBANDINGAN | %s | merekam t %.1f / %.0f s" \
                        % (mode, t_banding, config.DURASI_PERBANDINGAN)
                else:
                    teks_uji = "PEREKAM PERBANDINGAN | %s | tekan R lalu O / P (rekam %.0f s), 3 untuk mematikan" \
                        % (mode, config.DURASI_PERBANDINGAN)
            elif pengujian_berjalan:
                perekam_aktif = perekam_uji
                t_tampil = t_uji
                teks_uji = "PENGUJIAN | %s | merekam t %.1f s | O / P dorong, T selesai dan simpan" % (mode, t_uji)
            else:
                perekam_aktif = perekam
                t_tampil = t
                teks_uji = ""

            if siklus % 10 == 0:
                baris = []
                if komet:
                    baris.append("komet, motor mati  ->  R: ulang   E: simpan data   Q: keluar + simpan")
                elif pengujian_berjalan or mode_banding:
                    baris.append(teks_uji)
                elif mode == ise.MODE_PD:
                    baris.append("%s | %s | zeta %.1f deg" % (mode, gerak, zeta_dipakai))
                else:
                    baris.append("%s | %s | CBF: %s | zeta %.1f deg" % (mode, gerak, status_cbf, zeta_dipakai))
                baris.append("psi %+.1f / %.1f deg    dpsi %+.2f rad/s"
                             % (math.degrees(psi), math.degrees(config.psi_max), x[3]))
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
        if pengujian_berjalan:
            print("PENGUJIAN belum diselesaikan (T): disimpan sampai saat ini.")
            simpan_uji(perekam_uji)
        if not mode_banding:
            simpan_ise(perekam)
        else:
            print("Tidak ada pengujian / rekaman perbandingan yang belum disimpan.")
        uji_otomatis.tunggu_simpan()          # file PENGUJIAN OTOMATIS yang masih ditulis di proses terpisah
        sim.cetak_params()
        sim.tutup()


if __name__ == "__main__":
    main()