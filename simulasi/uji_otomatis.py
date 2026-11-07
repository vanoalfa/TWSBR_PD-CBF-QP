"""uji_otomatis.py - PENGUJIAN OTOMATIS simulasi ATERA (tombol U di atera_main.py).

Jalankan:  python3 atera_main.py          (jendela PyBullet, jendela Kontrol, dan jendela plot terbuka)
           lalu tekan U (tombol "U uji otomatis" di jendela Kontrol, atau keyboard U). U lagi = batalkan.
           Selama pengujian berjalan semua tombol lain diabaikan, kecuali Q (batalkan lalu keluar program).

Urutan (semua: mode gerak DIAM tanpa Kp_theta, constraint CBF-QP h1 .. h4, gain = nilai jendela Kontrol saat U ditekan):
    UJI 1  bidang datar (zeta 0)       : PD -> PD+CBF-QP 5 deg -> 10 deg -> 15 deg (config.UJI_PSI_MAX_DEG),
                                         tiap kontrol dipukul bandul 30, 40, ..., 90 deg (config.UJI1_SUDUT_BANDUL_DEG)
    UJI 2  tanjakan 7.5 deg, 2.41 m    : PD -> PD+CBF-QP 5 deg -> 10 deg -> 15 deg
Satu percobaan UJI 1:
    robot diletakkan -> tenang UJI_TENANG_S (tidak direkam) -> mulai merekam -> UJI_SEBELUM_PUKUL_S -> dipukul dari
    BELAKANG ke depan (seperti tombol O) dengan gaya F selama LAMA_DORONG -> direkam sampai robot DIAM
    UJI_STABIL_LAMA_S detik (BERHASIL), KOMET (GAGAL), atau lewat UJI_STABIL_BATAS_S sejak pukulan (GAGAL)
    DIAM = |psi| < UJI_STABIL_PSI_DEG dan poros roda bergeser < UJI_DIAM_GESER_M selama waktu itu (dari POSISI, jadi
    getaran kecil roda tidak membuat robot dianggap belum diam).
Satu percobaan UJI 2 (lintasan: bidang datar bawah -> tanjakan -> bidang datar atas, dibuat simulasi.buat_lintasan):
    robot diletakkan UJI2_DATAR_BAWAH_M sebelum tanjakan -> tenang (tidak direkam) -> mulai merekam -> diam
    UJI2_DIAM_AWAL_S -> NAIK (Kvel, arah maju) -> Kvel dilepas setelah poros roda UJI2_LEPAS_ATAS_M masuk bidang atas
    -> berhenti UJI2_DIAM_ATAS_S detik (waktu tetap, SELALU lanjut) -> MUNDUR (Kvel, arah mundur) HANYA di bidang atas,
    sampai poros roda melewati tepi atas tanjakan -> Kvel dilepas, robot TURUN karena gravitasi (bila macet: ketukan
    Kvel arah mundur UJI2_KETUKAN_S, maks UJI2_KETUKAN_MAKS kali) -> BERHENTI di bidang datar bawah: DIAM
    UJI_STABIL_LAMA_S detik (BERHASIL). Tidak maju UJI2_NAIK_MACET_M dalam UJI2_NAIK_MACET_S saat naik = GAGAL
    "tidak kuat naik".
Berhasil -> disimpan, lanjut. Gagal -> diulang sampai UJI_PERCOBAAN_MAKS kali; yang disimpan hanya GAGAL terakhir.

Gaya pukulan bandul (config.py bagian 11, sama dengan tabel Excel pengujian bandul):
    F = m_bd * sqrt(2 * g * l_bd * (1 - cos(phi_0))) / LAMA_DORONG          [N]
    m_bd = BANDUL_MASSA_KG [kg], l_bd = BANDUL_PANJANG_TALI_M [m], phi_0 = sudut lepas bandul [rad],
    g = config.g [m/s^2], LAMA_DORONG = lama tumbukan [s] (ASUMSI 0.033 s, tabel Excel)

Hasil (config.py bagian 13), .csv + .png + .eps (+ panel di subfolder PANEL/):
    PLOT_EVALUASI/PENGUJIAN/<PD | PDCBFQP5 | PDCBFQP10 | PDCBFQP15>/YYYYMMDD_HHmmSS_<kontrol>_<uji>_Simulasi_PENGUJIAN KE-XX
        <kontrol> = PD, PD+CBFQP5, PD+CBFQP10, PD+CBFQP15     <uji> = sudut bandul (30 .. 90) atau 7-5 (UJI 2)
    PLOT_EVALUASI/PENGUJIAN/YYYYMMDD_HHmmSS_RINGKASAN UJI OTOMATIS_Simulasi.csv   (semua percobaan, berhasil / gagal)
Kolom CSV sama dengan ise.py (dan robot fisik). Gambar memakai format Data UJI robot fisik: fungsi grafik
program/atera_program/ise.py dimuat lewat path (tanpa salinan, sama seperti kalman.py), ditambah satu baris catatan
pengujian di judul. Penyimpanan berjalan di proses terpisah supaya simulasi (dan rekaman layar) tidak tersendat.
"""

import collections
import csv
import importlib.util
import math
import multiprocessing
import os
import re
import time
from datetime import datetime

import config
import ise
from pd_control import MODE_DIAM

UJI1 = "UJI 1"
UJI2 = "UJI 2"
BERHASIL = "BERHASIL"
GAGAL = "GAGAL"

# Hasil jalankan()
SELESAI = "selesai"
BATAL = "batal"            # U ditekan lagi
KELUAR = "keluar"          # Q ditekan: atera_main.py ikut keluar


# =============================================================================
# Format gambar Data UJI: program/atera_program/ise.py (robot fisik)
# =============================================================================
_FOLDER = os.path.dirname(os.path.abspath(__file__))
_PATH_ISE_ROBOT = [os.path.join(_FOLDER, "..", "program", "atera_program", "ise.py"),
                   os.path.join(_FOLDER, "..", "atera_program", "ise.py")]
_ISE_ROBOT = {}


def path_ise_robot():
    """Path ise.py robot fisik, "" bila tidak ada."""
    for path in _PATH_ISE_ROBOT:
        if os.path.isfile(path):
            return os.path.abspath(path)
    return ""


def muat_ise_robot():
    """Modul ise.py robot fisik (dimuat sekali). Modul itu memakai config.py simulasi (nama modul config sama), jadi
    garis batas gambar memakai nilai simulasi."""
    if "modul" not in _ISE_ROBOT:
        path = path_ise_robot()
        if path == "":
            raise FileNotFoundError("ise.py robot fisik tidak ditemukan. Dicari di:\n  " + "\n  ".join(_PATH_ISE_ROBOT))
        spec = importlib.util.spec_from_file_location("ise_robot_fisik", path)
        modul = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(modul)
        _ISE_ROBOT["modul"] = modul
    return _ISE_ROBOT["modul"]


def cek_siap(cbf):
    """"" bila pengujian otomatis bisa dimulai, atau alasan tidak bisa."""
    if cbf is None:
        return "CBF-QP gagal disiapkan (lihat pesan saat program mulai), padahal PD+CBF-QP ikut diuji."
    if path_ise_robot() == "":
        return "ise.py robot fisik (format gambar Data UJI) tidak ditemukan di: " + ", ".join(_PATH_ISE_ROBOT)
    return ""


# =============================================================================
# Kontrol, bandul, nama file
# =============================================================================
def angka_kode(nilai):
    """Angka untuk nama file / folder: 30.0 -> '30', 7.5 -> '7-5'."""
    return ("%g" % float(nilai)).replace(".", "-")


def daftar_kontrol():
    """Kontrol yang diuji, berurutan: PD, lalu PD+CBF-QP untuk tiap config.UJI_PSI_MAX_DEG."""
    hasil = [{"label": "PD", "mode": ise.MODE_PD, "psi_max_deg": None, "folder": "PD", "kode": "PD"}]
    for derajat in config.UJI_PSI_MAX_DEG:
        hasil.append({"label": "PD+CBF-QP %g deg" % derajat, "mode": ise.MODE_PD_CBF, "psi_max_deg": float(derajat),
                      "folder": "PDCBFQP" + angka_kode(derajat), "kode": "PD+CBFQP" + angka_kode(derajat)})
    return hasil


def kecepatan_bandul(sudut_deg):
    """v_1 = sqrt(2 g l_bd (1 - cos phi_0)) [m/s]: kecepatan bandul sesaat sebelum memukul."""
    return math.sqrt(2.0 * config.g * config.BANDUL_PANJANG_TALI_M * (1.0 - math.cos(math.radians(sudut_deg))))


def impuls_bandul(sudut_deg):
    """J = m_bd v_1 [N s] (ASUMSI tabel Excel: bandul berhenti setelah memukul)."""
    return config.BANDUL_MASSA_KG * kecepatan_bandul(sudut_deg)


def gaya_bandul(sudut_deg):
    """F = J / LAMA_DORONG [N]: gaya dorongan simulasi yang setara dengan pukulan bandul."""
    return impuls_bandul(sudut_deg) / config.LAMA_DORONG


def folder_kontrol(kontrol):
    """PLOT_EVALUASI/PENGUJIAN/<PD | PDCBFQP5 | ...>."""
    return os.path.join(ise.folder_jenis(ise.JENIS_PENGUJIAN), kontrol["folder"])


_NOMOR_DIPAKAI = {}                # folder -> nomor KE-XX yang sudah dibagikan sesi ini (file bisa belum selesai ditulis)
POLA_NOMOR = re.compile(r"_PENGUJIAN KE-(\d+)\.(?:csv|png|eps)$")


def nama_berikutnya(folder, kode_kontrol, kode_uji):
    """YYYYMMDD_HHmmSS_<kode_kontrol>_<kode_uji>_Simulasi_PENGUJIAN KE-XX. KE-XX = nomor terbesar yang sudah ada di folder
    itu (semua tanggal) + 1."""
    os.makedirs(folder, exist_ok=True)
    terpakai = set(_NOMOR_DIPAKAI.setdefault(folder, set()))
    for nama_file in os.listdir(folder):
        cocok = POLA_NOMOR.search(nama_file)
        if cocok:
            terpakai.add(int(cocok.group(1)))
    nomor = max(terpakai, default=0) + 1
    _NOMOR_DIPAKAI[folder].add(nomor)
    sekarang = datetime.now()
    return "%s_%s_%s_%s_Simulasi_PENGUJIAN KE-%02d" % (sekarang.strftime("%Y%m%d"), sekarang.strftime("%H%M%S"),
                                                        kode_kontrol, kode_uji, nomor)


# =============================================================================
# Menyimpan (di proses terpisah)
# =============================================================================
def gambar_data_uji(robot, data, nama, mode, catatan, path_png):
    """Gambar satu percobaan, SAMA dengan gambar_percobaan() program/atera_program/ise.py (format Data UJI: psi, dpsi,
    dtheta, s, torsi, bidang fase psi - dpsi, ISE kumulatif, tabel metrik, + file panel PANEL/), ditambah catatan
    pengujian otomatis di judul. robot = modul ise.py robot fisik (muat_ise_robot)."""
    plt = robot.plt
    m = robot.metrik(data)
    satu = [(mode, robot.WARNA_MODE[mode], data)]
    mask = data["mask"]
    tabel = robot.tabel_metrik(m)

    fig = plt.figure(figsize=(14, 19.8))
    kisi = fig.add_gridspec(5, 2, height_ratios=[1, 1, 1, 1.15, 1.25])
    teks_c = ("\nconstraint CBF-QP aktif: " + robot.teks_mask(mask)) if mask is not None else ""
    fig.suptitle("%s  |  durasi %.1f s  |  zeta %.1f deg  |  loop %.0f Hz\n%s%s\n%s"
                 % (nama, m["durasi"], data["zeta_deg"], m["loop_hz"], robot.bungkus(robot.teks_gain(mode, data["gain"])),
                    teks_c, robot.bungkus(catatan, 170)), fontsize=10)
    letak = (("psi", kisi[0, 0]), ("dpsi", kisi[0, 1]), ("dtheta", kisi[1, 0]), ("s", kisi[1, 1]), ("u", kisi[2, :]),
             ("fase_psi", kisi[3, 0]), ("ise", kisi[3, 1]), ("tabel", kisi[4, :]))
    for panel, tempat in letak:
        robot._panel(fig.add_subplot(tempat), panel, satu, data["psi_max_deg"], mask, mode, tabel)
    fig.tight_layout(rect=(0, 0, 1, 0.955))
    robot.simpan_gambar(fig, path_png)
    plt.close(fig)
    robot.simpan_panel(os.path.dirname(path_png), nama, satu, data["psi_max_deg"], mask, mode, tabel)


def tulis_hasil(path, nama, mode, data, jumlah_c, catatan):
    """CSV (kolom ise.KOLOM_CSV + c_h1 .., sama dengan ise.py dan robot fisik), lalu gambar .png + .eps."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    kolom_c = ["c_h%d" % (i + 1) for i in range(jumlah_c)]
    with open(path + ".csv", "w", newline="") as f:
        tulis = csv.writer(f)
        tulis.writerow(ise.KOLOM_CSV + kolom_c)
        for baris in data:
            teks = [nilai if isinstance(nilai, str) else "%.8g" % nilai for nilai in baris]
            teks.extend([""] * (len(ise.KOLOM_CSV) + len(kolom_c) - len(teks)))   # baris tanpa nilai constraint
            tulis.writerow(teks)
    robot = muat_ise_robot()
    gambar_data_uji(robot, robot.baca_csv(path + ".csv"), nama, mode, catatan, path + ".png")


def _proses_simpan(path, nama, mode, data, jumlah_c, catatan):
    """Isi proses penyimpanan (juga dipakai langsung bila proses terpisah gagal dibuat)."""
    try:
        tulis_hasil(path, nama, mode, data, jumlah_c, catatan)
        print("Tersimpan: %s (.csv, .png, .eps)" % path, flush=True)
    except Exception as kesalahan:
        import traceback
        traceback.print_exc()
        print("GAGAL menyimpan %s -> %s: %s" % (nama, type(kesalahan).__name__, kesalahan), flush=True)


_PROSES = []


def simpan_latar(path, nama, mode, data, jumlah_c, catatan):
    """Simpan di proses terpisah (spawn) supaya simulasi tidak berhenti selama matplotlib menggambar."""
    try:
        proses = multiprocessing.get_context("spawn").Process(target=_proses_simpan,
                                                              args=(path, nama, mode, data, jumlah_c, catatan))
        proses.start()
        _PROSES.append(proses)
    except Exception as kesalahan:
        print("Proses penyimpanan terpisah gagal dibuat (%s): disimpan langsung." % kesalahan)
        _proses_simpan(path, nama, mode, data, jumlah_c, catatan)


def simpan_berjalan():
    """Jumlah penyimpanan yang belum selesai."""
    _PROSES[:] = [proses for proses in _PROSES if proses.is_alive()]
    return len(_PROSES)


def tunggu_simpan(batas_waktu=120.0):
    """Tunggu semua penyimpanan selesai (dipanggil atera_main.py saat keluar)."""
    jumlah = simpan_berjalan()
    if jumlah > 0:
        print("Menunggu %d penyimpanan PENGUJIAN OTOMATIS selesai ..." % jumlah)
    for proses in list(_PROSES):
        proses.join(batas_waktu)
    simpan_berjalan()


KOLOM_RINGKASAN = (["waktu", "kontrol", "uji", "sudut_bandul_deg", "massa_bandul_kg", "panjang_tali_m",
                    "lama_tumbukan_s", "gaya_N", "impuls_N_s", "zeta_deg", "psi_max_DEG", "percobaan_ke", "hasil",
                    "sebab", "ketukan_turun", "psi_maks_deg", "durasi_rekam_s", "file"]
                   + ["Kp_psi", "Kd_psi", "Kp_theta", "Kd_theta", "Kvel"] + list(config.NAMA_ALPHA))


# =============================================================================
# Pengujian
# =============================================================================
class Batal(Exception):
    """Pengujian dibatalkan: U lagi ("u") atau Q ("q")."""


class Komet(Exception):
    """KOMET terdeteksi saat percobaan (isi = sebab)."""


class WaktuHabis(Exception):
    """Batas waktu percobaan terlewati (isi = sebab)."""


class UjiOtomatis:
    """Seluruh PENGUJIAN OTOMATIS. Dibuat dan dijalankan atera_main.py saat U ditekan:
        UjiOtomatis(sim, pd, cbf, params).jalankan()   ->  SELESAI / BATAL / KELUAR
    Loop kontrolnya sendiri (urutan sama dengan atera_main.py): perintah -> state -> KOMET -> DOB -> PD -> CBF-QP ->
    PD heading -> DOB (catat u) -> step, waktu nyata."""

    def __init__(self, sim, pd, cbf, params):
        self.sim = sim
        self.pd = pd
        self.cbf = cbf
        self.params = dict(params)             # gain jendela Kontrol saat U ditekan (dipakai SEMUA pengujian)
        self.daftar_kontrol = daftar_kontrol()
        self.daftar_sudut = [float(s) for s in config.UJI1_SUDUT_BANDUL_DEG]
        self.mask = [bool(cbf.mask_mode[MODE_DIAM][k] > 0.0) for k in range(cbf.NH)]   # h1 .. h4 (C_MODE DIAM)
        self.nomor_h = [k + 1 for k in range(cbf.NH) if self.mask[k]]

        # Keadaan sekarang
        self.kontrol = self.daftar_kontrol[0]
        self.uji = ""
        self.sudut = None
        self.gaya = 0.0                        # [N] gaya pukulan percobaan ini (0 = UJI 2)
        self.percobaan_ke = 0
        self.zeta_cbf_deg = 0.0                # [deg] zeta model CBF-QP (UJI 1: 0, UJI 2: UJI2_CBF_ZETA_DEG)
        self.fase = "menyiapkan"
        self.komet = False
        self.ketukan = 0
        self.merekam = False
        self.t_rekam = 0.0                     # [s] waktu sejak mulai direkam
        self.psi_maks = 0.0                    # [rad] |psi| maksimum yang direkam
        self.perekam = ise.PerekamISE()
        self.hasil_terakhir = "-"
        self.jumlah = {BERHASIL: 0, GAGAL: 0}
        self.path_ringkasan = ""
        self.riwayat = collections.deque()     # (t sejak diletakkan, y poros, |psi|) 15 s terakhir (lihat diam())
        self.t_jalan = 0.0                     # [s] waktu sejak robot diletakkan

        # Nilai terakhir (tampilan)
        self.x = [0.0, 0.0, 0.0, 0.0]
        self.yaw = 0.0
        self.u_PD = 0.0
        self.u = 0.0
        self.status_cbf = "-"

        # Waktu nyata
        self.siklus_ke = 0
        self.waktu_berikutnya = time.time()
        self.waktu_ukur = time.time()
        self.siklus_ukur = 0
        self.kecepatan = 1.0

    # ------------------------------------------------------------------ urutan
    def jalankan(self):
        """Jalankan semua pengujian. Mengembalikan SELESAI, BATAL (U lagi), atau KELUAR (Q)."""
        sim = self.sim
        zeta_simpan = (config.zeta_DEG, config.zeta)
        lantai_simpan = sim.zeta
        hasil = SELESAI
        mulai = time.time()
        self.cetak_awal()
        try:
            self.buat_ringkasan()
            if abs(sim.zeta) > 1e-12:
                print("Lantai dibuat datar untuk pengujian otomatis (zeta lantai %.1f deg -> 0)." % math.degrees(sim.zeta))
                sim.ubah_zeta(0.0)
            sim.hapus_lintasan()

            # UJI 1: bidang datar, pukulan bandul
            sim.atur_kamera(config.UJI1_KAMERA_JARAK, config.KAMERA_YAW, config.KAMERA_PITCH, config.UJI1_KAMERA_TARGET)
            for kontrol in self.daftar_kontrol:
                for sudut in self.daftar_sudut:
                    self.uji_satu(kontrol, UJI1, sudut)

            # UJI 2: tanjakan. Robot selalu diletakkan dengan poros roda di y = 0 (simulasi.reset, lantai datar).
            lin = sim.buat_lintasan(config.UJI2_DATAR_BAWAH_M, config.UJI2_ZETA_DEG, config.UJI2_PANJANG_MIRING_M,
                                    config.UJI2_DATAR_ATAS_M, config.UJI2_LEBAR_M, config.UJI2_TEBAL_PAPAN_M,
                                    config.UJI2_WARNA_PAPAN, config.UJI2_WARNA_PANGGUNG)
            print("Lintasan UJI 2: tanjakan y %.2f .. %.2f m (tinggi %.3f m), bidang atas sampai y %.2f m."
                  % (lin["y_mulai"], lin["y_puncak"], lin["tinggi"], lin["y_akhir"]))
            sim.atur_kamera(config.UJI2_KAMERA_JARAK, config.KAMERA_YAW, config.UJI2_KAMERA_PITCH,
                            [0.0, 0.5 * (lin["y_akhir"] - 0.3), 0.5 * lin["tinggi"]])
            for kontrol in self.daftar_kontrol:
                self.uji_satu(kontrol, UJI2, None)
        except Batal as alasan:
            hasil = KELUAR if str(alasan) == "q" else BATAL
            print("PENGUJIAN OTOMATIS dibatalkan (%s). Percobaan yang sedang berjalan tidak disimpan."
                  % ("Q: keluar" if hasil == KELUAR else "U ditekan lagi"))
        finally:
            self.merekam = False
            config.zeta_DEG, config.zeta = zeta_simpan
            try:
                sim.hapus_lintasan()
                if abs(sim.zeta - lantai_simpan) > 1e-12:
                    sim.ubah_zeta(lantai_simpan)
                sim.kamera_awal()
                sim.kirim_catatan(["PENGUJIAN OTOMATIS %s: berhasil %d, gagal %d (lihat ringkasan %s)"
                                   % (hasil, self.jumlah[BERHASIL], self.jumlah[GAGAL],
                                      os.path.basename(self.path_ringkasan))])
            except Exception:
                pass                     # jendela PyBullet sudah ditutup
        self.cetak_akhir(hasil, time.time() - mulai)
        return hasil

    def uji_satu(self, kontrol, uji, sudut):
        """Satu kontrol + satu uji (UJI 1: satu sudut bandul): berhasil -> simpan, selesai. Gagal -> ulang sampai
        UJI_PERCOBAAN_MAKS kali, yang disimpan hanya percobaan gagal terakhir."""
        for ke in range(1, config.UJI_PERCOBAAN_MAKS + 1):
            hasil, sebab = self.percobaan(kontrol, uji, sudut, ke)
            terakhir = hasil == BERHASIL or ke >= config.UJI_PERCOBAAN_MAKS
            nama = self.simpan_percobaan(hasil, sebab) if terakhir else ""
            self.jumlah[hasil] = self.jumlah[hasil] + 1
            self.catat_ringkasan(hasil, sebab, nama)
            self.hasil_terakhir = "%s %s %s ke-%d: %s%s" % (kontrol["label"], uji, self.teks_uji_pendek(), ke, hasil,
                                                            (" -> KE-" + nama.rsplit("KE-", 1)[-1]) if nama
                                                            else " (tidak disimpan)")
            print("  -> %s (%s)%s" % (hasil, sebab, ("  disimpan: " + nama) if nama else "  (tidak disimpan, diulang)"))
            if hasil == BERHASIL:
                return

    def percobaan(self, kontrol, uji, sudut, ke):
        """Satu percobaan. Mengembalikan (BERHASIL / GAGAL, sebab)."""
        self.kontrol = kontrol
        self.uji = uji
        self.sudut = sudut
        self.percobaan_ke = ke
        self.gaya = gaya_bandul(sudut) if uji == UJI1 else 0.0
        self.zeta_cbf_deg = 0.0 if uji == UJI1 else float(config.UJI2_CBF_ZETA_DEG)
        self.ketukan = 0
        self.letakkan()
        print("")
        print("[PENGUJIAN OTOMATIS] %s | %s %s | percobaan %d / %d"
              % (kontrol["label"], uji, self.teks_uji_pendek(), ke, config.UJI_PERCOBAAN_MAKS))
        try:
            if uji == UJI1:
                hasil, sebab = self.jalankan_uji1()
            else:
                hasil, sebab = self.jalankan_uji2()
        except Komet as sebab_komet:
            hasil, sebab = GAGAL, "KOMET, %s" % sebab_komet
            self.merekam = False
            self.fase = "KOMET (%s): motor mati, robot dibiarkan jatuh" % sebab_komet
            self.jalan_selama(config.UJI_JEDA_KOMET_S)
        except WaktuHabis as alasan:
            hasil, sebab = GAGAL, str(alasan)
        self.merekam = False
        return hasil, sebab

    def jalankan_uji1(self):
        """UJI 1: tenang -> rekam -> 1 s -> dipukul dari belakang -> sampai DIAM UJI_STABIL_LAMA_S detik."""
        self.fase = "robot diletakkan, tenang %g s (belum direkam)" % config.UJI_TENANG_S
        self.jalan_selama(config.UJI_TENANG_S)
        self.mulai_rekam()
        self.fase = "merekam %g s sebelum pukulan" % config.UJI_SEBELUM_PUKUL_S
        self.jalan_selama(config.UJI_SEBELUM_PUKUL_S)

        self.sim.gaya_dorong = self.gaya
        self.sim.dorong(1)                       # dari belakang, ke depan (seperti tombol O)
        print("  dipukul dari belakang di t = %.2f s: bandul %g deg -> F %.2f N selama %.3f s (impuls %.3f N s)"
              % (self.t_rekam, self.sudut, self.gaya, config.LAMA_DORONG, impuls_bandul(self.sudut)))
        t_pukul = 0.0
        while True:
            gerak = self.geser(min(t_pukul, config.UJI_STABIL_LAMA_S)) if t_pukul > 0.0 else None
            self.fase = ("dipukul %.2f N, menunggu diam %g s (geser %s cm, batas %g cm) | sudah %.0f / %g s"
                         % (self.gaya, config.UJI_STABIL_LAMA_S, "-" if gerak is None else "%.1f" % (gerak * 100.0),
                            config.UJI_DIAM_GESER_M * 100.0, math.floor(t_pukul), config.UJI_STABIL_BATAS_S))
            self.siklus(0)
            t_pukul = t_pukul + config.DT
            if t_pukul >= config.UJI_STABIL_LAMA_S - 0.5 * config.DT and self.diam(config.UJI_STABIL_LAMA_S):
                return BERHASIL, "diam %g s" % config.UJI_STABIL_LAMA_S
            if t_pukul >= config.UJI_STABIL_BATAS_S - 0.5 * config.DT:
                return GAGAL, "tidak diam dalam %g s sejak pukulan" % config.UJI_STABIL_BATAS_S

    def jalankan_uji2(self):
        """UJI 2: diam -> NAIK (Kvel) -> berhenti di bidang atas UJI2_DIAM_ATAS_S detik -> MUNDUR (Kvel) sampai tepi
        turunan -> TURUN karena gravitasi -> BERHENTI (diam) di bidang datar bawah."""
        lin = self.sim.lintasan
        dt = config.DT

        def langkah(arah):
            x = self.siklus(arah)
            self.cek_waktu()
            return x

        self.fase = "robot diletakkan %g m sebelum tanjakan, tenang %g s (belum direkam)" \
            % (config.UJI2_DATAR_BAWAH_M, config.UJI_TENANG_S)
        self.jalan_selama(config.UJI_TENANG_S)
        self.mulai_rekam()
        self.fase = "diam %g s di bidang datar bawah" % config.UJI2_DIAM_AWAL_S
        for _ in range(int(round(config.UJI2_DIAM_AWAL_S / dt))):
            langkah(0)

        # 1. NAIK: Kvel arah maju sampai poros roda UJI2_LEPAS_ATAS_M masuk bidang datar atas. Bila poros tidak maju
        #    UJI2_NAIK_MACET_M dalam UJI2_NAIK_MACET_S detik: GAGAL "tidak kuat naik".
        batas = lin["y_puncak"] + config.UJI2_LEPAS_ATAS_M
        print("  NAIK dengan Kvel %g (arah maju) di t = %.2f s" % (self.params["Kvel"], self.t_rekam))
        y_terjauh = self.y_poros()
        t_terjauh = self.t_rekam
        while self.y_poros() < batas:
            self.fase = ("NAIK: Kvel %g arah maju | poros %.1f m dari awal tanjakan (Kvel dilepas di %.1f m)"
                         % (self.params["Kvel"], self.y_poros() - lin["y_mulai"], batas - lin["y_mulai"]))
            langkah(1)
            y = self.y_poros()
            if y > y_terjauh + config.UJI2_NAIK_MACET_M:
                y_terjauh = y
                t_terjauh = self.t_rekam
            elif self.t_rekam - t_terjauh >= config.UJI2_NAIK_MACET_S:
                return GAGAL, ("tidak kuat naik: tidak maju %g m dalam %g s, terjauh %.2f m dari awal tanjakan"
                               % (config.UJI2_NAIK_MACET_M, config.UJI2_NAIK_MACET_S, y_terjauh - lin["y_mulai"]))

        # 2. BERHENTI di bidang atas: Kvel dilepas, tunggu UJI2_DIAM_ATAS_S detik (waktu tetap), lalu SELALU mundur.
        print("  Kvel dilepas di bidang atas di t = %.2f s, berhenti %g s" % (self.t_rekam, config.UJI2_DIAM_ATAS_S))
        jumlah = int(round(config.UJI2_DIAM_ATAS_S / dt))
        for i in range(jumlah):
            self.fase = "BERHENTI di bidang atas (Kvel dilepas) %.1f / %g s, lalu mundur" \
                % (math.floor(i * dt * 10.0) / 10.0, config.UJI2_DIAM_ATAS_S)
            langkah(0)

        # 3. MUNDUR: Kvel arah mundur HANYA di bidang atas, sampai poros roda melewati tepi atas tanjakan
        batas = lin["y_puncak"] - config.UJI2_LEPAS_TURUN_M
        print("  MUNDUR dengan Kvel (arah mundur) di t = %.2f s" % self.t_rekam)
        while self.y_poros() > batas:
            self.fase = "MUNDUR: Kvel %g arah mundur sampai tepi turunan (%.2f m lagi)" \
                % (self.params["Kvel"], max(0.0, self.y_poros() - batas))
            langkah(-1)

        # 4. TURUN karena gravitasi (Kvel dilepas). Macet (poros turun < UJI2_KETUKAN_GESER_M selama
        #    UJI2_KETUKAN_DIAM_S, belum sampai bidang bawah) -> ketukan Kvel arah mundur UJI2_KETUKAN_S.
        # 5. BERHENTI di bidang datar bawah: DIAM UJI_STABIL_LAMA_S detik -> BERHASIL.
        print("  Kvel dilepas di tepi turunan di t = %.2f s: TURUN karena gravitasi" % self.t_rekam)
        self.fase = "TURUN karena gravitasi (Kvel dilepas) | ketukan Kvel %d / %d" \
            % (self.ketukan, config.UJI2_KETUKAN_MAKS)
        sisa_ketukan = 0.0
        t_boleh_ketuk = self.t_rekam + config.UJI2_KETUKAN_DIAM_S
        while True:
            arah = -1 if sisa_ketukan > 0.0 else 0
            langkah(arah)
            if sisa_ketukan > 0.0:
                sisa_ketukan = sisa_ketukan - dt
                t_boleh_ketuk = self.t_rekam + config.UJI2_KETUKAN_DIAM_S
                continue
            if self.y_poros() < lin["y_mulai"]:
                gerak = self.geser(config.UJI_STABIL_LAMA_S)
                self.fase = "sampai di bidang datar bawah, menunggu BERHENTI (diam %g s, geser %s cm)" \
                    % (config.UJI_STABIL_LAMA_S, "-" if gerak is None else "%.1f" % (gerak * 100.0))
                if self.diam(config.UJI_STABIL_LAMA_S):
                    return BERHASIL, "berhenti di bidang bawah" + (", %d ketukan Kvel" % self.ketukan
                                                                    if self.ketukan else "")
                continue
            y_lalu = self.posisi_lalu(config.UJI2_KETUKAN_DIAM_S)
            macet = (self.t_rekam >= t_boleh_ketuk and y_lalu is not None
                     and y_lalu - self.y_poros() < config.UJI2_KETUKAN_GESER_M)
            if self.ketukan < config.UJI2_KETUKAN_MAKS:
                self.fase = "TURUN karena gravitasi (Kvel dilepas) | ketukan Kvel %d / %d" \
                    % (self.ketukan, config.UJI2_KETUKAN_MAKS)
                if macet:
                    self.ketukan = self.ketukan + 1
                    sisa_ketukan = config.UJI2_KETUKAN_S
                    print("  macet di t = %.2f s: ketukan Kvel arah mundur %d / %d (%g s)"
                          % (self.t_rekam, self.ketukan, config.UJI2_KETUKAN_MAKS, config.UJI2_KETUKAN_S))
            else:
                self.fase = "berhenti di tanjakan setelah %d ketukan, menunggu diam %g s" \
                    % (self.ketukan, config.UJI_STABIL_LAMA_S)
                if self.diam(config.UJI_STABIL_LAMA_S):
                    return BERHASIL, "berhenti di tanjakan (%d ketukan Kvel)" % self.ketukan

    # ------------------------------------------------------------ satu siklus
    def letakkan(self):
        """Robot diletakkan ulang (posisi awal, mode DIAM, DOB kosong, constraint h1 .. h4), perekam dikosongkan."""
        sim = self.sim
        self.komet = False
        self.merekam = False
        self.t_rekam = 0.0
        self.psi_maks = 0.0
        if sim.judul != self.kontrol["mode"]:
            sim.atur_mode(self.kontrol["mode"])
        self.terapkan_parameter()
        sim.reset()
        self.pd.atur_mode(MODE_DIAM)
        self.pd.set_command(0, 0)
        self.pd.pasang_yaw_ref(sim.baca_yaw()[0])
        self.cbf.reset_dob()
        self.cbf.pilih_barrier(self.nomor_h)
        sim.atur_mask_plot(self.mask)
        self.perekam.mulai(self.kontrol["mode"], ise.JENIS_PENGUJIAN)
        self.riwayat = collections.deque()
        self.t_jalan = 0.0
        self.waktu_berikutnya = time.time()

    def mulai_rekam(self):
        self.perekam.mulai(self.kontrol["mode"], ise.JENIS_PENGUJIAN)
        self.t_rekam = 0.0
        self.psi_maks = 0.0
        self.merekam = True

    def psi_max_deg(self):
        """psi_max kontrol ini [deg] (PD: nilai jendela Kontrol, hanya untuk garis gambar)."""
        if self.kontrol["psi_max_deg"] is not None:
            return self.kontrol["psi_max_deg"]
        return float(self.params["psi_max_DEG"])

    def terapkan_parameter(self):
        """Gain (nilai saat U ditekan), psi_max kontrol ini, dan zeta model CBF-QP percobaan ini."""
        nilai = self.params
        self.pd.Kp_psi = nilai["Kp_psi"]
        self.pd.Kd_psi = nilai["Kd_psi"]
        self.pd.Kp_theta = nilai["Kp_theta"]
        self.pd.Kd_theta = nilai["Kd_theta"]
        self.pd.Kvel = nilai["Kvel"]
        for nama in config.NAMA_ALPHA:            # cbf_qp.py membaca Alpha dari config tiap siklus
            setattr(config, nama, nilai[nama])
        config.psi_max_DEG = self.psi_max_deg()
        config.psi_max = math.radians(config.psi_max_DEG)
        config.psi_min = -config.psi_max
        config.zeta_DEG = self.zeta_cbf_deg       # model CBF-QP (CBF_TAHU_ZETA); lantai PyBullet tidak berubah
        config.zeta = math.radians(self.zeta_cbf_deg)

    def gain_rekam(self):
        """Kolom gain CSV: nilai saat U ditekan, psi_max kontrol ini, zeta model CBF-QP."""
        gain = dict(self.params)
        gain["psi_max_DEG"] = self.psi_max_deg()
        gain["zeta_DEG"] = self.zeta_cbf_deg
        return gain

    def ekstra(self):
        """Kolom CSV tambahan (ise.KOLOM_EKSTRA), sama dengan atera_main.data_ekstra."""
        pd, cbf = self.pd, self.cbf
        d_dtheta, d_dpsi = cbf.d_hat
        return {"arah": pd.arah, "bebas": 0.0, "balancing": 0.0,
                "yaw_rad": pd.yaw, "dyaw_rad_s": pd.dyaw, "yaw_ref_rad": pd.yaw_ref, "u_belok_Nm": pd.u_belok,
                "dob_dtheta_rad_s2": d_dtheta, "dob_dpsi_rad_s2": d_dpsi,
                "slack": cbf.slack if self.kontrol["mode"] == ise.MODE_PD_CBF else 0.0}

    def siklus(self, arah=0):
        """Satu siklus kontrol (urutan sama dengan atera_main.py). arah = perintah Kvel (+1 maju, -1 mundur, 0 lepas).
        Mengembalikan state x. Raise Komet saat KOMET pertama terdeteksi (motor sudah dimatikan), Batal bila U / Q."""
        sim, pd, cbf = self.sim, self.pd, self.cbf
        tombol = sim.baca_keyboard()
        if "q" in tombol:
            raise Batal("q")
        if "u" in tombol:
            raise Batal("u")
        sim.baca_params()                  # jendela Kontrol tetap hidup; nilainya tidak dipakai selama pengujian
        sim.gaya_dorong = self.gaya
        self.terapkan_parameter()
        pd.set_command(0 if self.komet else arah, 0)

        x = sim.baca_state()
        yaw, dyaw = sim.baca_yaw()
        psi = x[2]

        # KOMET: |psi| > SAFE_TILT_DEG atau berputar (mode DIAM)
        sebab = ""
        if abs(psi) > math.radians(config.SAFE_TILT_DEG):
            sebab = "psi = %.1f deg" % math.degrees(psi)
        elif abs(yaw - pd.yaw_ref) > math.radians(config.YAW_KOMET_DEG):
            sebab = "berputar, yaw - yaw_ref = %.0f deg" % math.degrees(yaw - pd.yaw_ref)
        komet_baru = sebab != "" and not self.komet
        if komet_baru:
            self.komet = True
            cbf.reset_dob()
            print("  KOMET di t = %.2f s (%s): motor dimatikan." % (self.t_rekam, sebab))

        nilai_c = None
        if self.komet:
            u_PD = 0.0
            u = 0.0
            u_belok = 0.0
            pd.u_belok = 0.0
            status_cbf = "MATI"
        else:
            cbf.dob_perbarui(x, config.DT)
            u_PD = pd.compute(x)
            if self.kontrol["mode"] == ise.MODE_PD:
                u = max(config.u_min, min(config.u_max, u_PD))
                status_cbf = "-"
            else:
                u = cbf.compute(x, u_PD)
                status_cbf = cbf.status
                nilai_c = [float(c) if m > 0.0 and math.isfinite(c) else None for c, m in zip(cbf.c, cbf.mask)]
            u_belok = pd.compute_belok(yaw, dyaw, u, config.u_max, config.DT)
            cbf.dob_catat(x, u)

        dorong = sim.arah_dorong * sim.gaya_dorong if sim.sisa_dorong > 0.0 else 0.0
        sim.step(u, u_belok)

        if self.merekam and not self.komet:
            self.t_rekam = self.t_rekam + config.DT
            self.perekam.tambah(self.t_rekam, x, pd.error_theta, pd.error_dtheta, pd.error_psi, pd.error_dpsi,
                                u_PD, u, status_cbf, self.gain_rekam(), dorong, nilai_c, self.ekstra())
            self.psi_maks = max(self.psi_maks, abs(psi))

        # Riwayat posisi poros roda dan |psi| (untuk diam(), geser(), posisi_lalu())
        self.t_jalan = self.t_jalan + config.DT
        self.riwayat.append((self.t_jalan, self.y_poros(), abs(psi)))
        while self.riwayat and self.riwayat[0][0] < self.t_jalan - 15.0:
            self.riwayat.popleft()

        self.x = x
        self.yaw = yaw
        self.u_PD = u_PD
        self.u = u
        self.status_cbf = status_cbf
        self.siklus_ke = self.siklus_ke + 1
        if self.siklus_ke % 10 == 0:
            self.tampil()
        self.atur_waktu()
        if komet_baru:
            raise Komet(sebab)
        return x

    def jalan_selama(self, lama, arah=0):
        for _ in range(int(round(lama / config.DT))):
            self.siklus(arah)

    def _jendela(self, lama):
        """Isi riwayat `lama` detik terakhir, None bila riwayat belum sepanjang itu."""
        if not self.riwayat or self.riwayat[-1][0] - self.riwayat[0][0] < lama - 0.5 * config.DT:
            return None
        batas = self.riwayat[-1][0] - lama - 0.5 * config.DT
        return [isi for isi in self.riwayat if isi[0] >= batas]

    def geser(self, lama):
        """max - min posisi poros roda selama `lama` detik terakhir [m], None bila riwayat belum sepanjang itu."""
        isi = self._jendela(lama)
        if isi is None:
            return None
        semua = [y for _, y, _ in isi]
        return max(semua) - min(semua)

    def posisi_lalu(self, lama):
        """Posisi poros roda `lama` detik yang lalu [m], None bila riwayat belum sepanjang itu."""
        isi = self._jendela(lama)
        return None if isi is None else isi[0][1]

    def diam(self, lama):
        """Robot DIAM selama `lama` detik terakhir: |psi| < UJI_STABIL_PSI_DEG terus-menerus dan poros roda bergeser
        < UJI_DIAM_GESER_M. Dinilai dari posisi (bukan |dpsi| / |dtheta|), jadi getaran kecil roda tidak dihitung."""
        isi = self._jendela(lama)
        if isi is None:
            return False
        batas_psi = math.radians(config.UJI_STABIL_PSI_DEG)
        semua = [y for _, y, _ in isi]
        return max(semua) - min(semua) < config.UJI_DIAM_GESER_M and max(p for _, _, p in isi) < batas_psi

    def y_poros(self):
        """Letak poros roda searah lintasan (+Y dunia) [m]."""
        return float(self.sim.posisi_poros()[1])

    def cek_waktu(self):
        if self.t_rekam > config.UJI2_BATAS_S:
            raise WaktuHabis("tidak selesai dalam %g s" % config.UJI2_BATAS_S)

    def atur_waktu(self):
        """Simulasi berjalan dengan kecepatan waktu nyata (supaya bisa direkam layar)."""
        self.waktu_berikutnya = self.waktu_berikutnya + config.DT
        jeda = self.waktu_berikutnya - time.time()
        if jeda > 0.0:
            time.sleep(jeda)
        else:
            self.waktu_berikutnya = time.time()
        self.siklus_ukur = self.siklus_ukur + 1
        if self.siklus_ukur >= 200:
            sekarang = time.time()
            self.kecepatan = self.siklus_ukur * config.DT / max(sekarang - self.waktu_ukur, 1e-6)
            self.waktu_ukur = sekarang
            self.siklus_ukur = 0

    # ---------------------------------------------------------------- tampilan
    def teks_uji_pendek(self):
        if self.uji == UJI1:
            return "bandul %g deg" % self.sudut
        return "tanjakan %g deg" % config.UJI2_ZETA_DEG

    def baris_catatan(self):
        """Catatan pengujian (blok kiri bawah jendela PyBullet dan panel catatan jendela plot): list (teks, warna).
        Yang sedang berjalan ditandai >> <<."""
        biasa = config.WARNA_BIASA
        sekarang = config.WARNA_UJI_SEKARANG
        nilai = self.params
        teks_kontrol = "   ".join((">>%s<<" % k["label"]) if k is self.kontrol else k["label"]
                                  for k in self.daftar_kontrol)
        teks_sudut = " ".join((">>%g<<" % s) if (self.uji == UJI1 and s == self.sudut) else ("%g" % s)
                              for s in self.daftar_sudut)
        uji1 = "UJI 1   : %sbidang datar, bandul dipukul dari belakang, sudut %s deg%s" \
            % (">> " if self.uji == UJI1 else "", teks_sudut, "   (selesai)" if self.uji == UJI2 else "")
        uji2 = "UJI 2   : %stanjakan %g deg, panjang %g m (datar bawah %g m - miring - datar atas %g m)%s" \
            % (">> " if self.uji == UJI2 else "", config.UJI2_ZETA_DEG, config.UJI2_PANJANG_MIRING_M,
               config.UJI2_DATAR_BAWAH_M, config.UJI2_DATAR_ATAS_M, " <<" if self.uji == UJI2 else "")
        if self.uji == UJI1:
            teks_sekarang = "SEKARANG: %s | UJI 1 bandul %g deg -> F %.2f N | percobaan %d / %d" \
                % (self.kontrol["label"], self.sudut, self.gaya, self.percobaan_ke, config.UJI_PERCOBAAN_MAKS)
        else:
            teks_sekarang = "SEKARANG: %s | UJI 2 tanjakan %g deg | percobaan %d / %d | ketukan Kvel %d / %d" \
                % (self.kontrol["label"], config.UJI2_ZETA_DEG, self.percobaan_ke, config.UJI_PERCOBAAN_MAKS,
                   self.ketukan, config.UJI2_KETUKAN_MAKS)
        bandul = "BANDUL  : m_bd %g kg | l_bd %g m | dt tumbukan %g s | F = m_bd*sqrt(2*g*l_bd*(1-cos(phi)))/dt" \
            % (config.BANDUL_MASSA_KG, config.BANDUL_PANJANG_TALI_M, config.LAMA_DORONG)
        gain = "GAIN PD : Kp_psi %g | Kd_psi %g | Kd_theta %g | Kvel %g | Kp_theta tidak dipakai (mode DIAM)" \
            % (nilai["Kp_psi"], nilai["Kd_psi"], nilai["Kd_theta"], nilai["Kvel"])
        if self.kontrol["mode"] == ise.MODE_PD_CBF:
            alpha = " | ".join("%s %g" % (nama, nilai[nama]) for nama in config.NAMA_ALPHA)
            cbf = "CBF-QP  : %s | psi_max %g deg | %s | zeta model %g deg" \
                % (alpha, self.psi_max_deg(), ise.teks_mask(self.mask), self.zeta_cbf_deg)
        else:
            cbf = "CBF-QP  : tidak dipakai (kontrol PD saja)"
        stabil = "DIAM    : |psi| < %g deg dan poros bergeser < %g cm selama %g s | gagal = KOMET (|psi| > %g deg)" \
            % (config.UJI_STABIL_PSI_DEG, config.UJI_DIAM_GESER_M * 100.0, config.UJI_STABIL_LAMA_S,
               config.SAFE_TILT_DEG)
        hasil = "HASIL   : berhasil %d, gagal %d | terakhir: %s" \
            % (self.jumlah[BERHASIL], self.jumlah[GAGAL], self.hasil_terakhir)
        return [("PENGUJIAN OTOMATIS SIMULASI ATERA   (tekan U lagi untuk membatalkan)", config.WARNA_UJI_JUDUL),
                ("KONTROL : " + teks_kontrol, biasa),
                (uji1, biasa),
                (uji2, biasa),
                (teks_sekarang, sekarang),
                ("STATUS  : " + self.fase, sekarang),
                (bandul, biasa),
                (gain, biasa),
                (cbf, biasa),
                (stabil, biasa),
                (hasil, biasa)]

    def tampil(self):
        """Blok parameter (kiri atas), catatan pengujian (kiri bawah + jendela plot), dan jendela Kontrol."""
        x = self.x
        if self.komet:
            baris = ["PENGUJIAN OTOMATIS | KOMET, motor mati (percobaan diulang)"]
        elif self.kontrol["mode"] == ise.MODE_PD:
            baris = ["PENGUJIAN OTOMATIS | %s | mode DIAM | zeta model %.1f deg" % (self.kontrol["label"],
                                                                                   self.zeta_cbf_deg)]
        else:
            baris = ["PENGUJIAN OTOMATIS | %s | mode DIAM | CBF: %s | zeta model %.1f deg"
                     % (self.kontrol["label"], self.status_cbf, self.zeta_cbf_deg)]
        baris.append("psi %+.1f / %.1f deg    dpsi %+.2f rad/s" % (math.degrees(x[2]), config.psi_max_DEG, x[3]))
        baris.append("theta %+.2f rad (s %+.2f m)    dtheta %+.2f / %.2f rad/s"
                     % (x[0], config.R * x[0], x[1], config.dtheta_max))
        baris.append("u_PD %+.3f   u %+.3f N m    arus %+.2f / %.2f A"
                     % (self.u_PD, self.u, self.u / config.MOTOR_KT, config.MAX_CURRENT_A))
        baris.append("yaw %+.1f (ref %+.1f) deg  |  arah Kvel %+d  |  DOB ddpsi %+.1f  slack %.2g"
                     % (math.degrees(self.yaw), math.degrees(self.pd.yaw_ref), self.pd.arah, self.cbf.d_hat[1],
                        self.cbf.slack if self.kontrol["mode"] == ise.MODE_PD_CBF else 0.0))
        baris.append("ISE psi %.6f    t rekam %.1f s  (kecepatan x%.2f waktu nyata)"
                     % (self.perekam.ise_psi, self.t_rekam, self.kecepatan))
        catatan = self.baris_catatan()
        teks_kontrol = "PENGUJIAN OTOMATIS | %s | %s %s | percobaan %d / %d | U = batal" \
            % (self.kontrol["label"], self.uji, self.teks_uji_pendek(), self.percobaan_ke, config.UJI_PERCOBAAN_MAKS)
        self.sim.tampilkan(baris, MODE_DIAM, False, teks_kontrol, False, catatan, True)
        self.sim.kirim_catatan([teks for teks, _ in catatan])

    # ------------------------------------------------------------- simpan
    def teks_catatan_gambar(self, hasil, sebab):
        """Baris catatan di judul gambar (dibungkus per ' | ')."""
        if self.uji == UJI1:
            return ("SIMULASI PyBullet | UJI 1 bidang datar (zeta 0 deg), mode DIAM | bandul dipukul dari belakang | "
                    "phi_0 %g deg | m_bd %g kg | l_bd %g m | lama tumbukan dt %g s | "
                    "F = m_bd*sqrt(2*g*l_bd*(1-cos(phi_0)))/dt = %.2f N | impuls %.3f N s | dipukul di t = %g s | "
                    "percobaan %d / %d: %s (%s)"
                    % (self.sudut, config.BANDUL_MASSA_KG, config.BANDUL_PANJANG_TALI_M, config.LAMA_DORONG, self.gaya,
                       impuls_bandul(self.sudut), config.UJI_SEBELUM_PUKUL_S, self.percobaan_ke,
                       config.UJI_PERCOBAAN_MAKS, hasil, sebab))
        return ("SIMULASI PyBullet | UJI 2 tanjakan %g deg, panjang %g m (datar bawah %g m, datar atas %g m), mode DIAM | "
                "naik Kvel %g arah maju, dilepas %g m di bidang atas, berhenti %g s | mundur Kvel sampai tepi turunan | "
                "turun karena gravitasi, ketukan Kvel %d x %g s | zeta model CBF-QP %g deg | percobaan %d / %d: %s (%s)"
                % (config.UJI2_ZETA_DEG, config.UJI2_PANJANG_MIRING_M, config.UJI2_DATAR_BAWAH_M,
                   config.UJI2_DATAR_ATAS_M, self.params["Kvel"], config.UJI2_LEPAS_ATAS_M, config.UJI2_DIAM_ATAS_S,
                   self.ketukan, config.UJI2_KETUKAN_S, self.zeta_cbf_deg, self.percobaan_ke,
                   config.UJI_PERCOBAAN_MAKS, hasil, sebab))

    def simpan_percobaan(self, hasil, sebab):
        """Simpan rekaman percobaan ini (proses terpisah). Mengembalikan nama file ("" = tidak ada data)."""
        if len(self.perekam.data) == 0:
            print("  tidak ada data yang direkam, tidak disimpan.")
            return ""
        folder = folder_kontrol(self.kontrol)
        kode_uji = angka_kode(self.sudut) if self.uji == UJI1 else angka_kode(config.UJI2_ZETA_DEG)
        nama = nama_berikutnya(folder, self.kontrol["kode"], kode_uji)
        simpan_latar(os.path.join(folder, nama), nama, self.kontrol["mode"], list(self.perekam.data),
                     self.perekam.jumlah_c, self.teks_catatan_gambar(hasil, sebab))
        self.perekam.data = []
        return nama

    def buat_ringkasan(self):
        folder = ise.folder_jenis(ise.JENIS_PENGUJIAN)
        os.makedirs(folder, exist_ok=True)
        nama = "%s_RINGKASAN UJI OTOMATIS_Simulasi.csv" % datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path_ringkasan = os.path.join(folder, nama)
        with open(self.path_ringkasan, "w", newline="") as f:
            csv.writer(f).writerow(KOLOM_RINGKASAN)

    def catat_ringkasan(self, hasil, sebab, nama):
        """Satu baris ringkasan per percobaan (juga yang gagal dan tidak disimpan)."""
        if not self.path_ringkasan:
            return
        uji1 = self.uji == UJI1
        baris = [datetime.now().strftime("%Y-%m-%d %H:%M:%S"), self.kontrol["label"], self.uji,
                 "%g" % self.sudut if uji1 else "", "%g" % config.BANDUL_MASSA_KG if uji1 else "",
                 "%g" % config.BANDUL_PANJANG_TALI_M if uji1 else "", "%g" % config.LAMA_DORONG if uji1 else "",
                 "%.4f" % self.gaya if uji1 else "", "%.4f" % impuls_bandul(self.sudut) if uji1 else "",
                 "%g" % (0.0 if uji1 else config.UJI2_ZETA_DEG), "%g" % self.psi_max_deg(), self.percobaan_ke, hasil,
                 sebab, "" if uji1 else self.ketukan, "%.3f" % math.degrees(self.psi_maks), "%.3f" % self.t_rekam,
                 nama or "-"]
        baris += ["%g" % float(self.params[nama_gain]) for nama_gain in KOLOM_RINGKASAN[len(baris):]]
        try:
            with open(self.path_ringkasan, "a", newline="") as f:
                csv.writer(f).writerow(baris)
        except OSError as kesalahan:
            print("Ringkasan tidak bisa ditulis: %s" % kesalahan)

    # -------------------------------------------------------------- terminal
    def cetak_awal(self):
        print("")
        print("=================== PENGUJIAN OTOMATIS SIMULASI (U lagi = batal) ===================")
        print("Kontrol : " + ", ".join(k["label"] for k in self.daftar_kontrol) + "  (mode DIAM, constraint %s)"
              % ise.teks_mask(self.mask))
        print("Gain    : " + " | ".join("%s %g" % (n, self.params[n])
                                       for n in ["Kp_psi", "Kd_psi", "Kd_theta", "Kvel"] + list(config.NAMA_ALPHA)))
        print("UJI 1   : bidang datar, bandul dipukul dari belakang. m_bd %g kg, l_bd %g m, lama tumbukan %g s"
              % (config.BANDUL_MASSA_KG, config.BANDUL_PANJANG_TALI_M, config.LAMA_DORONG))
        for sudut in self.daftar_sudut:
            print("          phi_0 %5.1f deg -> v_1 %.3f m/s, impuls %.3f N s, F %.2f N"
                  % (sudut, kecepatan_bandul(sudut), impuls_bandul(sudut), gaya_bandul(sudut)))
        print("UJI 2   : tanjakan %g deg, panjang %g m (datar bawah %g m, datar atas %g m), Kvel %g, zeta model "
              "CBF-QP %g deg" % (config.UJI2_ZETA_DEG, config.UJI2_PANJANG_MIRING_M, config.UJI2_DATAR_BAWAH_M,
                                 config.UJI2_DATAR_ATAS_M, self.params["Kvel"], config.UJI2_CBF_ZETA_DEG))
        print("Hasil   : %s/<kontrol>/" % ise.folder_jenis(ise.JENIS_PENGUJIAN))
        print("===================================================================================")

    def cetak_akhir(self, hasil, lama):
        print("")
        print("PENGUJIAN OTOMATIS %s dalam %.1f menit: berhasil %d, gagal %d."
              % (hasil, lama / 60.0, self.jumlah[BERHASIL], self.jumlah[GAGAL]))
        if self.path_ringkasan:
            print("Ringkasan: %s" % self.path_ringkasan)
        jumlah = simpan_berjalan()
        if jumlah > 0:
            print("(%d file masih disimpan di latar, tunggu sebentar sebelum membuka foldernya)" % jumlah)