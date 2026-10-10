"""tuning_optuna.py - mencari gain PD dan Alpha CBF-QP terbaik secara otomatis dengan Optuna (simulasi PyBullet).

Dua profil (config.OPTUNA_PROFIL), masing-masing dengan study sendiri supaya hasilnya tidak tercampur:
    TERUKUR   : torsi roda 0.238 N m/A (hasil uji delay),  dorong 11 N
    DATASHEET : torsi roda 0.75 N m/A (datasheet Waveshare), dorong 26 N   -> BELUM TERVERIFIKASI
Dua tahap per profil (dijalankan terpisah, disimpan di database OPTUNA/atera_optuna.db):
    tahap PD  : mencari Kp_psi, Kd_psi, Kp_theta, Kd_theta              (mode PD saja)    study ATERA_PD_<PROFIL>
    tahap CBF : gain PD dikunci ke hasil terbaik tahap PD profil yang sama, mencari Alpha yang dipakai DAFTAR_C
                (mode PD + CBF-QP)                                                        study ATERA_PD+CBF_<PROFIL>
Batas atas pencarian tiap variabel diatur di config.OPTUNA_BATAS_ATAS (tabel per variabel).

Satu trial = dua pengujian tanpa jendela (headless), di lantai config.OPTUNA_ZETA_DEG:
    1. robot diletakkan, Mode Balancing, didorong ke DEPAN  (gaya dorong profil), direkam DURASI_PENGUJIAN s
    2. robot diletakkan ulang, didorong ke BELAKANG, direkam DURASI_PENGUJIAN s
Skor = ISE ternormalisasi + penalti (rumus di config.py bagian 12). Makin kecil makin baik.
Trial dihentikan lebih awal bila robot jatuh / lari, atau dipangkas (pruning) bila skornya sudah jelek.

Jalankan (di folder simulasi):
    python3 tuning_optuna.py --profil TERUKUR --tahap PD            tahap 1, config.OPTUNA_TRIAL_PD trial
    python3 tuning_optuna.py --profil TERUKUR --tahap CBF           tahap 2, config.OPTUNA_TRIAL_CBF trial
    python3 tuning_optuna.py --profil DATASHEET --tahap PD          profil datasheet (belum terverifikasi)
    python3 tuning_optuna.py --profil TERUKUR --tahap PD --trial 50 jumlah trial lain (MENAMBAH trial ke study yang ada)
    python3 tuning_optuna.py --hasil                                cetak hasil terbaik semua profil dan tahap, buat ulang CSV + PNG
    python3 tuning_optuna.py --hasil --profil TERUKUR               hanya satu profil
    python3 tuning_optuna.py --profil TERUKUR --hapus PD            hapus study tahap PD profil TERUKUR (mulai dari nol)

Hasil (folder OPTUNA):
    atera_optuna.db                              database semua trial (bisa dilanjutkan kapan saja)
    YYYYMMDD_HHmmSS_OPTUNA_PD_TERUKUR.csv / .png         riwayat trial + respon gain terbaik di zeta 0 dan 5.2 deg
    YYYYMMDD_HHmmSS_OPTUNA_PD+CBF_TERUKUR.csv / .png
    (untuk profil DATASHEET: ..._DATASHEET, diberi label BELUM TERVERIFIKASI)
Nilai terbaik dicetak di terminal dalam format config.py. Salin sendiri ke config.py.

Butuh: pip install optuna
Nama file sengaja BUKAN optuna.py: file bernama optuna.py akan menutupi library optuna saat "import optuna".
"""

import argparse
import csv
import math
import os
import sys
import time
from datetime import datetime

import numpy as np

try:
    import optuna
except ImportError:
    print("Library optuna belum terpasang:  pip install optuna")
    sys.exit(1)

import atera_main
import config
import simulasi
from pd_control import PDControl

FOLDER_INI = os.path.dirname(os.path.abspath(__file__))
FOLDER_HASIL = os.path.join(FOLDER_INI, "OPTUNA")
DATABASE = "sqlite:///" + os.path.join(FOLDER_HASIL, "atera_optuna.db")
AWAL_NAMA_STUDY = {"PD": "ATERA_PD", "CBF": "ATERA_PD+CBF"}
AWAL_NAMA_FILE = {"PD": "OPTUNA_PD", "CBF": "OPTUNA_PD+CBF"}
NAMA_GAIN_PD = ["Kp_psi", "Kd_psi", "Kp_theta", "Kd_theta"]
ARAH_DORONG = [1, -1]                       # depan, belakang
TEKS_ARAH = {1: "depan", -1: "belakang"}


def nama_study(tahap, profil):
    return AWAL_NAMA_STUDY[tahap] + "_" + profil          # ATERA_PD_TERUKUR, ATERA_PD+CBF_DATASHEET, ...


def nama_file(tahap, profil):
    return AWAL_NAMA_FILE[tahap] + "_" + profil


def batas_atas(nama):
    """Batas atas pencarian satu variabel dari config.OPTUNA_BATAS_ATAS."""
    if nama in config.OPTUNA_BATAS_ATAS:
        return float(config.OPTUNA_BATAS_ATAS[nama])
    return float(config.OPTUNA_BATAS_ATAS_LAINNYA)


def batas_bawah(nama, tahap):
    """Batas bawah: Alpha tidak boleh 0 (CBF-QP jadi tidak aktif), jadi minimal satu langkah."""
    if tahap == "CBF":
        return max(float(config.OPTUNA_BATAS_BAWAH), float(config.OPTUNA_LANGKAH))
    return float(config.OPTUNA_BATAS_BAWAH)


def teks_label(profil):
    """Label profil untuk terminal, judul PNG, dan atribut study."""
    data = config.OPTUNA_PROFIL[profil]
    teks = "profil %s (torsi roda %.3f N m/A, dorong %.1f N)" % (profil, data["kt_efektif"], data["gaya_dorong"])
    if not data["terverifikasi"]:
        teks = teks + " - BELUM TERVERIFIKASI"
    return teks


# =============================================================================
# PENGUJI: satu simulasi headless dipakai ulang untuk semua trial
# =============================================================================
class Penguji:
    def __init__(self, pakai_cbf, profil):
        # PyBullet hanya boleh satu dunia per program, jadi Penguji cukup dibuat SATU kali.
        self.sim = simulasi.Simulasi(gui=False)
        self.pd = PDControl()
        self.cbf = None
        self.pakai_cbf = pakai_cbf          # False: mode PD saja (CBF-QP yang sudah dibuat tidak dipakai)
        self.nama_alpha = []
        if pakai_cbf:
            print("Menyiapkan CBF-QP (acados), tunggu beberapa detik ...")
            from cbf_qp import CBFQP
            self.cbf = CBFQP()
            for i in self.cbf.alpha_dipakai:
                self.nama_alpha.append(self.cbf.nama_alpha[i])
        self.zeta_deg = None
        self.profil = None
        self.gaya_dorong = float(simulasi.GAYA_DORONG_AWAL)
        self.atur_profil(profil)

    def atur_profil(self, profil):
        """Pasang torsi roda nyata dan gaya dorong sesuai config.OPTUNA_PROFIL (hanya untuk simulasi ini)."""
        data = config.OPTUNA_PROFIL[profil]
        self.profil = profil
        self.sim.skala_torsi = float(data["kt_efektif"]) / config.MOTOR_KT      # torsi nyata / torsi perintah
        self.gaya_dorong = float(data["gaya_dorong"])

    def atur_zeta(self, zeta_deg):
        if self.zeta_deg == zeta_deg:
            return
        self.zeta_deg = zeta_deg
        config.zeta_DEG = float(zeta_deg)
        config.zeta = math.radians(zeta_deg)           # dipakai model.py dan cbf_qp.py
        self.sim.ubah_zeta(config.zeta)

    def atur_gain(self, gain):
        """gain: dict nama -> nilai. Gain PD ke PDControl, Alpha ke config (dibaca cbf_qp.py tiap siklus)."""
        for nama in gain:
            if nama in NAMA_GAIN_PD:
                setattr(self.pd, nama, float(gain[nama]))
            else:
                setattr(config, nama, float(gain[nama]))

    def jalankan(self, gain, zeta_deg, trial=None, rekam=False):
        """Jalankan dua pengujian (dorong depan, lalu belakang).

        Mengembalikan (skor, keterangan, rekaman). rekaman berisi t, psi [deg], s [m] tiap pengujian (rekam=True).
        Melempar optuna.TrialPruned bila trial dipangkas.
        """
        self.atur_zeta(zeta_deg)
        self.atur_gain(gain)

        dt = config.DT
        langkah_sebelum = int(round(config.OPTUNA_WAKTU_SEBELUM_DORONG / dt))
        langkah_uji = int(round(atera_main.DURASI_PENGUJIAN / dt))
        langkah_per_detik = int(round(1.0 / dt))
        waktu_total = len(ARAH_DORONG) * atera_main.DURASI_PENGUJIAN
        batas_jatuh = math.radians(config.SAFE_TILT_DEG)

        skor = 0.0
        laporan_ke = 0
        rekaman = []
        for nomor in range(len(ARAH_DORONG)):
            arah = ARAH_DORONG[nomor]
            # Robot diletakkan ulang, Mode Balancing, posisi tahan di s = 0.
            self.sim.reset()
            self.sim.acak = np.random.default_rng(config.OPTUNA_SEED + nomor)    # noise IMU sama tiap trial
            self.sim.gaya_dorong = self.gaya_dorong
            self.pd.theta_setpoint = float(config.theta_setpoint)
            self.pd.set_command(0, True)
            data = {"arah": arah, "t": [], "psi": [], "s": []}

            for k in range(langkah_sebelum + langkah_uji):
                if k == langkah_sebelum:
                    self.sim.dorong(arah)
                x = self.sim.baca_state()
                u_PD = self.pd.compute_PD(x)
                if self.cbf is None or not self.pakai_cbf:
                    u = max(config.u_min, min(config.u_max, u_PD))
                    status = "OK"
                else:
                    u = self.cbf.compute(x, u_PD)
                    status = self.cbf.status
                self.sim.step(u)

                # Dinilai dari state SEBENARNYA (bukan hasil sensor).
                psi = float(self.sim.x_benar[2])
                s = config.R * float(self.sim.x_benar[0])
                t_uji = (k - langkah_sebelum) * dt
                if rekam:
                    data["t"].append(t_uji)
                    data["psi"].append(math.degrees(psi))
                    data["s"].append(s)

                if abs(psi) > batas_jatuh or abs(s) > config.OPTUNA_S_HENTI:
                    sudah = nomor * atera_main.DURASI_PENGUJIAN + max(t_uji, 0.0)
                    sisa = waktu_total - sudah
                    skor = skor + config.OPTUNA_PENALTI_JATUH * (1.0 + sisa / waktu_total)
                    if abs(psi) > batas_jatuh:
                        keterangan = "JATUH (dorong %s, t %.1f s)" % (TEKS_ARAH[arah], t_uji)
                    else:
                        keterangan = "LARI |s| > %.1f m (dorong %s, t %.1f s)" % (config.OPTUNA_S_HENTI,
                                                                                TEKS_ARAH[arah], t_uji)
                    rekaman.append(data)
                    return skor, keterangan, rekaman

                if k < langkah_sebelum:
                    continue
                e_psi = (psi - config.psi_setpoint) / config.psi_max
                e_s = (s - config.R * config.theta_setpoint) / config.s_max
                skor = skor + (e_psi * e_psi + e_s * e_s) * dt
                if s > config.s_max or s < config.s_min:
                    skor = skor + config.OPTUNA_PENALTI_JARAK * dt
                if status != "OK":
                    skor = skor + config.OPTUNA_PENALTI_INFEASIBLE * dt

                # Laporan tiap 1 detik untuk pruning.
                if trial is not None and (k - langkah_sebelum + 1) % langkah_per_detik == 0:
                    trial.report(skor, laporan_ke)
                    laporan_ke = laporan_ke + 1
                    if trial.should_prune():
                        raise optuna.TrialPruned()
            rekaman.append(data)
        return skor, "selesai", rekaman


# =============================================================================
# STUDY
# =============================================================================
def buat_study(tahap, profil):
    os.makedirs(FOLDER_HASIL, exist_ok=True)
    sampler = optuna.samplers.TPESampler(seed=config.OPTUNA_SEED)
    pruner = optuna.pruners.MedianPruner(n_startup_trials=config.OPTUNA_PRUNER_STARTUP,
                                         n_warmup_steps=config.OPTUNA_PRUNER_WARMUP_S)
    study = optuna.create_study(study_name=nama_study(tahap, profil), storage=DATABASE, direction="minimize",
                                sampler=sampler, pruner=pruner, load_if_exists=True)
    periksa_profil_study(study, profil)
    return study


def periksa_profil_study(study, profil):
    """Catat profil di atribut study (tampil di Optuna dashboard). Bila study lama dibuat dengan torsi / gaya
    dorong yang berbeda dari config.OPTUNA_PROFIL sekarang, berhenti: trial lama dan baru tidak boleh dicampur."""
    data = config.OPTUNA_PROFIL[profil]
    atribut = study.user_attrs
    if "kt_efektif" in atribut:
        beda = (abs(atribut["kt_efektif"] - data["kt_efektif"]) > 1e-9
                or abs(atribut["gaya_dorong"] - data["gaya_dorong"]) > 1e-9)
        if beda:
            print("PROFIL %s di config.py berubah sejak study %s dibuat:" % (profil, study.study_name))
            print("    study : torsi roda %.3f N m/A, dorong %.1f N" % (atribut["kt_efektif"], atribut["gaya_dorong"]))
            print("    config: torsi roda %.3f N m/A, dorong %.1f N" % (data["kt_efektif"], data["gaya_dorong"]))
            print("Trial lama dan baru tidak boleh dicampur. Kembalikan nilai config.py, atau hapus study:")
            print("    python3 tuning_optuna.py --profil %s --hapus PD     (atau CBF)" % profil)
            sys.exit(1)
    study.set_user_attr("profil", profil)
    study.set_user_attr("kt_efektif", float(data["kt_efektif"]))
    study.set_user_attr("gaya_dorong", float(data["gaya_dorong"]))
    study.set_user_attr("terverifikasi", bool(data["terverifikasi"]))
    study.set_user_attr("catatan", data["catatan"])
    if not data["terverifikasi"]:
        study.set_user_attr("PERINGATAN", "BELUM TERVERIFIKASI")


def ada_study(tahap, profil):
    if not os.path.exists(os.path.join(FOLDER_HASIL, "atera_optuna.db")):
        return False
    for ringkasan in optuna.get_all_study_summaries(storage=DATABASE):
        if ringkasan.study_name == nama_study(tahap, profil):
            return True
    return False


def trial_selesai(study):
    daftar = []
    for t in study.trials:
        if t.state == optuna.trial.TrialState.COMPLETE:
            daftar.append(t)
    return daftar


def gain_pd_terbaik(profil):
    """Gain PD terbaik dari tahap PD profil yang sama. Bila tahap PD belum ada, pakai gain di config.py."""
    if ada_study("PD", profil):
        study = buat_study("PD", profil)
        if len(trial_selesai(study)) > 0:
            gain = {}
            for nama in NAMA_GAIN_PD:
                gain[nama] = study.best_trial.params[nama]
            return gain, "hasil terbaik tahap PD profil %s (trial %d)" % (profil, study.best_trial.number)
    gain = {}
    for nama in NAMA_GAIN_PD:
        gain[nama] = float(getattr(config, nama))
    return gain, "config.py (tahap PD belum dijalankan)"


def gain_dari_trial(tahap, params, gain_pd):
    gain = {}
    if tahap == "CBF":
        for nama in gain_pd:
            gain[nama] = gain_pd[nama]
    for nama in params:
        gain[nama] = params[nama]
    return gain


def jalankan_tahap(tahap, profil, jumlah_trial):
    penguji = Penguji(pakai_cbf=(tahap == "CBF"), profil=profil)
    study = buat_study(tahap, profil)

    gain_pd = {}
    if tahap == "PD":
        nama_dicari = list(NAMA_GAIN_PD)
    else:
        gain_pd, sumber = gain_pd_terbaik(profil)
        print("Gain PD dikunci dari %s:" % sumber)
        for nama in NAMA_GAIN_PD:
            print("    %s = %.4f" % (nama, gain_pd[nama]))
        nama_dicari = list(penguji.nama_alpha)
        if len(nama_dicari) == 0:
            print("Tidak ada Alpha yang dipakai DAFTAR_C di cbf_qp.py.")
            return

    # Rentang tiap variabel (batas atas dari config.OPTUNA_BATAS_ATAS).
    bawah = {}
    atas = {}
    for nama in nama_dicari:
        bawah[nama] = batas_bawah(nama, tahap)
        atas[nama] = batas_atas(nama)
        if atas[nama] <= bawah[nama]:
            print("Batas atas %s (%.4f) harus lebih besar dari batas bawah (%.4f). Periksa config.OPTUNA_BATAS_ATAS."
                  % (nama, atas[nama], bawah[nama]))
            return

    # Trial pertama = nilai di config.py sekarang (titik awal pembanding), hanya bila study masih kosong.
    if len(study.trials) == 0:
        awal = {}
        for nama in nama_dicari:
            nilai = float(getattr(config, nama))
            awal[nama] = min(atas[nama], max(bawah[nama], round(nilai, 4)))
        study.enqueue_trial(awal)

    print("=" * 78)
    print("STUDY %s | TAHAP %s" % (study.study_name, tahap))
    print(teks_label(profil))
    if not config.OPTUNA_PROFIL[profil]["terverifikasi"]:
        print("!! " + config.OPTUNA_PROFIL[profil]["catatan"])
    print("Rentang pencarian (langkah %.4f):" % config.OPTUNA_LANGKAH)
    for nama in nama_dicari:
        print("    %-9s %10.4f .. %10.4f" % (nama, bawah[nama], atas[nama]))
    print("Lantai %.1f deg | dorong %.1f N depan + belakang | %.0f s per pengujian | %d trial (sudah ada %d)"
          % (config.OPTUNA_ZETA_DEG, penguji.gaya_dorong, atera_main.DURASI_PENGUJIAN, jumlah_trial,
             len(study.trials)))
    print("Ctrl+C untuk berhenti: trial yang sudah selesai tetap tersimpan dan bisa dilanjutkan.")
    print("=" * 78 + "\n")

    def objektif(trial):
        params = {}
        for nama in nama_dicari:
            params[nama] = trial.suggest_float(nama, bawah[nama], atas[nama], step=config.OPTUNA_LANGKAH)
        gain = gain_dari_trial(tahap, params, gain_pd)
        t0 = time.time()
        try:
            skor, keterangan, _ = penguji.jalankan(gain, config.OPTUNA_ZETA_DEG, trial)
        except optuna.TrialPruned:
            trial.set_user_attr("keterangan", "DIPANGKAS")
            trial.set_user_attr("lama_s", time.time() - t0)
            raise
        trial.set_user_attr("keterangan", keterangan)
        trial.set_user_attr("lama_s", time.time() - t0)
        return skor

    def cetak_trial(study, trial):
        keterangan = trial.user_attrs.get("keterangan", str(trial.state))
        lama = trial.user_attrs.get("lama_s", 0.0)
        if trial.value is not None:
            teks_skor = "skor %10.4f" % trial.value
        else:
            teks_skor = "skor %10s" % "-"
        selesai = trial_selesai(study)
        if len(selesai) > 0:
            teks_terbaik = "terbaik %.4f (trial %d)" % (study.best_value, study.best_trial.number)
        else:
            teks_terbaik = "terbaik -"
        print("trial %4d | %s | %-38s | %5.1f s | %s" % (trial.number, teks_skor, keterangan, lama, teks_terbaik))

    optuna.logging.set_verbosity(optuna.logging.WARNING)
    try:
        study.optimize(objektif, n_trials=jumlah_trial, callbacks=[cetak_trial])
    except KeyboardInterrupt:
        print("\nDihentikan dengan Ctrl+C.")
    laporkan(tahap, profil, penguji)


# =============================================================================
# LAPORAN: CSV, PNG, nilai untuk config.py
# =============================================================================
def laporkan(tahap, profil, penguji):
    if not ada_study(tahap, profil):
        print("Tahap %s profil %s belum pernah dijalankan." % (tahap, profil))
        return
    study = buat_study(tahap, profil)
    selesai = trial_selesai(study)
    if len(selesai) == 0:
        print("Tahap %s profil %s: belum ada trial yang selesai." % (tahap, profil))
        return
    penguji.pakai_cbf = (tahap == "CBF")
    penguji.atur_profil(profil)

    terbaik = study.best_trial
    gain_pd = {}
    if tahap == "CBF":
        gain_pd, _ = gain_pd_terbaik(profil)
    gain = gain_dari_trial(tahap, terbaik.params, gain_pd)

    jumlah_pangkas = 0
    for t in study.trials:
        if t.state == optuna.trial.TrialState.PRUNED:
            jumlah_pangkas = jumlah_pangkas + 1

    # Gain terbaik diuji lagi (direkam) di lantai optimasi dan lantai uji.
    hasil_uji = []
    for zeta_deg in (config.OPTUNA_ZETA_DEG, config.OPTUNA_ZETA_UJI_DEG):
        skor, keterangan, rekaman = penguji.jalankan(gain, zeta_deg, rekam=True)
        hasil_uji.append((zeta_deg, skor, keterangan, rekaman))

    print("\n================ HASIL TAHAP %s | %s ================" % (tahap, study.study_name))
    print(teks_label(profil))
    print("trial: %d total, %d selesai, %d dipangkas" % (len(study.trials), len(selesai), jumlah_pangkas))
    print("trial terbaik: %d, skor %.4f (%s)" % (terbaik.number, terbaik.value,
                                                terbaik.user_attrs.get("keterangan", "-")))
    for zeta_deg, skor, keterangan, _ in hasil_uji:
        print("  uji lantai %4.1f deg: skor %10.4f  (%s)" % (zeta_deg, skor, keterangan))
    print("\nSalin ke config.py:")
    if tahap == "CBF":
        print("# gain PD (dari tahap PD)")
    for nama in NAMA_GAIN_PD:
        print("%s = %.4f" % (nama, gain[nama]))
    if tahap == "CBF":
        for nama in terbaik.params:
            print("%s = %.4f" % (nama, terbaik.params[nama]))
    print("=================================================\n")

    awalan = os.path.join(FOLDER_HASIL, datetime.now().strftime("%Y%m%d_%H%M%S") + "_" + nama_file(tahap, profil))
    simpan_csv(awalan + ".csv", study, profil)
    simpan_png(awalan + ".png", tahap, profil, study, gain, hasil_uji, penguji.gaya_dorong)


def simpan_csv(path, study, profil):
    nama_param = []
    for t in study.trials:
        for nama in t.params:
            if nama not in nama_param:
                nama_param.append(nama)
    with open(path, "w", newline="") as f:
        penulis = csv.writer(f)
        penulis.writerow(["trial", "status", "skor", "keterangan", "lama_s", "profil", "terverifikasi"] + nama_param)
        for t in study.trials:
            if t.value is not None:
                skor = "%.6f" % t.value
            else:
                skor = ""
            baris = [t.number, t.state.name, skor, t.user_attrs.get("keterangan", ""),
                     "%.1f" % t.user_attrs.get("lama_s", 0.0), profil,
                     "YA" if config.OPTUNA_PROFIL[profil]["terverifikasi"] else "BELUM"]
            for nama in nama_param:
                if nama in t.params:
                    baris.append("%.4f" % t.params[nama])
                else:
                    baris.append("")
            penulis.writerow(baris)
    print("Tersimpan:", path)


def simpan_png(path, tahap, profil, study, gain, hasil_uji, gaya_dorong):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib tidak terpasang -> PNG tidak dibuat")
        return

    fig, ax = plt.subplots(3, 1, figsize=(11, 11))

    # 1. Riwayat skor tiap trial
    nomor_selesai = []
    skor_selesai = []
    nomor_terbaik = []
    skor_terbaik = []
    terbaik = math.inf
    nomor_pangkas = []
    for t in study.trials:
        if t.state == optuna.trial.TrialState.COMPLETE:
            nomor_selesai.append(t.number)
            skor_selesai.append(t.value)
            if t.value < terbaik:
                terbaik = t.value
            nomor_terbaik.append(t.number)
            skor_terbaik.append(terbaik)
        elif t.state == optuna.trial.TrialState.PRUNED:
            nomor_pangkas.append(t.number)
    ax[0].plot(nomor_selesai, skor_selesai, "o", ms=3, color="tab:blue", label="trial selesai")
    ax[0].step(nomor_terbaik, skor_terbaik, where="post", color="red", label="terbaik sejauh ini")
    for i in range(len(nomor_pangkas)):
        if i == 0:
            ax[0].axvline(nomor_pangkas[i], color="0.85", lw=0.8, zorder=0, label="dipangkas")
        else:
            ax[0].axvline(nomor_pangkas[i], color="0.85", lw=0.8, zorder=0)
    ax[0].set_yscale("log")
    ax[0].set_xlabel("trial")
    ax[0].set_ylabel("skor (log)")
    ax[0].set_title("Riwayat optimasi tahap %s | %s" % (tahap, teks_label(profil)), fontsize=10,
                    color="black" if config.OPTUNA_PROFIL[profil]["terverifikasi"] else "red")
    ax[0].grid(True)
    ax[0].legend(fontsize=8)

    # 2 dan 3. Respon gain terbaik
    gaya_garis = {1: "-", -1: "--"}
    warna = ["tab:blue", "tab:orange"]
    for i in range(len(hasil_uji)):
        zeta_deg, skor, keterangan, rekaman = hasil_uji[i]
        for data in rekaman:
            label = "zeta %.1f deg, dorong %s" % (zeta_deg, TEKS_ARAH[data["arah"]])
            ax[1].plot(data["t"], data["psi"], gaya_garis[data["arah"]], color=warna[i], lw=1, label=label)
            ax[2].plot(data["t"], data["s"], gaya_garis[data["arah"]], color=warna[i], lw=1, label=label)
    batas_psi = math.degrees(config.psi_max)
    ax[1].axhline(batas_psi, color="red", lw=2.2)
    ax[1].axhline(-batas_psi, color="red", lw=2.2)
    ax[2].axhline(config.s_max, color="red", lw=2.2)
    ax[2].axhline(config.s_min, color="red", lw=2.2)
    ax[1].set_ylabel("psi sebenarnya [deg]")
    ax[2].set_ylabel("s sebenarnya [m]")
    for a in (ax[1], ax[2]):
        a.set_xlabel("t sejak dorongan [s]")
        a.axvline(0.0, color="black", lw=0.8)
        a.grid(True)
        a.legend(fontsize=8)
    teks_gain = ""
    nomor = 0
    for nama in gain:
        if nomor == 4:
            teks_gain = teks_gain + "\n"          # Alpha di baris kedua
        elif nomor > 0:
            teks_gain = teks_gain + ", "
        teks_gain = teks_gain + "%s=%.4f" % (nama, gain[nama])
        nomor = nomor + 1
    ax[1].set_title("Respon gain terbaik (dorong %.1f N): %s" % (gaya_dorong, teks_gain), fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)
    print("Tersimpan:", path)


def hapus_tahap(tahap, profil):
    if not ada_study(tahap, profil):
        print("Tahap %s profil %s belum ada di database." % (tahap, profil))
        return
    jawaban = input("Hapus SEMUA trial study %s dari database? ketik YA: " % nama_study(tahap, profil))
    if jawaban.strip() == "YA":
        optuna.delete_study(study_name=nama_study(tahap, profil), storage=DATABASE)
        print("Study %s dihapus." % nama_study(tahap, profil))
    else:
        print("Batal.")


def main():
    daftar_profil = list(config.OPTUNA_PROFIL.keys())
    parser = argparse.ArgumentParser(description="Tuning gain PD dan Alpha CBF-QP ATERA dengan Optuna")
    parser.add_argument("--profil", choices=daftar_profil,
                        help="profil torsi roda + gaya dorong (config.OPTUNA_PROFIL); wajib untuk --tahap dan --hapus")
    parser.add_argument("--tahap", choices=["PD", "CBF"], help="PD = gain PD, CBF = Alpha (gain PD dikunci)")
    parser.add_argument("--trial", type=int, default=None, help="jumlah trial (default dari config.py)")
    parser.add_argument("--hasil", action="store_true",
                        help="cetak hasil terbaik, buat ulang CSV + PNG (tanpa --profil: semua profil)")
    parser.add_argument("--hapus", choices=["PD", "CBF"], help="hapus study satu tahap dari satu profil")
    args = parser.parse_args()

    if args.hapus:
        if args.profil is None:
            parser.error("--hapus butuh --profil")
        hapus_tahap(args.hapus, args.profil)
    elif args.hasil:
        if args.profil is None:
            profil_dilaporkan = daftar_profil
        else:
            profil_dilaporkan = [args.profil]
        ada_cbf = False
        ada_apa_saja = False
        for profil in profil_dilaporkan:
            if ada_study("CBF", profil):
                ada_cbf = True
            if ada_study("PD", profil) or ada_study("CBF", profil):
                ada_apa_saja = True
        if not ada_apa_saja:
            print("Belum ada study untuk profil yang diminta.")
            return
        penguji = Penguji(pakai_cbf=ada_cbf, profil=profil_dilaporkan[0])
        for profil in profil_dilaporkan:
            laporkan("PD", profil, penguji)
            laporkan("CBF", profil, penguji)
    elif args.tahap:
        if args.profil is None:
            parser.error("--tahap butuh --profil (%s)" % " / ".join(daftar_profil))
        if args.trial is not None:
            jumlah = args.trial
        elif args.tahap == "PD":
            jumlah = config.OPTUNA_TRIAL_PD
        else:
            jumlah = config.OPTUNA_TRIAL_CBF
        jalankan_tahap(args.tahap, args.profil, jumlah)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()