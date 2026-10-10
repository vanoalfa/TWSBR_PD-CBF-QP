"""tuning_optuna.py - mencari gain PD dan Alpha CBF-QP terbaik secara otomatis dengan Optuna (simulasi PyBullet).

Dua tahap (dijalankan terpisah, hasilnya disimpan di database OPTUNA/atera_optuna.db):
    tahap PD  : mencari Kp_psi, Kd_psi, Kp_theta, Kd_theta              (mode PD saja)
    tahap CBF : gain PD dikunci ke hasil terbaik tahap PD, mencari Alpha yang dipakai DAFTAR_C (mode PD + CBF-QP)

Satu trial = dua pengujian tanpa jendela (headless), di lantai config.OPTUNA_ZETA_DEG:
    1. robot diletakkan, Mode Balancing, didorong ke DEPAN  (GAYA_DORONG_AWAL N), direkam DURASI_PENGUJIAN s
    2. robot diletakkan ulang, didorong ke BELAKANG, direkam DURASI_PENGUJIAN s
Skor = ISE ternormalisasi + penalti (rumus di config.py bagian 12). Makin kecil makin baik.
Trial dihentikan lebih awal bila robot jatuh / lari, atau dipangkas (pruning) bila skornya sudah jelek.

Jalankan (di folder simulasi):
    python3 tuning_optuna.py --tahap PD                 tahap 1, config.OPTUNA_TRIAL_PD trial
    python3 tuning_optuna.py --tahap CBF                tahap 2, config.OPTUNA_TRIAL_CBF trial
    python3 tuning_optuna.py --tahap PD --trial 50      jumlah trial lain (MENAMBAH trial ke database yang sudah ada)
    python3 tuning_optuna.py --hasil                    cetak hasil terbaik kedua tahap, buat ulang CSV + PNG
    python3 tuning_optuna.py --hapus PD                 hapus hasil tahap PD dari database (mulai dari nol)

Hasil (folder OPTUNA):
    atera_optuna.db                              database semua trial (bisa dilanjutkan kapan saja)
    YYYYMMDD_HHmmSS_OPTUNA_PD.csv / .png         riwayat trial + respon gain terbaik di zeta 0 dan 5.2 deg
    YYYYMMDD_HHmmSS_OPTUNA_PD+CBF.csv / .png
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
NAMA_STUDY = {"PD": "ATERA_PD", "CBF": "ATERA_PD+CBF"}
NAMA_FILE = {"PD": "OPTUNA_PD", "CBF": "OPTUNA_PD+CBF"}
NAMA_GAIN_PD = ["Kp_psi", "Kd_psi", "Kp_theta", "Kd_theta"]
ARAH_DORONG = [1, -1]                       # depan, belakang
TEKS_ARAH = {1: "depan", -1: "belakang"}


# =============================================================================
# PENGUJI: satu simulasi headless dipakai ulang untuk semua trial
# =============================================================================
class Penguji:
    def __init__(self, pakai_cbf):
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
            self.sim.gaya_dorong = float(simulasi.GAYA_DORONG_AWAL)
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
def buat_study(tahap):
    os.makedirs(FOLDER_HASIL, exist_ok=True)
    sampler = optuna.samplers.TPESampler(seed=config.OPTUNA_SEED)
    pruner = optuna.pruners.MedianPruner(n_startup_trials=config.OPTUNA_PRUNER_STARTUP,
                                         n_warmup_steps=config.OPTUNA_PRUNER_WARMUP_S)
    return optuna.create_study(study_name=NAMA_STUDY[tahap], storage=DATABASE, direction="minimize",
                               sampler=sampler, pruner=pruner, load_if_exists=True)


def ada_study(tahap):
    if not os.path.exists(os.path.join(FOLDER_HASIL, "atera_optuna.db")):
        return False
    for ringkasan in optuna.get_all_study_summaries(storage=DATABASE):
        if ringkasan.study_name == NAMA_STUDY[tahap]:
            return True
    return False


def trial_selesai(study):
    daftar = []
    for t in study.trials:
        if t.state == optuna.trial.TrialState.COMPLETE:
            daftar.append(t)
    return daftar


def gain_pd_terbaik():
    """Gain PD terbaik dari tahap PD. Bila tahap PD belum ada, pakai gain di config.py."""
    if ada_study("PD"):
        study = buat_study("PD")
        if len(trial_selesai(study)) > 0:
            gain = {}
            for nama in NAMA_GAIN_PD:
                gain[nama] = study.best_trial.params[nama]
            return gain, "hasil terbaik tahap PD (trial %d)" % study.best_trial.number
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


def jalankan_tahap(tahap, jumlah_trial):
    penguji = Penguji(pakai_cbf=(tahap == "CBF"))
    study = buat_study(tahap)

    gain_pd = {}
    if tahap == "PD":
        nama_dicari = list(NAMA_GAIN_PD)
        bawah = config.OPTUNA_BATAS_BAWAH
    else:
        gain_pd, sumber = gain_pd_terbaik()
        print("Gain PD dikunci dari %s:" % sumber)
        for nama in NAMA_GAIN_PD:
            print("    %s = %.4f" % (nama, gain_pd[nama]))
        nama_dicari = list(penguji.nama_alpha)
        bawah = max(config.OPTUNA_BATAS_BAWAH, config.OPTUNA_LANGKAH)
        if len(nama_dicari) == 0:
            print("Tidak ada Alpha yang dipakai DAFTAR_C di cbf_qp.py.")
            return

    # Trial pertama = nilai di config.py sekarang (titik awal pembanding), hanya bila study masih kosong.
    if len(study.trials) == 0:
        awal = {}
        for nama in nama_dicari:
            nilai = float(getattr(config, nama))
            awal[nama] = min(config.OPTUNA_BATAS_ATAS, max(bawah, round(nilai, 4)))
        study.enqueue_trial(awal)

    print("TAHAP %s | dicari: %s | rentang %.4f .. %.4f (langkah %.4f)"
          % (tahap, ", ".join(nama_dicari), bawah, config.OPTUNA_BATAS_ATAS, config.OPTUNA_LANGKAH))
    print("Lantai %.1f deg | dorong %.1f N depan + belakang | %.0f s per pengujian | %d trial (sudah ada %d)"
          % (config.OPTUNA_ZETA_DEG, simulasi.GAYA_DORONG_AWAL, atera_main.DURASI_PENGUJIAN, jumlah_trial,
             len(study.trials)))
    print("Ctrl+C untuk berhenti: trial yang sudah selesai tetap tersimpan dan bisa dilanjutkan.\n")

    def objektif(trial):
        params = {}
        for nama in nama_dicari:
            params[nama] = trial.suggest_float(nama, bawah, config.OPTUNA_BATAS_ATAS, step=config.OPTUNA_LANGKAH)
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
    laporkan(tahap, penguji)


# =============================================================================
# LAPORAN: CSV, PNG, nilai untuk config.py
# =============================================================================
def laporkan(tahap, penguji):
    if not ada_study(tahap):
        print("Tahap %s belum pernah dijalankan." % tahap)
        return
    study = buat_study(tahap)
    selesai = trial_selesai(study)
    if len(selesai) == 0:
        print("Tahap %s: belum ada trial yang selesai." % tahap)
        return
    penguji.pakai_cbf = (tahap == "CBF")

    terbaik = study.best_trial
    gain_pd = {}
    if tahap == "CBF":
        gain_pd, _ = gain_pd_terbaik()
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

    print("\n================ HASIL TAHAP %s ================" % tahap)
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

    awalan = os.path.join(FOLDER_HASIL, datetime.now().strftime("%Y%m%d_%H%M%S") + "_" + NAMA_FILE[tahap])
    simpan_csv(awalan + ".csv", study)
    simpan_png(awalan + ".png", tahap, study, gain, hasil_uji)


def simpan_csv(path, study):
    nama_param = []
    for t in study.trials:
        for nama in t.params:
            if nama not in nama_param:
                nama_param.append(nama)
    with open(path, "w", newline="") as f:
        penulis = csv.writer(f)
        penulis.writerow(["trial", "status", "skor", "keterangan", "lama_s"] + nama_param)
        for t in study.trials:
            if t.value is not None:
                skor = "%.6f" % t.value
            else:
                skor = ""
            baris = [t.number, t.state.name, skor, t.user_attrs.get("keterangan", ""),
                     "%.1f" % t.user_attrs.get("lama_s", 0.0)]
            for nama in nama_param:
                if nama in t.params:
                    baris.append("%.4f" % t.params[nama])
                else:
                    baris.append("")
            penulis.writerow(baris)
    print("Tersimpan:", path)


def simpan_png(path, tahap, study, gain, hasil_uji):
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
    ax[0].set_title("Riwayat optimasi tahap %s" % tahap, fontsize=10)
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
    ax[1].set_title("Respon gain terbaik (dorong %.1f N): %s" % (simulasi.GAYA_DORONG_AWAL, teks_gain), fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)
    print("Tersimpan:", path)


def hapus_tahap(tahap):
    if not ada_study(tahap):
        print("Tahap %s belum ada di database." % tahap)
        return
    jawaban = input("Hapus SEMUA trial tahap %s dari database? ketik YA: " % tahap)
    if jawaban.strip() == "YA":
        optuna.delete_study(study_name=NAMA_STUDY[tahap], storage=DATABASE)
        print("Tahap %s dihapus." % tahap)
    else:
        print("Batal.")


def main():
    parser = argparse.ArgumentParser(description="Tuning gain PD dan Alpha CBF-QP ATERA dengan Optuna")
    parser.add_argument("--tahap", choices=["PD", "CBF"], help="PD = gain PD, CBF = Alpha (gain PD dikunci)")
    parser.add_argument("--trial", type=int, default=None, help="jumlah trial (default dari config.py)")
    parser.add_argument("--hasil", action="store_true", help="cetak hasil terbaik, buat ulang CSV + PNG")
    parser.add_argument("--hapus", choices=["PD", "CBF"], help="hapus hasil satu tahap dari database")
    args = parser.parse_args()

    if args.hapus:
        hapus_tahap(args.hapus)
    elif args.hasil:
        penguji = Penguji(pakai_cbf=ada_study("CBF"))
        laporkan("PD", penguji)
        laporkan("CBF", penguji)
    elif args.tahap:
        if args.trial is not None:
            jumlah = args.trial
        elif args.tahap == "PD":
            jumlah = config.OPTUNA_TRIAL_PD
        else:
            jumlah = config.OPTUNA_TRIAL_CBF
        jalankan_tahap(args.tahap, jumlah)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()