#!/usr/bin/env python3
"""
Identifikasi momen inersia roda DDSM115 (uji akselerasi + coast-down).

PENTING: roda harus TERANGKAT dan bebas berputar. Jangan sentuh roda saat uji.

Urutan tiap run:
  1. diam        : arus 0 selama 0,2 s
  2. akselerasi  : arus konstan sampai kecepatan target (atau batas waktu)
  3. coast       : arus 0, roda melambat sendiri sampai berhenti

Yang diukur (semuanya dari kolom POSISI, bukan dari rpm umpan balik):
  a_naik  = percepatan sudut saat arus konstan            [rad/s^2]
  a_coast = perlambatan sudut saat arus nol (gesekan)      [rad/s^2]
  G       = (a_naik - a_coast) / i_perintah                [rad/s^2 per A]

Model:  J * a = K * i_perintah - torsi_gesek   ->   G = K / J
G adalah hasil utama dan tidak bergantung pada konstanta torsi. Untuk mendapat J
sendiri diperlukan skala torsi; program memberi dua taksiran (lihat keluaran) dan
menyediakan uji beban tambahan (--j-add, --g-ref) yang tidak bergantung pada Kt.

Catatan perilaku DDSM115 yang sudah terukur dan ditangani di sini:
  - rpm umpan balik tertinggal ~0,1 s dari kecepatan sebenarnya,
  - arah hitung posisi bisa berlawanan dengan tanda perintah,
  - arus umpan balik tidak sama dengan arus perintah,
  - setelah arus dinolkan, torsi motor baru hilang setelah ~0,15-0,2 s.

Pemakaian:
  python3 ddsm115_inertia_test.py                       kanan (ACM0) lalu kiri (ACM1), sekali start
  python3 ddsm115_inertia_test.py --wheels kanan=/dev/ttyACM0            hanya satu roda
  python3 ddsm115_inertia_test.py --currents 0.2 0.25 0.3 --plot
  python3 ddsm115_inertia_test.py --analyze DATA_DDSM115/inersia_20261004-*.csv
  python3 ddsm115_inertia_test.py --label beban --j-add 4.0e-4 --g-ref kanan=255 kiri=272

Tekan S satu kali untuk mulai (Q untuk batal). Roda diuji SATU PER SATU supaya laju
sampel tetap tinggi dan catu daya tidak terbebani dua roda sekaligus.
Data tiap run disimpan di folder DATA_DDSM115 (di sebelah file program ini) dengan nama:
  inersia_<yyyymmdd>-<percobaan>-<urutan>.csv   contoh: inersia_20261004-kanan_0.20A-01.csv
<percobaan> = [label_]roda_arus ("neg" untuk arah sebaliknya); <urutan> naik otomatis.

Butuh: pip install pyserial numpy   (matplotlib opsional, untuk --plot)
"""
import argparse
import csv
import os
import re
import sys
import time
from collections import deque
from datetime import datetime

import numpy as np

BAUD = 115200
RAW_FULL = 32767        # nilai mentah skala penuh
I_FULL = 8.0            # A, skala penuh arus (perintah maupun umpan balik)
POS_MOD = 32768         # posisi mentah 0..32767 = 0..360 derajat
MAX_CURRENT = 1.5       # A, batas aman perintah (arus rated motor)

WHEEL_MASS = 0.745      # kg, massa satu unit roda+motor (hasil timbang)
WHEEL_DIAMETER = 0.100  # m, diameter roda (hasil ukur)
DEFAULT_WHEELS = ["kanan=/dev/ttyACM0", "kiri=/dev/ttyACM1"]   # diuji berurutan
DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "DATA_DDSM115")

MODE_CURRENT = 0x01
MODE_VELOCITY = 0x02

ACC_SKIP = 0.06         # s, awal akselerasi yang dibuang (lepas dari diam, lonjakan arus)
COAST_SKIP = 0.25       # s, awal coast yang dibuang (torsi motor belum hilang)
V_HI, V_LO_ACC, V_LO_COAST = 0.85, 0.08, 0.20   # batas jendela, relatif terhadap kecepatan puncak
RPM = 60 / (2 * np.pi)


class RunError(RuntimeError):
    pass


# ---------------------------------------------------------------- protokol --
def crc8_maxim(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8C if crc & 1 else crc >> 1
    return crc


def frame_drive(motor_id: int, value: int) -> bytes:
    """Frame 0x64: nilai = arus / kecepatan / posisi, tergantung mode aktif."""
    body = bytes([motor_id, 0x64]) + int(value).to_bytes(2, "big", signed=True) + bytes(5)
    return body + bytes([crc8_maxim(body)])


def frame_mode(motor_id: int, mode: int) -> bytes:
    """Frame ganti mode: byte terakhir adalah nilai mode (bukan CRC)."""
    return bytes([motor_id, 0xA0]) + bytes(7) + bytes([mode])


def parse_feedback(resp: bytes, motor_id: int):
    if len(resp) != 10 or resp[0] != motor_id or crc8_maxim(resp[:9]) != resp[9]:
        return None
    return {
        "mode": resp[1],
        "i": int.from_bytes(resp[2:4], "big", signed=True) * I_FULL / RAW_FULL,  # A
        "rpm": int.from_bytes(resp[4:6], "big", signed=True),                    # rpm (tertinggal)
        "pos": int.from_bytes(resp[6:8], "big", signed=False),                   # 0..32767
        "err": resp[8],
    }


class Motor:
    def __init__(self, port: str, motor_id: int):
        import serial  # diimpor di sini supaya mode --analyze tidak butuh pyserial
        self.id = motor_id
        self.ser = serial.Serial(port, BAUD, timeout=0.02)
        time.sleep(0.1)
        self.ser.reset_input_buffer()

    def set_mode(self, mode: int):
        self.ser.write(frame_mode(self.id, mode))  # perintah ini tidak dibalas
        self.ser.flush()
        time.sleep(0.05)

    def drive(self, value: int):
        self.ser.reset_input_buffer()
        self.ser.write(frame_drive(self.id, value))
        self.ser.flush()
        return parse_feedback(self.ser.read(10), self.id)

    def close(self):
        self.ser.close()


# ----------------------------------------------------------------- perekam --
def wrap_diff(d):
    """Selisih posisi mentah dengan memperhitungkan lompatan 32767 <-> 0."""
    return (d + POS_MOD / 2) % POS_MOD - POS_MOD / 2


class SpeedMeter:
    """Kecepatan [rpm, tanpa tanda] dari posisi, dirata-rata atas beberapa sampel terakhir."""

    def __init__(self, window: int = 10):
        self.buf = deque(maxlen=window + 1)
        self.count, self.last = 0.0, None

    def update(self, t: float, pos: int) -> float:
        if self.last is not None:
            self.count += wrap_diff(pos - self.last)
        self.last = pos
        self.buf.append((t, self.count))
        (t0, c0), (t1, c1) = self.buf[0], self.buf[-1]
        return abs(c1 - c0) / POS_MOD / (t1 - t0) * 60 if t1 > t0 else 0.0


def record(m: Motor, amps: float, a) -> list:
    """Jalankan satu run, kembalikan list (t, fase, i_cmd, i_fb, rpm, pos, err)."""
    raw_cmd = int(round(amps / I_FULL * RAW_FULL))
    rows, cmd, phase = [], 0, "diam"
    t_phase, low, miss = 0.0, 0, 0
    meter = SpeedMeter()
    t0 = time.perf_counter()
    while True:
        ta = time.perf_counter()
        fb = m.drive(cmd)
        tb = time.perf_counter()
        if fb is None:
            miss += 1
            if miss > 20:
                raise RunError("motor tidak membalas (cek port, ID, catu daya)")
            continue
        miss = 0
        t = 0.5 * (ta + tb) - t0
        v = meter.update(t, fb["pos"])       # rpm dari posisi; rpm umpan balik terlalu lambat
        rows.append((t, phase, cmd * I_FULL / RAW_FULL, fb["i"], fb["rpm"], fb["pos"], fb["err"]))
        if fb["err"]:
            raise RunError(f"motor melaporkan error 0x{fb['err']:02X}")

        if phase == "diam":
            if t >= 0.2:
                phase, cmd, t_phase = "akselerasi", raw_cmd, t
        elif phase == "akselerasi":
            if v >= a.rpm_target or t - t_phase >= a.t_acc:
                phase, cmd, t_phase = "coast", 0, t
            elif t - t_phase > 1.0 and v < 3:
                raise RunError("roda tidak bergerak, arus terlalu kecil untuk melawan gesekan")
        else:  # coast
            low = low + 1 if v <= a.rpm_stop else 0
            if low >= 15 or t - t_phase >= a.t_coast:
                return rows


def wait_stop(m: Motor, timeout=10.0):
    """Kirim arus 0 sampai roda diam dan rpm umpan balik sudah turun ke nol."""
    t0, still = time.perf_counter(), 0
    while time.perf_counter() - t0 < timeout and still < 50:
        fb = m.drive(0)
        still = still + 1 if fb and fb["rpm"] == 0 else 0
        time.sleep(0.005)


def get_key():
    """Baca satu tombol tanpa Enter (Linux). Jika bukan terminal, pakai input biasa."""
    if not sys.stdin.isatty():
        return (sys.stdin.readline().strip() or "q")[0]
    import termios
    import tty
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def wait_for_start() -> bool:
    print("Tekan S untuk mulai, Q untuk batal.")
    while True:
        key = get_key().lower()
        if key == "s":
            return True
        if key in ("q", "\x03", "\x1b", ""):
            return False


def exp_label(amps: float, label: str = "") -> str:
    """Nama percobaan untuk nama file: [label_]0.20A, atau neg0.20A untuk arah sebaliknya."""
    cur = f"{'neg' if amps < 0 else ''}{abs(amps):.2f}A"
    label = re.sub(r"[^A-Za-z0-9_.]+", "_", label).strip("_")
    return f"{label}_{cur}" if label else cur


def next_path(label: str, ext: str = "csv") -> str:
    """DATA_DDSM115/inersia_<yyyymmdd>-<percobaan>-<urutan>.<ext>, urutan naik otomatis."""
    os.makedirs(DATA_DIR, exist_ok=True)
    prefix = f"inersia_{datetime.now():%Y%m%d}-{label}-"
    pat = re.compile(re.escape(prefix) + r"(\d+)\." + re.escape(ext) + "$")
    used = [int(mm.group(1)) for mm in map(pat.match, os.listdir(DATA_DIR)) if mm]
    return os.path.join(DATA_DIR, f"{prefix}{max(used, default=0) + 1:02d}.{ext}")


def save_csv(path, rows):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "fase", "i_cmd_A", "i_fb_A", "rpm_fb", "pos_raw", "err"])
        w.writerows(rows)


def load_csv(path):
    with open(path) as f:
        r = [x for x in csv.reader(f) if x][1:]
    return [(float(x[0]), x[1], float(x[2]), float(x[3]), int(x[4]), int(x[5]), int(x[6])) for x in r]


# ---------------------------------------------------------------- analisis --
def quad_accel(t, th):
    """Percepatan konstan terbaik untuk data sudut: th = c0 + c1 t + (a/2) t^2."""
    tt = t - t.mean()
    return 2 * np.polyfit(tt, th, 2)[0]


def analyze(rows, kt: float, k: int = 8) -> dict:
    t = np.array([r[0] for r in rows])
    icmd = np.array([r[2] for r in rows])
    i = np.array([r[3] for r in rows])
    rpm = np.array([r[4] for r in rows], dtype=float)
    pos = np.array([r[5] for r in rows], dtype=float)
    ph = [r[1] for r in rows]
    n = len(t)
    if "akselerasi" not in ph or "coast" not in ph:
        raise RunError("data tidak memuat fase akselerasi dan coast")
    ia, ic = ph.index("akselerasi"), ph.index("coast")   # sampel saat perintah arus berubah
    cmd = icmd[ic - 1]
    if cmd == 0:
        raise RunError("arus perintah nol")

    # sudut kontinu [rad]; diarahkan supaya gerak selama akselerasi bernilai positif
    theta = np.concatenate([[0.0], np.cumsum(wrap_diff(np.diff(pos)))]) * 2 * np.pi / POS_MOD
    travel = theta[ic] - theta[ia]
    if abs(travel) < 1.0:
        raise RunError("roda hampir tidak bergerak selama akselerasi")
    pos_dir = int(np.sign(travel) * np.sign(cmd))        # -1: posisi menghitung berlawanan perintah
    theta *= np.sign(travel)
    i_dir = i * np.sign(cmd)                             # arus umpan balik, positif = searah perintah

    # kecepatan halus [rad/s], hanya untuk memilih jendela analisis
    w = np.full(n, np.nan)
    w[k:n - k] = (theta[2 * k:] - theta[:n - 2 * k]) / (t[2 * k:] - t[:n - 2 * k])
    wpk = np.nanmax(w)
    idx = np.arange(n)

    acc = (idx > ia) & (idx <= ic) & (t >= t[ia] + ACC_SKIP) & (w > V_LO_ACC * wpk) & (w < V_HI * wpk)
    slow = np.where((idx > ic) & (w < V_LO_COAST * wpk))[0]
    end = slow[0] if len(slow) else n
    cst = (idx < end) & (t >= t[ic] + COAST_SKIP) & (w < V_HI * wpk) & (w > V_LO_COAST * wpk)
    if acc.sum() < 15:
        raise RunError(f"fase akselerasi terlalu singkat ({int(acc.sum())} sampel terpakai); kecilkan arus")
    if cst.sum() < 30:
        raise RunError(f"fase coast terlalu singkat ({int(cst.sum())} sampel terpakai)")

    a_up = quad_accel(t[acc], theta[acc])
    a_dn = quad_accel(t[cst], theta[cst])
    if a_up <= 0 or a_dn >= 0:
        raise RunError(f"percepatan tidak masuk akal (a_naik {a_up:.1f}, a_coast {a_dn:.1f})")

    # gesekan viskos: bandingkan perlambatan di paruh cepat dan paruh lambat fase coast
    ci = np.where(cst)[0]
    h1, h2 = ci[:len(ci) // 2], ci[len(ci) // 2:]
    d1, d2 = -quad_accel(t[h1], theta[h1]), -quad_accel(t[h2], theta[h2])
    w1, w2 = np.nanmean(w[h1]), np.nanmean(w[h2])
    visc = max((d1 - d2) / (w1 - w2), 0.0) if w1 > w2 else 0.0       # [1/s]  = b / J
    coul = max(-a_dn - visc * np.nanmean(w[cst]), 0.0)                # [rad/s^2] = tau_c / J

    i_acc, i_cst = i_dir[acc].mean(), i_dir[cst].mean()
    da = a_up - a_dn
    G = da / abs(cmd)
    lag = np.nanmean(w[acc] - rpm[acc] * np.sign(cmd) / RPM) / a_up   # ketertinggalan rpm umpan balik [s]

    return {"cmd": cmd, "a_up": a_up, "a_dn": a_dn, "G": G, "visc": visc, "coul": coul,
            "i_acc": i_acc, "i_cst": i_cst,
            "J_fb": kt * (i_acc - i_cst) / da,        # torsi dari perubahan arus umpan balik
            "J_cmd": kt / G,                          # torsi dari arus perintah
            "peak": wpk * RPM, "lag": lag, "pos_dir": pos_dir,
            "n_acc": int(acc.sum()), "n_coast": int(cst.sum()), "rate": (n - 1) / (t[-1] - t[0]),
            "t": t, "w": w, "i": i_dir, "rpm": rpm * np.sign(cmd), "acc": acc, "cst": cst}


def report(label, r):
    print(f"  {label}: a_naik = {r['a_up']:+.1f} rad/s^2 | a_coast = {r['a_dn']:+.1f} rad/s^2 | "
          f"G = {r['G']:.0f} rad/s^2 per A")
    print(f"      arus umpan balik: {r['i_acc']:+.3f} A saat akselerasi, {r['i_cst']:+.3f} A saat coast "
          f"(perintah {abs(r['cmd']):.3f} A)")
    print(f"      puncak {r['peak']:.0f} rpm | rpm umpan balik tertinggal ~{r['lag']:.2f} s | "
          f"arah posisi {'searah' if r['pos_dir'] > 0 else 'BERLAWANAN dengan'} perintah")
    print(f"      sampel terpakai: {r['n_acc']} akselerasi + {r['n_coast']} coast, laju {r['rate']:.0f} Hz")


def summarize(results, a, name="", g_ref=None):
    """Cetak ringkasan satu roda; kembalikan angka-angkanya untuk tabel perbandingan."""
    title = f" roda {name}" if name else ""
    if not results:
        print(f"\nHASIL{title}: tidak ada run yang berhasil.")
        return None
    mean = lambda key: float(np.mean([r[key] for r in results]))
    G = np.array([r["G"] for r in results])
    r_w = a.diameter / 2
    j_ring = a.mass * r_w ** 2
    spread = f" (simpangan baku {G.std(ddof=1):.0f}, {100 * G.std(ddof=1) / G.mean():.1f}%)" if len(G) > 1 else ""
    out = {"name": name, "n": len(results), "G": G.mean(), "a_dn": -mean("a_dn")}

    print(f"\nHASIL{title} dari {len(results)} run")
    print(f"  G = K/J = {G.mean():.0f} rad/s^2 per A perintah{spread}")
    print(f"  perlambatan gesek saat arus nol = {-mean('a_dn'):.1f} rad/s^2")

    if a.j_add and g_ref:
        if g_ref <= G.mean():
            print("  Uji beban: G dengan beban harus lebih kecil dari G tanpa beban. Cek --g-ref.")
            return out
        J = a.j_add * G.mean() / (g_ref - G.mean())
        out.update(J=J, K=J * g_ref, basis="uji beban")
        print(f"  Uji beban tambahan (J_tambah = {a.j_add:.2e} kg m^2, G tanpa beban = {g_ref:.0f}):")
        print(f"    J_w = {J:.2e} kg m^2 = {J / j_ring:.2f} m r^2   (tidak bergantung pada Kt)")
        print(f"    K   = {J * g_ref:.3f} N m per A perintah")
        print(f"    massa ekuivalen translasi J_w / r^2 = {J / r_w ** 2:.3f} kg")
        print("    tau_c dan b: ambil dari uji tanpa beban, dikalikan J_w ini.")
        return out

    J, J_cmd = mean("J_fb"), a.kt / G.mean()
    ok = 0.15 * j_ring <= J <= j_ring
    out.update(J=J, K=J * G.mean(), basis="arus umpan balik")
    print(f"  J_w = {J:.2e} kg m^2 = {J / j_ring:.2f} m r^2   "
          f"[taksiran dari perubahan arus umpan balik x Kt {a.kt}; {'masuk akal' if ok else 'DI LUAR rentang wajar'}]")
    print(f"  K   = {J * G.mean():.3f} N m per A perintah")
    print(f"  gesekan: total {J * -mean('a_dn'):.4f} N m; pembagian kasar tau_c = {J * mean('coul'):.4f} N m, "
          f"b = {J * mean('visc'):.1e} N m s/rad")
    print(f"  massa ekuivalen translasi J_w / r^2 = {J / r_w ** 2:.3f} kg")
    if J_cmd > j_ring:
        print(f"  Info: Kt datasheet dikenakan langsung ke arus perintah memberi {J_cmd:.2e} kg m^2 (> m r^2 = "
              f"{j_ring:.2e}),")
        print("        jadi taksiran itu tidak dipakai. Ini bukan kesalahan pengukuran.")
    return out


def compare(summaries, a):
    """Tabel perbandingan antar-roda."""
    rows = [s for s in summaries if s]
    if len(rows) < 2:
        return
    print("\nPERBANDINGAN RODA")
    print(f"  {'roda':<10}{'run':>4}{'G [rad/s^2/A]':>16}{'gesek [rad/s^2]':>17}{'J_w [kg m^2]':>15}{'K [N m/A]':>11}")
    for s in rows:
        J = f"{s['J']:.2e}" if "J" in s else "-"
        K = f"{s['K']:.3f}" if "K" in s else "-"
        print(f"  {s['name']:<10}{s['n']:>4}{s['G']:>16.0f}{s['a_dn']:>17.1f}{J:>15}{K:>11}")
    G = [s["G"] for s in rows]
    print(f"  selisih G antar-roda: {100 * (max(G) - min(G)) / np.mean(G):.1f}%")
    if all("J" in s for s in rows):
        basis = rows[0]["basis"]
        print(f"  J_w dan K di atas dari {basis}"
              + ("; skala torsinya belum terverifikasi (lihat --j-add)." if basis != "uji beban" else "."))


def plot(results, labels, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(2, 1, sharex=True, figsize=(9, 6.5))
    for n, (r, lab) in enumerate(zip(results, labels)):
        c = f"C{n % 10}"
        ax[0].plot(r["t"], r["w"] * RPM, color=c, label=f"{lab} (posisi)")
        ax[0].plot(r["t"], r["rpm"], color=c, ls=":", lw=1)
        for m in (r["acc"], r["cst"]):
            ax[0].plot(r["t"][m], r["w"][m] * RPM, color=c, lw=3, alpha=0.35)
        ax[1].plot(r["t"], r["i"], color=c, lw=0.8)
    ax[0].set_ylabel("kecepatan [rpm]")
    ax[0].set_title("garis tebal = jendela analisis, titik-titik = rpm umpan balik", fontsize=9)
    ax[1].set_ylabel("arus umpan balik [A]"); ax[1].set_xlabel("t [s]")
    ax[0].legend(fontsize=8); ax[0].grid(True); ax[1].grid(True)
    fig.tight_layout(); fig.savefig(path, dpi=120)
    print(f"Grafik disimpan: {path}")


# -------------------------------------------------------------------- main --
def parse_pairs(items, option):
    out = {}
    for it in items:
        if "=" not in it:
            raise ValueError(f"{option}: '{it}' harus berbentuk nama=nilai")
        key, val = it.split("=", 1)
        out[key.strip()] = val.strip()
    return out


def run_wheel(name, port, a):
    """Uji satu roda untuk semua arus. Kembalikan (results, labels)."""
    results, labels = [], []
    print(f"\n=== Roda {name} ({port}) ===")
    try:
        m = Motor(port, a.id)
    except Exception as e:  # port tidak ada / tidak bisa dibuka
        print(f"  GAGAL membuka port: {e}")
        return results, labels
    try:
        wait_stop(m, timeout=3.0)
        m.set_mode(MODE_CURRENT)
        fb = m.drive(0)
        if fb is None:
            raise RunError("motor tidak membalas (cek port, ID, catu daya)")
        if fb["mode"] != MODE_CURRENT:
            raise RunError(f"gagal masuk mode arus (mode terbaca {fb['mode']})")

        for amps in a.currents:
            label = f"{name} {amps:+.2f} A"
            print(f"\nRun {label} ...")
            try:
                rows = record(m, amps, a)
            except RunError as e:
                print(f"  GAGAL - {e}")
                m.drive(0)
                if "error" in str(e) or "membalas" in str(e):
                    break
                continue
            path = next_path(exp_label(amps, "_".join(x for x in (a.label, name) if x)))
            save_csv(path, rows)
            print(f"  data disimpan: {path}")
            try:
                r = analyze(rows, a.kt)
                report(label, r); results.append(r); labels.append(label)
            except RunError as e:
                print(f"  analisis gagal - {e}")
            wait_stop(m)
    except RunError as e:
        print(f"  GAGAL - {e}")
    finally:
        # arus nol, lalu kembalikan ke mode kecepatan dengan target 0 rpm (juga saat Ctrl+C)
        try:
            m.drive(0)
            m.set_mode(MODE_VELOCITY)
            m.drive(0)
        finally:
            m.close()
        print(f"  Roda {name}: motor dikembalikan ke mode kecepatan, port ditutup.")
    return results, labels


def main():
    p = argparse.ArgumentParser(description="Identifikasi inersia roda DDSM115 (satu atau beberapa roda)")
    p.add_argument("--wheels", nargs="+", metavar="NAMA=PORT", default=DEFAULT_WHEELS,
                   help="roda yang diuji berurutan, mis. kanan=/dev/ttyACM0 kiri=/dev/ttyACM1")
    p.add_argument("--id", type=int, default=1, help="ID motor (sama untuk semua port)")
    p.add_argument("--label", default="", help="awalan tambahan untuk nama file, mis. beban")
    p.add_argument("--currents", type=float, nargs="+", default=[0.2, 0.25, 0.3],
                   help="arus uji [A]; nilai negatif = arah sebaliknya")
    p.add_argument("--kt", type=float, default=0.75, help="konstanta torsi datasheet [N m/A]")
    p.add_argument("--rpm-target", type=float, default=110,
                   help="kecepatan (dari posisi) untuk mengakhiri akselerasi; jauh di bawah kecepatan tanpa beban")
    p.add_argument("--rpm-stop", type=float, default=3, help="kecepatan (dari posisi) untuk mengakhiri coast")
    p.add_argument("--t-acc", type=float, default=3.0, help="batas waktu akselerasi [s]")
    p.add_argument("--t-coast", type=float, default=20.0, help="batas waktu coast [s]")
    p.add_argument("--mass", type=float, default=WHEEL_MASS, help="massa unit roda [kg]")
    p.add_argument("--diameter", type=float, default=WHEEL_DIAMETER, help="diameter roda [m]")
    p.add_argument("--j-add", type=float, help="inersia beban tambahan yang terpasang [kg m^2]")
    p.add_argument("--g-ref", nargs="+", metavar="G",
                   help="G tanpa beban: satu angka untuk semua roda, atau kanan=255 kiri=272")
    p.add_argument("--analyze", nargs="+", metavar="CSV", help="analisis ulang file CSV, tanpa motor")
    p.add_argument("--plot", action="store_true", help="simpan grafik (butuh matplotlib)")
    a = p.parse_args()

    try:
        wheels = parse_pairs(a.wheels, "--wheels")
        g_all, g_by = None, {}
        if a.g_ref:
            if len(a.g_ref) == 1 and "=" not in a.g_ref[0]:
                g_all = float(a.g_ref[0])
            else:
                g_by = {k: float(v) for k, v in parse_pairs(a.g_ref, "--g-ref").items()}
    except ValueError as e:
        p.error(str(e))
    if bool(a.j_add) != bool(a.g_ref):
        p.error("--j-add dan --g-ref harus diberikan bersama")
    g_ref = lambda name: g_by.get(name, g_all)

    if a.analyze:
        # kelompokkan file per roda berdasarkan nama roda di nama file
        groups = {}
        for path in a.analyze:
            base = os.path.basename(path)
            name = next((w for w in wheels if re.search(rf"(?<![A-Za-z]){re.escape(w)}(?![A-Za-z])", base)), "")
            try:
                r = analyze(load_csv(path), a.kt)
                report(base, r)
                groups.setdefault(name, ([], []))
                groups[name][0].append(r); groups[name][1].append(base)
            except (RunError, OSError, ValueError, IndexError) as e:
                print(f"  {base}: GAGAL - {e}")
        summaries = [summarize(res, a, name, g_ref(name)) for name, (res, _) in groups.items()]
        compare(summaries, a)
        if not groups:
            print("\nTidak ada file yang berhasil dianalisis.")
        if a.plot:
            for name, (res, labs) in groups.items():
                plot(res, labs, next_path("_".join(x for x in (name, "analisis") if x), "png"))
        return

    for amps in a.currents:
        if abs(amps) > MAX_CURRENT:
            p.error(f"arus {amps} A melebihi batas aman {MAX_CURRENT} A")

    print("Roda yang diuji, satu per satu: " + ", ".join(f"{n} ({prt})" for n, prt in wheels.items()))
    print("Pastikan SEMUA roda TERANGKAT dan bebas berputar.")
    if not wait_for_start():
        print("Dibatalkan.")
        return
    print("Mulai...")

    done = []
    try:
        for name, port in wheels.items():
            res, labs = run_wheel(name, port, a)
            done.append((name, res, labs))
    except KeyboardInterrupt:
        print("\nDihentikan dengan Ctrl+C")

    summaries = [summarize(res, a, name, g_ref(name)) for name, res, _ in done]
    compare(summaries, a)
    if a.plot:
        for name, res, labs in done:
            if res:
                plot(res, labs, next_path("_".join(x for x in (a.label, name, "grafik") if x), "png"))


if __name__ == "__main__":
    main()