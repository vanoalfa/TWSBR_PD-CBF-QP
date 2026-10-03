#!/usr/bin/env python3
"""
Body pitch inertia of the TWSBR from a compound-pendulum (swing) test with the MPU6050.

The robot (or the body alone) hangs from a pivot that is parallel to the wheel axle and
swings freely with a small angle. The swing period T is taken from the gyroscope only:
the accelerometer / Kalman angle is not valid while the robot swings as a pendulum.

    I_pivot = m * g * d * T0^2 / (4 pi^2)      d = distance pivot -> centre of mass
    I_cm    = I_pivot - m * d^2

T0 is the measured period corrected for finite amplitude and damping.

Geometry is given by ONE number, --pivot: the distance from the wheel axle to the pivot,
measured along the body towards the body centre of mass (0 = pivot on the wheel axle,
negative = pivot on the other side of the axle).

Defaults match the current set-up: whole robot hanging on a rod 300.556 mm from the wheel
axle, wheels attached and taped to the body so they swing with it.

  python3 mpu6050_inersia_badan.py                     record one swing test
  python3 mpu6050_inersia_badan.py --plot              same, and save a graph
  python3 mpu6050_inersia_badan.py --analyze DATA_INERSIA_BADAN/bandul_*.csv
  python3 mpu6050_inersia_badan.py --pivot 0.250       rod at another position
  python3 mpu6050_inersia_badan.py --body-only --pivot 0.250    wheels removed

Press S once: the gyro bias is measured while the robot hangs still, then you pull the
robot aside (less than about 10 degrees), release it, and the recording starts by itself.
Each run is saved in DATA_INERSIA_BADAN (next to this file) as
  bandul_<yyyymmdd>-<label>-<nn>.csv

Sensor settings follow program/apalah_bisa/mpu6050.py: I2C bus 1, address 0x68,
gyro full scale +-250 deg/s (131 LSB per deg/s).

Requires: pip install smbus2 numpy   (matplotlib optional, for --plot)
"""
import argparse
import csv
import os
import re
import sys
import time
from datetime import datetime

import numpy as np

# ---- robot parameters (defaults = values measured so far) ---------------------
MASS_BODY = 1.667        # kg, body without the wheel units
L_COM = 0.131224         # m, wheel axle -> body centre of mass
MASS_WHEEL_UNIT = 0.745  # kg, ONE wheel unit (rotor + stator), treated as a mass on the axle
J_WHEEL = 9.2e-4         # kg m^2, rotating part of ONE wheel (from the DDSM115 test)
PIVOT = 0.300556         # m, wheel axle -> hanging rod (pivot), towards the body CoM
GRAVITY = 9.78           # m/s^2, local value (Java is about 9.78; 9.81 changes I_cm by ~1 %)

# ---- MPU6050 -------------------------------------------------------------------
I2C_BUS = 1
MPU6050_ADDRESS = 0x68
PWR_MGMT_1, SMPLRT_DIV, CONFIG, GYRO_CONFIG, ACCEL_CONFIG = 0x6B, 0x19, 0x1A, 0x1B, 0x1C
ACCEL_XOUT_H = 0x3B
GYRO_LSB_PER_DPS = 131.0     # full scale +-250 deg/s
GYRO_LIMIT_DPS = 240.0       # above this the gyro is close to saturation

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "DATA_INERSIA_BADAN")
AXES = "xyz"


class RunError(RuntimeError):
    pass


class Gyro:
    """Minimal MPU6050 reader: one 14-byte burst per sample, gyro rates in deg/s."""

    def __init__(self, bus_id: int = I2C_BUS, address: int = MPU6050_ADDRESS):
        from smbus2 import SMBus  # imported here so that --analyze works without smbus2
        self.address = address
        self.bus = SMBus(bus_id)
        self.bus.write_byte_data(address, PWR_MGMT_1, 0x00)
        time.sleep(0.05)
        self.bus.write_byte_data(address, SMPLRT_DIV, 0x00)
        self.bus.write_byte_data(address, CONFIG, 0x03)        # DLPF 44 Hz: less noise, swing is ~1 Hz
        self.bus.write_byte_data(address, GYRO_CONFIG, 0x00)   # +-250 deg/s
        self.bus.write_byte_data(address, ACCEL_CONFIG, 0x00)  # +-2 g
        time.sleep(0.05)

    def read(self):
        """Return (t, gx, gy, gz) with t = mid-time of the I2C transfer."""
        ta = time.perf_counter()
        raw = self.bus.read_i2c_block_data(self.address, ACCEL_XOUT_H, 14)
        tb = time.perf_counter()
        g = [int.from_bytes(bytes(raw[k:k + 2]), "big", signed=True) / GYRO_LSB_PER_DPS for k in (8, 10, 12)]
        return 0.5 * (ta + tb), g[0], g[1], g[2]

    def close(self):
        self.bus.close()


# ---------------------------------------------------------------- recording --
def measure_bias(gyro: Gyro, seconds: float = 2.0):
    """Mean gyro rate while hanging still. Raises if the robot is moving."""
    t_end = time.perf_counter() + seconds
    rows = []
    while time.perf_counter() < t_end:
        rows.append(gyro.read()[1:])
        time.sleep(0.002)
    g = np.array(rows)
    if g.std(axis=0).max() > 1.0:
        raise RunError("robot belum diam saat kalibrasi bias giroskop; tunggu sampai berhenti lalu ulangi")
    return g.mean(axis=0)


def record(gyro: Gyro, bias, a) -> list:
    """Wait for the release, skip the first moments, then record until the swing dies out."""
    period = 1.0 / a.rate
    # 1. wait for motion
    t_wait = time.perf_counter()
    while True:
        _, gx, gy, gz = gyro.read()
        if max(abs(gx - bias[0]), abs(gy - bias[1]), abs(gz - bias[2])) > a.start_dps:
            break
        if time.perf_counter() - t_wait > 60:
            raise RunError("tidak ada ayunan terdeteksi dalam 60 s")
        time.sleep(0.002)
    print(f"Ayunan terdeteksi. Lepaskan tangan; perekaman mulai {a.settle:.0f} s lagi...")
    time.sleep(a.settle)

    # 2. record
    print(f"Merekam sampai {a.duration:.0f} s (berhenti sendiri kalau ayunan sudah habis). Jangan disentuh.")
    rows, t0, nxt = [], time.perf_counter(), time.perf_counter()
    while True:
        t, gx, gy, gz = gyro.read()
        rows.append((t - t0, gx - bias[0], gy - bias[1], gz - bias[2]))
        if t - t0 >= a.duration:
            break
        if len(rows) % 50 == 0 and t - t0 > 4.0:
            recent = np.array([r[1:] for r in rows[-int(2.0 * a.rate):]])
            if np.abs(recent).max() < a.stop_dps:       # swing has died out
                break
        nxt += period
        pause = nxt - time.perf_counter()
        if pause > 0:
            time.sleep(pause)
    return rows


def get_key():
    """Read one key without Enter (Linux). Falls back to a normal line read if not a terminal."""
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


def next_path(label: str, ext: str = "csv") -> str:
    """DATA_INERSIA_BADAN/bandul_<yyyymmdd>-<label>-<nn>.<ext>, nn increases automatically."""
    os.makedirs(DATA_DIR, exist_ok=True)
    label = re.sub(r"[^A-Za-z0-9_.]+", "_", label).strip("_") or "badan"
    prefix = f"bandul_{datetime.now():%Y%m%d}-{label}-"
    pat = re.compile(re.escape(prefix) + r"(\d+)\." + re.escape(ext) + "$")
    used = [int(m.group(1)) for m in map(pat.match, os.listdir(DATA_DIR)) if m]
    return os.path.join(DATA_DIR, f"{prefix}{max(used, default=0) + 1:02d}.{ext}")


def save_csv(path, rows):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "gyro_x_dps", "gyro_y_dps", "gyro_z_dps"])
        w.writerows(rows)


def load_csv(path):
    with open(path) as f:
        return [tuple(float(v) for v in x) for x in list(csv.reader(f))[1:] if x]


# ----------------------------------------------------------------- analysis --
def crossings(t, y, rising: bool):
    """Times where y crosses zero in one direction (linear interpolation)."""
    s = y[:-1] < 0 if rising else y[:-1] > 0
    e = y[1:] >= 0 if rising else y[1:] <= 0
    k = np.where(s & e)[0]
    return t[k] - y[k] * (t[k + 1] - t[k]) / (y[k + 1] - y[k])


def fit_period(tc):
    """Least-squares period from equally spaced crossing times; returns (T, standard error)."""
    n = np.arange(len(tc))
    A = np.column_stack([n, np.ones_like(n)])
    (T, _), res, *_ = np.linalg.lstsq(A, tc, rcond=None)
    dof = max(len(tc) - 2, 1)
    sigma = np.sqrt(res[0] / dof) if len(res) else 0.0
    return T, sigma / np.sqrt(np.sum((n - n.mean()) ** 2))


def analyze(rows, axis: str = "auto", min_dps: float = 3.0) -> dict:
    data = np.array(rows, dtype=float)
    if len(data) < 200:
        raise RunError("rekaman terlalu pendek")
    t = data[:, 0]
    g = data[:, 1:4] - data[:, 1:4].mean(axis=0)
    k_ax = int(np.argmax(g.std(axis=0))) if axis == "auto" else AXES.index(axis)
    y = g[:, k_ax]
    cross_axis = float(np.delete(g.std(axis=0), k_ax).max() / max(y.std(), 1e-9))
    if np.abs(data[:, 1 + k_ax]).max() > GYRO_LIMIT_DPS:
        raise RunError("giroskop jenuh (> 240 deg/s); ayunkan dengan sudut lebih kecil")

    # rough period from the spectrum, then smooth over 1/12 of a period
    dt = float(np.median(np.diff(t)))
    tu = np.arange(t[0], t[-1], dt)
    yu = np.interp(tu, t, y)
    spec = np.abs(np.fft.rfft(yu * np.hanning(len(yu))))
    freq = np.fft.rfftfreq(len(yu), dt)
    band = (freq > 0.2) & (freq < 6.0)
    if not band.any() or spec[band].max() <= 0:
        raise RunError("tidak ada ayunan di data")
    T_rough = 1.0 / freq[band][np.argmax(spec[band])]
    w = max(int(T_rough / 12 / dt) | 1, 1)
    ys = np.convolve(yu, np.ones(w) / w, mode="same")

    # keep the part of the record where the swing is still clearly above the noise
    n_per = max(int(T_rough / dt), 4)
    env = np.array([np.abs(ys[max(0, i - n_per // 2):i + n_per // 2 + 1]).max() for i in range(len(ys))])
    good = env >= max(min_dps, 0.08 * env.max())
    last = len(good) - 1 - int(np.argmax(good[::-1]))
    first = int(np.argmax(good))
    tu, ys = tu[first:last + 1], ys[first:last + 1]

    up, dn = crossings(tu, ys, True), crossings(tu, ys, False)
    if len(up) < 5 or len(dn) < 5:
        raise RunError(f"terlalu sedikit ayunan ({len(up)} siklus); butuh minimal 5")
    (T_up, s_up), (T_dn, s_dn) = fit_period(up), fit_period(dn)
    T = 0.5 * (T_up + T_dn)
    sigma_T = max(0.5 * np.hypot(s_up, s_dn), abs(T_up - T_dn) / 2)

    # per-cycle peak rate -> angle amplitude; decay -> damping ratio
    pk_t, pk = [], []
    for a0, a1 in zip(up[:-1], up[1:]):
        m = (tu >= a0) & (tu < a1)
        if m.sum() > 3:
            pk_t.append(0.5 * (a0 + a1)); pk.append(np.abs(ys[m]).max())
    pk_t, pk = np.array(pk_t), np.array(pk)
    theta = np.radians(pk) * T / (2 * np.pi)                   # angle amplitude [rad]
    decay = -np.polyfit(pk_t, np.log(pk), 1)[0] if len(pk) >= 3 else 0.0   # [1/s]
    zeta = max(decay, 0.0) * T / (2 * np.pi)
    theta_ms = float(np.mean(theta ** 2))

    # corrections: finite amplitude (T grows by theta0^2/16) and damping (T grows by 1/sqrt(1-zeta^2))
    T0 = T * np.sqrt(1 - zeta ** 2) / (1 + theta_ms / 16)

    return {"T": T, "T0": T0, "sigma_T": sigma_T, "cycles": len(up) - 1, "axis": AXES[k_ax],
            "cross_axis": cross_axis, "theta_max": float(np.degrees(theta.max())),
            "theta_min": float(np.degrees(theta.min())), "zeta": zeta,
            "amp_corr": theta_ms / 16, "rate": 1.0 / dt, "t": tu, "y": ys, "pk_t": pk_t, "pk": pk}


def inertia(T0: float, sigma_T: float, a) -> dict:
    """Body inertia from the corrected period and the pivot position."""
    g, p = a.g, a.pivot
    d_body = p - a.l_com                       # signed distance body CoM -> pivot along the body
    if a.wheels_attached:
        m = a.mass_body + 2 * a.mass_wheel
        com = a.mass_body * a.l_com / m        # CoM of the whole robot, measured from the axle
        d = abs(p - com)
        I_pivot = m * g * d * T0 ** 2 / (4 * np.pi ** 2)
        # remove both wheel units: point masses on the axle plus the rotors' own inertia
        I_body_pivot = I_pivot - 2 * a.mass_wheel * p ** 2 - 2 * a.j_wheel
    else:
        m, d = a.mass_body, abs(d_body)
        I_pivot = m * g * d * T0 ** 2 / (4 * np.pi ** 2)
        I_body_pivot = I_pivot
    if d < 0.01:
        raise RunError("poros terlalu dekat dengan pusat massa (< 1 cm); pindahkan poros")
    I_cm = I_body_pivot - a.mass_body * d_body ** 2
    I_axle = I_cm + a.mass_body * a.l_com ** 2

    # sensitivity: period error, and +-1 mm in the pivot position
    dI_dT = 2 * I_pivot / T0
    eps = 1e-3
    b = argparse.Namespace(**vars(a)); b.pivot = p + eps
    I_cm_shift = _inertia_cm_only(T0, b)
    return {"m": m, "d": d, "I_pivot": I_pivot, "I_cm": I_cm, "I_axle": I_axle,
            "err_T": dI_dT * sigma_T, "err_d": abs(I_cm_shift - I_cm),
            "k": np.sqrt(I_cm / a.mass_body) if I_cm > 0 else float("nan")}


def _inertia_cm_only(T0, a):
    g, p = a.g, a.pivot
    if a.wheels_attached:
        m = a.mass_body + 2 * a.mass_wheel
        d = abs(p - a.mass_body * a.l_com / m)
        Ib = m * g * d * T0 ** 2 / (4 * np.pi ** 2) - 2 * a.mass_wheel * p ** 2 - 2 * a.j_wheel
    else:
        Ib = a.mass_body * g * abs(p - a.l_com) * T0 ** 2 / (4 * np.pi ** 2)
    return Ib - a.mass_body * (p - a.l_com) ** 2


def report(name, r, res):
    print(f"  {name}: T = {r['T']:.4f} s dari {r['cycles']} siklus (galat baku {r['sigma_T'] * 1000:.2f} ms), "
          f"sumbu giroskop {r['axis']}")
    print(f"      amplitudo {r['theta_max']:.1f} -> {r['theta_min']:.1f} deg | koreksi amplitudo "
          f"-{100 * r['amp_corr']:.2f}% | redaman zeta = {r['zeta']:.4f} | T0 = {r['T0']:.4f} s")
    print(f"      I_badan thd pusat massa = {res['I_cm']:.5f} kg m^2 | thd sumbu roda = {res['I_axle']:.5f} kg m^2")
    if r["theta_max"] > 12:
        print("      PERINGATAN: amplitudo awal > 12 deg; ulangi dengan ayunan lebih kecil.")
    if r["cross_axis"] > 0.25:
        print(f"      PERINGATAN: sumbu lain ikut berayun ({100 * r['cross_axis']:.0f}% dari sumbu utama); "
              "robot goyang ke samping atau poros tidak sejajar sumbu roda.")
    if r["zeta"] > 0.05:
        print("      PERINGATAN: redaman besar; gesekan di poros terlalu tinggi untuk hasil yang teliti.")
    if not res["I_cm"] > 0:
        print("      PERINGATAN: I terhadap pusat massa <= 0. Cek --pivot, massa, dan jarak pusat massa.")


def summarize(results, a):
    if not results:
        print("\nTidak ada rekaman yang berhasil dianalisis.")
        return
    I_cm = np.array([res["I_cm"] for _, res in results])
    I_ax = np.array([res["I_axle"] for _, res in results])
    T0 = np.array([r["T0"] for r, _ in results])
    res0 = results[0][1]
    err_T = float(np.mean([res["err_T"] for _, res in results]))
    err_d = float(np.mean([res["err_d"] for _, res in results]))
    spread = I_cm.std(ddof=1) if len(I_cm) > 1 else 0.0
    total = float(np.sqrt(err_T ** 2 + err_d ** 2 + spread ** 2))

    print(f"\nHASIL dari {len(results)} rekaman ({'robot utuh, roda terpasang' if a.wheels_attached else 'badan saja'})")
    print(f"  yang berayun: m = {res0['m']:.3f} kg, jarak poros ke pusat massanya d = {res0['d'] * 1000:.1f} mm")
    print(f"  T0 = {T0.mean():.4f} s" + (f" (simpangan baku {T0.std(ddof=1) * 1000:.2f} ms)" if len(T0) > 1 else ""))
    print(f"  I_badan terhadap pusat massa badan = {I_cm.mean():.5f} kg m^2   <- CBF_J_BODY_KGM2")
    print(f"  I_badan terhadap sumbu roda        = {I_ax.mean():.5f} kg m^2   (= I_cm + m_b l^2)")
    print(f"  jari-jari girasi badan = {res0['k'] * 1000:.0f} mm")
    print(f"  ketidakpastian I_cm: +-{total:.5f} kg m^2 ({100 * total / abs(I_cm.mean()):.0f}%)")
    print(f"     dari periode {err_T:.5f} | dari +-1 mm posisi poros {err_d:.5f}"
          + (f" | sebaran antar-rekaman {spread:.5f}" if len(I_cm) > 1 else " | ulangi 3-5 kali untuk sebaran"))


def plot(results, labels, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(len(results), 1, figsize=(9, 2.6 * len(results) + 0.6), squeeze=False)
    for axis, (r, _), lab in zip(ax[:, 0], results, labels):
        axis.plot(r["t"], r["y"], lw=0.8)
        axis.plot(r["pk_t"], r["pk"], "o", ms=3)
        axis.set_ylabel("laju [deg/s]"); axis.grid(True)
        axis.set_title(f"{lab}: T = {r['T']:.4f} s, {r['cycles']} siklus", fontsize=9)
    ax[-1, 0].set_xlabel("t [s]")
    fig.tight_layout(); fig.savefig(path, dpi=120)
    print(f"Grafik disimpan: {path}")


# --------------------------------------------------------------------- main --
def main():
    p = argparse.ArgumentParser(description="Inersia badan TWSBR dari uji bandul dengan MPU6050")
    p.add_argument("--pivot", type=float, default=PIVOT,
                   help="jarak sumbu roda -> poros gantung [m], positif ke arah pusat massa badan")
    p.add_argument("--body-only", dest="wheels_attached", action="store_false",
                   help="hanya badan yang berayun (roda dilepas); bawaan: robot utuh, roda dikunci ke badan")
    p.add_argument("--mass-body", type=float, default=MASS_BODY, help="massa badan tanpa roda [kg]")
    p.add_argument("--l-com", type=float, default=L_COM, help="jarak sumbu roda -> pusat massa badan [m]")
    p.add_argument("--mass-wheel", type=float, default=MASS_WHEEL_UNIT, help="massa SATU unit roda [kg]")
    p.add_argument("--j-wheel", type=float, default=J_WHEEL, help="inersia SATU roda [kg m^2]")
    p.add_argument("--g", type=float, default=GRAVITY, help="percepatan gravitasi [m/s^2]")
    p.add_argument("--axis", choices=["auto", "x", "y", "z"], default="auto", help="sumbu giroskop ayunan")
    p.add_argument("--duration", type=float, default=30.0, help="lama rekaman maksimum [s]")
    p.add_argument("--settle", type=float, default=2.0, help="jeda setelah dilepas sebelum merekam [s]")
    p.add_argument("--rate", type=float, default=250.0, help="laju sampel target [Hz]")
    p.add_argument("--start-dps", type=float, default=8.0, help="laju untuk mendeteksi ayunan [deg/s]")
    p.add_argument("--stop-dps", type=float, default=2.0, help="laju di bawah ini = ayunan habis [deg/s]")
    p.add_argument("--label", default="badan", help="nama percobaan untuk nama file")
    p.add_argument("--analyze", nargs="+", metavar="CSV", help="analisis ulang file CSV, tanpa sensor")
    p.add_argument("--plot", action="store_true", help="simpan grafik (butuh matplotlib)")
    a = p.parse_args()

    results, labels = [], []

    def process(name, rows):
        r = analyze(rows, a.axis)
        res = inertia(r["T0"], r["sigma_T"], a)
        report(name, r, res)
        results.append((r, res)); labels.append(name)

    if a.analyze:
        for path in a.analyze:
            name = os.path.basename(path)
            try:
                process(name, load_csv(path))
            except (RunError, OSError, ValueError, IndexError) as e:
                print(f"  {name}: GAGAL - {e}")
        summarize(results, a)
        if a.plot and results:
            plot(results, labels, next_path(a.label + "_analisis", "png"))
        return

    what = "robot utuh (roda dikunci ke badan)" if a.wheels_attached else "badan saja (tanpa roda)"
    print(f"Uji bandul: {what}, poros {a.pivot * 1000:.0f} mm dari sumbu roda.")
    print("Gantung robot, biarkan DIAM, lalu mulai.")
    if not wait_for_start():
        print("Dibatalkan.")
        return

    gyro = None
    try:
        gyro = Gyro()
        print("Mengukur bias giroskop 2 s, jangan disentuh...")
        bias = measure_bias(gyro)
        print("Tarik robot ke samping (kurang dari 10 derajat) lalu lepaskan.")
        rows = record(gyro, bias, a)
        path = next_path(a.label)
        save_csv(path, rows)
        print(f"Data disimpan: {path}")
        process(os.path.basename(path), rows)
    except KeyboardInterrupt:
        print("\nDihentikan dengan Ctrl+C")
    except RunError as e:
        print(f"GAGAL - {e}")
    except ImportError:
        print("GAGAL - modul smbus2 belum terpasang (pip install smbus2)")
    except OSError as e:
        print(f"GAGAL - tidak bisa membaca MPU6050 di bus {I2C_BUS} alamat 0x{MPU6050_ADDRESS:02X}: {e}")
    finally:
        if gyro is not None:
            gyro.close()

    summarize(results, a)
    if a.plot and results:
        plot(results, labels, next_path(a.label + "_grafik", "png"))


if __name__ == "__main__":
    main()