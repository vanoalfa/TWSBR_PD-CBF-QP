"""
simulasi.py
============
Simulasi PD Control untuk Two-Wheeled Self-Balancing Robot (TWSBR) di PyBullet.

Alur program:
1. Setup path & load URDF robot ke dalam dunia simulasi
2. Deteksi otomatis index joint roda kiri/kanan (berdasarkan child_link_name)
3. Nonaktifkan motor constraint default PyBullet
4. Loop kontrol tertutup: baca state -> hitung PD -> kirim torsi -> step simulasi
5. Logging berkala untuk monitoring & debugging

Catatan penting (WAJIB diverifikasi saat pertama kali dijalankan):
- AXIS_TILT_INDEX (index sumbu Euler untuk sudut tilt) adalah ASUMSI awal.
- TORQUE_SIGN (arah torsi) adalah ASUMSI awal.
  Lihat bagian "PANDUAN TUNING & DEBUGGING" di akhir file ini.
"""

import pybullet as p
import pybullet_data
import time
import os
import math


# ============================================================
# BAGIAN 1: KONFIGURASI & PARAMETER
# ============================================================
# Semua angka "yang bisa diubah" dikumpulkan di sini,
# supaya tuning tidak perlu mengubah logika program di bagian bawah.

# --- Parameter PD Control (titik awal, WAJIB di-tuning) ---
KP = 50.0              # Gain proporsional: respons terhadap besar sudut error
KD = 2.0               # Gain derivatif: peredam osilasi (respons terhadap kecepatan sudut)
MAX_TORQUE = 10.0      # Batas saturasi torsi (Nm) -- sesuaikan dengan spesifikasi DDSM115

# --- Parameter simulasi ---
GRAVITY = -9.8
SIM_FREQUENCY = 240        # Hz, standar PyBullet
SIM_TIMESTEP = 1.0 / SIM_FREQUENCY
LOG_INTERVAL_STEPS = 120   # cetak log tiap 120 step (~0.5 detik pada 240 Hz)

# --- Posisi & orientasi awal robot ---
START_POS = [0, 0, 0.3]                                    # (x, y, z) meter -- sesuaikan tinggi robot Anda
START_ORIENTATION = p.getQuaternionFromEuler([math.pi/2, 0, 0])  # koreksi Y-up (Fusion) -> Z-up (PyBullet)

# --- Konfigurasi sumbu tilt (ASUMSI AWAL, lihat panduan debugging di akhir file) ---
AXIS_TILT_INDEX = 1        # index sudut Euler yang mewakili sudut tilt robot (0=roll, 1=pitch, 2=yaw)
TORQUE_SIGN = 1            # kalikan -1 di sini kalau arah torsi terbukti terbalik saat uji coba

# --- Pattern nama child_link untuk deteksi otomatis joint roda ---
LEFT_WHEEL_PATTERN = "wheelL"
RIGHT_WHEEL_PATTERN = "wheelR"


# ============================================================
# BAGIAN 2: PATH HANDLING
# ============================================================
# Path dihitung berdasarkan lokasi file script ini sendiri (__file__),
# BUKAN berdasarkan folder tempat perintah `python` dijalankan.
# Ini membuat script tetap benar dijalankan dari folder mana pun.

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)
URDF_PATH = os.path.join(PROJECT_ROOT, "urdf", "urdf", "robot.urdf")

if not os.path.exists(URDF_PATH):
    raise FileNotFoundError(f"URDF tidak ditemukan di: {URDF_PATH}")


# ============================================================
# BAGIAN 3: SETUP DUNIA SIMULASI
# ============================================================

def setup_world():
    """Inisialisasi physics client, gravitasi, dan lantai."""
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, GRAVITY)
    p.loadURDF("plane.urdf")
    print("Dunia simulasi siap: gravitasi & lantai terpasang.")


# ============================================================
# BAGIAN 4: LOAD ROBOT
# ============================================================

def load_robot():
    """Load URDF robot ke posisi & orientasi awal yang sudah dikoreksi Z-up."""
    robot_id = p.loadURDF(
        URDF_PATH,
        basePosition=START_POS,
        baseOrientation=START_ORIENTATION,
        useFixedBase=False,   # False -> robot bebas bergerak/jatuh, wajib untuk self-balancing
    )
    print(f"Robot berhasil dimuat dengan ID: {robot_id}")
    return robot_id


# ============================================================
# BAGIAN 5: DETEKSI JOINT RODA (OTOMATIS)
# ============================================================

def detect_wheel_joints(robot_id):
    """
    Mendeteksi index joint roda kiri & kanan secara otomatis,
    berdasarkan nama CHILD LINK (bukan nama joint -- karena Fusion
    menamai joint secara generik seperti 'Revolute 1', 'Revolute 2').

    getJointInfo() index penting yang dipakai:
        [1]  -> nama joint (bytes)
        [2]  -> tipe joint (0 = revolute)
        [12] -> nama child link (bytes)
    """
    num_joints = p.getNumJoints(robot_id)
    print(f"\nJumlah joint terdeteksi: {num_joints}")

    left_idx = None
    right_idx = None

    for i in range(num_joints):
        info = p.getJointInfo(robot_id, i)
        joint_name = info[1].decode("utf-8")
        child_link_name = info[12].decode("utf-8")

        print(f"  Joint {i}: nama_joint='{joint_name}', child_link='{child_link_name}'")

        if LEFT_WHEEL_PATTERN in child_link_name:
            left_idx = i
        elif RIGHT_WHEEL_PATTERN in child_link_name:
            right_idx = i

    if left_idx is None or right_idx is None:
        raise RuntimeError(
            f"Index joint roda tidak ditemukan otomatis "
            f"(left_idx={left_idx}, right_idx={right_idx}). "
            f"Cek nama child_link di atas dan sesuaikan "
            f"LEFT_WHEEL_PATTERN / RIGHT_WHEEL_PATTERN."
        )

    print(f"\n>> Joint roda kiri  : index {left_idx}")
    print(f">> Joint roda kanan : index {right_idx}\n")

    return left_idx, right_idx


# ============================================================
# BAGIAN 6: PERSIAPAN MOTOR UNTUK KONTROL TORSI
# ============================================================

def disable_default_motors(robot_id, left_idx, right_idx):
    """
    PyBullet secara default memasang velocity motor dengan target 0
    di setiap joint (semacam 'rem' otomatis bawaan). Ini HARUS
    dimatikan dulu (force=0) sebelum kita mengirim torsi kontrol kita
    sendiri lewat TORQUE_CONTROL -- kalau tidak, torsi kita akan
    "dilawan" oleh rem bawaan ini.
    """
    p.setJointMotorControl2(robot_id, left_idx, p.VELOCITY_CONTROL, force=0)
    p.setJointMotorControl2(robot_id, right_idx, p.VELOCITY_CONTROL, force=0)
    print("Motor constraint default PyBullet sudah dinonaktifkan.")


# ============================================================
# BAGIAN 7: STATE EXTRACTION
# ============================================================

def get_tilt_state(robot_id):
    """
    Mengambil sudut tilt (theta) dan kecepatan sudut tilt (theta_dot)
    dari base robot.

    Return:
        theta      : sudut tilt saat ini (radian)
        theta_dot  : kecepatan sudut tilt saat ini (radian/detik)
    """
    _, orientation_quat = p.getBasePositionAndOrientation(robot_id)
    euler = p.getEulerFromQuaternion(orientation_quat)
    theta = euler[AXIS_TILT_INDEX]

    _, angular_velocity = p.getBaseVelocity(robot_id)
    theta_dot = angular_velocity[AXIS_TILT_INDEX]

    return theta, theta_dot


# ============================================================
# BAGIAN 8: PD CONTROLLER
# ============================================================

def pd_control(theta, theta_dot):
    """
    Hukum kontrol PD standar untuk balancing:
        u = Kp * error + Kd * error_dot
    dengan target sudut = 0 (robot tegak lurus vertikal).

    Torsi hasil disaturasi ke rentang [-MAX_TORQUE, MAX_TORQUE]
    untuk mensimulasikan batas fisik aktuator (DDSM115).
    """
    error = 0.0 - theta
    error_dot = 0.0 - theta_dot

    torque = TORQUE_SIGN * (KP * error + KD * error_dot)
    torque = max(-MAX_TORQUE, min(MAX_TORQUE, torque))

    return torque


# ============================================================
# BAGIAN 9: KIRIM TORSI KE RODA
# ============================================================

def apply_wheel_torque(robot_id, left_idx, right_idx, torque):
    """
    Mengirim torsi yang sama ke kedua roda (gerak maju/mundur bersama,
    sesuai kebutuhan balancing dasar -- belum ada yaw/turning control).
    """
    p.setJointMotorControl2(robot_id, left_idx, p.TORQUE_CONTROL, force=torque)
    p.setJointMotorControl2(robot_id, right_idx, p.TORQUE_CONTROL, force=torque)


# ============================================================
# BAGIAN 10: LOOP SIMULASI UTAMA (CLOSED-LOOP)
# ============================================================

def run_simulation(robot_id, left_idx, right_idx):
    """
    Loop kontrol tertutup:
        baca state -> hitung PD -> kirim torsi -> step fisika -> ulangi

    Loop berhenti rapi saat Ctrl+C ditekan (KeyboardInterrupt),
    dan selalu menutup koneksi PyBullet di blok finally.
    """
    print("Memulai loop simulasi. Tekan Ctrl+C untuk berhenti.\n")

    step_count = 0
    try:
        while True:
            theta, theta_dot = get_tilt_state(robot_id)
            torque = pd_control(theta, theta_dot)
            apply_wheel_torque(robot_id, left_idx, right_idx, torque)

            p.stepSimulation()
            time.sleep(SIM_TIMESTEP)

            step_count += 1
            if step_count % LOG_INTERVAL_STEPS == 0:
                print(
                    f"theta={math.degrees(theta):7.2f} deg | "
                    f"theta_dot={theta_dot:7.3f} rad/s | "
                    f"torque={torque:6.2f} Nm"
                )

    except KeyboardInterrupt:
        print("\nSimulasi dihentikan oleh user (Ctrl+C).")

    finally:
        p.disconnect()
        print("Koneksi PyBullet ditutup dengan bersih.")


# ============================================================
# BAGIAN 11: ENTRY POINT
# ============================================================

def main():
    setup_world()
    robot_id = load_robot()
    left_idx, right_idx = detect_wheel_joints(robot_id)
    disable_default_motors(robot_id, left_idx, right_idx)
    run_simulation(robot_id, left_idx, right_idx)


if __name__ == "__main__":
    main()


# ============================================================
# PANDUAN TUNING & DEBUGGING (baca ini setelah menjalankan program)
# ============================================================
#
# 1. THETA SELALU ~0 PADAHAL ROBOT TERLIHAT MIRING
#    -> AXIS_TILT_INDEX salah. Ganti dari 1 ke 0 (atau sebaliknya)
#       di BAGIAN 1, lalu jalankan ulang.
#
# 2. ROBOT MALAH SEMAKIN CEPAT JATUH (bukan melawan kemiringan)
#    -> Arah torsi terbalik. Ganti TORQUE_SIGN dari 1 menjadi -1
#       di BAGIAN 1.
#
# 3. ROBOT BEROSILASI / BERGOYANG TERUS TANPA STABIL
#    -> KD terlalu kecil relatif terhadap KP. Naikkan KD bertahap
#       (misal 2.0 -> 4.0 -> 6.0), amati apakah osilasi meredam.
#
# 4. ROBOT JATUH PERLAHAN TANPA PERLAWANAN BERARTI
#    -> KP terlalu kecil. Naikkan bertahap (misal 50 -> 80 -> 120).
#
# 5. ROBOT BERGETAR CEPAT (high-frequency jitter) DI SEKITAR TEGAK
#    -> KP/KD terlalu besar relatif terhadap MAX_TORQUE, atau
#       torsi terlalu sering menyentuh saturasi. Turunkan KP/KD
#       sedikit, atau naikkan MAX_TORQUE jika masih dalam batas
#       realistis spesifikasi motor DDSM115.
#
# Urutan tuning yang disarankan:
#   (a) Pastikan dulu axis & arah torsi benar (poin 1 & 2) --
#       robot HARUS terlihat "berusaha melawan" jatuh, walau
#       belum stabil, sebelum masuk ke tuning gain.
#   (b) Baru setelah axis & arah benar, tuning KP dulu sampai
#       robot bisa menahan diri (meski berosilasi).
#   (c) Terakhir naikkan KD untuk meredam osilasi tersebut.
# ============================================================
