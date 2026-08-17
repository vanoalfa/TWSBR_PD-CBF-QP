"""
drive_awsd.py
=============
Program latihan dasar: menggerakkan robot pakai tombol W-A-S-D,
dengan badan robot DIKUNCI TEGAK PAKSA (bukan hasil kontrol PD/balancing).

Tujuan program ini:
- Belajar mekanisme keyboard input di PyBullet
- Belajar cara menggerakkan roda (kontrol kecepatan)
- Melihat spesifikasi fisik robot (massa, inersia) yang sudah ada di URDF
- Memverifikasi roda benar-benar berputar lewat getJointState()

CATATAN PENTING:
Gerakan di sini bersifat KINEMATIK (posisi robot digeser langsung lewat
kode), BUKAN hasil murni gaya fisika dari roda menapak lantai. Ini
simplifikasi yang disengaja supaya Anda bisa fokus belajar mekanisme
kontrol dulu, tanpa terganggu kerumitan fisika kontak & tilt yang
akan kita selesaikan di sesi balancing berikutnya.

Kontrol:
    W = maju
    S = mundur
    A = belok kiri (rotasi yaw)
    D = belok kanan (rotasi yaw)
    Ctrl+C di terminal = keluar
"""

import pybullet as p
import pybullet_data
import time
import os
import math


# ============================================================
# BAGIAN 1: KONFIGURASI
# ============================================================

# --- Spesifikasi motor DDSM115 (dari datasheet resmi) ---
RATED_TORQUE = 0.96     # Nm, torsi kontinu aman
STALL_TORQUE = 2.0      # Nm, torsi maksimum (locked-rotor)
NO_LOAD_SPEED_RPM = 200 # rpm, kecepatan tanpa beban
RATED_SPEED_RPM = 115   # rpm, kecepatan pada torsi rated

# Untuk mode driving ini, wheel kita gerakkan pakai VELOCITY_CONTROL,
# forcenya kita batasi ke STALL_TORQUE supaya tetap realistis
# terhadap kemampuan motor asli.
WHEEL_MAX_FORCE = STALL_TORQUE
WHEEL_DRIVE_VELOCITY = 15.0   # rad/s, kecepatan putar roda saat W/S ditekan (nilai bebas untuk demo visual)

# --- Kecepatan gerak robot (kinematik, bebas diatur untuk demo) ---
LINEAR_SPEED = 0.5      # meter/detik, kecepatan maju/mundur
YAW_SPEED = 1.0          # radian/detik, kecepatan belok

# --- Simulasi ---
GRAVITY = -9.8
SIM_FREQUENCY = 240
SIM_TIMESTEP = 1.0 / SIM_FREQUENCY

# --- Posisi & orientasi awal ---
START_POS = [0, 0, 0.04]
# Koreksi Y-up (Fusion) -> Z-up (PyBullet). Orientasi INI JUGA yang akan
# kita jadikan acuan "tegak" yang dikunci paksa sepanjang simulasi.
UPRIGHT_ORIENTATION_EULER = [math.pi / 2, 0, 0]

LEFT_WHEEL_PATTERN = "wheelL"
RIGHT_WHEEL_PATTERN = "wheelR"


# ============================================================
# BAGIAN 2: PATH HANDLING
# ============================================================

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)
URDF_PATH = os.path.join(PROJECT_ROOT, "urdf", "urdf", "robot.urdf")

if not os.path.exists(URDF_PATH):
    raise FileNotFoundError(f"URDF tidak ditemukan di: {URDF_PATH}")


# ============================================================
# BAGIAN 3: SETUP DUNIA & ROBOT
# ============================================================

def setup_world():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, GRAVITY)
    p.loadURDF("plane.urdf")


def load_robot():
    orn = p.getQuaternionFromEuler(UPRIGHT_ORIENTATION_EULER)
    robot_id = p.loadURDF(URDF_PATH, basePosition=START_POS, baseOrientation=orn, useFixedBase=False)
    print(f"Robot dimuat dengan ID: {robot_id}")
    return robot_id


def detect_wheel_joints(robot_id):
    num_joints = p.getNumJoints(robot_id)
    left_idx, right_idx = None, None

    for i in range(num_joints):
        info = p.getJointInfo(robot_id, i)
        child_link_name = info[12].decode("utf-8")
        if LEFT_WHEEL_PATTERN in child_link_name:
            left_idx = i
        elif RIGHT_WHEEL_PATTERN in child_link_name:
            right_idx = i

    if left_idx is None or right_idx is None:
        raise RuntimeError("Index joint roda tidak ditemukan otomatis.")

    print(f"Joint roda kiri: {left_idx} | Joint roda kanan: {right_idx}")
    return left_idx, right_idx


# ============================================================
# BAGIAN 4: INSPEKSI SPESIFIKASI FISIK ROBOT (dari URDF)
# ============================================================

def print_robot_specs(robot_id, left_idx, right_idx):
    """
    Menampilkan spesifikasi fisik robot yang SUDAH ADA di URDF
    (dihitung otomatis oleh Fusion 360 dari geometri & material CAD).
    Kita hanya membaca, tidak mengubah nilai ini di sini.
    """
    print("\n" + "=" * 55)
    print("SPESIFIKASI FISIK ROBOT (dari URDF)")
    print("=" * 55)

    # link_index -1 merujuk ke base_link
    base_dynamics = p.getDynamicsInfo(robot_id, -1)
    base_mass = base_dynamics[0]
    print(f"Massa base_link (badan robot) : {base_mass:.4f} kg")

    left_dynamics = p.getDynamicsInfo(robot_id, left_idx)
    right_dynamics = p.getDynamicsInfo(robot_id, right_idx)
    print(f"Massa roda kiri                : {left_dynamics[0]:.4f} kg")
    print(f"Massa roda kanan                : {right_dynamics[0]:.4f} kg")

    total_mass = base_mass + left_dynamics[0] + right_dynamics[0]
    print(f"Massa total robot               : {total_mass:.4f} kg")

    print("\nSPESIFIKASI MOTOR DDSM115 (datasheet, bukan dari URDF)")
    print(f"Rated torque (torsi aman kontinu) : {RATED_TORQUE} Nm")
    print(f"Locked-rotor torque (torsi maks)  : {STALL_TORQUE} Nm")
    print(f"No-load speed                     : {NO_LOAD_SPEED_RPM} rpm")
    print(f"Rated speed                       : {RATED_SPEED_RPM} rpm")
    print("=" * 55 + "\n")


# ============================================================
# BAGIAN 5: KUNCI ORIENTASI TEGAK (INTI PERMINTAAN ANDA)
# ============================================================

def lock_upright(robot_id, yaw):
    """
    Memaksa base robot SELALU dalam orientasi tegak (tidak ada roll/pitch),
    hanya yaw (arah hadap horizontal) yang boleh berubah sesuai kontrol
    A/D. Posisi (x, y, z) TETAP mengikuti hasil simulasi/kontrol kinematik
    kita di bagian gerak.

    Ini "curang" secara fisika (kita override physics engine setiap step),
    tapi itu memang tujuannya: menghilangkan variabel balancing dari
    persamaan, supaya fokus belajar hanya ke mekanisme gerak & kontrol.
    """
    pos, _ = p.getBasePositionAndOrientation(robot_id)

    # orientasi tegak dasar (koreksi Y-up->Z-up)
    base_upright = p.getQuaternionFromEuler(UPRIGHT_ORIENTATION_EULER)

    # tambahkan rotasi yaw (putar mengelilingi sumbu Z dunia) di atas orientasi dasar
    yaw_quat = p.getQuaternionFromEuler([0, 0, yaw])
    _, final_orn = p.multiplyTransforms([0, 0, 0], yaw_quat, [0, 0, 0], base_upright)

    p.resetBasePositionAndOrientation(robot_id, pos, final_orn)

    # netralkan kecepatan sudut supaya tidak ada 'sisa' rotasi dari physics
    # yang membuat koreksi ini terlihat bergetar
    p.resetBaseVelocity(robot_id, angularVelocity=[0, 0, 0])


# ============================================================
# BAGIAN 6: BACA INPUT KEYBOARD & HITUNG GERAK
# ============================================================

def read_keyboard_and_compute_motion(x, y, yaw, dt):
    """
    Membaca status tombol W/A/S/D dan menghitung posisi & yaw baru
    secara kinematik sederhana (bukan dari gaya fisika roda).
    """
    keys = p.getKeyboardEvents()

    moving_forward = False
    moving_backward = False

    if ord('w') in keys and keys[ord('w')] & p.KEY_IS_DOWN:
        moving_forward = True
    if ord('s') in keys and keys[ord('s')] & p.KEY_IS_DOWN:
        moving_backward = True
    if ord('a') in keys and keys[ord('a')] & p.KEY_IS_DOWN:
        yaw += YAW_SPEED * dt
    if ord('d') in keys and keys[ord('d')] & p.KEY_IS_DOWN:
        yaw -= YAW_SPEED * dt

    direction = 0.0
    if moving_forward:
        direction = 1.0
    elif moving_backward:
        direction = -1.0

    # arah maju robot mengikuti yaw saat ini (bergerak searah hadapnya)
    x += direction * LINEAR_SPEED * dt * math.cos(-yaw)
    y += direction * LINEAR_SPEED * dt * math.sin(-yaw)

    return x, y, yaw, direction


# ============================================================
# BAGIAN 7: PUTAR RODA SECARA VISUAL SESUAI ARAH GERAK
# ============================================================

def spin_wheels_visual(robot_id, left_idx, right_idx, direction):
    """
    Memutar roda pakai VELOCITY_CONTROL, force dibatasi WHEEL_MAX_FORCE
    (mengikuti torsi maksimum motor asli DDSM115) supaya nilainya tetap
    realistis, meski gerak robot secara keseluruhan di program ini
    kinematik (bukan murni hasil fisika roda-lantai).
    """
    target_velocity = direction * WHEEL_DRIVE_VELOCITY

    p.setJointMotorControl2(
        robot_id, left_idx, p.VELOCITY_CONTROL,
        targetVelocity=target_velocity, force=WHEEL_MAX_FORCE
    )
    p.setJointMotorControl2(
        robot_id, right_idx, p.VELOCITY_CONTROL,
        targetVelocity=target_velocity, force=WHEEL_MAX_FORCE
    )


# ============================================================
# BAGIAN 8: VERIFIKASI RODA BENAR-BENAR BERPUTAR
# ============================================================

def get_wheel_status(robot_id, left_idx, right_idx):
    left_state = p.getJointState(robot_id, left_idx)
    right_state = p.getJointState(robot_id, right_idx)
    return {
        "left_velocity": left_state[1],
        "right_velocity": right_state[1],
        "left_torque": left_state[3],
        "right_torque": right_state[3],
    }


# ============================================================
# BAGIAN 9: LOOP UTAMA
# ============================================================

def run(robot_id, left_idx, right_idx):
    print("Kontrol: W=maju, S=mundur, A=belok kiri, D=belok kanan")
    print("Klik jendela simulasi PyBullet dulu supaya keyboard terbaca.")
    print("Tekan Ctrl+C di terminal untuk keluar.\n")

    x, y, yaw = START_POS[0], START_POS[1], 0.0
    step_count = 0

    try:
        while True:
            x, y, yaw, direction = read_keyboard_and_compute_motion(x, y, yaw, SIM_TIMESTEP)

            lock_upright(robot_id, yaw)
            # update posisi x,y hasil kinematik (z tetap dari START_POS)
            pos, orn = p.getBasePositionAndOrientation(robot_id)
            p.resetBasePositionAndOrientation(robot_id, [x, y, pos[2]], orn)

            spin_wheels_visual(robot_id, left_idx, right_idx, direction)

            p.stepSimulation()
            time.sleep(SIM_TIMESTEP)

            step_count += 1
            if step_count % 60 == 0:  # tiap ~0.25 detik
                status = get_wheel_status(robot_id, left_idx, right_idx)
                print(
                    f"pos=({x:5.2f},{y:5.2f}) yaw={math.degrees(yaw):6.1f}deg | "
                    f"v_roda_kiri={status['left_velocity']:6.2f} rad/s | "
                    f"v_roda_kanan={status['right_velocity']:6.2f} rad/s"
                )

    except KeyboardInterrupt:
        print("\nDihentikan oleh user.")
    finally:
        p.disconnect()


# ============================================================
# ENTRY POINT
# ============================================================

def main():
    setup_world()
    robot_id = load_robot()
    left_idx, right_idx = detect_wheel_joints(robot_id)
    print_robot_specs(robot_id, left_idx, right_idx)
    run(robot_id, left_idx, right_idx)


if __name__ == "__main__":
    main()