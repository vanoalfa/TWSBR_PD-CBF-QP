"""
==========================================================================
 TWO-WHEELED SELF-BALANCING ROBOT (TWSBR) - PHASE 1
 WASD Keyboard Control + Upright Lock (TANPA PID Balancing)
==========================================================================

Tahap ini BELUM menggunakan PID self-balancing. Base robot dipertahankan
tegak (roll = 0, pitch = 0) secara kinematik setiap simulation step,
sementara translasi X/Y dan rotasi yaw tetap bebas sehingga robot dapat
maju, mundur, dan berbelok menggunakan differential drive.

Alur data:
    Keyboard -> WASD command -> linear/angular command
             -> differential drive -> target wheel velocity
             -> velocity ramp (smoothing) -> PyBullet motor control
             -> stepSimulation() -> keep_robot_upright()

Struktur ini sengaja dibuat modular agar mudah dikembangkan ke:
    Phase 2: PID self-balancing (menggantikan keep_robot_upright)
    Phase 3: WASD -> velocity/setpoint untuk PID
    Phase 4: PID balancing + turning bersamaan
==========================================================================
"""

import pybullet as p
import pybullet_data
import time
import math
import os


# ==========================================================================
# CONFIGURATION
# ==========================================================================

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)
URDF_PATH = os.path.join(PROJECT_ROOT, "urdf", "urdf", "robot.urdf")

if not os.path.exists(URDF_PATH):
    raise FileNotFoundError(f"URDF tidak ditemukan di: {URDF_PATH}")

# Nama joint HARUS PERSIS sama dengan yang ada di file URDF
LEFT_WHEEL_JOINT_NAME = "Revolute 1"
RIGHT_WHEEL_JOINT_NAME = "Revolute 2"

# --- Parameter gerak (boleh di-tuning) ---
MAX_WHEEL_SPEED = 10.0        # rad/s, kecepatan roda maksimum saat maju/mundur
TURN_SPEED = 6.0              # rad/s, kecepatan roda untuk belok di tempat
MAX_MOTOR_FORCE = 30.0        # Nm, torsi maksimum motor (force di setJointMotorControl2)
WHEEL_ACCELERATION = 20.0     # rad/s^2, laju perubahan kecepatan roda (velocity ramp)

# Jika arah maju/mundur terbalik (W membuat robot mundur), ubah nilai ini
# menjadi -1.0. Tidak perlu mengubah bagian kode lain.
MOTOR_DIRECTION = 1.0

# --- Friction ---
WHEEL_LATERAL_FRICTION = 1.0
WHEEL_ROLLING_FRICTION = 0.001
WHEEL_SPINNING_FRICTION = 0.001
GROUND_LATERAL_FRICTION = 1.0

# --- Posisi awal robot ---
# initial_z harus disesuaikan dengan radius roda TWSBR_URDF Anda agar roda
# tepat menyentuh ground, bukan menembus atau melayang. 0.15 hanyalah nilai
# awal yang aman untuk dicoba; sesuaikan jika robot muncul menembus lantai
# atau melayang terlalu tinggi saat program dijalankan.
INITIAL_Z = 0.15
INITIAL_POSITION = [0.0, 0.0, INITIAL_Z]
INITIAL_YAW = 0.0

TIME_STEP = 1.0 / 240.0
PRINT_INTERVAL = 0.5   # detik, interval print info debug


# ==========================================================================
# HELPER FUNCTIONS
# ==========================================================================

def find_joint_by_name(body_id, joint_name):
    """
    Mencari index joint berdasarkan nama persis seperti di URDF.
    JANGAN mengasumsikan index joint selalu 0 dan 1.
    Mengembalikan index joint (int) atau None jika tidak ditemukan.
    """
    num_joints = p.getNumJoints(body_id)
    for i in range(num_joints):
        joint_info = p.getJointInfo(body_id, i)
        name_in_urdf = joint_info[1].decode("utf-8")
        if name_in_urdf == joint_name:
            return i
    return None


def disable_default_motor(body_id, joint_index):
    """
    Menonaktifkan motor velocity control default yang otomatis dipasang
    PyBullet pada setiap joint saat URDF di-load. Wajib dipanggil sebelum
    kontrol utama (set_wheel_velocity) digunakan, jika tidak motor default
    (force tidak nol) akan menahan/mengganggu gerakan roda.
    """
    p.setJointMotorControl2(
        body_id,
        joint_index,
        controlMode=p.VELOCITY_CONTROL,
        targetVelocity=0,
        force=0
    )


def set_wheel_velocity(body_id, left_joint, right_joint,
                        left_velocity, right_velocity,
                        max_force=MAX_MOTOR_FORCE):
    """
    Mengirim target velocity ke motor roda kiri dan kanan menggunakan
    VELOCITY_CONTROL. force menentukan torsi maksimum motor.
    """
    p.setJointMotorControl2(
        body_id,
        jointIndex=left_joint,
        controlMode=p.VELOCITY_CONTROL,
        targetVelocity=left_velocity,
        force=max_force
    )
    p.setJointMotorControl2(
        body_id,
        jointIndex=right_joint,
        controlMode=p.VELOCITY_CONTROL,
        targetVelocity=right_velocity,
        force=max_force
    )


def keep_robot_upright(body_id):
    """
    Mengunci roll dan pitch base robot menjadi 0 setiap simulation step,
    tanpa mengunci posisi X/Y ataupun rotasi yaw.

    Metode yang digunakan (kinematic upright lock):
      1. Baca posisi & orientasi base saat ini (JANGAN direset ke [0,0,z]
         tetap, gunakan posisi aktual agar X/Y tidak "teleport" balik).
      2. Konversi quaternion -> Euler (roll, pitch, yaw).
      3. Buat ulang quaternion hanya dari yaw (roll = pitch = 0).
      4. Terapkan kembali dengan resetBasePositionAndOrientation
         menggunakan posisi ASLI (bukan posisi awal) dan orientasi baru.
      5. Sebagai tambahan kecil untuk stabilitas: nolkan komponen roll/pitch
         dari angular velocity (biarkan yaw rate tetap jalan), supaya base
         tidak terus "melawan" koreksi orientasi di step berikutnya.

    Ini murni koreksi kinematik (bukan dinamik/PID), sesuai permintaan
    Phase 1: sederhana, stabil, dan mudah diganti dengan PID di Phase 2.
    """
    position, orientation = p.getBasePositionAndOrientation(body_id)
    roll, pitch, yaw = p.getEulerFromQuaternion(orientation)

    upright_orientation = p.getQuaternionFromEuler([math.pi/2, 0.0, yaw])

    p.resetBasePositionAndOrientation(
        body_id,
        position,          # posisi aktual (X, Y, Z) tetap dipertahankan
        upright_orientation
    )

    # Nolkan angular velocity roll/pitch, sisakan yaw rate agar belok tetap halus
    linear_velocity, angular_velocity = p.getBaseVelocity(body_id)
    p.resetBaseVelocity(
        body_id,
        linearVelocity=linear_velocity,
        angularVelocity=[0.0, 0.0, angular_velocity[2]]
    )


def move_towards(current, target, max_delta):
    """
    Velocity ramp sederhana: menggerakkan 'current' menuju 'target'
    dengan langkah maksimum 'max_delta' per pemanggilan. Digunakan agar
    perubahan kecepatan roda tidak menyentak (smooth acceleration).
    """
    if current < target:
        return min(current + max_delta, target)
    else:
        return max(current - max_delta, target)


def reset_robot(body_id, left_joint, right_joint):
    """
    Mengembalikan robot ke posisi awal (INITIAL_POSITION, INITIAL_YAW),
    menolkan seluruh kecepatan base dan roda.
    Mengembalikan (current_left_vel, current_right_vel) = (0.0, 0.0)
    agar variabel velocity ramp di main loop ikut direset.
    """
    reset_orientation = p.getQuaternionFromEuler([0.0, 0.0, INITIAL_YAW])

    p.resetBasePositionAndOrientation(
        body_id,
        INITIAL_POSITION,
        reset_orientation
    )
    p.resetBaseVelocity(
        body_id,
        linearVelocity=[0.0, 0.0, 0.0],
        angularVelocity=[0.0, 0.0, 0.0]
    )

    p.resetJointState(body_id, left_joint, targetValue=0.0, targetVelocity=0.0)
    p.resetJointState(body_id, right_joint, targetValue=0.0, targetVelocity=0.0)

    return 0.0, 0.0


# ==========================================================================
# INITIALIZE PYBULLET
# ==========================================================================

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setTimeStep(TIME_STEP)
p.setRealTimeSimulation(0)


# ==========================================================================
# LOAD ENVIRONMENT
# ==========================================================================

plane_id = p.loadURDF("plane.urdf")
p.changeDynamics(plane_id, -1, lateralFriction=GROUND_LATERAL_FRICTION)


# ==========================================================================
# LOAD ROBOT
# ==========================================================================

try:
    robot_id = p.loadURDF(
        URDF_PATH,
        basePosition=INITIAL_POSITION,
        baseOrientation=p.getQuaternionFromEuler([0.0, 0.0, INITIAL_YAW]),
        useFixedBase=False
    )
except p.error as e:
    print(f"[ERROR] Gagal load URDF di '{URDF_PATH}': {e}")
    print("Pastikan file URDF (dan mesh-nya) berada di folder yang sama dengan script ini.")
    p.disconnect()
    raise SystemExit(1)


# ==========================================================================
# FIND JOINTS BY NAME
# ==========================================================================

print("========================================")
print(" Scanning joints...")
print("========================================")
num_joints = p.getNumJoints(robot_id)
for i in range(num_joints):
    info = p.getJointInfo(robot_id, i)
    print(f"  joint index {i} -> name: {info[1].decode('utf-8')}")

left_wheel_joint = find_joint_by_name(robot_id, LEFT_WHEEL_JOINT_NAME)
right_wheel_joint = find_joint_by_name(robot_id, RIGHT_WHEEL_JOINT_NAME)

if left_wheel_joint is None or right_wheel_joint is None:
    print("[ERROR] Salah satu atau kedua joint roda tidak ditemukan di URDF.")
    print(f"  Mencari: '{LEFT_WHEEL_JOINT_NAME}' dan '{RIGHT_WHEEL_JOINT_NAME}'")
    p.disconnect()
    raise SystemExit(1)

print("----------------------------------------")
print(f" {LEFT_WHEEL_JOINT_NAME} -> joint index {left_wheel_joint}")
print(f" {RIGHT_WHEEL_JOINT_NAME} -> joint index {right_wheel_joint}")
print("----------------------------------------")


# ==========================================================================
# DISABLE DEFAULT JOINT MOTORS
# ==========================================================================

disable_default_motor(robot_id, left_wheel_joint)
disable_default_motor(robot_id, right_wheel_joint)


# ==========================================================================
# WHEEL & GROUND FRICTION
# ==========================================================================

for wheel_joint in (left_wheel_joint, right_wheel_joint):
    p.changeDynamics(
        robot_id,
        wheel_joint,
        lateralFriction=WHEEL_LATERAL_FRICTION,
        rollingFriction=WHEEL_ROLLING_FRICTION,
        spinningFriction=WHEEL_SPINNING_FRICTION
    )


# ==========================================================================
# CAMERA
# ==========================================================================

p.resetDebugVisualizerCamera(
    cameraDistance=1.5,
    cameraYaw=45,
    cameraPitch=-25,
    cameraTargetPosition=[0, 0, 0.2]
)


# ==========================================================================
# DEBUG INFO
# ==========================================================================

print("========================================")
print(" TWO WHEELED ROBOT - PYBULLET CONTROL")
print("========================================")
print("")
print("Controls:")
print("W     : Forward")
print("S     : Backward")
print("A     : Turn Left")
print("D     : Turn Right")
print("SPACE : Stop")
print("R     : Reset")
print("ESC   : Exit")
print("")
print(f"Left Wheel Joint  : {LEFT_WHEEL_JOINT_NAME} (index {left_wheel_joint})")
print(f"Right Wheel Joint : {RIGHT_WHEEL_JOINT_NAME} (index {right_wheel_joint})")
print("========================================")


# ==========================================================================
# MAIN LOOP
# ==========================================================================

current_left_vel = 0.0
current_right_vel = 0.0
last_print_time = time.time()

ESC_KEY_CODE = 27  # ASCII Escape

while p.isConnected():

    keys = p.getKeyboardEvents()

    # ---- ESC: keluar ----
    if ESC_KEY_CODE in keys and (keys[ESC_KEY_CODE] & p.KEY_WAS_TRIGGERED):
        print("ESC ditekan, keluar dari simulasi...")
        break

    # ---- R: reset posisi robot ----
    if ord('r') in keys and (keys[ord('r')] & p.KEY_WAS_TRIGGERED):
        current_left_vel, current_right_vel = reset_robot(
            robot_id, left_wheel_joint, right_wheel_joint
        )
        print("Robot direset ke posisi awal.")

    # ---- Baca status tombol (bisa ditahan / held) ----
    w_down = ord('w') in keys and (keys[ord('w')] & p.KEY_IS_DOWN)
    s_down = ord('s') in keys and (keys[ord('s')] & p.KEY_IS_DOWN)
    a_down = ord('a') in keys and (keys[ord('a')] & p.KEY_IS_DOWN)
    d_down = ord('d') in keys and (keys[ord('d')] & p.KEY_IS_DOWN)
    space_down = ord(' ') in keys and (keys[ord(' ')] & p.KEY_IS_DOWN)

    # ---- Differential drive command ----
    linear_command = 0.0
    angular_command = 0.0

    if space_down:
        # SPACE menimpa (override) semua perintah gerak lainnya -> berhenti
        linear_command = 0.0
        angular_command = 0.0
    else:
        if w_down and not s_down:
            linear_command = MAX_WHEEL_SPEED
        elif s_down and not w_down:
            linear_command = -MAX_WHEEL_SPEED

        if a_down and not d_down:
            angular_command = TURN_SPEED
        elif d_down and not a_down:
            angular_command = -TURN_SPEED

    # left = linear - angular, right = linear + angular
    # (kombinasi W/S dengan A/D otomatis tergabung lewat linear & angular command)
    left_target = linear_command - angular_command
    right_target = linear_command + angular_command

    # Clamp agar tidak melebihi batas kecepatan roda
    left_target = max(-MAX_WHEEL_SPEED, min(MAX_WHEEL_SPEED, left_target))
    right_target = max(-MAX_WHEEL_SPEED, min(MAX_WHEEL_SPEED, right_target))

    # Koreksi arah motor jika W/S ternyata terbalik di simulasi
    left_target *= MOTOR_DIRECTION
    right_target *= MOTOR_DIRECTION

    # ---- Smooth acceleration (velocity ramp) ----
    max_delta = WHEEL_ACCELERATION * TIME_STEP
    current_left_vel = move_towards(current_left_vel, left_target, max_delta)
    current_right_vel = move_towards(current_right_vel, right_target, max_delta)

    # ---- Kirim ke motor ----
    set_wheel_velocity(
        robot_id, left_wheel_joint, right_wheel_joint,
        current_left_vel, current_right_vel
    )

    # ---- Kunci roll & pitch, biarkan X/Y/yaw bebas ----
    keep_robot_upright(robot_id)

    # ---- Step simulasi ----
    p.stepSimulation()
    time.sleep(TIME_STEP)

    # ---- Debug print berkala ----
    now = time.time()
    if now - last_print_time >= PRINT_INTERVAL:
        pos, orn = p.getBasePositionAndOrientation(robot_id)
        _, _, yaw = p.getEulerFromQuaternion(orn)
        print(
            f"Pos=({pos[0]:+.2f}, {pos[1]:+.2f}, {pos[2]:+.2f})  "
            f"Yaw={math.degrees(yaw):+6.1f} deg  "
            f"L={current_left_vel:+5.2f} rad/s  R={current_right_vel:+5.2f} rad/s"
        )
        last_print_time = now

p.disconnect()