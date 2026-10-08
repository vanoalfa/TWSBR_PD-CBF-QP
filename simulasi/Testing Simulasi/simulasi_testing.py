import pybullet as p
import pybullet_data
import time
import os
import math

# --- Bagian path handling (paling penting) ---
# __file__ = lokasi script ini sendiri, jadi path selalu benar
# tidak peduli dari folder mana Anda menjalankan python
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)  # naik satu level dari simulasi/ ke root proyek

URDF_PATH = os.path.join(PROJECT_ROOT, "urdf", "urdf", "robot.urdf")

# cek dulu file benar-benar ada sebelum load,
# supaya error yang muncul jelas ("file tidak ketemu"), bukan error PyBullet yang ambigu
if not os.path.exists(URDF_PATH):
    raise FileNotFoundError(f"URDF tidak ditemukan di: {URDF_PATH}")

print(f"Memuat URDF dari: {URDF_PATH}")

# --- Setup simulasi ---
physicsClient = p.connect(p.GUI)  # p.DIRECT kalau tidak butuh visual (lebih cepat)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # supaya plane.urdf bawaan pybullet ketemu
p.setGravity(0, 0, -9.8)

# load lantai bawaan pybullet sebagai referensi
plane_id = p.loadURDF("plane.urdf")

# --- Load robot ---
start_pos = [0, 0, 0.3]       # posisi awal (x, y, z) — sesuaikan tinggi z dengan robot Anda
start_orientation = p.getQuaternionFromEuler([math.pi/2, 0, 0])

robot_id = p.loadURDF(
    URDF_PATH,
    basePosition=start_pos,
    baseOrientation=start_orientation,
    useFixedBase=False  # False karena robot self-balancing harus bebas jatuh/bergerak
)



print(f"Robot berhasil dimuat dengan ID: {robot_id}")

# --- Cek info dasar robot (jumlah joint, dsb) ---
num_joints = p.getNumJoints(robot_id)
print(f"Jumlah joint pada robot: {num_joints}")
for i in range(num_joints):
    joint_info = p.getJointInfo(robot_id, i)
    print(f"  Joint {i}: nama={joint_info[1].decode('utf-8')}, tipe={joint_info[2]}")

num_joints = p.getNumJoints(robot_id)
wheel_left_idx = None
wheel_right_idx = None

for i in range(num_joints):
    joint_info = p.getJointInfo(robot_id, i)
    joint_name = joint_info[1].decode('utf-8')
    child_link_name = joint_info[12].decode('utf-8')  # nama link anak
    print(f"Joint {i}: nama_joint={joint_name}, child_link={child_link_name}")

    if "wheelL" in child_link_name:
        wheel_left_idx = i
    elif "wheelR" in child_link_name:
        wheel_right_idx = i

if wheel_left_idx is None or wheel_right_idx is None:
    raise RuntimeError(
        f"Index joint roda tidak ketemu otomatis. "
        f"wheel_left_idx={wheel_left_idx}, wheel_right_idx={wheel_right_idx}. "
        f"Cek nama child_link di output di atas."
    )

print(f"\nJoint roda kiri: index {wheel_left_idx}")
print(f"Joint roda kanan: index {wheel_right_idx}\n")


# --- Loop simulasi kosong (tanpa kontrol apa pun dulu) ---
try:
    while True:
        p.stepSimulation()
        time.sleep(1/240)  # 240 Hz, standar PyBullet
except KeyboardInterrupt:
    print("Simulasi dihentikan oleh user (Ctrl+C)")
finally:
    p.disconnect()
