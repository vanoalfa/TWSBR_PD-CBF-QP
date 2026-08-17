#Program Simulasi
import pybullet as p
import pybullet_data
import time
import os
import math

# ======================
# KONFIGURASI DAN PARAMETER
# ======================
# Semua angka "yang bisa diubah" dikumpulkan di sini,
# supaya tuning tidak perlu mengubah logika program di bagian bawah.

# ---- PD -----
# Kp = 50.0
# Kd = 3.0

# ---- CBF-QP ----
# ALPHA_1 = 5
# ALPHA_2 = 5

# ---- Parameter Simulasi ----
GRAVITY = -9.8
SIM_FREQUENCY = 240        # Hz, standar PyBullet
SIM_TIMESTEP = 1.0 / SIM_FREQUENCY
LOG_INTERVAL_STEPS = 120   # cetak log tiap 120 step (~0.5 detik pada 240 Hz)

# ---- Posisi dan Orientasi ----
#  ---- Robot ----
START_POS_ROB0T = [0, 0, 0.04]                                         # Inisialisasi posisi robot
START_ORIENTATION_ROBOT = p.getQuaternionFromEuler([math.pi/2, 0, 0])  # koreksi Y-up (Fusion) -> Z-up (PyBullet)
#  ---- Rintangan ----
START_POS_RINTANGAN = [0, 0, 0]
START_ORIENTATION_RINTANGAN = [0, 0, 0]

# --- Konfigurasi sumbu tilt (ASUMSI AWAL, lihat panduan debugging di akhir file) ---
AXIS_TILT_INDEX = 1        # index sudut Euler yang mewakili sudut tilt robot (0=roll, 1=pitch, 2=yaw)
TORQUE_SIGN = 1            # kalikan -1 di sini kalau arah torsi terbukti terbalik saat uji coba

# --- Pattern nama child_link untuk deteksi otomatis joint roda ---
LEFT_WHEEL_PATTERN = "wheelL"
RIGHT_WHEEL_PATTERN = "wheelR"

# ==============
# PATH HANDLING
# ==============
# Path dihitung berdasarkan lokasi file script ini sendiri (__file__),
# BUKAN berdasarkan folder tempat perintah `python` dijalankan.
# Ini membuat script tetap benar dijalankan dari folder mana pun.

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)
URDF_PATH = os.path.join(PROJECT_ROOT, "urdf", "urdf", "robot.urdf")

if not os.path.exists(URDF_PATH):
    raise FileNotFoundError(f"URDF tidak ditemukan di: {URDF_PATH}")

# =====================
# SETUP DUNIA SIMULASI
# =====================

def setup_world():
    """Inisialisasi physics client, gravitasi, dan lantai."""
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, GRAVITY)
    p.loadURDF("plane.urdf")
    print("Dunia simulasi siap: gravitasi & lantai terpasang.")