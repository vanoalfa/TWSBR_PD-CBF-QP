"""config.py - SATU-SATUNYA sumber nilai untuk program ATERA.

Semua konstanta fisik, gain, batas, port, dan pengaturan runtime ada di sini.
File lain hanya membaca (import config), tidak pernah mendefinisikan angka sendiri.

Satuan: SI (kg, m, s, rad, N m) kecuali nama variabel berakhiran _DEG / _A / _HZ.

Konvensi tanda (berlaku di semua file):
    psi   > 0  : badan miring ke DEPAN
    theta > 0  : roda menggelinding ke DEPAN (arah yang sama dengan psi > 0)
    u     > 0  : torsi yang mendorong roda ke DEPAN (reaksinya mendorong badan ke belakang)
    state x = [theta, dtheta, psi, dpsi]
"""

import math

# =============================================================================
# 1. PARAMETER FISIK ROBOT  (dari <proyek> di Konteks_ATERA.md)
# =============================================================================
m_r = 0.745          # [kg]     massa SATU roda (dua roda = 1.490 kg)
m_b = 1.667          # [kg]     massa badan
R = 0.050            # [m]      jari-jari roda
L = 0.131224         # [m]      jarak pusat massa badan ke sumbu roda
j_theta = 9.2e-4     # [kg m^2] momen inersia roda.  ASUMSI: nilai untuk SATU roda
j_psi = 0.012        # [kg m^2] momen inersia badan. ASUMSI: terhadap PUSAT MASSA badan
GRAVITY = 9.81       # [m/s^2]

# Kemiringan bidang: isi 0.0 (datar) atau 8.0 (miring). zeta > 0 = menanjak ke arah DEPAN.
zeta_DEG = 0.0
zeta = math.radians(zeta_DEG)

# =============================================================================
# 2. SAFETY SET  (dipakai CBF-QP)
# =============================================================================
psi_max_DEG = 15.0
psi_max = math.radians(psi_max_DEG)      # [rad]    |psi|    <= psi_max
dtheta_max = 20.0                        # [rad/s]  |dtheta| <= dtheta_max

# =============================================================================
# 3. GAIN KONTROL  (input manual, default 0)
# =============================================================================
# u_PD = Kp_theta*theta + Kd_dtheta*dtheta + Kp_psi*psi + Kd_dpsi*dpsi + Kvel
# u_PD adalah torsi TOTAL kedua roda [N m]; tiap roda menerima u/2.
#
# Dengan konvensi tanda di atas, KEEMPAT gain bernilai POSITIF untuk sistem stabil
# (lihat model.linearize() dan catatan di pd_control.py).
#
# Titik awal dari program lama yang sudah bisa berdiri (apalah_bisa/tunning.py,
# Kp = 0.200 dan Kd = 0.014 per derajat, ternormalisasi ke 1.80 A per roda):
#   Kp_psi  = 0.200 * 1.80 A * 0.75 N m/A * 2 roda * (180/pi) = 30.9  N m/rad
#   Kd_dpsi = 0.014 * 1.80 A * 0.75 N m/A * 2 roda * (180/pi) =  2.17 N m s/rad
# Kp_theta dan Kd_dtheta adalah yang membuat robot DIAM di tempat (tidak hanyut).
Kp_theta = 0.0       # [N m/rad]    sudut roda
Kd_dtheta = 0.0      # [N m s/rad]  kecepatan sudut roda
Kp_psi = 30.9         # [N m/rad]    sudut badan
Kd_dpsi = 14.0        # [N m s/rad]  kecepatan sudut badan
Kvel = 0.0           # [N m]        konstanta torsi (tidak dikalikan state)

# Gain class-K (linear) CBF-QP.  Harus > 0 agar mode PD+CBF bisa dipakai.
#   h1, h2 (batas psi,    relative degree 2): Alpha_1 dan Alpha_2
#   h3, h4 (batas dtheta, relative degree 1): Alpha_1
# Makin besar = filter makin longgar (baru mengoreksi dekat batas).
# Di posisi tegak, HOCBF membatasi |u| kira-kira Alpha_1*Alpha_2*psi_max/74.7 N m, jadi nilai
# kecil ikut memotong torsi PD yang dibutuhkan untuk pulih (simulasi: 10 terlalu ketat).
# Titik awal yang wajar: Alpha_1 = Alpha_2 = 20 .. 30 [1/s].
Alpha_1 = 0.0        # [1/s]
Alpha_2 = 0.0        # [1/s]

# =============================================================================
# 4. CBF-QP  (cbf_qp.py, acados + HPIPM, N = 1)
# =============================================================================
CBF_N_HORIZON = 1                  # sesuai <batasan>: N = 1
# "acados"   : QP diselesaikan HPIPM lewat acados (sesuai skripsi)
# "analitik" : solusi tertutup QP 1 variabel (untuk uji tanpa acados, mis. di laptop)
CBF_SOLVER = "acados"
# Constraint dibuat SOFT supaya QP selalu punya solusi. Bobot slack batas psi jauh lebih
# besar dari batas dtheta: bila keduanya bertabrakan, batas psi yang dimenangkan.
CBF_SLACK_WEIGHT_PSI = 1.0e4
CBF_SLACK_WEIGHT_DTHETA = 1.0e2
CBF_SLACK_TOL = 1.0e-3             # [N m] slack di atas ini -> feasible = False
CBF_ACTIVE_TOL = 1.0e-3            # [N m] |uQP - uPD| di atas ini -> filter dianggap aktif
CBF_FORCE_REBUILD = False          # True sekali untuk memaksa generate ulang kode C acados

# =============================================================================
# 5. MOTOR DDSM115  (sudah dicek ke wiki Waveshare DDSM115)
# =============================================================================
LEFT_MOTOR_PORT = "/dev/ttyACM0"
RIGHT_MOTOR_PORT = "/dev/ttyACM1"
LEFT_MOTOR_ID = 1                  # tiap motor punya USB-RS485 sendiri, ID boleh sama
RIGHT_MOTOR_ID = 1
MOTOR_BAUDRATE = 115200            # datasheet: 115200, 8N1, frame 10 byte, CRC-8/MAXIM
MOTOR_TIMEOUT_S = 0.03
MOTOR_MAX_COMM_ERRORS = 5          # gagal komunikasi berturut-turut sebelum FAULT
MOTOR_RETRY_S = 2.0                # jeda coba-ulang inisialisasi motor yang bermasalah
# Uji roda langsung (L3 = kiri, R3 = kanan), tanpa lewat kontrol. Robot harus DIANGKAT.
MOTOR_TEST_CURRENT_A = 0.30
MOTOR_TEST_DURATION_S = 1.0
MOTOR_TEST_MIN_ANGLE = 0.20        # [rad] putaran minimum agar uji dianggap berhasil
# Diagnosa "roda diperintah tapi tidak berputar"
STALL_CURRENT_A = 0.30             # [A] arus perintah di atas ini dianggap sedang diperintah
STALL_TIME_S = 1.0                 # [s]
STALL_MIN_SPEED = 0.2              # [rad/s] di bawah ini roda dianggap diam

# Arah. Nilai ini dari program lama yang sudah bisa berdiri.
LEFT_MOTOR_SIGN = -1.0             # arus positif (kerangka robot) = roda ke DEPAN
RIGHT_MOTOR_SIGN = 1.0
# Arah hitung posisi encoder relatif terhadap arah arus positif motor.
# Data uji inersia roda (inersia_*.csv): arus positif -> posisi MENURUN, jadi -1.
ENCODER_DIRECTION = -1.0

# Skala arus mode current loop: -32767..32767 <-> -8..8 A (datasheet).
MOTOR_CURRENT_FULL_SCALE_A = 8.0
MOTOR_RAW_FULL_SCALE = 32767
MOTOR_POSITION_COUNTS = 32768      # posisi 0..32767 <-> 0..360 derajat
# Konstanta torsi datasheet. PERHATIAN: uji inersia Anda memberi G = K/J sekitar
# 255..272 rad/s^2 per A; dengan j_theta = 9.2e-4 itu berarti K efektif sekitar 0.24 N m/A.
# Angka ini menskalakan torsi <-> arus, jadi memengaruhi arti semua gain dan model CBF.
MOTOR_KT = 0.75                    # [N m/A] datasheet
MAX_CURRENT_A = 1.80               # [A] batas arus per roda (rated 1.5 A, locked-rotor <= 2.7 A)

# Sumber dtheta: "posisi" (turunan posisi encoder, disarankan) atau "rpm" (umpan balik rpm
# motor; hasil uji Anda menunjukkan rpm ini tertinggal sekitar 0.1 s).
DTHETA_SOURCE = "posisi"
DTHETA_FILTER_TAU_S = 0.020        # konstanta waktu low-pass dtheta
# Encoder motor mengukur roda RELATIF terhadap badan. True: theta = theta_encoder + psi.
ENCODER_RELATIVE_TO_BODY = True

# =============================================================================
# 6. IMU MPU6050
# =============================================================================
I2C_BUS = 1
MPU6050_ADDRESS = 0x68
IMU_AXIS = "pitch"                 # "pitch" (sumbu Y sensor) atau "roll" (sumbu X sensor)
IMU_SIGN = 1.0                     # balik ke -1.0 bila psi terbaca negatif saat miring ke depan
# DLPF_CFG register 0x1A: 0 = 260 Hz (seperti program lama), 2 = 94 Hz, 3 = 44 Hz.
# Naikkan bila suku Kd_dpsi terasa bergetar/berisik.
MPU6050_DLPF_CFG = 0
GYRO_CALIBRATION_SAMPLES = 800
GYRO_CALIBRATION_DELAY_S = 0.002
ZERO_CALIBRATION_SAMPLES = 350
ZERO_CALIBRATION_DELAY_S = 0.004
IMU_RETRY_S = 1.0                  # jeda coba-ulang membuka IMU yang bermasalah
IMU_FROZEN_SAMPLES = 40            # sampel mentah identik berturut-turut -> IMU dianggap macet
IMU_CAL_MAX_GYRO_STD = 1.5         # [deg/s] di atas ini robot dianggap bergerak saat kalibrasi

# =============================================================================
# 7. JOYSTICK  (Fantech WGP-13s)
# =============================================================================
JOYSTICK_PORT = "/dev/input/event5"
JOYSTICK_DEADZONE = 0.15
JOYSTICK_RECONNECT_S = 1.0
# D-Pad: sumbu mana untuk maju/mundur dan belok.
# ASUMSI: PAD Atas/Bawah (code 17) = maju/mundur, PAD Kiri/Kanan (code 16) = belok,
# sama seperti program lama. Tukar dua angka ini bila ingin sebaliknya.
DPAD_DRIVE_CODE = 17
DPAD_TURN_CODE = 16

# Perintah gerak (hanya aktif setelah Tombol A). Referensi digeser, PD tetap sama:
#   u_PD = pd.compute(x - x_ref)
JOY_DTHETA_REF = 3.0               # [rad/s] kecepatan roda referensi saat perintah penuh (0.15 m/s)
JOY_PSI_REF_DEG = 1.2              # [deg]   kemiringan referensi saat perintah penuh (program lama)
JOY_THETA_LEAD_MAX = 2.0           # [rad]   theta_ref tidak boleh mendahului theta lebih dari ini
TURN_TORQUE = 0.24                 # [N m]   selisih torsi per roda saat belok penuh
TURN_SIGN = 1.0                    # balik ke -1.0 bila arah belok terbalik

# =============================================================================
# 8. RUNTIME / KEAMANAN / GUI
# =============================================================================
CONTROL_HZ = 200.0                 # DDSM115: maksimum 500 Hz
UI_HZ = 10.0
SAFE_TILT_DEG = 30.0               # |psi| di atas ini -> motor dimatikan (FAULT)
START_TILT_DEG = 15.0              # MULAI ditolak bila |psi| lebih besar dari ini (= psi_max)
SATURATION_FAULT_S = 1.0           # |u| = u_max terus-menerus selama ini -> FAULT
CBF_INFEASIBLE_FAULT_S = 0.5       # CBF-QP infeasible terus-menerus selama ini -> FAULT
LOG_FILE = "atera.log"

# =============================================================================
# 9. EVALUASI ISE  (ise.py)
# =============================================================================
ISE_PLOT_DIR = "PLOT_EVALUASI"
ISE_MIN_DURATION_S = 1.0           # percobaan lebih singkat dari ini tidak disimpan

# =============================================================================
# 10. DIAGNOSA  (ambang peringatan di GUI)
# =============================================================================
# Peringatan "gain terlalu kecil": arus per roda pada psi = 5 deg di bawah nilai ini.
DIAG_MIN_CURRENT_AT_5DEG_A = 0.10
DIAG_MIN_LOOP_RATIO = 0.80         # loop di bawah 80% CONTROL_HZ -> peringatan

# =============================================================================
# Turunan (jangan diubah): batas aktuator u
# =============================================================================
TORQUE_PER_WHEEL_MAX = MOTOR_KT * MAX_CURRENT_A      # [N m] per roda
u_max = 2.0 * TORQUE_PER_WHEEL_MAX                   # [N m] total dua roda