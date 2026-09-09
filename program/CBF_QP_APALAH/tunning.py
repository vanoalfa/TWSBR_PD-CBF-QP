# ============================================================================
# tunning.py  --  KONFIGURASI TERPUSAT ROBOT TWSBR "ATERA" (PD + CBF-QP)
# ----------------------------------------------------------------------------
# Semua parameter yang mungkin ingin Anda ubah-ubah dikumpulkan di file ini.
# Bagian-bagian:
#   1. Tuning kontrol PD (nominal controller)
#   2. Tuning CBF-QP (safety filter HOCBF orde-2, acados + HPIPM, N=1)
#   3. Parameter MODEL FISIK robot  <-- WAJIB ANDA ISI (masih placeholder!)
#   4. Parameter motor / serial DDSM115
#   5. Parameter IMU / MPU6050
#   6. Joystick
#   7. Runtime / UI / safety
#   8. Evaluasi (ISE) untuk skripsi
# ============================================================================

# =========================
# 1. Tunning Control PD
# =========================
# Output kontrol PD dinormalisasi ke -1.0 .. +1.0
Kp = 0.100
Kd = 0.014
OUTPUT_LIMIT = 1.0
CONTROLLER_DEADBAND_DEG = 0.05

# Jika arah respon balancing terbalik (robot makin jatuh), ubah jadi -1.0
BALANCE_DIRECTION_SIGN = 1.0

# Perintah manual saat balancing aktif
MANUAL_FORWARD_TARGET_DEG = 1.20
MANUAL_BACKWARD_TARGET_DEG = -1.20

# Fraksi output untuk belok (differential drive)
TURN_OUTPUT_FRACTION = 0.18

# =========================
# 2. Tunning CBF-QP (HOCBF orde-2)
# =========================
# Saklar utama: True  = PD -> CBF-QP -> motor  (safety filter aktif)
#               False = PD -> motor langsung   (perilaku seperti program lama)
CBF_ENABLED = True

# --- Parameter class-K HOCBF (SILAKAN UBAH-UBAH DI SINI) ---
# Kondisi HOCBF orde-2 untuk setiap barrier h_i(x):
#   LgLf*h_i(x) * u  >=  -Lf^2 h_i(x) - Lf[alpha1(h_i)] - alpha2( Lf h_i(x) + alpha1(h_i) )
# dengan alpha1(h) = ALPHA_1 * h  dan  alpha2(h) = ALPHA_2 * h  (linear class-K).
Alpha_1 = 10.0     # <-- isi angka Anda di sini
Alpha_2 = 10.0     # <-- isi angka Anda di sini

# --- Batas barrier function ---
# h1 = -psi + PSI_MAX >= 0  (psi <= +PSI_MAX)
# h2 =  psi + PSI_MAX >= 0  (psi >= -PSI_MAX)
# Batas kemiringan badan robot (pendulum) yang DIJAGA oleh CBF.
# Catatan: cutoff keras tetap di SAFE_TILT_DEG (30 deg) via set_fault.
PSI_MAX_DEG = 15.0

# h3 = -theta_dot + THETA_DOT_MAX >= 0  (theta_dot <= +THETA_DOT_MAX)
# h4 =  theta_dot + THETA_DOT_MAX >= 0  (theta_dot >= -THETA_DOT_MAX)
# Batas kecepatan sudut roda (deg/s) yang DIJAGA oleh CBF.
THETA_DOT_MAX_DEG_S = 13.0

# --- Bobot QP ---
# Fungsi biaya: min  (1/2) u^T (2I) u + (-2 u_PD)^T u  +  W_SLACK * sum(delta_i^2)
# delta_i = variabel slack agar QP tidak pernah infeasible (dihukum sangat berat).
W_SLACK = 1.0e4

# --- Batas input arus motor (hard constraint di QP, Ampere) ---
CBF_U_MAX_A = 1.80

# --- Pengaturan solver acados / HPIPM ---
CBF_SOLVER_N = 1                    # horizon N = 1 (sesuai rencana)
CBF_HPIPM_MODE = "SPEED"            # "SPEED" cocok untuk QP kecil real-time
CBF_SOLVER_FOLDER = "c_generated_code_cbf"  # folder hasil generate acados

# =========================
# 3. Parameter MODEL FISIK robot   <<<<<< WAJIB ANDA ISI >>>>>>
# =========================
# Nilai di bawah ini hanya PLACEHOLDER (float). Ganti dengan hasil pengukuran
# / perhitungan / identifikasi robot Anda. Model yang dipakai adalah dinamika
# pendulum-roda terlinearisasi di sekitar posisi tegak:
#
#   psi_ddot     = A1*psi + A2*psi_dot + A3*theta_dot + B1*(I_L + I_R)
#   theta_ddot   = A4*psi + A5*psi_dot + A6*theta_dot + B2*(I_L + I_R)
#
# dengan u = [I_L, I_R] adalah arus motor (Ampere).
#
# Hubungan parameter fisik -> koefisien A1..A6, B1..B2 dihitung OTOMATIS
# oleh robot_model.py dari variabel-variabel berikut:

# --- Geometri & massa ---
MASS_BODY_KG = 0.0          # massa badan robot (tanpa roda), kg            [ISI]
MASS_WHEEL_KG = 0.0         # massa SATU roda, kg                            [ISI]
LENGTH_COM_M = 0.0          # jarak sumbu roda ke pusat massa badan, m       [ISI]
WHEEL_RADIUS_M = 0.0        # radius roda, m                                 [ISI]
INERTIA_BODY_KGM2 = 0.0     # momen inersia badan thd sumbu roda, kg.m^2     [ISI]
INERTIA_WHEEL_KGM2 = 0.0    # momen inersia SATU roda thd porosnya, kg.m^2   [ISI]
GRAVITY_M_S2 = 9.81         # percepatan gravitasi, m/s^2

# --- Motor DDSM115 ---
TORQUE_CONSTANT_NM_PER_A = 0.0   # konstanta torsi motor Kt, Nm/A           [ISI]
GEAR_RATIO = 1.0                 # rasio gearbox (1.0 = direct drive)        [ISI]

# --- Redaman (estimasi, boleh mulai dari nilai kecil) ---
DAMPING_PENDULUM_NMS = 0.0    # redaman pada sendi pendulum, Nm.s/rad        [ISI]
DAMPING_WHEEL_NMS = 0.0       # redaman pada sumbu roda, Nm.s/rad            [ISI]

# =========================
# 4. Motor / serial parameters
# =========================
LEFT_MOTOR_PORT = "/dev/ttyACM0"
RIGHT_MOTOR_PORT = "/dev/ttyACM1"

# Karena tiap motor memakai USB-RS485 terpisah, ID bisa sama-sama 1.
LEFT_MOTOR_ID = 1
RIGHT_MOTOR_ID = 1

MOTOR_BAUDRATE = 115200
MOTOR_TIMEOUT_S = 0.03

# Pilihan: "current" atau "speed"
# Untuk balancing, mode "current" lebih responsif dan WAJIB untuk CBF-QP
# karena output QP adalah arus (torque).
MOTOR_CONTROL_MODE = "current"

# Jika arah putaran salah, ubah salah satu sign ini.
LEFT_MOTOR_SIGN = -1.0
RIGHT_MOTOR_SIGN = 1.0

# Valid saat speed loop. Satuan mengikuti protokol Waveshare:
# waktu akselerasi per 1 rpm dalam kelipatan 0.1 ms.
MOTOR_ACCEL_TIME = 3

# Batas aman awal saat bring-up
MAX_CURRENT_A = 1.80
MAX_SPEED_RPM = 120.0

# =========================
# 5. IMU / MPU6050 parameters
# =========================
I2C_BUS = 1
MPU6050_ADDRESS = 0x68

# Ubah ke "roll" jika sumbu balancing Anda adalah roll.
IMU_AXIS = "pitch"

# Koreksi orientasi pemasangan IMU.
IMU_SIGN = 1.0

GYRO_CALIBRATION_SAMPLES = 800
GYRO_CALIBRATION_DELAY_S = 0.002

ZERO_CALIBRATION_SAMPLES = 350
ZERO_CALIBRATION_DELAY_S = 0.004

# =========================
# 6. JOYSTICK
# =========================
JOYSTICK_PATH = '/dev/input/event5'

# =========================
# 7. Runtime / UI / safety
# =========================
CONTROL_HZ = 200.0
UI_HZ = 20.0
KEY_HOLD_TIMEOUT_S = 0.18

# Cutoff KERAS: jika |psi| >= 30 deg -> set_fault (tetap seperti program lama)
SAFE_TILT_DEG = 30.0

# Interval log status ke terminal/file jika dibutuhkan
STATUS_LOG_PERIOD_S = 0.5

# =========================
# 8. Evaluasi (ISE) untuk skripsi
# =========================
EVAL_ENABLED = True                 # aktifkan pencatatan data eksperimen
EVAL_LOG_DIR = "logs_evaluasi"      # folder output CSV & grafik
EVAL_MAX_DURATION_S = 60.0          # durasi maksimum perekaman per sesi
EVAL_FILENAME_PREFIX = "cbf_qp"     # prefix nama file: cbf_qp_YYYYMMDD_HHMMSS
