# ==========================
# TUNNING
# ==========================
# Tunning Parameter
IMU_KP = 2.0      # Kp Value for IMU (psi)
IMU_KD = 0.5      # Kd Value for IMU (psi)
MOTOR_KP = 2.0    # Kp Value for DDSM (theta)
MOTOR_KD = 0.5    # Kd Value for DDSM (theta)
ALPHA_1 = 0.0     # Value for QP  
ALPHA_2 = 0.0     # Value for QP


# Setpoint
SETPOINT_IMU = 0 # Set point IMU's Y axis degree
SETPOINT_WHEEL = 0 # Set point DDSM's Y axis degree

#Safety
OUTPUT_LIMIT = 1.0
CONTROLLER_DEADBAND_DEG = 0.05
HARD_SAFE_TILT_DEG = 30.0

# For manual move (forward and backward) [modified IMU's setpoint]
MANUAL_FORWARD_TARGET_DEG = 1.30
MANUAL_BACKWARD_TARGET_DEG = -1.30

# ==========================
# IMU
# ==========================
I2C_BUS = 1
MPU6050_ADDRESS = 0x68

# ini bisa ubah ke roll atau pitch tergantung posisi IMU nya (asal bukan Z).
IMU_AXIS = "pitch"

# Koreksi orientasi pemasangan IMU (negatif kalau terbalik).
IMU_SIGN = 1.0

GYRO_CALIBRATION_SAMPLES = 800
GYRO_CALIBRATION_DELAY_S = 0.002

ZERO_CALIBRATION_SAMPLES = 350
ZERO_CALIBRATION_DELAY_S = 0.004

# ==========================
# DDSM/MOTOR
# ==========================
# Serial ports
RIGHT_MOTOR_PORT = "/dev/ttyACM0"  #Right Port
LEFT_MOTOR_PORT = "/dev/ttyACM1" #Left Port

# Baudrates
WHEEL_BAUDRATE = 115200  #Baudrate for motor controller

# Karena tiap motor memakai USB-RS485 terpisah, ID bisa sama-sama 1.
LEFT_MOTOR_ID = 1
RIGHT_MOTOR_ID = 1

# Mode Motor on DDSM
MOTOR_CONTROL_MODE = "current"

# Jika arah putaran salah, ubah salah satu sign ini.
LEFT_MOTOR_SIGN = 1.0
RIGHT_MOTOR_SIGN = -1.0

# Jika robot justru makin jatuh saat balancing mulai aktif,
# ubah jadi -1.0.
BALANCE_DIRECTION_SIGN = 1.0

# Valid saat speed loop. Satuan mengikuti protokol Waveshare:
# waktu akselerasi per 1 rpm dalam kelipatan 0.1 ms.
MOTOR_ACCEL_TIME = 3

# Batas aman awal saat bring-up
MAX_CURRENT_A = 1.80
MAX_SPEED_RPM = 120.0
TURN_OUTPUT_FRACTION = 0.18

# =========================
# Runtime / UI / safety
# =========================
CONTROL_HZ = 200.0
UI_HZ = 20.0
KEY_HOLD_TIMEOUT_S = 0.18

# Interval log status ke terminal/file jika dibutuhkan
STATUS_LOG_PERIOD_S = 0.5