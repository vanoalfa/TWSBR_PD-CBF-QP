"""Parameter tuning untuk self balancing robot.

File ini sengaja hanya berisi parameter yang sering diubah saat bring-up,
kalibrasi, dan tuning di lapangan.
"""

# =========================
# Tunning Control
# =========================
# Output kontrol dinormalisasi ke -1.0 .. +1.0
Kp = 0.080
Kd = 0.014
Alpha_1 = 0.0
Alpha_2 = 0.0
OUTPUT_LIMIT = 1.0
CONTROLLER_DEADBAND_DEG = 0.05

# Perintah manual saat balancing aktif
MANUAL_FORWARD_TARGET_DEG = 1.20
MANUAL_BACKWARD_TARGET_DEG = -1.20

# =========================
# Motor / serial parameters
# =========================
LEFT_MOTOR_PORT = "/dev/ttyACM0"
RIGHT_MOTOR_PORT = "/dev/ttyACM1"

# Karena tiap motor memakai USB-RS485 terpisah, ID bisa sama-sama 1.
LEFT_MOTOR_ID = 1
RIGHT_MOTOR_ID = 1

MOTOR_BAUDRATE = 115200
MOTOR_TIMEOUT_S = 0.03

# Pilihan: "current" atau "speed"
# Untuk balancing, mode "current" biasanya lebih responsif.
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
# IMU / MPU6050 parameters
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
# Runtime / UI / safety
# =========================
CONTROL_HZ = 200.0
UI_HZ = 20.0
KEY_HOLD_TIMEOUT_S = 0.18
SAFE_TILT_DEG = 30.0

# Interval log status ke terminal/file jika dibutuhkan
STATUS_LOG_PERIOD_S = 0.5
