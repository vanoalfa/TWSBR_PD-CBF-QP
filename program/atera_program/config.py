# =========================
# Tunning Control
# =========================
# u_PD = Kp_psi*(angle_psi - setpoint_psi) + Kd_psi*angular_dot_psi
#      + Kp_theta*(angle_theta - setpoint_theta) + Kd_theta*angular_dot_theta
#      + K_vel
#
# All gains are plain integers (0 .. unlimited), there is no clamp inside the PD.
# Unit of u_PD: raw DDSM115 current command count per wheel
# (-32767..32767 <-> -8 A..+8 A, so 1 count = 0.000244 A; verify against datasheet).
#
# Conversion from the previous (stable) normalized tuning:
#   Kp_psi = Kp_old * MAX_CURRENT_A / 8 * 32767 = 0.200 * 1.80 / 8 * 32767 = 1474.5 -> 1475
#   Kd_psi = Kd_old * MAX_CURRENT_A / 8 * 32767 = 0.014 * 1.80 / 8 * 32767 =  103.2 ->  103
Kp_psi = 1475      # [count/deg]      body angle (MPU6050)
Kd_psi = 103       # [count/(deg/s)]  body angular rate (MPU6050)
Kp_theta = 0       # [count/deg]      wheel angle (DDSM115 encoder), 0 = same behaviour as before
Kd_theta = 0       # [count/(deg/s)]  wheel angular rate (DDSM115 encoder), 0 = same behaviour as before
K_vel = 0          # [count]          manual constant offset for balancing

# Reserved for PD + CBF-QP (HOCBF gains, see the [CBF-QP] section below)
Alpha_1 = 0.0
Alpha_2 = 0.0

CONTROLLER_DEADBAND_DEG = 0.05

# Setpoints (deg)
SETPOINT_PSI_DEG = 0.0      # body setpoint (MPU6050)
SETPOINT_THETA_DEG = 0.0    # wheel setpoint (DDSM115), relative to the wheel angle when MULAI is pressed

# Perintah manual saat balancing aktif (ditambahkan ke SETPOINT_PSI_DEG)
MANUAL_FORWARD_TARGET_DEG = 1.20
MANUAL_BACKWARD_TARGET_DEG = -1.20

# Lecturer's formula: output = (torque_left + torque_right) * wheel_distance / 2
# With torque_left = torque_right this gives torque_per_wheel = output / wheel_distance.
# False -> both wheels receive u_PD directly (identical to the previous stable program).
# True  -> both wheels receive u_PD / WHEEL_DISTANCE_M (gains must then be re-scaled by WHEEL_DISTANCE_M).
USE_WHEEL_DISTANCE_FORMULA = True
WHEEL_DISTANCE_M = 0.300169     # distance between the two wheels [m], fill in from the real robot

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

# Jika arah putaran bebeda, ubah salah satu sign '-' ini.
LEFT_MOTOR_SIGN = -1.0
RIGHT_MOTOR_SIGN = 1.0

# Jika arah roda robot sama jatuhnya badan sama maka ubah sign '-'. dihilangkan/ditambah,
BALANCE_DIRECTION_SIGN = -1.0

# Valid saat speed loop. Satuan mengikuti protokol Waveshare:
# waktu akselerasi per 1 rpm dalam kelipatan 0.1 ms.
MOTOR_ACCEL_TIME = 3

# Batas aman awal saat bring-up
# MAX_CURRENT_A is the ACTUATOR limit applied in ddsm115.py (not in the PD).
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
# JOYSTICK
# =========================
JOYSTICK_PATH = '/dev/input/event5'

# =========================
# Runtime / UI / safety
# =========================
CONTROL_HZ = 200.0
UI_HZ = 20.0
KEY_HOLD_TIMEOUT_S = 0.18
SAFE_TILT_DEG = 30.0

# Interval log status ke terminal/file jika dibutuhkan
STATUS_LOG_PERIOD_S = 0.5

# =========================
# ISE evaluation (ise.py)
# =========================
ISE_PLOT_DIR = "PLOT_EVALUASI"   # folder next to the program files
ISE_LIVE_PLOT = True             # show the plot window when MULAI is pressed (needs a display)
ISE_PLOT_REFRESH_HZ = 5.0        # refresh rate of the live plot (runs in a separate process)
ISE_SAVE_CSV = True              # also save the raw data as YYYYMMDD-URUTAN.csv

# ==========================================================================
# [CBF-QP]  PARAMETERS FOR cbf_qp.py  (acados + HPIPM, N = 1)
# --------------------------------------------------------------------------
# Fill in every value that is still None. While any of them is None (or an
# Alpha is 0) the RB button (PD + CBF-QP) is refused and the robot stays in PD.
# ==========================================================================
# [CBF-QP] physical parameters of the robot (SI units)
CBF_M_BODY_KG = None            # M        : body mass [kg]
CBF_M_WHEEL_KG = None           # m        : mass of ONE wheel [kg]
CBF_R_WHEEL_M = None            # R        : wheel radius [m]
CBF_L_COM_M = None              # L        : wheel axle -> body centre of mass [m]
CBF_J_WHEEL_KGM2 = None         # j_omega  : inertia of ONE wheel about its axle [kg m^2]
CBF_J_BODY_KGM2 = None          # j_psi    : body inertia about its centre of mass [kg m^2]
CBF_GRAVITY_M_S2 = 9.81         # g        : gravity [m/s^2]
CBF_ZETA_DEG = 0.0              # zeta     : floor slope [deg], 0 = flat floor

# [CBF-QP] motor torque constant, used to convert current <-> torque
CBF_TORQUE_CONSTANT_NM_PER_A = None   # Kt [N m / A], verify against the DDSM115 datasheet

# [CBF-QP] safe set
CBF_PSI_MAX_DEG = None          # psi_max        : |psi| <= psi_max [deg], must be < SAFE_TILT_DEG
CBF_THETA_DOT_MAX_DEG_S = None  # theta_dot_max  : |theta_dot| <= theta_dot_max [deg/s]

# [CBF-QP] class-K gains
#   h1, h2 (body angle, relative degree 2): Alpha_1 and Alpha_2 at the top of this file [1/s]
#   h3, h4 (wheel speed, relative degree 1): Alpha_3 below [1/s]
Alpha_3 = 0.0

# [CBF-QP] slack penalties (soft constraints keep the QP feasible; tilt has priority)
CBF_SLACK_WEIGHT_PSI = 1.0e5        # per rad of violation of the body-angle condition
CBF_SLACK_WEIGHT_THETA_DOT = 1.0e1  # per rad/s of violation of the wheel-speed condition

# [CBF-QP] set True once to force regeneration of the acados C code
CBF_FORCE_REBUILD = False