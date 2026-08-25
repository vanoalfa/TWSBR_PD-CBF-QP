# PID
PID_KP = 2.0      # Kp Value
PID_KD = 0.5      # Kd Value

# IMU Parameter
SETPOINT = 0 # Set point IMU's Y axis degree

# Serial ports
WHEEL_PORT_RIGHT = "COM4"  # Motor controller port
WHEEL_PORT_LEFT = "COM4"
IMU_PORT = "COM15"  # Magnetic sensor port

# Baudrates
WHEEL_BAUDRATE = 115200  # Baudrate for motor controller
IMU_BAUDRATE = 9600  # Baudrate for sensor

# Config DDSM 115
CURRENT_LOOP  = 0x01
VELOCITY_LOOP = 0x02
POSITION_LOOP = 0x03

TORQUE_CONSTANT_NM_PER_A = 0.75  # Nm per Ampere
MAX_CURRENT_A = 8.0              # sesuai datasheet: -32767..32767 <=> -8A..8A
MIN_CURRENT_A = -8.0