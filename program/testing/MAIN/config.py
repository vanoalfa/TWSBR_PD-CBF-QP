# Serial ports
WHEEL_RIGHT_PORT = "/dev/ttyACM0"  # Port Motor Kanan
WHEEL_LEFT_PORT = "/dev/ttyACM1" # Port Motor Kiri
IMU_PORT = "COM15"  # Magnetic sensor port

# Baudrates
WHEEL_BAUDRATE = 115200  # Baudrate for motor controller
IMU_BAUDRATE = 9600  # Baudrate for sensor

# Timeout
WHEEL_TIMEOUT = 0.2

# PID
PID_KP = 2.0      # Kp Value
PID_KD = 0.5      # Kd Value

# IMU Parameter
SETPOINT = 0 # Set point IMU's Y axis degree


"""
DDSM115 Command Helper (Waveshare)
==================================
Referensi: https://www.waveshare.com/wiki/DDSM115

Baudrate  : 115200, 8N1
Frame     : 10 byte, byte terakhir = CRC8/MAXIM (Dallas 1-Wire)
Torque const (Kt): 0.75 Nm/A
Rentang arus fisik: -8A s/d 8A  -> data signed 16-bit -32767 s/d 32767
Rentang kecepatan : -330 s/d 330 rpm (data = rpm langsung)
Rentang posisi    : 0 s/d 360 deg -> data unsigned 16-bit 0 s/d 32767

CRC8 sudah diverifikasi cocok 100% dengan seluruh contoh command
di dokumentasi resmi Waveshare (ID set, brake, current, velocity, dll).
"""

CURRENT_LOOP  = 0x01
VELOCITY_LOOP = 0x02
POSITION_LOOP = 0x03

TORQUE_CONSTANT_NM_PER_A = 0.75  # Nm per Ampere
MAX_CURRENT_A = 8.0              # sesuai datasheet: -32767..32767 <=> -8A..8A


def crc8_maxim(data: bytes) -> int:
    """CRC-8/MAXIM-DOW (Dallas 1-Wire), sesuai spesifikasi Waveshare."""
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x01:
                crc = (crc >> 1) ^ 0x8C
            else:
                crc >>= 1
    return crc & 0xFF


def _to_signed16_bytes(value: int) -> tuple[int, int]:
    value = max(-32767, min(32767, int(round(value))))
    raw = value & 0xFFFF
    return (raw >> 8) & 0xFF, raw & 0xFF


def set_mode_frame(motor_id: int, mode: int) -> bytes:
    """Bangun frame untuk pindah mode (current/velocity/position loop)."""
    frame = bytes([motor_id, 0xA0, 0, 0, 0, 0, 0, 0, 0, mode])
    return frame


def drive_frame(motor_id: int, value: int, accel: int = 0, brake: int = 0xFF) -> bytes:
    """
    Bangun frame 'drive' generik (Protocol 1).
    `value` sudah dalam satuan data mentah (bukan Nm/rpm/derajat).
    """
    hi, lo = _to_signed16_bytes(value)
    body = bytes([motor_id, 0x64, hi, lo, 0, 0, accel, brake, 0])
    return body + bytes([crc8_maxim(body)])


def torque_to_current_data(torque_nm: float) -> int:
    """Konversi torsi (Nm) -> nilai data current-loop (-32767..32767)."""
    current_a = torque_nm / TORQUE_CONSTANT_NM_PER_A
    current_a = max(-MAX_CURRENT_A, min(MAX_CURRENT_A, current_a))
    return int(round((current_a / MAX_CURRENT_A) * 32767))


def torque_command_frame(motor_id: int, torque_nm: float, brake: int = 0xFF) -> bytes:
    """Frame siap kirim: berikan torsi target langsung dalam Nm (mode current loop)."""
    return drive_frame(motor_id, torque_to_current_data(torque_nm), accel=0, brake=brake)


def velocity_command_frame(motor_id: int, rpm: float, accel: int = 1, brake: int = 0xFF) -> bytes:
    """Frame siap kirim: berikan kecepatan target dalam rpm (mode velocity loop)."""
    rpm = max(-330, min(330, rpm))
    return drive_frame(motor_id, rpm, accel=accel, brake=brake)


def position_command_frame(motor_id: int, degrees: float) -> bytes:
    """Frame siap kirim: berikan posisi target dalam derajat (mode position loop)."""
    degrees = max(0.0, min(360.0, degrees))
    data = int(round((degrees / 360.0) * 32767))
    return drive_frame(motor_id, data)


if __name__ == "__main__":
    # --- Contoh pemakaian (tanpa koneksi serial, cuma print HEX) ---
    print("Set mode -> current loop :", set_mode_frame(1, CURRENT_LOOP).hex(' ').upper())
    print("Torsi target 0.3 Nm      :", torque_command_frame(1, 0.3).hex(' ').upper())
    print("Torsi target -0.5 Nm     :", torque_command_frame(1, -0.5).hex(' ').upper())
    print("Kecepatan target 50 rpm  :", velocity_command_frame(1, 50).hex(' ').upper())

    # --- Contoh pemakaian nyata via RS485 (uncomment & sesuaikan port) ---
    # import serial  # pip install pyserial
    # ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.05)
    # ser.write(set_mode_frame(1, CURRENT_LOOP))
    # ser.write(torque_command_frame(1, 0.35))       # minta torsi 0.35 Nm
    # feedback = ser.read(10)                        # frame balasan 10 byte
    # ser.close()