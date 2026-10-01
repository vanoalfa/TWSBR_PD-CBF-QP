"""
Filename: testing_DDSM115_Thetaencoder.py
Deskripsi: Program Python mandiri (standalone) untuk mempelajari cara kerja 
           pembacaan data encoder (Theta / sudut derajat) dari motor DDSM115 via RS485.
"""

import struct
import time
import serial

# ==============================================================================
# 1. KONFIGURASI PARAMETER RS485 & PROTOKOL WAVESHARE DDSM115
# ==============================================================================
SERIAL_PORT = "/dev/ttyACM0"  # Sesuaikan dengan port USB/RS485 Anda
BAUDRATE = 115200             # Kecepatan baudrate standar DDSM115 (115200 8N1)
TIMEOUT_S = 0.1               # Timeout komunikasi serial (100 ms)

MOTOR_ID_LEFT = 1             # ID Motor Kiri
MOTOR_ID_RIGHT = 2            # ID Motor Kanan

# Perintah protokol DDSM115
CMD_CONTROL = 0x64            # Perintah kontrol utama (mengembalikan encoder 16-bit)
CMD_QUERY = 0x74              # Perintah query telemetri (mengembalikan encoder 8-bit + Suhu)
CMD_SWITCH_MODE = 0xA0        # Perintah ganti mode operasi

MODE_CURRENT = 0x01           # Mode Arus / Torsi (Current Mode)
MODE_SPEED = 0x02             # Mode Kecepatan (Velocity Mode)

FRAME_LEN = 10                # Panjang frame komunikasi DDSM115 (selalu 10 byte)
CRC8_INIT = 0x00
CRC8_POLY_REVERSED = 0x8C     # Polinomial terbalik CRC-8/MAXIM (0x8C)


# ==============================================================================
# 2. FUNSI KALKULASI CRC-8/MAXIM & BINER
# ==============================================================================
def crc8_maxim(data: bytes) -> int:
    """
    Hitung stempel pengaman CRC-8/MAXIM dari 9 byte data payload.
    """
    crc = CRC8_INIT
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x01:
                crc = (crc >> 1) ^ CRC8_POLY_REVERSED
            else:
                crc >>= 1
    return crc & 0xFF


def build_frame(b0: int, b1: int, b2: int, b3: int, b4: int, b5: int, b6: int, b7: int, b8: int) -> bytes:
    """
    Menyusun 9 byte payload dan menambahkan 1 byte stempel CRC-8 di byte ke-10.
    """
    payload = bytes([
        b0 & 0xFF, b1 & 0xFF, b2 & 0xFF, b3 & 0xFF,
        b4 & 0xFF, b5 & 0xFF, b6 & 0xFF, b7 & 0xFF, b8 & 0xFF
    ])
    crc = crc8_maxim(payload)
    return payload + bytes([crc])


def hi_lo_to_int16(hi: int, lo: int) -> int:
    """Menggabungkan High & Low byte menjadi signed int16 (-32768 s.d +32767)."""
    return struct.unpack(">h", bytes([hi & 0xFF, lo & 0xFF]))[0]


def hi_lo_to_uint16(hi: int, lo: int) -> int:
    """Menggabungkan High & Low byte menjadi unsigned uint16 (0 s.d 65535)."""
    return struct.unpack(">H", bytes([hi & 0xFF, lo & 0xFF]))[0]


# ==============================================================================
# 3. FUNGSI KOMUNIKASI RS485 & DEKODER ENCODER
# ==============================================================================
def send_query_command(ser: serial.Serial, motor_id: int) -> tuple[float, int, int]:
    """
    Mengirim perintah Query (CMD 0x74) untuk membaca data encoder resolusi 8-bit,
    kecepatan RPM, dan suhu driver motor.
    
    Format Balasan Query (10 Byte):
    Byte 0: Motor ID
    Byte 1: Mode
    Byte 2-3: Torque Raw (int16)
    Byte 4-5: Speed RPM (int16)
    Byte 6: Temp (°C)
    Byte 7: Encoder Posisi 8-bit (0-255)
    Byte 8: Error Code
    Byte 9: CRC-8
    """
    # 1. Buat frame query
    frame = build_frame(motor_id, CMD_QUERY, 0, 0, 0, 0, 0, 0, 0)
    
    # 2. Kirim via serial
    ser.reset_input_buffer()
    ser.write(frame)
    ser.flush()
    
    # 3. Baca balasan 10 byte
    reply = ser.read(FRAME_LEN)
    if len(reply) != FRAME_LEN:
        raise TimeoutError(f"Motor {motor_id} tidak membalas / timeout.")
    
    # 4. Validasi CRC
    expected_crc = crc8_maxim(reply[:9])
    if expected_crc != reply[9]:
        raise ValueError(f"CRC Mismatch pada Motor {motor_id}: ekspetasi {hex(expected_crc)}, dapat {hex(reply[9])}")
    
    # 5. Dekode data encoder & telemetri
    speed_rpm = hi_lo_to_int16(reply[4], reply[5])
    temp_c = reply[6]
    position_u8 = reply[7]
    
    # Konversi encoder 8-bit (0-255) ke Sudut Derajat (0 - 360°)
    position_deg = (position_u8 / 255.0) * 360.0
    
    return position_deg, speed_rpm, temp_c


def send_control_command(ser: serial.Serial, motor_id: int, command_raw: int = 0) -> tuple[float, int]:
    """
    Mengirim perintah Kontrol (CMD 0x64) dengan nilai daya 0 (stop) 
    untuk membaca data encoder resolusi tinggi 16-bit (0-32767).
    
    Format Balasan Kontrol (10 Byte):
    Byte 0: Motor ID
    Byte 1: Mode
    Byte 2-3: Torque Raw (int16)
    Byte 4-5: Speed RPM (int16)
    Byte 6-7: Encoder Posisi 16-bit (0-32767)
    Byte 8: Error Code
    Byte 9: CRC-8
    """
    # Konversi command_raw ke High dan Low byte
    hi = (command_raw >> 8) & 0xFF
    lo = command_raw & 0xFF
    
    # Frame kontrol (Byte 6: accel_time, Byte 7: brake = 0)
    frame = build_frame(motor_id, CMD_CONTROL, hi, lo, 0, 0, 10, 0, 0)
    
    ser.reset_input_buffer()
    ser.write(frame)
    ser.flush()
    
    reply = ser.read(FRAME_LEN)
    if len(reply) != FRAME_LEN:
        raise TimeoutError(f"Motor {motor_id} tidak membalas / timeout.")
    
    if crc8_maxim(reply[:9]) != reply[9]:
        raise ValueError(f"CRC Error pada Motor {motor_id}")
    
    speed_rpm = hi_lo_to_int16(reply[4], reply[5])
    position_raw = hi_lo_to_uint16(reply[6], reply[7])
    
    # Konversi encoder 16-bit ke Sudut Derajat (0 - 360°)
    position_deg = (position_raw / 32767.0) * 360.0 if position_raw <= 32767 else 0.0
    
    return position_deg, speed_rpm


# ==============================================================================
# 4. PROGRAM UTAMA (MAIN LOOP)
# ==============================================================================
def main():
    print("==================================================")
    print("  DDSM115 Encoder Theta Reader (Standalone Test)")
    print("==================================================")
    
    # Membuka koneksi serial pySerial
    try:
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUDRATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=TIMEOUT_S,
        )
        print(f"[INFO] Port {SERIAL_PORT} berhasil dibuka pada {BAUDRATE} bps.")
    except Exception as e:
        print(f"[ERROR] Gagal membuka port serial: {e}")
        return

    try:
        # Pilihan mode baca: True = Menggunakan CMD 0x74 (Query), False = CMD 0x64 (Control)
        USE_QUERY_MODE = True 

        print("[INFO] Mulai membaca data encoder theta secara real-time...\n")
        
        while True:
            try:
                if USE_QUERY_MODE:
                    # Membaca posisi encoder theta via CMD Query (0x74)
                    theta_left, speed_left, temp_left = send_query_command(ser, MOTOR_ID_LEFT)
                    theta_right, speed_right, temp_right = send_query_command(ser, MOTOR_ID_RIGHT)
                    
                    theta_avg = (theta_left + theta_right) / 2.0
                    
                    print(
                        f"\r[QUERY THETA] "
                        f"M1: {theta_left:6.2f}° ({speed_left:3d} RPM, {temp_left}°C) | "
                        f"M2: {theta_right:6.2f}° ({speed_right:3d} RPM, {temp_right}°C) | "
                        f"Theta Avg: {theta_avg:6.2f}°",
                        end="",
                        flush=True
                    )
                else:
                    # Membaca posisi encoder theta via CMD Control 16-bit (0x64)
                    theta_left, speed_left = send_control_command(ser, MOTOR_ID_LEFT, command_raw=0)
                    theta_right, speed_right = send_control_command(ser, MOTOR_ID_RIGHT, command_raw=0)
                    
                    theta_avg = (theta_left + theta_right) / 2.0
                    
                    print(
                        f"\r[CTRL THETA] "
                        f"M1: {theta_left:6.2f}° ({speed_left:3d} RPM) | "
                        f"M2: {theta_right:6.2f}° ({speed_right:3d} RPM) | "
                        f"Theta Avg: {theta_avg:6.2f}°",
                        end="",
                        flush=True
                    )

            except (TimeoutError, ValueError) as err:
                print(f"\n[WARN] Kesalahan komunikasi: {err}")

            time.sleep(0.05)  # Jeda 50 ms (~20 Hz)

    except KeyboardInterrupt:
        print("\n\n[INFO] Menghentikan program...")
    finally:
        ser.close()
        print("[INFO] Port serial ditutup. Selesai.")


if __name__ == "__main__":
    main()