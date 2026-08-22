import sys
import time
import struct
import serial
import crcmod.predefined

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
BAUD = 115200
MOTOR_ID = int(sys.argv[2]) if len(sys.argv) > 2 else 1

crc8 = crcmod.predefined.mkPredefinedCrcFun("crc-8-maxim")


def with_crc(data9: bytes) -> bytes:
    return data9 + bytes([crc8(data9)])


def set_velocity_mode(ser, motor_id: int):
    # Official mode-switch frame is 10 bytes total, last byte is mode value 0x02.
    frame = bytes([motor_id, 0xA0, 0, 0, 0, 0, 0, 0, 0, 0x02])
    ser.write(frame)
    print("SET MODE:", frame.hex(" "))


def set_speed_rpm(ser, motor_id: int, rpm: int, acc_time: int = 1, brake: int = 0):
    rpm = max(-330, min(330, int(rpm)))
    data9 = struct.pack(">BBhBBBBB", motor_id, 0x64, rpm, 0, 0, acc_time & 0xFF, brake & 0xFF, 0)
    frame = with_crc(data9)
    ser.write(frame)
    print("SET SPEED:", frame.hex(" "))


def read_info(ser, motor_id: int):
    frame = with_crc(bytes([motor_id, 0x74, 0, 0, 0, 0, 0, 0, 0]))
    ser.reset_input_buffer()
    ser.write(frame)
    resp = ser.read(10)
    print("QUERY INFO:", frame.hex(" "))
    print("RESPONSE  :", resp.hex(" "))
    if len(resp) == 10:
        current_raw = struct.unpack(">h", resp[2:4])[0]
        rpm = struct.unpack(">h", resp[4:6])[0]
        temp = resp[6]
        pos_u8 = resp[7]
        err = resp[8]
        current_a = current_raw / 32767.0 * 8.0
        print(f"mode={resp[1]} rpm={rpm} current={current_a:.2f}A temp={temp}C pos_u8={pos_u8} err=0x{err:02X}")
    else:
        print("Tidak ada balasan 10 byte. Cek wiring RS485, port serial, ID motor, dan power.")


def main():
    print(f"Open {PORT} @ {BAUD} for motor ID {MOTOR_ID}")
    with serial.Serial(PORT, BAUD, bytesize=8, parity='N', stopbits=1, timeout=0.2) as ser:
        time.sleep(0.2)
        set_velocity_mode(ser, MOTOR_ID)
        time.sleep(0.1)
        read_info(ser, MOTOR_ID)
        set_speed_rpm(ser, MOTOR_ID, 30, acc_time=3)
        time.sleep(2.0)
        read_info(ser, MOTOR_ID)
        set_speed_rpm(ser, MOTOR_ID, 0)
        time.sleep(0.2)
        read_info(ser, MOTOR_ID)


if __name__ == "__main__":
    main()
