import serial
import time
import sys
import tty
import termios

PORT = "/dev/ttyACM0"
BAUD = 115200
TIMEOUT = 0.2

COMMANDS = {
    "w": bytes.fromhex("01 64 00 64 00 00 00 00 00 4F"),  # maju
    "s": bytes.fromhex("01 64 FF 9C 00 00 00 00 00 9A"),  # mundur
    "x": bytes.fromhex("01 64 00 00 00 00 00 00 00 50"),  # brake
}

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def send_command(ser, key):
    frame = COMMANDS[key]
    ser.reset_input_buffer()
    ser.write(frame)
    ser.flush()
    time.sleep(0.05)

    waiting = ser.in_waiting
    resp = ser.read(waiting if waiting > 0 else 64)

    print(f"TX [{key.upper()}]: {frame.hex(' ')}")
    if resp:
        print(f"RX        : {resp.hex(' ')}")
    else:
        print("RX        : <no response>")
    print()

def main():
    ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
    time.sleep(0.1)
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    print(f"Opened: {ser.name}")
    print("Kontrol:")
    print("  W = maju")
    print("  S = mundur")
    print("  X = brake")
    print("  Q = keluar")
    print()

    try:
        while True:
            key = get_key().lower()

            if key == "q":
                print("Keluar program...")
                break

            if key in COMMANDS:
                send_command(ser, key)

    except KeyboardInterrupt:
        print("\nDihentikan dengan Ctrl+C")

    finally:
        try:
            brake_frame = COMMANDS["x"]
            ser.write(brake_frame)
            ser.flush()
            time.sleep(0.05)
            print("Brake dikirim sebelum close.")
        except Exception:
            pass

        ser.close()
        print("Serial port ditutup.")

if __name__ == "__main__":
    main()
