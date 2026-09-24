import serial
import time
import sys
import tty
import termios

# Daftar port yang akan dikontrol bersamaan
PORTS = ["/dev/ttyACM0", "/dev/ttyACM1"]
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

def send_command_to_all(ser_list, key):
    frame = COMMANDS[key]
    print(f"TX [{key.upper()}]: {frame.hex(' ')}")
    
    # Send perintah ke semua port secara berturut-turut
    for ser in ser_list:
        ser.reset_input_buffer()
        ser.write(frame)
        ser.flush()
    
    time.sleep(0.05)

    # Baca balasan dari setiap port
    for ser in ser_list:
        waiting = ser.in_waiting
        resp = ser.read(waiting if waiting > 0 else 64)
        if resp:
            print(f"RX [{ser.port}] : {resp.hex(' ')}")
        else:
            print(f"RX [{ser.port}] : <no response>")
    print()

def main():
    ser_list = []
    
    # Buka seluruh port serial yang terhubung
    for port in PORTS:
        try:
            ser = serial.Serial(port, BAUD, timeout=TIMEOUT)
            ser_list.append(ser)
            print(f"Opened: {ser.name}")
        except serial.SerialException as e:
            print(f"Error: Gagal membuka port {port}: {e}")

    if not ser_list:
        print("Tidak ada port serial yang berhasil dibuka. Program berhenti.")
        return

    # Buffer clearing awal
    time.sleep(0.1)
    for ser in ser_list:
        ser.reset_input_buffer()
        ser.reset_output_buffer()

    print("\nKontrol Robot:")
    print("  W = maju")
    print("  S = mundur")
    print("  X = brake")
    print("  Q = keluar\n")

    try:
        while True:
            key = get_key().lower()

            if key == "q":
                print("Keluar program...")
                break

            if key in COMMANDS:
                send_command_to_all(ser_list, key)

    except KeyboardInterrupt:
        print("\nDihentikan dengan Ctrl+C")

    finally:
        # Kirim perintah brake ke seluruh port saat keluar
        brake_frame = COMMANDS["x"]
        for ser in ser_list:
            try:
                ser.write(brake_frame)
                ser.flush()
                print(f"Brake dikirim ke {ser.port}")
                ser.close()
                print(f"Port {ser.port} ditutup.")
            except Exception as e:
                print(f"Gagal menutup port {ser.port}: {e}")

if __name__ == "__main__":
    main()