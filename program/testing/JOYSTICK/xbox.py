import sys
from evdev import InputDevice, ecodes
from inspect_mapping_copy import BTN_MAP, ABS_MAP, DPAD_MAP
from config import JOYSTICK_PATH


def parse_event(event):
    # 1. Menangani Tombol Digital
    if event.type == ecodes.EV_KEY:
        btn_name = BTN_MAP.get(event.code, f"Unknown_BTN_{event.code}")
        state = "TEKAN" if event.value == 1 else ("LEPAS" if event.value == 0 else "HOLD")
        print(f"[TOMBOL] {btn_name:<18} -> {state}")

    # 2. Menangani Sumbu Analog / D-Pad
    elif event.type == ecodes.EV_ABS:
        # Cek apakah event berasal dari D-Pad (Code 16 atau 17)
        if event.code in DPAD_MAP:
            direction_map = DPAD_MAP[event.code]
            if event.value in direction_map:
                print(f"[D-PAD]  {direction_map[event.value]:<18} -> TEKAN")
            elif event.value == 0:
                print(f"[D-PAD]  Code {event.code:<13} -> LEPAS")
        
        # Sumbu Analog Reguler
        elif event.code in ABS_MAP:
            axis_name = ABS_MAP[event.code]
            print(f"[ANALOG] {axis_name:<18} -> Nilai: {event.value}")

def main():
    try:
        dev = InputDevice(JOYSTICK_PATH)
        print(f"[+] Berhasil terhubung ke {dev.name} ({dev.path})")
        print("[+] Membaca input WGP13s...\n")

        for event in dev.read_loop():
            if event.type != ecodes.EV_SYN:
                parse_event(event)

    except PermissionError:
        print("[-] Akses ditolak. Jalankan perintah menggunakan 'sudo'.")
    except FileNotFoundError:
        print(f"[-] Device {JOYSTICK_PATH} tidak ditemukan.")
    except KeyboardInterrupt:
        print("\n[+] Selesai.")

if __name__ == "__main__":
    main()