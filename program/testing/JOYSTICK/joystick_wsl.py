#!/usr/bin/env python3
"""joystick_wsl.py - Cari joystick di WSL dan tampilkan mapping tombolnya.

Jalankan:  python3 joystick_wsl.py

Yang dikerjakan:
  1. Memeriksa apakah kernel WSL dan modul evdev siap membaca joystick.
  2. Menampilkan perangkat USB yang terlihat oleh WSL.
  3. Menampilkan semua /dev/input/event* dan memilih yang berupa joystick.
  4. Mencetak setiap tombol / D-Pad / analog yang ditekan (code, nilai, nama di mapping ATERA).

Port yang ditemukan dicetak di akhir langkah 3. Program ATERA (joystick_mapping.py) sudah mencari
port sendiri, jadi config.JOYSTICK_PORT tidak wajib diubah.
"""

import glob
import gzip
import os
import select
import sys

# Mapping ATERA (sama dengan joystick_mapping.py)
BTN_MAP = {
    306: "Tombol A", 305: "Tombol B", 307: "Tombol X", 304: "Tombol Y",
    308: "LB", 309: "RB", 310: "LT (Digital)", 311: "RT (Digital)",
    312: "QUIT", 313: "MULAI", 314: "L3", 315: "R3",
}
ABS_MAP = {
    1: "Analog Kiri (Y)", 0: "Analog Kiri (X)", 5: "Analog Kanan (Y)", 2: "Analog Kanan (X)",
    9: "Analog RT", 10: "Analog LT",
}
DPAD_MAP = {
    16: {-1: "PAD Kiri", 1: "PAD Kanan", 0: "PAD Kiri/Kanan dilepas"},
    17: {-1: "PAD Atas", 1: "PAD Bawah", 0: "PAD Atas/Bawah dilepas"},
}

AMBANG_CETAK_ANALOG = 0.25      # analog baru dicetak bila berubah lebih dari ini (0..1)


def cetak_cara_usbipd():
    print("")
    print("  Cara meneruskan joystick USB dari Windows ke WSL (PowerShell sebagai Administrator):")
    print("    winget install usbipd                 (sekali saja)")
    print("    usbipd list                            (catat BUSID joystick, contoh 2-3)")
    print("    usbipd bind --busid 2-3                (sekali saja)")
    print("    usbipd attach --wsl --busid 2-3        (ulangi tiap joystick dicabut / WSL dimatikan)")
    print("  Lalu jalankan lagi program ini di WSL.")


def periksa_kernel():
    """Cek driver di kernel WSL. Mengembalikan False bila evdev pasti tidak tersedia."""
    print("[1] Kernel:", os.uname().release)
    pilihan = ["CONFIG_INPUT_EVDEV", "CONFIG_USB_HID", "CONFIG_HID_GENERIC",
               "CONFIG_JOYSTICK_XPAD", "CONFIG_INPUT_JOYDEV", "CONFIG_USBIP_VHCI_HCD"]
    try:
        with gzip.open("/proc/config.gz", "rt") as f:
            isi = f.read()
    except Exception:
        print("    /proc/config.gz tidak bisa dibaca, pemeriksaan driver dilewati.")
        return True

    hasil = {}
    for nama in pilihan:
        if nama + "=y" in isi:
            hasil[nama] = "ada (y)"
        elif nama + "=m" in isi:
            hasil[nama] = "modul (m)"
        else:
            hasil[nama] = "TIDAK ADA"
        print("    %-24s %s" % (nama, hasil[nama]))

    if hasil["CONFIG_INPUT_EVDEV"] == "TIDAK ADA":
        print("    -> Kernel WSL ini tidak punya evdev, jadi /dev/input/event* tidak akan muncul.")
        print("       Perbarui WSL dari PowerShell: wsl --update , lalu wsl --shutdown.")
        return False
    if hasil["CONFIG_JOYSTICK_XPAD"] == "TIDAK ADA" and hasil["CONFIG_USB_HID"] == "TIDAK ADA":
        print("    -> Tidak ada driver joystick (xpad maupun usbhid). Joystick akan terlihat di USB")
        print("       tetapi tidak menjadi /dev/input/event*. Perlu kernel WSL dengan driver tersebut.")
    elif hasil["CONFIG_JOYSTICK_XPAD"] == "TIDAK ADA":
        print("    -> Driver xpad tidak ada. Pakai joystick di mode D-input (bukan X-input) supaya dibaca usbhid.")
    for nama in ("CONFIG_JOYSTICK_XPAD", "CONFIG_USB_HID"):
        if hasil[nama] == "modul (m)":
            print("    -> %s berupa modul. Bila joystick tidak muncul: sudo modprobe %s"
                  % (nama, "xpad" if "XPAD" in nama else "usbhid"))
    return True


def periksa_usb():
    print("[2] Perangkat USB yang terlihat oleh WSL:")
    ada = False
    for folder in sorted(glob.glob("/sys/bus/usb/devices/*")):
        path_nama = os.path.join(folder, "product")
        if not os.path.exists(path_nama):
            continue
        try:
            nama = open(path_nama).read().strip()
            vid = open(os.path.join(folder, "idVendor")).read().strip()
            pid = open(os.path.join(folder, "idProduct")).read().strip()
        except Exception:
            continue
        if "host controller" in nama.lower() or "vhci" in nama.lower():
            continue
        print("    %s:%s  %s" % (vid, pid, nama))
        ada = True
    if not ada:
        print("    (tidak ada) -> joystick belum diteruskan ke WSL.")
        cetak_cara_usbipd()
    return ada


def cari_joystick(evdev):
    """Tampilkan semua perangkat input, kembalikan joystick yang dipilih (atau None)."""
    from evdev import ecodes

    print("[3] Perangkat input di /dev/input/:")
    daftar = sorted(glob.glob("/dev/input/event*"), key=lambda p: int(p.replace("/dev/input/event", "")))
    if len(daftar) == 0:
        print("    (tidak ada /dev/input/event*)")
        return None

    pilihan = None
    nilai_terbaik = 0
    for path in daftar:
        try:
            dev = evdev.InputDevice(path)
        except PermissionError:
            print("    %-20s TIDAK BISA DIBUKA (izin). Jalankan: sudo chmod a+r %s" % (path, path))
            continue
        except Exception as kesalahan:
            print("    %-20s tidak bisa dibuka: %s" % (path, kesalahan))
            continue

        kemampuan = dev.capabilities()
        tombol = kemampuan.get(ecodes.EV_KEY, [])
        sumbu = [kode[0] for kode in kemampuan.get(ecodes.EV_ABS, [])]

        # Nilai kecocokan dengan mapping ATERA
        nilai = 0
        for kode in BTN_MAP:
            if kode in tombol:
                nilai = nilai + 1
        if 16 in sumbu and 17 in sumbu:
            nilai = nilai + 5

        print("    %-20s %-40s tombol cocok %2d/%d | D-Pad %s"
              % (path, dev.name[:40], nilai - (5 if (16 in sumbu and 17 in sumbu) else 0), len(BTN_MAP),
                 "ada" if (16 in sumbu and 17 in sumbu) else "tidak"))

        if nilai > nilai_terbaik:
            if pilihan is not None:
                pilihan.close()
            pilihan = dev
            nilai_terbaik = nilai
        else:
            dev.close()

    if pilihan is None or nilai_terbaik < 3:
        return None
    return pilihan


def tampilkan_event(dev):
    from evdev import ecodes

    print("[4] Tekan tombol, D-Pad, dan analog. Ctrl+C untuk berhenti.")
    print("    %-8s %-6s %-8s %s" % ("jenis", "code", "nilai", "nama di mapping ATERA"))
    analog_terakhir = {}
    try:
        while True:
            siap, _, _ = select.select([dev], [], [], 0.5)
            if not siap:
                continue
            for event in dev.read():
                if event.type == ecodes.EV_KEY:
                    if event.value == 2:
                        continue
                    nama = BTN_MAP.get(event.code, "(belum ada di BTN_MAP)")
                    aksi = "tekan" if event.value == 1 else "lepas"
                    print("    %-8s %-6d %-8s %s" % ("EV_KEY", event.code, aksi, nama))

                elif event.type == ecodes.EV_ABS:
                    if event.code in DPAD_MAP:
                        nama = DPAD_MAP[event.code].get(event.value, "?")
                        print("    %-8s %-6d %-8d %s" % ("EV_ABS", event.code, event.value, nama))
                    else:
                        try:
                            info = dev.absinfo(event.code)
                            tengah = (info.max + info.min) / 2.0
                            setengah = (info.max - info.min) / 2.0
                            norm = (event.value - tengah) / setengah if setengah > 0 else 0.0
                        except Exception:
                            norm = event.value / 32767.0
                        if abs(norm - analog_terakhir.get(event.code, 0.0)) < AMBANG_CETAK_ANALOG:
                            continue
                        analog_terakhir[event.code] = norm
                        nama = ABS_MAP.get(event.code, "(belum ada di ABS_MAP)")
                        print("    %-8s %-6d %-+8.2f %s" % ("EV_ABS", event.code, norm, nama))
    except KeyboardInterrupt:
        print("\nSelesai.")
    except OSError as kesalahan:
        print("\nJoystick terputus:", kesalahan)


def main():
    print("=== Pemeriksaan joystick untuk WSL ===")
    kernel_siap = periksa_kernel()
    usb_ada = periksa_usb()

    try:
        import evdev
    except ImportError:
        print("[3] Modul evdev belum terpasang. Jalankan: pip install evdev")
        return 1

    dev = cari_joystick(evdev)
    if dev is None:
        print("")
        print("HASIL: joystick TIDAK ditemukan.")
        if not usb_ada:
            print("  Penyebab: joystick belum diteruskan dari Windows ke WSL (lihat cara usbipd di atas).")
        elif not kernel_siap:
            print("  Penyebab: kernel WSL tidak mendukung evdev (lihat langkah [1]).")
        else:
            print("  Joystick terlihat di USB tetapi tidak menjadi /dev/input/event*, atau tidak boleh dibaca.")
            print("  Coba: sudo chmod a+r /dev/input/event*   lalu jalankan lagi.")
            print("  Bila tetap tidak ada, lihat catatan driver di langkah [1].")
        return 1

    print("")
    print("HASIL: joystick ditemukan -> %s (%s)" % (dev.path, dev.name))
    print("  Bila ingin diisi manual di config.py:  JOYSTICK_PORT = \"%s\"" % dev.path)
    print("")
    tampilkan_event(dev)
    dev.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())