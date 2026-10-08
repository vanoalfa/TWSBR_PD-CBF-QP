"""joystick_mapping.py - Mapping joystick Fantech WGP-13s untuk robot ATERA.

Joystick dibaca lewat evdev dari config.JOYSTICK_PORT. Bila port itu tidak ada, program mencari
sendiri perangkat joystick di /dev/input/.

Pemakaian (lihat atera_main.py):
    joystick = Joystick()
    joystick.buka()
    tombol = joystick.baca()      # daftar nama tombol yang baru ditekan, contoh ["LB", "MULAI"]
    joystick.arah                 # +1 maju, 0 lepas, -1 mundur
    joystick.belok                # -1 kiri, 0 lepas, +1 kanan
"""

import select

import config

try:
    import evdev
    from evdev import ecodes
except ImportError:
    evdev = None

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

class Joystick:
    def __init__(self):
        self.device = None
        self.nama = ""
        self.pesan = ""

        self.dpad_maju = 0       # dari D-Pad:  +1 maju, -1 mundur
        self.dpad_belok = 0      # dari D-Pad:  -1 kiri, +1 kanan
        self.analog_maju = 0     # dari analog kiri (Y)
        self.analog_belok = 0    # dari analog kiri (X) / analog kanan (X)

        self.arah = 0            # +1 maju, 0 lepas, -1 mundur
        self.belok = 0           # -1 kiri, 0 lepas, +1 kanan

    def terhubung(self):
        return self.device is not None

    def buka(self):
        """Buka joystick. Mengembalikan True bila berhasil."""
        if evdev is None:
            self.pesan = "modul evdev belum terpasang (pip install evdev)"
            return False

        # 1) Coba port dari config.py
        try:
            self.device = evdev.InputDevice(config.JOYSTICK_PORT)
        except Exception:
            self.device = None

        # 2) Bila gagal, cari perangkat yang punya Tombol A (304) dan D-Pad (16, 17)
        if self.device is None:
            for path in evdev.list_devices():
                try:
                    calon = evdev.InputDevice(path)
                    kemampuan = calon.capabilities()
                    tombol = kemampuan.get(ecodes.EV_KEY, [])
                    sumbu = [kode[0] for kode in kemampuan.get(ecodes.EV_ABS, [])]
                    if 304 in tombol and 16 in sumbu and 17 in sumbu:
                        self.device = calon
                        break
                    calon.close()
                except Exception:
                    pass

        if self.device is None:
            self.pesan = "joystick tidak ditemukan di %s maupun di /dev/input/" % config.JOYSTICK_PORT
            return False

        self.nama = self.device.name
        self.pesan = "%s (%s)" % (self.nama, self.device.path)
        return True

    def normalisasi_analog(self, kode, nilai):
        """Ubah nilai mentah analog menjadi -1.0 .. +1.0."""
        try:
            info = self.device.absinfo(kode)
            tengah = (info.max + info.min) / 2.0
            setengah = (info.max - info.min) / 2.0
            if setengah <= 0:
                return 0.0
            return (nilai - tengah) / setengah
        except Exception:
            return nilai / 32767.0

    def baca(self):
        """Baca semua event yang masuk. Mengembalikan daftar nama tombol yang baru ditekan."""
        ditekan = []
        if self.device is None:
            return ditekan

        try:
            siap, _, _ = select.select([self.device], [], [], 0)
            if not siap:
                return ditekan

            for event in self.device.read():
                # 1. Tombol digital
                if event.type == ecodes.EV_KEY:
                    if event.value == 1 and event.code in BTN_MAP:
                        ditekan.append(BTN_MAP[event.code])

                # 2. D-Pad dan analog
                elif event.type == ecodes.EV_ABS:
                    if event.code == 17:
                        nama = DPAD_MAP[17].get(event.value, "")
                        if nama == "PAD Atas":
                            self.dpad_maju = 1
                        elif nama == "PAD Bawah":
                            self.dpad_maju = -1
                        else:
                            self.dpad_maju = 0

                    elif event.code == 16:
                        nama = DPAD_MAP[16].get(event.value, "")
                        if nama == "PAD Kiri":
                            self.dpad_belok = -1
                        elif nama == "PAD Kanan":
                            self.dpad_belok = 1
                        else:
                            self.dpad_belok = 0

                    elif event.code in ABS_MAP:
                        nama = ABS_MAP[event.code]
                        nilai = self.normalisasi_analog(event.code, event.value)
                        if abs(nilai) < config.JOYSTICK_AMBANG_ANALOG:
                            langkah = 0
                        elif nilai > 0:
                            langkah = 1
                        else:
                            langkah = -1

                        if nama == "Analog Kiri (Y)":
                            self.analog_maju = -langkah      # analog didorong ke atas = nilai negatif = maju
                        elif nama in ("Analog Kiri (X)", "Analog Kanan (X)"):
                            self.analog_belok = langkah

        except (OSError, RuntimeError) as kesalahan:
            self.pesan = "joystick terputus: %s" % kesalahan
            self.device = None
            self.dpad_maju = 0
            self.dpad_belok = 0
            self.analog_maju = 0
            self.analog_belok = 0

        # D-Pad lebih diutamakan, analog dipakai bila D-Pad dilepas.
        if self.dpad_maju != 0:
            self.arah = self.dpad_maju
        else:
            self.arah = self.analog_maju

        if self.dpad_belok != 0:
            self.belok = self.dpad_belok
        else:
            self.belok = self.analog_belok

        return ditekan

    def tutup(self):
        if self.device is not None:
            try:
                self.device.close()
            except Exception:
                pass
            self.device = None


if __name__ == "__main__":
    # Uji joystick: python3 joystick_mapping.py  (tekan tombol, lihat namanya di terminal)
    import time

    joystick = Joystick()
    if not joystick.buka():
        print("GAGAL:", joystick.pesan)
    else:
        print("Joystick:", joystick.pesan)
        print("Tekan tombol / D-Pad. Ctrl+C untuk berhenti.")
        arah_lama = None
        belok_lama = None
        try:
            while joystick.terhubung():
                for nama in joystick.baca():
                    print("Tombol:", nama)
                if joystick.arah != arah_lama or joystick.belok != belok_lama:
                    print("arah = %+d | belok = %+d" % (joystick.arah, joystick.belok))
                    arah_lama = joystick.arah
                    belok_lama = joystick.belok
                time.sleep(0.01)
        except KeyboardInterrupt:
            pass
        joystick.tutup()