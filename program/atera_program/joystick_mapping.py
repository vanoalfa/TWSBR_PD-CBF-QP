"""joystick_mapping.py - mapping dan pembaca joystick Fantech WGP-13s (evdev, non-blocking).

Fungsi tombol (Konteks_ATERA.md, ditambah L3 / R3 untuk perekaman ISE):
    Tombol A : mode gerak Navigate (analog / D-Pad dipakai, Kp_theta = 0)
    Tombol B : titik nol s pindah ke posisi sekarang (kapan saja); saat balancing juga masuk mode Balancing
               (diam di tempat, Kp_theta aktif)
    Tombol X : matikan roda (percobaan disimpan)
    Tombol Y : kalibrasi IMU (robot diam dan tegak di titik seimbang) + titik nol s di posisi sekarang
    LB / RB  : mode kontrol PD / PD + CBF-QP
    LT / RT  : halaman GUI ke kiri / kanan
    MULAI    : mulai balancing (setelah kalibrasi; ditolak bila |s| > s_max dari titik nol)
    QUIT     : keluar program
    L3       : mulai satu PENGUJIAN (direkam config.DURASI_PENGUJIAN detik; dorong robot dengan tangan)
    R3       : simpan PERCOBAAN sekarang, perekaman baru dimulai (robot tetap balancing)
Perintah gerak (hanya mode Navigate): arah -1 / 0 / +1 dan belok -1 / 0 / +1, dari D-Pad atau analog kiri.
"""

import select
import time

import config

# Pemetaan Tombol Digital (Code EV_KEY)
BTN_MAP = {
    304: "Tombol A",
    305: "Tombol B",
    307: "Tombol X",
    308: "Tombol Y",
    310: "LB",
    311: "RB",
    312: "LT (Digital)",
    313: "RT (Digital)",
    314: "QUIT",
    315: "MULAI",
    317: "L3",
    318: "R3",
}

# Pemetaan Sumbu Analog Kontinu (Code EV_ABS)
ABS_MAP = {
    0: "Analog Kiri (Y)",
    1: "Analog Kiri (X)",
    2: "Analog Kanan (Y)",
    5: "Analog Kanan (X)",
    9: "Analog RT",
    10: "Analog LT",
}

# Pemetaan D-Pad / PAD (Code EV_ABS dengan Nilai Diskrit)
DPAD_MAP = {
    16: {-1: "PAD Kiri", 1: "PAD Kanan"},
    17: {-1: "PAD Atas", 1: "PAD Bawah"},
}

# Aksi program untuk tiap tombol
AKSI = {
    "Tombol A": "NAVIGATE",
    "Tombol B": "BALANCING",
    "Tombol X": "RODA_MATI",
    "Tombol Y": "KALIBRASI",
    "LB": "MODE_PD",
    "RB": "MODE_PD_CBF",
    "LT (Digital)": "HALAMAN_KIRI",
    "RT (Digital)": "HALAMAN_KANAN",
    "QUIT": "KELUAR",
    "MULAI": "MULAI",
    "L3": "PENGUJIAN",
    "R3": "SIMPAN",
}

# ASUMSI: analog kiri code 0 = maju/mundur (stik ke atas bernilai negatif = maju), code 1 = belok (kanan = positif).
ANALOG_MAJU = 0
ANALOG_BELOK = 1

BANTUAN = (
    "Tombol Y : kalibrasi IMU (robot diam, tegak di titik seimbang) + s = 0 di sini",
    "MULAI    : mulai balancing, mode Balancing (ditolak bila |s| > %g m dari titik nol)" % config.s_max,
    "Tombol X : matikan roda, percobaan disimpan",
    "LB / RB  : mode kontrol PD / PD + CBF-QP",
    "Tombol B : s = 0 di sini (kapan saja); saat balancing juga mode Balancing (diam di tempat)",
    "Tombol A : mode Navigate (D-Pad / analog kiri: maju-mundur dan belok)",
    "L3       : mulai PENGUJIAN %.0f s (dorong robot dengan tangan setelah menekan L3)" % config.DURASI_PENGUJIAN,
    "R3       : simpan PERCOBAAN sekarang, rekaman baru dimulai",
    "LT / RT  : halaman GUI",
    "QUIT     : keluar (roda dimatikan, data disimpan)",
)


def _arah(nilai):
    ambang = float(config.JOYSTICK_AMBANG_ANALOG)
    return 1 if nilai > ambang else (-1 if nilai < -ambang else 0)


class Joystick:
    def __init__(self, path=config.JOYSTICK_PORT):
        self.path = path
        self.device = None
        self.name = ""
        self.problem = "belum dibuka"
        self.last_event = "belum ada event"
        self._rentang = {}             # code -> (tengah, setengah rentang)
        self._analog = {}              # code -> -1..1
        self._dpad = {}                # code -> -1 / 0 / +1
        self._coba_lagi = 0.0

    @property
    def connected(self):
        return self.device is not None

    def open(self):
        try:
            import evdev
            self.device = evdev.InputDevice(self.path)
            self.name = self.device.name
            self._rentang = {}
            for code in ABS_MAP:
                try:
                    info = self.device.absinfo(code)
                    if info.max > info.min:
                        self._rentang[code] = (0.5 * (info.max + info.min), 0.5 * (info.max - info.min))
                except Exception:
                    pass
            self.problem = ""
            return True
        except Exception as exc:
            self.device = None
            teks = str(exc)
            if "No such file" in teks or "Errno 2" in teks:
                self.problem = "%s tidak ada -> joystick belum terhubung, atau nomor event berubah" % self.path
            elif "Permission denied" in teks or "Errno 13" in teks:
                self.problem = "tidak ada izin membuka %s -> sudo usermod -aG input $USER, lalu login ulang" % self.path
            elif "No module named" in teks:
                self.problem = "modul evdev belum terpasang -> pip install evdev"
            else:
                self.problem = "%s: %s" % (self.path, teks)
            return False

    def poll(self):
        """Baca semua event yang menunggu. Mengembalikan daftar aksi dari tombol yang baru ditekan."""
        if self.device is None:
            if time.monotonic() >= self._coba_lagi:
                self._coba_lagi = time.monotonic() + float(config.JOYSTICK_RECONNECT_S)
                self.open()
            return []
        aksi = []
        try:
            from evdev import ecodes
            while select.select([self.device.fd], [], [], 0)[0]:
                for event in self.device.read():
                    if event.type == ecodes.EV_KEY:
                        nama = BTN_MAP.get(event.code, "tidak ada di BTN_MAP")
                        self.last_event = "tombol code %d nilai %d -> %s" % (event.code, event.value, nama)
                        if event.value == 1 and nama in AKSI:
                            aksi.append(AKSI[nama])
                    elif event.type == ecodes.EV_ABS:
                        nama = ABS_MAP.get(event.code) or ("D-Pad" if event.code in DPAD_MAP else "tidak dipetakan")
                        self.last_event = "sumbu code %d nilai %d -> %s" % (event.code, event.value, nama)
                        if event.code in DPAD_MAP:
                            self._dpad[event.code] = max(-1, min(1, int(event.value)))
                        elif event.code in ABS_MAP:
                            tengah, setengah = self._rentang.get(event.code, (0.0, 32767.5))
                            self._analog[event.code] = max(-1.0, min(1.0, (event.value - tengah) / setengah))
        except (OSError, RuntimeError) as exc:
            self.device = None
            self.problem = "joystick terputus saat dipakai (%s)" % exc
            self._analog, self._dpad = {}, {}           # perintah gerak nol saat joystick hilang
        return aksi

    def perintah(self):
        """(arah, belok): arah +1 maju / -1 mundur, belok +1 kanan / -1 kiri, 0 = lepas. D-Pad didahulukan."""
        arah = -self._dpad.get(config.DPAD_MAJU_CODE, 0)          # PAD Atas = -1 = maju
        if arah == 0:
            arah = _arah(-self._analog.get(ANALOG_MAJU, 0.0))
        belok = self._dpad.get(config.DPAD_BELOK_CODE, 0)          # PAD Kanan = +1 = belok kanan
        if belok == 0:
            belok = _arah(self._analog.get(ANALOG_BELOK, 0.0))
        return arah, belok

    def close(self):
        if self.device is not None:
            try:
                self.device.close()
            except Exception:
                pass
            self.device = None