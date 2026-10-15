"""joystick_mapping.py - mapping dan pembaca joystick Fantech WGP-13s (evdev, non-blocking), dan keyboard terminal.

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

Keyboard terminal (KeyboardTerminal, terminal SSH tempat GUI berjalan): aksi sama dengan joystick, lihat
TOMBOL_KEYBOARD. Panah kiri / kanan = halaman GUI. W A S D = gerak (mode Navigate); karena terminal tidak memberi
tahu saat tombol dilepas, "ditahan" ditebak dari pengulangan otomatis keyboard (config.KEYBOARD_TAHAN_*).
"""

import os
import select
import sys
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


# Keyboard terminal: huruf (huruf kecil / besar sama) -> aksi (nama aksi sama dengan AKSI joystick)
TOMBOL_KEYBOARD = {
    "y": "KALIBRASI",
    "m": "MULAI", "\r": "MULAI", "\n": "MULAI",          # m atau Enter
    "x": "RODA_MATI", " ": "RODA_MATI",                   # x atau spasi
    "b": "BALANCING",
    "n": "NAVIGATE",
    "1": "MODE_PD",
    "2": "MODE_PD_CBF",
    "t": "PENGUJIAN",
    "e": "SIMPAN",
    "q": "KELUAR",
}
PANAH = {"C": "HALAMAN_KANAN", "D": "HALAMAN_KIRI"}      # ESC [ C / ESC [ D (atau ESC O C / ESC O D)
GERAK_KEYBOARD = {"w": ("arah", 1), "s": ("arah", -1), "a": ("belok", -1), "d": ("belok", 1)}

BANTUAN_KEYBOARD = (
    "Keyboard terminal: Y kalibrasi | M / Enter MULAI | X / Spasi roda mati | B Balancing | N Navigate",
    "  1 / 2 mode PD / PD+CBF | T PENGUJIAN | E simpan PERCOBAAN | Q keluar | Panah kiri / kanan halaman GUI",
    "  W / S maju / mundur, A / D belok (tahan; hanya mode Navigate, berhenti +-%.1f s setelah dilepas)"
    % config.KEYBOARD_TAHAN_ULANG_S,
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


class KeyboardTerminal:
    """Keyboard dari terminal tempat program berjalan (juga lewat SSH). Terminal dipasang ke mode cbreak (huruf langsung
    terbaca tanpa Enter, tidak ditampilkan; Ctrl+C tetap menghentikan program). close() mengembalikan terminal.
    fd: None = stdin; boleh fd lain (untuk uji)."""

    def __init__(self, fd=None):
        self.fd = fd
        self.aktif = False
        self.problem = "belum dibuka"
        self.last_event = "belum ada tombol"
        self._asli = None                  # pengaturan terminal sebelum cbreak (dikembalikan saat close)
        self._sisa = ""                    # potongan urutan ESC yang belum lengkap
        self._gerak = {"arah": [0, 0.0, 0], "belok": [0, 0.0, 0]}   # nilai, waktu huruf terakhir, jumlah huruf

    def open(self):
        if not config.KEYBOARD_AKTIF:
            self.problem = "dimatikan (config.KEYBOARD_AKTIF = False)"
            return False
        if self.fd is None:
            if not sys.stdin.isatty():
                self.problem = "stdin bukan terminal (program dijalankan tanpa terminal)"
                return False
            self.fd = sys.stdin.fileno()
            try:
                import termios
                import tty
                self._asli = termios.tcgetattr(self.fd)
                tty.setcbreak(self.fd)
            except Exception as exc:
                self.problem = "terminal tidak bisa dipasang ke mode cbreak: %s" % exc
                return False
        self.aktif = True
        self.problem = ""
        return True

    def close(self):
        if self._asli is not None:
            try:
                import termios
                termios.tcsetattr(self.fd, termios.TCSADRAIN, self._asli)
            except Exception:
                pass
            self._asli = None
        self.aktif = False

    def poll(self):
        """Baca semua huruf yang menunggu (tanpa menunggu). Mengembalikan daftar aksi."""
        if not self.aktif:
            return []
        teks = self._sisa
        try:
            while select.select([self.fd], [], [], 0)[0]:
                data = os.read(self.fd, 256)
                if not data:
                    break
                teks += data.decode("utf-8", "ignore")
        except OSError as exc:
            self.aktif = False
            self.problem = "keyboard terminal gagal dibaca (%s)" % exc
            return []
        self._sisa = ""
        aksi = []
        sekarang = time.monotonic()
        i = 0
        while i < len(teks):
            huruf = teks[i]
            if huruf == "\x1b":                               # urutan ESC: panah dan tombol khusus lain
                if i + 1 >= len(teks) or (teks[i + 1] in "[O" and i + 2 >= len(teks)):
                    self._sisa = teks[i:]                      # belum lengkap, lanjut di poll berikutnya
                    break
                if teks[i + 1] in "[O":
                    j = i + 2
                    while j < len(teks) and not teks[j].isalpha() and teks[j] != "~":
                        j += 1
                    kode = teks[j] if j < len(teks) else ""
                    if kode in PANAH:
                        aksi.append(PANAH[kode])
                        self.last_event = "panah %s -> %s" % ("kanan" if kode == "C" else "kiri", PANAH[kode])
                    i = j + 1
                else:
                    i += 1                                     # ESC sendiri: diabaikan
                continue
            kecil = huruf.lower()
            if kecil in GERAK_KEYBOARD:
                nama, nilai = GERAK_KEYBOARD[kecil]
                gerak = self._gerak[nama]
                if gerak[0] != nilai:
                    gerak[2] = 0                               # arah baru: hitung ulang tekanan
                gerak[0], gerak[1], gerak[2] = nilai, sekarang, gerak[2] + 1
                self.last_event = "%s -> %s %+d" % (kecil.upper(), nama, nilai)
            elif kecil in TOMBOL_KEYBOARD:
                aksi.append(TOMBOL_KEYBOARD[kecil])
                if TOMBOL_KEYBOARD[kecil] == "RODA_MATI":      # roda mati: perintah gerak ikut berhenti
                    for gerak in self._gerak.values():
                        gerak[0] = 0
                self.last_event = "%s -> %s" % (repr(huruf) if huruf in " \r\n" else huruf.upper(),
                                                TOMBOL_KEYBOARD[kecil])
            i += 1
        return aksi

    def perintah(self):
        """(arah, belok) dari W A S D: dianggap ditahan selama pengulangan otomatis keyboard masih datang."""
        sekarang = time.monotonic()
        hasil = []
        for nama in ("arah", "belok"):
            gerak = self._gerak[nama]
            tahan = config.KEYBOARD_TAHAN_AWAL_S if gerak[2] <= 1 else config.KEYBOARD_TAHAN_ULANG_S
            if gerak[0] != 0 and sekarang - gerak[1] > tahan:
                gerak[0], gerak[2] = 0, 0
            hasil.append(gerak[0])
        return hasil[0], hasil[1]