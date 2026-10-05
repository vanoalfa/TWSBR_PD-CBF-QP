"""joystick_mapping.py - mapping dan pembaca joystick Fantech WGP-13s (evdev, non-blocking).

Fungsi tiap tombol (Konteks_ATERA.md):
    Tombol A  : aktifkan perintah gerak (analog + D-Pad)
    Tombol B  : mode balancing saja (perintah gerak diabaikan)
    Tombol X  : matikan roda
    Tombol Y  : kalibrasi sensor IMU
    LB        : mode kontrol PD
    RB        : mode kontrol PD + CBF-QP
    LT / RT   : pindah halaman GUI ke kiri / kanan
    QUIT      : keluar program
    MULAI     : mulai balancing (setelah kalibrasi)
    L3 / R3   : uji roda kiri / kanan langsung, tanpa lewat kontrol (TAMBAHAN untuk debug)
"""

from __future__ import annotations

import logging
import select
import time
from typing import List, Optional

import config

LOGGER = logging.getLogger(__name__)

# Pemetaan Tombol Digital (Code EV_KEY)
BTN_MAP = {
    304: "Tombol A",        # Press this if wanna using analog (ABS_MAP)
    305: "Tombol B",        # Press this if wanna Just balancing (can't and no joystick (ABS_MAP)) (Mode Balancing)
    307: "Tombol X",        # Turn off wheel
    308: "Tombol Y",        # Kalibrasi Sensor IMU
    310: "LB",              # Untuk switch ke mode PD
    311: "RB",              # Untuk Switch ke Mode PD + CBF-QP
    312: "LT (Digital)",    # Pindah pages GUI ke kiri
    313: "RT (Digital)",    # Pindah pages GUI ke kanan
    314: "QUIT",            # Quit Program
    315: "MULAI",           # Starting Program (Balancing after kalibrasi)
    317: "L3",              # Uji roda kiri  (debug; hanya saat roda mati, robot diangkat)
    318: "R3",              # Uji roda kanan (debug; hanya saat roda mati, robot diangkat)
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
ACTION_START = "START"
ACTION_MOTOR_OFF = "MOTOR_OFF"
ACTION_CALIBRATE = "CALIBRATE"
ACTION_DRIVE_ON = "DRIVE_ON"
ACTION_BALANCE_ONLY = "BALANCE_ONLY"
ACTION_MODE_PD = "MODE_PD"
ACTION_MODE_PD_CBF = "MODE_PD_CBF"
ACTION_PAGE_PREV = "PAGE_PREV"
ACTION_PAGE_NEXT = "PAGE_NEXT"
ACTION_QUIT = "QUIT"
ACTION_TEST_LEFT = "TEST_LEFT"
ACTION_TEST_RIGHT = "TEST_RIGHT"

BTN_ACTION = {
    "Tombol A": ACTION_DRIVE_ON,
    "Tombol B": ACTION_BALANCE_ONLY,
    "Tombol X": ACTION_MOTOR_OFF,
    "Tombol Y": ACTION_CALIBRATE,
    "LB": ACTION_MODE_PD,
    "RB": ACTION_MODE_PD_CBF,
    "LT (Digital)": ACTION_PAGE_PREV,
    "RT (Digital)": ACTION_PAGE_NEXT,
    "QUIT": ACTION_QUIT,
    "MULAI": ACTION_START,
    "L3": ACTION_TEST_LEFT,
    "R3": ACTION_TEST_RIGHT,
}

# Sumbu analog yang dipakai untuk gerak
ANALOG_DRIVE = ("Analog Kiri (Y)",)                         # dorong ke atas = maju
ANALOG_TURN = ("Analog Kiri (X)", "Analog Kanan (X)")       # ke kanan = belok kanan

HELP_LINES = (
    "Tombol Y        : kalibrasi IMU (robot diam, tegak di titik seimbang)",
    "MULAI           : mulai balancing",
    "Tombol X        : matikan roda",
    "LB / RB         : mode PD / mode PD + CBF-QP",
    "Tombol A        : aktifkan perintah gerak (analog + D-Pad)",
    "Tombol B        : balancing saja, perintah gerak diabaikan",
    "Analog Kiri (Y) : maju / mundur        Analog (X): belok",
    "PAD Atas/Bawah  : maju / mundur        PAD Kiri/Kanan: belok   (lepas = berhenti)",
    "LT / RT         : halaman GUI ke kiri / kanan",
    "QUIT            : keluar program (motor dimatikan)",
    "L3 / R3         : uji roda kiri / kanan (roda mati, robot DIANGKAT)",
)


class Joystick:
    def __init__(self, path: str = config.JOYSTICK_PORT) -> None:
        self.path = path
        self.device = None
        self.name = ""
        self._ranges = {}               # code -> (tengah, setengah rentang)
        self._axes = {}                 # nama sumbu analog -> nilai -1..1
        self._dpad_drive = 0.0
        self._dpad_turn = 0.0
        self._next_retry = 0.0
        self.last_button = ""
        self.last_event = "belum ada event"     # event mentah terakhir, untuk mengecek mapping
        self.event_count = 0
        self.problem = "belum dibuka"
        self._devices: List[str] = []
        self._devices_until = 0.0

    @property
    def connected(self) -> bool:
        return self.device is not None

    def open(self) -> bool:
        try:
            import evdev
            self.device = evdev.InputDevice(self.path)
            self.name = self.device.name
            self._ranges = {}
            for code in ABS_MAP:
                try:
                    info = self.device.absinfo(code)
                    centre = 0.5 * (info.max + info.min)
                    half = 0.5 * (info.max - info.min)
                    if half > 0:
                        self._ranges[code] = (centre, half)
                except Exception:
                    pass
            LOGGER.info("Joystick terhubung: %s (%s)", self.name, self.path)
            self.problem = ""
            return True
        except Exception as exc:
            self.device = None
            text = str(exc)
            if "No such file" in text or "Errno 2" in text:
                problem = f"{self.path} tidak ada -> joystick belum tercolok/terhubung, atau nomor event berubah"
            elif "Permission denied" in text or "Errno 13" in text:
                problem = f"tidak ada izin membuka {self.path} -> sudo usermod -aG input $USER, lalu login ulang"
            elif "No module named" in text:
                problem = "modul evdev belum terpasang -> pip install evdev"
            else:
                problem = f"{self.path}: {text}"
            if problem != self.problem:
                LOGGER.warning("Joystick: %s", problem)
            self.problem = problem
            return False

    def available_devices(self) -> List[str]:
        """Daftar perangkat input, untuk membantu mencari JOYSTICK_PORT yang benar."""
        now = time.monotonic()
        if now < self._devices_until:
            return self._devices
        self._devices_until = now + 3.0
        try:
            import evdev
            self._devices = [f"{p}: {evdev.InputDevice(p).name}" for p in evdev.list_devices()]
        except Exception as exc:
            self._devices = [f"(tidak bisa membaca daftar perangkat: {exc})"]
        return self._devices

    def _release_all(self) -> None:
        self._axes = {}
        self._dpad_drive = 0.0
        self._dpad_turn = 0.0

    def poll(self) -> List[str]:
        """Baca semua event yang menunggu. Kembalikan daftar aksi (tombol yang baru ditekan)."""
        if self.device is None:
            now = time.monotonic()
            if now >= self._next_retry:
                self._next_retry = now + float(config.JOYSTICK_RECONNECT_S)
                self.open()
            return []
        actions: List[str] = []
        try:
            from evdev import ecodes
            while True:
                ready, _, _ = select.select([self.device.fd], [], [], 0)
                if not ready:
                    break
                for event in self.device.read():
                    if event.type == ecodes.EV_KEY:
                        self.event_count += 1
                        self.last_event = (f"tombol code {event.code} nilai {event.value} -> "
                                           f"{BTN_MAP.get(event.code, 'TIDAK ADA di BTN_MAP')}")
                    elif event.type == ecodes.EV_ABS:
                        self.event_count += 1
                        label = ABS_MAP.get(event.code) or ("D-Pad" if event.code in DPAD_MAP else "TIDAK ADA di ABS_MAP")
                        self.last_event = f"sumbu code {event.code} nilai {event.value} -> {label}"
                    if event.type == ecodes.EV_KEY and event.value == 1:
                        name = BTN_MAP.get(event.code, "")
                        if name:
                            self.last_button = name
                        if name in BTN_ACTION:
                            actions.append(BTN_ACTION[name])
                    elif event.type == ecodes.EV_ABS:
                        self._handle_abs(event.code, event.value)
        except (OSError, RuntimeError) as exc:
            LOGGER.error("Joystick terputus: %s", exc)
            self.device = None
            self.problem = f"joystick terputus saat dipakai ({exc})"
            self._release_all()         # perintah gerak nol saat joystick hilang
        return actions

    def _handle_abs(self, code: int, value: int) -> None:
        if code in DPAD_MAP:
            # nilai negatif (Atas / Kiri) = maju pada sumbu gerak, belok kiri pada sumbu belok
            if code == int(config.DPAD_DRIVE_CODE):
                self._dpad_drive = -float(max(-1, min(1, value)))
            elif code == int(config.DPAD_TURN_CODE):
                self._dpad_turn = float(max(-1, min(1, value)))
        elif code in ABS_MAP:
            centre, half = self._ranges.get(code, (0.0, 32767.5))
            norm = max(-1.0, min(1.0, (value - centre) / half))
            if abs(norm) < float(config.JOYSTICK_DEADZONE):
                norm = 0.0
            self._axes[ABS_MAP[code]] = norm

    def command(self) -> tuple:
        """(drive, turn), masing-masing -1..1. drive > 0 = maju, turn > 0 = belok kanan."""
        drive = 0.0
        for name in ANALOG_DRIVE:
            value = -self._axes.get(name, 0.0)          # stik ke atas bernilai negatif
            if abs(value) > abs(drive):
                drive = value
        if drive == 0.0:
            drive = self._dpad_drive
        turn = 0.0
        for name in ANALOG_TURN:
            value = self._axes.get(name, 0.0)
            if abs(value) > abs(turn):
                turn = value
        if turn == 0.0:
            turn = self._dpad_turn
        return drive, turn

    def close(self) -> None:
        if self.device is not None:
            try:
                self.device.close()
            except Exception:
                pass
            self.device = None