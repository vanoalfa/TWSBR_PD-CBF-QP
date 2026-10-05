"""ddsm115.py - driver motor roda Waveshare DDSM115 (RS485) untuk ATERA.

Protokol (wiki Waveshare DDSM115): 115200 8N1, frame 10 byte, CRC-8/MAXIM, maks 500 Hz.
    perintah : [ID, 0x64, nilai_hi, nilai_lo, 0, 0, accel, brake, 0, CRC]
    balasan  : [ID, mode, arus_hi, arus_lo, rpm_hi, rpm_lo, pos_hi, pos_lo, error, CRC]
    mode     : [ID, 0xA0, 0, 0, 0, 0, 0, 0, 0, mode]   (tanpa CRC, tidak dibalas)
    arus     : -32767..32767 <-> -8..8 A     posisi : 0..32767 <-> 0..360 derajat

Perbedaan dari driver lama (apalah_bisa/ddsm115.py), frame yang dikirim sama:
- Perintah dalam TORSI [N m] (u dari PD / CBF-QP), dikonversi ke arus lewat config.MOTOR_KT.
- Kedua motor ditulis dulu baru dibaca, jadi waktu tunggu balasan berjalan paralel.
- theta dihitung dari posisi encoder (di-unwrap jadi multi-putaran) dan dtheta dari
  turunan posisi + low-pass, karena rpm umpan balik motor tertinggal sekitar 0.1 s.
- Satu frame hilang tidak langsung FAULT: baru setelah MOTOR_MAX_COMM_ERRORS berturut-turut.
- Tiap kegagalan diberi penjelasan (motor.problem) untuk ditampilkan di GUI, dan motor yang
  gagal diinisialisasi tidak menghentikan program: bisa dicoba ulang dengan initialize().

Kerangka robot: arus/torsi/theta positif = roda ke DEPAN (tanda diatur di config.py).
"""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass
from typing import List, Optional

import serial

import config

LOGGER = logging.getLogger(__name__)

MODE_CURRENT = 0x01
MODE_SPEED = 0x02
MODE_POSITION = 0x03
MODE_NAMES = {MODE_CURRENT: "current loop", MODE_SPEED: "speed loop", MODE_POSITION: "position loop"}
CMD_CONTROL = 0x64
CMD_SWITCH_MODE = 0xA0
FRAME_LEN = 10
ACCEL_TIME = 3                  # sama dengan driver lama; hanya berpengaruh di speed loop

RAW_PER_AMP = config.MOTOR_RAW_FULL_SCALE / config.MOTOR_CURRENT_FULL_SCALE_A
RAD_PER_COUNT = 2.0 * math.pi / config.MOTOR_POSITION_COUNTS
RPM_TO_RAD_S = 2.0 * math.pi / 60.0

ERROR_BITS = ((0x01, "sensor"), (0x02, "overcurrent"), (0x04, "phase overcurrent"),
              (0x08, "stall"), (0x10, "troubleshooting"))


class DDSM115Error(Exception):
    """Kesalahan komunikasi / protokol DDSM115."""


def crc8_maxim(data: bytes) -> int:
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8C if crc & 0x01 else crc >> 1
    return crc & 0xFF


def decode_error(code: int) -> str:
    names = [name for bit, name in ERROR_BITS if code & bit]
    return ", ".join(names) if names else "-"


def explain_port_error(exc: Exception, port: str) -> str:
    """Ubah exception pembukaan/penulisan port menjadi penjelasan + saran."""
    text = str(exc)
    if "No such file" in text or "Errno 2" in text:
        return (f"port {port} tidak ada -> USB-RS485 tidak tercolok atau nama port berubah "
                f"(cek: ls /dev/ttyACM*)")
    if "Permission denied" in text or "Errno 13" in text:
        return f"tidak ada izin membuka {port} -> sudo usermod -aG dialout $USER, lalu login ulang"
    if "busy" in text or "Errno 16" in text:
        return f"port {port} sedang dipakai program lain -> tutup program lain yang memakai motor"
    if "Errno 5" in text or "Input/output" in text or "disconnected" in text:
        return f"USB-RS485 di {port} terlepas saat dipakai -> cek kabel USB"
    return f"port {port}: {text}"


@dataclass
class MotorFeedback:
    mode: int
    current_a: float        # arus umpan balik, kerangka robot [A]
    speed_rpm: float        # rpm umpan balik, kerangka robot (tertinggal)
    position_raw: int       # 0..32767
    error_code: int


class DDSM115Motor:
    def __init__(self, port: str, motor_id: int, sign: float, name: str) -> None:
        self.port = port
        self.motor_id = int(motor_id)
        self.sign = 1.0 if sign >= 0 else -1.0
        self.encoder_sign = self.sign * (1.0 if config.ENCODER_DIRECTION >= 0 else -1.0)
        self.name = name
        self._ser: Optional[serial.Serial] = None
        self.feedback: Optional[MotorFeedback] = None
        self.command_a = 0.0            # arus perintah terakhir, kerangka robot [A]
        self.command_raw = 0            # nilai mentah yang dikirim ke motor (-32767..32767)

        # --- status untuk diagnosa ---
        self.ready = False              # True: port terbuka, motor membalas, mode current loop
        self.problem = "belum diinisialisasi"
        self.comm_errors = 0            # gagal berturut-turut
        self.count_ok = 0
        self.count_timeout = 0          # tidak ada / kurang byte balasan
        self.count_bad_frame = 0        # ID atau CRC salah
        self.last_error = ""
        self.last_reply_t = 0.0

        self._counts = 0                # posisi multi-putaran (hitungan mentah encoder)
        self._last_raw: Optional[int] = None
        self._last_t: Optional[float] = None
        self._dtheta = 0.0

    # --------------------------------------------------------------- serial --
    def open(self) -> None:
        if self._ser is not None and self._ser.is_open:
            return
        self._ser = serial.Serial(
            port=self.port, baudrate=config.MOTOR_BAUDRATE, bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
            timeout=config.MOTOR_TIMEOUT_S, write_timeout=config.MOTOR_TIMEOUT_S,
        )
        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()
        LOGGER.info("%s dibuka di %s", self.name, self.port)

    def close(self) -> None:
        try:
            if self._ser is not None and self._ser.is_open:
                self._ser.close()
        finally:
            self._ser = None
            self.ready = False

    def _write(self, frame: bytes) -> None:
        if self._ser is None:
            raise DDSM115Error(f"port {self.port} belum terbuka")
        self._ser.reset_input_buffer()
        self._ser.write(frame)
        self._ser.flush()

    # ------------------------------------------------------------- protokol --
    def set_mode(self, mode: int) -> None:
        self._write(bytes([self.motor_id, CMD_SWITCH_MODE, 0, 0, 0, 0, 0, 0, 0, mode & 0xFF]))
        time.sleep(0.05)

    def write_current(self, current_a: float) -> None:
        """Kirim perintah arus (kerangka robot). Balasan dibaca dengan read_feedback()."""
        limit = float(config.MAX_CURRENT_A)
        current_a = max(-limit, min(limit, float(current_a)))
        self.command_a = current_a
        self.command_raw = int(round(self.sign * current_a * RAW_PER_AMP))
        body = (bytes([self.motor_id, CMD_CONTROL]) + self.command_raw.to_bytes(2, "big", signed=True)
                + bytes([0, 0, ACCEL_TIME, 0, 0]))
        self._write(body + bytes([crc8_maxim(body)]))

    def read_feedback(self) -> MotorFeedback:
        if self._ser is None:
            raise DDSM115Error(f"port {self.port} belum terbuka")
        reply = self._ser.read(FRAME_LEN)
        now = time.monotonic()
        if len(reply) != FRAME_LEN:
            self.count_timeout += 1
            raise DDSM115Error(
                f"tidak ada balasan ({len(reply)}/{FRAME_LEN} byte) -> catu daya motor (12-24 V) mati, "
                f"kabel RS485 A/B lepas atau tertukar, atau ID motor bukan {self.motor_id}")
        if reply[0] != self.motor_id or crc8_maxim(reply[:9]) != reply[9]:
            self.count_bad_frame += 1
            self._ser.reset_input_buffer()
            raise DDSM115Error(
                f"balasan rusak (ID {reply[0]}, CRC salah) -> gangguan di kabel RS485 "
                f"atau baudrate bukan {config.MOTOR_BAUDRATE}")
        fb = MotorFeedback(
            mode=reply[1],
            current_a=self.sign * int.from_bytes(reply[2:4], "big", signed=True) / RAW_PER_AMP,
            speed_rpm=self.sign * int.from_bytes(reply[4:6], "big", signed=True),
            position_raw=int.from_bytes(reply[6:8], "big", signed=False),
            error_code=reply[8],
        )
        self._update_wheel_state(fb, now)
        self.feedback = fb
        self.count_ok += 1
        self.last_reply_t = now
        return fb

    def exchange(self, current_a: float) -> MotorFeedback:
        self.write_current(current_a)
        return self.read_feedback()

    # ------------------------------------------------------ theta dan dtheta --
    def _update_wheel_state(self, fb: MotorFeedback, now: float) -> None:
        half = config.MOTOR_POSITION_COUNTS // 2
        if self._last_raw is not None and self._last_t is not None:
            delta = (fb.position_raw - self._last_raw + half) % config.MOTOR_POSITION_COUNTS - half
            self._counts += delta
            dt = now - self._last_t
            if str(config.DTHETA_SOURCE).strip().lower() == "rpm":
                self._dtheta = fb.speed_rpm * RPM_TO_RAD_S
            elif dt > 1e-4:
                raw_rate = self.encoder_sign * delta * RAD_PER_COUNT / dt
                weight = dt / (float(config.DTHETA_FILTER_TAU_S) + dt)
                self._dtheta += weight * (raw_rate - self._dtheta)
        self._last_raw = fb.position_raw
        self._last_t = now

    @property
    def theta(self) -> float:
        """Sudut roda relatif terhadap badan, kerangka robot [rad]."""
        return self.encoder_sign * self._counts * RAD_PER_COUNT

    @property
    def dtheta(self) -> float:
        """Kecepatan sudut roda relatif terhadap badan, kerangka robot [rad/s]."""
        return self._dtheta

    def reset_wheel_angle(self) -> None:
        self._counts = 0

    # ------------------------------------------------------------ inisialisasi --
    def initialize(self) -> bool:
        """Buka port, pindah ke current loop, pastikan motor membalas dengan mode yang benar.

        Tidak melempar exception: hasilnya di self.ready dan self.problem.
        """
        self.ready = False
        try:
            self.close()
            self.open()
        except Exception as exc:
            self.problem = explain_port_error(exc, self.port)
            LOGGER.error("%s: %s", self.name, self.problem)
            return False
        self.problem = "motor tidak membalas"
        for _ in range(3):
            try:
                self.set_mode(MODE_CURRENT)
                fb = self.exchange(0.0)
            except DDSM115Error as exc:
                self.problem = str(exc)
                continue
            except Exception as exc:
                self.problem = explain_port_error(exc, self.port)
                continue
            if fb.mode == MODE_CURRENT:
                self.ready = True
                self.comm_errors = 0
                self.problem = ""
                LOGGER.info("%s siap (mode current loop)", self.name)
                return True
            self.problem = (f"motor membalas mode {fb.mode} ({MODE_NAMES.get(fb.mode, 'tidak dikenal')}), "
                            f"bukan current loop -> perintah pindah mode tidak diterima motor")
        LOGGER.error("%s: %s", self.name, self.problem)
        return False

    def status_text(self) -> str:
        if self.ready:
            fb = self.feedback
            err = decode_error(fb.error_code) if fb else "-"
            return "siap" if err == "-" else f"siap, tetapi motor melaporkan error: {err}"
        return self.problem


class DualDDSM115:
    """Dua roda. u = torsi TOTAL [N m]; tiap roda menerima u/2 (+/- selisih belok)."""

    def __init__(self) -> None:
        self.left = DDSM115Motor(config.LEFT_MOTOR_PORT, config.LEFT_MOTOR_ID,
                                 config.LEFT_MOTOR_SIGN, "motor kiri")
        self.right = DDSM115Motor(config.RIGHT_MOTOR_PORT, config.RIGHT_MOTOR_ID,
                                  config.RIGHT_MOTOR_SIGN, "motor kanan")
        self.motors = (self.left, self.right)
        self.u_applied = 0.0

    @property
    def ready(self) -> bool:
        return self.left.ready and self.right.ready

    def initialize(self) -> bool:
        """Inisialisasi motor yang belum siap. Kembalikan True bila keduanya siap."""
        for motor in self.motors:
            if not motor.ready:
                motor.initialize()
        return self.ready

    def problems(self) -> List[str]:
        return [f"{m.name}: {m.problem}" for m in self.motors if not m.ready]

    def command_currents(self, left_a: float, right_a: float, strict: bool = False) -> None:
        """Kirim arus ke tiap roda lalu baca umpan baliknya.

        strict=True (saat balancing): melempar DDSM115Error bila ada motor yang tidak siap atau
        gagal komunikasi berturut-turut. strict=False: motor bermasalah ditandai tidak siap.
        """
        if strict and not self.ready:
            raise DDSM115Error("; ".join(self.problems()))
        written = []
        for motor, current in zip(self.motors, (left_a, right_a)):
            if not motor.ready:
                written.append(False)
                continue
            try:
                motor.write_current(current)
                written.append(True)
            except Exception as exc:
                written.append(False)
                self._count_error(motor, exc)
        for motor, ok in zip(self.motors, written):
            if not ok:
                continue
            try:
                motor.read_feedback()
                motor.comm_errors = 0
            except Exception as exc:
                self._count_error(motor, exc)
        for motor in self.motors:
            if motor.comm_errors > int(config.MOTOR_MAX_COMM_ERRORS):
                motor.ready = False
                motor.problem = f"{motor.comm_errors} kali gagal berturut-turut: {motor.last_error}"
                LOGGER.error("%s: %s", motor.name, motor.problem)
                if strict:
                    raise DDSM115Error(f"{motor.name}: {motor.problem}")

    def command_torque(self, u: float, turn: float = 0.0, strict: bool = False) -> float:
        """Kirim u ke kedua roda. Kembalikan u yang benar-benar dikirim (setelah batas arus).

        turn [N m] adalah selisih torsi per roda: positif = belok kanan (roda kiri lebih maju).
        """
        half = 0.5 * float(u)
        self.command_currents((half + turn) / config.MOTOR_KT, (half - turn) / config.MOTOR_KT, strict)
        self.u_applied = config.MOTOR_KT * (self.left.command_a + self.right.command_a)
        return self.u_applied

    @staticmethod
    def _count_error(motor: DDSM115Motor, exc: Exception) -> None:
        motor.comm_errors += 1
        motor.last_error = str(exc) if isinstance(exc, DDSM115Error) else explain_port_error(exc, motor.port)

    def stop_all(self) -> None:
        """Arus nol ke kedua roda. Dicoba beberapa kali; tidak pernah melempar exception."""
        self.u_applied = 0.0
        for motor in self.motors:
            motor.command_a = 0.0
            if motor._ser is None:
                continue
            for _ in range(3):
                try:
                    motor.exchange(0.0)
                    break
                except Exception as exc:
                    motor.last_error = str(exc)

    @property
    def theta(self) -> float:
        return 0.5 * (self.left.theta + self.right.theta)

    @property
    def dtheta(self) -> float:
        return 0.5 * (self.left.dtheta + self.right.dtheta)

    def reset_wheel_angles(self) -> None:
        for motor in self.motors:
            motor.reset_wheel_angle()

    def close(self) -> None:
        for motor in self.motors:
            try:
                motor.close()
            except Exception as exc:
                LOGGER.error("Gagal menutup %s: %s", motor.name, exc)