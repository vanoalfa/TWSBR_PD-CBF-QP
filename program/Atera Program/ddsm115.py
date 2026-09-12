from __future__ import annotations

import logging
import math
import struct
import time
from dataclasses import dataclass
from typing import Dict, Optional

import serial

import config

LOGGER = logging.getLogger(__name__)

# --- KONFIGURASI PROTOKOL DDSM115 WAVESHARE ---
MODE_CURRENT = 0x01   # Mode Arus/Torsi Listrik
MODE_SPEED = 0x02     # Mode Kecepatan (RPM)
MODE_POSITION = 0x03  # Mode Posisi Sudut

CMD_CONTROL = 0x64      # Perintah Kendali Utama
CMD_QUERY = 0x74        # Perintah Query Telemetri/Suhu
CMD_SWITCH_MODE = 0xA0  # Perintah Ganti Mode Operasi

FRAME_LEN = 10           # Panjang Frame RS485 (10 Byte)
CRC8_INIT = 0x00         # Nilai Awal CRC-8
CRC8_POLY_REVERSED = 0x8C # Polinomial Terbalik CRC-8/MAXIM


class DDSM115Error(Exception):
    """Base exception khusus driver DDSM115."""
    pass


class CRCError(DDSM115Error):
    """Exception jika frame data mengalami kesalahan CRC."""
    pass


class SerialTimeoutError(DDSM115Error):
    """Exception jika komunikasi serial mengalami batas waktu (timeout)."""
    pass


@dataclass
class MotorFeedback:
    """Struktur data laporan umpan balik (telemetri) dari motor DDSM115."""
    motor_id: int
    mode: int
    torque_raw: int
    torque_ampere: float
    speed_rpm: int
    position_raw: int
    position_deg: float
    error_code: int
    temperature_c: Optional[int] = None
    source: str = "control"


@dataclass
class Angle_THETA:
    """
    Struktur data posisi sudut (Theta) dan kecepatan sudut (Dot Theta) roda 
    dari encoder DDSM115 untuk diolah oleh PD Control dan CBF-QP.
    """
    left_theta_deg: float
    right_theta_deg: float
    avg_theta_deg: float
    left_speed_rpm: float
    right_speed_rpm: float
    avg_speed_rpm: float
    left_theta_rad: float
    right_theta_rad: float
    avg_theta_rad: float
    avg_speed_rad_s: float


class DDSM115Motor:
    """Kelas driver tingkat rendah (Low-Level OOP) untuk kontrol 1 unit motor DDSM115."""

    def __init__(
        self,
        port: str,
        motor_id: int = 1,
        baudrate: int = config.MOTOR_BAUDRATE,
        timeout: float = config.MOTOR_TIMEOUT_S,
        sign: float = 1.0,
        control_mode: str = config.MOTOR_CONTROL_MODE,
        accel_time: int = config.MOTOR_ACCEL_TIME,
        name: str = "motor",
    ) -> None:
        self.port = port
        self.motor_id = int(motor_id)
        self.baudrate = int(baudrate)
        self.timeout = float(timeout)
        self.sign = 1.0 if sign >= 0 else -1.0
        self.control_mode = control_mode.strip().lower()
        self.accel_time = max(0, min(255, int(accel_time)))
        self.name = name
        self._ser: Optional[serial.Serial] = None
        self.last_feedback: Optional[MotorFeedback] = None

    def open(self) -> None:
        """Membuka jalur serial RS485."""
        if self._ser and self._ser.is_open:
            return
        self._ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.timeout,
            write_timeout=self.timeout,
        )
        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()
        LOGGER.info(f"{self.name} terbuka pada port {self.port}")

    def close(self) -> None:
        """Menutup jalur serial RS485."""
        if self._ser and self._ser.is_open:
            try:
                self._ser.close()
            finally:
                LOGGER.info(f"{self.name} ditutup")

    @property
    def is_open(self) -> bool:
        return bool(self._ser and self._ser.is_open)

    def ensure_open(self) -> None:
        if not self.is_open:
            self.open()

    @staticmethod
    def crc8_maxim(data: bytes) -> int:
        """Kalkulasi stempel integritas CRC-8/MAXIM."""
        crc = CRC8_INIT
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x01:
                    crc = (crc >> 1) ^ CRC8_POLY_REVERSED
                else:
                    crc >>= 1
        return crc & 0xFF

    @classmethod
    def build_frame(cls, b0: int, b1: int, b2: int, b3: int, b4: int, b5: int, b6: int, b7: int, b8: int) -> bytes:
        """Menyusun 9 byte payload + 1 byte stempel CRC menjadi 10 byte frame utuh."""
        payload = bytes([b0 & 0xFF, b1 & 0xFF, b2 & 0xFF, b3 & 0xFF, b4 & 0xFF, b5 & 0xFF, b6 & 0xFF, b7 & 0xFF, b8 & 0xFF])
        crc = cls.crc8_maxim(payload)
        return payload + bytes([crc])

    def _write_and_read(self, frame: bytes, expect_reply: bool = True) -> Optional[bytes]:
        """Kirim frame instruksi dan baca balasan 10 byte dari motor."""
        self.ensure_open()
        assert self._ser is not None
        self._ser.reset_input_buffer()
        self._ser.write(frame)
        self._ser.flush()
        if not expect_reply:
            return None
        reply = self._ser.read(FRAME_LEN)
        if len(reply) != FRAME_LEN:
            raise SerialTimeoutError(f"{self.name} timeout/frame pendek: diterima {len(reply)} bytes")
        self.validate_reply(reply)
        return reply

    @classmethod
    def validate_reply(cls, frame: bytes) -> None:
        """Validasi integritas data balasan."""
        if len(frame) != FRAME_LEN:
            raise DDSM115Error(f"Panjang frame tidak valid: {len(frame)}")
        expected = cls.crc8_maxim(frame[:9])
        if expected != frame[9]:
            raise CRCError(f"CRC Mismatch: expected 0x{expected:02X}, got 0x{frame[9]:02X}")

    def _parse_control_feedback(self, frame: bytes) -> MotorFeedback:
        """Menerjemahkan byte telemetri kendali motor."""
        motor_id = frame[0]
        mode = frame[1]
        torque_raw = struct.unpack(">h", bytes([frame[2], frame[3]]))[0]
        speed_rpm = struct.unpack(">h", bytes([frame[4], frame[5]]))[0]
        position_raw = struct.unpack(">H", bytes([frame[6], frame[7]]))[0]
        error_code = frame[8]

        torque_ampere = (torque_raw / 32767.0) * 8.0
        position_deg = (position_raw / 32767.0) * 360.0 if position_raw <= 32767 else 0.0

        fb = MotorFeedback(
            motor_id=motor_id,
            mode=mode,
            torque_raw=torque_raw,
            torque_ampere=torque_ampere,
            speed_rpm=speed_rpm,
            position_raw=position_raw,
            position_deg=position_deg,
            error_code=error_code,
            source="control"
        )
        self.last_feedback = fb
        return fb

    def set_mode(self, mode: str | int) -> None:
        """Mengatur mode operasi motor (Current / Speed / Position)."""
        if isinstance(mode, str):
            mode_key = mode.strip().lower()
            if mode_key == "current":
                mode_val = MODE_CURRENT
            elif mode_key == "speed":
                mode_val = MODE_SPEED
            elif mode_key == "position":
                mode_val = MODE_POSITION
            else:
                raise ValueError(f"Unknown mode: {mode}")
        else:
            mode_val = int(mode)

        frame = bytes([self.motor_id & 0xFF, CMD_SWITCH_MODE, 0, 0, 0, 0, 0, 0, 0, mode_val & 0xFF])
        self._write_and_read(frame, expect_reply=False)
        time.sleep(0.02)

    def send_raw_command(self, command_value: int, brake: bool = False) -> MotorFeedback:
        """Mengirim nilai int16 mentah ke motor."""
        packed = struct.pack(">h", int(command_value))
        hi, lo = packed[0], packed[1]
        frame = self.build_frame(
            self.motor_id, CMD_CONTROL, hi, lo, 0, 0, self.accel_time, 0xFF if brake else 0x00, 0
        )
        reply = self._write_and_read(frame, expect_reply=True)
        assert reply is not None
        return self._parse_control_feedback(reply)

    def command_normalized(self, normalized: float) -> MotorFeedback:
        """Mengirim sinyal ter-normalisasi (-1.0 s.d +1.0)."""
        normalized = max(-1.0, min(1.0, float(normalized))) * self.sign
        if self.control_mode == "current":
            raw = int((normalized * config.MAX_CURRENT_A / 8.0) * 32767.0)
            return self.send_raw_command(raw)
        elif self.control_mode == "speed":
            raw = int(round(normalized * config.MAX_SPEED_RPM))
            return self.send_raw_command(raw)
        else:
            raise ValueError(f"Mode kontrol tidak didukung: {self.control_mode}")

    def stop(self) -> MotorFeedback:
        """Menghentikan perputaran motor."""
        return self.command_normalized(0.0)

    def initialize(self) -> None:
        """Inisialisasi awal motor."""
        self.ensure_open()
        self.set_mode(self.control_mode)
        self.stop()


class DDSM115Dual:
    """
    Kelas Abstraksi Tingkat Tinggi (High-Level OOP) untuk mengendalikan kedua motor DDSM115
    secara bersamaan dengan dukungan Manuver Penuh (Maju, Mundur, Kiri, Kanan).
    """

    def __init__(self) -> None:
        self.left = DDSM115Motor(
            port=config.LEFT_MOTOR_PORT,
            motor_id=config.LEFT_MOTOR_ID,
            baudrate=config.MOTOR_BAUDRATE,
            timeout=config.MOTOR_TIMEOUT_S,
            sign=config.LEFT_MOTOR_SIGN,
            control_mode=config.MOTOR_CONTROL_MODE,
            accel_time=config.MOTOR_ACCEL_TIME,
            name="left_motor",
        )
        self.right = DDSM115Motor(
            port=config.RIGHT_MOTOR_PORT,
            motor_id=config.RIGHT_MOTOR_ID,
            baudrate=config.MOTOR_BAUDRATE,
            timeout=config.MOTOR_TIMEOUT_S,
            sign=config.RIGHT_MOTOR_SIGN,
            control_mode=config.MOTOR_CONTROL_MODE,
            accel_time=config.MOTOR_ACCEL_TIME,
            name="right_motor",
        )

    def open(self) -> None:
        """Membuka koneksi serial kedua motor."""
        self.left.open()
        self.right.open()

    def initialize(self) -> None:
        """Menjalankan prosedur inisialisasi awal kedua motor."""
        self.left.initialize()
        self.right.initialize()

    def drive_individual(self, left_value: float, right_value: float) -> Dict[str, MotorFeedback]:
        """Menggerakkan masing-masing roda secara mandiri dengan input ter-normalisasi [-1.0, 1.0]."""
        fb_left = self.left.command_normalized(left_value)
        fb_right = self.right.command_normalized(right_value)
        return {"left": fb_left, "right": fb_right}

    def drive_maneuver(self, throttle: float, turn: float = 0.0) -> Dict[str, MotorFeedback]:
        """
        Menggerakkan robot secara full (Maju, Mundur, Kiri, Kanan)
        menggunakan pencampuran kemudi diferensial (Differential Drive Mixer).
        """
        turn_frac = config.TURN_OUTPUT_FRACTION * turn
        left_out = max(-1.0, min(1.0, throttle + turn_frac))
        right_out = max(-1.0, min(1.0, throttle - turn_frac))
        return self.drive_individual(left_out, right_out)

    def get_angle_theta(self) -> Angle_THETA:
        """
        Membaca data encoder roda (Theta) dan kecepatan (Dot Theta) dari kedua motor,
        kemudian menghitung nilai rata-rata yang siap dipakai oleh PD Control dan CBF-QP.
        """
        fb_l = self.left.last_feedback
        fb_r = self.right.last_feedback

        theta_l_deg = fb_l.position_deg if fb_l else 0.0
        theta_r_deg = fb_r.position_deg if fb_r else 0.0
        avg_theta_deg = (theta_l_deg + theta_r_deg) / 2.0

        speed_l_rpm = float(fb_l.speed_rpm) if fb_l else 0.0
        speed_r_rpm = float(fb_r.speed_rpm) if fb_r else 0.0
        avg_speed_rpm = (speed_l_rpm + speed_r_rpm) / 2.0

        theta_l_rad = math.radians(theta_l_deg)
        theta_r_rad = math.radians(theta_r_deg)
        avg_theta_rad = math.radians(avg_theta_deg)
        avg_speed_rad_s = avg_speed_rpm * (2.0 * math.pi / 60.0)

        return Angle_THETA(
            left_theta_deg=theta_l_deg,
            right_theta_deg=theta_r_deg,
            avg_theta_deg=avg_theta_deg,
            left_speed_rpm=speed_l_rpm,
            right_speed_rpm=speed_r_rpm,
            avg_speed_rpm=avg_speed_rpm,
            left_theta_rad=theta_l_rad,
            right_theta_rad=theta_r_rad,
            avg_theta_rad=avg_theta_rad,
            avg_speed_rad_s=avg_speed_rad_s,
        )

    def stop_all(self) -> None:
        """Menghentikan putaran kedua roda secara bersamaan."""
        errors = []
        for motor in (self.left, self.right):
            try:
                motor.stop()
            except Exception as exc:
                errors.append(exc)
        if errors:
            raise DDSM115Error(f"Gagal menghentikan motor: {errors}")

    def close(self) -> None:
        """Menutup port serial kedua motor."""
        self.left.close()
        self.right.close()


class DDSM115Balance(DDSM115Dual):
    """
    Kelas khusus Mode Penyeimbang (Self-Balancing Mode).
    Roda kiri dan kanan selalu bergerak serentak dengan daya yang persis sama (output_left == output_right).
    Hanya mengeksekusi gerakan Maju dan Mundur tanpa manuver belok.
    """

    def drive_together(self, output_value: float) -> Dict[str, MotorFeedback]:
        """Menggerakkan kedua roda secara serentak dengan nilai keluaran yang sama persis."""
        return self.drive_individual(output_value, output_value)

    def move_forward(self, speed_fraction: float = 0.5) -> Dict[str, MotorFeedback]:
        """Fungsi pembantu menggerakkan robot maju lurus secara serentak."""
        val = abs(speed_fraction)
        return self.drive_together(val)

    def move_backward(self, speed_fraction: float = 0.5) -> Dict[str, MotorFeedback]:
        """Fungsi pembantu menggerakkan robot mundur lurus secara serentak."""
        val = -abs(speed_fraction)
        return self.drive_together(val)