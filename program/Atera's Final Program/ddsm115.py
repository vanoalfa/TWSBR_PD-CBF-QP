"""
Berdasarkan dokumentasi Waveshare DDSM115:
- Baudrate 115200, 8N1
- Panjang frame 10 byte
- CRC: CRC-8/MAXIM
- Mode: current(1), velocity(2), position(3)
"""

from __future__ import annotations

import logging
import struct
import time
from dataclasses import dataclass
from typing import Optional, Dict, Any

import serial

import config

LOGGER = logging.getLogger(__name__)

MODE_CURRENT = 0x01
MODE_SPEED = 0x02
MODE_POSITION = 0x03

CMD_CONTROL = 0x64
CMD_QUERY = 0x74
CMD_SWITCH_MODE = 0xA0

FRAME_LEN = 10
CRC8_INIT = 0x00
CRC8_POLY_REVERSED = 0x8C  # CRC-8/MAXIM reversed polynomial


class DDSM115Error(Exception):
    """Base exception untuk driver DDSM115."""


class CRCError(DDSM115Error):
    """CRC frame tidak valid."""


class SerialTimeoutError(DDSM115Error):
    """Timeout saat menunggu reply serial."""


@dataclass
class MotorFeedback:
    motor_id: int
    mode: int
    torque_raw: int
    torque_ampere: float
    speed_rpm: int
    position_raw: int
    position_deg: float
    error_code: int
    temperature_c: Optional[int] = None
    position_u8: Optional[int] = None
    source: str = "control"


class DDSM115Motor:
    def __init__(
        self,
        port: str,
        motor_id: int = 1,
        baudrate: int = config.MOTOR_BAUDRATE,
        timeout: float = config.MOTOR_TIMEOUT_S,
        sign: float = 1.0,
        control_mode: str = "current",
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
        self._current_mode_value: Optional[int] = None
        self.last_feedback: Optional[MotorFeedback] = None

    def open(self) -> None:
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
        LOGGER.info("%s opened on %s", self.name, self.port)

    def close(self) -> None:
        if self._ser and self._ser.is_open:
            try:
                self._ser.close()
            finally:
                LOGGER.info("%s closed", self.name)

    @property
    def is_open(self) -> bool:
        return bool(self._ser and self._ser.is_open)

    def ensure_open(self) -> None:
        if not self.is_open:
            self.open()

    @staticmethod
    def crc8_maxim(data: bytes) -> int:
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
        payload = bytes([
            b0 & 0xFF,
            b1 & 0xFF,
            b2 & 0xFF,
            b3 & 0xFF,
            b4 & 0xFF,
            b5 & 0xFF,
            b6 & 0xFF,
            b7 & 0xFF,
            b8 & 0xFF,
        ])
        crc = cls.crc8_maxim(payload)
        return payload + bytes([crc])

    @staticmethod
    def int16_to_hi_lo(value: int) -> tuple[int, int]:
        packed = struct.pack(">h", int(value))
        return packed[0], packed[1]

    @staticmethod
    def uint16_to_hi_lo(value: int) -> tuple[int, int]:
        packed = struct.pack(">H", int(value))
        return packed[0], packed[1]

    @staticmethod
    def hi_lo_to_int16(hi: int, lo: int) -> int:
        return struct.unpack(">h", bytes([hi & 0xFF, lo & 0xFF]))[0]

    @staticmethod
    def hi_lo_to_uint16(hi: int, lo: int) -> int:
        return struct.unpack(">H", bytes([hi & 0xFF, lo & 0xFF]))[0]

    def _write_and_read(self, frame: bytes, expect_reply: bool = True) -> Optional[bytes]:
        self.ensure_open()
        assert self._ser is not None
        self._ser.reset_input_buffer()
        self._ser.write(frame)
        self._ser.flush()
        if not expect_reply:
            return None
        reply = self._ser.read(FRAME_LEN)
        if len(reply) != FRAME_LEN:
            raise SerialTimeoutError(f"{self.name} timeout/read short frame: got {len(reply)} bytes")
        self.validate_reply(reply)
        return reply

    @classmethod
    def validate_reply(cls, frame: bytes) -> None:
        if len(frame) != FRAME_LEN:
            raise DDSM115Error(f"Invalid frame length: {len(frame)}")
        expected = cls.crc8_maxim(frame[:9])
        got = frame[9]
        if expected != got:
            raise CRCError(f"CRC mismatch: expected 0x{expected:02X}, got 0x{got:02X}")

    def _parse_control_feedback(self, frame: bytes) -> MotorFeedback:
        motor_id = frame[0]
        mode = frame[1]
        torque_raw = self.hi_lo_to_int16(frame[2], frame[3])
        speed_rpm = self.hi_lo_to_int16(frame[4], frame[5])
        position_raw = self.hi_lo_to_uint16(frame[6], frame[7])
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
            source="control",
        )
        self.last_feedback = fb
        return fb

    def _parse_query_feedback(self, frame: bytes) -> MotorFeedback:
        motor_id = frame[0]
        mode = frame[1]
        torque_raw = self.hi_lo_to_int16(frame[2], frame[3])
        speed_rpm = self.hi_lo_to_int16(frame[4], frame[5])
        temperature_c = frame[6]
        position_u8 = frame[7]
        error_code = frame[8]
        torque_ampere = (torque_raw / 32767.0) * 8.0
        position_deg = (position_u8 / 255.0) * 360.0
        fb = MotorFeedback(
            motor_id=motor_id,
            mode=mode,
            torque_raw=torque_raw,
            torque_ampere=torque_ampere,
            speed_rpm=speed_rpm,
            position_raw=position_u8,
            position_deg=position_deg,
            error_code=error_code,
            temperature_c=temperature_c,
            position_u8=position_u8,
            source="query",
        )
        self.last_feedback = fb
        return fb

    def set_mode(self, mode: str | int) -> None:
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
        frame = bytes([
            self.motor_id & 0xFF,
            CMD_SWITCH_MODE,
            0, 0, 0, 0, 0, 0, 0,
            mode_val & 0xFF,
        ])
        self._write_and_read(frame, expect_reply=False)
        self._current_mode_value = mode_val
        time.sleep(0.02)

    def query_status(self) -> MotorFeedback:
        frame = self.build_frame(self.motor_id, CMD_QUERY, 0, 0, 0, 0, 0, 0, 0)
        reply = self._write_and_read(frame, expect_reply=True)
        assert reply is not None
        return self._parse_query_feedback(reply)

    def send_raw_command(self, command_value: int, brake: bool = False) -> MotorFeedback:
        hi, lo = self.int16_to_hi_lo(command_value)
        frame = self.build_frame(
            self.motor_id,
            CMD_CONTROL,
            hi,
            lo,
            0,
            0,
            self.accel_time,
            0xFF if brake else 0x00,
            0,
        )
        reply = self._write_and_read(frame, expect_reply=True)
        assert reply is not None
        return self._parse_control_feedback(reply)

    def command_current_amp(self, current_amp: float) -> MotorFeedback:
        current_amp = max(-8.0, min(8.0, float(current_amp)))
        raw = int((current_amp / 8.0) * 32767.0)
        return self.send_raw_command(raw, brake=False)

    def command_speed_rpm(self, speed_rpm: float) -> MotorFeedback:
        speed_rpm = max(-330.0, min(330.0, float(speed_rpm)))
        raw = int(round(speed_rpm))
        return self.send_raw_command(raw, brake=False)

    def stop(self) -> MotorFeedback:
        if self.control_mode == "current":
            return self.command_current_amp(0.0)
        return self.command_speed_rpm(0.0)

    def command_normalized(self, normalized: float) -> MotorFeedback:
        normalized = max(-1.0, min(1.0, float(normalized)))
        normalized *= self.sign
        if self.control_mode == "current":
            return self.command_current_amp(normalized * config.MAX_CURRENT_A)
        if self.control_mode == "speed":
            return self.command_speed_rpm(normalized * config.MAX_SPEED_RPM)
        raise ValueError(f"Unsupported control_mode for balancing: {self.control_mode}")

    def initialize(self) -> None:
        self.ensure_open()
        self.set_mode(self.control_mode)
        self.stop()

    def decode_error_flags(self, error_code: int) -> Dict[str, bool]:
        code = int(error_code) & 0xFF
        return {
            "sensor_error": bool(code & 0x01),
            "overcurrent_error": bool(code & 0x02),
            "phase_overcurrent_error": bool(code & 0x04),
            "stall_error": bool(code & 0x08),
            "troubleshooting": bool(code & 0x10),
        }


class DualDDSM115:
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
        self.left.open()
        self.right.open()

    def initialize(self) -> None:
        self.left.initialize()
        self.right.initialize()

    def stop_all(self) -> None:
        errors = []
        for motor in (self.left, self.right):
            try:
                motor.stop()
            except Exception as exc:  # pragma: no cover
                errors.append(exc)
        if errors:
            raise DDSM115Error(f"Stop all encountered {len(errors)} error(s): {errors}")

    def command_normalized(self, left_value: float, right_value: float) -> Dict[str, MotorFeedback]:
        left_fb = self.left.command_normalized(left_value)
        right_fb = self.right.command_normalized(right_value)
        return {"left": left_fb, "right": right_fb}

    def query_both(self) -> Dict[str, MotorFeedback]:
        return {
            "left": self.left.query_status(),
            "right": self.right.query_status(),
        }

    def close(self) -> None:
        self.left.close()
        self.right.close()
