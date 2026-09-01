"""Pembacaan MPU6050 dengan KalmanAngle.

Modul ini diasumsikan berjalan di Raspberry Pi dengan I2C aktif.
Pemrosesan sudut memakai Kalman filter dari modul eksternal:
    from kalman import KalmanAngle
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Dict

from smbus2 import SMBus

from kalman import KalmanAngle
import tunning

PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
INT_ENABLE = 0x38
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43


@dataclass
class AngleState:
    timestamp: float
    dt: float
    acc_angle_x: float
    acc_angle_y: float
    gyro_rate_x: float
    gyro_rate_y: float
    kalman_x: float
    kalman_y: float
    angle_deg: float
    axis_used: str


class MPU6050Reader:
    def __init__(self, bus_id: int = tunning.I2C_BUS, address: int = tunning.MPU6050_ADDRESS) -> None:
        self.bus_id = int(bus_id)
        self.address = int(address)
        self.bus: SMBus | None = None
        self.kalman_x = KalmanAngle()
        self.kalman_y = KalmanAngle()
        self.gyro_bias_x = 0.0
        self.gyro_bias_y = 0.0
        self.zero_offset_deg = 0.0
        self._last_time: float | None = None
        self._initialized = False

    def open(self) -> None:
        if self.bus is None:
            self.bus = SMBus(self.bus_id)
        self._configure_sensor()
        self._initialized = True

    def close(self) -> None:
        if self.bus is not None:
            self.bus.close()
            self.bus = None
        self._initialized = False

    def ensure_open(self) -> None:
        if self.bus is None or not self._initialized:
            self.open()

    def _write_byte(self, reg: int, value: int) -> None:
        assert self.bus is not None
        self.bus.write_byte_data(self.address, reg, value)

    def _read_byte(self, reg: int) -> int:
        assert self.bus is not None
        return self.bus.read_byte_data(self.address, reg)

    def _read_word(self, reg: int) -> int:
        high = self._read_byte(reg)
        low = self._read_byte(reg + 1)
        value = (high << 8) | low
        if value >= 0x8000:
            value = -((65535 - value) + 1)
        return value

    def _configure_sensor(self) -> None:
        self.ensure_bus_only()
        self._write_byte(PWR_MGMT_1, 0x00)
        time.sleep(0.05)
        self._write_byte(SMPLRT_DIV, 0x07)
        self._write_byte(CONFIG, 0x00)
        self._write_byte(GYRO_CONFIG, 0x00)
        self._write_byte(ACCEL_CONFIG, 0x00)
        self._write_byte(INT_ENABLE, 0x01)
        time.sleep(0.05)
        ax, ay = self._compute_acc_angles()
        self.kalman_x.setAngle(ax)
        self.kalman_y.setAngle(ay)
        self._last_time = time.monotonic()

    def ensure_bus_only(self) -> None:
        if self.bus is None:
            self.bus = SMBus(self.bus_id)

    def calibrate_gyro_bias(self, samples: int = tunning.GYRO_CALIBRATION_SAMPLES, delay_s: float = tunning.GYRO_CALIBRATION_DELAY_S) -> Dict[str, float]:
        self.ensure_open()
        sum_x = 0.0
        sum_y = 0.0
        for _ in range(int(samples)):
            gx = self._read_word(GYRO_XOUT_H) / 131.0
            gy = self._read_word(GYRO_XOUT_H + 2) / 131.0
            sum_x += gx
            sum_y += gy
            time.sleep(delay_s)
        self.gyro_bias_x = sum_x / float(samples)
        self.gyro_bias_y = sum_y / float(samples)
        ax, ay = self._compute_acc_angles()
        self.kalman_x.setAngle(ax)
        self.kalman_y.setAngle(ay)
        self._last_time = time.monotonic()
        return {"gyro_bias_x": self.gyro_bias_x, "gyro_bias_y": self.gyro_bias_y}

    def _compute_acc_angles(self) -> tuple[float, float]:
        acc_x = self._read_word(ACCEL_XOUT_H) / 16384.0
        acc_y = self._read_word(ACCEL_XOUT_H + 2) / 16384.0
        acc_z = self._read_word(ACCEL_XOUT_H + 4) / 16384.0
        acc_angle_x = math.degrees(math.atan2(acc_y, math.sqrt(acc_x * acc_x + acc_z * acc_z)))
        acc_angle_y = math.degrees(math.atan2(-acc_x, math.sqrt(acc_y * acc_y + acc_z * acc_z)))
        return acc_angle_x, acc_angle_y

    def read_angles(self) -> AngleState:
        self.ensure_open()
        now = time.monotonic()
        if self._last_time is None:
            self._last_time = now
        dt = max(1e-4, now - self._last_time)
        self._last_time = now

        acc_angle_x, acc_angle_y = self._compute_acc_angles()
        gyro_rate_x = (self._read_word(GYRO_XOUT_H) / 131.0) - self.gyro_bias_x
        gyro_rate_y = (self._read_word(GYRO_XOUT_H + 2) / 131.0) - self.gyro_bias_y

        kalman_x = self.kalman_x.getAngle(acc_angle_x, gyro_rate_x, dt)
        kalman_y = self.kalman_y.getAngle(acc_angle_y, gyro_rate_y, dt)

        axis = tunning.IMU_AXIS.strip().lower()
        if axis == "roll":
            angle_deg = kalman_x
        else:
            angle_deg = kalman_y
            axis = "pitch"

        angle_deg = (angle_deg - self.zero_offset_deg) * float(tunning.IMU_SIGN)

        return AngleState(
            timestamp=now,
            dt=dt,
            acc_angle_x=acc_angle_x,
            acc_angle_y=acc_angle_y,
            gyro_rate_x=gyro_rate_x,
            gyro_rate_y=gyro_rate_y,
            kalman_x=kalman_x,
            kalman_y=kalman_y,
            angle_deg=angle_deg,
            axis_used=axis,
        )

    def calibrate_zero_angle(self, samples: int = tunning.ZERO_CALIBRATION_SAMPLES, delay_s: float = tunning.ZERO_CALIBRATION_DELAY_S) -> float:
        self.ensure_open()
        total = 0.0
        for _ in range(int(samples)):
            state = self.read_angles()
            raw_angle = state.kalman_x if state.axis_used == "roll" else state.kalman_y
            total += raw_angle
            time.sleep(delay_s)
        self.zero_offset_deg = total / float(samples)
        return self.zero_offset_deg

    def get_debug_snapshot(self) -> Dict[str, float]:
        state = self.read_angles()
        return {
            "angle_deg": state.angle_deg,
            "dt": state.dt,
            "gyro_rate_x": state.gyro_rate_x,
            "gyro_rate_y": state.gyro_rate_y,
            "kalman_x": state.kalman_x,
            "kalman_y": state.kalman_y,
            "zero_offset_deg": self.zero_offset_deg,
        }