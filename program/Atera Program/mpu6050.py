# mpu6050.py
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Dict, Tuple
from smbus2 import SMBus
from kalman import KalmanAngle
import config

# Alamat Register Hardware MPU6050
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H  = 0x43


@dataclass
class AngleState:
    """Dataclass pembungkus laporan telemetri sudut IMU MPU6050."""
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


class MPU6050Sensor:
    """Class Pengelola IMU MPU6050 berorientasi Objek untuk Self-Balancing Robot."""

    def __init__(self, bus_id: int = config.I2C_BUS, address: int = config.MPU6050_ADDRESS) -> None:
        self.bus_id = int(bus_id)
        self.address = int(address)
        self.bus: SMBus | None = None

        # Instansi Kalman Filter terpisah untuk Roll (X) dan Pitch (Y)
        self.kalman_x = KalmanAngle()
        self.kalman_y = KalmanAngle()

        # Parameter Kalibrasi
        self.gyro_bias_x: float = 0.0
        self.gyro_bias_y: float = 0.0
        self.zero_offset_deg: float = 0.0

        self._last_time: float | None = None
        self._latest_state: AngleState | None = None
        self._initialized: bool = False

    # --- KONEKSI & KONFIGURASI HARDWARE ---

    def open(self) -> None:
        """Membuka koneksi I2C dan menginisialisasi register sensor."""
        if self.bus is None:
            self.bus = SMBus(self.bus_id)
        self._configure_sensor()
        self._initialized = True

    def close(self) -> None:
        """Menutup koneksi I2C secara aman."""
        if self.bus is not None:
            self.bus.close()
            self.bus = None
        self._initialized = False

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
        """Pengaturan awal register MPU6050."""
        self._write_byte(PWR_MGMT_1, 0x00)     # Membangunkan dari mode Sleep
        time.sleep(0.05)
        self._write_byte(SMPLRT_DIV, 0x07)     # Sample rate divider
        self._write_byte(CONFIG, 0x00)         # Low Pass Filter internal
        self._write_byte(GYRO_CONFIG, 0x00)    # Scale Gyro: +/- 250 deg/s
        self._write_byte(ACCEL_CONFIG, 0x00)   # Scale Accel: +/- 2g
        self._write_byte(INT_ENABLE, 0x01)     # Aktifkan interupsi
        time.sleep(0.05)

        ax, ay = self._compute_acc_angles()
        self.kalman_x.set_angle(ax)
        self.kalman_y.set_angle(ay)
        self._last_time = time.monotonic()

    def _compute_acc_angles(self) -> Tuple[float, float]:
        """Menghitung sudut murni Accelerometer (Roll & Pitch) dalam derajat."""
        acc_x = self._read_word(ACCEL_XOUT_H) / 16384.0
        acc_y = self._read_word(ACCEL_XOUT_H + 2) / 16384.0
        acc_z = self._read_word(ACCEL_XOUT_H + 4) / 16384.0

        acc_angle_x = math.degrees(math.atan2(acc_y, math.sqrt(acc_x * acc_x + acc_z * acc_z)))
        acc_angle_y = math.degrees(math.atan2(-acc_x, math.sqrt(acc_y * acc_y + acc_z * acc_z)))
        return acc_angle_x, acc_angle_y

    # --- MEMBACA SUDAH IMU & FILTER ---

    def read_angles(self) -> AngleState:
        """Membaca sensor MPU6050, memperbarui Kalman Filter, dan mengembalikan AngleState."""
        if self.bus is None or not self._initialized:
            self.open()

        now = time.monotonic()
        if self._last_time is None:
            self._last_time = now
        dt = max(1e-4, now - self._last_time)
        self._last_time = now

        acc_angle_x, acc_angle_y = self._compute_acc_angles()

        gyro_rate_x = (self._read_word(GYRO_XOUT_H) / 131.0) - self.gyro_bias_x
        gyro_rate_y = (self._read_word(GYRO_XOUT_H + 2) / 131.0) - self.gyro_bias_y

        kalman_x = self.kalman_x.get_angle(acc_angle_x, gyro_rate_x, dt)
        kalman_y = self.kalman_y.get_angle(acc_angle_y, gyro_rate_y, dt)

        axis = config.IMU_AXIS.strip().lower()
        raw_kalman = kalman_x if axis == "roll" else kalman_y
        axis_name = "roll" if axis == "roll" else "pitch"

        # KoreksiOffset & IMU Sign
        angle_deg = (raw_kalman - self.zero_offset_deg) * float(config.IMU_SIGN)

        self._latest_state = AngleState(
            timestamp=now,
            dt=dt,
            acc_angle_x=acc_angle_x,
            acc_angle_y=acc_angle_y,
            gyro_rate_x=gyro_rate_x,
            gyro_rate_y=gyro_rate_y,
            kalman_x=kalman_x,
            kalman_y=kalman_y,
            angle_deg=angle_deg,
            axis_used=axis_name,
        )
        return self._latest_state

    # --- FUNGSI KHUSUS PD CONTROL & CBF-QP ---

    def Angle_PSI(self) -> float:
        """Fungsi utama pengambil sudut pendulum (psi) dalam derajat.

        Digunakan oleh PD Control dan CBF-QP Safety Filter.
        """
        state = self.read_angles()
        return state.angle_deg

    def Angular_Rate_PSI(self) -> float:
        """Fungsi pengambil kecepatan sudut pendulum (dot_psi) dalam deg/s.

        Digunakan oleh PD Control dan CBF-QP.
        """
        state = self._latest_state if self._latest_state is not None else self.read_angles()
        return state.gyro_rate_x if state.axis_used == "roll" else state.gyro_rate_y

    # --- FUNGSI KALIBRASI DARI MAIN ---

    def calibrate_gyro_bias(self, samples: int = config.GYRO_CALIBRATION_SAMPLES, delay_s: float = config.GYRO_CALIBRATION_DELAY_S) -> Dict[str, float]:
        """Kalibrasi bias Gyroscope saat robot posisi diam sempurna."""
        if self.bus is None or not self._initialized:
            self.open()

        sum_x, sum_y = 0.0, 0.0
        for _ in range(int(samples)):
            sum_x += self._read_word(GYRO_XOUT_H) / 131.0
            sum_y += self._read_word(GYRO_XOUT_H + 2) / 131.0
            time.sleep(delay_s)

        self.gyro_bias_x = sum_x / float(samples)
        self.gyro_bias_y = sum_y / float(samples)

        ax, ay = self._compute_acc_angles()
        self.kalman_x.set_angle(ax)
        self.kalman_y.set_angle(ay)
        self._last_time = time.monotonic()

        return {"gyro_bias_x": self.gyro_bias_x, "gyro_bias_y": self.gyro_bias_y}

    def calibrate_zero_angle(self, samples: int = config.ZERO_CALIBRATION_SAMPLES, delay_s: float = config.ZERO_CALIBRATION_DELAY_S) -> float:
        """Kalibrasi offset titik nol mekanis saat robot diposisikan tegak lurus seimbang."""
        if self.bus is None or not self._initialized:
            self.open()

        total = 0.0
        for _ in range(int(samples)):
            state = self.read_angles()
            raw_angle = state.kalman_x if state.axis_used == "roll" else state.kalman_y
            total += raw_angle
            time.sleep(delay_s)

        self.zero_offset_deg = total / float(samples)
        return self.zero_offset_deg

    def run_full_calibration(self) -> Dict[str, float]:
        """Fungsi 'Satu Pintu' untuk mengeksekusi kalibrasi gyro bias & zero angle dari main script."""
        gyro_res = self.calibrate_gyro_bias()
        zero_res = self.calibrate_zero_angle()
        return {
            "gyro_bias_x": gyro_res["gyro_bias_x"],
            "gyro_bias_y": gyro_res["gyro_bias_y"],
            "zero_offset_deg": zero_res,
        }

    # --- FUNGSI DEBUG ---

    def get_debug_snapshot(self) -> Dict[str, float]:
        """Mengambil ringkasan telemetri sudut & filter untuk monitoring UI / terminal."""
        state = self.read_angles()
        return {
            "angle_psi": state.angle_deg,
            "angular_rate_psi": self.Angular_Rate_PSI(),
            "dt": state.dt,
            "kalman_x": state.kalman_x,
            "kalman_y": state.kalman_y,
            "zero_offset_deg": self.zero_offset_deg,
        }