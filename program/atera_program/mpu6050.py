"""mpu6050.py - sensor IMU MPU6050 untuk ATERA: psi dan dpsi badan robot.

Perbedaan dari versi lama (apalah_bisa/mpu6050.py), cara hitungnya sama:
- 14 byte (accel, suhu, gyro) dibaca dalam SATU transaksi I2C, bukan 12 kali baca per byte.
  Lebih cepat, dan accel + gyro berasal dari sampel yang sama.
- Hasil utama dalam rad dan rad/s (state x), nilai derajat tetap tersedia untuk GUI.
- IMU_SIGN dikenakan ke sudut DAN ke kecepatan sudut.
- Pemeriksaan kesehatan untuk GUI: WHO_AM_I, data macet, akselerometer tidak wajar,
  gyro jenuh. Lihat problems().

Skala (datasheet MPU6050): gyro +-250 deg/s = 131 LSB per deg/s, accel +-2 g = 16384 LSB per g.
kalman.py bekerja dalam derajat, jadi di dalam file ini sudut tetap derajat.
"""

from __future__ import annotations

import math
import struct
import time
from dataclasses import dataclass
from typing import Dict, List, Optional

from smbus2 import SMBus

import config
from kalman import KalmanAngle

PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
INT_ENABLE = 0x38
ACCEL_XOUT_H = 0x3B
WHO_AM_I = 0x75

GYRO_LSB_PER_DEG_S = 131.0
ACCEL_LSB_PER_G = 16384.0
KNOWN_IDS = {0x68: "MPU6050", 0x70: "MPU6500", 0x71: "MPU9250", 0x72: "MPU6555", 0x98: "klon MPU6050"}


def explain_i2c_error(exc: Exception) -> str:
    """Ubah exception I2C menjadi penjelasan + saran."""
    text = str(exc)
    address = f"0x{config.MPU6050_ADDRESS:02X}"
    if "Errno 121" in text or "Remote I/O" in text:
        return (f"MPU6050 tidak menjawab di alamat {address} -> cek kabel SDA/SCL/VCC/GND; "
                f"bila pin AD0 terhubung ke 3.3 V alamatnya 0x69 (cek: i2cdetect -y {config.I2C_BUS})")
    if "Errno 2" in text or "No such file" in text:
        return (f"/dev/i2c-{config.I2C_BUS} tidak ada -> aktifkan I2C: sudo raspi-config, "
                f"Interface Options, I2C")
    if "Errno 13" in text or "Permission denied" in text:
        return "tidak ada izin memakai I2C -> sudo usermod -aG i2c $USER, lalu login ulang"
    if "Errno 110" in text or "timed out" in text or "Errno 5" in text or "Input/output" in text:
        return "bus I2C macet / timeout -> kabel longgar atau gangguan dari kabel motor"
    return f"I2C: {text}"


@dataclass
class ImuSample:
    timestamp: float
    dt: float
    psi: float              # sudut badan [rad], sudah dikurangi zero offset
    dpsi: float             # kecepatan sudut badan [rad/s]
    psi_deg: float
    dpsi_deg: float
    acc_angle_deg: float    # sudut dari akselerometer saja (sumbu terpakai)
    kalman_x_deg: float
    kalman_y_deg: float
    gyro_x_deg_s: float
    gyro_y_deg_s: float


class MPU6050:
    def __init__(self, bus_id: int = config.I2C_BUS, address: int = config.MPU6050_ADDRESS) -> None:
        self.bus_id = int(bus_id)
        self.address = int(address)
        self.bus: Optional[SMBus] = None
        self.kalman_x = KalmanAngle()
        self.kalman_y = KalmanAngle()
        self.gyro_bias_x = 0.0      # [deg/s]
        self.gyro_bias_y = 0.0
        self.zero_offset_deg = 0.0
        self.use_roll = str(config.IMU_AXIS).strip().lower() == "roll"
        self.sign = 1.0 if config.IMU_SIGN >= 0 else -1.0
        self._last_time: Optional[float] = None

        # --- status untuk diagnosa ---
        self.ready = False
        self.problem = "belum dibuka"
        self.who_am_i = -1
        self.read_count = 0
        self.error_count = 0
        self.acc_norm = 0.0             # |a| [g]; sekitar 1 saat diam
        self.gyro_saturated = False
        self._last_bytes = b""
        self._same_count = 0

    # ----------------------------------------------------------------- I2C --
    def open(self) -> bool:
        """Buka dan konfigurasi sensor. Tidak melempar exception: lihat ready / problem."""
        self.ready = False
        try:
            if self.bus is None:
                self.bus = SMBus(self.bus_id)
            write = self.bus.write_byte_data
            write(self.address, PWR_MGMT_1, 0x00)       # bangunkan sensor (keluar dari sleep)
            time.sleep(0.05)
            dlpf = int(config.MPU6050_DLPF_CFG) & 0x07
            # Laju sampel 1 kHz pada kedua kasus (gyro 8 kHz tanpa DLPF, 1 kHz dengan DLPF),
            # supaya tiap siklus kontrol mendapat sampel baru.
            write(self.address, SMPLRT_DIV, 0x07 if dlpf == 0 else 0x00)
            write(self.address, CONFIG, dlpf)
            write(self.address, GYRO_CONFIG, 0x00)      # +-250 deg/s
            write(self.address, ACCEL_CONFIG, 0x00)     # +-2 g
            write(self.address, INT_ENABLE, 0x01)
            time.sleep(0.05)
            self.who_am_i = self.bus.read_byte_data(self.address, WHO_AM_I)
            self._same_count = 0
            self._reset_filter()
        except Exception as exc:
            self.problem = explain_i2c_error(exc)
            self._close_bus()
            return False
        self.ready = True
        self.problem = ""
        return True

    def _close_bus(self) -> None:
        if self.bus is not None:
            try:
                self.bus.close()
            except Exception:
                pass
            self.bus = None

    def close(self) -> None:
        self._close_bus()
        self.ready = False

    def _read_raw(self) -> tuple:
        """(acc_angle_x, acc_angle_y) [deg] dan (gyro_x, gyro_y) [deg/s] dari satu sampel."""
        if self.bus is None:
            raise OSError("IMU belum dibuka")
        data = bytes(self.bus.read_i2c_block_data(self.address, ACCEL_XOUT_H, 14))
        self.read_count += 1
        self._same_count = self._same_count + 1 if data == self._last_bytes else 0
        self._last_bytes = data
        ax, ay, az, _temp, gx, gy, _gz = struct.unpack(">7h", data)
        self.gyro_saturated = max(abs(gx), abs(gy)) >= 32700
        ax, ay, az = ax / ACCEL_LSB_PER_G, ay / ACCEL_LSB_PER_G, az / ACCEL_LSB_PER_G
        self.acc_norm = math.sqrt(ax * ax + ay * ay + az * az)
        acc_angle_x = math.degrees(math.atan2(ay, math.sqrt(ax * ax + az * az)))
        acc_angle_y = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
        return acc_angle_x, acc_angle_y, gx / GYRO_LSB_PER_DEG_S, gy / GYRO_LSB_PER_DEG_S

    def _reset_filter(self) -> None:
        acc_x, acc_y, _, _ = self._read_raw()
        self.kalman_x.setAngle(acc_x)
        self.kalman_y.setAngle(acc_y)
        self._last_time = time.monotonic()

    # -------------------------------------------------------------- bacaan --
    def read(self) -> ImuSample:
        """Satu sampel. Melempar OSError bila I2C gagal (ready menjadi False)."""
        try:
            acc_x, acc_y, gyro_x, gyro_y = self._read_raw()
        except Exception as exc:
            self.error_count += 1
            self.ready = False
            self.problem = explain_i2c_error(exc)
            self._close_bus()
            raise OSError("IMU: " + self.problem) from exc
        now = time.monotonic()
        dt = max(1e-4, now - self._last_time) if self._last_time is not None else 1e-4
        self._last_time = now
        gyro_x -= self.gyro_bias_x
        gyro_y -= self.gyro_bias_y
        kalman_x = self.kalman_x.getAngle(acc_x, gyro_x, dt)
        kalman_y = self.kalman_y.getAngle(acc_y, gyro_y, dt)

        if self.use_roll:
            angle, rate, acc_angle = kalman_x, gyro_x, acc_x
        else:
            angle, rate, acc_angle = kalman_y, gyro_y, acc_y
        psi_deg = (angle - self.zero_offset_deg) * self.sign
        dpsi_deg = rate * self.sign
        return ImuSample(
            timestamp=now, dt=dt,
            psi=math.radians(psi_deg), dpsi=math.radians(dpsi_deg),
            psi_deg=psi_deg, dpsi_deg=dpsi_deg, acc_angle_deg=acc_angle,
            kalman_x_deg=kalman_x, kalman_y_deg=kalman_y,
            gyro_x_deg_s=gyro_x, gyro_y_deg_s=gyro_y,
        )

    # ------------------------------------------------------------ diagnosa --
    def problems(self) -> List[tuple]:
        """Daftar (tingkat, pesan). Tingkat: 'ERROR' = tidak boleh balancing, 'AWAS' = peringatan."""
        if not self.ready:
            return [("ERROR", self.problem)]
        found = []
        if self._same_count >= int(config.IMU_FROZEN_SAMPLES):
            found.append(("ERROR", f"data IMU tidak berubah selama {self._same_count} sampel -> sensor macet "
                                   f"atau kembali sleep (cek catu 3.3 V dan kabel)"))
        if self.read_count > 5 and not 0.5 <= self.acc_norm <= 1.5:
            found.append(("ERROR" if self.acc_norm < 0.05 else "AWAS",
                          f"akselerometer terbaca {self.acc_norm:.2f} g (seharusnya sekitar 1 g saat diam) "
                          f"-> sensor rusak/sleep, atau robot sedang terguncang"))
        if self.gyro_saturated:
            found.append(("AWAS", "gyro jenuh (melewati +-250 deg/s) -> dpsi tidak akurat saat ini"))
        if self.who_am_i not in KNOWN_IDS:
            found.append(("AWAS", f"WHO_AM_I = 0x{self.who_am_i & 0xFF:02X}, bukan keluarga MPU6050 "
                                  f"-> skala sensor mungkin berbeda"))
        return found

    @property
    def chip_name(self) -> str:
        return KNOWN_IDS.get(self.who_am_i, "tidak dikenal")

    # ----------------------------------------------------------- kalibrasi --
    def calibrate(self) -> Dict[str, float]:
        """Kalibrasi bias gyro lalu sudut nol. Robot harus DIAM dan TEGAK (titik seimbang).

        Memblokir sekitar 3 detik; hanya dipanggil saat motor mati. Melempar OSError bila I2C gagal.
        'gyro_std' pada hasil dipakai untuk menilai apakah robot benar-benar diam.
        """
        try:
            n = int(config.GYRO_CALIBRATION_SAMPLES)
            sum_x = sum_y = sum_sq = 0.0
            for _ in range(n):
                _, _, gyro_x, gyro_y = self._read_raw()
                sum_x += gyro_x
                sum_y += gyro_y
                used = gyro_x if self.use_roll else gyro_y
                sum_sq += used * used
                time.sleep(config.GYRO_CALIBRATION_DELAY_S)
            self.gyro_bias_x = sum_x / n
            self.gyro_bias_y = sum_y / n
            mean_used = self.gyro_bias_x if self.use_roll else self.gyro_bias_y
            gyro_std = math.sqrt(max(0.0, sum_sq / n - mean_used * mean_used))
            self._reset_filter()
        except Exception as exc:
            self.error_count += 1
            self.ready = False
            self.problem = explain_i2c_error(exc)
            self._close_bus()
            raise OSError("IMU: " + self.problem) from exc

        n = int(config.ZERO_CALIBRATION_SAMPLES)
        self.zero_offset_deg = 0.0
        total = 0.0
        for _ in range(n):
            sample = self.read()
            total += sample.kalman_x_deg if self.use_roll else sample.kalman_y_deg
            time.sleep(config.ZERO_CALIBRATION_DELAY_S)
        self.zero_offset_deg = total / n
        return {"gyro_bias_x": self.gyro_bias_x, "gyro_bias_y": self.gyro_bias_y,
                "zero_offset_deg": self.zero_offset_deg, "gyro_std": gyro_std}