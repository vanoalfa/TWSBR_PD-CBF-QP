# Menunda evaluasi type hints agar penulisan tipe data modern (seperti |) disimpan sebagai string lebih dahulu[cite: 1]
from __future__ import annotations

# Mengimpor modul matematika bawaan Python untuk kalkulasi trigonometri (atan2, sqrt, degrees)[cite: 1]
import math
# Mengimpor modul time untuk fungsi jeda (sleep) dan pencatatan waktu presisi (monotonic)[cite: 1]
import time
# Mengimpor pembuat struktur pembungkus data otomatis (@dataclass)[cite: 1]
from dataclasses import dataclass
# Mengimpor penanda tipe data Dictionary dari modul typing[cite: 1]
from typing import Dict

# Mengimpor kelas SMBus dari smbus2 untuk komunikasi I2C pada Raspberry Pi[cite: 1]
from smbus2 import SMBus

# Mengimpor kelas estimator sudut Kalman Filter dari modul eksternal kalman.py[cite: 1]
from kalman import KalmanAngle
# Mengimpor berkas konfigurasi parameter global sistem[cite: 1]
import config

# --- KONFIGURASI ALAMAT REGISTER MPU6050 ---
PWR_MGMT_1 = 0x6B    # Register manajemen daya (Power Management 1)[cite: 1]
SMPLRT_DIV = 0x19    # Register pembagi laju sampel (Sample Rate Divider)[cite: 1]
CONFIG = 0x1A        # Register konfigurasi filter internal (DLPF)[cite: 1]
GYRO_CONFIG = 0x1B   # Register konfigurasi skala penuh Gyroscope[cite: 1]
ACCEL_CONFIG = 0x1C  # Register konfigurasi skala penuh Accelerometer[cite: 1]
INT_ENABLE = 0x38    # Register pengaktifan interupsi data[cite: 1]
ACCEL_XOUT_H = 0x3B  # Alamat byte tinggi awal data Accelerometer sumbu X[cite: 1]
GYRO_XOUT_H = 0x43   # Alamat byte tinggi awal data Gyroscope sumbu X[cite: 1]


# Struktur pembungkus data laporan telemetri sudut hasil pengukuran[cite: 1]
@dataclass
class AngleState:
    timestamp: float    # Catatan waktu persis saat data dibaca (detik)[cite: 1]
    dt: float           # Selisih waktu (Delta t) dibanding pembacaan sebelumnya[cite: 1]
    acc_angle_x: float  # Sudut sumbu X murni dari perhitungan Accelerometer (derajat)[cite: 1]
    acc_angle_y: float  # Sudut sumbu Y murni dari perhitungan Accelerometer (derajat)[cite: 1]
    gyro_rate_x: float  # Kecepatan putar sumbu X murni dari Gyroscope (derajat/detik)[cite: 1]
    gyro_rate_y: float  # Kecepatan putar sumbu Y murni dari Gyroscope (derajat/detik)[cite: 1]
    kalman_x: float     # Hasil estimasi sudut sumbu X dari Kalman Filter (derajat)[cite: 1]
    kalman_y: float     # Hasil estimasi sudut sumbu Y dari Kalman Filter (derajat)[cite: 1]
    angle_deg: float    # Sudut akhir terkalibrasi yang dipakai untuk balancing (derajat)[cite: 1]
    axis_used: str      # Sumbu acuan yang sedang aktif digunakan ("roll" atau "pitch")[cite: 1]


class MPU6050Reader:

    # 📖 Cerita Fungsi:
    # Sebelum robot beroperasi, manajer MPU6050 menyiapkan semua perlengkapan dasar.
    # Dia mencatat nomor jalur komunikasi I2C, menyiapkan dua filter kacamata (Kalman Filter X dan Y),
    # serta menyiapkan lembar catatan kosong untuk menyimpan angka bias gyro dan offset titik nol[cite: 1].
    def __init__(self, bus_id: int = config.I2C_BUS, address: int = config.MPU6050_ADDRESS) -> None:
        self.bus_id = int(bus_id)                       # Menyimpan ID bus I2C (misal: 1)[cite: 1]
        self.address = int(address)                     # Menyimpan alamat I2C MPU6050 (misal: 0x68)[cite: 1]
        self.bus: SMBus | None = None                   # Mempersiapkan variabel koneksi bus I2C (awal: Kosong)[cite: 1]
        self.kalman_x = KalmanAngle()                   # Membuat objek Kalman Filter khusus sumbu X[cite: 1]
        self.kalman_y = KalmanAngle()                   # Membuat objek Kalman Filter khusus sumbu Y[cite: 1]
        self.gyro_bias_x = 0.0                          # Mempersiapkan penampung nilai bias gyro sumbu X[cite: 1]
        self.gyro_bias_y = 0.0                          # Mempersiapkan penampung nilai bias gyro sumbu Y[cite: 1]
        self.zero_offset_deg = 0.0                      # Mempersiapkan penampung offset titik nol lurus mekanis[cite: 1]
        self._last_time: float | None = None            # Mempersiapkan catatan timestamp pembacaan terakhir[cite: 1]
        self._initialized = False                       # Penanda sakelar status apakah sensor sudah dikonfigurasi[cite: 1]

    # 📖 Cerita Fungsi:
    # Manajer membuka pintu saluran I2C agar kabel komunikasi terhubung ke sensor,
    # lalu memanggil prosedur penyalaan sensor MPU6050 agar siap dipakai[cite: 1].
    def open(self) -> None:
        if self.bus is None:                            # Jika koneksi bus belum terbuka[cite: 1]
            self.bus = SMBus(self.bus_id)               # Buka jalur komunikasi I2C berdasarkan bus_id[cite: 1]
        self._configure_sensor()                        # Jalankan fungsi pengaturan register sensor[cite: 1]
        self._initialized = True                        # Ubah status penanda menjadi terinisialisasi[cite: 1]

    # 📖 Cerita Fungsi:
    # Ketika robot selesai bertugas, manajer menutup pintu saluran I2C dan mematikan
    # sambungan agar jalur komunikasi hemat daya dan tidak berbenturan dengan program lain[cite: 1].
    def close(self) -> None:
        if self.bus is not None:                        # Jika saluran I2C masih aktif terbuka[cite: 1]
            self.bus.close()                            # Tutup koneksi bus I2C[cite: 1]
            self.bus = None                             # Kosongkan kembali variabel bus[cite: 1]
        self._initialized = False                       # Ubah status penanda menjadi tidak aktif[cite: 1]

    # 📖 Cerita Fungsi:
    # Sebelum melakukan transaksi data, manajer memeriksa apakah pintu I2C sudah terbuka.
    # Jika pintu ternyata masih terkunci atau belum siap, ia akan membukanya terlebih dahulu[cite: 1].
    def ensure_open(self) -> None:
        if self.bus is None or not self._initialized:   # Jika bus belum dibuat atau belum siap[cite: 1]
            self.open()                                 # Buka dan konfigurasi sensor[cite: 1]

    # 📖 Cerita Fungsi:
    # Manajer mengirimkan satu amplop data (1 byte) ke salah satu laci register
    # di dalam chip MPU6050 untuk mengubah atau mengatur kinerjanya[cite: 1].
    def _write_byte(self, reg: int, value: int) -> None:
        assert self.bus is not None                     # Memastikan saluran bus I2C tidak kosong[cite: 1]
        self.bus.write_byte_data(self.address, reg, value) # Tuliskan 1 byte data ke register tujuan[cite: 1]

    # 📖 Cerita Fungsi:
    # Manajer mengintip satu laci register di dalam chip MPU6050 dan
    # mengambil satu lembar angka (1 byte) yang tersimpan di dalamnya[cite: 1].
    def _read_byte(self, reg: int) -> int:
        assert self.bus is not None                     # Memastikan saluran bus I2C tidak kosong[cite: 1]
        return self.bus.read_byte_data(self.address, reg) # Baca dan kembalikan 1 byte data dari register[cite: 1]

    # 📖 Cerita Fungsi:
    # MPU6050 menyimpan angka pengukuran besar dalam dua laci terpisah (High & Low byte).
    # Manajer mengambil isi dari kedua laci tersebut, menggabungkannya menjadi satu angka 16-bit,
    # dan menerjemahkan tandanya jika bernilai negatif[cite: 1].
    def _read_word(self, reg: int) -> int:
        high = self._read_byte(reg)                     # Baca byte tinggi dari register awal[cite: 1]
        low = self._read_byte(reg + 1)                  # Baca byte rendah dari register sebelahnya[cite: 1]
        value = (high << 8) | low                       # Geser byte tinggi 8 bit ke kiri lalu gabungkan dengan byte rendah[cite: 1]
        if value >= 0x8000:                             # Jika bit paling kiri bernilai 1 (menandakan angka negatif 16-bit)[cite: 1]
            value = -((65535 - value) + 1)              # Ubah menjadi nilai negatif menggunakan matematika Two's Complement[cite: 1]
        return value                                    # Kembalikan nilai integer bertanda (signed integer)[cite: 1]

    # 📖 Cerita Fungsi:
    # Manajer membangunkan MPU6050 dari mode tidur, mengatur ritme kecepatan pembacaan data,
    # menyetel rentang sensitivitas gyro dan accel, serta memberikan sudut awal kepada Kalman Filter[cite: 1].
    def _configure_sensor(self) -> None:
        self.ensure_bus_only()                          # Pastikan koneksi bus I2C dasar terhubung[cite: 1]
        self._write_byte(PWR_MGMT_1, 0x00)              # Tulis 0x00 ke PWR_MGMT_1 untuk membangunkan sensor dari mode sleep[cite: 1]
        time.sleep(0.05)                                # Beri jeda 50 ms agar sensor stabil[cite: 1]
        self._write_byte(SMPLRT_DIV, 0x07)              # Set pembagi laju sampel[cite: 1]
        self._write_byte(CONFIG, 0x00)                  # Set filter DLPF bawaan[cite: 1]
        self._write_byte(GYRO_CONFIG, 0x00)             # Set skala penuh Gyroscope ke +/- 250 deg/s (131 LSB/deg/s)[cite: 1]
        self._write_byte(ACCEL_CONFIG, 0x00)            # Set skala penuh Accelerometer ke +/- 2g (16384 LSB/g)[cite: 1]
        self._write_byte(INT_ENABLE, 0x01)              # Aktifkan interupsi data[cite: 1]
        time.sleep(0.05)                                # Beri jeda 50 ms[cite: 1]
        ax, ay = self._compute_acc_angles()             # Hitung sudut awal murni dari Accelerometer[cite: 1]
        self.kalman_x.setAngle(ax)                      # Set titik sudut awal pada Kalman Filter sumbu X[cite: 1]
        self.kalman_y.setAngle(ay)                      # Set titik sudut awal pada Kalman Filter sumbu Y[cite: 1]
        self._last_time = time.monotonic()              # Catat timestamp awal sebagai acuan dt[cite: 1]

    # 📖 Cerita Fungsi:
    # Manajer memastikan objek hardware bus I2C (SMBus) sudah dibuat sebelum dipakai[cite: 1].
    def ensure_bus_only(self) -> None:
        if self.bus is None:                            # Jika objek bus belum terinisialisasi[cite: 1]
            self.bus = SMBus(self.bus_id)               # Buat instansi SMBus baru[cite: 1]

    # 📖 Cerita Fungsi:
    # Saat robot diminta diam sempurna, manajer membaca gemetar halus (drift) bawaan gyro ratusan kali,
    # menghitung rata-rata penyimpangannya, lalu mencatat angka pengganggu tersebut untuk dikurangi di setiap pembacaan[cite: 1].
    def calibrate_gyro_bias(self, samples: int = config.GYRO_CALIBRATION_SAMPLES, delay_s: float = config.GYRO_CALIBRATION_DELAY_S) -> Dict[str, float]:
        self.ensure_open()                              # Pastikan sensor terbuka dan siap[cite: 1]
        sum_x = 0.0                                     # Variabel penampung total pembacaan gyro sumbu X[cite: 1]
        sum_y = 0.0                                     # Variabel penampung total pembacaan gyro sumbu Y[cite: 1]
        for _ in range(int(samples)):                   # Ulangi pembacaan sebanyak jumlah sampel yang ditentukan[cite: 1]
            gx = self._read_word(GYRO_XOUT_H) / 131.0   # Baca gyro X mentah lalu konversi ke deg/s[cite: 1]
            gy = self._read_word(GYRO_XOUT_H + 2) / 131.0 # Baca gyro Y mentah lalu konversi ke deg/s[cite: 1]
            sum_x += gx                                 # Tambahkan ke total sum_x[cite: 1]
            sum_y += gy                                 # Tambahkan ke total sum_y[cite: 1]
            time.sleep(delay_s)                         # Jeda waktu antar sampel[cite: 1]
        self.gyro_bias_x = sum_x / float(samples)       # Hitung rata-rata bias gyro sumbu X[cite: 1]
        self.gyro_bias_y = sum_y / float(samples)       # Hitung rata-rata bias gyro sumbu Y[cite: 1]
        ax, ay = self._compute_acc_angles()             # Baca ulang sudut accelerometer terkini[cite: 1]
        self.kalman_x.setAngle(ax)                      # Reset acuan sudut Kalman X[cite: 1]
        self.kalman_y.setAngle(ay)                      # Reset acuan sudut Kalman Y[cite: 1]
        self._last_time = time.monotonic()              # Perbarui catatan waktu terkini[cite: 1]
        return {"gyro_bias_x": self.gyro_bias_x, "gyro_bias_y": self.gyro_bias_y} # Kembalikan hasil bias[cite: 1]

    # 📖 Cerita Fungsi:
    # Manajer mengintip data gaya gravitasi dari accelerometer (sumbu X, Y, Z),
    # lalu menggunakan rumus trigonometri untuk menghitung berapa derajat robot sedang miring[cite: 1].
    def _compute_acc_angles(self) -> tuple[float, float]:
        acc_x = self._read_word(ACCEL_XOUT_H) / 16384.0 # Baca accelerometer X mentah lalu konversi ke satuan G[cite: 1]
        acc_y = self._read_word(ACCEL_XOUT_H + 2) / 16384.0 # Baca accelerometer Y mentah lalu konversi ke satuan G[cite: 1]
        acc_z = self._read_word(ACCEL_XOUT_H + 4) / 16384.0 # Baca accelerometer Z mentah lalu konversi ke satuan G[cite: 1]
        # Hitung sudut X (Roll) memakai rumus atan2 berdasarkan proyeksi gravitasi Y dan Z[cite: 1]
        acc_angle_x = math.degrees(math.atan2(acc_y, math.sqrt(acc_x * acc_x + acc_z * acc_z)))[cite: 1]
        # Hitung sudut Y (Pitch) memakai rumus atan2 berdasarkan proyeksi gravitasi X dan Z[cite: 1]
        acc_angle_y = math.degrees(math.atan2(-acc_x, math.sqrt(acc_y * acc_y + acc_z * acc_z)))[cite: 1]
        return acc_angle_x, acc_angle_y                # Kembalikan pasangan sudut accelerometer (deg)[cite: 1]

    # 📖 Cerita Fungsi:
    # Ini adalah tugas utama yang dijalankan terus-menerus pada siklus tinggi.
    # Manajer membaca kemiringan dari accelerometer, membaca kecepatan rotasi gyro (yang sudah dikurangi bias),
    # lalu mengumpankannya ke Kalman Filter. Hasil bersihnya dikoreksi dengan offset nol dan dikemas dalam Laporan AngleState[cite: 1].
    def read_angles(self) -> AngleState:
        self.ensure_open()                              # Pastikan sensor aktif[cite: 1]
        now = time.monotonic()                          # Ambil timestamp waktu terkini[cite: 1]
        if self._last_time is None:                     # Jika pembacaan pertama kali[cite: 1]
            self._last_time = now                       # Samakan waktu lalu dengan waktu sekarang[cite: 1]
        dt = max(1e-4, now - self._last_time)           # Hitung selisih waktu dt (minimal 0.0001 detik agar tidak bagi nol)[cite: 1]
        self._last_time = now                           # Perbarui waktu lalu dengan waktu sekarang[cite: 1]

        acc_angle_x, acc_angle_y = self._compute_acc_angles() # Hitung sudut accelerometer X dan Y[cite: 1]
        # Baca kecepatan rotasi gyro X dan kurangi dengan gyro_bias_x[cite: 1]
        gyro_rate_x = (self._read_word(GYRO_XOUT_H) / 131.0) - self.gyro_bias_x[cite: 1]
        # Baca kecepatan rotasi gyro Y dan kurangi dengan gyro_bias_y[cite: 1]
        gyro_rate_y = (self._read_word(GYRO_XOUT_H + 2) / 131.0) - self.gyro_bias_y[cite: 1]

        # Proses penyaringan Kalman Filter untuk sumbu X[cite: 1]
        kalman_x = self.kalman_x.getAngle(acc_angle_x, gyro_rate_x, dt)[cite: 1]
        # Proses penyaringan Kalman Filter untuk sumbu Y[cite: 1]
        kalman_y = self.kalman_y.getAngle(acc_angle_y, gyro_rate_y, dt)[cite: 1]

        axis = config.IMU_AXIS.strip().lower()          # Buka konfigurasi sumbu yang dipilih[cite: 1]
        if axis == "roll":                              # Jika memilih sumbu roll[cite: 1]
            angle_deg = kalman_x                        # Gunakan hasil kalman_x[cite: 1]
        else:                                           # Jika memilih sumbu pitch[cite: 1]
            angle_deg = kalman_y                        # Gunakan hasil kalman_y[cite: 1]
            axis = "pitch"                              # Tetapkan nama sumbu aktif ke "pitch"[cite: 1]

        # Koreksi sudut dengan mengurangi zero_offset_deg lalu dikalikan dengan penanda arah IMU_SIGN[cite: 1]
        angle_deg = (angle_deg - self.zero_offset_deg) * float(config.IMU_SIGN)[cite: 1]

        # Bungkus seluruh data pembacaan terbaru ke dalam objek AngleState lalu kembalikan[cite: 1]
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

    # 📖 Cerita Fungsi:
    # Saat robot dipegang tegak lurus secara fisik, manajer mengukur sudut kemiringan rata-rata beberapa saat.
    # Nilai ini dijadikan acuan 'titik nol baru' agar robot tahu kondisi persis saat ia berdiri seimbang[cite: 1].
    def calibrate_zero_angle(self, samples: int = config.ZERO_CALIBRATION_SAMPLES, delay_s: float = config.ZERO_CALIBRATION_DELAY_S) -> float:
        self.ensure_open()                              # Pastikan sensor terbuka[cite: 1]
        total = 0.0                                     # Variabel akumulator total nilai sudut[cite: 1]
        for _ in range(int(samples)):                   # Ulangi pembacaan sebanyak jumlah sampel[cite: 1]
            state = self.read_angles()                  # Ambil state pembacaan sudut terkini[cite: 1]
            # Pilih nilai sudut mentah (kalman_x / kalman_y) sesuai sumbu yang digunakan[cite: 1]
            raw_angle = state.kalman_x if state.axis_used == "roll" else state.kalman_y[cite: 1]
            total += raw_angle                          # Tambahkan ke akumulator[cite: 1]
            time.sleep(delay_s)                         # Jeda waktu antar sampel[cite: 1]
        self.zero_offset_deg = total / float(samples)   # Hitung rata-rata offset titik nol mekanis[cite: 1]
        return self.zero_offset_deg                     # Kembalikan nilai zero_offset_deg[cite: 1]

    # 📖 Cerita Fungsi:
    # Manajer mengambil foto instan (snapshot) berisi ringkasan data sudut dan status penting saat itu juga,
    # lalu membungkusnya dalam format kamus (dictionary) agar mudah dicetak di terminal[cite: 1].
    def get_debug_snapshot(self) -> Dict[str, float]:
        state = self.read_angles()                      # Ambil state pembacaan sudut terkini[cite: 1]
        return {
            "angle_deg": state.angle_deg,
            "dt": state.dt,
            "gyro_rate_x": state.gyro_rate_x,
            "gyro_rate_y": state.gyro_rate_y,
            "kalman_x": state.kalman_x,
            "kalman_y": state.kalman_y,
            "zero_offset_deg": self.zero_offset_deg,
        }                                               # Kembalikan dictionary ringkasan data telemetri[cite: 1]