# Menunda evaluasi type hints agar penulisan tipe data modern (seperti | atau Optional) disimpan sebagai string lebih dahulu[cite: 1]
from __future__ import annotations

# Mengimpor modul logging bawaan Python untuk mencatat pesan status dan info sistem[cite: 1]
import logging
# Mengimpor modul struct untuk mengonversi nilai biner C-style ke tipe data Python (pack/unpack int16/uint16)[cite: 1]
import struct
# Mengimpor modul time untuk fungsi jeda waktu (sleep)[cite: 1]
import time
# Mengimpor generator struktur data otomatis (@dataclass)[cite: 1]
from dataclasses import dataclass
# Mengimpor penanda tipe data opsional dan dictionary dari modul typing[cite: 1]
from typing import Optional, Dict, Any

# Mengimpor modul pySerial untuk mengelola komunikasi serial RS485/UART ke driver motor[cite: 1, 3]
import serial

# Mengimpor berkas konfigurasi parameter global sistem[cite: 1, 3]
import config

# Membuat objek logger khusus untuk modul ini agar catatan error/info tercatat dengan nama berkas[cite: 1]
LOGGER = logging.getLogger(__name__)

# --- KONFIGURASI MODE DDSM115 (BERDASARKAN DOKUMENTASI WAVESHARE) ---
MODE_CURRENT = 0x01   # Kode Biner Mode Current/Torque (Mode Arus listrik)[cite: 1, 3]
MODE_SPEED = 0x02     # Kode Biner Mode Velocity/Speed (Mode Kecepatan RPM)[cite: 1, 3]
MODE_POSITION = 0x03  # Kode Biner Mode Position (Mode Posisi Sudut)[cite: 1, 3]

# --- KONFIGURASI PERINDAH (COMMAND BYTE) ---
CMD_CONTROL = 0x64      # Kode perintah untuk mengirim sinyal kendali (Current/Velocity/Position)[cite: 1, 3]
CMD_QUERY = 0x74        # Kode perintah untuk meminta status telemetri motor (Suhu, Encoder, Error)[cite: 1, 3]
CMD_SWITCH_MODE = 0xA0  # Kode perintah untuk mengganti mode operasi motor[cite: 1, 3]

# --- PARAMETER FRAME PROTOKOL RS485 ---
FRAME_LEN = 10           # Panjang satu paket frame komunikasi RS485 (selalu 10 byte)[cite: 1, 3]
CRC8_INIT = 0x00         # Nilai inisialisasi awal kalkulasi CRC-8/MAXIM[cite: 1, 3]
CRC8_POLY_REVERSED = 0x8C # Polinomial terbalik untuk algoritma CRC-8/MAXIM[cite: 1, 3]


# Class pengecualian dasar (Base Exception) khusus untuk error driver DDSM115[cite: 1]
class DDSM115Error(Exception):
    """Base exception untuk driver DDSM115."""


# Class pengecualian saat hasil kalkulasi CRC-8 tidak cocok (frame rusak/corrupt)[cite: 1]
class CRCError(DDSM115Error):
    """CRC frame tidak valid."""


# Class pengecualian saat koneksi serial mengalami batas waktu tunggu (Timeout)[cite: 1]
class SerialTimeoutError(DDSM115Error):
    """Timeout saat menunggu reply serial."""


# Struktur pembungkus data laporan umpan balik (telemetri) dari motor DDSM115[cite: 1, 3]
@dataclass
class MotorFeedback:
    motor_id: int                    # ID unik motor (misal: ID 1 atau ID 2)[cite: 1, 3]
    mode: int                        # Mode aktif motor saat ini (1: Current, 2: Speed, 3: Position)[cite: 1, 3]
    torque_raw: int                  # Nilai torsi/arus mentah dalam bentuk integer signed 16-bit[cite: 1, 3]
    torque_ampere: float             # Nilai arus listrik terhitung dalam satuan Ampere (A)[cite: 1, 3]
    speed_rpm: int                   # Kecepatan putar roda terukur dalam satuan RPM[cite: 1, 3]
    position_raw: int                # Nilai posisi encoder mentah (16-bit atau 8-bit)[cite: 1, 3]
    position_deg: float              # Nilai posisi sudut roda dalam satuan derajat (0-360 deg)[cite: 1, 3]
    error_code: int                  # Byte kode indikator kesalahan/peringatan dari driver motor[cite: 1, 3]
    temperature_c: Optional[int] = None # Suhu internal driver motor dalam derajat Celsius (&deg;C)[cite: 1, 3]
    position_u8: Optional[int] = None   # Posisi encoder 8-bit dari paket Query[cite: 1, 3]
    source: str = "control"          # Sumber paket umpan balik ("control" atau "query")[cite: 1, 3]


class DDSM115Motor:

    # 📖 Cerita Fungsi:
    # Sebelum dapat memerintah motor, manajer DDSM115Motor menyiapkan semua dokumen identitas motor.
    # Dia mencatat lokasi port USB/serial, nomor ID motor, kecepatan baudrate, batas waktu timeout,
    # arah putaran positif/negatif, serta menyiapkan tempat penyimpanan laporan telemetri terbaru[cite: 1, 3].
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
        self.port = port                                       # Menyimpan string alamat port serial (misal: '/dev/ttyUSB0')[cite: 1, 3]
        self.motor_id = int(motor_id)                          # Menyimpan ID unik motor DDSM115[cite: 1, 3]
        self.baudrate = int(baudrate)                          # Menyimpan kecepatan baudrate komunikasi (115200 bps)[cite: 1, 3]
        self.timeout = float(timeout)                          # Menyimpan batas waktu tunggu balasan serial (detik)[cite: 1, 3]
        self.sign = 1.0 if sign >= 0 else -1.0                 # Menentukan pembalik arah putaran roda (+1.0 atau -1.0)[cite: 1, 3]
        self.control_mode = control_mode.strip().lower()       # Menentukan mode kontrol utama ("current" atau "speed")[cite: 1, 3]
        self.accel_time = max(0, min(255, int(accel_time)))    # Membatasi waktu akselerasi dalam rentang 0-255[cite: 1, 3]
        self.name = name                                       # Nama panggilan motor untuk keperluan log (misal: 'left_motor')[cite: 1, 3]
        self._ser: Optional[serial.Serial] = None              # Mempersiapkan variabel penampung koneksi pySerial[cite: 1, 3]
        self._current_mode_value: Optional[int] = None         # Mempersiapkan penampung status mode biner aktif[cite: 1, 3]
        self.last_feedback: Optional[MotorFeedback] = None     # Mempersiapkan tempat menyimpan laporan MotorFeedback terakhir[cite: 1, 3]

    # 📖 Cerita Fungsi:
    # Manajer membuka jalur kabel serial RS485 ke motor dengan konfigurasi standar Waveshare (115200 baud, 8N1).
    # Setelah pintu terbuka, ia membersihkan memori penampung (buffer) agar data lama tidak mengganggu[cite: 1, 3].
    def open(self) -> None:
        if self._ser and self._ser.is_open:                    # Jika koneksi serial sudah terbuka sebelumnya[cite: 1, 3]
            return                                             # Langsung keluar, tidak perlu membuka ulang[cite: 1, 3]
        self._ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,                         # Format 8 bit data[cite: 1, 3]
            parity=serial.PARITY_NONE,                         # Tanpa bit paritas[cite: 1, 3]
            stopbits=serial.STOPBITS_ONE,                      # 1 bit stop (8N1)[cite: 1, 3]
            timeout=self.timeout,
            write_timeout=self.timeout,
        )                                                      # Buka koneksi serial RS485[cite: 1, 3]
        self._ser.reset_input_buffer()                         # Bersihkan buffer penerimaan data (RX)[cite: 1, 3]
        self._ser.reset_output_buffer()                        # Bersihkan buffer pengiriman data (TX)[cite: 1, 3]
        LOGGER.info("%s opened on %s", self.name, self.port)   # Catat log bahwa port serial berhasil dibuka[cite: 1, 3]

    # 📖 Cerita Fungsi:
    # Ketika robot dimatikan, manajer menutup saluran komunikasi serial RS485 dengan aman[cite: 1, 3].
    def close(self) -> None:
        if self._ser and self._ser.is_open:                    # Jika saluran serial aktif terbuka[cite: 1, 3]
            try:
                self._ser.close()                              # Tutup koneksi serial pySerial[cite: 1, 3]
            finally:
                LOGGER.info("%s closed", self.name)            # Catat log bahwa koneksi telah ditutup[cite: 1, 3]

    # 📖 Cerita Fungsi:
    # Manajer memeriksa status sakelar saluran serial, memberikan kepastian apakah saluran sedang terbuka atau mati[cite: 1, 3].
    @property
    def is_open(self) -> bool:
        return bool(self._ser and self._ser.is_open)          # Kembalikan True jika serial terhubung dan terbuka[cite: 1, 3]

    # 📖 Cerita Fungsi:
    # Sebelum mengirim perintah, manajer memastikan pintu serial terbuka. Jika belum, ia membukanya secara otomatis[cite: 1, 3].
    def ensure_open(self) -> None:
        if not self.is_open:                                   # Jika serial belum terbuka[cite: 1, 3]
            self.open()                                        # Jalankan prosedur membuka serial[cite: 1, 3]

    # 📖 Cerita Fungsi:
    # Ini adalah mesin kalkulator pemeriksaan integritas data (CRC-8/MAXIM).
    # Ia mengolah susunan byte data menggunakan rumus pemutar bit matematis untuk menghasilkan 1 byte stempel pengaman[cite: 1, 3].
    @staticmethod
    def crc8_maxim(data: bytes) -> int:
        crc = CRC8_INIT                                        # Set nilai awal CRC = 0x00[cite: 1, 3]
        for byte in data:                                      # Ulangi untuk setiap byte dalam data[cite: 1, 3]
            crc ^= byte                                        # Operasi XOR byte ke akumulator CRC[cite: 1, 3]
            for _ in range(8):                                 # Ulangi pergeseran untuk 8 bit[cite: 1, 3]
                if crc & 0x01:                                 # Jika bit paling kanan bernilai 1[cite: 1, 3]
                    crc = (crc >> 1) ^ CRC8_POLY_REVERSED      # Geser kanan 1 bit lalu XOR dengan polinomial 0x8C[cite: 1, 3]
                else:
                    crc >>= 1                                  # Geser kanan 1 bit saja[cite: 1, 3]
        return crc & 0xFF                                      # Kembalikan hasil akhir 8-bit (1 byte)[cite: 1, 3]

    # 📖 Cerita Fungsi:
    # Manajer menyusun 9 byte isi perintah utama, lalu menghitung stempel pengaman CRC-8 byte ke-10,
    # dan menggabungkannya menjadi 1 paket frame utuh 10 byte yang siap dikirim via kabel serial[cite: 1, 3].
    @classmethod
    def build_frame(cls, b0: int, b1: int, b2: int, b3: int, b4: int, b5: int, b6: int, b7: int, b8: int) -> bytes:
        payload = bytes([
            b0 & 0xFF, b1 & 0xFF, b2 & 0xFF, b3 & 0xFF, b4 & 0xFF,
            b5 & 0xFF, b6 & 0xFF, b7 & 0xFF, b8 & 0xFF,
        ])                                                     # Susun 9 byte pertama paket payload[cite: 1, 3]
        crc = cls.crc8_maxim(payload)                          # Hitung stempel CRC-8 dari 9 byte tersebut[cite: 1, 3]
        return payload + bytes([crc])                          # Gabungkan 9 byte payload + 1 byte CRC menjadi 10 byte utuh[cite: 1, 3]

    # 📖 Cerita Fungsi:
    # Mengubah angka integer bertanda 16-bit (-32768 s.d +32767) menjadi 2 potongan byte (High byte & Low byte) format Big-Endian[cite: 1, 3].
    @staticmethod
    def int16_to_hi_lo(value: int) -> tuple[int, int]:
        packed = struct.pack(">h", int(value))                 # Bungkus angka ke format signed 16-bit Big-Endian (>h)[cite: 1, 3]
        return packed[0], packed[1]                            # Kembalikan High Byte dan Low Byte[cite: 1, 3]

    # 📖 Cerita Fungsi:
    # Mengubah angka integer positif 16-bit (0 s.d 65535) menjadi 2 potongan byte (High byte & Low byte) format Big-Endian[cite: 1, 3].
    @staticmethod
    def uint16_to_hi_lo(value: int) -> tuple[int, int]:
        packed = struct.pack(">H", int(value))                 # Bungkus angka ke format unsigned 16-bit Big-Endian (>H)[cite: 1, 3]
        return packed[0], packed[1]                            # Kembalikan High Byte dan Low Byte[cite: 1, 3]

    # 📖 Cerita Fungsi:
    # Menggabungkan 2 potongan byte (High & Low) kembali menjadi satu angka integer bertanda 16-bit (signed int16)[cite: 1, 3].
    @staticmethod
    def hi_lo_to_int16(hi: int, lo: int) -> int:
        return struct.unpack(">h", bytes([hi & 0xFF, lo & 0xFF]))[0] # Unpack 2 byte menjadi signed short int[cite: 1, 3]

    # 📖 Cerita Fungsi:
    # Menggabungkan 2 potongan byte (High & Low) kembali menjadi satu angka integer positif 16-bit (unsigned uint16)[cite: 1, 3].
    @staticmethod
    def hi_lo_to_uint16(hi: int, lo: int) -> int:
        return struct.unpack(">H", bytes([hi & 0xFF, lo & 0xFF]))[0] # Unpack 2 byte menjadi unsigned short int[cite: 1, 3]

    # 📖 Cerita Fungsi:
    # Manajer mengirimkan paket 10 byte ke kabel RS485, lalu menunggu jawaban 10 byte dari motor.
    # Jika balasan datang tepat waktu dan valid, ia menyerahkan paket jawaban tersebut untuk diproses[cite: 1, 3].
    def _write_and_read(self, frame: bytes, expect_reply: bool = True) -> Optional[bytes]:
        self.ensure_open()                                     # Pastikan koneksi serial aktif[cite: 1, 3]
        assert self._ser is not None                           # Penegasan bahwa objek serial terdefinisi[cite: 1, 3]
        self._ser.reset_input_buffer()                         # Bersihkan sisa data lama di buffer masukan[cite: 1, 3]
        self._ser.write(frame)                                 # Kirimkan paket 10 byte frame via serial RS485[cite: 1, 3]
        self._ser.flush()                                      # Pastikan seluruh byte terkirim sempurna[cite: 1, 3]
        if not expect_reply:                                   # Jika perintah tidak membutuhkan balasan (misal Switch Mode)[cite: 1, 3]
            return None                                        # Langsung keluar[cite: 1, 3]
        reply = self._ser.read(FRAME_LEN)                      # Baca balasan tepat 10 byte dari motor[cite: 1, 3]
        if len(reply) != FRAME_LEN:                            # Jika panjang balasan kurang dari 10 byte (timeout)[cite: 1, 3]
            raise SerialTimeoutError(f"{self.name} timeout/read short frame: got {len(reply)} bytes") # Lempar error timeout[cite: 1, 3]
        self.validate_reply(reply)                             # Periksa integritas CRC paket balasan[cite: 1, 3]
        return reply                                           # Kembalikan paket biner balasan[cite: 1, 3]

    # 📖 Cerita Fungsi:
    # Manajer memeriksa apakah paket balasan dari motor memiliki panjang 10 byte dan
    # apakah stempel pengaman CRC-8 di byte ke-10 cocok dengan perhitungan matematis. Jika cacat, paket dibuang[cite: 1, 3]!
    @classmethod
    def validate_reply(cls, frame: bytes) -> None:
        if len(frame) != FRAME_LEN:                            # Jika panjang frame tidak sama dengan 10 byte[cite: 1, 3]
            raise DDSM115Error(f"Invalid frame length: {len(frame)}") # Lempar error panjang frame tidak valid[cite: 1, 3]
        expected = cls.crc8_maxim(frame[:9])                   # Hitung CRC dari 9 byte pertama paket balasan[cite: 1, 3]
        got = frame[9]                                         # Ambil byte CRC ke-10 dari motor[cite: 1, 3]
        if expected != got:                                    # Jika hasil hitungan tidak sama dengan byte ke-10[cite: 1, 3]
            raise CRCError(f"CRC mismatch: expected 0x{expected:02X}, got 0x{got:02X}") # Lempar error CRC Mismatch[cite: 1, 3]

    # 📖 Cerita Fungsi:
    # Menerjemahkan paket biner balasan instruksi kendali (CMD 0x64) dari motor menjadi angka fisik:
    # nilai torsi raw dikonversi ke Ampere, dan nilai encoder raw dikonversi ke derajat (0-360 deg)[cite: 1, 3].
    def _parse_control_feedback(self, frame: bytes) -> MotorFeedback:
        motor_id = frame[0]                                    # Byte 0: ID motor pengirim balasan[cite: 1, 3]
        mode = frame[1]                                        # Byte 1: Mode aktif motor[cite: 1, 3]
        torque_raw = self.hi_lo_to_int16(frame[2], frame[3])   # Byte 2-3: Torsi mentah (signed int16)[cite: 1, 3]
        speed_rpm = self.hi_lo_to_int16(frame[4], frame[5])    # Byte 4-5: Kecepatan RPM mentah (signed int16)[cite: 1, 3]
        position_raw = self.hi_lo_to_uint16(frame[6], frame[7]) # Byte 6-7: Posisi encoder mentah (unsigned uint16)[cite: 1, 3]
        error_code = frame[8]                                  # Byte 8: Kode status error motor[cite: 1, 3]
        torque_ampere = (torque_raw / 32767.0) * 8.0           # Konversi skala mentah int16 ke satuan Ampere (max +/-8A)[cite: 1, 3]
        position_deg = (position_raw / 32767.0) * 360.0 if position_raw <= 32767 else 0.0 # Konversi encoder ke derajat 0-360 deg[cite: 1, 3]
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
        )                                                      # Bungkus data ke dalam objek MotorFeedback[cite: 1, 3]
        self.last_feedback = fb                                # Simpan ke catatan laporan terakhir[cite: 1, 3]
        return fb                                              # Kembalikan objek MotorFeedback[cite: 1, 3]

    # 📖 Cerita Fungsi:
    # Menerjemahkan paket biner balasan instruksi query (CMD 0x74) dari motor.
    # Paket ini memuat informasi tambahan berupa suhu internal driver motor dalam Celsius (&deg;C)[cite: 1, 3].
    def _parse_query_feedback(self, frame: bytes) -> MotorFeedback:
        motor_id = frame[0]                                    # Byte 0: ID motor[cite: 1, 3]
        mode = frame[1]                                        # Byte 1: Mode aktif motor[cite: 1, 3]
        torque_raw = self.hi_lo_to_int16(frame[2], frame[3])   # Byte 2-3: Torsi mentah[cite: 1, 3]
        speed_rpm = self.hi_lo_to_int16(frame[4], frame[5])    # Byte 4-5: Kecepatan RPM mentah[cite: 1, 3]
        temperature_c = frame[6]                               # Byte 6: Suhu driver motor (deg C)[cite: 1, 3]
        position_u8 = frame[7]                                 # Byte 7: Posisi encoder resolusi 8-bit[cite: 1, 3]
        error_code = frame[8]                                  # Byte 8: Kode status error[cite: 1, 3]
        torque_ampere = (torque_raw / 32767.0) * 8.0           # Konversi skala mentah ke Ampere[cite: 1, 3]
        position_deg = (position_u8 / 255.0) * 360.0          # Konversi encoder 8-bit ke derajat (0-360 deg)[cite: 1, 3]
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
        )                                                      # Bungkus data query ke dalam objek MotorFeedback[cite: 1, 3]
        self.last_feedback = fb                                # Simpan ke catatan laporan terakhir[cite: 1, 3]
        return fb                                              # Kembalikan objek MotorFeedback[cite: 1, 3]

    # 📖 Cerita Fungsi:
    # Manajer mengirimkan instruksi pergantian mode kerja ke motor (CMD 0xA0)
    # untuk memilih apakah motor bekerja dalam mode Current (1), Speed (2), atau Position (3)[cite: 1, 3].
    def set_mode(self, mode: str | int) -> None:
        if isinstance(mode, str):                              # Jika masukan berupa teks string[cite: 1, 3]
            mode_key = mode.strip().lower()                    # Bersihkan spasi dan ubah ke huruf kecil[cite: 1, 3]
            if mode_key == "current":                          # Mode Arus Listrik[cite: 1, 3]
                mode_val = MODE_CURRENT
            elif mode_key == "speed":                          # Mode Kecepatan RPM[cite: 1, 3]
                mode_val = MODE_SPEED
            elif mode_key == "position":                       # Mode Posisi Sudut[cite: 1, 3]
                mode_val = MODE_POSITION
            else:
                raise ValueError(f"Unknown mode: {mode}")       # Lempar error jika mode tidak dikenal[cite: 1, 3]
        else:
            mode_val = int(mode)                               # Jika masukan sudah berupa angka integer[cite: 1, 3]
        frame = bytes([
            self.motor_id & 0xFF, CMD_SWITCH_MODE,
            0, 0, 0, 0, 0, 0, 0, mode_val & 0xFF,
        ])                                                     # Susun frame ganti mode (CMD 0xA0)[cite: 1, 3]
        self._write_and_read(frame, expect_reply=False)        # Kirim frame tanpa menunggu balasan[cite: 1, 3]
        self._current_mode_value = mode_val                    # Simpan status mode aktif[cite: 1, 3]
        time.sleep(0.02)                                       # Beri jeda 20 ms agar motor memproses mode baru[cite: 1, 3]

    # 📖 Cerita Fungsi:
    # Manajer Mengirimkan surat pertanyaan (CMD 0x74) untuk menanyakan kondisi kesehatan motor,
    # termasuk membaca nilai suhu internal driver motor[cite: 1, 3].
    def query_status(self) -> MotorFeedback:
        frame = self.build_frame(self.motor_id, CMD_QUERY, 0, 0, 0, 0, 0, 0, 0) # Susun frame perintah query (CMD 0x74)[cite: 1, 3]
        reply = self._write_and_read(frame, expect_reply=True) # Kirim dan terima balasan 10 byte[cite: 1, 3]
        assert reply is not None                               # Memastikan balasan tidak kosong[cite: 1, 3]
        return self._parse_query_feedback(reply)               # Terjemahkan dan kembalikan laporan query[cite: 1, 3]

    # 📖 Cerita Fungsi:
    # Fungsi dasar mengirimkan nilai perintah mentah int16 ke motor (CMD 0x64),
    # serta mengatur byte pengereman aktif (brake) jika diperlukan[cite: 1, 3].
    def send_raw_command(self, command_value: int, brake: bool = False) -> MotorFeedback:
        hi, lo = self.int16_to_hi_lo(command_value)            # Pecah angka perintah ke High dan Low byte[cite: 1, 3]
        frame = self.build_frame(
            self.motor_id,
            CMD_CONTROL,
            hi,
            lo,
            0,
            0,
            self.accel_time,
            0xFF if brake else 0x00,                            # Set byte 7 = 0xFF jika pengereman aktif[cite: 1, 3]
            0,
        )                                                      # Susun frame perintah kendali (CMD 0x64)[cite: 1, 3]
        reply = self._write_and_read(frame, expect_reply=True) # Kirim dan terima balasan telemetri terbaru[cite: 1, 3]
        assert reply is not None                               # Memastikan balasan tidak kosong[cite: 1, 3]
        return self._parse_control_feedback(reply)             # Terjemahkan dan kembalikan laporan kendali[cite: 1, 3]

    # 📖 Cerita Fungsi:
    # Mengonversi perintah arus dalam satuan Ampere (-8.0A s.d +8.0A) menjadi nilai mentah int16,
    # lalu mengirimkannya ke motor DDSM115[cite: 1, 3].
    def command_current_amp(self, current_amp: float) -> MotorFeedback:
        current_amp = max(-8.0, min(8.0, float(current_amp)))  # Batasi masukan arus dalam rentang aman -8.0A hingga +8.0A[cite: 1, 3]
        raw = int((current_amp / 8.0) * 32767.0)               # Konversi nilai Ampere ke skala int16 (-32767 s.d +32767)[cite: 1, 3]
        return self.send_raw_command(raw, brake=False)         # Kirim nilai mentah ke motor[cite: 1, 3]

    # 📖 Cerita Fungsi:
    # Mengonversi perintah kecepatan dalam satuan RPM (-330 RPM s.d +330 RPM) menjadi nilai mentah int16,
    # lalu mengirimkannya ke motor DDSM115[cite: 1, 3].
    def command_speed_rpm(self, speed_rpm: float) -> MotorFeedback:
        speed_rpm = max(-330.0, min(330.0, float(speed_rpm)))  # Batasi masukan kecepatan dalam rentang -330 hingga +330 RPM[cite: 1, 3]
        raw = int(round(speed_rpm))                            # Bulatkan nilai RPM ke integer[cite: 1, 3]
        return self.send_raw_command(raw, brake=False)         # Kirim nilai mentah ke motor[cite: 1, 3]

    # 📖 Cerita Fungsi:
    # Memerintahkan motor untuk berhenti berputar dengan mengirimkan sinyal daya 0.0 Ampere atau 0.0 RPM
    # sesuai dengan mode kontrol yang sedang aktif[cite: 1, 3].
    def stop(self) -> MotorFeedback:
        if self.control_mode == "current":                     # Jika bekerja pada mode arus listrik[cite: 1, 3]
            return self.command_current_amp(0.0)               # Kirim perintah 0.0 Ampere[cite: 1, 3]
        return self.command_speed_rpm(0.0)                     # Kirim perintah 0.0 RPM[cite: 1, 3]

    # 📖 Cerita Fungsi:
    # Menerima nilai daya ter-normalisasi dari controller (-1.0 s.d +1.0),
    # mengalikkannya dengan penanda arah (sign) serta batas maksimum fisik (MAX_CURRENT_A / MAX_SPEED_RPM),
    # lalu mengeksekusinya ke motor[cite: 1, 3].
    def command_normalized(self, normalized: float) -> MotorFeedback:
        normalized = max(-1.0, min(1.0, float(normalized)))   # Batasi sinyal masukan ke rentang [-1.0, 1.0][cite: 1, 3]
        normalized *= self.sign                                # Dikalikan dengan sign untuk penyesuaian arah fisik roda[cite: 1, 3]
        if self.control_mode == "current":                     # Jika mode kontrol adalah "current"[cite: 1, 3]
            return self.command_current_amp(normalized * config.MAX_CURRENT_A) # Skalakan ke Ampere maksimum (misal 1.8A)[cite: 1, 3]
        if self.control_mode == "speed":                       # Jika mode kontrol adalah "speed"[cite: 1, 3]
            return self.command_speed_rpm(normalized * config.MAX_SPEED_RPM)   # Skalakan ke RPM maksimum (misal 120 RPM)[cite: 1, 3]
        raise ValueError(f"Unsupported control_mode for balancing: {self.control_mode}") # Lempar error jika mode tidak didukung[cite: 1, 3]

    # 📖 Cerita Fungsi:
    # Prosedur penyiapan awal motor. Membuka saluran serial, mengatur mode kerja aktif (Current/Speed),
    # dan memastikan motor dalam kondisi berhenti diam sebelum digunakan[cite: 1, 3].
    def initialize(self) -> None:
        self.ensure_open()                                     # Pastikan port serial terbuka[cite: 1, 3]
        self.set_mode(self.control_mode)                       # Atur mode kerja motor berdasarkan konfigurasi[cite: 1, 3]
        self.stop()                                            # Hentikan motor agar tidak berputar liar saat menyala[cite: 1, 3]

    # 📖 Cerita Fungsi:
    # Menerjemahkan byte error_code biner dari motor menggunakan operasi bitwise AND (&)
    # untuk mengetahui jenis kerusakan fisik yang sedang terjadi (sensor error, overcurrent, stall/macet)[cite: 1, 3].
    def decode_error_flags(self, error_code: int) -> Dict[str, bool]:
        code = int(error_code) & 0xFF                          # Pastikan nilai berada pada rentang 8-bit[cite: 1, 3]
        return {
            "sensor_error": bool(code & 0x01),                 # Bit 0: Indikator kerusakan sensor internal[cite: 1, 3]
            "overcurrent_error": bool(code & 0x02),            # Bit 1: Indikator kelebihan arus (Overcurrent)[cite: 1, 3]
            "phase_overcurrent_error": bool(code & 0x04),      # Bit 2: Indikator kelebihan arus fasa (Phase Overcurrent)[cite: 1, 3]
            "stall_error": bool(code & 0x08),                  # Bit 3: Indikator motor macet/terkunci (Stall)[cite: 1, 3]
            "troubleshooting": bool(code & 0x10),              # Bit 4: Indikator perlunya tindakan troubleshooting[cite: 1, 3]
        }


class DualDDSM115:

    # 📖 Cerita Fungsi:
    # Pengelola tingkat tinggi (High-level Wrapper) yang membungkus dua unit motor DDSM115 (Roda Kiri dan Roda Kanan)
    # sehingga keduanya dapat dikendalikan dan dikalibrasi secara bersamaan[cite: 1, 3].
    def __init__(self) -> None:
        # Inisialisasi objek DDSM115Motor untuk Roda Kiri[cite: 1, 3]
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
        # Inisialisasi objek DDSM115Motor untuk Roda Kanan[cite: 1, 3]
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

    # 📖 Cerita Fungsi:
    # Membuka jalur komunikasi serial RS485 untuk kedua motor (Kiri dan Kanan) sekaligus[cite: 1, 3].
    def open(self) -> None:
        self.left.open()                                       # Buka port serial motor kiri[cite: 1, 3]
        self.right.open()                                      # Buka port serial motor kanan[cite: 1, 3]

    # 📖 Cerita Fungsi:
    # Menjalankan prosedur inisialisasi dasar (buka port, set mode, stop) untuk kedua motor sekaligus[cite: 1, 3].
    def initialize(self) -> None:
        self.left.initialize()                                 # Inisialisasi motor kiri[cite: 1, 3]
        self.right.initialize()                                # Inisialisasi motor kanan[cite: 1, 3]

    # 📖 Cerita Fungsi:
    # Prosedur penghentian darurat. Memerintahkan kedua motor untuk berhenti berputar.
    # Jika salah satu motor mengalami error saat dihentikan, ia akan mengumpulkan catatan error tersebut[cite: 1, 3].
    def stop_all(self) -> None:
        errors = []                                            # Tempat penampung catatan error[cite: 1, 3]
        for motor in (self.left, self.right):                  # Ulangi untuk motor kiri dan motor kanan[cite: 1, 3]
            try:
                motor.stop()                                   # Hentikan putaran motor[cite: 1, 3]
            except Exception as exc:                           # Tangkap jika terjadi kegagalan/error[cite: 1, 3]
                errors.append(exc)                             # Catat pesan error ke daftar[cite: 1, 3]
        if errors:                                             # Jika terdapat error saat menghentikan motor[cite: 1, 3]
            raise DDSM115Error(f"Stop all encountered {len(errors)} error(s): {errors}") # Lempar exception ringkasan error[cite: 1, 3]

    # 📖 Cerita Fungsi:
    # Menerima dua sinyal daya ter-normalisasi (-1.0 s.d +1.0) untuk roda kiri dan roda kanan,
    # mengeksekusinya ke masing-masing motor, dan mengembalikan dictionary laporan umpan balik keduanya[cite: 1, 3].
    def command_normalized(self, left_value: float, right_value: float) -> Dict[str, MotorFeedback]:
        left_fb = self.left.command_normalized(left_value)     # Kirim sinyal daya ke motor kiri[cite: 1, 3]
        right_fb = self.right.command_normalized(right_value)   # Kirim sinyal daya ke motor kanan[cite: 1, 3]
        return {"left": left_fb, "right": right_fb}            # Kembalikan pasangan laporan MotorFeedback[cite: 1, 3]

    # 📖 Cerita Fungsi:
    # Mengirimkan perintah query status ke kedua motor untuk membaca kondisi kesehatan dan suhu internal driver keduanya[cite: 1, 3].
    def query_both(self) -> Dict[str, MotorFeedback]:
        return {
            "left": self.left.query_status(),                  # Lakukan query status ke motor kiri[cite: 1, 3]
            "right": self.right.query_status(),                # Lakukan query status ke motor kanan[cite: 1, 3]
        }

    # 📖 Cerita Fungsi:
    # Menutup seluruh jalur komunikasi serial RS485 kedua motor dengan aman saat program selesai[cite: 1, 3].
    def close(self) -> None:
        self.left.close()                                      # Tutup port serial motor kiri[cite: 1, 3]
        self.right.close()                                     # Tutup port serial motor kanan[cite: 1, 3]