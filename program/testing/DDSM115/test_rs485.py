import serial
import time

SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE = 115200

try:
    # Konfigurasi port yang disesuaikan khusus untuk driver ttyACM Linux fisik
    ser = serial.Serial(
        port=SERIAL_PORT,
        baudrate=BAUDRATE,
        timeout=0.1,          # Timeout diperkecil agar deteksi data masuk (RX) instan
        write_timeout=1.0,
        rtscts=False,         # Wajib False: Waveshare (B) mengatur arah TX/RX otomatis di dalam hardware
        dsrdtr=False          # Wajib False: Mencegah driver Linux membekukan chip CH343
    )
    print(f"--- Berhasil membuka {SERIAL_PORT} ---")
except Exception as e:
    print(f"Gagal membuka port: {e}")
    exit()

# Data Hex simulasi perintah DDSM115 (ID: 0x01)
data_test = bytes([0x01, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50])

print("Memulai Loop Pengiriman Data Khusus Raspberry Pi 5...\n")
counter = 1

try:
    while True:
        print(f"[{counter}] Mengirim TX: {data_test.hex().upper()}")
        
        # PENTING DI LINUX FISIK: Jangan membersihkan buffer tepat sebelum write!
        # Menghapus buffer di ttyACM0 sesaat sebelum kirim bisa merusak timing transmisi.
        
        # Kirim data
        ser.write(data_test)
        
        # Penanganan khusus Raspberry Pi 5:
        # ser.flush() di Linux terkadang selesai terlalu cepat sebelum chip selesai mengirim data secara fisik.
        # Kita beri jeda mikro agar bytes benar-benar keluar lewat pin Waveshare ke kabel.
        time.sleep(0.01) 
        
        # Cek apakah ada respons balik (RX)
        # Menunggu sebentar untuk memberikan waktu respon bagi motor DDSM115
        time.sleep(0.04) 
        
        if ser.in_waiting > 0:
            data_masuk = ser.read(ser.in_waiting)
            print(f"    -> Menerima RX ({len(data_masuk)} bytes): {data_masuk.hex().upper()}")
        else:
            print("    -> RX: (Kosong / Tidak ada respons)")
            
        print("-" * 40)
        counter += 1
        time.sleep(1) # Jeda pengulangan antar perintah

except KeyboardInterrupt:
    print("\nPengujian dihentikan oleh pengguna.")
finally:
    ser.close()
    print("Serial port ditutup.")
