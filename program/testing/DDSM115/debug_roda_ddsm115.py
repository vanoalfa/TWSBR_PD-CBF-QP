"""
debug_wheel.py

Script diagnosa untuk 1 roda DDSM115 pada satu waktu.
Membaca ID, mode, error code, dan feedback RPM aktual dari encoder motor
untuk memastikan di mana letak masalahnya:
  - Komunikasi (serial/RS485)
  - Daya motor (power supply 12-24V)
  - Command/mode yang salah
  - Mekanis (roda macet/terkunci)

CARA PAKAI:
    python debug_wheel.py /dev/ttyACM0
    python debug_wheel.py /dev/ttyACM1
(jalankan satu per satu untuk tiap port, supaya jelas mana yang bermasalah)
"""

import sys
import time
from ddsm115 import ddsm115

MOTOR_ID = 1
TEST_RPM = 30


def main():
    if len(sys.argv) < 2:
        print("Pakai: python debug_wheel.py <port>")
        print("Contoh: python debug_wheel.py /dev/ttyACM0")
        sys.exit(1)

    port = sys.argv[1]
    print(f"\n{'='*50}")
    print(f"DEBUG PORT: {port}")
    print(f"{'='*50}\n")

    # ------------------------------------------------
    # STEP 1: Buka koneksi serial
    # ------------------------------------------------
    try:
        drive = ddsm115.MotorControl(device=port)
        print(f"[OK] Berhasil membuka koneksi serial ke {port}")
    except Exception as e:
        print(f"[GAGAL] Tidak bisa membuka {port}: {e}")
        print("-> Cek apakah port benar, atau device sedang dipakai proses lain")
        sys.exit(1)

    # ------------------------------------------------
    # STEP 2: Cek apakah motor merespons (query ID)
    # ------------------------------------------------
    print("\n--- Test 1: Cek komunikasi dengan motor (query ID) ---")
    try:
        drive.get_motor_id()
        print("[OK] Motor merespons query ID (lihat ID/Mode/Error di atas)")
        print("     -> Kalau nilai di atas 0 semua/aneh, kemungkinan TIDAK ada balasan asli")
        print("        dari motor (motor mungkin tidak dapat daya, hanya timeout)")
    except Exception as e:
        print(f"[GAGAL] Error saat query ID: {e}")

    # ------------------------------------------------
    # STEP 3: Set mode velocity
    # ------------------------------------------------
    print("\n--- Test 2: Set mode velocity ---")
    try:
        drive.set_drive_mode(MOTOR_ID, 2)
        time.sleep(0.2)
        print("[OK] Perintah set drive mode terkirim")
    except Exception as e:
        print(f"[GAGAL] Error saat set drive mode: {e}")

    # ------------------------------------------------
    # STEP 4: Kirim RPM sambil baca feedback encoder
    # ------------------------------------------------
    print(f"\n--- Test 3: Kirim RPM={TEST_RPM} sambil baca feedback encoder selama 5 detik ---")
    print("Perhatikan kolom 'fb_rpm' di bawah:")
    print("  - Kalau fb_rpm tetap 0 terus & tidak ada error -> kemungkinan besar MASALAH DAYA")
    print("    (motor tidak tersambung ke power supply 12-24V, atau power belum menyala)")
    print("  - Kalau fb_rpm berubah sesuai TEST_RPM -> motor SUDAH BERPUTAR (roda mungkin macet")
    print("    secara mekanis, atau memang sudah OK dan geraknya kamu tidak sadari)")
    print("  - Kalau ada error code muncul -> baca artinya di bagian error di bawah\n")

    try:
        start = time.time()
        drive.send_rpm(MOTOR_ID, TEST_RPM)

        while time.time() - start < 5.0:
            fb_rpm, fb_cur = drive.get_motor_feedback(MOTOR_ID)
            elapsed = time.time() - start
            print(f"  t={elapsed:4.1f}s | fb_rpm={fb_rpm:6d} | fb_cur={fb_cur:6.2f}A")
            time.sleep(0.3)

    except Exception as e:
        print(f"[GAGAL] Error saat kirim RPM / baca feedback: {e}")

    finally:
        # ------------------------------------------------
        # STEP 5: Hentikan motor & tutup koneksi
        # ------------------------------------------------
        try:
            drive.send_rpm(MOTOR_ID, 0)
            time.sleep(0.1)
            print("\n[OK] Motor dihentikan (RPM=0)")
        except Exception:
            pass
        drive.close()
        print(f"\n{'='*50}")
        print("Debug selesai")
        print(f"{'='*50}\n")


if __name__ == "__main__":
    main()