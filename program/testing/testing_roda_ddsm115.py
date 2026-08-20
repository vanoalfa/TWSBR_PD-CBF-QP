"""
test_both_wheels.py

Script untuk testing dasar 2 roda DDSM115 (kiri & kanan) pada robot TWSBR.
Motor akan diputar perlahan maju, lalu mundur, lalu berhenti.

CARA PAKAI:
1. Sesuaikan PORT_LEFT dan PORT_RIGHT di bawah sesuai hasil `ls /dev/ttyACM*`
   (cek mana port untuk roda kiri, mana untuk roda kanan dengan cabut-colok satu per satu)
2. Jalankan: python test_both_wheels.py
3. Tekan Ctrl+C kapan saja untuk menghentikan motor dengan aman
"""

import time
import sys
from ddsm115 import ddsm115

# ==============================
# KONFIGURASI - SESUAIKAN INI
# ==============================
PORT_LEFT = "/dev/ttyACM0"   # ganti sesuai port roda kiri
PORT_RIGHT = "/dev/ttyACM1"  # ganti sesuai port roda kanan

MOTOR_ID = 1        # ID motor (default pabrik biasanya 1, kecuali sudah pernah di-set_id)
TEST_RPM = 30        # kecepatan test, mulai KECIL dulu demi keamanan
MOVE_DURATION = 2.0  # detik, lama tiap fase gerakan


def print_step(text):
    print(f"\n=== {text} ===")


def safe_stop(drive_left, drive_right):
    """Hentikan kedua motor dengan aman."""
    try:
        if drive_left is not None:
            drive_left.send_rpm(MOTOR_ID, 0)
        if drive_right is not None:
            drive_right.send_rpm(MOTOR_ID, 0)
        time.sleep(0.1)
        print_step("Motor dihentikan (RPM = 0)")
    except Exception as e:
        print(f"Peringatan saat menghentikan motor: {e}")


def main():
    drive_left = None
    drive_right = None

    try:
        # ------------------------------------------------
        # 1. Buka koneksi ke kedua motor
        # ------------------------------------------------
        print_step(f"Menghubungkan ke roda kiri ({PORT_LEFT})")
        drive_left = ddsm115.MotorControl(device=PORT_LEFT)

        print_step(f"Menghubungkan ke roda kanan ({PORT_RIGHT})")
        drive_right = ddsm115.MotorControl(device=PORT_RIGHT)

        # ------------------------------------------------
        # 2. Set mode velocity (mode 2) untuk kedua motor
        # ------------------------------------------------
        print_step("Mengatur mode velocity pada kedua motor")
        drive_left.set_drive_mode(MOTOR_ID, 2)
        drive_right.set_drive_mode(MOTOR_ID, 2)
        time.sleep(0.2)

        # ------------------------------------------------
        # 3. Test maju pelan
        # ------------------------------------------------
        print_step(f"Maju pelan (RPM={TEST_RPM}) selama {MOVE_DURATION}s")
        drive_left.send_rpm(MOTOR_ID, TEST_RPM)
        drive_right.send_rpm(MOTOR_ID, TEST_RPM)
        time.sleep(MOVE_DURATION)

        # ------------------------------------------------
        # 4. Berhenti sejenak
        # ------------------------------------------------
        safe_stop(drive_left, drive_right)
        time.sleep(1.0)

        # ------------------------------------------------
        # 5. Test mundur pelan
        # ------------------------------------------------
        print_step(f"Mundur pelan (RPM=-{TEST_RPM}) selama {MOVE_DURATION}s")
        drive_left.send_rpm(MOTOR_ID, -TEST_RPM)
        drive_right.send_rpm(MOTOR_ID, -TEST_RPM)
        time.sleep(MOVE_DURATION)

        # ------------------------------------------------
        # 6. Berhenti final
        # ------------------------------------------------
        safe_stop(drive_left, drive_right)

        print_step("Test selesai. Kedua roda berhasil digerakkan.")

    except KeyboardInterrupt:
        print_step("Dihentikan manual (Ctrl+C)")
        safe_stop(drive_left, drive_right)

    except Exception as e:
        print(f"\nERROR: {e}")
        safe_stop(drive_left, drive_right)
        sys.exit(1)

    finally:
        # Pastikan koneksi serial ditutup rapi
        try:
            if drive_left is not None:
                drive_left.close()
            if drive_right is not None:
                drive_right.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()