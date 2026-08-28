#Program Kalibrasi Sudut MPU6050 (Sumbu Y / Pitch)
#
#Koneksi:
#MPU6050 - Raspberry pi
#VCC - 5V  (2 or 4 Board)
#GND - GND (6 - Board)
#SCL - SCL (5 - Board)
#SDA - SDA (3 - Board)
#
#Cara pakai (jalankan langsung di terminal, bukan lewat editor):
#   - Tekan 'k' / 'K' -> kalibrasi: posisi sensor saat ini ditetapkan sebagai 0 derajat
#   - Tekan 'm' / 'M' -> mulai menampilkan sudut (hanya bisa jika sudah dikalibrasi)
#   - Tekan 'q' / 'Q' -> keluar dari program
#
#Setelah kalibrasi & mulai:
#   - Bergerak ke arah +  -> sudut naik dari 0 menuju +180
#   - Bergerak ke arah -  -> sudut turun dari 0 menuju -180

from Kalman import KalmanAngle
import smbus            #import SMBus module of I2C
import time
import math
import sys
import select
import tty
import termios

kalmanY = KalmanAngle()
radToDeg = 57.2957786

#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_ZOUT_H = 0x3F
GYRO_YOUT_H  = 0x45


def MPU_Init():
	#write to sample rate register
	bus.write_byte_data(DeviceAddress, SMPLRT_DIV, 7)

	#Write to power management register
	bus.write_byte_data(DeviceAddress, PWR_MGMT_1, 1)

	#Write to Configuration register (DLPF, mengurangi noise akibat getaran)
	bus.write_byte_data(DeviceAddress, CONFIG, int('0000110', 2))

	#Write to Gyro configuration register
	bus.write_byte_data(DeviceAddress, GYRO_CONFIG, 24)

	#Write to interrupt enable register
	bus.write_byte_data(DeviceAddress, INT_ENABLE, 1)


def read_raw_data(addr):
	#Accelero and Gyro value are 16-bit
	high = bus.read_byte_data(DeviceAddress, addr)
	low = bus.read_byte_data(DeviceAddress, addr + 1)

	#concatenate higher and lower value
	value = ((high << 8) | low)

	#to get signed value from mpu6050
	if value > 32768:
		value = value - 65536
	return value


def get_pitch():
	#Rumus rentang penuh (-180 s/d +180 derajat), dibutuhkan agar sudut bisa 0 -> +-180
	accX = read_raw_data(ACCEL_XOUT_H)
	accZ = read_raw_data(ACCEL_ZOUT_H)
	return math.atan2(-accX, accZ) * radToDeg


def wrap180(angle):
	#Menjaga hasil (sudut - offset) tetap berada di rentang -180 s/d 180
	while angle > 180:
		angle -= 360
	while angle < -180:
		angle += 360
	return angle


#---- Setup keyboard non-blocking (baca tombol tanpa perlu tekan Enter) ----
old_terminal_settings = termios.tcgetattr(sys.stdin)
tty.setcbreak(sys.stdin.fileno())


def get_key_if_pressed():
	if select.select([sys.stdin], [], [], 0)[0]:
		return sys.stdin.read(1)
	return None


#---- Setup sensor ----
bus = smbus.SMBus(1)    # or bus = smbus.SMBus(0) for older version boards
DeviceAddress = 0x68     # MPU6050 device address

MPU_Init()
time.sleep(1)

pitch = get_pitch()
kalmanY.setAngle(pitch)
kalAngleY = pitch

offset = 0.0
calibrated = False
running = False

timer = time.time()
flag = 0

print("Program siap.")
print("Tekan 'K' untuk kalibrasi (menetapkan posisi sekarang sebagai 0 derajat).")

try:
	while True:
		if flag > 100:  # Problem with the connection
			print("Ada masalah dengan koneksi sensor")
			flag = 0
			continue
		try:
			gyroY = read_raw_data(GYRO_YOUT_H)

			dt = time.time() - timer
			timer = time.time()

			pitch = get_pitch()
			gyroYRate = gyroY / 131.0

			#Tangani lompatan nilai di sekitar +-180 derajat
			if (pitch < -90 and kalAngleY > 90) or (pitch > 90 and kalAngleY < -90):
				kalmanY.setAngle(pitch)
				kalAngleY = pitch
			else:
				kalAngleY = kalmanY.getAngle(pitch, gyroYRate, dt)

			#Cek input keyboard
			key = get_key_if_pressed()
			if key:
				key = key.lower()
				if key == 'k':
					offset = kalAngleY
					calibrated = True
					print(f"\n[KALIBRASI] Posisi sekarang ditetapkan sebagai 0 derajat.")
					print("Tekan 'M' untuk mulai membaca sudut.")
				elif key == 'm':
					if calibrated:
						running = True
						print("\n[MULAI] IMU mulai membaca sudut...")
					else:
						print("\n[PERINGATAN] Lakukan kalibrasi dulu dengan menekan 'K'.")
				elif key == 'q':
					print("\nKeluar dari program.")
					break

			if running:
				angle = wrap180(kalAngleY - offset)
				print(f"Sudut: {angle:7.2f}", end='\r')

			time.sleep(0.02)

		except Exception as exc:
			flag += 1

finally:
	#Kembalikan setting terminal seperti semula
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_terminal_settings)