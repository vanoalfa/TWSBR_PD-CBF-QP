#Connections
#MPU6050 - Raspberry pi
#VCC - 5V  (2 or 4 Board)
#GND - GND (6 - Board)
#SCL - SCL (5 - Board)
#SDA - SDA (3 - Board)

from Kalman import KalmanAngle
import smbus            #import SMBus module of I2C
import time
import math

kalmanY = KalmanAngle()

RestrictPitch = True     #Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
radToDeg = 57.2957786
kalAngleY = 0

#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_YOUT_H  = 0x45


#Inisialisasi MPU6050
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


bus = smbus.SMBus(1)    # or bus = smbus.SMBus(0) for older version boards
DeviceAddress = 0x68     # MPU6050 device address

MPU_Init()
time.sleep(1)

#Nilai accX tetap dibutuhkan karena rumus pitch bergantung pada accX, accY, accZ
accX = read_raw_data(ACCEL_XOUT_H)
accY = read_raw_data(ACCEL_YOUT_H)
accZ = read_raw_data(ACCEL_ZOUT_H)

if RestrictPitch:
	pitch = math.atan(-accX / math.sqrt((accY ** 2) + (accZ ** 2))) * radToDeg
else:
	pitch = math.atan2(-accX, accZ) * radToDeg

kalmanY.setAngle(pitch)
gyroYAngle = pitch
compAngleY = pitch

timer = time.time()
flag = 0

while True:
	if flag > 100:  # Problem with the connection
		print("There is a problem with the connection")
		flag = 0
		continue
	try:
		#Read Accelerometer raw value
		accX = read_raw_data(ACCEL_XOUT_H)
		accY = read_raw_data(ACCEL_YOUT_H)
		accZ = read_raw_data(ACCEL_ZOUT_H)

		#Read Gyroscope raw value (hanya sumbu Y)
		gyroY = read_raw_data(GYRO_YOUT_H)

		dt = time.time() - timer
		timer = time.time()

		if RestrictPitch:
			pitch = math.atan(-accX / math.sqrt((accY ** 2) + (accZ ** 2))) * radToDeg
		else:
			pitch = math.atan2(-accX, accZ) * radToDeg

		gyroYRate = gyroY / 131

		kalAngleY = kalmanY.getAngle(pitch, gyroYRate, dt)

		#angle = (rate of change of angle) * change in time
		gyroYAngle = gyroYAngle * dt

		#compAngle = constant * (old_compAngle + angle_obtained_from_gyro) + constant * angle_obtained_from accelerometer
		compAngleY = 0.93 * (compAngleY + gyroYRate * dt) + 0.07 * pitch

		if (gyroYAngle < -180) or (gyroYAngle > 180):
			gyroYAngle = kalAngleY

		print(f"Angle Y: {kalAngleY:.2f}")
		time.sleep(1)

	except Exception as exc:
		flag += 1