"""
Read MPU6050 accelerometer and gyroscope data
and calculate pitch and roll in degrees.
"""

from mpu6050 import mpu6050
import math
import time

# Initialize MPU6050 at default I2C address 0x68
sensor = mpu6050(0x68)

def get_pitch_roll(accel_data):
    """
    Calculate pitch and roll from accelerometer data.
    Formula:
        pitch = atan2(y, sqrt(x^2 + z^2))
        roll  = atan2(-x, z)
    Returns values in degrees.
    """
    ax = accel_data['x']
    ay = accel_data['y']
    az = accel_data['z']

    try:
        pitch = math.degrees(math.atan2(ay, math.sqrt(ax**2 + az**2)))
        roll = math.degrees(math.atan2(-ax, az))
    except ZeroDivisionError:
        pitch, roll = 0.0, 0.0

    return pitch, roll

try:
    while True:
        accel_data = sensor.get_accel_data()
        gyro_data = sensor.get_gyro_data()

        pitch, roll = get_pitch_roll(accel_data)

        print(f"Accel: {accel_data}")
        print(f"Gyro : {gyro_data}")
        print(f"Pitch: {pitch:.2f}°, Roll: {roll:.2f}°")
        print("-" * 40)

        time.sleep(0.5)

except KeyboardInterrupt:
    print("\nStopped by user.")
except Exception as e:
    print(f"Error: {e}")