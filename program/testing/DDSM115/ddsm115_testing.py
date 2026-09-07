import serial
import time

PORT = "/dev/ttyACM0"
BAUD = 115200

# GANTI ini dengan frame hex yang sebelumnya SUDAH terbukti berhasil
frame = bytes.fromhex("01 64 00 00 00 00 00 FF 00 D1")

ser = serial.Serial(PORT, BAUD, timeout=0.2)
time.sleep(0.1)

ser.reset_input_buffer()
ser.reset_output_buffer()

print("Opened:", ser.name)
print("TX:", frame.hex(" "))

ser.write(frame)
ser.flush()

time.sleep(0.1)
resp = ser.read(64)

if resp:
    print("RX:", resp.hex(" "))
else:
    print("RX: <no response>")

ser.close()
