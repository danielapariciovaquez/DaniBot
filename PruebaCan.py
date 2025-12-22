import serial
import time

ser = serial.Serial("/dev/ttyUSB0", 2000000, timeout=1)

print("Sending ENABLE frame")
ser.write(bytes([
    0xAA, 0xC3,
    0x01, 0x00,
    0xF3, 0x01, 0xF5
]))

time.sleep(0.5)

print("Sending RUN 100 RPM frame")
ser.write(bytes([
    0xAA, 0xC5,
    0x01, 0x00,
    0xF6, 0x80, 0x64, 0x0A, 0xE5
]))

ser.close()
