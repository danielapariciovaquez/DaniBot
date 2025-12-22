import serial
import time

PORT = "/dev/ttyUSB0"
BAUD = 2_000_000

ser = serial.Serial(PORT, BAUD, timeout=1)
print("Opened:", ser.portstr)

# -----------------------------
# SET + START (OBLIGATORIO)
# -----------------------------
set_can = [
    0xAA, 0x55,
    0x12,       # variable length
    0x03,       # 500 kbps
    0x01,       # standard frame
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00,
    0x00,
    0x00, 0x00, 0x00, 0x00
]

checksum = sum(set_can[2:]) & 0xFF
set_can.append(checksum)

ser.write(bytes(set_can))
print("SET+START sent")
time.sleep(0.5)

# -----------------------------
# DUMMY CAN FRAME (TX)
# -----------------------------
pkt = bytes([
    0xAA,
    0xE1,       # TX, DLC=1
    0x01, 0x00, # CAN ID = 1
    0x00,       # DATA dummy
    0x55
])

ser.write(pkt)
print("DUMMY TX sent")

time.sleep(1)
ser.close()
print("Done")
