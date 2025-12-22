import serial
import time

PORT = "/dev/ttyUSB0"
BAUD = 2_000_000
CAN_ID = 0x01

def checksum_setting(pkt):
    return sum(pkt[2:]) & 0xFF

def send_can(ser, can_id, data):
    dlc = len(data)
    type_byte = 0xE0 | dlc   # el que YA sabes que enciende TX
    pkt = (
        bytes([0xAA, type_byte,
               can_id & 0xFF, (can_id >> 8) & 0xFF]) +
        bytes(data) +
        bytes([0x55])
    )
    ser.write(pkt)
    print("TX:", pkt.hex())

# -----------------------------
# PROGRAMA
# -----------------------------
ser = serial.Serial(PORT, BAUD, timeout=0.5)
print("Opened:", ser.portstr)

# SET + START (obligatorio)
set_can = [
    0xAA, 0x55,
    0x12,       # variable length
    0x03,       # 500 kbps
    0x01,       # standard frame
    0x00, 0x00, 0x00, 0x00,  # filter
    0x00, 0x00, 0x00, 0x00,  # mask
    0x00,       # normal mode
    0x00,       # auto resend
    0x00, 0x00, 0x00, 0x00
]
set_can.append(checksum_setting(set_can))
ser.write(bytes(set_can))
time.sleep(0.2)

# -----------------------------
# READ POSITION
# -----------------------------
# CAN payload: 31 32
send_can(ser, CAN_ID, [0x31, 0x32])

time.sleep(0.3)

rx = ser.read(256)
if rx:
    print("RX RAW:", rx.hex())
else:
    print("RX RAW: <nada>")

ser.close()
