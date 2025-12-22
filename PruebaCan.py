import serial
import time

PORT = "/dev/ttyUSB0"
BAUD = 2_000_000
CAN_ID = 0x01

def checksum_setting(pkt):
    # Waveshare: checksum = sum(bytes desde índice 2) & 0xFF
    return sum(pkt[2:]) & 0xFF

ser = serial.Serial(PORT, BAUD, timeout=1)
print("Opened:", ser.portstr)

# ==========================================================
# 1) CONFIGURAR CONVERTIDOR (OBLIGATORIO)
#    Variable length + 500 kbps + STANDARD frame
# ==========================================================
set_can = [
    0xAA, 0x55,   # headers
    0x12,         # setting + variable length protocol
    0x03,         # CAN baud = 500 kbps
    0x01,         # 0x01 = standard frame (11-bit)
    0x00, 0x00, 0x00, 0x00,  # Filter ID
    0x00, 0x00, 0x00, 0x00,  # Mask ID
    0x00,         # normal mode
    0x00,         # auto resend
    0x00, 0x00, 0x00, 0x00  # spare
]

set_can.append(checksum_setting(set_can))
ser.write(bytes(set_can))
print("SET+START sent")
time.sleep(0.2)

# ==========================================================
# 2) FUNCIÓN PARA ENVIAR CAN ESTÁNDAR
# ==========================================================
def send_can(can_id, data):
    dlc = len(data)
    type_byte = 0xE0 | dlc   # firmware Waveshare (TX + variable length)
    pkt = (
        bytes([0xAA, type_byte,
               can_id & 0xFF, (can_id >> 8) & 0xFF]) +
        bytes(data) +
        bytes([0x55])        # cola obligatoria
    )
    ser.write(pkt)
    print("TX:", pkt.hex())

# ==========================================================
# 3) ENABLE MOTOR (ID = 1)
# ==========================================================
# CAN payload SERVO42D: F3 01 F5
send_can(CAN_ID, [0xF3, 0x01, 0xF5])
time.sleep(0.1)

# ==========================================================
# 4) RUN MOTOR A 100 RPM
# ==========================================================
# CAN payload: F6 80 64 0A E5
send_can(CAN_ID, [0xF6, 0x80, 0x64, 0x0A, 0xE5])

ser.close()
print("Done")
