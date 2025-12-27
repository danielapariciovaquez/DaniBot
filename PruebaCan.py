import serial
import time

PORT = "/dev/ttyUSB0"
BAUDRATE = 2000000
CAN_ID = 0x01

def mks_crc(can_id, data):
    return (can_id + sum(data)) & 0xFF

def build_speed_frame(can_id, rpm, acc, direction=0):
    byte1 = 0xF6
    byte2 = ((direction & 1) << 7) | ((rpm >> 8) & 0x0F)
    byte3 = rpm & 0xFF
    byte4 = acc
    data = [byte1, byte2, byte3, byte4]
    crc = mks_crc(can_id, data)
    return data + [crc]

ser = serial.Serial(PORT, BAUDRATE, timeout=1)
time.sleep(0.2)

# ---- SECUENCIA SLCAN OBLIGATORIA ----
ser.write(b"C\r")      # cerrar canal
time.sleep(0.05)

ser.write(b"S6\r")     # 500 kbps
time.sleep(0.05)

ser.write(b"O\r")      # abrir canal
time.sleep(0.05)
# ------------------------------------

data = build_speed_frame(
    CAN_ID,
    rpm=10,
    acc=1,
    direction=0
)

frame = "t{:02X}{:X}{}".format(
    CAN_ID,
    len(data),
    "".join(f"{b:02X}" for b in data)
) + "\r"

ser.write(frame.encode("ascii"))

print("Enviado:", frame.strip())
ser.close()
