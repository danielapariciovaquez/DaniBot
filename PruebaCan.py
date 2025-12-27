import serial
import time

# ===============================
# Configuración
# ===============================
PORT = "/dev/ttyUSB0"
BAUDRATE = 115200   # típico en CANable / slcan
CAN_ID = 0x01

# ===============================
# CRC MKS
# CRC = (ID + data bytes) & 0xFF
# ===============================
def mks_crc(can_id, data):
    return (can_id + sum(data)) & 0xFF

# ===============================
# Comando F6: speed mode
# ===============================
def build_speed_frame(can_id, rpm, acc, direction=0):
    if not (0 <= rpm <= 3000):
        raise ValueError("RPM fuera de rango")
    if not (0 <= acc <= 255):
        raise ValueError("ACC fuera de rango")

    byte1 = 0xF6
    byte2 = ((direction & 0x01) << 7) | ((rpm >> 8) & 0x0F)
    byte3 = rpm & 0xFF
    byte4 = acc

    data = [byte1, byte2, byte3, byte4]
    crc = mks_crc(can_id, data)

    return data + [crc]

# ===============================
# Envío por SLCAN
# ===============================
ser = serial.Serial(PORT, BAUDRATE, timeout=1)
time.sleep(0.2)

data = build_speed_frame(
    can_id=CAN_ID,
    rpm=10,
    acc=1,
    direction=0  # CW
)

# Construcción SLCAN
slcan_frame = "t{:02X}{:01X}{}".format(
    CAN_ID,
    len(data),
    "".join(f"{b:02X}" for b in data)
) + "\r"

ser.write(slcan_frame.encode("ascii"))

print("Trama enviada:", slcan_frame.strip())

ser.close()
