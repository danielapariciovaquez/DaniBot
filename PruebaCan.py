import serial
import time

# =====================================================
# CONFIG
# =====================================================
PORT = "/dev/ttyUSB0"
SER_BAUD = 2000000

CAN_ID = 0x01
SPEED_RPM = 100
ACC = 2
DIR = 0  # 0=CW, 1=CCW

# =====================================================
# Helpers inline (sin funciones externas)
# =====================================================
def usbcan_cfg_checksum(pkt_aa55: list[int]) -> int:
    # Igual que tus ejemplos: suma desde data[2:] y &0xFF
    return sum(pkt_aa55[2:]) & 0xFF

# =====================================================
# Open
# =====================================================
ser = serial.Serial(PORT, SER_BAUD)
print("USB-CAN abierto en:", ser.portstr)
time.sleep(0.2)

# Limpieza de RX por si hay basura previa
ser.reset_input_buffer()

# =====================================================
# 1) Configurar el conversor: protocolo variable + 500 kbps
# (idéntico a tu send.py/receive.py)
# =====================================================
set_can_baudrate = [
    0xAA, 0x55,
    0x12,  # setting, variable protocol
    0x03,  # 500 kbps
    0x02,  # (tal como en tu ejemplo) "Extended Frame" en config del conversor
    0x00, 0x00, 0x00, 0x00,  # Filter
    0x00, 0x00, 0x00, 0x00,  # Mask
    0x00,  # normal mode
    0x00,  # auto resend
    0x00, 0x00, 0x00, 0x00   # spare
]
set_can_baudrate.append(usbcan_cfg_checksum(set_can_baudrate))

ser.write(bytes(set_can_baudrate))
print("Config 500 kbps enviada al conversor.")
time.sleep(0.2)

# =====================================================
# 2) Construir comando F6 a 100 rpm
# DATA = [F6, byte2, byte3, ACC]
# byte2 = (DIR<<7) | speed[11:8]
# byte3 = speed[7:0]
# =====================================================
speed = SPEED_RPM & 0x0FFF
byte2 = ((DIR & 1) << 7) | ((speed >> 8) & 0x0F)
byte3 = speed & 0xFF

data = [0xF6, byte2, byte3, ACC]
dlc = len(data)

# CRC SERVO42D: (CAN_ID + sum(DATA)) & 0xFF
crc = (CAN_ID + sum(data)) & 0xFF

# Frame control del protocolo variable:
# 0xC0 = variable + standard + data frame, OR DLC
frame_ctrl = 0xC0 | dlc

frame = bytes([0xAA, frame_ctrl, CAN_ID & 0xFF, (CAN_ID >> 8) & 0xFF] + data + [crc, 0x55])

print(f"Enviando F6: speed={SPEED_RPM}rpm acc={ACC} dir={'CCW' if DIR else 'CW'}  CRC=0x{crc:02X}")
ser.write(frame)

# =====================================================
# 3) Leer respuesta cruda (si CanRSP está Enable)
# =====================================================
time.sleep(0.1)
if ser.in_waiting:
    rx = ser.read(ser.in_waiting)
    print("RX:", [hex(b) for b in rx])
else:
    print("Sin RX (posible CanRSP=Disable o el conversor/motor no responde).")

ser.close()
