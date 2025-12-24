import serial
import time

# =====================================================
# CONFIGURACIÓN
# =====================================================
PORT = "/dev/ttyUSB0"
BAUDRATE = 2000000
CAN_ID = 0x01        # SERVO42D ID
SPEED_RPM = 100
ACC = 2              # aceleración válida
DIR = 0              # 0 = CW, 1 = CCW

# =====================================================
# ABRIR PUERTO USB-CAN
# =====================================================
ser = serial.Serial(PORT, BAUDRATE)
print(f"USB-CAN abierto en {PORT}")
time.sleep(0.2)

# =====================================================
# CONSTRUIR DATA CAN (F6 – SPEED MODE)
# =====================================================
# Velocidad en 12 bits
# speed = 100 = 0x064
# byte2:
#   bit7 = dir
#   bits3-0 = speed[11:8]
# byte3:
#   speed[7:0]
#
speed = SPEED_RPM & 0x0FFF

byte2 = ((DIR & 0x01) << 7) | ((speed >> 8) & 0x0F)
byte3 = speed & 0xFF

data = bytes([
    0xF6,   # comando speed mode
    byte2,
    byte3,
    ACC
])

dlc = len(data)

# =====================================================
# CÁLCULO CRC SERVO42D (CLARO Y EXPLÍCITO)
# =====================================================
# CRC = (CAN_ID + sum(DATA_BYTES)) & 0xFF
crc = (CAN_ID + sum(data)) & 0xFF

# =====================================================
# FRAME CONTROL (PROTOCOLO VARIABLE)
# bits7–6 = 11  -> protocolo variable
# bit5    = 0   -> standard frame
# bit4    = 0   -> data frame
# bits3–0 = DLC
# =====================================================
frame_ctrl = 0xC0 | dlc

# =====================================================
# CONSTRUIR TRAMA USB-CAN COMPLETA
# =====================================================
frame = bytes([
    0xAA,                   # header
    frame_ctrl,
    CAN_ID & 0xFF,          # CAN ID low
    (CAN_ID >> 8) & 0xFF,   # CAN ID high
]) + data + bytes([
    crc,
    0x55                    # end frame
])

# =====================================================
# ENVÍO
# =====================================================
print("Enviando comando F6 (100 RPM)...")
ser.write(frame)

# =====================================================
# LECTURA RAW DE RESPUESTA (OPCIONAL)
# =====================================================
time.sleep(0.1)
if ser.in_waiting:
    rx = ser.read(ser.in_waiting)
    print("RX:", [hex(b) for b in rx])

# =====================================================
# CIERRE
# =====================================================
ser.close()
print("Puerto cerrado")
