import serial
import time

# =====================================================
# CONFIGURACIÓN USB-CAN
# =====================================================
PORT = "/dev/ttyUSB0"
BAUDRATE = 2000000

# =====================================================
# PARÁMETROS DE CONTROL
# =====================================================
CAN_ID = 0x04      # Número de motor (1–2047)
SPEED_RPM = 1000    # Velocidad en RPM (0–3000)
ACC = 2            # Aceleración (0–255)
DIR = 0            # 0 = CW, 1 = CCW

# =====================================================
# ABRIR PUERTO
# =====================================================
ser = serial.Serial(PORT, BAUDRATE)
print("USB-CAN abierto en:", ser.portstr)
time.sleep(0.2)

# =====================================================
# CONSTRUIR COMANDO F6 (SPEED MODE)
# =====================================================
# Velocidad en 12 bits
speed = SPEED_RPM & 0x0FFF

byte2 = ((DIR & 0x01) << 7) | ((speed >> 8) & 0x0F)
byte3 = speed & 0xFF

# Datos CAN
data = [
    0xF6,   # Comando speed mode
    byte2,
    byte3,
    ACC
]

# =====================================================
# CÁLCULO CRC SERVO42D
# CRC = (CAN_ID + sum(DATA)) & 0xFF
# =====================================================
crc = (CAN_ID + sum(data)) & 0xFF

# =====================================================
# TRAMA USB-CAN
# FrameControl = 0xC5 (DLC=5, protocolo variable, standard frame)
# =====================================================
frame = bytes([
    0xAA,
    0xC5,                  # <-- FIJO, como en el código que funciona
    CAN_ID & 0xFF,         # CAN ID low
    (CAN_ID >> 8) & 0xFF,  # CAN ID high
] + data + [
    crc,
    0x55
])

# =====================================================
# ENVÍO
# =====================================================
print(f"Motor {CAN_ID} → {SPEED_RPM} RPM")
ser.write(frame)

# =====================================================
# LECTURA RX (OPCIONAL)
# =====================================================
time.sleep(0.1)
if ser.in_waiting:
    rx = ser.read(ser.in_waiting)
    print("RX:", [hex(b) for b in rx])

ser.close()
