import serial
import time

# =====================================================
# CONFIGURACIÓN USB-CAN
# =====================================================
PORT = "/dev/ttyUSB0"
BAUDRATE = 2000000

ser = serial.Serial(PORT, BAUDRATE)
print("USB-CAN abierto en:", ser.portstr)

time.sleep(0.2)

# =====================================================
# COMANDO: GIRAR A 100 RPM (F6)
# =====================================================
# ID = 0x01
# speed = 100 RPM
# acc = 2
# dir = CW

send_speed_100rpm = bytes([
    0xAA,       # Header
    0xC5,       # Variable protocol, standard frame, data frame, DLC=5
    0x01,       # CAN ID low byte
    0x00,       # CAN ID high byte
    0xF6,       # Speed mode command
    0x00,       # Dir=0 (CW) + speed high nibble
    0x64,       # Speed low byte (100 RPM)
    0x02,       # Acceleration
    0x5D,       # CRC
    0x55        # End frame
])

print("Enviando comando: 100 RPM")
ser.write(send_speed_100rpm)

# =====================================================
# LECTURA RAW DE RESPUESTA (opcional)
# =====================================================
time.sleep(0.1)
if ser.in_waiting:
    data = ser.read(ser.in_waiting)
    print("RX:", [hex(b) for b in data])

ser.close()
