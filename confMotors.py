import serial
import time

# =====================================================
# CONFIGURACIÓN
# =====================================================
PORT = "/dev/ttyUSB0"
BAUD = 2000000

MOTOR_ID = 0x01
WORK_CURRENT_MA = 1000   # mA

# =====================================================
# AUX
# =====================================================
def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def build_frame(can_id, data):
    """
    Encapsulado usado por tu USB-CAN:
    AA | DLC | ID_L | ID_H | data... | CRC | 55
    CRC = (ID + sum(data)) & 0xFF
    """
    crc = (can_id + sum(data)) & 0xFF
    return bytes([
        0xAA,
        0xC3,               # DLC = 3 (83 + 2 bytes)
        can_id & 0xFF,
        (can_id >> 8) & 0xFF,
        *data,
        crc,
        0x55
    ])

# =====================================================
# MAIN
# =====================================================
ser = serial.Serial(PORT, BAUD)
time.sleep(0.2)
print("Puerto abierto")

ma = clamp(WORK_CURRENT_MA, 0, 3000)
ma_lo = ma & 0xFF
ma_hi = (ma >> 8) & 0xFF

print(f"Enviando corriente {ma} mA al motor ID {MOTOR_ID:02X}")

frame = build_frame(MOTOR_ID, [0x83, ma_lo, ma_hi])
ser.write(frame)

# Espera corta solo para asegurar envío
time.sleep(0.05)

ser.close()
print("Comando enviado. Fin.")
