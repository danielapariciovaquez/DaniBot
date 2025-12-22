import serial
import time

# ==========================================================
# CONFIGURACIÓN
# ==========================================================
PORT = "/dev/ttyUSB0"
BAUD = 2_000_000
CAN_ID = 0x01   # Motor ID

# ==========================================================
# FUNCIONES AUXILIARES
# ==========================================================
def checksum_setting(pkt):
    """
    Checksum SOLO para el paquete SET del USB-CAN-A
    Regla Waveshare: sum(bytes desde índice 2) & 0xFF
    """
    return sum(pkt[2:]) & 0xFF


def send_can(ser, can_id, data):
    """
    Enviar una trama CAN estándar (11 bits) usando
    protocolo variable length del USB-CAN-A.
    """
    dlc = len(data)
    type_byte = 0xE0 | dlc   # Firmware Waveshare (TX)
    pkt = (
        bytes([
            0xAA,
            type_byte,
            can_id & 0xFF,
            (can_id >> 8) & 0xFF
        ])
        + bytes(data)
        + bytes([0x55])
    )
    ser.write(pkt)
    print("TX:", pkt.hex())


# ==========================================================
# PROGRAMA PRINCIPAL
# ==========================================================
ser = serial.Serial(PORT, BAUD, timeout=0.2)
print("Opened:", ser.portstr)

# ----------------------------------------------------------
# 1) CONFIGURAR EL CONVERTIDOR (OBLIGATORIO)
#    Variable length + 500 kbps + CAN estándar
# ----------------------------------------------------------
set_can = [
    0xAA, 0x55,   # Headers
    0x12,         # Setting + variable length protocol
    0x03,         # CAN baudrate = 500 kbps
    0x01,         # Frame type = STANDARD (11-bit)
    0x00, 0x00, 0x00, 0x00,  # Filter ID (accept all)
    0x00, 0x00, 0x00, 0x00,  # Mask ID   (accept all)
    0x00,         # CAN mode = normal
    0x00,         # auto resend
    0x00, 0x00, 0x00, 0x00  # spare
]

set_can.append(checksum_setting(set_can))
ser.write(bytes(set_can))
print("SET+START sent")
time.sleep(0.2)

# ----------------------------------------------------------
# 2) ENABLE MOTOR
#    CAN payload: F3 01 F5
# ----------------------------------------------------------
send_can(ser, CAN_ID, [0xF3, 0x01, 0xF5])
time.sleep(0.1)

# ----------------------------------------------------------
# 3) RUN MOTOR A 100 RPM
#    CAN payload: F6 80 64 0A E5
# ----------------------------------------------------------
send_can(ser, CAN_ID, [0xF6, 0x80, 0x64, 0x0A, 0xE5])
time.sleep(0.2)

# ----------------------------------------------------------
# 4) QUERY MOTOR STATUS (DEBE RESPONDER SI CanRSP=Enable)
#    CAN payload: F1 F2
# ----------------------------------------------------------
send_can(ser, CAN_ID, [0xF1, 0xF2])
time.sleep(0.2)

# ----------------------------------------------------------
# 5) LEER RX CRUDO
# ----------------------------------------------------------
rx = ser.read(256)
if rx:
    print("RX RAW:", rx.hex())
else:
    print("RX RAW: <nada recibido>")

ser.close()
print("Done")
