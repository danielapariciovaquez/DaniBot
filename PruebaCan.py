import serial
import time

# =========================
# CONFIGURACIÓN
# =========================
PORT = "/dev/ttyUSB0"
BAUDRATE = 2_000_000
CAN_ID = 0x01

# =========================
# CLASE SIMPLE USB-CAN-A
# =========================
class USBCAN_A:
    def __init__(self, port, baudrate):
        self.ser = serial.Serial(port, baudrate, timeout=1)

    def send_can_standard(self, can_id, data):
        """
        Enviar una trama CAN estándar (11 bits) usando
        protocolo variable length de Waveshare.
        """
        dlc = len(data)
        type_byte = 0xE0 | dlc   # firmware Waveshare (el tuyo)
        packet = (
            bytes([0xAA, type_byte, can_id & 0xFF, (can_id >> 8) & 0xFF]) +
            bytes(data) +
            bytes([0x55])
        )
        self.ser.write(packet)

    def close(self):
        self.ser.close()

# =========================
# PROGRAMA PRINCIPAL
# =========================
can = USBCAN_A(PORT, BAUDRATE)

# ---- ENABLE MOTOR (ID=1)
# CAN payload: F3 01 F5
can.send_can_standard(
    CAN_ID,
    [0xF3, 0x01, 0xF5]
)

time.sleep(0.1)

# ---- RUN MOTOR A 100 RPM (CW)
# CAN payload: F6 80 64 0A E5
can.send_can_standard(
    CAN_ID,
    [0xF6, 0x80, 0x64, 0x0A, 0xE5]
)

can.close()
print("Motor 1 enviado a 100 RPM")
