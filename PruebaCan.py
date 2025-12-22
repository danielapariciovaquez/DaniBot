import can
import time

CAN_ID = 0x01
BITRATE = 500000

# =========================
# Abrir bus CAN (SocketCAN)
# =========================
bus = can.interface.Bus(
    channel='can0',
    interface='socketcan'
)

# =========================
# Construir trama READ POSITION
# =========================
cmd = 0x31
crc = (CAN_ID + cmd) & 0xFF

msg = can.Message(
    arbitration_id=CAN_ID,
    is_extended_id=False,
    data=[cmd, crc]
)

# =========================
# Enviar trama
# =========================
print("Sending position request")
bus.send(msg)

# =========================
# Esperar respuesta
# =========================
response = bus.recv(timeout=1.0)

if response is None:
    print("No response from motor")
else:
    data = response.data
    print("Raw RX data:", data.hex())

    if data[0] != 0x31:
        print("Unexpected response")
    else:
        position = data[1] | (data[2] << 8) | (data[3] << 16)
        print("Encoder position:", position)
