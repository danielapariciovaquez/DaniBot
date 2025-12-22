import serial
import time

PORT = "/dev/ttyUSB0"
BAUD = 2_000_000

MOTOR_ID = 0x01  # CAN ID del SERVO42D

def calc_checksum_setting(pkt_without_checksum: list[int]) -> int:
    # Según el ejemplo Waveshare: checksum = sum(data[2:]) & 0xFF
    return (sum(pkt_without_checksum[2:]) & 0xFF)

ser = serial.Serial(PORT, BAUD, timeout=1)
print("Opened:", ser.portstr)

# ------------------------------------------------------------
# 1) CONFIGURACIÓN DEL CONVERTIDOR (variable length, 500k, STANDARD frame)
#    Paquete de setting: 0xAA 0x55 ... checksum
#    En el ejemplo, Frame Type: 0x02 (extended). Aquí lo ponemos a 0x01 (standard).
# ------------------------------------------------------------
set_can = [
    0xAA, 0x55,   # headers
    0x12,         # setting + variable length protocol
    0x03,         # CAN baud: 500 kbps
    0x01,         # Frame Type: 0x01 standard frame (11-bit)
    0x00, 0x00, 0x00, 0x00,   # Filter ID (4 bytes)
    0x00, 0x00, 0x00, 0x00,   # Mask ID   (4 bytes)
    0x00,         # CAN mode: normal
    0x00,         # automatic resend
    0x00, 0x00, 0x00, 0x00    # spare
]
set_can.append(calc_checksum_setting(set_can))
ser.write(bytes(set_can))
print("Sent SET+START:", bytes(set_can).hex())
time.sleep(0.2)

# ------------------------------------------------------------
# 2) Envío de trama CAN (variable length protocol)
#    IMPORTANTE: siguiendo el ejemplo, termina con 0x55 (byte de cola/framing).
#    Tipo 0xE? = TX, y bits bajos = DLC.
#
#    En el ejemplo: 0xE8 = extended + data + DLC=8.
#    Para STANDARD + data + DLC=N -> 0xC0|N, pero el ejemplo usa 0xE0 como base.
#    Para ser coherentes con el ejemplo (TX base 0xE0), usamos:
#        0xE0 con bit5=0 (standard) => 0xC0, PERO como a ti no te parpadea con 0xC?,
#        usamos base 0xE0 y ponemos ID en 2 bytes (standard) igual que la wiki suele indicar.
#
#    Si tu firmware exige estrictamente 0xC?, lo verás porque con 0xE? no enviará.
# ------------------------------------------------------------

def send_can_std(arbid_11bit: int, data: list[int]):
    dlc = len(data)
    if not (0 <= arbid_11bit <= 0x7FF):
        raise ValueError("arbid_11bit must be 0..0x7FF")
    if not (0 <= dlc <= 8):
        raise ValueError("DLC must be 0..8")

    id_l = arbid_11bit & 0xFF
    id_h = (arbid_11bit >> 8) & 0xFF

    # Tipo “como el ejemplo”: base 0xE0 y DLC en nibble bajo.
    # (bit5=1 en 0xE0 implicaría extended; aquí estamos usando ID de 2 bytes,
    #  así que si tu firmware interpreta bit5 estrictamente, prueba más abajo con 0xC0|dlc.)
    typ = 0xE0 | dlc

    pkt = [0xAA, typ, id_l, id_h] + data + [0x55]
    ser.write(bytes(pkt))
    print("TX:", bytes(pkt).hex())

# ---- ENABLE SERVO42D (CAN payload exacto para ID=01): F3 01 F5
send_can_std(MOTOR_ID, [0xF3, 0x01, 0xF5])
time.sleep(0.1)

# ---- RUN 100 RPM (CAN payload exacto para ID=01): F6 80 64 0A E5
send_can_std(MOTOR_ID, [0xF6, 0x80, 0x64, 0x0A, 0xE5])

time.sleep(0.2)
ser.close()
print("Done")
