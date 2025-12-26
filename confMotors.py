import serial
import time

# =====================================================
# CONFIGURACIÓN
# =====================================================
PORT = "/dev/ttyUSB0"
BAUD = 2000000

MOTORS = [0x01, 0x02, 0x03, 0x04]

WORK_CURRENT_MA = 1600   # mA (SERVO42D: 0–3000)

# =====================================================
# AUX
# =====================================================
def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def build_frame(can_id, data):
    crc = (can_id + sum(data)) & 0xFF
    return bytes([
        0xAA,
        0xC3,               # DLC = 3 (83 + 2 bytes de corriente)
        can_id & 0xFF,
        (can_id >> 8) & 0xFF,
        *data,
        crc,
        0x55
    ])

def read_raw(ser, timeout=0.1):
    t0 = time.time()
    buf = bytearray()
    while time.time() - t0 < timeout:
        if ser.in_waiting:
            buf += ser.read(ser.in_waiting)
        time.sleep(0.001)
    return bytes(buf)

# =====================================================
# DECODIFICACIÓN RESPUESTA 0x83
# =====================================================
def parse_response_83(raw):
    """
    Busca tramas que contengan:
    83 status
    """
    responses = []
    for i in range(len(raw) - 1):
        if raw[i] == 0x83 and raw[i + 1] in (0x00, 0x01):
            responses.append(raw[i + 1])
    return responses

# =====================================================
# MAIN
# =====================================================
ser = serial.Serial(PORT, BAUD)
time.sleep(0.2)
print("Puerto abierto")

ma = clamp(WORK_CURRENT_MA, 0, 3000)
ma_lo = ma & 0xFF
ma_hi = (ma >> 8) & 0xFF

print(f"\nEnviando corriente = {ma} mA\n")

for mid in MOTORS:
    print(f"Motor ID {mid:02X} -> ", end="")

    frame = build_frame(mid, [0x83, ma_lo, ma_hi])
    ser.reset_input_buffer()
    ser.write(frame)

    raw = read_raw(ser)

    if not raw:
        print("SIN RESPUESTA")
        continue

    statuses = parse_response_83(raw)

    if not statuses:
        print(f"Respuesta no reconocida: {raw.hex(' ')}")
    else:
        for st in statuses:
            if st == 1:
                print("OK (corriente aceptada)")
            else:
                print("FAIL")

ser.close()
print("\nFin")
