import serial
import time

# =====================================================
# CONFIGURACIÓN
# =====================================================
PORT = "/dev/ttyUSB0"
BAUD = 2000000

MOTORS = [0x01, 0x02, 0x03, 0x04]

TEST_RPM = 100     # velocidad baja y segura
ACC = 50

# =====================================================
# AUX
# =====================================================
def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def build_frame(dlc, can_id, data):
    crc = (can_id + sum(data)) & 0xFF
    return bytes([
        0xAA,
        dlc & 0xFF,
        can_id & 0xFF,
        (can_id >> 8) & 0xFF,
        *data,
        crc,
        0x55
    ])

def read_raw(ser, timeout=0.2):
    t0 = time.time()
    buf = bytearray()
    while time.time() - t0 < timeout:
        if ser.in_waiting:
            buf += ser.read(ser.in_waiting)
        time.sleep(0.001)
    return bytes(buf)

# =====================================================
# COMANDOS
# =====================================================
def send_enable(ser, can_id, enable):
    ser.write(build_frame(0xC2, can_id, [0xF3, 0x01 if enable else 0x00]))

def send_speed(ser, can_id, rpm):
    direction = 0
    if rpm < 0:
        direction = 1
        rpm = -rpm

    rpm = clamp(int(rpm), 0, 3000)
    speed = rpm & 0x0FFF

    b2 = (direction << 7) | ((speed >> 8) & 0x0F)
    b3 = speed & 0xFF

    ser.write(build_frame(0xC5, can_id, [0xF6, b2, b3, ACC]))

# =====================================================
# MAIN
# =====================================================
ser = serial.Serial(PORT, BAUD, timeout=0)
time.sleep(0.2)
print("Puerto abierto\n")

# --- ENABLE ---
print("Enviando ENABLE (F3 = 1)")
for mid in MOTORS:
    ser.write(build_frame(0xC2, mid, [0xF3, 0x01]))
time.sleep(0.1)

raw = read_raw(ser)
if raw:
    print("RX tras ENABLE:", raw.hex(" "))
else:
    print("RX tras ENABLE: <nada>")

# --- MOVER ---
print("\nEnviando F6 (mover motor)")
for mid in MOTORS:
    send_speed(ser, mid, TEST_RPM)

time.sleep(0.1)

raw = read_raw(ser)
if raw:
    print("RX tras F6:", raw.hex(" "))
else:
    print("RX tras F6: <nada>")

# --- STOP ---
print("\nParando motores (F6 = 0)")
for mid in MOTORS:
    send_speed(ser, mid, 0)

time.sleep(0.1)

raw = read_raw(ser)
if raw:
    print("RX tras STOP:", raw.hex(" "))
else:
    print("RX tras STOP: <nada>")

ser.close()
print("\nFin")
