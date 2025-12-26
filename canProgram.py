import serial
import time
import pygame

# =====================================================
# CONFIGURACIÓN GENERAL
# =====================================================
CAN_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 2000000     # enlace serie al USB-CAN

# =====================================================
# CONFIGURACIÓN DE CORRIENTE
# =====================================================
WORK_CURRENT_MA = 3000    # mA (SERVO42D máx 3000)
HOLDING_PERCENT = 10     # 10,20,...,90 %

# =====================================================
# MAPEOS
# =====================================================
MOTOR_LEFT  = [0x03, 0x04]
MOTOR_RIGHT = [0x01, 0x02]
ALL_MOTORS  = MOTOR_LEFT + MOTOR_RIGHT

MAX_RPM = 500
ACC = 240
DEADZONE = 0.001
SEND_PERIOD = 0.05

SLOW_FACTOR = 0.4
FAST_FACTOR = 2.0

BTN_START = 7
BTN_L1 = 4
BTN_R1 = 5

# =====================================================
# AUX
# =====================================================
def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def apply_deadzone(x, dz):
    return 0.0 if abs(x) < dz else x

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

def read_raw(ser, t=0.03):
    t0 = time.time()
    buf = bytearray()
    while time.time() - t0 < t:
        if ser.in_waiting:
            buf += ser.read(ser.in_waiting)
        time.sleep(0.001)
    return bytes(buf)

# =====================================================
# COMANDOS MKS
# =====================================================
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

def send_enable(ser, can_id, en):
    ser.write(build_frame(0xC2, can_id, [0xF3, 0x01 if en else 0x00]))

def read_enable_state(ser, can_id):
    ser.reset_input_buffer()
    ser.write(build_frame(0xC1, can_id, [0x3A]))
    raw = read_raw(ser)
    for i in range(len(raw) - 1):
        if raw[i] == 0x3A and raw[i + 1] in (0, 1):
            return raw[i + 1] == 1
    return None

# -----------------------------------------------------
# 0x83 – Set working current (mA)
# -----------------------------------------------------
def set_work_current(ser, can_id, ma):
    ma = clamp(int(ma), 0, 3000)
    lo = ma & 0xFF
    hi = (ma >> 8) & 0xFF
    ser.write(build_frame(0xC3, can_id, [0x83, lo, hi]))
    read_raw(ser)

# -----------------------------------------------------
# 0x9B – Set holding current percentage
# -----------------------------------------------------
def set_holding_current(ser, can_id, percent):
    if percent not in (10,20,30,40,50,60,70,80,90):
        raise ValueError("Holding current inválido")
    code = (percent // 10) - 1   # 10%→0 … 90%→8
    ser.write(build_frame(0xC2, can_id, [0x9B, code]))
    read_raw(ser)

# =====================================================
# SECUENCIAS
# =====================================================
def disable_all(ser):
    for _ in range(2):
        for m in ALL_MOTORS:
            send_speed(ser, m, 0)
        time.sleep(0.03)

    for m in ALL_MOTORS:
        for _ in range(3):
            send_enable(ser, m, False)
            time.sleep(0.02)
            if read_enable_state(ser, m) is False:
                break

def enable_all(ser):
    for m in ALL_MOTORS:
        # 1) Corriente de trabajo
        set_work_current(ser, m, WORK_CURRENT_MA)
        time.sleep(0.01)

        # 2) Holding current
        set_holding_current(ser, m, HOLDING_PERCENT)
        time.sleep(0.01)

        # 3) Enable real
        for _ in range(3):
            send_enable(ser, m, True)
            time.sleep(0.02)
            if read_enable_state(ser, m) is True:
                break

# =====================================================
# INIT
# =====================================================
ser = serial.Serial(CAN_PORT, SERIAL_BAUD)
time.sleep(0.2)
print("USB-CAN listo")

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    raise RuntimeError("No se detecta mando")

joy = pygame.joystick.Joystick(0)
joy.init()

motors_enabled = False
prev_start = 0
last_send = 0.0

# =====================================================
# LOOP
# =====================================================
try:
    while True:
        pygame.event.pump()

        start = joy.get_button(BTN_START)
        if start and not prev_start:
            motors_enabled = not motors_enabled
            print("MOTORES", "ENABLE" if motors_enabled else "DISABLE")
            enable_all(ser) if motors_enabled else disable_all(ser)

        prev_start = start

        v = apply_deadzone(-joy.get_axis(1), DEADZONE)
        w = apply_deadzone( joy.get_axis(3) / 2, DEADZONE)

        factor = FAST_FACTOR if joy.get_button(BTN_R1) else SLOW_FACTOR if joy.get_button(BTN_L1) else 1.0

        left  = -clamp(v + w, -1, 1) * MAX_RPM * factor
        right =  clamp(v - w, -1, 1) * MAX_RPM * factor

        now = time.time()
        if motors_enabled and now - last_send >= SEND_PERIOD:
            for m in MOTOR_LEFT:
                send_speed(ser, m, left)
            for m in MOTOR_RIGHT:
                send_speed(ser, m, right)
            last_send = now

        time.sleep(0.005)

except KeyboardInterrupt:
    print("SALIDA SEGURA")
    disable_all(ser)
    ser.close()
    pygame.quit()
