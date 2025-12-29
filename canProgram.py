import serial
import time
import pygame

# =====================================================
# CONFIGURACIÓN GENERAL
# =====================================================
PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 2000000

# =====================================================
# CONFIGURACIÓN DE CORRIENTE
# =====================================================
WORK_CURRENT_MA = 1600
HOLDING_PERCENT = 50

# =====================================================
# IDs DE MOTORES
# =====================================================
MOTOR_LEFT  = [0x03, 0x04]
MOTOR_RIGHT = [0x01, 0x02]
ALL_MOTORS  = MOTOR_LEFT + MOTOR_RIGHT

# =====================================================
# CONTROL
# =====================================================
ACC = 0
DEADZONE = 0.001
SEND_PERIOD = 0.001

BTN_START = 7
BTN_L1    = 4

BTN_A = 0
BTN_B = 1
BTN_X = 2
BTN_Y = 3

# =====================================================
# MODOS DE VELOCIDAD (RPM MÁX)
# =====================================================
MODE_RPM = {
    1: 50,
    2: 100,
    3: 200,
    4: 400
}

current_mode = 2            # modo por defecto
rpm_limit = MODE_RPM[current_mode]

# =====================================================
# ACELERACIÓN LINEAL ABSOLUTA
# =====================================================
RPM_PER_100_TIME = 0.5     # 100 RPM en 0.25 s

# =====================================================
# AUXILIARES
# =====================================================
def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def apply_deadzone(x, dz):
    return 0.0 if abs(x) < dz else x

def ramp(current, target, max_delta):
    if target > current + max_delta:
        return current + max_delta
    if target < current - max_delta:
        return current - max_delta
    return target

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

def read_raw(ser, timeout=0.04):
    t0 = time.time()
    buf = bytearray()
    while time.time() - t0 < timeout:
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

def send_enable(ser, can_id, enable):
    ser.write(build_frame(0xC2, can_id, [0xF3, 0x01 if enable else 0x00]))

def read_enable_state(ser, can_id):
    ser.reset_input_buffer()
    ser.write(build_frame(0xC1, can_id, [0x3A]))
    raw = read_raw(ser)
    for i in range(len(raw) - 1):
        if raw[i] == 0x3A and raw[i + 1] in (0, 1):
            return raw[i + 1] == 1
    return None

def set_work_current(ser, can_id, ma):
    ma = clamp(int(ma), 0, 3000)
    lo = ma & 0xFF
    hi = (ma >> 8) & 0xFF
    ser.write(build_frame(0xC3, can_id, [0x83, lo, hi]))
    read_raw(ser)

def set_holding_current(ser, can_id, percent):
    code = (percent // 10) - 1
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
        send_enable(ser, m, False)
        time.sleep(0.03)

def enable_all(ser):
    for m in ALL_MOTORS:
        send_enable(ser, m, False)
        time.sleep(0.05)

        set_work_current(ser, m, WORK_CURRENT_MA)
        time.sleep(0.02)

        set_holding_current(ser, m, HOLDING_PERCENT)
        time.sleep(0.02)

        send_enable(ser, m, True)
        time.sleep(0.05)

# =====================================================
# INIT
# =====================================================
ser = serial.Serial(PORT, SERIAL_BAUD)
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

v_rpm_filtered = 0.0
last_time = time.time()

# =====================================================
# LOOP PRINCIPAL
# =====================================================
try:
    while True:
        pygame.event.pump()

        # -------- ENABLE / DISABLE --------
        start = joy.get_button(BTN_START)
        if start and not prev_start:
            motors_enabled = not motors_enabled
            print("MOTORES", "ENABLE" if motors_enabled else "DISABLE")
            enable_all(ser) if motors_enabled else disable_all(ser)
        prev_start = start

        # -------- CAMBIO DE MODO (L1 + BOTÓN) --------
        if joy.get_button(BTN_L1):
            if joy.get_button(BTN_A):
                current_mode = 1
            elif joy.get_button(BTN_X):
                current_mode = 2
            elif joy.get_button(BTN_Y):
                current_mode = 3
            elif joy.get_button(BTN_B):
                current_mode = 4

        rpm_limit = MODE_RPM[current_mode]

        # Reanclar estado interno al nuevo límite
        v_rpm_filtered = clamp(v_rpm_filtered, -rpm_limit, rpm_limit)

        # -------- LECTURA EJES --------
        v_cmd = apply_deadzone(-joy.get_axis(1), DEADZONE)
        w     = apply_deadzone( joy.get_axis(3)/(rpm_limit*10), DEADZONE)

        # -------- CONSIGNA LINEAL --------
        v_rpm_cmd = v_cmd * rpm_limit

        # -------- RAMPA --------
        now = time.time()
        dt = now - last_time
        last_time = now

        acc_rpm = 100.0 / RPM_PER_100_TIME
        max_step = acc_rpm * dt

        v_rpm_filtered = ramp(v_rpm_filtered, v_rpm_cmd, max_step)

        # -------- MEZCLA SKID STEERING --------
        w_rpm = w * rpm_limit

        left  = -clamp(v_rpm_filtered + w_rpm, -rpm_limit, rpm_limit)
        right =  clamp(v_rpm_filtered - w_rpm, -rpm_limit, rpm_limit)

        # -------- ENVÍO --------
        if motors_enabled:
            for m in MOTOR_LEFT:
                send_speed(ser, m, left)
            for m in MOTOR_RIGHT:
                send_speed(ser, m, right)

        time.sleep(0.005)

except KeyboardInterrupt:
    print("SALIDA SEGURA")
    disable_all(ser)
    ser.close()
    pygame.quit()
