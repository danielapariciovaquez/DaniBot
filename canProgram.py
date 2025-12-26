import serial
import time
import pygame

# =====================================================
# CONFIGURACIÓN GENERAL
# =====================================================
CAN_PORT = "/dev/ttyUSB0"
CAN_BAUD = 500000          # CORREGIDO (válido según manual)

# IDs CAN de los motores
MOTOR_LEFT  = [0x03, 0x04]
MOTOR_RIGHT = [0x01, 0x02]
ALL_MOTORS  = MOTOR_LEFT + MOTOR_RIGHT

# Parámetros de control
MAX_RPM = 500
ACC = 240
DEADZONE = 0.001
SEND_PERIOD = 0.05         # 20 Hz

# Factores de velocidad
SLOW_FACTOR = 0.4
FAST_FACTOR = 2.0

# Botones mando Xbox
BTN_START = 7
BTN_L1 = 4
BTN_R1 = 5

# =====================================================
# FUNCIONES AUXILIARES
# =====================================================
def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def apply_deadzone(x, dz):
    return 0.0 if abs(x) < dz else x

# =====================================================
# CAN – ENVÍO DE TRAMAS
# =====================================================
def send_speed(ser, can_id, rpm):
    direction = 0
    if rpm < 0:
        direction = 1
        rpm = -rpm

    rpm = clamp(int(rpm), 0, 3000)
    speed = rpm & 0x0FFF

    byte2 = (direction << 7) | ((speed >> 8) & 0x0F)
    byte3 = speed & 0xFF

    data = [0xF6, byte2, byte3, ACC]
    crc = (can_id + sum(data)) & 0xFF

    frame = bytes([
        0xAA,
        0xC5,
        can_id & 0xFF,
        (can_id >> 8) & 0xFF,
        *data,
        crc,
        0x55
    ])
    ser.write(frame)

def send_enable(ser, can_id, enable):
    en = 0x01 if enable else 0x00
    data = [0xF3, en]
    crc = (can_id + sum(data)) & 0xFF

    frame = bytes([
        0xAA,
        0xC2,
        can_id & 0xFF,
        (can_id >> 8) & 0xFF,
        *data,
        crc,
        0x55
    ])
    ser.write(frame)

# =====================================================
# CAN – LECTURA ESTADO REAL ENABLE (0x3A)
# =====================================================
def read_enable_state(ser, can_id):
    data = [0x3A]
    crc = (can_id + sum(data)) & 0xFF

    frame = bytes([
        0xAA,
        0xC1,
        can_id & 0xFF,
        (can_id >> 8) & 0xFF,
        *data,
        crc,
        0x55
    ])

    ser.reset_input_buffer()
    ser.write(frame)
    time.sleep(0.01)

    if ser.in_waiting >= 9:
        resp = ser.read(ser.in_waiting)
        if resp[5] == 0x3A:
            return resp[6] == 1

    return None

# =====================================================
# SECUENCIAS ROBUSTAS
# =====================================================
def disable_all_motors(ser):
    # Salida limpia de speed mode
    for _ in range(2):
        for mid in ALL_MOTORS:
            send_speed(ser, mid, 0)
        time.sleep(0.03)

    # Disable real con verificación
    for mid in ALL_MOTORS:
        for _ in range(3):
            send_enable(ser, mid, False)
            time.sleep(0.02)
            state = read_enable_state(ser, mid)
            if state is False:
                break

def enable_all_motors(ser):
    for mid in ALL_MOTORS:
        for _ in range(3):
            send_enable(ser, mid, True)
            time.sleep(0.02)
            state = read_enable_state(ser, mid)
            if state is True:
                break

# =====================================================
# INICIALIZACIÓN
# =====================================================
ser = serial.Serial(CAN_PORT, CAN_BAUD)
time.sleep(0.2)
print("USB-CAN listo a 500 kbps")

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    raise RuntimeError("No se detecta ningún mando")

joy = pygame.joystick.Joystick(0)
joy.init()
print("Mando detectado:", joy.get_name())

# =====================================================
# ESTADO
# =====================================================
motors_enabled = False
prev_start = 0
last_send = 0

# =====================================================
# BUCLE PRINCIPAL
# =====================================================
try:
    while True:
        pygame.event.pump()

        # -------- START: ENABLE / DISABLE REAL --------
        start = joy.get_button(BTN_START)
        if start and not prev_start:
            motors_enabled = not motors_enabled
            print("MOTORES", "ENABLE" if motors_enabled else "DISABLE")

            if motors_enabled:
                enable_all_motors(ser)
            else:
                disable_all_motors(ser)

        prev_start = start

        # -------- EJES --------
        v = apply_deadzone(-joy.get_axis(1), DEADZONE)
        w = apply_deadzone( joy.get_axis(3) / 2, DEADZONE)

        # -------- FACTOR VELOCIDAD --------
        factor = 1.0
        if joy.get_button(BTN_L1):
            factor = SLOW_FACTOR
        elif joy.get_button(BTN_R1):
            factor = FAST_FACTOR

        # -------- SKID STEERING --------
        left_rpm  = -clamp(v + w, -1, 1) * MAX_RPM * factor
        right_rpm =  clamp(v - w, -1, 1) * MAX_RPM * factor

        # -------- ENVÍO PERIÓDICO --------
        now = time.time()
        if motors_enabled and (now - last_send) >= SEND_PERIOD:
            for mid in MOTOR_LEFT:
                send_speed(ser, mid, left_rpm)
            for mid in MOTOR_RIGHT:
                send_speed(ser, mid, right_rpm)
            last_send = now

        time.sleep(0.005)

# =====================================================
# SALIDA SEGURA
# =====================================================
except KeyboardInterrupt:
    print("SALIDA SEGURA: deshabilitando motores")
    disable_all_motors(ser)
    ser.close()
    pygame.quit()
    print("Sistema cerrado correctamente")
