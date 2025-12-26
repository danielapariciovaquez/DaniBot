import serial
import time
import pygame

# =====================================================
# CONFIGURACIÓN GENERAL
# =====================================================
CAN_PORT = "/dev/ttyUSB0"
CAN_BAUD = 2000000

MOTOR_RIGHT = [0x01, 0x02]
MOTOR_LEFT  = [0x03, 0x04]
ALL_MOTORS = MOTOR_LEFT + MOTOR_RIGHT

MAX_RPM = 500
ACC = 240
DEADZONE = 0.001
SEND_PERIOD = 0.05

SLOW_FACTOR = 0.4
FAST_FACTOR = 1.5

BTN_START = 7
BTN_L1 = 4
BTN_R1 = 5

# =====================================================
# FUNCIONES AUXILIARES
# =====================================================
def clamp(val, vmin, vmax):
    return max(vmin, min(vmax, val))

def apply_deadzone(x, dz):
    return 0.0 if abs(x) < dz else x

def send_speed(ser, can_id, rpm):
    dir_bit = 0
    if rpm < 0:
        dir_bit = 1
        rpm = -rpm

    rpm = clamp(int(rpm), 0, 3000)

    speed = rpm & 0x0FFF
    byte2 = (dir_bit << 7) | ((speed >> 8) & 0x0F)
    byte3 = speed & 0xFF

    data = [0xF6, byte2, byte3, ACC]
    crc = (can_id + sum(data)) & 0xFF

    frame = bytes([
        0xAA, 0xC5,
        can_id & 0xFF, (can_id >> 8) & 0xFF,
        *data, crc, 0x55
    ])
    ser.write(frame)

def send_enable(ser, can_id, enable):
    en = 0x01 if enable else 0x00
    data = [0xF3, en]
    crc = (can_id + sum(data)) & 0xFF

    frame = bytes([
        0xAA, 0xC3,
        can_id & 0xFF, (can_id >> 8) & 0xFF,
        *data, crc, 0x55
    ])
    ser.write(frame)

# =====================================================
# INIT
# =====================================================
ser = serial.Serial(CAN_PORT, CAN_BAUD)
time.sleep(0.2)

pygame.init()
pygame.joystick.init()
joy = pygame.joystick.Joystick(0)
joy.init()

motors_enabled = False
prev_start = 0
last_send = 0

# =====================================================
# LOOP
# =====================================================
try:
    while True:
        pygame.event.pump()

        # ---------- START TOGGLE ----------
        start = joy.get_button(BTN_START)
        if start == 1 and prev_start == 0:

            motors_enabled = not motors_enabled
            print("MOTORES", "ENABLE" if motors_enabled else "DISABLE")

            # BLOQUE DE TRANSICIÓN (CRÍTICO)
            for _ in range(3):  # reenviar para robustez
                for mid in ALL_MOTORS:
                    send_speed(ser, mid, 0)
                    send_enable(ser, mid, motors_enabled)
                time.sleep(0.02)

            # saltamos este ciclo completamente
            prev_start = start
            continue

        prev_start = start

        # ---------- EJES ----------
        v = apply_deadzone(-joy.get_axis(1), DEADZONE)
        w = apply_deadzone( joy.get_axis(3) / 4, DEADZONE)

        speed_factor = 1.0
        if joy.get_button(BTN_L1):
            speed_factor = SLOW_FACTOR
        elif joy.get_button(BTN_R1):
            speed_factor = FAST_FACTOR

        left = -clamp(v + w, -1, 1) * MAX_RPM * speed_factor
        right =  clamp(v - w, -1, 1) * MAX_RPM * speed_factor

        # ---------- ENVÍO NORMAL ----------
        now = time.time()
        if motors_enabled and now - last_send >= SEND_PERIOD:
            for mid in MOTOR_LEFT:
                send_speed(ser, mid, left)
            for mid in MOTOR_RIGHT:
                send_speed(ser, mid, right)
            last_send = now

        time.sleep(0.005)

except KeyboardInterrupt:
    print("SALIDA SEGURA")

    for _ in range(3):
        for mid in ALL_MOTORS:
            send_speed(ser, mid, 0)
            send_enable(ser, mid, False)
        time.sleep(0.02)

    ser.close()
    pygame.quit()
