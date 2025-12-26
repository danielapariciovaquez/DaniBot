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

MAX_RPM = 500
ACC = 240
DEADZONE = 0.001
SEND_PERIOD = 0.05   # 20 Hz

# Factores de velocidad
SLOW_FACTOR = 0.4
FAST_FACTOR = 1.5

# Botones mando Xbox
BTN_START = 7
BTN_L1 = 4
BTN_R1 = 5

# =====================================================
# FUNCIONES AUXILIARES
# =====================================================
def clamp(val, vmin, vmax):
    return max(vmin, min(vmax, val))

def apply_deadzone(x, dz):
    if abs(x) < dz:
        return 0.0
    return x

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
        0xAA,
        0xC5,                      # DLC=5
        can_id & 0xFF,
        (can_id >> 8) & 0xFF,
        *data,
        crc,
        0x55
    ])

    ser.write(frame)

def send_enable(ser, can_id, enable):
    en_byte = 0x01 if enable else 0x00

    data = [0xF3, en_byte]
    crc = (can_id + sum(data)) & 0xFF

    frame = bytes([
        0xAA,
        0xC3,                      # DLC=3
        can_id & 0xFF,
        (can_id >> 8) & 0xFF,
        *data,
        crc,
        0x55
    ])

    ser.write(frame)

# =====================================================
# INICIALIZAR CAN
# =====================================================
ser = serial.Serial(CAN_PORT, CAN_BAUD)
time.sleep(0.2)
print("USB-CAN listo")

# =====================================================
# INICIALIZAR MANDO XBOX
# =====================================================
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    raise RuntimeError("No se detecta ningún mando")

joy = pygame.joystick.Joystick(0)
joy.init()

print("Mando detectado:", joy.get_name())

# =====================================================
# ESTADO DE SEGURIDAD
# =====================================================
motors_enabled = False
prev_start_state = 0

# =====================================================
# LOOP PRINCIPAL
# =====================================================
last_send = 0

try:
    while True:
        pygame.event.pump()

        # ---------------------------------------------
        # BOTÓN START → ENABLE / DISABLE REAL
        # ---------------------------------------------
        start_state = joy.get_button(BTN_START)
        if start_state == 1 and prev_start_state == 0:
            motors_enabled = not motors_enabled

            if motors_enabled:
                print("MOTORES ENABLE")
                for mid in MOTOR_LEFT + MOTOR_RIGHT:
                    send_enable(ser, mid, True)
            else:
                print("MOTORES DISABLE")
                for mid in MOTOR_LEFT + MOTOR_RIGHT:
                    send_speed(ser, mid, 0)
                    send_enable(ser, mid, False)

        prev_start_state = start_state

        # ---------------------------------------------
        # LECTURA DE EJES
        # ---------------------------------------------
        axis_speed = -joy.get_axis(1)       # eje 1
        axis_turn  =  joy.get_axis(3) / 4   # eje 3

        axis_speed = apply_deadzone(axis_speed, DEADZONE)
        axis_turn  = apply_deadzone(axis_turn, DEADZONE)

        # ---------------------------------------------
        # FACTOR DE VELOCIDAD (L1 / R1)
        # ---------------------------------------------
        speed_factor = 1.0
        if joy.get_button(BTN_L1):
            speed_factor = SLOW_FACTOR
        elif joy.get_button(BTN_R1):
            speed_factor = FAST_FACTOR

        # ---------------------------------------------
        # MEZCLA SKID STEERING
        # ---------------------------------------------
        v = axis_speed
        w = axis_turn

        left_cmd  = -clamp(v + w, -1.0, 1.0)
        right_cmd =  clamp(v - w, -1.0, 1.0)

        left_rpm  = left_cmd  * MAX_RPM * speed_factor
        right_rpm = right_cmd * MAX_RPM * speed_factor

        # ---------------------------------------------
        # ENVÍO PERIÓDICO
        # ---------------------------------------------
        now = time.time()
        if now - last_send >= SEND_PERIOD:

            if motors_enabled:
                for mid in MOTOR_LEFT:
                    send_speed(ser, mid, left_rpm)
                for mid in MOTOR_RIGHT:
                    send_speed(ser, mid, right_rpm)
            else:
                for mid in MOTOR_LEFT + MOTOR_RIGHT:
                    send_speed(ser, mid, 0)

            last_send = now

        time.sleep(0.005)

except KeyboardInterrupt:
    print("Cerrando programa: DISABLE motores")

    motors_enabled = False
    for mid in MOTOR_LEFT + MOTOR_RIGHT:
        send_speed(ser, mid, 0)
        send_enable(ser, mid, False)

    time.sleep(0.1)
    ser.close()
    pygame.quit()

    print("Programa cerrado en estado seguro.")
