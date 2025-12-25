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

MAX_RPM = 300
ACC = 2
DEADZONE = 0.01
SEND_PERIOD = 0.05   # 20 Hz

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
        0xC5,
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
# LOOP PRINCIPAL
# =====================================================
last_send = 0

try:
    while True:
        pygame.event.pump()

        # ---------------------------------------------
        # LECTURA DE EJES (según tu indicación)
        # ---------------------------------------------
        axis_speed = -joy.get_axis(1)   # eje 1 → velocidad (adelante +)
        axis_turn  =  joy.get_axis(3)   # eje 3 → dirección

        axis_speed = apply_deadzone(axis_speed, DEADZONE)
        axis_turn  = apply_deadzone(axis_turn, DEADZONE)

        # ---------------------------------------------
        # MEZCLA SKID STEERING
        # ---------------------------------------------
        v = axis_speed
        w = axis_turn

        left_cmd  = -clamp(v + w, -1.0, 1.0)
        right_cmd = clamp(v - w, -1.0, 1.0)

        left_rpm  = left_cmd  * MAX_RPM
        right_rpm = right_cmd * MAX_RPM

        # ---------------------------------------------
        # ENVÍO PERIÓDICO
        # ---------------------------------------------
        now = time.time()
        if now - last_send >= SEND_PERIOD:
            for mid in MOTOR_LEFT:
                send_speed(ser, mid, left_rpm)
            for mid in MOTOR_RIGHT:
                send_speed(ser, mid, right_rpm)

            last_send = now

        time.sleep(0.005)

except KeyboardInterrupt:
    print("Parando vehículo...")

    for mid in MOTOR_LEFT + MOTOR_RIGHT:
        send_speed(ser, mid, 0)

    ser.close()
    pygame.quit()
