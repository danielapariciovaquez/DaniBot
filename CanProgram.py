import serial
import time
import pygame
import math

# =====================================================
# CONFIGURACIÓN GENERAL
# =====================================================
CAN_PORT = "/dev/ttyUSB0"
CAN_BAUD = 2000000

MOTOR_RIGHT = [0x01, 0x02]
MOTOR_LEFT  = [0x03, 0x04]

MAX_RPM = 300      # velocidad máxima permitida
ACC = 200            # aceleración fija
DEADZONE = 0.08    # zona muerta del stick
SEND_PERIOD = 0.05 # 20 Hz

# =====================================================
# FUNCIONES AUXILIARES
# =====================================================
def clamp(val, vmin, vmax):
    return max(vmin, min(vmax, val))

def send_speed(ser, can_id, rpm):
    """
    Envía comando F6 a un motor concreto.
    Basado 100% en el frame validado.
    """
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

        # Stick izquierdo
        axis_y = -joy.get_axis(1)  # adelante positivo
        axis_x =  joy.get_axis(2)/4  # giro

        # Deadzone
        if abs(axis_y) < DEADZONE:
            axis_y = 0.0
        if abs(axis_x) < DEADZONE:
            axis_x = 0.0

        # Mezcla skid steering
        v = axis_y
        w = axis_x

        left_cmd  = clamp(v + w, -1.0, 1.0)
        right_cmd = clamp(v - w, -1.0, 1.0)

        left_rpm  = left_cmd  * MAX_RPM
        right_rpm = right_cmd * MAX_RPM

        # Envío periódico
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

    # Parada limpia
    for mid in MOTOR_LEFT + MOTOR_RIGHT:
        send_speed(ser, mid, 0)

    ser.close()
    pygame.quit()
#this is a Python program for controlling the DaniBot with Servo42d Can version motors
