import serial
import time
import pygame

# =====================================================
# CONFIGURACIÓN GENERAL
# =====================================================
CAN_PORT = "/dev/ttyUSB0"
CAN_BAUD = 2000000

# IDs CAN de los motores
MOTOR_LEFT  = [0x03, 0x04]
MOTOR_RIGHT = [0x01, 0x02]
ALL_MOTORS = MOTOR_LEFT + MOTOR_RIGHT

# Parámetros de control
MAX_RPM = 500
ACC = 0
DEADZONE = 0.001
SEND_PERIOD = 0.05   # 20 Hz

# Factores de velocidad
SLOW_FACTOR = 0.4
FAST_FACTOR = 2

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

# -----------------------------------------------------
# Envío F6: control de velocidad (speed mode)
# -----------------------------------------------------
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
        0xC5,                       # DLC = 4 (F6)
        can_id & 0xFF,
        (can_id >> 8) & 0xFF,
        *data,
        crc,
        0x55
    ])
    ser.write(frame)

# -----------------------------------------------------
# Envío F3: enable / disable real del driver
# -----------------------------------------------------
def send_enable(ser, can_id, enable):
    en = 0x01 if enable else 0x00
    data = [0xF3, en]
    crc = (can_id + sum(data)) & 0xFF

    frame = bytes([
        0xAA,
        0xC2,                       # DLC = 2  (CORRECTO)
        can_id & 0xFF,
        (can_id >> 8) & 0xFF,
        *data,
        crc,
        0x55
    ])
    ser.write(frame)

# =====================================================
# INICIALIZACIÓN DE CAN
# =====================================================
ser = serial.Serial(CAN_PORT, CAN_BAUD)
time.sleep(0.2)
print("USB-CAN listo")

# =====================================================
# INICIALIZACIÓN DEL MANDO XBOX
# =====================================================
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    raise RuntimeError("No se detecta ningún mando")

joy = pygame.joystick.Joystick(0)
joy.init()
print("Mando detectado:", joy.get_name())

# =====================================================
# ESTADO DEL SISTEMA
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

        # ---------------------------------------------
        # BOTÓN START → ENABLE / DISABLE REAL
        # ---------------------------------------------
        start = joy.get_button(BTN_START)
        if start == 1 and prev_start == 0:
            motors_enabled = not motors_enabled
            print("MOTORES", "ENABLE" if motors_enabled else "DISABLE")

            if not motors_enabled:
                # ---- SECUENCIA CORRECTA DE DESACTIVACIÓN ----
                # 1) Salir del modo speed (F6 a 0)
                for _ in range(2):
                    for mid in ALL_MOTORS:
                        send_speed(ser, mid, 0)
                    time.sleep(0.03)

                # 2) Disable real del driver
                for _ in range(2):
                    for mid in ALL_MOTORS:
                        send_enable(ser, mid, False)
                    time.sleep(0.03)

            else:
                # ---- SECUENCIA DE ACTIVACIÓN ----
                for _ in range(2):
                    for mid in ALL_MOTORS:
                        send_enable(ser, mid, True)
                    time.sleep(0.03)

            prev_start = start
            continue

        prev_start = start

        # ---------------------------------------------
        # LECTURA DE EJES
        # ---------------------------------------------
        v = apply_deadzone(-joy.get_axis(1), DEADZONE)
        w = apply_deadzone( joy.get_axis(3) / 2, DEADZONE)

        # ---------------------------------------------
        # FACTOR DE VELOCIDAD (L1 / R1)
        # ---------------------------------------------
        factor = 1.0
        if joy.get_button(BTN_L1):
            factor = SLOW_FACTOR
        elif joy.get_button(BTN_R1):
            factor = FAST_FACTOR

        # ---------------------------------------------
        # MEZCLA SKID STEERING
        # ---------------------------------------------
        left_rpm  = -clamp(v + w, -1, 1) * MAX_RPM * factor
        right_rpm =  clamp(v - w, -1, 1) * MAX_RPM * factor

        # ---------------------------------------------
        # ENVÍO PERIÓDICO DE VELOCIDAD
        # ---------------------------------------------
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

    # Salida limpia del modo speed
    for _ in range(2):
        for mid in ALL_MOTORS:
            send_speed(ser, mid, 0)
        time.sleep(0.03)

    # Disable real
    for _ in range(2):
        for mid in ALL_MOTORS:
            send_enable(ser, mid, False)
        time.sleep(0.03)

    ser.close()
    pygame.quit()
    print("Sistema cerrado correctamente")
