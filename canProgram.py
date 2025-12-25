#!/usr/bin/python3
# =====================================================
# DaniBot – Control CAN con mando Xbox (modo seguro)
# =====================================================

import os
import sys
import time
import signal
import serial
import pygame

# =====================================================
# PYGAME EN MODO HEADLESS (OBLIGATORIO EN BACKGROUND)
# =====================================================
os.environ["SDL_VIDEODRIVER"] = "dummy"
os.environ["SDL_AUDIODRIVER"] = "dummy"

# =====================================================
# CONFIGURACIÓN GENERAL
# =====================================================
CAN_PORT = "/dev/ttyUSB0"
CAN_BAUD = 2000000

MOTOR_RIGHT = [0x01, 0x02]
MOTOR_LEFT  = [0x03, 0x04]

MAX_RPM = 800
ACC = 252
DEADZONE = 0.001
SEND_PERIOD = 0.02   # 50 Hz

# Factores de velocidad
SLOW_FACTOR = 0.1    # L1
FAST_FACTOR = 4.0    # R1

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
        0xC5,
        can_id & 0xFF,
        (can_id >> 8) & 0xFF,
        *data,
        crc,
        0x55
    ])

    ser.write(frame)

# =====================================================
# PARADA SEGURA CENTRALIZADA
# =====================================================
def stop_all_motors():
    try:
        for mid in MOTOR_LEFT + MOTOR_RIGHT:
            send_speed(ser, mid, 0)
    except Exception:
        pass

# =====================================================
# HANDLER DE SEÑALES (systemd / kill / reboot)
# =====================================================
def signal_handler(signum, frame):
    print(f"Señal {signum} recibida → parada segura")
    stop_all_motors()
    try:
        ser.close()
    except Exception:
        pass
    pygame.quit()
    sys.exit(0)

signal.signal(signal.SIGTERM, signal_handler)
signal.signal(signal.SIGINT,  signal_handler)
signal.signal(signal.SIGHUP,  signal_handler)

# =====================================================
# INICIALIZAR CAN
# =====================================================
print("Inicializando USB-CAN...")
ser = serial.Serial(CAN_PORT, CAN_BAUD, timeout=0.1)
time.sleep(0.3)
print("USB-CAN listo")

# =====================================================
# INICIALIZAR MANDO XBOX
# =====================================================
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    raise RuntimeError("No se detecta ningún mando Xbox")

joy = pygame.joystick.Joystick(0)
joy.init()

print("Mando detectado:", joy.get_name())

# =====================================================
# ESTADO DE SEGURIDAD
# =====================================================
motors_enabled = False
prev_start_state = 0
last_send = 0

# =====================================================
# LOOP PRINCIPAL
# =====================================================
try:
    while True:
        pygame.event.pump()

        # ---------------------------------------------
        # BOTÓN START (TOGGLE ENABLE)
        # ---------------------------------------------
        start_state = joy.get_button(BTN_START)
        if start_state == 1 and prev_start_state == 0:
            motors_enabled = not motors_enabled
            print("MOTORES", "HABILITADOS" if motors_enabled else "DESHABILITADOS")
        prev_start_state = start_state

        # ---------------------------------------------
        # LECTURA DE EJES
        # ---------------------------------------------
        axis_speed = -joy.get_axis(1)
        axis_turn  =  joy.get_axis(3) / 4.0

        axis_speed = apply_deadzone(axis_speed, DEADZONE)
        axis_turn  = apply_deadzone(axis_turn, DEADZONE)

        # ---------------------------------------------
        # FACTOR DE VELOCIDAD
        # ---------------------------------------------
        speed_factor = 1.0
        if joy.get_button(BTN_L1):
            speed_factor = SLOW_FACTOR
        elif joy.get_button(BTN_R1):
            speed_factor = FAST_FACTOR

        # ---------------------------------------------
        # MEZCLA SKID-STEERING
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
                stop_all_motors()

            last_send = now

        time.sleep(0.005)

# =====================================================
# GESTIÓN DE ERRORES
# =====================================================
except Exception as e:
    print("ERROR CRÍTICO:", e)
    raise

finally:
    print("Finalizando → parada segura")
    stop_all_motors()
    try:
        ser.close()
    except Exception:
        pass
    pygame.quit()
