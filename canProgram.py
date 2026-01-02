import serial
import time
import pygame
import RPi.GPIO as GPIO

# =====================================================
# CONFIGURACIÓN GENERAL
# =====================================================
PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 2000000

WATCHDOG_TIMEOUT = 0.5   # s sin mando -> EMERGENCY STOP

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
    1: 20,
    2: 150,
    3: 300,
    4: 500
}

current_mode = 2

# =====================================================
# ACELERACIÓN LINEAL
# =====================================================
RPM_PER_100_TIME = 0.2   # 100 RPM en 0.2 s

# =====================================================
# GPIO LEDS
# =====================================================
LED_RED = 24
LED_GREEN = 25

GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_RED, GPIO.OUT)
GPIO.setup(LED_GREEN, GPIO.OUT)

def led_disable():
    GPIO.output(LED_RED, GPIO.HIGH)
    GPIO.output(LED_GREEN, GPIO.LOW)

def led_enable():
    GPIO.output(LED_RED, GPIO.LOW)
    GPIO.output(LED_GREEN, GPIO.HIGH)

def led_off():
    GPIO.output(LED_RED, GPIO.LOW)
    GPIO.output(LED_GREEN, GPIO.LOW)

led_disable()

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

def set_work_current(ser, can_id, ma):
    ma = clamp(int(ma), 0, 3000)
    ser.write(build_frame(0xC3, can_id, [0x83, ma & 0xFF, (ma >> 8) & 0xFF]))

def set_holding_current(ser, can_id, percent):
    code = (percent // 10) - 1
    ser.write(build_frame(0xC2, can_id, [0x9B, code]))

# =====================================================
# SECUENCIAS DE SEGURIDAD
# =====================================================
def disable_all(ser):
    led_disable()
    for _ in range(2):
        for m in ALL_MOTORS:
            send_speed(ser, m, 0)
        time.sleep(0.02)
    for _ in range(2):
        for m in ALL_MOTORS:
            send_enable(ser, m, False)
        time.sleep(0.02)

def enable_all(ser):
    for m in ALL_MOTORS:
        send_enable(ser, m, False)
        time.sleep(0.03)
        set_work_current(ser, m, WORK_CURRENT_MA)
        time.sleep(0.02)
        set_holding_current(ser, m, HOLDING_PERCENT)
        time.sleep(0.02)
        send_enable(ser, m, True)
        time.sleep(0.03)
    led_enable()

def emergency_stop(ser):
    print("EMERGENCY STOP: mando perdido")
    disable_all(ser)

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

last_joy_time = time.time()
joystick_ok = True

# =====================================================
# LOOP PRINCIPAL
# =====================================================
try:
    while True:

        # -------- EVENTOS (desconexión mando) --------
        for event in pygame.event.get():
            if event.type == pygame.JOYDEVICEREMOVED:
                joystick_ok = False
                motors_enabled = False
                emergency_stop(ser)

        # -------- WATCHDOG --------
        if joystick_ok and (time.time() - last_joy_time) > WATCHDOG_TIMEOUT:
            joystick_ok = False
            motors_enabled = False
            emergency_stop(ser)

        if not joystick_ok:
            time.sleep(0.05)
            continue

        # -------- ENABLE / DISABLE --------
        start = joy.get_button(BTN_START)
        if start and not prev_start:
            motors_enabled = not motors_enabled
            print("MOTORES", "ENABLE" if motors_enabled else "DISABLE")
            enable_all(ser) if motors_enabled else disable_all(ser)
        prev_start = start

        # -------- CAMBIO DE MODO --------
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
        v_rpm_filtered = clamp(v_rpm_filtered, -rpm_limit, rpm_limit)

        # -------- EJES --------
        v_cmd = apply_deadzone(-joy.get_axis(1), DEADZONE)
        w     = apply_deadzone( joy.get_axis(3)/(current_mode*3), DEADZONE)
        last_joy_time = time.time()

        # -------- RAMPA --------
        now = time.time()
        dt = now - last_time
        last_time = now

        acc_rpm = 100.0 / RPM_PER_100_TIME
        v_rpm_cmd = v_cmd * rpm_limit
        v_rpm_filtered = ramp(v_rpm_filtered, v_rpm_cmd, acc_rpm * dt)

        # -------- MEZCLA --------
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

# =====================================================
# SALIDA SEGURA
# =====================================================
except KeyboardInterrupt:
    print("SALIDA SEGURA")
    disable_all(ser)
    led_off()
    ser.close()
    pygame.quit()
    GPIO.cleanup()