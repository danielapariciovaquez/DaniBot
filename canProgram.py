import serial
import time
import pygame

# =====================================================
# CONFIGURACIÓN GENERAL
# =====================================================
PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 2000000     # baudrate del enlace serie USB-CAN (no el CAN bitrate)

# =====================================================
# CONFIGURACIÓN DE CORRIENTE
# =====================================================
WORK_CURRENT_MA = 1600     # SERVO42D: 0–3000 mA
HOLDING_PERCENT = 50       # 10,20,...,90  (en vFOC se ignora)

# =====================================================
# IDs DE MOTORES
# =====================================================
MOTOR_LEFT  = [0x03, 0x04]
MOTOR_RIGHT = [0x01, 0x02]
ALL_MOTORS  = MOTOR_LEFT + MOTOR_RIGHT

# =====================================================
# CONTROL
# =====================================================
MAX_RPM = 500
ACC = 240
DEADZONE = 0.001
SEND_PERIOD = 0.05         # 20 Hz

BTN_START = 7
BTN_L1 = 4
BTN_R1 = 5

SLOW_FACTOR = 0.4
FAST_FACTOR = 2.0

# =====================================================
# ESTADOS (bloqueo de envío durante enable/disable)
# =====================================================
STATE_DISABLED  = 0
STATE_ENABLING  = 1
STATE_ENABLED   = 2
STATE_DISABLING = 3

# =====================================================
# AUXILIARES
# =====================================================
def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def apply_deadzone(x, dz):
    return 0.0 if abs(x) < dz else x

def build_frame(dlc, can_id, data):
    # CRC según manual: (ID + byte1 + ... + byte(n)) & 0xFF
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

def read_raw(ser, timeout=0.05):
    t0 = time.time()
    buf = bytearray()
    while time.time() - t0 < timeout:
        n = ser.in_waiting
        if n:
            buf += ser.read(n)
        time.sleep(0.001)
    return bytes(buf)

# =====================================================
# COMANDOS MKS (encapsulados por tu USB-CAN)
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

    # Encapsulado que vienes usando: 0xC5 para F6 (4 bytes de data: F6 b2 b3 acc)
    ser.write(build_frame(0xC5, can_id, [0xF6, b2, b3, ACC]))

def send_enable(ser, can_id, enable):
    ser.write(build_frame(0xC2, can_id, [0xF3, 0x01 if enable else 0x00]))

def read_enable_state(ser, can_id):
    # Comando 0x3A: Read the En pins status (devuelve enable 0/1)
    ser.reset_input_buffer()
    ser.write(build_frame(0xC1, can_id, [0x3A]))
    raw = read_raw(ser, timeout=0.05)

    # Parse mínimo robusto: buscar patrón 0x3A, (0x00|0x01) en el flujo
    for i in range(len(raw) - 1):
        if raw[i] == 0x3A and raw[i + 1] in (0, 1):
            return raw[i + 1] == 1
    return None

def set_work_current(ser, can_id, ma):
    # Comando 0x83: Set working current (uint16_t mA)
    ma = clamp(int(ma), 0, 3000)
    lo = ma & 0xFF
    hi = (ma >> 8) & 0xFF
    ser.write(build_frame(0xC3, can_id, [0x83, lo, hi]))
    read_raw(ser, timeout=0.05)

def set_holding_current(ser, can_id, percent):
    # Comando 0x9B: Set holding current percentage (00..08 => 10..90%)
    if percent not in (10, 20, 30, 40, 50, 60, 70, 80, 90):
        raise ValueError("HOLDING_PERCENT debe ser 10..90 en pasos de 10")
    code = (percent // 10) - 1
    ser.write(build_frame(0xC2, can_id, [0x9B, code]))
    read_raw(ser, timeout=0.05)

# =====================================================
# SECUENCIAS DETERMINISTAS (sin interferencia del loop)
# =====================================================
def disable_all(ser):
    # 1) Parar en speed mode (2 veces) para asegurar salida limpia
    for _ in range(2):
        for m in ALL_MOTORS:
            send_speed(ser, m, 0)
        time.sleep(0.03)

    # 2) Disable real (reintento + verificación 0x3A)
    for m in ALL_MOTORS:
        for _ in range(4):
            send_enable(ser, m, False)
            time.sleep(0.03)
            st = read_enable_state(ser, m)
            if st is False:
                break

def enable_all(ser):
    for m in ALL_MOTORS:
        # 0) Asegurar disable antes de configurar corrientes
        for _ in range(2):
            send_enable(ser, m, False)
            time.sleep(0.03)
            st = read_enable_state(ser, m)
            if st is False:
                break

        # 1) Configurar corriente de trabajo (mA)
        set_work_current(ser, m, WORK_CURRENT_MA)
        time.sleep(0.02)

        # 2) Configurar holding current (%). (En vFOC el firmware lo ignora)
        set_holding_current(ser, m, HOLDING_PERCENT)
        time.sleep(0.02)

        # 3) Enable real (reintento + verificación 0x3A)
        for _ in range(4):
            send_enable(ser, m, True)
            time.sleep(0.04)
            st = read_enable_state(ser, m)
            if st is True:
                break

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
print("Mando detectado:", joy.get_name())

system_state = STATE_DISABLED
prev_start = 0
last_send = 0.0

# =====================================================
# LOOP PRINCIPAL
# =====================================================
try:
    while True:
        pygame.event.pump()

        # --------------------------
        # START: toggle enable/disable (bloqueante)
        # --------------------------
        start = joy.get_button(BTN_START)
        if start and not prev_start:

            if system_state == STATE_ENABLED:
                print("TRANSICIÓN: DISABLING")
                system_state = STATE_DISABLING
                disable_all(ser)
                system_state = STATE_DISABLED
                print("MOTORES DESACTIVADOS")

            elif system_state == STATE_DISABLED:
                print("TRANSICIÓN: ENABLING (configurando corriente + holding)")
                system_state = STATE_ENABLING
                enable_all(ser)
                system_state = STATE_ENABLED
                print("MOTORES ACTIVADOS")

        prev_start = start

        # --------------------------
        # SOLO EN ENABLED se envía F6
        # --------------------------
        if system_state == STATE_ENABLED:
            v = apply_deadzone(-joy.get_axis(1), DEADZONE)
            w = apply_deadzone(joy.get_axis(3) / 2, DEADZONE)

            factor = 1.0
            if joy.get_button(BTN_L1):
                factor = SLOW_FACTOR
            elif joy.get_button(BTN_R1):
                factor = FAST_FACTOR

            left_rpm  = -clamp(v + w, -1, 1) * MAX_RPM * factor
            right_rpm =  clamp(v - w, -1, 1) * MAX_RPM * factor

            now = time.time()
            if (now - last_send) >= SEND_PERIOD:
                for m in MOTOR_LEFT:
                    send_speed(ser, m, left_rpm)
                for m in MOTOR_RIGHT:
                    send_speed(ser, m, right_rpm)
                last_send = now

        time.sleep(0.005)

except KeyboardInterrupt:
    print("SALIDA SEGURA: deshabilitando motores")
    system_state = STATE_DISABLING
    disable_all(ser)
    system_state = STATE_DISABLED
    ser.close()
    pygame.quit()
    print("Sistema cerrado correctamente")
