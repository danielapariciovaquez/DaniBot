import serial
import time
import pygame

# =====================================================
# CONFIGURACIÓN GENERAL
# =====================================================
CAN_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 2000000       # IMPORTANTE: esto es el baudrate del enlace serie al USB-CAN (no el CAN bitrate)

# CAN bitrate en el motor (según manual): 125K/250K/500K/1M
# bitRate = 00 125K, 01 250K, 02 500K, 03 1M  (comando 0x8A)
MOTOR_CAN_BITRATE_CODE = 0x02   # 0x02 = 500K (cámbialo a 0x03 si tu bus es 1M)

# IDs CAN de los motores
MOTOR_LEFT  = [0x03, 0x04]
MOTOR_RIGHT = [0x01, 0x02]
ALL_MOTORS  = MOTOR_LEFT + MOTOR_RIGHT

# Parámetros de control
MAX_RPM = 500
ACC = 0
DEADZONE = 0.001
SEND_PERIOD = 0.05          # 20 Hz

# Factores de velocidad
SLOW_FACTOR = 0.2
FAST_FACTOR = 3

# Botones mando Xbox
BTN_START = 7
BTN_L1 = 4
BTN_R1 = 5

# =====================================================
# AUX
# =====================================================
def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def apply_deadzone(x, dz):
    return 0.0 if abs(x) < dz else x

def _build_frame(dlc, can_id, data_bytes):
    """
    Encapsulado serie de tu adaptador: 0xAA, Cx, IDlo, IDhi, data..., crc, 0x55
    CRC según manual MKS: (ID + sum(data)) & 0xFF
    """
    crc = (can_id + sum(data_bytes)) & 0xFF
    return bytes([
        0xAA,
        dlc & 0xFF,
        can_id & 0xFF,
        (can_id >> 8) & 0xFF,
        *data_bytes,
        crc,
        0x55
    ])

def _read_response_once(ser, timeout_s=0.03):
    """
    Lee lo que haya llegado en un pequeño intervalo.
    Ojo: el formato de respuesta depende del adaptador, aquí solo devolvemos raw bytes.
    """
    t0 = time.time()
    buf = bytearray()
    while (time.time() - t0) < timeout_s:
        n = ser.in_waiting
        if n:
            buf += ser.read(n)
        time.sleep(0.001)
    return bytes(buf)

# =====================================================
# COMANDOS MKS SOBRE CAN (ENCAPSULADOS EN SERIE)
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
    frame = _build_frame(0xC5, can_id, data)   # DLC=5 en encapsulado (1 code + 3 params + ???) según tu adaptador
    ser.write(frame)

def send_enable(ser, can_id, enable):
    en = 0x01 if enable else 0x00
    data = [0xF3, en]
    frame = _build_frame(0xC2, can_id, data)   # tu encapsulado: C2 => 2 bytes data
    ser.write(frame)

def read_enable_state(ser, can_id):
    # MKS: comando 0x3A devuelve enable=0/1 :contentReference[oaicite:3]{index=3}
    data = [0x3A]
    frame = _build_frame(0xC1, can_id, data)   # tu encapsulado: C1 => 1 byte data

    ser.reset_input_buffer()
    ser.write(frame)
    raw = _read_response_once(ser, timeout_s=0.03)

    # Parse mínimo: buscamos un patrón con code=0x3A y un byte de dato detrás.
    # Formato uplink MKS (CAN) sería: [ID][DLC][code][data][CRC], pero tu adaptador lo re-encapsula.
    # Por robustez, buscamos la secuencia 0x3A, <0x00|0x01> en el flujo.
    for i in range(len(raw) - 1):
        if raw[i] == 0x3A and raw[i+1] in (0x00, 0x01):
            return (raw[i+1] == 0x01)

    return None  # no se pudo determinar

def set_motor_canrate(ser, can_id, bitrate_code):
    """
    MKS comando 0x8A: Set CAN bitRate (00..03) :contentReference[oaicite:4]{index=4}
    """
    bitrate_code = clamp(int(bitrate_code), 0, 3)
    data = [0x8A, bitrate_code]
    frame = _build_frame(0xC2, can_id, data)
    ser.write(frame)
    _ = _read_response_once(ser, timeout_s=0.05)  # opcional: descartar ack si existe (depende de CanRSP)

# =====================================================
# SECUENCIAS ROBUSTAS
# =====================================================
def disable_all_motors(ser):
    for _ in range(2):
        for mid in ALL_MOTORS:
            send_speed(ser, mid, 0)
        time.sleep(0.03)

    for mid in ALL_MOTORS:
        for _ in range(3):
            send_enable(ser, mid, False)
            time.sleep(0.02)
            st = read_enable_state(ser, mid)
            if st is False:
                break

def enable_all_motors(ser):
    for mid in ALL_MOTORS:
        for _ in range(3):
            send_enable(ser, mid, True)
            time.sleep(0.02)
            st = read_enable_state(ser, mid)
            if st is True:
                break

# =====================================================
# INICIALIZACIÓN
# =====================================================
ser = serial.Serial(CAN_PORT, SERIAL_BAUD)
time.sleep(0.2)
print(f"USB-CAN listo (enlace serie a {SERIAL_BAUD} bps)")

# (Opcional pero recomendado) Forzar CanRate en cada motor (motor-side)
# NOTA: cambiar CanRate implica coherencia con el adaptador y el resto del bus.
for mid in ALL_MOTORS:
    set_motor_canrate(ser, mid, MOTOR_CAN_BITRATE_CODE)
time.sleep(0.1)

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    raise RuntimeError("No se detecta ningún mando")

joy = pygame.joystick.Joystick(0)
joy.init()
print("Mando detectado:", joy.get_name())

motors_enabled = False
prev_start = 0
last_send = 0.0

# =====================================================
# BUCLE PRINCIPAL
# =====================================================
try:
    while True:
        pygame.event.pump()

        start = joy.get_button(BTN_START)
        if start and not prev_start:
            motors_enabled = not motors_enabled
            print("MOTORES", "ENABLE" if motors_enabled else "DISABLE")

            if motors_enabled:
                enable_all_motors(ser)
            else:
                disable_all_motors(ser)

        prev_start = start

        v = apply_deadzone(-joy.get_axis(1), DEADZONE)
        w = apply_deadzone( joy.get_axis(3) / 2, DEADZONE)

        factor = 1.0
        if joy.get_button(BTN_L1):
            factor = SLOW_FACTOR
        elif joy.get_button(BTN_R1):
            factor = FAST_FACTOR

        left_rpm  = -clamp(v + w, -1, 1) * MAX_RPM * factor
        right_rpm =  clamp(v - w, -1, 1) * MAX_RPM * factor

        now = time.time()
        if motors_enabled and (now - last_send) >= SEND_PERIOD:
            for mid in MOTOR_LEFT:
                send_speed(ser, mid, left_rpm)
            for mid in MOTOR_RIGHT:
                send_speed(ser, mid, right_rpm)
            last_send = now

        time.sleep(0.005)

except KeyboardInterrupt:
    print("SALIDA SEGURA: deshabilitando motores")
    disable_all_motors(ser)
    ser.close()
    pygame.quit()
    print("Sistema cerrado correctamente")
