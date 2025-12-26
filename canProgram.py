import serial
import time
import pygame

# =====================================================
# CONFIGURACIÓN
# =====================================================
PORT = "/dev/ttyUSB0"
BAUD = 2000000

MOTOR_LEFT  = [0x03, 0x04]
MOTOR_RIGHT = [0x01, 0x02]

MAX_RPM = 500
ACC = 240
DEADZONE = 0.01
SEND_PERIOD = 0.05   # 20 Hz

# =====================================================
# AUX
# =====================================================
def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def deadzone(x, dz):
    return 0.0 if abs(x) < dz else x

def build_frame(can_id, data):
    crc = (can_id + sum(data)) & 0xFF
    return bytes([
        0xAA,
        0xC5,               # DLC para F6
        can_id & 0xFF,
        (can_id >> 8) & 0xFF,
        *data,
        crc,
        0x55
    ])

def send_speed(ser, can_id, rpm):
    direction = 0
    if rpm < 0:
        direction = 1
        rpm = -rpm

    rpm = clamp(int(rpm), 0, 3000)
    speed = rpm & 0x0FFF

    b2 = (direction << 7) | ((speed >> 8) & 0x0F)
    b3 = speed & 0xFF

    frame = build_frame(can_id, [0xF6, b2, b3, ACC])
    ser.write(frame)

# =====================================================
# INIT SERIAL
# =====================================================
ser = serial.Serial(PORT, BAUD)
time.sleep(0.2)
print("USB-CAN abierto")

# =====================================================
# INIT MANDO
# =====================================================
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    raise RuntimeError("No hay mando")

joy = pygame.joystick.Joystick(0)
joy.init()
print("Mando:", joy.get_name())

last_send = 0

# =====================================================
# LOOP
# =====================================================
try:
    while True:
        pygame.event.pump()

        v = deadzone(-joy.get_axis(1), DEADZONE)
        w = deadzone( joy.get_axis(3), DEADZONE)

        left_rpm  = -clamp(v + w, -1, 1) * MAX_RPM
        right_rpm =  clamp(v - w, -1, 1) * MAX_RPM

        now = time.time()
        if now - last_send >= SEND_PERIOD:
            for m in MOTOR_LEFT:
                send_speed(ser, m, left_rpm)
            for m in MOTOR_RIGHT:
                send_speed(ser, m, right_rpm)
            last_send = now

        time.sleep(0.005)

except KeyboardInterrupt:
    print("Saliendo")
    ser.close()
    pygame.quit()
