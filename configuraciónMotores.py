import serial
import time

PORT = "/dev/ttyUSB0"
BAUDRATE = 38400

# Abrir puerto
ser = serial.Serial(
    port=PORT,
    baudrate=BAUDRATE,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=0.2,
    write_timeout=0.2,
)

def send(frame_hex: str):
    frame = bytes(int(b, 16) for b in frame_hex.split())
    ser.write(frame)
    ser.flush()
    print("TX:", frame_hex)
    time.sleep(0.05)
    rx = ser.read(16)
    print("RX:", " ".join(f"{b:02X}" for b in rx) if rx else "(vacío)")

try:
    # ENABLE motor (BUS control)
    send("FA 01 F3 01 EF")

    time.sleep(0.2)

    # F6: velocidad +50 RPM, ACC=2
    send("FA 01 F6 00 32 02 25")

    # Mantener 2 segundos
    time.sleep(2.0)

    # STOP (0 RPM)
    send("FA 01 F6 00 00 02 F9")

finally:
    ser.close()
