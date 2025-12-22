import serial
import time

PORT = "/dev/ttyUSB0"
BAUD = 2_000_000

ser = serial.Serial(PORT, BAUD, timeout=0.1)
print("Opened:", ser.portstr)

def read_packet(ser, timeout=1.0):
    start = time.time()
    buf = bytearray()

    while time.time() - start < timeout:
        b = ser.read(1)
        if not b:
            continue

        if not buf:
            if b[0] == 0xAA:
                buf.append(b[0])
        else:
            buf.append(b[0])
            if b[0] == 0x55:
                return bytes(buf)

    return None

# Escuchar RX durante 2 segundos
end = time.time() + 2.0
while time.time() < end:
    pkt = read_packet(ser)
    if pkt:
        print("RX PKT:", pkt.hex())

ser.close()
