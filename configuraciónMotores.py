#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial
import time

PORT = "/dev/ttyUSB0"
BAUDRATE = 38400
ADDR = 0x01

TIMEOUT_S = 0.3
READ_CHUNK = 64
MAX_WAIT_S = 1.0

def hx(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)

def checksum8(data: bytes) -> int:
    return sum(data) & 0xFF

def main():
    # ============================================================
    # PON AQUÍ el comando real que ya te está respondiendo
    # ============================================================
    CMD_READ_POS = 0x00   # <-- CAMBIAR por el CMD que usas
    DATA = []            # <-- CAMBIAR si tu consulta lleva payload

    tx_wo = bytes([0xFA, ADDR & 0xFF, CMD_READ_POS & 0xFF] + [x & 0xFF for x in DATA])
    tx = tx_wo + bytes([checksum8(tx_wo)])

    print(f"Opening {PORT} @ {BAUDRATE} ...")
    with serial.Serial(PORT, BAUDRATE, timeout=TIMEOUT_S, write_timeout=TIMEOUT_S) as ser:
        ser.reset_input_buffer()

        print("TX:", hx(tx))
        ser.write(tx)
        ser.flush()

        t0 = time.time()
        rx_acc = bytearray()

        while True:
            chunk = ser.read(READ_CHUNK)
            if chunk:
                rx_acc.extend(chunk)
                # si llega menos de READ_CHUNK, normalmente ya no hay más
                if len(chunk) < READ_CHUNK:
                    break
            else:
                break

            if (time.time() - t0) > MAX_WAIT_S:
                break

        rx = bytes(rx_acc)
        if not rx:
            print("RX: (vacío)")
            return

        print(f"RX ({len(rx)} bytes):", hx(rx))

        # Inspección mínima (sin asumir estructura)
        print("RX[0:4]:", hx(rx[:4]))
        if len(rx) >= 2:
            print(f"Byte0=0x{rx[0]:02X}, Byte1=0x{rx[1]:02X}")

        # Chequeo hipotético de checksum8 al final
        if len(rx) >= 2:
            calc = checksum8(rx[:-1])
            last = rx[-1]
            print(f"Checksum8 hipotético: calc=0x{calc:02X} last=0x{last:02X} match={calc==last}")

if __name__ == "__main__":
    main()
