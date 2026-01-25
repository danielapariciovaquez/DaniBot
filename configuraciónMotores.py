#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial
import time

PORT = "/dev/ttyUSB0"
BAUDRATE = 38400
ADDR = 0x01

RX_LEN = 64          # lee hasta 64 bytes (ajustable)
TIMEOUT_S = 0.3      # timeout de lectura

def checksum8(frame_wo_chk):
    return sum(frame_wo_chk) & 0xFF

def build_frame(cmd, data):
    frame = [0xFA, ADDR & 0xFF, cmd & 0xFF] + [x & 0xFF for x in data]
    frame.append(checksum8(frame))
    return bytes(frame)

def hx(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)

def main():
    # ============================================================
    # TODO: rellena esto con el comando real de "consulta posición"
    # ============================================================
    CMD_READ_POS = 0x00         # <-- CAMBIAR
    DATA = []                  # <-- CAMBIAR (si aplica)

    tx = build_frame(CMD_READ_POS, DATA)

    print(f"Opening {PORT} @ {BAUDRATE} ...")
    with serial.Serial(PORT, BAUDRATE, timeout=TIMEOUT_S, write_timeout=TIMEOUT_S) as ser:
        ser.reset_input_buffer()

        print("TX:", hx(tx))
        ser.write(tx)
        ser.flush()

        # Lee “lo que llegue” hasta timeout. Una lectura puede traer parcial, así que acumulamos.
        t0 = time.time()
        rx_acc = bytearray()
        while True:
            chunk = ser.read(RX_LEN)
            if chunk:
                rx_acc.extend(chunk)
                # Si dejan de llegar bytes, salimos tras un pequeño margen
                # (para no quedarnos esperando infinitamente si la respuesta es corta).
                if len(chunk) < RX_LEN:
                    break
            else:
                break
            if (time.time() - t0) > 1.0:
                break

        if rx_acc:
            print("RX:", hx(bytes(rx_acc)))
        else:
            print("RX: (vacío)")

if __name__ == "__main__":
    main()
