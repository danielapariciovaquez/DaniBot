#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial
import time

PORT = "/dev/ttyUSB0"
BAUDRATE = 38400
ADDR = 0x01

TIMEOUT_S = 0.3

def checksum8(data: bytes) -> int:
    return sum(data) & 0xFF

def hx(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)

def int48_be_signed(b6: bytes) -> int:
    """Convierte 6 bytes big-endian a int48 con signo (two's complement)."""
    if len(b6) != 6:
        raise ValueError("int48 requiere exactamente 6 bytes")
    u = int.from_bytes(b6, byteorder="big", signed=False)
    if u & (1 << 47):  # bit de signo
        u -= (1 << 48)
    return u

def main():
    CMD = 0x31  # Read the cumulative multi-turn encoder value (int48_t)

    tx_wo = bytes([0xFA, ADDR & 0xFF, CMD & 0xFF])
    tx = tx_wo + bytes([checksum8(tx_wo)])

    print(f"Opening {PORT} @ {BAUDRATE} ...")
    with serial.Serial(PORT, BAUDRATE, timeout=TIMEOUT_S, write_timeout=TIMEOUT_S) as ser:
        ser.reset_input_buffer()

        print("TX:", hx(tx))
        ser.write(tx)
        ser.flush()

        # Esperamos el frame típico: FB addr 31 + 6 bytes + CRC = 10 bytes
        rx = ser.read(10)

        if not rx:
            print("RX: (vacío)")
            return

        print(f"RX ({len(rx)} bytes):", hx(rx))

        # Validaciones mínimas (sin suposiciones extra)
        if len(rx) != 10:
            print("WARN: longitud inesperada (esperado 10 bytes para cmd 0x31).")
            return

        if rx[0] != 0xFB or rx[1] != (ADDR & 0xFF) or rx[2] != CMD:
            print("WARN: cabecera/addr/cmd no coinciden con lo esperado.")
            return

        calc = checksum8(rx[:-1])
        if calc != rx[-1]:
            print(f"WARN: checksum no coincide: calc=0x{calc:02X}, rx_last=0x{rx[-1]:02X}")
            return

        value_bytes = rx[3:9]  # 6 bytes int48
        pos_counts = int48_be_signed(value_bytes)
        print("Encoder cumulative (counts):", pos_counts)

        # Si quieres expresar en vueltas (16384 counts/vuelta):
        turns = pos_counts / 16384.0
        print("Turns:", turns)

if __name__ == "__main__":
    main()
