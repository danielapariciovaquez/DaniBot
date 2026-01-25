#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import serial

PORT = "/dev/ttyUSB0"
BAUDRATE = 38400
ADDR = 0x01

TIMEOUT_S = 0.25
RX_LEN = 64
INTER_FRAME_DELAY = 0.015  # 15 ms de gap entre tramas (ajustable)

def checksum8(data: bytes) -> int:
    return sum(data) & 0xFF

def hx(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)

def build(cmd: int, data: list[int]) -> bytes:
    wo = bytes([0xFA, ADDR & 0xFF, cmd & 0xFF] + [x & 0xFF for x in data])
    return wo + bytes([checksum8(wo)])

def int48_be_signed(b6: bytes) -> int:
    u = int.from_bytes(b6, "big", signed=False)
    if u & (1 << 47):
        u -= (1 << 48)
    return u

class Link:
    def __init__(self):
        self.ser = None

    def open(self):
        self.close()
        self.ser = serial.Serial(
            PORT, BAUDRATE,
            timeout=TIMEOUT_S,
            write_timeout=TIMEOUT_S,
            rtscts=False,
            dsrdtr=False,
        )
        # Forzamos estados (algunos adaptadores se comportan mejor así)
        try:
            self.ser.rts = False
            self.ser.dtr = False
        except Exception:
            pass
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        print(f"[INFO] Open {PORT} @ {BAUDRATE}")

    def close(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def txrx(self, tx: bytes, expect_min: int = 0) -> bytes:
        if self.ser is None or not self.ser.is_open:
            self.open()
        s = self.ser
        s.reset_input_buffer()
        print("TX:", hx(tx))
        s.write(tx)
        s.flush()
        time.sleep(INTER_FRAME_DELAY)
        rx = s.read(RX_LEN)
        if rx:
            print("RX:", hx(rx))
        else:
            print("RX: (vacío)")
        if expect_min and len(rx) < expect_min:
            return b""
        return rx

def send_f6(link: Link, rpm_signed: int):
    rpm = int(rpm_signed)
    direction = 1 if rpm < 0 else 0
    speed = abs(rpm)
    if speed > 3000:
        speed = 3000
    acc = 2

    byte4 = ((direction & 1) << 7) | ((speed >> 8) & 0x0F)
    byte5 = speed & 0xFF
    tx = build(0xF6, [byte4, byte5, acc])
    link.txrx(tx)  # no exigimos RX aquí

def read_31(link: Link) -> int | None:
    tx = build(0x31, [])
    rx = link.txrx(tx, expect_min=10)
    if len(rx) != 10:
        return None
    if rx[0] != 0xFB or rx[1] != (ADDR & 0xFF) or rx[2] != 0x31:
        return None
    if checksum8(rx[:-1]) != rx[-1]:
        return None
    pos = int48_be_signed(rx[3:9])
    return pos

def main():
    link = Link()
    link.open()

    # Secuencia: mandamos varias consignas y comprobamos 0x31 después de cada una
    seq = [50, 100, 150, 200, 100, 0, -80, 0]

    for rpm in seq:
        print(f"\n=== Set RPM {rpm} ===")
        try:
            send_f6(link, rpm)
            pos = read_31(link)
            if pos is None:
                print("[FAIL] 0x31 no responde/ inválido. Reabriendo puerto y reintentando...")
                link.open()
                pos2 = read_31(link)
                print("[RETRY] pos:", pos2)
            else:
                print("[OK] pos counts:", pos)
        except Exception as e:
            print("[EXC]", e)
            print("[ACTION] Reopen port")
            link.open()

        time.sleep(0.2)

if __name__ == "__main__":
    main()
