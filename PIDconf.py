#!/usr/bin/env python3
import serial
import time
from typing import Optional, Tuple, List

# =====================================================
# CONFIGURACIÓN USB-CAN (tu convertidor)
# =====================================================
PORT = "/dev/ttyUSB0"
BAUDRATE = 2_000_000

# Motores
MOTORS = [0x01, 0x02, 0x03, 0x04]

# =====================================================
# PID A ESCRIBIR
# =====================================================
KP = 1000
KI = 0
KD = 0
KV = 1000

# Elegir modo PID según el manual:
#   vFOC  -> code 0x96
#   CLOSE -> code 0x97
PID_MODE = "vFOC"   # "vFOC" o "CLOSE"

# Timeouts
READ_TIMEOUT_S = 0.20
INTER_CMD_DELAY_S = 0.02

# =====================================================
# UTILIDADES
# =====================================================
def u16_le(x: int) -> Tuple[int, int]:
    x = int(x) & 0xFFFF
    return (x & 0xFF, (x >> 8) & 0xFF)

def clamp(x: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, int(x)))

def can_crc(can_id: int, data: List[int]) -> int:
    # Manual: CRC = (CAN_ID + sum(data_bytes_without_crc)) & 0xFF
    return (int(can_id) + sum(int(b) & 0xFF for b in data)) & 0xFF

def build_usbcan_frame(can_id: int, data: List[int]) -> bytes:
    """
    Encapsulado del convertidor USB-CAN (variable protocol).
    - Header: 0xAA
    - Ctrl: 0xC0 | DLC
    - CAN ID: little-endian (low, high)
    - DATA: bytes CAN (sin CRC)
    - CRC: 8bit
    - Tail: 0x55
    """
    dlc = len(data) + 1  # +1 por el CRC CAN
    if not (0 <= can_id <= 0x7FF):
        raise ValueError("CAN_ID fuera de 11 bits (0..0x7FF)")
    if not (0 <= dlc <= 8):
        raise ValueError("DLC CAN inválido (0..8)")

    crc = can_crc(can_id, data)
    ctrl = 0xC0 | (dlc & 0x0F)

    return bytes([
        0xAA,
        ctrl,
        can_id & 0xFF,
        (can_id >> 8) & 0xFF,
        *data,
        crc,
        0x55
    ])

# =====================================================
# PARSER DE RESPUESTAS DEL CONVERTIDOR (robusto)
# =====================================================
class UsbCanParser:
    def __init__(self):
        self.buf = bytearray()

    def feed(self, chunk: bytes):
        self.buf.extend(chunk)

    def next_frame(self) -> Optional[Tuple[int, List[int]]]:
        """
        Devuelve (can_id, data_with_code_and_payload_without_crc) si hay un frame completo.
        Asume el mismo encapsulado AA ... 55 y ctrl = 0xC0|dlc.
        """
        while True:
            # Buscar cabecera 0xAA
            try:
                i = self.buf.index(0xAA)
            except ValueError:
                self.buf.clear()
                return None

            # Descarta basura antes de 0xAA
            if i > 0:
                del self.buf[:i]

            if len(self.buf) < 6:
                return None  # mínimo para seguir

            ctrl = self.buf[1]
            if (ctrl & 0xF0) != 0xC0:
                # cabecera falsa, descartar 0xAA
                del self.buf[0]
                continue

            dlc = ctrl & 0x0F
            # Longitud total = AA + ctrl + idL + idH + (dlc bytes CAN) + 55
            # Ojo: dlc incluye CRC CAN como último byte del payload CAN
            total_len = 2 + 2 + dlc + 1
            if len(self.buf) < total_len:
                return None

            if self.buf[total_len - 1] != 0x55:
                # Frame corrupto, descartar 0xAA y reintentar
                del self.buf[0]
                continue

            can_id = self.buf[2] | (self.buf[3] << 8)
            can_payload = list(self.buf[4:4 + dlc])  # incluye CRC CAN al final
            del self.buf[:total_len]

            if dlc < 1:
                continue

            data_wo_crc = can_payload[:-1]
            rx_crc = can_payload[-1]
            exp_crc = can_crc(can_id, data_wo_crc)
            if rx_crc != exp_crc:
                # CRC malo: descartar y continuar (no re-lanzar, queremos robustez)
                continue

            return can_id, data_wo_crc

def read_one_reply(ser: serial.Serial, expect_id: int, expect_code: int, timeout_s: float) -> Optional[int]:
    """
    Espera una respuesta del motor con (CAN_ID=expect_id, code=expect_code).
    Devuelve 'status' si el frame tiene formato [code, status], si no, None.
    """
    parser = UsbCanParser()
    t0 = time.time()

    while time.time() - t0 < timeout_s:
        if ser.in_waiting:
            parser.feed(ser.read(ser.in_waiting))

        fr = parser.next_frame()
        if fr is None:
            time.sleep(0.001)
            continue

        can_id, data = fr
        if can_id != expect_id:
            continue
        if len(data) < 2:
            continue
        code = data[0]
        if code != (expect_code & 0xFF):
            continue

        status = data[1]
        return status

    return None

# =====================================================
# COMANDOS PID (manual V1.0.9)
# =====================================================
def pid_codes() -> int:
    if PID_MODE.upper() == "VFOC":
        return 0x96
    if PID_MODE.upper() == "CLOSE":
        return 0x97
    raise ValueError("PID_MODE debe ser 'vFOC' o 'CLOSE'")

def write_kp_ki(ser: serial.Serial, can_id: int, kp: int, ki: int) -> Optional[int]:
    code = pid_codes()
    kp = clamp(kp, 0, 1024)
    ki = clamp(ki, 0, 1024)
    kp_lo, kp_hi = u16_le(kp)
    ki_lo, ki_hi = u16_le(ki)

    data = [code, 0x00, kp_lo, kp_hi, ki_lo, ki_hi]
    ser.reset_input_buffer()
    ser.write(build_usbcan_frame(can_id, data))
    return read_one_reply(ser, can_id, code, READ_TIMEOUT_S)

def write_kd_kv(ser: serial.Serial, can_id: int, kd: int, kv: int) -> Optional[int]:
    code = pid_codes()
    kd = clamp(kd, 0, 1024)
    kv = clamp(kv, 0, 1024)
    kd_lo, kd_hi = u16_le(kd)
    kv_lo, kv_hi = u16_le(kv)

    data = [code, 0x01, kd_lo, kd_hi, kv_lo, kv_hi]
    ser.reset_input_buffer()
    ser.write(build_usbcan_frame(can_id, data))
    return read_one_reply(ser, can_id, code, READ_TIMEOUT_S)

# =====================================================
# MAIN
# =====================================================
def main():
    print(f"Abriendo USB-CAN en {PORT} @ {BAUDRATE}...")
    ser = serial.Serial(PORT, BAUDRATE, timeout=0)
    time.sleep(0.2)
    print(f"PID_MODE={PID_MODE} | KP={KP} KI={KI} KD={KD} KV={KV}")
    print(f"Motores: {MOTORS}")

    try:
        for mid in MOTORS:
            s1 = write_kp_ki(ser, mid, KP, KI)
            time.sleep(INTER_CMD_DELAY_S)
            s2 = write_kd_kv(ser, mid, KD, KV)
            time.sleep(INTER_CMD_DELAY_S)

            # Según manual: status=1 success, status=0 fail
            print(f"Motor {mid:02d}: KP/KI status={s1} | KD/KV status={s2}")

    finally:
        ser.close()
        print("Cerrado.")

if __name__ == "__main__":
    main()
