#!/usr/bin/env python3
import serial
import time
from typing import Optional, Tuple, List

# =====================================================
# CONFIGURACIÓN USB-CAN (tu convertidor)
# =====================================================
PORT = "/dev/ttyUSB0"
BAUDRATE = 2_000_000

MOTORS = [0x01, 0x02, 0x03, 0x04]

# =====================================================
# OBJETIVO
# =====================================================
# 1) Forzar modo SR_vFOC (Bus FOC) -> comando 0x82, mode=0x05
# 2) Escribir PID vFOC -> comando 0x96
KP = 1000
KI = 0
KD = 0
KV = 1000

# Timings
READ_TIMEOUT_S = 0.25
INTER_CMD_DELAY_S = 0.03

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
    - Ctrl = 0xC0 | DLC
    - DLC CAN incluye el CRC CAN como último byte del payload CAN
    """
    dlc = len(data) + 1  # +1 CRC CAN
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
# PARSER SIMPLE DE RESPUESTAS (AA ... 55)
# =====================================================
class UsbCanParser:
    def __init__(self):
        self.buf = bytearray()

    def feed(self, chunk: bytes):
        self.buf.extend(chunk)

    def next_frame(self) -> Optional[Tuple[int, List[int]]]:
        while True:
            try:
                i = self.buf.index(0xAA)
            except ValueError:
                self.buf.clear()
                return None

            if i > 0:
                del self.buf[:i]

            if len(self.buf) < 6:
                return None

            ctrl = self.buf[1]
            if (ctrl & 0xF0) != 0xC0:
                del self.buf[0]
                continue

            dlc = ctrl & 0x0F
            total_len = 2 + 2 + dlc + 1  # AA,ctrl + idL,idH + dlc + 55
            if len(self.buf) < total_len:
                return None

            if self.buf[total_len - 1] != 0x55:
                del self.buf[0]
                continue

            can_id = self.buf[2] | (self.buf[3] << 8)
            can_payload = list(self.buf[4:4 + dlc])  # incluye CRC CAN
            del self.buf[:total_len]

            if dlc < 1:
                continue

            data_wo_crc = can_payload[:-1]
            rx_crc = can_payload[-1]
            exp_crc = can_crc(can_id, data_wo_crc)
            if rx_crc != exp_crc:
                continue

            return can_id, data_wo_crc

def read_status_reply(ser: serial.Serial, expect_id: int, expect_code: int, timeout_s: float) -> Optional[int]:
    """
    Espera respuesta típica: [code, status]
    Devuelve status o None si no llega / no cuadra.
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
        if data[0] != (expect_code & 0xFF):
            continue
        return data[1]

    return None

# =====================================================
# COMANDOS SEGÚN MANUAL V1.0.9
# =====================================================
def set_work_mode_sr_vfoc(ser: serial.Serial, can_id: int) -> Optional[int]:
    """
    Code 0x82: Set working mode
    mode 0x05: SR_vFOC (Bus Interface FOC Mode)
    Respuesta: [0x82, status]
    """
    CODE = 0x82
    MODE_SR_vFOC = 0x05
    ser.reset_input_buffer()
    ser.write(build_usbcan_frame(can_id, [CODE, MODE_SR_vFOC]))
    return read_status_reply(ser, can_id, CODE, READ_TIMEOUT_S)

def write_pid_vfoc_kp_ki(ser: serial.Serial, can_id: int, kp: int, ki: int) -> Optional[int]:
    """
    Code 0x96: PID in vFOC
    CMD  0x00: set Kp, Ki (uint16)
    Respuesta: [0x96, status]
    """
    CODE = 0x96
    kp = clamp(kp, 0, 1024)
    ki = clamp(ki, 0, 1024)
    kp_lo, kp_hi = u16_le(kp)
    ki_lo, ki_hi = u16_le(ki)

    ser.reset_input_buffer()
    ser.write(build_usbcan_frame(can_id, [CODE, 0x00, kp_lo, kp_hi, ki_lo, ki_hi]))
    return read_status_reply(ser, can_id, CODE, READ_TIMEOUT_S)

def write_pid_vfoc_kd_kv(ser: serial.Serial, can_id: int, kd: int, kv: int) -> Optional[int]:
    """
    Code 0x96: PID in vFOC
    CMD  0x01: set Kd, Kv (uint16)
    Respuesta: [0x96, status]
    """
    CODE = 0x96
    kd = clamp(kd, 0, 1024)
    kv = clamp(kv, 0, 1024)
    kd_lo, kd_hi = u16_le(kd)
    kv_lo, kv_hi = u16_le(kv)

    ser.reset_input_buffer()
    ser.write(build_usbcan_frame(can_id, [CODE, 0x01, kd_lo, kd_hi, kv_lo, kv_hi]))
    return read_status_reply(ser, can_id, CODE, READ_TIMEOUT_S)

# =====================================================
# MAIN
# =====================================================
def main():
    print(f"Abriendo USB-CAN en {PORT} @ {BAUDRATE}...")
    ser = serial.Serial(PORT, BAUDRATE, timeout=0)
    time.sleep(0.2)

    print(f"Motores: {MOTORS}")
    print("Configurando modo SR_vFOC (FOC por bus) y escribiendo PID vFOC...")
    print(f"PID: Kp={KP} Ki={KI} Kd={KD} Kv={KV}")

    try:
        for mid in MOTORS:
            st_mode = set_work_mode_sr_vfoc(ser, mid)
            time.sleep(INTER_CMD_DELAY_S)

            st_kpki = write_pid_vfoc_kp_ki(ser, mid, KP, KI)
            time.sleep(INTER_CMD_DELAY_S)

            st_kdkv = write_pid_vfoc_kd_kv(ser, mid, KD, KV)
            time.sleep(INTER_CMD_DELAY_S)

            print(f"Motor {mid:02d}: set_mode(0x82->0x05) status={st_mode} | "
                  f"PID(Kp/Ki) status={st_kpki} | PID(Kd/Kv) status={st_kdkv}")

    finally:
        ser.close()
        print("Cerrado.")

if __name__ == "__main__":
    main()
