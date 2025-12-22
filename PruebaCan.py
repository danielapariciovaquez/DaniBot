import serial

class USB_CAN_A:
    def __init__(self, port="/dev/ttyUSB0", baud=2_000_000):
        self.ser = serial.Serial(port, baud, timeout=1)

    def send_can_std(self, can_id: int, data: list[int]):
        dlc = len(data)
        typ = 0xE0 | dlc          # firmware Waveshare (el tuyo)
        pkt = (
            bytes([0xAA, typ, can_id & 0xFF, (can_id >> 8) & 0xFF])
            + bytes(data)
            + bytes([0x55])
        )
        self.ser.write(pkt)

    def close(self):
        self.ser.close()
