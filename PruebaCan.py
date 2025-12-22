import serial

ser = serial.Serial("/dev/ttyUSB1", 2000000)

send_read_speed = bytes([
    0xAA,
    0xC2,
    0x01,
    0x00,
    0x32,
    0x33,
    0x55
])

ser.write(send_read_speed)
