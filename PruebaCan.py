# QUERY STATUS
send_can(0x01, [0xF1, 0xF2])

time.sleep(0.2)

rx = ser.read(128)
print("RX RAW:", rx.hex())
