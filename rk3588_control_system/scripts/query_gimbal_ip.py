import binascii
import time

import serial

port = "/dev/ttyS3"
baud = 115200
payload = b'{"sh":"ifconfig"}'

frame = bytearray(44)
frame[0] = 0xFB
frame[1] = 0x2C
frame[2] = 0x90
frame[3] = len(payload) & 0xFF
frame[4] = (len(payload) >> 8) & 0xFF
frame[7:7 + len(payload)] = payload[:35]
checksum = 0
for index in range(2, 42):
    checksum ^= frame[index]
frame[42] = checksum & 0xFF
frame[43] = 0xF0

print("tx", binascii.hexlify(frame).decode(), payload.decode())

ser = serial.Serial(port, baud, timeout=0.2, write_timeout=1)
ser.reset_input_buffer()
for _ in range(5):
    ser.write(frame)
    ser.flush()
    time.sleep(0.08)

deadline = time.time() + 3
buf = bytearray()
while time.time() < deadline:
    data = ser.read(512)
    if data:
        buf.extend(data)
        deadline = time.time() + 0.5
ser.close()

print("rx_len", len(buf))
print("rx_hex", binascii.hexlify(buf).decode())
print("rx_ascii", buf.decode("utf-8", "replace"))
