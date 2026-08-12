import struct

with open('/home/uavlab1/dump.bin', 'rb') as f:
    data = f.read()

idx = data.find(b'\x00\x00\x00\x01')
if idx != -1:
    print(f"NAL UNIT FOUND at offset {idx}!")
    print(f"Bytes around it: {data[max(0, idx-16):idx+16].hex()}")
else:
    print("NO NAL UNIT FOUND.")
