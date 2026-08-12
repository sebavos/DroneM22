import socket
import time
import struct
import threading

DRONE_IP = "192.168.169.1"
CMD_PORT = 8800
UDP_PORT = 8804
DRONE_SSID = "LYHFPV_M22 2B967C"

CMD_REGISTER = b'\x01\x67<i=2^bf_ssid=' + DRONE_SSID.encode('ascii') + b'>'
CMD_START = bytes.fromhex("EF 00 01 00")
CMD_HEARTBEAT = bytes.fromhex("EF 00 04 00")
CMD_BITRATE = b"\x01\x67<i>=2^bf_bitrate=2048>"
CMD_FPS = b"\x01\x67<i>=2^bf_fps=30>"

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(("0.0.0.0", UDP_PORT))
sock.settimeout(2.0)

def keepalive():
    sock.sendto(CMD_REGISTER, (DRONE_IP, CMD_PORT))
    time.sleep(0.1)
    sock.sendto(CMD_START, (DRONE_IP, CMD_PORT))
    while True:
        sock.sendto(CMD_HEARTBEAT, (DRONE_IP, CMD_PORT))
        time.sleep(0.1)

t = threading.Thread(target=keepalive, daemon=True)
t.start()

print("Esperando 10 paquetes...")
with open("dump.bin", "wb") as f:
    for _ in range(10):
        try:
            data, addr = sock.recvfrom(65536)
            f.write(data)
            print(f"RECIBIDO: len={len(data)}")
            print(f"Primeros 32 bytes: {data[:32].hex()}")
        except socket.timeout:
            print("Timeout")
            break
