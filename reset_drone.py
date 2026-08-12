import socket
import time

DRONE_IP = "192.168.169.1" # Wait, the user's drone is LYHFPV_M22. The IP might be 192.168.169.1 or 192.168.1.1. In drone_stream.py it's 192.168.169.1.
DRONE_PORT = 8800

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Let's send reset commands
commands = [
    b'\x01\x67<i=2^bf_resolution=1>',
    b'\x01\x67<i=2^bf_resolution=2>',
    b'\x01\x67<i=2^bf_fps=20>',
    b'\x01\x67<i=2^bf_bitrate=1024>',
]

for cmd in commands:
    print(f"Enviando {cmd}...")
    sock.sendto(cmd, (DRONE_IP, DRONE_PORT))
    time.sleep(0.5)

print("Comandos de reset enviados.")
