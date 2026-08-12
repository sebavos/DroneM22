import socket
import time

DRONE_IP = "192.168.169.1"
CMD_PORT = 8800
TCP_PORT = 11000

CMD_REGISTER = bytes.fromhex("01 67 3C 69 3D 32 5E 62 66 5F 73 73 69 64 3D 6c 79 68 3e")
CMD_START = bytes.fromhex("EF 00 01 00")

sock_udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
print("[+] Enviando CMD_REGISTER por UDP...")
sock_udp.sendto(CMD_REGISTER, (DRONE_IP, CMD_PORT))
time.sleep(0.5)
print("[+] Enviando CMD_START por UDP...")
sock_udp.sendto(CMD_START, (DRONE_IP, CMD_PORT))
time.sleep(1.0)

print("[+] Intentando conectar a TCP 11000...")
try:
    sock_tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock_tcp.settimeout(3.0)
    sock_tcp.connect((DRONE_IP, TCP_PORT))
    print("[>>>] ¡ÉXITO! TCP 11000 SE ABRIÓ TRAS EL REGISTRO!")
    sock_tcp.close()
except Exception as e:
    print(f"[-] Fallo TCP 11000: {e}")

print("[+] Intentando conectar a TCP 8800...")
try:
    sock_tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock_tcp.settimeout(3.0)
    sock_tcp.connect((DRONE_IP, 8800))
    print("[>>>] ¡ÉXITO! TCP 8800 SE ABRIÓ TRAS EL REGISTRO!")
    sock_tcp.close()
except Exception as e:
    print(f"[-] Fallo TCP 8800: {e}")
