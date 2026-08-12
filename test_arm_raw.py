#!/usr/bin/env python3
"""
Test MINIMO de armado del dron M22.

Envia el paquete exacto 66 80 80 80 80 40 40 99
a ambos puertos (8080 y 8800) para ver cual desbloquea el dron.

Uso:
    python3 test_arm_raw.py

Observar los LEDs del dron:
  - Si DEJAN de parpadear = el puerto correcto fue encontrado.
"""

import socket
import time
import sys

DRONE_IP = "192.168.169.1"

# Paquete exacto de la app: 66 80 80 80 80 40 40 99
ARM_PACKET = bytes([0x66, 0x80, 0x80, 0x80, 0x80, 0x40, 0x40, 0x99])

# Paquete neutral (sin flag): 66 80 80 80 80 00 00 99
NEUTRAL_PACKET = bytes([0x66, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x99])

# Heartbeat para mantener vivo el enlace
HB_PACKET = bytes.fromhex("EF000400")

# SSID de registro
SSID = "LYHFPV_M22 2B967C"

def build_ssid_register(ssid):
    """Construye el comando de registro SSID."""
    payload = "<i>=2^ssid=" + ssid + ">"
    return bytes([0x01, 0x67]) + payload.encode("ascii")

def test_port(port, duration=3.0):
    """Envia el paquete ARM a un puerto especifico durante 'duration' segundos."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    print(f"\n{'='*50}")
    print(f"  PROBANDO PUERTO {port}")
    print(f"  Paquete: {ARM_PACKET.hex().upper()}")
    print(f"  Duracion: {duration}s a 50Hz")
    print(f"{'='*50}")
    
    count = 0
    t0 = time.time()
    while (time.time() - t0) < duration:
        sock.sendto(ARM_PACKET, (DRONE_IP, port))
        count += 1
        time.sleep(0.02)  # 50Hz
    
    print(f"  Enviados: {count} paquetes")
    print(f"  >>> MIRA LOS LEDS DEL DRON <<<")
    
    # Mantener neutral por 2 segundos para no perder el enlace
    print(f"  Enviando neutral por 2s...")
    t0 = time.time()
    while (time.time() - t0) < 2.0:
        sock.sendto(NEUTRAL_PACKET, (DRONE_IP, port))
        time.sleep(0.02)
    
    sock.close()

def test_shared_socket(duration=3.0):
    """Prueba enviando desde el mismo socket que recibe video (puerto 8804)."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", 8804))
    
    print(f"\n{'='*50}")
    print(f"  PROBANDO DESDE SOCKET 8804 (compartido con video)")
    print(f"{'='*50}")
    
    # Primero registrar SSID y arrancar video
    reg = build_ssid_register(SSID)
    print(f"  Registrando SSID...")
    sock.sendto(reg, (DRONE_IP, 8800))
    time.sleep(0.1)
    sock.sendto(bytes.fromhex("EF000100"), (DRONE_IP, 8800))  # CMD_START
    time.sleep(0.1)
    
    # Enviar heartbeats por 1 segundo para establecer enlace
    print(f"  Enviando heartbeats por 1s...")
    t0 = time.time()
    while (time.time() - t0) < 1.0:
        sock.sendto(HB_PACKET, (DRONE_IP, 8800))
        time.sleep(0.1)
    
    # Ahora enviar ARM al puerto 8080
    print(f"  Enviando ARM a puerto 8080 por {duration}s...")
    count = 0
    t0 = time.time()
    while (time.time() - t0) < duration:
        sock.sendto(ARM_PACKET, (DRONE_IP, 8080))
        count += 1
        time.sleep(0.02)
    print(f"  Enviados: {count} paquetes ARM a 8080")
    print(f"  >>> MIRA LOS LEDS <<<")
    
    # Ahora probar ARM al puerto 8800
    print(f"\n  Enviando ARM a puerto 8800 por {duration}s...")
    count = 0
    t0 = time.time()
    while (time.time() - t0) < duration:
        sock.sendto(ARM_PACKET, (DRONE_IP, 8800))
        count += 1
        time.sleep(0.02)
    print(f"  Enviados: {count} paquetes ARM a 8800")
    print(f"  >>> MIRA LOS LEDS <<<")
    
    # Neutral
    print(f"  Enviando neutral...")
    t0 = time.time()
    while (time.time() - t0) < 2.0:
        sock.sendto(NEUTRAL_PACKET, (DRONE_IP, 8080))
        sock.sendto(HB_PACKET, (DRONE_IP, 8800))
        time.sleep(0.05)
    
    sock.close()


def main():
    print("\n" + "=" * 50)
    print("  TEST DE ARMADO RAW — DRON M22")
    print("  Paquete ARM: 66 80 80 80 80 40 40 99")
    print("=" * 50)
    
    print("\nOpciones:")
    print("  1) Probar puerto 8080 (RC)")
    print("  2) Probar puerto 8800 (CMD)")
    print("  3) Probar AMBOS puertos por separado")
    print("  4) Probar desde socket compartido 8804 (como la app)")
    print("  5) Salir")
    
    try:
        opcion = input("\nElige [1-5]: ").strip()
    except (EOFError, KeyboardInterrupt):
        return
    
    if opcion == "1":
        test_port(8080, duration=3.0)
    elif opcion == "2":
        test_port(8800, duration=3.0)
    elif opcion == "3":
        input("\n  Observa los LEDs. Presiona ENTER para probar puerto 8080...")
        test_port(8080, duration=3.0)
        input("\n  ¿Dejaron de parpadear? Presiona ENTER para probar puerto 8800...")
        test_port(8800, duration=3.0)
    elif opcion == "4":
        test_shared_socket(duration=3.0)
    elif opcion == "5":
        return
    else:
        print("Opcion invalida")
        return
    
    print("\n" + "=" * 50)
    print("  RESULTADO:")
    print("  - Si los LEDs dejaron de parpadear → ARMADO EXITOSO")
    print("  - Si siguen parpadeando → Probar otra opcion")
    print("=" * 50)


if __name__ == "__main__":
    main()
