"""
Explorador de comandos UDP del dron M22 (LYHFPV).

Solo se conocen 2 comandos:
    EF 00 01 00  -> enciende el transmisor de video
    EF 00 04 00  -> keepalive / heartbeat

Todo el espacio EF 00 xx yy esta sin explorar: ahi pueden estar resolución,
framerate, foto, grabación, gimbal, etc. Este script barre ese espacio de
forma controlada y registra cualquier cambio observable.

    python3 explorar_comandos.py              # ejecutar barrido
    python3 explorar_comandos.py --dry-run    # solo mostrar qué haría
    python3 explorar_comandos.py --range 0x05 0x20  # rango personalizado

IMPORTANTE: ejecutar en la Jetson con el dron encendido y video activo.
"""

import argparse
import csv
import os
import socket
import struct
import sys
import time
import threading

DRONE_IP = "192.168.169.1"
DRONE_PORT = 8800
LOCAL_PORT = 8804

CMD_START = bytes.fromhex("EF 00 01 00")
CMD_HEARTBEAT = bytes.fromhex("EF 00 04 00")

# Offsets del paquete de video (para detectar cambios)
POS_CHUNK_IDX = 32
POS_CHUNK_TOTAL = 36
POS_FRAME_SIZE = 40
POS_WIDTH = 44
POS_HEIGHT = 46
OFF_PAYLOAD = 56

KNOWN_COMMANDS = {0x01, 0x04}

# Estado compartido entre hilos
monitor = {
    "packets": 0,
    "frames": 0,
    "width": 0,
    "height": 0,
    "last_frame_time": 0.0,
    "running": True,
}
monitor_lock = threading.Lock()


def keepalive_loop(sock):
    """Mantiene el dron despierto mientras se explora."""
    while monitor["running"]:
        try:
            sock.sendto(CMD_HEARTBEAT, (DRONE_IP, DRONE_PORT))
        except Exception:
            pass
        time.sleep(0.1)


def packet_monitor(sock):
    """Cuenta paquetes y cuadros para detectar cambios tras cada comando."""
    sock.settimeout(0.2)
    chunks = {}
    while monitor["running"]:
        try:
            data, _ = sock.recvfrom(65535)
        except socket.timeout:
            continue
        except Exception:
            time.sleep(0.01)
            continue

        if len(data) < OFF_PAYLOAD:
            continue

        with monitor_lock:
            monitor["packets"] += 1
            idx = data[POS_CHUNK_IDX]
            if idx == 0:
                if chunks:
                    monitor["frames"] += 1
                    monitor["last_frame_time"] = time.time()
                chunks = {}
                w = struct.unpack_from("<H", data, POS_WIDTH)[0]
                h = struct.unpack_from("<H", data, POS_HEIGHT)[0]
                monitor["width"] = w
                monitor["height"] = h
            chunks[idx] = True


def measure_baseline(duration=5.0):
    """Mide FPS y resolución de base durante 'duration' segundos."""
    with monitor_lock:
        p0 = monitor["packets"]
        f0 = monitor["frames"]
    t0 = time.time()
    time.sleep(duration)
    with monitor_lock:
        p1 = monitor["packets"]
        f1 = monitor["frames"]
        w = monitor["width"]
        h = monitor["height"]
    dt = time.time() - t0
    return {
        "fps": (f1 - f0) / dt if dt > 0 else 0,
        "pps": (p1 - p0) / dt if dt > 0 else 0,
        "width": w,
        "height": h,
    }


def send_probe(tx_sock, cmd_byte, param_byte=0x00):
    """Envía un comando de prueba y mide el efecto."""
    cmd = bytes([0xEF, 0x00, cmd_byte, param_byte])
    tx_sock.sendto(cmd, (DRONE_IP, DRONE_PORT))
    return cmd.hex()


def main():
    parser = argparse.ArgumentParser(
        description="Explorador de comandos UDP del dron M22"
    )
    parser.add_argument("--dry-run", action="store_true",
                        help="Solo mostrar qué comandos se probarían")
    parser.add_argument("--range", nargs=2, type=lambda x: int(x, 0),
                        default=[0x00, 0xFF],
                        metavar=("START", "END"),
                        help="Rango de bytes a explorar (hex, ej: 0x05 0x20)")
    parser.add_argument("--params", nargs="*", type=lambda x: int(x, 0),
                        default=[0x00],
                        metavar="BYTE",
                        help="Valores del 4to byte a probar (hex, ej: 0x00 0x01 0x02)")
    parser.add_argument("--wait", type=float, default=3.0,
                        help="Segundos de medición tras cada comando (default: 3)")
    parser.add_argument("--output", default="explorar_resultados.csv",
                        help="Archivo CSV de salida")
    args = parser.parse_args()

    start_byte, end_byte = args.range
    commands_to_try = []
    for cmd_b in range(start_byte, end_byte + 1):
        if cmd_b in KNOWN_COMMANDS:
            continue
        for param_b in args.params:
            commands_to_try.append((cmd_b, param_b))

    print("=" * 60)
    print(" Explorador de Comandos — Dron M22 (LYHFPV)")
    print(" Rango: 0x%02X..0x%02X, params: %s"
          % (start_byte, end_byte, [hex(p) for p in args.params]))
    print(" Comandos a probar: %d (saltando 0x01 y 0x04 conocidos)" % len(commands_to_try))
    print(" Espera por comando: %.1f s" % args.wait)
    print("=" * 60)

    if args.dry_run:
        print("\n[DRY RUN] Comandos que se enviarían:\n")
        for cmd_b, param_b in commands_to_try:
            print("  EF 00 %02X %02X" % (cmd_b, param_b))
        print("\nTotal: %d comandos. Tiempo estimado: %.0f s"
              % (len(commands_to_try), len(commands_to_try) * (args.wait + 0.5)))
        return 0

    # Crear sockets
    rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx_sock.bind(("0.0.0.0", LOCAL_PORT))

    tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Arrancar hilos de soporte
    threading.Thread(target=keepalive_loop, args=(tx_sock,), daemon=True).start()
    threading.Thread(target=packet_monitor, args=(rx_sock,), daemon=True).start()

    # Encender cámara
    print("\n[*] Encendiendo cámara...")
    tx_sock.sendto(CMD_START, (DRONE_IP, DRONE_PORT))
    time.sleep(1.0)

    # Medir baseline
    print("[*] Midiendo baseline (5 s)...")
    baseline = measure_baseline(5.0)
    print("    Baseline: %.1f fps, %.0f pps, %dx%d"
          % (baseline["fps"], baseline["pps"], baseline["width"], baseline["height"]))

    # Abrir CSV
    csv_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), args.output)
    csvfile = open(csv_path, "w", newline="")
    writer = csv.writer(csvfile)
    writer.writerow([
        "comando_hex", "cmd_byte", "param_byte",
        "fps_antes", "fps_despues", "delta_fps",
        "pps_antes", "pps_despues",
        "res_antes", "res_despues",
        "nota"
    ])

    print("\n[*] Iniciando exploración...\n")
    for i, (cmd_b, param_b) in enumerate(commands_to_try):
        # Medir estado actual rápido (1 s)
        pre = measure_baseline(1.0)

        # Enviar comando de prueba
        cmd_hex = send_probe(tx_sock, cmd_b, param_b)
        sys.stdout.write("  [%3d/%d] EF 00 %02X %02X ... "
                         % (i + 1, len(commands_to_try), cmd_b, param_b))
        sys.stdout.flush()

        # Esperar y medir efecto
        time.sleep(0.5)
        post = measure_baseline(args.wait)

        # Detectar cambios
        delta_fps = post["fps"] - pre["fps"]
        res_antes = "%dx%d" % (pre["width"], pre["height"])
        res_despues = "%dx%d" % (post["width"], post["height"])

        notas = []
        if abs(delta_fps) > 2.0:
            notas.append("FPS_CHANGE")
        if res_antes != res_despues:
            notas.append("RES_CHANGE")
        if post["fps"] < 0.5 and pre["fps"] > 1.0:
            notas.append("VIDEO_STOPPED")
            # Re-encender cámara
            tx_sock.sendto(CMD_START, (DRONE_IP, DRONE_PORT))
            time.sleep(1.0)
        if post["fps"] > baseline["fps"] * 1.5:
            notas.append("SIGNIFICANT_IMPROVEMENT")

        nota_str = ",".join(notas) if notas else "sin_cambio"
        marker = " <<<" if notas else ""
        print("fps=%.1f->%.1f  res=%s->%s  %s%s"
              % (pre["fps"], post["fps"], res_antes, res_despues, nota_str, marker))

        writer.writerow([
            cmd_hex, "0x%02X" % cmd_b, "0x%02X" % param_b,
            "%.2f" % pre["fps"], "%.2f" % post["fps"], "%.2f" % delta_fps,
            "%.0f" % pre["pps"], "%.0f" % post["pps"],
            res_antes, res_despues,
            nota_str
        ])
        csvfile.flush()

    monitor["running"] = False
    csvfile.close()
    rx_sock.close()
    tx_sock.close()

    print("\n" + "=" * 60)
    print(" Exploración completada. Resultados en: %s" % csv_path)
    print(" Baseline fue: %.1f fps, %dx%d"
          % (baseline["fps"], baseline["width"], baseline["height"]))
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())
