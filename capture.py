"""
Captura cruda de paquetes UDP del dron M22 (LYHFPV) para analisis de formato.
No decodifica nada: guarda los cuadros ensamblados tal cual llegan.
"""
import socket, time, threading, os, sys, collections

DRONE_IP   = "192.168.169.1"
DRONE_PORT = 8800
LOCAL_PORT = 8804
OUTDIR     = "/home/uavlab1/capture"
N_FRAMES   = 6            # cuadros a guardar
MAX_SECS   = 25           # corte duro
HB_INTERVAL = 0.1         # ver el comentario en heartbeat()

CMD_START     = bytes.fromhex("EF 00 01 00")
CMD_HEARTBEAT = bytes.fromhex("EF 00 04 00")

os.makedirs(OUTDIR, exist_ok=True)
stop = threading.Event()
recibidos = 0             # paquetes recibidos; lo actualiza main()


def heartbeat(sock):
    """Mantiene despierto el transmisor del dron.

    CMD_START (encendido de camara) se reenvia cada segundo MIENTRAS no llegue
    nada, y se deja de mandar apenas empieza a entrar video. Mandarlo una sola
    vez no alcanza: el dron no siempre lo toma y la captura se queda en cero.
    Seguir mandandolo despues tampoco sirve, porque reinicia el transmisor.

    El keepalive va cada HB_INTERVAL. Ojo con este valor: el dron emite una
    rafaga de cuadros por cada keepalive que recibe, asi que la frecuencia del
    keepalive determina el framerate. Con 1.0 s (el valor original) el video
    llega a tirones a ~3.4 fps; con 0.1 s sube a ~7 fps. Ver el README.
    """
    ultimo_start = 0.0
    while not stop.is_set():
        ahora = time.time()
        if recibidos == 0 and ahora - ultimo_start >= 1.0:
            try:
                sock.sendto(CMD_START, (DRONE_IP, DRONE_PORT))
            except Exception as e:
                print("[-] encendido:", e)
            ultimo_start = ahora
            time.sleep(0.05)
        try:
            sock.sendto(CMD_HEARTBEAT, (DRONE_IP, DRONE_PORT))
        except Exception as e:
            print("[-] heartbeat:", e)
        stop.wait(HB_INTERVAL)


def main():
    global recibidos
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", LOCAL_PORT))
    sock.settimeout(1.0)

    threading.Thread(target=heartbeat, args=(sock,), daemon=True).start()
    print("[+] Enviando comandos y escuchando en :%d ..." % LOCAL_PORT)

    sizes    = collections.Counter()
    senders  = collections.Counter()
    npkt     = 0
    saved    = 0
    chunks   = {}
    order    = []
    first_pkt_dumped = False
    t0 = time.time()

    while saved < N_FRAMES and time.time() - t0 < MAX_SECS:
        try:
            data, addr = sock.recvfrom(65535)
        except socket.timeout:
            continue

        npkt += 1
        recibidos = npkt
        sizes[len(data)] += 1
        senders[addr[0]] += 1

        if not first_pkt_dumped and len(data) >= 56:
            first_pkt_dumped = True
            print("\n[*] Primer paquete de datos desde %s:%d  len=%d" % (addr[0], addr[1], len(data)))
            print("    outer[0:32] : %s" % data[0:32].hex())
            print("    meta [32:56]: %s" % data[32:56].hex())
            print("    payload[0:32]: %s" % data[56:88].hex())

        if len(data) < 56:
            continue

        idx = data[32]
        if idx == 0 and chunks:
            assembled = bytearray()
            for i in sorted(chunks):
                assembled.extend(chunks[i])
            path = os.path.join(OUTDIR, "frame_%02d.bin" % saved)
            with open(path, "wb") as f:
                f.write(assembled)
            # guardar tambien los headers del cuadro para inspeccion
            with open(os.path.join(OUTDIR, "frame_%02d.meta" % saved), "w") as f:
                f.write("chunks=%d  bytes=%d\n" % (len(chunks), len(assembled)))
                f.write("orden_idx=%s\n" % order[:40])
            print("[+] frame_%02d.bin  chunks=%-4d bytes=%d" % (saved, len(chunks), len(assembled)))
            saved += 1
            chunks = {}
            order = []

        chunks[idx] = data[56:]
        order.append(idx)

    stop.set()
    time.sleep(0.2)
    sock.close()

    print("\n=== RESUMEN ===")
    print("paquetes totales : %d en %.1fs" % (npkt, time.time() - t0))
    print("origenes         : %s" % dict(senders))
    print("tamanos          : %s" % dict(sizes.most_common(8)))
    print("cuadros guardados: %d -> %s" % (saved, OUTDIR))


if __name__ == "__main__":
    main()
