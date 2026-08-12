import socket
import threading
import time
import queue
import collections
import struct
import numpy as np
import subprocess
import os
from flask import Flask, Response, jsonify

# Configuracion de red
DRONE_IP = "192.168.169.1"  # IP oficial del dron
CMD_PORT = 8800
UDP_PORT = 8804
TCP_CMD_PORT = 11000        # Puerto oficial para comandos Protobuf (lxProtoPro)

CMD_START = bytes.fromhex("EF 00 01 00")
CMD_HEARTBEAT = bytes.fromhex("EF 00 04 00")
# MAGIA DESCUBIERTA: Comando Protobuf (TCP) para obligar a la camara a transmitir en H.265
CMD_START_HEVC = bytes.fromhex("08 00 00 00 08 03 2A 04 08 02 10 02")

RCV_BUF_SIZE = 26214400 # Ampliado a ~26MB para evitar cuello de botella en Linux
HB_INTERVAL = 1.0

# Flask App y Stats
app = Flask(__name__)
jitter_queue = queue.Queue(maxsize=15) # Jitter buffer para los NAL Units H.265
stats_lock = threading.Lock()
stats = {
    "fps": 0.0,
    "frames_ok": 0,
    "packets": 0,
    "resolucion": "H.265 HEVC (Hardware Decoded)",
    "calidad_jpeg": 5
}

ffmpeg_cmd = [
    'ffmpeg',
    '-hide_banner',
    '-loglevel', 'error',
    '-f', 'hevc',          # Formato de entrada: H.265 (HEVC) raw
    '-i', 'pipe:0',        # Leemos H.265 del stdin
    '-f', 'image2pipe',    # Salida por pipe en formato imagenes separadas
    '-vcodec', 'mjpeg',    # Codificar la salida a JPEG (Rapidísimo)
    '-q:v', '5',           # Calidad visual del JPEG resultante
    'pipe:1'               # Enviar JPEGs al stdout
]

ffmpeg_process = None

def init_ffmpeg():
    global ffmpeg_process
    try:
        ffmpeg_process = subprocess.Popen(
            ffmpeg_cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        print("[FFMPEG] Subproceso de decodificación por hardware iniciado correctamente.")
    except Exception as e:
        print(f"[FFMPEG] Error crítico iniciando FFmpeg: {e}. ¿Está ffmpeg instalado en la Jetson?")

def get_jpeg_frames():
    """Generador que lee los JPEGs resultantes desde la salida de FFmpeg"""
    buffer = b""
    while True:
        if ffmpeg_process is None or ffmpeg_process.stdout is None:
            time.sleep(0.1)
            continue
            
        # Leemos pedazos del output de FFmpeg
        chunk = ffmpeg_process.stdout.read(8192)
        if not chunk:
            print("[FFMPEG] El proceso se cerró repentinamente.")
            break
            
        buffer += chunk
        
        while True:
            # Buscar el inicio de un JPEG (FF D8)
            a = buffer.find(b'\xff\xd8')
            if a == -1:
                # Si no hay inicio, descartamos la basura pero guardamos 2 bytes
                buffer = buffer[-2:] if len(buffer) > 2 else buffer
                break
                
            # Buscar el final del JPEG (FF D9)
            b = buffer.find(b'\xff\xd9', a)
            if b == -1:
                break
                
            # Extraer la imagen completa JPEG creada por la Jetson
            jpg = buffer[a:b+2]
            buffer = buffer[b+2:]
            
            # Enviarla al navegador
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpg + b'\r\n')

@app.route("/")
def index():
    return """<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>NVIDIA Jetson - Interfaz Dron M22 (H.265)</title>
<style>
 body{background:#121212;color:#fff;font-family:Arial,sans-serif;text-align:center;margin:0;padding:20px}
 h1{color:#76b900}
 .container{max-width:1280px;margin:0 auto;background:#1e1e1e;padding:15px;border-radius:10px;box-shadow:0 4px 10px rgba(0,0,0,.5)}
 img{width:100%;height:auto;border-radius:5px;border:2px solid #76b900}
 #s{margin-top:10px;font-family:monospace;color:#9c9}
</style></head>
<body><div class="container">
 <h1>NVIDIA Jetson - Dron M22 (Hardware H.265 Decoding)</h1>
 <img src="/video_feed" alt="Transmisi&oacute;n del Dron"/>
 <div id="s">cargando...</div>
</div>
<script>
setInterval(function(){
  fetch('/stats').then(function(r){return r.json()}).then(function(d){
    document.getElementById('s').textContent =
      d.resolucion+' | '+d.fps.toFixed(1)+' fps | cuadros OK '+d.frames_ok+
      ' | paquetes '+d.packets+' | q='+d.calidad_jpeg;
  });
},1000);
</script></body></html>"""

@app.route("/stats")
def stats_json():
    with stats_lock:
        snapshot = dict(stats)
    return jsonify(snapshot)

@app.route('/video_feed')
def video_feed():
    return Response(get_jpeg_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

def ffmpeg_writer_loop():
    """Toma cuadros H265 del Jitter Buffer y los inyecta a máxima velocidad a FFmpeg."""
    while True:
        nal_unit = jitter_queue.get()
        if ffmpeg_process and ffmpeg_process.stdin:
            try:
                ffmpeg_process.stdin.write(nal_unit)
                ffmpeg_process.stdin.flush()
            except Exception as e:
                pass

def control_loop():
    """Mantiene la conexión y pide H.265 usando nuestro comando secreto."""
    sock_udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # 1. Iniciar stream (Por defecto inicia en MJPEG)
    print("[CONTROL] Iniciando stream UDP base...")
    sock_udp.sendto(CMD_START, (DRONE_IP, CMD_PORT))
    
    # 2. Enviar el comando secreto Protobuf para cambiarlo a H.265
    time.sleep(1.0)
    print(f"[CONTROL] Abriendo puerto {TCP_CMD_PORT} TCP para inyectar comando secreto H.265...")
    try:
        sock_tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock_tcp.settimeout(3.0)
        sock_tcp.connect((DRONE_IP, TCP_CMD_PORT))
        sock_tcp.sendall(CMD_START_HEVC)
        print("[CONTROL] >>> ¡ÉXITO! Comando H.265 (HEVC) inyectado correctamente. <<<")
        sock_tcp.close()
    except Exception as e:
        print(f"[CONTROL] Fallo TCP ({TCP_CMD_PORT}): {e}. Si esto falla, el dron seguirá enviando MJPEG y no funcionará.")

    # 3. Heartbeats
    while True:
        try:
            sock_udp.sendto(CMD_HEARTBEAT, (DRONE_IP, CMD_PORT))
        except:
            pass
        time.sleep(HB_INTERVAL)

def receiver_loop():
    """Recibe los NAL Units H.265 del dron de forma limpia y rápida"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", UDP_PORT))
    
    # Intentamos maximizar el buffer de nuevo
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, RCV_BUF_SIZE)
    except:
        pass

    chunks = {}
    last_idx = -1
    
    print(f"[RECEPTOR] Escuchando UDP {UDP_PORT}...")
    
    frames_ok = 0
    total_pkts = 0
    t_start = time.time()
    
    while True:
        try:
            data, addr = sock.recvfrom(65536)
            total_pkts += 1
            
            if len(data) <= 56:
                continue

            header = data[:56]
            payload = data[56:]

            if header[0:4] != b'\x55\xaa\x55\xaa':
                continue

            idx, exp_chunks = struct.unpack("<HH", header[32:36])
            
            if idx == 0:
                chunks.clear()
            
            # Rechazar paquetes duplicados
            if idx in chunks:
                continue
                
            # Rechazar secuencias rotas
            if idx > 0 and idx != last_idx + 1:
                chunks.clear()
                last_idx = -1
                continue
                
            chunks[idx] = payload
            last_idx = idx

            # Ensamblado completo del frame
            if len(chunks) == exp_chunks:
                frame_data = b"".join(chunks[i] for i in range(exp_chunks))
                
                # Para H.265, enviamos los datos crudos a FFmpeg
                try:
                    jitter_queue.put_nowait(frame_data)
                    frames_ok += 1
                except queue.Full:
                    pass
                    
                chunks.clear()
                last_idx = -1

            # Mostrar FPS
            if total_pkts % 200 == 0:
                dt = time.time() - t_start
                if dt >= 2.0:
                    fps = frames_ok / dt
                    print(f"[H.265 STREAM] Rendimiento: {fps:.1f} FPS | Buffer: {jitter_queue.qsize()}/15")
                    with stats_lock:
                        stats["fps"] = fps
                        stats["frames_ok"] += frames_ok
                        stats["packets"] = total_pkts
                        
                    frames_ok = 0
                    t_start = time.time()
                    
        except Exception as e:
            print(f"Error receiver: {e}")

if __name__ == '__main__':
    print("=========================================")
    print("   SERVIDOR DRON M22 - MODO H.265/HEVC   ")
    print("=========================================")
    
    # Iniciar decodificador por hardware
    init_ffmpeg()
    
    # Iniciar hilos
    threading.Thread(target=control_loop, daemon=True).start()
    threading.Thread(target=receiver_loop, daemon=True).start()
    threading.Thread(target=ffmpeg_writer_loop, daemon=True).start()

    print("\n[WEB] Visualizador disponible en http://0.0.0.0:5000\n")
    # Usar threaded=True para evitar bloqueos
    app.run(host='0.0.0.0', port=5000, threaded=True)
