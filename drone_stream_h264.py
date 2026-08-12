import queue
import struct
import socket
import threading
import subprocess
import time
import os
from flask import Flask, Response, jsonify

# Configuracion de red
DRONE_IP = "192.168.169.1"
CMD_PORT = 8800
UDP_PORT = 8804
RCV_BUF_SIZE = 26214400
DRONE_SSID = os.environ.get("DRONE_SSID", "LYHFPV_M22 2B967C")

# Comandos de configuración del SDK
CMD_PREFIX = bytes([0x01, 0x67])
CMD_REGISTER = b'\x01\x67<i=2^bf_ssid=' + DRONE_SSID.encode('ascii') + b'>'
CMD_START = bytes.fromhex("EF 00 01 00")

# Comandos de optimización (H.264, 30 FPS, Bitrate, Resolución)
CMD_BITRATE = CMD_PREFIX + b"<i=2^bf_bitrate=2048>"
CMD_FPS = CMD_PREFIX + b"<i=2^bf_fps=30>"
CMD_RES = CMD_PREFIX + b"<i=2^bf_resolution=0>"

# Flask App y Stats
app = Flask(__name__)
stats_lock = threading.Lock()
stats = {
    "fps": 0.0,
    "frames_ok": 0,
    "resolucion": "1280x720 (H.264 Hardware GStreamer)",
}

latest_frame = None
frame_lock = threading.Lock()

def gstreamer_pipeline():
    """Pipeline de GStreamer acelerado por hardware (Subproceso puro)."""
    return [
        'gst-launch-1.0', '-q',
        'fdsrc', 'fd=0', '!',
        'h264parse', '!',
        'nvv4l2decoder', '!',
        'nvvidconv', '!',
        'video/x-raw,width=1280,height=720,format=I420', '!',
        'nvjpegenc', '!',
        'fdsink', 'fd=1'
    ]

def control_loop(sock_udp):
    """Inyector de comandos UDP (Puerto 8800)."""
    
    # Autorizar cliente para no ser bloqueados!
    print(f"[CONTROL] Registrando SSID '{DRONE_SSID}' del cliente...")
    try:
        sock_udp.sendto(CMD_REGISTER, (DRONE_IP, CMD_PORT))
    except Exception:
        pass
    time.sleep(0.5)

    # Iniciar transmisión
    print(f"[CONTROL] Enviando orden de encendido de video...")
    try:
        sock_udp.sendto(CMD_START, (DRONE_IP, CMD_PORT))
    except Exception as e:
        print(f"[CONTROL] Error encendiendo video: {e}")

    ultimo_overclock = 0.0
    while True:
        try:
            ahora = time.time()
            # Enviar Keep-Alive (CRITICO para que no se apague)
            sock_udp.sendto(bytes.fromhex("EF 00 04 00"), (DRONE_IP, CMD_PORT))
            
            # Re-despertar si no recibimos cuadros (esto arranca el video inicialmente y lo mantiene)
            sock_udp.sendto(CMD_START, (DRONE_IP, CMD_PORT))
            
            # Enviar comandos de optimización periódicamente
            if ahora - ultimo_overclock > 1.5:
                sock_udp.sendto(CMD_BITRATE, (DRONE_IP, CMD_PORT))
                sock_udp.sendto(CMD_FPS, (DRONE_IP, CMD_PORT))
                sock_udp.sendto(CMD_RES, (DRONE_IP, CMD_PORT))
                ultimo_overclock = ahora
                
        except Exception as e:
            print(f"[CONTROL] Error inyectando comandos: {e}")
            
        time.sleep(0.05) # Frecuencia rápida para que el dron no pare de mandar frames

def capture_loop(process):
    """Captura de video leyendo los JPEGs generados por GStreamer."""
    global latest_frame
    
    print("[GSTREAMER] ¡Proceso abierto con éxito! Esperando frames (JPEGs)...")
    
    frames_ok = 0
    t_start = time.time()
    buffer = b""
    
    while True:
        if process.stdout is None:
            time.sleep(0.1)
            continue
            
        chunk = process.stdout.read(8192)
        if not chunk:
            print("[GSTREAMER] El proceso se cerró repentinamente.")
            break
            
        buffer += chunk
        
        while True:
            # Buscar inicio JPEG
            a = buffer.find(b'\xff\xd8')
            if a == -1:
                buffer = buffer[-2:] if len(buffer) > 2 else buffer
                break
                
            # Buscar fin JPEG
            b = buffer.find(b'\xff\xd9', a)
            if b == -1:
                break
                
            jpg = buffer[a:b+2]
            buffer = buffer[b+2:]
            
            with frame_lock:
                latest_frame = jpg
                
            frames_ok += 1
            
        # Calcular FPS cada 30 cuadros
        if frames_ok % 30 == 0:
            dt = time.time() - t_start
            if dt > 0:
                fps = 30.0 / dt
                print(f"[GSTREAMER] Rendimiento de decodificación: {fps:.1f} FPS (720p hardware dec/enc)")
                with stats_lock:
                    stats["fps"] = fps
                    stats["frames_ok"] += 30
            t_start = time.time()

def receiver_loop(sock, process):
    """Recibe paquetes UDP, quita la cabecera de 13 bytes y escribe el H.264 puro al stdin de GStreamer."""
    print(f"[RECEPTOR] Escuchando UDP {UDP_PORT} (Modo Raw H.264)...")
    first_packet = True
    
    while True:
        try:
            data, addr = sock.recvfrom(65536)
            
            if first_packet and len(data) > 16:
                first_packet = False
                print(f"\n[DEBUG H264] Primer paquete Len: {len(data)}")
                hex_str = ' '.join(f'{b:02x}' for b in data[:128])
                print(f"[DEBUG H264] Primeros 128 bytes:\n{hex_str}\n")
                
            # Evitar enviar basura a gstreamer mientras analizamos
            if False and process and process.stdin:
                try:
                    process.stdin.write(data)
                    process.stdin.flush()
                except Exception:
                    pass
        except Exception as e:
            print(f"Error receiver: {e}")

def generate_web_stream():
    """Generador para el streaming HTTP (MJPEG)."""
    while True:
        with frame_lock:
            frame = latest_frame
            
        if frame is None:
            time.sleep(0.1)
            continue
            
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        # Limitar a ~30 fps la vista web para no sobrecargar CPU
        time.sleep(1.0 / 30.0) 

@app.route("/")
def index():
    return """<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>Jetson M22 - H.264 (GStreamer)</title>
<style>
 body{background:#000;color:#fff;font-family:sans-serif;text-align:center;margin:0;padding:20px}
 h1{color:#76b900}
 .container{max-width:1280px;margin:0 auto;background:#111;padding:15px;border-radius:10px;box-shadow:0 0 15px rgba(118,185,0,.3)}
 img{width:100%;height:auto;border-radius:5px;border:2px solid #76b900}
 #s{margin-top:10px;font-family:monospace;color:#9c9;font-size:1.2em}
</style></head>
<body><div class="container">
 <h1>NVIDIA Jetson - Dron M22 (Hardware H.264)</h1>
 <img src="/video_feed" alt="Video Feed"/>
 <div id="s">Iniciando pipeline GStreamer...</div>
</div>
<script>
setInterval(function(){
  fetch('/stats').then(function(r){return r.json()}).then(function(d){
    document.getElementById('s').textContent =
      '[' + d.resolucion + '] ' + d.fps.toFixed(1) + ' FPS | Cuadros H.264 Decodificados: ' + d.frames_ok;
  });
},1000);
</script></body></html>"""

@app.route("/stats")
def stats_json():
    with stats_lock:
        return jsonify(stats)

@app.route('/video_feed')
def video_feed():
    return Response(generate_web_stream(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    print("===========================================")
    print("  SERVIDOR DRON M22 - MODO H.264/GSTREAMER ")
    print("===========================================")
    
    # Socket UDP unificado anclado al puerto 8804 (Requerido por el dron para recibir comandos y enviar video)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, RCV_BUF_SIZE)
    except:
        pass
        
    sock.bind(("0.0.0.0", UDP_PORT))
    
    # Iniciar GStreamer como subproceso para pasarle los NAL Units limpios por stdin
    cmd = gstreamer_pipeline()
    print(f"[GSTREAMER] Lanzando subproceso puro:\n{' '.join(cmd)}")
    try:
        process = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        # Hilo para imprimir los errores de GStreamer al vuelo (crucial para depurar)
        def log_stderr():
            while True:
                line = process.stderr.readline()
                if not line: break
                print(f"[GST-LOG] {line.decode().strip()}")
        threading.Thread(target=log_stderr, daemon=True).start()
    except Exception as e:
        print(f"[ERROR] No se pudo lanzar gst-launch-1.0: {e}")
        exit(1)
    
    # Iniciar hilos de red
    threading.Thread(target=control_loop, args=(sock,), daemon=True).start()
    threading.Thread(target=capture_loop, args=(process,), daemon=True).start()
    threading.Thread(target=receiver_loop, args=(sock, process), daemon=True).start()

    print("\n[WEB] Visualizador H.264 disponible en http://0.0.0.0:5000\n")
    app.run(host='0.0.0.0', port=5000, threaded=True)
