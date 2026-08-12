import os
import ctypes
import socket
import threading
import time
import subprocess
from flask import Flask, Response, jsonify

# ---------------------------------------------------------------------------
# Configuracion
# ---------------------------------------------------------------------------
DRONE_IP   = "192.168.169.1"
DRONE_PORT = 8800
LOCAL_PORT = 8804
DRONE_SSID = os.environ.get("DRONE_SSID", "LYHFPV_M22 2B967C")
HB_INTERVAL = 0.05  # 50ms heartbeat para forzar 15 FPS

CMD_START     = bytes.fromhex("EF 00 01 00")
CMD_HEARTBEAT = bytes.fromhex("EF 00 04 00")
CMD_REGISTER  = b'\x01\x67<i=2^bf_ssid=' + DRONE_SSID.encode('ascii') + b'>'

# Comandos "Overclock" descubiertos
CMD_BITRATE   = b'\x01\x67<i=2^bf_bitrate=2048>'
CMD_FPS_30    = b'\x01\x67<i=2^bf_fps=30>'
CMD_RES_720   = b'\x01\x67<i=2^bf_resolution=0>'

app = Flask(__name__)
frame_condition = threading.Condition()
latest_jpeg = None

# Stats
stats_lock = threading.Lock()
stats = {
    "frames_ok": 0,
    "fps": 0.0,
    "ultimo_cuadro": 0.0,
}

# ---------------------------------------------------------------------------
# C Receiver (Zero-Copy overhead reduction)
# ---------------------------------------------------------------------------
def load_c_receiver():
    so_path = "./fast_receiver.so"
    if not os.path.exists(so_path) or os.path.getmtime("fast_receiver.c") > os.path.getmtime(so_path):
        print("[*] Compilando modulo C optimizado (fast_receiver.so)...")
        subprocess.run(["gcc", "-O3", "-shared", "-fPIC", "fast_receiver.c", "-o", so_path], check=True)
    
    lib = ctypes.CDLL(so_path)
    lib.init_receiver.argtypes = [ctypes.c_int, ctypes.c_int]
    lib.init_receiver.restype = ctypes.c_int
    lib.receive_frame.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.receive_frame.restype = ctypes.c_int
    return lib

def fast_receiver_loop(lib, sock_fd):
    global latest_jpeg
    
    # 2MB Buffer pre-alocado
    MAX_SIZE = 2 * 1024 * 1024
    buffer = ctypes.create_string_buffer(MAX_SIZE)
    
    res = lib.init_receiver(sock_fd, 2 * 1024 * 1024) # 2MB rcvbuf
    if res < 0:
        print(f"[-] Error iniciando socket C: {res}")
        return
        
    print(f"[+] Hilo C Receiver corriendo en puerto {LOCAL_PORT}...")
    
    frames = 0
    t0 = time.time()
    
    while True:
        size = lib.receive_frame(buffer, MAX_SIZE)
        if size > 0:
            # Zero-copy conversion a bytes (ctypes memoryview es rapido)
            # Solo copiamos el payload final
            jpeg_data = ctypes.string_at(buffer, size)
            
            with frame_condition:
                latest_jpeg = jpeg_data
                frame_condition.notify_all()
                
            frames += 1
            ahora = time.time()
            dt = ahora - t0
            
            with stats_lock:
                stats["ultimo_cuadro"] = ahora
                stats["frames_ok"] = frames
                if dt >= 2.0:
                    stats["fps"] = frames / dt
                    frames = 0
                    t0 = ahora

# ---------------------------------------------------------------------------
# TX Heartbeat Loop
# ---------------------------------------------------------------------------
def heartbeat_loop(sock):
    # Registro inicial
    try:
        sock.sendto(CMD_REGISTER, (DRONE_IP, DRONE_PORT))
        time.sleep(0.1)
        sock.sendto(CMD_START, (DRONE_IP, DRONE_PORT))
    except Exception:
        pass
        
    ultimo_overclock = 0.0
    while True:
        try:
            ahora = time.time()
            sock.sendto(CMD_HEARTBEAT, (DRONE_IP, DRONE_PORT))
            
            # Re-despertar
            with stats_lock:
                ultimo = stats["ultimo_cuadro"]
            if ahora - ultimo > 2.0:
                sock.sendto(CMD_START, (DRONE_IP, DRONE_PORT))
                
        except Exception:
            pass
            
        time.sleep(HB_INTERVAL)

# ---------------------------------------------------------------------------
# Flask / MJPEG
# ---------------------------------------------------------------------------
def generate_mjpeg():
    sent = None
    while True:
        with frame_condition:
            frame_condition.wait(timeout=1.0)
            frame = latest_jpeg

        if frame is not None and frame is not sent:
            sent = frame
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                   + frame + b"\r\n")

@app.route("/")
def index():
    return """<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>NVIDIA Jetson - Interfaz Dron M22 (FAST)</title>
<style>
 body{background:#121212;color:#fff;font-family:Arial,sans-serif;text-align:center;margin:0;padding:20px}
 h1{color:#76b900}
 .container{max-width:1280px;margin:0 auto;background:#1e1e1e;padding:15px;border-radius:10px;box-shadow:0 4px 10px rgba(0,0,0,.5)}
 img{width:100%;height:auto;border-radius:5px;border:2px solid #76b900}
 #s{margin-top:10px;font-family:monospace;color:#9c9;font-size:1.2em;}
</style></head>
<body><div class="container">
 <h1>NVIDIA Jetson - M22 (FAST MJPEG)</h1>
 <img src="/video_feed" alt="Transmisi&oacute;n del Dron"/>
 <div id="s">cargando...</div>
</div>
<script>
setInterval(function(){
  fetch('/stats').then(r=>r.json()).then(d=>{
    document.getElementById('s').textContent =
      d.fps.toFixed(1)+' FPS | OK: '+d.frames_ok;
  });
},500);
</script></body></html>"""

@app.route("/stats")
def stats_json():
    with stats_lock:
        return jsonify(stats)

@app.route("/video_feed")
def video_feed():
    return Response(generate_mjpeg(), mimetype="multipart/x-mixed-replace; boundary=frame")

if __name__ == "__main__":
    print("[*] Iniciando Servidor Jetson Optimizado (Fast C-Receiver + Direct Stream)...")
    try:
        lib = load_c_receiver()
    except Exception as e:
        print(f"[-] Error cargando receptor C: {e}")
        exit(1)
        
    # Crear socket unico y compartirlo
    shared_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    shared_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    shared_sock.bind(("0.0.0.0", LOCAL_PORT))
    
    threading.Thread(target=fast_receiver_loop, args=(lib, shared_sock.fileno()), daemon=True).start()
    threading.Thread(target=heartbeat_loop, args=(shared_sock,), daemon=True).start()
    
    print("\n[+] Servidor Flask escuchando en http://0.0.0.0:5000")
    app.run(host="0.0.0.0", port=5000, threaded=True)
