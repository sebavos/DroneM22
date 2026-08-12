"""
Interfaz de Recepcion y Streaming de Video para Dron M22 (LYHFPV)
=================================================================
Servidor de Video en Vivo desarrollado para NVIDIA Jetson Nano.

Formato del cuadro (ingenieria inversa + analisis del SDK BL-UAVSDK):
    El dron NO envia un JPEG completo: envia unicamente el 'scan data'
    (datos entropy-coded) sin cabecera. La cabecera (SOI+DQT+DHT+SOF0+SOS)
    hay que aportarla: submuestreo 4:4:4, tablas Huffman estandar (Annex K)
    y tablas de cuantizacion extraidas de libuav_lib.so (calidad ~75 IJG).

    Cuadro reproducible = JPEG_HEADER + scan + FF D9

Protocolo (descubierto via descompilacion del APK):
    - CMD_REGISTER: paquete Custom Msg (ID 0x67) que registra el SSID del
      cliente en el MCU del dron. Sin esto, el dron limita ancho de banda.
    - CMD_START (EF 00 01 00): enciende el transmisor.
    - CMD_HEARTBEAT (EF 00 04 00): keepalive. El dron emite una rafaga de
      cuadros por cada heartbeat que recibe.

Arquitectura de estabilidad:
    - receiver_loop: hilo dedicado, sin bloqueos mas alla de recvfrom.
    - frame_condition (Condition): notificacion instantanea al streamer
      sin sleep ni polling.
    - stats_lock separado del frame_condition.
"""

import io
import json
import os
import queue
import socket
import struct
import threading
import time

import cv2
import numpy as np
from flask import Flask, Response, jsonify, request

from drone_control import DroneController
from drone_ai import DroneAI

# ---------------------------------------------------------------------------
# Configuracion de red
# ---------------------------------------------------------------------------
DRONE_IP   = "192.168.169.1"
DRONE_PORT = 8800
LOCAL_PORT = 8804
DRONE_SSID = os.environ.get("DRONE_SSID", "LYHFPV_M22 2B967C")

# ---------------------------------------------------------------------------
# Comandos UDP (descubiertos via descompilacion del SDK BL-UAVSDK)
# ---------------------------------------------------------------------------
CMD_START     = bytes.fromhex("EF 00 01 00")  # nativeStart()
CMD_STOP      = bytes.fromhex("EF 00 02 00")  # nativeStop()
CMD_PHOTO     = bytes.fromhex("EF 00 03 00")  # Snapshot
CMD_HEARTBEAT = bytes.fromhex("EF 00 04 00")  # nativeGetVersion() / keepalive
CMD_RECORD    = bytes.fromhex("EF 00 05 00")  # Grabar a SD

# Custom Msg (ID 0x67): registra el SSID del cliente en el MCU del dron.
# Sin este comando, el dron puede limitar el ancho de banda o ignorar paquetes.
# Formato: 01 67 <i=2^bf_ssid=<SSID>>
CMD_REGISTER  = b'\x01\x67<i=2^bf_ssid=' + DRONE_SSID.encode('ascii') + b'>'

# Comandos de "Overclock" (descubiertos via ingeniería inversa del APK).
# Fuerzan el firmware a subir bitrate, fps y fijar resolución 720p nativa.
CMD_BITRATE   = b'\x01\x67<i=2^bf_bitrate=2048>'
CMD_FPS_30    = b'\x01\x67<i=2^bf_fps=30>'
CMD_RES_720   = b'\x01\x67<i=2^bf_resolution=0>'

# ---------------------------------------------------------------------------
# Offsets dentro del paquete UDP de 1080 bytes
# ---------------------------------------------------------------------------
OFF_PAYLOAD     = 56   # inicio del payload de video
POS_CHUNK_IDX   = 32   # indice de fragmento (0 = inicio de cuadro)
POS_CHUNK_TOTAL = 36   # cantidad total de fragmentos del cuadro
POS_FRAME_SIZE  = 40   # uint32 LE: tamano del scan completo
POS_WIDTH       = 44   # uint16 LE
POS_HEIGHT      = 46   # uint16 LE

# ---------------------------------------------------------------------------
# Parametros ajustables via variables de entorno
# ---------------------------------------------------------------------------

# Calidad JPEG. Los DQT del firmware (libuav_lib.so offset 0x003E4560)
# corresponden a quality=75 en la escala IJG. q=40 era una estimacion anterior.
JPEG_Q = int(os.environ.get("DRONE_JPEG_Q", "75"))

# Intervalo del keepalive en segundos. La app oficial usa 1000 ms, pero
# pruebas empiricas en este hardware muestran que 100 ms da mejor fps
# (7 fps vs 3.4 fps). Configurable para experimentar.
HB_INTERVAL = float(os.environ.get("DRONE_HB_INTERVAL", "0.1"))

# Buffer de recepcion UDP. La app configura SO_RCVBUF a 512 KB (0x80000).
RCV_BUF_SIZE = int(os.environ.get("DRONE_RCVBUF", str(512 * 1024)))


# ---------------------------------------------------------------------------
# Cabecera JPEG generada con la calidad correcta via PIL
# ---------------------------------------------------------------------------
def _build_jpeg_header(width=1280, height=720, quality=JPEG_Q):
    """Genera la cabecera JPEG que el dron no manda.

    Usa PIL para codificar una imagen vacia con los parametros correctos
    (submuestreo 4:4:4, Huffman estandar Annex K, calidad del firmware)
    y corta justo al final del segmento SOS. Todo lo anterior al SOS es
    la cabecera exacta.
    """
    from PIL import Image
    buf = io.BytesIO()
    Image.new("RGB", (width, height)).save(
        buf, "JPEG",
        quality=quality,
        subsampling=0,      # 0 = 4:4:4
        optimize=False,     # Huffman estandar (Annex K)
        progressive=False,
    )
    t = buf.getvalue()
    p = t.find(b"\xff\xda")               # SOS: Start Of Scan
    length = (t[p + 2] << 8) | t[p + 3]   # largo del segmento SOS
    return t[: p + 2 + length]


JPEG_HEADER = _build_jpeg_header()
JPEG_EOI    = b"\xff\xd9"

print("[+] Cabecera JPEG construida: %d bytes (q=%d, 4:4:4, Annex K)"
      % (len(JPEG_HEADER), JPEG_Q))


# ---------------------------------------------------------------------------
# Aplicacion Flask
# ---------------------------------------------------------------------------
app = Flask(__name__)

# Sincronizacion: Condition variable para notificacion instantanea
# entre jitter_buffer (productor) y generate_mjpeg (consumidor).
frame_condition = threading.Condition()
latest_jpeg     = None     # bytes del ultimo JPEG valido

# Buffer Jitter: Absorbe las rafagas de cuadros del dron y los entrega
# a una cadencia uniforme. maxsize=15 (aprox 2 segundos a 7 fps).
jitter_queue = queue.Queue(maxsize=15)
PLAYBACK_FPS = float(os.environ.get("DRONE_PLAYBACK_FPS", "10.0"))

# Stats separadas con su propio lock para no interferir con el frame path
stats_lock = threading.Lock()
stats = {
    "packets": 0,
    "frames_ok": 0,
    "frames_incompletos": 0,
    "fps": 0.0,
    "ultimo_cuadro": 0.0,
    "resolucion": "-",
    "calidad_jpeg": JPEG_Q,
}

# ---------------------------------------------------------------------------
# DroneController y DroneAI (instancias globales)
# ---------------------------------------------------------------------------
controller: DroneController = None   # Se inicializa en main()
drone_ai: DroneAI = None             # Se inicializa bajo demanda


# ---------------------------------------------------------------------------
# HUD (solo cuando no hay video)
# ---------------------------------------------------------------------------
def encode_hud(text_status):
    """HUD de telemetria; solo se usa cuando no hay video valido."""
    img = np.zeros((720, 1280, 3), dtype=np.uint8)
    img[:] = (20, 20, 20)
    f = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(img, "NVIDIA JETSON - INTERFAZ DRON M22 (LYHFPV)", (50, 70),  f, 1.1, (0, 255, 0), 2)
    cv2.putText(img, "Estado: %s" % text_status,                    (50, 140), f, 0.9, (255, 255, 255), 2)
    cv2.putText(img, "IP Dron: %s:%d" % (DRONE_IP, DRONE_PORT),    (50, 220), f, 0.8, (200, 200, 200), 2)
    with stats_lock:
        pkt, fok, finc, fps_val = (stats["packets"], stats["frames_ok"],
                                    stats["frames_incompletos"], stats["fps"])
    cv2.putText(img, "Paquetes UDP: %d" % pkt,                              (50, 270), f, 0.8, (0, 255, 255), 2)
    cv2.putText(img, "Cuadros OK: %d  (incompletos: %d)" % (fok, finc),     (50, 320), f, 0.8, (0, 255, 255), 2)
    cv2.putText(img, "FPS: %.1f" % fps_val,                                  (50, 370), f, 0.8, (0, 255, 255), 2)
    cv2.putText(img, "Hora Jetson: %s" % time.strftime("%H:%M:%S"),          (50, 430), f, 0.8, (0, 200, 255), 2)
    cv2.rectangle(img, (30, 30), (1250, 690), (0, 255, 0), 2)
    ok, enc = cv2.imencode(".jpg", img)
    return enc.tobytes() if ok else b""


# ---------------------------------------------------------------------------
# Publicacion de frames y Jitter Buffer
# ---------------------------------------------------------------------------
def _enqueue_frame(jpg):
    """Pone un cuadro en el Jitter Buffer si hay espacio."""
    try:
        jitter_queue.put_nowait(jpg)
    except queue.Full:
        pass


def jitter_buffer_loop():
    """Consume frames de la cola a una tasa adaptativa para suavizar ráfagas.
    Si la cola está vacía, mantiene el último frame, eliminando el parpadeo.
    Utiliza un algoritmo de control de velocidad (PLL) basado en el nivel del buffer.
    """
    global latest_jpeg
    
    while True:
        start_time = time.time()
        
        with stats_lock:
            real_fps = stats["fps"]
            
        # Algoritmo de Jitter Buffer Adaptativo:
        # Si reproducimos muy rapido (ej. 10fps con entrada de 8fps), el buffer
        # se vacia y causa tirones ("micro stutters").
        if real_fps < 2.0:
            current_fps = 10.0 # Default fallback
        else:
            q_size = jitter_queue.qsize()
            if q_size > 8:
                current_fps = real_fps * 1.15 # Buffer lleno -> acelerar reproduccion
            elif q_size < 4:
                current_fps = real_fps * 0.85 # Buffer bajo -> frenar un poco para acumular
            else:
                current_fps = real_fps        # Zona dorada -> reproducir a tasa real
                
        # Limites de cordura
        current_fps = max(1.0, min(current_fps, 30.0))
        frame_interval = 1.0 / current_fps
        
        try:
            jpg = jitter_queue.get(timeout=0.05)
            with frame_condition:
                latest_jpeg = jpg
                frame_condition.notify_all()
        except queue.Empty:
            pass
            
        elapsed = time.time() - start_time
        sleep_time = frame_interval - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)


def _publish_hud(text):
    """Publica un frame HUD (sin video real)."""
    global latest_jpeg
    hud = encode_hud(text)
    with frame_condition:
        latest_jpeg = hud
        frame_condition.notify_all()


# ---------------------------------------------------------------------------
# NOTA: El hilo de heartbeat ahora lo maneja DroneController._heartbeat_loop()
# Se mantiene esta función como fallback por si se ejecuta sin el controller.
# ---------------------------------------------------------------------------


# ---------------------------------------------------------------------------
# Hilo de recepcion UDP
# ---------------------------------------------------------------------------
def receiver_loop(sock):
    """Recibe fragmentos UDP, ensambla el scan y publica el JPEG.

    Prioridad maxima: este hilo no hace nada que no sea leer del socket
    y ensamblar fragmentos. Publica via _publish_frame() que solo toma el
    Condition un instante. Ningun sleep en el camino caliente.
    """
    sock.settimeout(0.5)

    chunks     = {}
    exp_chunks = 0
    last_idx   = -1
    current_frame_corrupted = False
    fps_t0     = time.time()
    fps_n      = 0

    _publish_hud("Iniciando receptor UDP...")
    print("[+] Receptor UDP :%d  rcvbuf=%d KB" % (LOCAL_PORT, RCV_BUF_SIZE // 1024))

    while True:
        # --- Recibir paquete ---
        try:
            data, _ = sock.recvfrom(2048)
        except socket.timeout:
            with stats_lock:
                sin_video = (time.time() - stats["ultimo_cuadro"]) > 2.0
            if sin_video:
                _publish_hud("Esperando respuesta del dron...")
            continue
        except Exception:
            continue

        # --- Validar tamaño mínimo ---
        if len(data) < OFF_PAYLOAD:
            # Si es muy chico, podría ser telemetría (13 bytes, empieza en 0x66)
            if len(data) == 13 and data[0] == 0x66:
                if controller is not None:
                    controller._parse_telemetry(data)
            continue

        with stats_lock:
            stats["packets"] += 1

        idx = data[POS_CHUNK_IDX]

        # --- Inicio de cuadro nuevo ---
        if idx == 0:
            if chunks and exp_chunks > 0:
                if not current_frame_corrupted and len(chunks) == exp_chunks:
                    # Cuadro completo y sin perdidas: ensamblar y publicar
                    try:
                        scan = b"".join(chunks[i] for i in range(exp_chunks))
                        jpg  = JPEG_HEADER + scan + JPEG_EOI
                        fps_n += 1
                        now = time.time()
                        if now - fps_t0 >= 1.0:
                            with stats_lock:
                                stats["fps"] = fps_n / (now - fps_t0)
                            fps_t0, fps_n = now, 0
                        
                        with stats_lock:
                            stats["frames_ok"] += 1
                            stats["ultimo_cuadro"] = now
                            
                        _enqueue_frame(jpg)

                        # Alimentar frame a la IA si está activa
                        if drone_ai is not None and drone_ai._running:
                            drone_ai.feed_frame(jpg)
                    except KeyError:
                        with stats_lock:
                            stats["frames_incompletos"] += 1
                else:
                    # Incompleto o corrupto: descartar silenciosamente (EVITA FLASHES MORADOS)
                    with stats_lock:
                        stats["frames_incompletos"] += 1

            # Iniciar nuevo cuadro
            chunks     = {}
            exp_chunks = data[POS_CHUNK_TOTAL]
            current_frame_corrupted = False
            w = struct.unpack_from("<H", data, POS_WIDTH)[0]
            h = struct.unpack_from("<H", data, POS_HEIGHT)[0]
            with stats_lock:
                stats["resolucion"] = "%dx%d" % (w, h)
        else:
            # Validacion estricta de secuencia UDP para evitar Frankensteins
            if idx == last_idx:
                continue  # Ignorar duplicados de red inofensivos
            if idx != last_idx + 1:
                current_frame_corrupted = True

        last_idx = idx
        # --- Guardar fragmento ---
        chunks[idx] = data[OFF_PAYLOAD:]


# ---------------------------------------------------------------------------
# Generador MJPEG
# ---------------------------------------------------------------------------
def generate_mjpeg():
    """Generador MJPEG con notificacion instantanea via Condition.

    No hace polling ni sleep activo. Espera a que receiver_loop publique
    un frame nuevo. Timeout de 1s evita conexiones zombi.
    """
    sent = None
    while True:
        with frame_condition:
            frame_condition.wait(timeout=1.0)
            frame = latest_jpeg

        if frame is not None and frame is not sent:
            sent = frame
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                   + frame + b"\r\n")


# ---------------------------------------------------------------------------
# Rutas Flask — Video y Stats (originales)
# ---------------------------------------------------------------------------
@app.route("/")
def index():
    return """<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>NVIDIA Jetson - Interfaz Dron M22</title>
<style>
 body{background:#121212;color:#fff;font-family:Arial,sans-serif;text-align:center;margin:0;padding:20px}
 h1{color:#76b900}
 .container{max-width:1280px;margin:0 auto;background:#1e1e1e;padding:15px;border-radius:10px;box-shadow:0 4px 10px rgba(0,0,0,.5)}
 img{width:100%;height:auto;border-radius:5px;border:2px solid #76b900}
 #s{margin-top:10px;font-family:monospace;color:#9c9}
</style></head>
<body><div class="container">
 <h1>NVIDIA Jetson - Interfaz Dron M22 (LYHFPV)</h1>
 <p>Transmisi&oacute;n de Video en Vivo (JPEG reconstruido)</p>
 <img src="/video_feed" alt="Transmisi&oacute;n del Dron"/>
 <div id="s">cargando...</div>
</div>
<script>
setInterval(function(){
  fetch('/stats').then(function(r){return r.json()}).then(function(d){
    document.getElementById('s').textContent =
      d.resolucion+' | '+d.fps.toFixed(1)+' fps | cuadros OK '+d.frames_ok+
      ' | incompletos '+d.frames_incompletos+' | paquetes '+d.packets+' | q='+d.calidad_jpeg;
  });
},1000);
</script></body></html>"""


@app.route("/stats")
def stats_json():
    with stats_lock:
        snapshot = dict(stats)
    return jsonify(snapshot)


@app.route("/video_feed")
def video_feed():
    return Response(generate_mjpeg(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")


# ---------------------------------------------------------------------------
# Rutas Flask — API de Control del Dron
# ---------------------------------------------------------------------------
@app.route("/api/arm", methods=["POST"])
def api_arm():
    """Arma el dron: habilita envío de comandos RC a 50Hz."""
    if controller is None:
        return jsonify({"error": "Controller no inicializado"}), 503
    controller.arm()
    return jsonify({"status": "armed"})


@app.route("/api/disarm", methods=["POST"])
def api_disarm():
    """Desarma el dron: detiene comandos RC (failsafe)."""
    if controller is None:
        return jsonify({"error": "Controller no inicializado"}), 503
    controller.disarm()
    return jsonify({"status": "disarmed"})


@app.route("/api/takeoff", methods=["POST"])
def api_takeoff():
    """Despega el dron."""
    if controller is None:
        return jsonify({"error": "Controller no inicializado"}), 503
    controller.takeoff()
    return jsonify({"status": "takeoff"})


@app.route("/api/land", methods=["POST"])
def api_land():
    """Aterriza el dron."""
    if controller is None:
        return jsonify({"error": "Controller no inicializado"}), 503
    controller.land()
    return jsonify({"status": "landing"})


@app.route("/api/emergency", methods=["POST"])
def api_emergency():
    """Emergency stop — para motores inmediatamente."""
    if controller is None:
        return jsonify({"error": "Controller no inicializado"}), 503
    controller.emergency_stop()
    return jsonify({"status": "emergency_stop"})


@app.route("/api/calibrate", methods=["POST"])
def api_calibrate():
    """Calibra el giroscopio (dron en suelo plano)."""
    if controller is None:
        return jsonify({"error": "Controller no inicializado"}), 503
    controller.calibrate_gyro()
    return jsonify({"status": "calibrating"})


@app.route("/api/rc", methods=["POST"])
def api_rc():
    """Envía comando RC manual. Body JSON: {roll, pitch, throttle, yaw}"""
    if controller is None:
        return jsonify({"error": "Controller no inicializado"}), 503
    data = request.get_json(silent=True) or {}
    controller.set_rc(
        roll=float(data.get("roll", 0)),
        pitch=float(data.get("pitch", 0)),
        throttle=float(data.get("throttle", 0)),
        yaw=float(data.get("yaw", 0)),
    )
    return jsonify({"status": "ok", "rc": controller.get_rc_state()})


@app.route("/api/hover", methods=["POST"])
def api_hover():
    """Mantiene posición (neutrales)."""
    if controller is None:
        return jsonify({"error": "Controller no inicializado"}), 503
    controller.hover()
    return jsonify({"status": "hovering"})


@app.route("/api/photo", methods=["POST"])
def api_photo():
    """Captura foto a SD."""
    if controller is None:
        return jsonify({"error": "Controller no inicializado"}), 503
    controller.take_photo()
    return jsonify({"status": "photo_taken"})


@app.route("/api/record", methods=["POST"])
def api_record():
    """Toggle grabación a SD."""
    if controller is None:
        return jsonify({"error": "Controller no inicializado"}), 503
    controller.toggle_recording()
    return jsonify({"status": "recording_toggled"})


@app.route("/api/config", methods=["POST"])
def api_config():
    """Configura cámara. Body JSON: {bitrate, fps, resolution}"""
    if controller is None:
        return jsonify({"error": "Controller no inicializado"}), 503
    data = request.get_json(silent=True) or {}
    if "bitrate" in data:
        controller.set_bitrate(int(data["bitrate"]))
    if "fps" in data:
        controller.set_fps(int(data["fps"]))
    if "resolution" in data:
        controller.set_resolution(int(data["resolution"]))
    return jsonify({"status": "config_updated"})


@app.route("/api/obstacle_avoidance", methods=["POST"])
def api_obstacle_avoidance():
    """Activa/desactiva sensores de proximidad. Body JSON: {enable: true/false}"""
    if controller is None:
        return jsonify({"error": "Controller no inicializado"}), 503
    data = request.get_json(silent=True) or {}
    controller.enable_obstacle_avoidance(data.get("enable", True))
    return jsonify({"status": "ok"})


@app.route("/api/raw", methods=["POST"])
def api_raw():
    """Envía comando crudo. Body JSON: {hex: "EF000100", channel: "CMD"}"""
    if controller is None:
        return jsonify({"error": "Controller no inicializado"}), 503
    data = request.get_json(silent=True) or {}
    hex_str = data.get("hex", "")
    channel = data.get("channel", "CMD").upper()
    try:
        raw = bytes.fromhex(hex_str)
    except ValueError:
        return jsonify({"error": "hex inválido"}), 400
    if channel == "RC":
        controller.send_raw_rc(raw)
    else:
        controller.send_raw_cmd(raw)
    return jsonify({"status": "sent", "hex": hex_str, "channel": channel})


@app.route("/api/telemetry")
def api_telemetry():
    """Devuelve datos de telemetría del dron."""
    if controller is None:
        return jsonify({"error": "Controller no inicializado"}), 503
    return jsonify(controller.get_telemetry())


@app.route("/api/control/state")
def api_control_state():
    """Estado completo del controller."""
    if controller is None:
        return jsonify({"error": "Controller no inicializado"}), 503
    return jsonify({
        "connected": controller.is_connected,
        "armed": controller.is_armed,
        "rc": controller.get_rc_state(),
        "telemetry": controller.get_telemetry(),
    })


@app.route("/api/control/history")
def api_control_history():
    """Historial de comandos enviados."""
    if controller is None:
        return jsonify({"error": "Controller no inicializado"}), 503
    n = int(request.args.get("n", 50))
    return jsonify(controller.get_command_history(n))


# ---------------------------------------------------------------------------
# Rutas Flask — API de IA
# ---------------------------------------------------------------------------
@app.route("/api/ai/start", methods=["POST"])
def api_ai_start():
    """Inicia el motor de IA. Body JSON: {target_class, model}"""
    global drone_ai
    if controller is None:
        return jsonify({"error": "Controller no inicializado"}), 503
    data = request.get_json(silent=True) or {}
    target = data.get("target_class", "person")
    model = data.get("model", "yolov8n.pt")

    if drone_ai is None:
        drone_ai = DroneAI(controller, target_class=target, model_name=model)
    else:
        drone_ai.set_target_class(target)

    if not controller.is_armed:
        controller.arm()
    drone_ai.start()
    return jsonify({"status": "ai_started", "target": target, "model": model})


@app.route("/api/ai/stop", methods=["POST"])
def api_ai_stop():
    """Detiene el motor de IA."""
    if drone_ai is None:
        return jsonify({"error": "IA no inicializada"}), 404
    drone_ai.stop()
    return jsonify({"status": "ai_stopped"})


@app.route("/api/ai/stats")
def api_ai_stats():
    """Estadísticas del motor de IA."""
    if drone_ai is None:
        return jsonify({"error": "IA no inicializada"}), 404
    return jsonify(drone_ai.get_stats())


@app.route("/api/ai/config", methods=["POST"])
def api_ai_config():
    """Actualiza configuración de IA. Body JSON: {target_class, pid_*}"""
    if drone_ai is None:
        return jsonify({"error": "IA no inicializada"}), 404
    data = request.get_json(silent=True) or {}
    if "target_class" in data:
        drone_ai.set_target_class(data["target_class"])
    # Actualizar PID gains si se proporcionan
    for axis in ("yaw", "throttle", "pitch", "roll"):
        kp = data.get(f"pid_{axis}_kp")
        ki = data.get(f"pid_{axis}_ki")
        kd = data.get(f"pid_{axis}_kd")
        if any(v is not None for v in (kp, ki, kd)):
            drone_ai.update_pid_gains(
                axis,
                kp=float(kp) if kp else None,
                ki=float(ki) if ki else None,
                kd=float(kd) if kd else None,
            )
    return jsonify({"status": "config_updated", "config": drone_ai.config.to_dict()})


# ---------------------------------------------------------------------------
# Inicio
# ---------------------------------------------------------------------------
def main():
    global controller

    # Socket dedicado para recepción de video y envío de comandos/heartbeat (puerto 8804)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, RCV_BUF_SIZE)
    sock.bind(("0.0.0.0", LOCAL_PORT))

    # --- Inicializar DroneController ---
    # Pasamos 'sock' para que envíe CMD_REGISTER y heartbeats desde 8804,
    # lo cual obliga al dron a responder con video a este mismo puerto.
    controller = DroneController()
    controller.connect(cmd_sock=sock)

    # Iniciar hilos de video (el heartbeat ya lo maneja el controller)
    threading.Thread(target=jitter_buffer_loop, daemon=True).start()
    threading.Thread(target=receiver_loop, args=(sock,), daemon=True).start()

    print("\n" + "=" * 65)
    print(" Servidor de Video en Vivo + Control + IA")
    print(" JPEG q=%d | UDP buf=%d KB"
          % (JPEG_Q, RCV_BUF_SIZE // 1024))
    print(" SSID registrado: %s" % DRONE_SSID)
    print(" DroneController: RC→%d CMD→%d" % (controller.rc_port, controller.cmd_port))
    print("")
    print(" Endpoints:")
    print("   Video:   http://192.168.14.8:5000/video_feed")
    print("   Stats:   http://192.168.14.8:5000/stats")
    print("   Control: POST /api/{arm,disarm,takeoff,land,emergency,rc,...}")
    print("   IA:      POST /api/ai/{start,stop}  GET /api/ai/stats")
    print("=" * 65 + "\n")

    try:
        app.run(host="0.0.0.0", port=5000, threaded=True)
    finally:
        controller.disconnect()


if __name__ == "__main__":
    main()
