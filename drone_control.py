"""
Módulo de Control del Dron M22 (LYHFPV)
========================================
API completa para control de vuelo, video y configuración del dron M22
vía comandos hexadecimales UDP.

Dos canales de comunicación:
    - RC (puerto 8080): Paquetes joystick a 50Hz (header 0x66)
    - CMD (puerto 8800): Configuración y video (header 0x01 0x67 / 0xEF)

Protocolo RC (8-10 bytes):
    66 [Roll] [Pitch] [Throttle] [Yaw] [Flags] [TrimR] [TrimP] [Checksum] 99
    - Neutral: Roll=0x80, Pitch=0x80, Throttle=0x00, Yaw=0x80
    - Checksum: XOR de bytes 1..5
    - Envío cada 20ms (50Hz) obligatorio — failsafe si se detiene

Flags (byte 5):
    0x01 = Takeoff (mantener 500ms)
    0x02 = Land
    0x04 = Emergency Stop
    0x08 = Calibrate Gyro

Uso:
    from drone_control import DroneController

    ctrl = DroneController()
    ctrl.connect()
    ctrl.arm()
    ctrl.takeoff()
    ctrl.set_rc(roll=0, pitch=20, throttle=50, yaw=0)  # valores -100..+100
    ctrl.land()
    ctrl.disarm()
    ctrl.disconnect()
"""

import collections
import logging
import os
import socket
import struct
import threading
import time

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
logger = logging.getLogger("drone_control")
logger.setLevel(logging.DEBUG)
if not logger.handlers:
    _h = logging.StreamHandler()
    _h.setFormatter(logging.Formatter(
        "[%(asctime)s] %(levelname)s %(message)s", datefmt="%H:%M:%S"))
    logger.addHandler(_h)


# ---------------------------------------------------------------------------
# Constantes de red
# ---------------------------------------------------------------------------
DRONE_IP      = "192.168.169.1"
RC_PORT       = 12000       # Control de vuelo (joystick / armado)
CMD_PORT      = 8800        # Comandos de configuración y video
TELEMETRY_PORT = 8804       # Video + telemetría de sensores
DRONE_SSID    = os.environ.get("DRONE_SSID", "LYHFPV_M22 2B967C")

# ---------------------------------------------------------------------------
# Constantes del protocolo RC
# ---------------------------------------------------------------------------
RC_HEADER     = 0x66
RC_FOOTER     = 0x99
RC_NEUTRAL    = 0x80        # Valor neutral para Roll, Pitch, Yaw
RC_THROTTLE_MIN = 0x00
RC_THROTTLE_MAX = 0xFF
RC_HZ         = 50          # Frecuencia de envío obligatoria
RC_INTERVAL   = 1.0 / RC_HZ  # 20ms

# ---------------------------------------------------------------------------
# Constantes del protocolo CMD (puerto 8800)
# ---------------------------------------------------------------------------
CMD_START     = bytes.fromhex("EF000100")  # Enciende transmisor de video
CMD_STOP      = bytes.fromhex("EF000200")  # Detiene transmisor de video
CMD_PHOTO     = bytes.fromhex("EF000300")  # Capturar foto
CMD_HEARTBEAT = bytes.fromhex("EF000400")  # Keepalive
CMD_RECORD    = bytes.fromhex("EF000500")  # Toggle grabación a SD

# ---------------------------------------------------------------------------
# Flags de control de vuelo (byte 5 del paquete RC)
# ---------------------------------------------------------------------------
FLAG_NONE       = 0x00
FLAG_ARM        = 0x40  # Desbloquea motores (luces fijas)
FLAG_TAKEOFF    = 0x02  # Despegue automático
FLAG_LAND       = 0x04  # Aterrizaje automático
FLAG_EMERGENCY  = 0x08  # Parada de emergencia
FLAG_FLIP       = 0x10  # Acrobacia

# ---------------------------------------------------------------------------
# Parámetros de seguridad
# ---------------------------------------------------------------------------
WATCHDOG_TIMEOUT = 2.0      # Segundos sin comando RC → hover automático
COMMAND_LOG_SIZE = 1000     # Tamaño del historial circular
TAKEOFF_FLAG_DURATION = 0.5  # Segundos que se mantiene el flag de takeoff
HB_INTERVAL = float(os.environ.get("DRONE_HB_INTERVAL", "0.1"))


# ---------------------------------------------------------------------------
# Helper: Custom Msg builder
# ---------------------------------------------------------------------------
def _build_custom_msg(key: str, value: str) -> bytes:
    """Construye un paquete Custom Msg (0x01 0x67) para el MCU del dron.

    Formato: 01 67 <i=2^bf_{key}={value}>
    """
    payload = f"<i=2^bf_{key}={value}>"
    return b'\x01\x67' + payload.encode('ascii')


# ===========================================================================
# Entrada de historial
# ===========================================================================
class CommandEntry:
    """Registro de un comando enviado al dron."""
    __slots__ = ("timestamp", "channel", "raw_hex", "description")

    def __init__(self, channel: str, raw: bytes, description: str):
        self.timestamp = time.time()
        self.channel = channel        # "RC" o "CMD"
        self.raw_hex = raw.hex()
        self.description = description

    def to_dict(self):
        return {
            "t": self.timestamp,
            "channel": self.channel,
            "hex": self.raw_hex,
            "desc": self.description,
        }


# ===========================================================================
# Datos de telemetría
# ===========================================================================
class TelemetryData:
    """Datos de telemetría recibidos del dron."""
    def __init__(self):
        self.dist_front = -1    # cm, -1 = no disponible
        self.dist_back = -1
        self.dist_left = -1
        self.dist_right = -1
        self.battery_voltage = 0.0
        self.optical_flow_x = 0
        self.optical_flow_y = 0
        self.timestamp = 0.0

    def to_dict(self):
        return {
            "dist_f": self.dist_front,
            "dist_b": self.dist_back,
            "dist_l": self.dist_left,
            "dist_r": self.dist_right,
            "vbat": self.battery_voltage,
            "oflow_x": self.optical_flow_x,
            "oflow_y": self.optical_flow_y,
            "ts": self.timestamp,
        }


# ===========================================================================
# DroneController
# ===========================================================================
class DroneController:
    """API de control del dron M22 vía comandos hexadecimales UDP.

    Maneja dos canales UDP simultáneos:
      - RC (puerto 8080): paquetes joystick a 50Hz
      - CMD (puerto 8800): configuración, video, heartbeat

    Thread-safe: todos los envíos protegidos con locks.
    """

    def __init__(self, drone_ip: str = DRONE_IP, rc_port: int = RC_PORT,
                 cmd_port: int = CMD_PORT, ssid: str = DRONE_SSID):
        self.drone_ip = drone_ip
        self.rc_port = rc_port
        self.cmd_port = cmd_port
        self.ssid = ssid

        # --- Sockets ---
        self._rc_sock = None
        self._cmd_sock = None

        # --- Estado de vuelo (protegido por _rc_lock) ---
        self._rc_lock = threading.Lock()
        self._throttle = RC_NEUTRAL   # Byte 1: 0x00..0xFF (neutral 0x80)
        self._pitch = RC_NEUTRAL      # Byte 2: 0x00..0xFF
        self._roll = RC_NEUTRAL       # Byte 3: 0x00..0xFF
        self._yaw = RC_NEUTRAL        # Byte 4: 0x00..0xFF
        self._flags = FLAG_NONE       # Byte 5: Flags

        # --- Estado global ---
        self._armed = False
        self._connected = False
        self._running = False

        # --- Threads ---
        self._rc_thread = None
        self._hb_thread = None
        self._telemetry_thread = None

        # --- Watchdog ---
        self._last_rc_command_time = 0.0

        # --- Telemetría ---
        self._telemetry = TelemetryData()
        self._telemetry_lock = threading.Lock()
        self._telemetry_callbacks: list = []

        # --- Historial de comandos ---
        self._cmd_lock = threading.Lock()
        self._history = collections.deque(maxlen=COMMAND_LOG_SIZE)

        # --- Callbacks de eventos ---
        self._event_callbacks = {
            "connected": [],
            "disconnected": [],
            "armed": [],
            "disarmed": [],
            "takeoff": [],
            "land": [],
            "emergency": [],
            "telemetry": [],
        }

    # -----------------------------------------------------------------------
    # Propiedades de estado
    # -----------------------------------------------------------------------
    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_armed(self) -> bool:
        return self._armed

    # -----------------------------------------------------------------------
    # Checksum
    # -----------------------------------------------------------------------
    @staticmethod
    def calculate_checksum(data: bytes) -> int:
        """Calcula el checksum XOR de los bytes 1..5 del paquete RC.

        Protocolo: checksum = (byte1 ^ byte2 ^ byte3 ^ byte4 ^ byte5) & 0xFF
        """
        return (data[1] ^ data[2] ^ data[3] ^ data[4] ^ data[5]) & 0xFF

    # -----------------------------------------------------------------------
    # Construcción de paquetes RC
    # -----------------------------------------------------------------------
    def _build_rc_packet(self) -> bytes:
        """Construye el paquete RC de 8 bytes con los valores actuales.

        Formato: 66 [Roll] [Pitch] [Throttle] [Yaw] [Flags] [Checksum] 99
        """
        with self._rc_lock:
            data = bytes([
                RC_HEADER,          # 0: Header (0x66)
                self._roll,         # 1: Roll
                self._pitch,        # 2: Pitch
                self._throttle,     # 3: Throttle
                self._yaw,          # 4: Yaw
                self._flags,        # 5: Flags
                0x00,               # 6: Checksum (placeholder)
                RC_FOOTER,          # 7: Footer (0x99)
            ])
        # Calcular checksum
        chk = self.calculate_checksum(data)
        data = data[:6] + bytes([chk]) + data[7:]
        return data

    # -----------------------------------------------------------------------
    # Envío de datos
    # -----------------------------------------------------------------------
    def _send_rc(self, packet: bytes, description: str = "RC"):
        """Envía un paquete RC al puerto de control de vuelo."""
        if self._rc_sock is None:
            return
        try:
            self._rc_sock.sendto(packet, (self.drone_ip, self.rc_port))
        except Exception as e:
            logger.warning("Error enviando RC: %s", e)

    def _send_cmd(self, packet: bytes, description: str = "CMD"):
        """Envía un comando al puerto de configuración."""
        if self._cmd_sock is None:
            return
        try:
            self._cmd_sock.sendto(packet, (self.drone_ip, self.cmd_port))
            self._log_command("CMD", packet, description)
        except Exception as e:
            logger.warning("Error enviando CMD: %s", e)

    def _log_command(self, channel: str, raw: bytes, description: str):
        """Registra un comando en el historial circular."""
        entry = CommandEntry(channel, raw, description)
        with self._cmd_lock:
            self._history.append(entry)

    # -----------------------------------------------------------------------
    # Loops internos
    # -----------------------------------------------------------------------
    def _rc_loop(self):
        """Hilo de control RC a 50Hz. NUNCA para mientras esté corriendo.

        Si el dron está armado, envía los valores RC actuales.
        Si NO está armado, envía neutral (hover) igualmente para
        mantener el enlace y evitar failsafe cuando se arme.
        """
        logger.info("RC loop iniciado (50Hz → puerto %d)", self.rc_port)
        rc_log_counter = 0

        while self._running:
            t0 = time.time()

            if self._armed:
                # Watchdog: si nadie actualizó RC en WATCHDOG_TIMEOUT → hover
                if (t0 - self._last_rc_command_time) > WATCHDOG_TIMEOUT:
                    if self._last_rc_command_time > 0:
                        logger.warning("Watchdog RC: sin comando en %.1fs → hover",
                                       WATCHDOG_TIMEOUT)
                        self._reset_rc_to_neutral()
                        self._last_rc_command_time = t0  # Evitar spam del log

                packet = self._build_rc_packet()
                self._send_rc(packet, "RC control")

                # Loguear cada 50 paquetes (~1 segundo) para no saturar
                rc_log_counter += 1
                if rc_log_counter >= 50:
                    with self._rc_lock:
                        logger.debug(
                            "RC → R:%02X P:%02X T:%02X Y:%02X F:%02X",
                            self._roll, self._pitch, self._throttle,
                            self._yaw, self._flags)
                    rc_log_counter = 0

            # Dormir el tiempo restante para mantener 50Hz
            elapsed = time.time() - t0
            sleep_time = RC_INTERVAL - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _heartbeat_loop(self):
        """Hilo de heartbeat al puerto CMD (8800). Mantiene el video activo."""
        logger.info("Heartbeat loop iniciado (%.0fms → puerto %d)",
                     HB_INTERVAL * 1000, self.cmd_port)
        ultimo_start = 0.0
        ultimo_overclock = 0.0

        while self._running:
            try:
                ahora = time.time()
                self._send_cmd(CMD_HEARTBEAT, "heartbeat")

                # Re-despertar cámara si no hay video reciente
                if ahora - ultimo_start >= 2.0:
                    self._send_cmd(CMD_START, "auto-restart video")
                    ultimo_start = ahora

                # Inyectar overclock periódicamente
                if ahora - ultimo_overclock >= 1.5:
                    self._send_cmd(
                        _build_custom_msg("bitrate", "2048"), "overclock bitrate")
                    self._send_cmd(
                        _build_custom_msg("fps", "30"), "overclock fps")
                    self._send_cmd(
                        _build_custom_msg("resolution", "0"), "overclock res")
                    ultimo_overclock = ahora

            except Exception as e:
                logger.warning("Heartbeat error: %s", e)

            time.sleep(HB_INTERVAL)

    def _telemetry_loop(self, shared_sock = None):
        """Hilo que escucha telemetría de sensores en el puerto 8804.

        Los paquetes de telemetría vienen con header 0x66 mezclados con
        el video. Este parser se ejecuta solo si no hay un socket compartido
        (cuando drone_stream.py maneja su propio receptor, la telemetría
        se parsea desde ahí).
        """
        if shared_sock is None:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                sock.bind(("0.0.0.0", TELEMETRY_PORT))
                sock.settimeout(0.5)
            except OSError:
                logger.info("Puerto %d ya en uso (probablemente drone_stream). "
                            "Telemetría se parseará desde ahí.", TELEMETRY_PORT)
                return
        else:
            sock = shared_sock

        logger.info("Telemetry parser iniciado (puerto %d)", TELEMETRY_PORT)

        while self._running:
            try:
                data, _ = sock.recvfrom(2048)
            except socket.timeout:
                continue
            except Exception:
                continue

            # Los paquetes de telemetría empiezan con 0x66
            if len(data) > 2 and data[0] == RC_HEADER:
                self._parse_telemetry_packet(data)

        if shared_sock is None:
            sock.close()

    def _parse_telemetry_packet(self, data: bytes):
        """Parsea un paquete de telemetría del dron (header 0x66).

        Los datos de sensores de proximidad y flujo óptico vienen
        en formato binario después del header.
        """
        try:
            with self._telemetry_lock:
                self._telemetry.timestamp = time.time()

                # Parsear si tiene suficientes bytes para datos de sensores
                if len(data) >= 10:
                    # Formato binario de telemetría (estimado — ajustar
                    # con datos reales del dron):
                    # [0x66] [tipo] [dist_f_h] [dist_f_l] [dist_b_h] [dist_b_l]
                    # [dist_l_h] [dist_l_l] [dist_r_h] [dist_r_l] ...
                    if data[1] == 0x01:  # Tipo: sensores de proximidad
                        self._telemetry.dist_front = (data[2] << 8) | data[3]
                        self._telemetry.dist_back = (data[4] << 8) | data[5]
                        self._telemetry.dist_left = (data[6] << 8) | data[7]
                        self._telemetry.dist_right = (data[8] << 8) | data[9]
                    elif data[1] == 0x02:  # Tipo: flujo óptico
                        self._telemetry.optical_flow_x = struct.unpack_from(
                            "<h", data, 2)[0]
                        self._telemetry.optical_flow_y = struct.unpack_from(
                            "<h", data, 4)[0]

                # Notificar callbacks
                telemetry_copy = self._telemetry.to_dict()

            for cb in self._telemetry_callbacks:
                try:
                    cb(telemetry_copy)
                except Exception as e:
                    logger.warning("Telemetry callback error: %s", e)

        except Exception as e:
            logger.debug("Error parseando telemetría: %s", e)

    # -----------------------------------------------------------------------
    # Helpers internos
    # -----------------------------------------------------------------------
    def _reset_rc_to_neutral(self):
        """Devuelve los ejes a su posición central (hover)."""
        with self._rc_lock:
            self._throttle = RC_NEUTRAL
            self._pitch = RC_NEUTRAL
            self._roll = RC_NEUTRAL
            self._yaw = RC_NEUTRAL
            self._flags = FLAG_NONE

    @staticmethod
    def _clamp(value: int, lo: int, hi: int) -> int:
        return max(lo, min(hi, value))

    @staticmethod
    def _scale_axis(val: float) -> int:
        """Convierte un valor de -100 a 100 al rango 0..255 (neutral 128)."""
        val = max(-100.0, min(100.0, val))
        scaled = int(((val + 100.0) / 200.0) * 255.0)
        # Garantizar que 0.0 sea exactamente 128
        if -0.1 < val < 0.1:
            return RC_NEUTRAL
        return scaled

    @staticmethod
    def _scale_throttle(value: float) -> int:
        """Convierte un valor 0..100 a 0x00..0xFF para throttle.

        0   → 0x00
        50  → 0x80
        100 → 0xFF
        """
        clamped = max(0.0, min(100.0, value))
        return int(clamped * 255 / 100)

    def _fire_event(self, event: str, **kwargs):
        """Dispara callbacks de un evento."""
        for cb in self._event_callbacks.get(event, []):
            try:
                cb(**kwargs)
            except Exception as e:
                logger.warning("Event callback error [%s]: %s", event, e)

    # -----------------------------------------------------------------------
    # API Pública — Conexión
    # -----------------------------------------------------------------------
    def connect(self, cmd_sock=None):
        """Establece la conexión con el dron.

        1. Crea sockets UDP para RC y CMD
        2. Registra el SSID del cliente en el MCU
        3. Enciende el transmisor de video
        4. Arranca los hilos de heartbeat y telemetría
        """
        if self._connected:
            logger.warning("Ya conectado al dron")
            return

        logger.info("Conectando al dron %s (RC:%d, CMD:%d)...",
                     self.drone_ip, self.rc_port, self.cmd_port)

        # Crear sockets
        self._rc_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        if cmd_sock is None:
            self._cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # Telemetría en un hilo aparte (solo si controlamos el socket)
            self._telemetry_thread = threading.Thread(
                target=self._telemetry_loop, daemon=True, name="drone-telemetry")
            self._telemetry_thread.start()
        else:
            self._cmd_sock = cmd_sock

        self._running = True

        # Registrar SSID
        register_cmd = _build_custom_msg("ssid", self.ssid)
        self._send_cmd(register_cmd, f"register SSID '{self.ssid}'")
        time.sleep(0.1)

        # Encender video
        self._send_cmd(CMD_START, "start video")
        time.sleep(0.05)

        # Arrancar hilos
        self._hb_thread = threading.Thread(
            target=self._heartbeat_loop, daemon=True, name="drone-heartbeat")
        self._hb_thread.start()

        self._rc_thread = threading.Thread(
            target=self._rc_loop, daemon=True, name="drone-rc")
        self._rc_thread.start()

        self._connected = True
        logger.info("✓ Conectado al dron")
        self._fire_event("connected")

    def disconnect(self):
        """Desconecta del dron limpiamente.

        1. Desarma si estaba armado
        2. Detiene el video
        3. Para todos los hilos
        4. Cierra sockets
        """
        if not self._connected:
            return

        logger.info("Desconectando del dron...")

        if self._armed:
            self.disarm()

        self._send_cmd(CMD_STOP, "stop video")
        self._running = False

        # Esperar que los hilos terminen
        for t in (self._rc_thread, self._hb_thread, self._telemetry_thread):
            if t and t.is_alive():
                t.join(timeout=1.0)

        # Cerrar sockets
        for s in (self._rc_sock, self._cmd_sock):
            if s:
                try:
                    s.close()
                except Exception:
                    pass

        self._rc_sock = None
        self._cmd_sock = None
        self._connected = False

        logger.info("✓ Desconectado del dron")
        self._fire_event("disconnected")

    # -----------------------------------------------------------------------
    # API Pública — Armado
    # -----------------------------------------------------------------------
    def arm(self):
        """Arma (desbloquea) los motores del dron.

        Envía FLAG_ARM (0x40) durante 500ms, exactamente como hace la app
        cuando se presiona el botón del avión. Las luces del dron dejan
        de parpadear y queda listo para recibir comandos.
        """
        if not self._connected:
            logger.error("No se puede armar: no conectado")
            return
        if self._armed:
            logger.warning("Ya armado")
            return

        logger.info("⚡ ARMANDO dron — Enviando FLAG_ARM (0x40)")
        self._reset_rc_to_neutral()
        self._last_rc_command_time = time.time()
        self._armed = True

        # Enviar el flag de armado durante 500ms (como la app)
        with self._rc_lock:
            self._flags = FLAG_ARM

        def _clear_arm():
            time.sleep(TAKEOFF_FLAG_DURATION)
            with self._rc_lock:
                if self._flags == FLAG_ARM:
                    self._flags = FLAG_NONE
            logger.debug("Arm flag limpiado — dron desbloqueado")

        threading.Thread(target=_clear_arm, daemon=True).start()
        self._log_command("RC", bytes([FLAG_ARM]), "ARM/UNLOCK")
        self._fire_event("armed")

    def disarm(self):
        """Desarma el dron: deja de enviar comandos RC.

        El dron entrará en failsafe y aterrizará solo al no recibir
        más paquetes RC.
        """
        if not self._armed:
            return

        logger.info("🔒 DESARMANDO dron — RC loop detenido")
        self._reset_rc_to_neutral()
        self._armed = False
        self._fire_event("disarmed")

    # -----------------------------------------------------------------------
    # API Pública — Control de vuelo
    # -----------------------------------------------------------------------
    def takeoff(self):
        """Despega el dron.

        Envía FLAG_TAKEOFF (0x01) durante 500ms, después limpia el flag.
        El dron sube a una altura de hover automática.
        """
        if not self._armed:
            logger.error("No se puede despegar: dron no armado")
            return

        logger.info("🚀 TAKEOFF")
        with self._rc_lock:
            self._flags = FLAG_TAKEOFF
        self._last_rc_command_time = time.time()
        self._log_command("RC", bytes([FLAG_TAKEOFF]), "takeoff")
        self._fire_event("takeoff")

        # Mantener flag durante 500ms, luego limpiar
        def _clear_takeoff():
            time.sleep(TAKEOFF_FLAG_DURATION)
            with self._rc_lock:
                if self._flags == FLAG_TAKEOFF:
                    self._flags = FLAG_NONE
            logger.debug("Takeoff flag limpiado")

        threading.Thread(target=_clear_takeoff, daemon=True).start()

    def land(self):
        """Aterriza el dron de forma controlada.

        Envía FLAG_LAND (0x04) hasta que se desarme manualmente
        o el dron confirme aterrizaje.
        """
        if not self._armed:
            logger.warning("Land: dron no armado, enviando de todos modos")

        logger.info("🛏 LAND")
        with self._rc_lock:
            self._flags = FLAG_LAND
            # Throttle se queda en neutral (altitude hold)
        self._last_rc_command_time = time.time()
        self._log_command("RC", bytes([FLAG_LAND]), "land")
        self._fire_event("land")

    def emergency_stop(self):
        """Para los motores inmediatamente.

        ⚠️  PELIGRO: El dron caerá al suelo desde cualquier altura.
        Usar solo en emergencias reales.
        """
        logger.critical("🛑 EMERGENCY STOP")
        with self._rc_lock:
            self._flags = FLAG_EMERGENCY
            self._roll = RC_NEUTRAL
            self._pitch = RC_NEUTRAL
            self._throttle = RC_NEUTRAL
            self._yaw = RC_NEUTRAL
        self._last_rc_command_time = time.time()
        self._log_command("RC", bytes([FLAG_EMERGENCY]), "EMERGENCY STOP")
        self._fire_event("emergency")

    def calibrate_gyro(self):
        """Calibra el giroscopio. El dron DEBE estar en suelo plano.

        Envía FLAG_CALIBRATE (0x08) durante 1 segundo.
        """
        logger.info("🔧 Calibrando giroscopio...")
        with self._rc_lock:
            self._flags = FLAG_CALIBRATE
        self._log_command("RC", bytes([FLAG_CALIBRATE]), "calibrate gyro")

        def _clear_calibrate():
            time.sleep(1.0)
            with self._rc_lock:
                if self._flags == FLAG_CALIBRATE:
                    self._flags = FLAG_NONE
            logger.info("Calibración completada")

        threading.Thread(target=_clear_calibrate, daemon=True).start()

    def set_rc(self, roll: float, pitch: float, throttle: float, yaw: float):
        """Establece los valores de vuelo. Todos esperan un valor de -100 a +100.
        Para el acelerador (throttle), 0 es hover (mantener altitud), -100 es bajar, +100 es subir.
        """
        with self._rc_lock:
            self._roll = self._scale_axis(roll)
            self._pitch = self._scale_axis(pitch)
            self._yaw = self._scale_axis(yaw)
            self._throttle = self._scale_axis(throttle)
            self._last_rc_command_time = time.time()
            # Limpiar flags de acciones especiales al recibir RC manual
            if self._flags in (FLAG_TAKEOFF, FLAG_ARM):
                pass  # No interferir con takeoff/arm en progreso
            elif self._flags != FLAG_LAND:
                self._flags = FLAG_NONE

        self._last_rc_command_time = time.time()

    def hover(self):
        """Mantiene posición: todos los ejes a neutral, throttle a 0.

        El dron debería mantener su altura actual si tiene
        estabilización barométrica o de flujo óptico.
        """
        self._reset_rc_to_neutral()
        self._last_rc_command_time = time.time()

    def set_trim(self, roll_trim: int = 0, pitch_trim: int = 0):
        """Ajusta el trim (compensación fina) de roll y pitch.

        Valores: 0x00..0x3F (0..63)
        """
        with self._rc_lock:
            self._trim_roll = self._clamp(roll_trim, 0x00, 0x3F)
            self._trim_pitch = self._clamp(pitch_trim, 0x00, 0x3F)
        logger.debug("Trim: roll=%02X pitch=%02X",
                      self._trim_roll, self._trim_pitch)

    # -----------------------------------------------------------------------
    # API Pública — Video y Cámara (puerto 8800)
    # -----------------------------------------------------------------------
    def start_video(self):
        """Enciende el transmisor de video del dron."""
        logger.info("📹 Start video")
        self._send_cmd(CMD_START, "start video")

    def stop_video(self):
        """Detiene el transmisor de video."""
        logger.info("📹 Stop video")
        self._send_cmd(CMD_STOP, "stop video")

    def take_photo(self):
        """Captura una foto y la guarda en la SD del dron."""
        logger.info("📸 Photo")
        self._send_cmd(CMD_PHOTO, "take photo")

    def toggle_recording(self):
        """Inicia/detiene la grabación de video a la SD."""
        logger.info("🔴 Toggle recording")
        self._send_cmd(CMD_RECORD, "toggle recording")

    # -----------------------------------------------------------------------
    # API Pública — Configuración (Custom Msg)
    # -----------------------------------------------------------------------
    def set_bitrate(self, kbps: int = 2048):
        """Configura el bitrate del video (kbps). Default: 2048."""
        cmd = _build_custom_msg("bitrate", str(kbps))
        self._send_cmd(cmd, f"set bitrate={kbps}")

    def set_fps(self, fps: int = 30):
        """Configura los FPS del video. Default: 30."""
        cmd = _build_custom_msg("fps", str(fps))
        self._send_cmd(cmd, f"set fps={fps}")

    def set_resolution(self, resolution: int = 0):
        """Configura la resolución. 0=720p (nativo)."""
        cmd = _build_custom_msg("resolution", str(resolution))
        self._send_cmd(cmd, f"set resolution={resolution}")

    def enable_obstacle_avoidance(self, enable: bool = True):
        """Activa/desactiva los sensores de proximidad (si el dron los tiene)."""
        val = "1" if enable else "0"
        cmd = _build_custom_msg("avoid", val)
        self._send_cmd(cmd, f"obstacle avoidance={'ON' if enable else 'OFF'}")

    def send_custom_msg(self, key: str, value: str):
        """Envía un Custom Msg arbitrario al MCU del dron.

        Formato: 01 67 <i=2^bf_{key}={value}>
        Útil para experimentar con parámetros no documentados.
        """
        cmd = _build_custom_msg(key, value)
        self._send_cmd(cmd, f"custom: {key}={value}")

    # -----------------------------------------------------------------------
    # API Pública — Comandos crudos (debug/experimentación)
    # -----------------------------------------------------------------------
    def send_raw_rc(self, raw_bytes: bytes):
        """Envía un paquete RC crudo (con header/footer/checksum propios)."""
        self._send_rc(raw_bytes, "raw RC")
        self._log_command("RC", raw_bytes, "raw RC")

    def send_raw_cmd(self, raw_bytes: bytes):
        """Envía un comando crudo al puerto CMD (8800)."""
        self._send_cmd(raw_bytes, "raw CMD")

    # -----------------------------------------------------------------------
    # API Pública — Telemetría
    # -----------------------------------------------------------------------
    def get_telemetry(self) -> dict:
        """Devuelve los últimos datos de telemetría como diccionario."""
        with self._telemetry_lock:
            return self._telemetry.to_dict()

    def on_telemetry(self, callback):
        """Registra un callback que se invoca con cada paquete de telemetría.

        El callback recibe un dict con las claves de TelemetryData.to_dict().
        """
        self._telemetry_callbacks.append(callback)

    def parse_telemetry_from_packet(self, data: bytes):
        """Parsea telemetría desde un paquete recibido externamente.

        Usar cuando drone_stream.py recibe paquetes 0x66 en su propio
        socket y quiere delegarle el parsing al controller.
        """
        if len(data) > 2 and data[0] == RC_HEADER:
            self._parse_telemetry_packet(data)

    # -----------------------------------------------------------------------
    # API Pública — Eventos
    # -----------------------------------------------------------------------
    def on(self, event: str, callback):
        """Registra un callback para un evento del controller.

        Eventos: connected, disconnected, armed, disarmed,
                 takeoff, land, emergency, telemetry
        """
        if event in self._event_callbacks:
            self._event_callbacks[event].append(callback)
        else:
            logger.warning("Evento desconocido: %s", event)

    # -----------------------------------------------------------------------
    # API Pública — Historial
    # -----------------------------------------------------------------------
    def get_command_history(self, n: int = 50):
        """Devuelve los últimos N comandos enviados."""
        with self._cmd_lock:
            items = list(self._history)
        return [e.to_dict() for e in items[-n:]]

    # -----------------------------------------------------------------------
    # API Pública — Estado actual del RC
    # -----------------------------------------------------------------------
    def get_rc_state(self) -> dict:
        """Devuelve los valores actuales del paquete RC."""
        with self._rc_lock:
            return {
                "roll": self._roll,
                "pitch": self._pitch,
                "throttle": self._throttle,
                "yaw": self._yaw,
                "flags": self._flags,
                "trim_roll": self._trim_roll,
                "trim_pitch": self._trim_pitch,
                "armed": self._armed,
            }

    # -----------------------------------------------------------------------
    # Context manager
    # -----------------------------------------------------------------------
    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *args):
        self.disconnect()

    # -----------------------------------------------------------------------
    # Representación
    # -----------------------------------------------------------------------
    def __repr__(self):
        state = "ARMED" if self._armed else "DISARMED"
        conn = "CONNECTED" if self._connected else "DISCONNECTED"
        return (f"<DroneController {conn} {state} "
                f"→ {self.drone_ip} RC:{self.rc_port} CMD:{self.cmd_port}>")


# ===========================================================================
# Script de prueba standalone
# ===========================================================================
if __name__ == "__main__":
    print("=" * 65)
    print(" DroneController M22 -- Test de modulo")
    print("=" * 65)

    # Test 1: Checksum
    test_data = bytes([0x66, 0x80, 0x80, 0x80, 0x80, 0x40, 0x00, 0x99])
    chk = DroneController.calculate_checksum(test_data)
    expected = (0x80 ^ 0x80 ^ 0x80 ^ 0x80 ^ 0x40) & 0xFF
    assert chk == expected, f"Checksum fallo: {chk:#x} != {expected:#x}"
    print(f"[OK] Checksum ARMADO: 0x{chk:02X} (esperado: 0x{expected:02X})")

    # Test 2: Construccion de paquete
    ctrl = DroneController.__new__(DroneController)
    ctrl._rc_lock = threading.Lock()
    ctrl._roll = 0x80
    ctrl._pitch = 0x80
    ctrl._throttle = 0x80
    ctrl._yaw = 0x80
    ctrl._flags = 0x40
    pkt = ctrl._build_rc_packet()
    assert pkt[0] == 0x66, f"Header incorrecto: {pkt[0]:#x}"
    assert pkt[-1] == 0x99, f"Footer incorrecto: {pkt[-1]:#x}"
    assert len(pkt) == 8, f"Largo incorrecto: {len(pkt)}"
    print(f"[OK] Paquete de ARMADO (simulado app): {pkt.hex().upper()}")

    # Test 3: Escalado de ejes
    assert DroneController._scale_axis(0) == 128, "Neutral debe ser 128"
    assert DroneController._scale_axis(-100) == 1, "Min debe ser ~0"
    assert DroneController._scale_axis(100) == 255, "Max debe ser 255"
    print("[OK] Escalado de ejes correcto")

    # Test 4: Custom Msg builder
    msg = _build_custom_msg("bitrate", "2048")
    assert msg == b'\x01\x67<i=2^bf_bitrate=2048>'
    print(f"[OK] Custom Msg: {msg}")

    # Test 5: Paquete con takeoff flag
    ctrl._flags = FLAG_TAKEOFF
    pkt_takeoff = ctrl._build_rc_packet()
    assert pkt_takeoff[5] == FLAG_TAKEOFF
    print(f"[OK] Paquete takeoff: {pkt_takeoff.hex().upper()}")

    print("\n" + "=" * 65)
    print(" Todos los tests pasaron OK")
    print(" Modulo listo: from drone_control import DroneController")
    print("=" * 65)
