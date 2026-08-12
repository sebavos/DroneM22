"""
Módulo de IA para Control Autónomo del Dron M22 (YOLO + PID)
=============================================================
Utiliza detección de objetos (YOLOv8) y controladores PID para
seguimiento autónomo de un objetivo.

Arquitectura:
    Cámara → YOLO (detectar objetivo) → Error de posición → PID → DroneController

El error se calcula como la diferencia entre el centro del bounding box
del objetivo y el centro del frame. El controlador PID convierte ese error
en valores de Roll, Pitch, Yaw y Throttle.

Modos de operación:
    IDLE     → Esperando activación
    SEARCH   → Buscando objetivo (rotación lenta)
    TRACK    → Siguiendo objetivo detectado
    HOVER    → Objetivo perdido temporalmente, manteniendo posición
    LAND     → Aterrizando

Uso:
    from drone_control import DroneController
    from drone_ai import DroneAI

    ctrl = DroneController()
    ctrl.connect()
    ctrl.arm()

    ai = DroneAI(ctrl, target_class="person")
    ai.start()   # Empieza a procesar frames
    # ...
    ai.stop()
    ctrl.land()
"""

import collections
import logging
import math
import threading
import time
from enum import Enum, auto

logger = logging.getLogger("drone_ai")
logger.setLevel(logging.DEBUG)
if not logger.handlers:
    _h = logging.StreamHandler()
    _h.setFormatter(logging.Formatter(
        "[%(asctime)s] %(levelname)s [AI] %(message)s", datefmt="%H:%M:%S"))
    logger.addHandler(_h)


# ===========================================================================
# Controlador PID
# ===========================================================================
class PIDController:
    """Controlador PID con anti-windup y filtro derivativo.

    Parámetros:
        kp: ganancia proporcional
        ki: ganancia integral
        kd: ganancia derivativa
        output_limits: tupla (min, max) para saturación de salida
        integral_limits: tupla (min, max) para anti-windup
        d_filter_alpha: coeficiente del filtro EMA para el término D (0..1)
    """

    def __init__(self, kp: float = 1.0, ki: float = 0.0, kd: float = 0.0,
                 output_limits: tuple = (-100.0, 100.0),
                 integral_limits: tuple = (-50.0, 50.0),
                 d_filter_alpha: float = 0.3):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        self.integral_limits = integral_limits
        self.d_filter_alpha = d_filter_alpha

        # Estado interno
        self._integral = 0.0
        self._last_error = 0.0
        self._last_time = 0.0
        self._d_filtered = 0.0
        self._initialized = False

    def reset(self):
        """Resetea el estado interno del PID."""
        self._integral = 0.0
        self._last_error = 0.0
        self._last_time = 0.0
        self._d_filtered = 0.0
        self._initialized = False

    def update(self, error: float, current_time: float = None) -> float:
        """Calcula la salida del PID dado un error.

        Args:
            error: error actual (setpoint - medición)
            current_time: timestamp actual (usa time.time() si None)

        Returns:
            Salida del PID (saturada a output_limits)
        """
        if current_time is None:
            current_time = time.time()

        if not self._initialized:
            self._last_error = error
            self._last_time = current_time
            self._initialized = True
            return 0.0

        dt = current_time - self._last_time
        if dt <= 0:
            return 0.0

        # Proporcional
        p_term = self.kp * error

        # Integral con anti-windup
        self._integral += error * dt
        self._integral = max(self.integral_limits[0],
                             min(self.integral_limits[1], self._integral))
        i_term = self.ki * self._integral

        # Derivativo con filtro EMA (reduce ruido)
        d_raw = (error - self._last_error) / dt
        self._d_filtered = (self.d_filter_alpha * d_raw +
                            (1 - self.d_filter_alpha) * self._d_filtered)
        d_term = self.kd * self._d_filtered

        # Salida total
        output = p_term + i_term + d_term
        output = max(self.output_limits[0],
                     min(self.output_limits[1], output))

        # Actualizar estado
        self._last_error = error
        self._last_time = current_time

        return output


# ===========================================================================
# Estados de la máquina de estados
# ===========================================================================
class AIState(Enum):
    IDLE = auto()
    SEARCH = auto()
    TRACK = auto()
    HOVER = auto()
    LAND = auto()


# ===========================================================================
# Configuración de tracking
# ===========================================================================
class TrackingConfig:
    """Parámetros ajustables del sistema de tracking."""

    def __init__(self):
        # --- PID Gains (Roll: movimiento lateral) ---
        self.pid_roll_kp = 0.35
        self.pid_roll_ki = 0.02
        self.pid_roll_kd = 0.10

        # --- PID Gains (Pitch: movimiento frontal/trasero) ---
        #     Se usa para mantener distancia al objetivo basado en
        #     el tamaño del bounding box.
        self.pid_pitch_kp = 0.30
        self.pid_pitch_ki = 0.01
        self.pid_pitch_kd = 0.08

        # --- PID Gains (Yaw: rotación para centrar horizontalmente) ---
        self.pid_yaw_kp = 0.40
        self.pid_yaw_ki = 0.02
        self.pid_yaw_kd = 0.12

        # --- PID Gains (Throttle: altura para centrar verticalmente) ---
        self.pid_throttle_kp = 0.25
        self.pid_throttle_ki = 0.01
        self.pid_throttle_kd = 0.05

        # --- Zona muerta (deadband) ---
        #     Si el error es menor que esto, se ignora (evita oscilaciones).
        self.deadband_x = 0.05     # 5% del ancho del frame
        self.deadband_y = 0.05     # 5% del alto del frame
        self.deadband_size = 0.03  # 3% del error de tamaño

        # --- Tamaño objetivo del bounding box ---
        #     Fracción del frame que el bbox debe ocupar.
        #     Si el bbox es más chico → acercarse (pitch+), más grande → alejarse.
        self.target_bbox_ratio = 0.25   # 25% del frame

        # --- Timeouts ---
        self.target_lost_timeout = 2.0   # Segundos sin detección → HOVER
        self.hover_timeout = 5.0         # Segundos en HOVER → SEARCH
        self.search_yaw_speed = 15.0     # Velocidad de rotación en búsqueda

        # --- YOLO ---
        self.confidence_threshold = 0.5  # Confianza mínima para aceptar detección
        self.target_class = "person"     # Clase a seguir por defecto

        # --- Seguridad ---
        self.max_speed = 40.0           # Velocidad máxima de salida PID (0-100)
        self.throttle_base = 50.0       # Throttle base en tracking (hover)
        self.throttle_range = 20.0      # ±rango de throttle desde la base

    def to_dict(self) -> dict:
        return {k: v for k, v in self.__dict__.items() if not k.startswith("_")}


# ===========================================================================
# Detección YOLO
# ===========================================================================
class Detection:
    """Resultado de una detección YOLO."""
    __slots__ = ("class_name", "confidence", "x1", "y1", "x2", "y2")

    def __init__(self, class_name: str, confidence: float,
                 x1: float, y1: float, x2: float, y2: float):
        self.class_name = class_name
        self.confidence = confidence
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2

    @property
    def center_x(self) -> float:
        return (self.x1 + self.x2) / 2

    @property
    def center_y(self) -> float:
        return (self.y1 + self.y2) / 2

    @property
    def width(self) -> float:
        return self.x2 - self.x1

    @property
    def height(self) -> float:
        return self.y2 - self.y1

    @property
    def area(self) -> float:
        return self.width * self.height

    def to_dict(self) -> dict:
        return {
            "class": self.class_name,
            "conf": self.confidence,
            "bbox": [self.x1, self.y1, self.x2, self.y2],
            "center": [self.center_x, self.center_y],
        }


# ===========================================================================
# Motor de IA
# ===========================================================================
class DroneAI:
    """Motor de IA para control autónomo del dron M22.

    Usa YOLOv8 para detección de objetos y controladores PID para
    convertir el error de posición en comandos de vuelo.

    El motor se alimenta con frames JPEG (como los que produce
    drone_stream.py) y envía comandos vía DroneController.
    """

    def __init__(self, controller, target_class: str = "person",
                 model_name: str = "yolov8n.pt",
                 config: TrackingConfig = None):
        """
        Args:
            controller: instancia de DroneController
            target_class: clase YOLO a seguir ("person", "car", etc.)
            model_name: modelo YOLO a usar (yolov8n.pt, yolov8s.pt, etc.)
            config: configuración de tracking (usa defaults si None)
        """
        self.ctrl = controller
        self.config = config or TrackingConfig()
        self.config.target_class = target_class
        self.model_name = model_name

        # --- Estado ---
        self._state = AIState.IDLE
        self._state_lock = threading.Lock()
        self._running = False
        self._ai_thread = None

        # --- YOLO (carga lazy) ---
        self._yolo = None
        self._yolo_loaded = False

        # --- PIDs ---
        self._pid_yaw = PIDController(
            kp=self.config.pid_yaw_kp,
            ki=self.config.pid_yaw_ki,
            kd=self.config.pid_yaw_kd,
            output_limits=(-self.config.max_speed, self.config.max_speed))
        self._pid_throttle = PIDController(
            kp=self.config.pid_throttle_kp,
            ki=self.config.pid_throttle_ki,
            kd=self.config.pid_throttle_kd,
            output_limits=(-self.config.throttle_range,
                           self.config.throttle_range))
        self._pid_pitch = PIDController(
            kp=self.config.pid_pitch_kp,
            ki=self.config.pid_pitch_ki,
            kd=self.config.pid_pitch_kd,
            output_limits=(-self.config.max_speed, self.config.max_speed))
        self._pid_roll = PIDController(
            kp=self.config.pid_roll_kp,
            ki=self.config.pid_roll_ki,
            kd=self.config.pid_roll_kd,
            output_limits=(-self.config.max_speed, self.config.max_speed))

        # --- Frame buffer ---
        self._frame_lock = threading.Lock()
        self._latest_frame = None       # Frame JPEG crudo (bytes)
        self._frame_updated = False

        # --- Tracking state ---
        self._last_detection_time = 0.0
        self._hover_start_time = 0.0
        self._search_direction = 1.0   # 1.0 = derecha, -1.0 = izquierda
        self._last_target = None

        # --- Stats ---
        self._stats_lock = threading.Lock()
        self._stats = {
            "state": "IDLE",
            "fps": 0.0,
            "detections": 0,
            "target": None,
            "rc_output": {"roll": 0, "pitch": 0, "throttle": 0, "yaw": 0},
            "errors": {"x": 0, "y": 0, "size": 0},
        }

        # --- Debug: historial de errores para graficar ---
        self._error_history = collections.deque(maxlen=200)

    # -----------------------------------------------------------------------
    # Propiedades
    # -----------------------------------------------------------------------
    @property
    def state(self) -> AIState:
        with self._state_lock:
            return self._state

    @state.setter
    def state(self, new_state: AIState):
        with self._state_lock:
            old = self._state
            self._state = new_state
        if old != new_state:
            logger.info("Estado: %s → %s", old.name, new_state.name)

    # -----------------------------------------------------------------------
    # Carga de YOLO
    # -----------------------------------------------------------------------
    def _load_yolo(self):
        """Carga el modelo YOLO (lazy, solo la primera vez)."""
        if self._yolo_loaded:
            return

        logger.info("Cargando modelo YOLO: %s ...", self.model_name)
        try:
            from ultralytics import YOLO
            self._yolo = YOLO(self.model_name)
            self._yolo_loaded = True
            logger.info("✓ YOLO cargado: %s", self.model_name)
        except ImportError:
            logger.error(
                "ultralytics no instalado. Instalar con: pip install ultralytics"
            )
            raise
        except Exception as e:
            logger.error("Error cargando YOLO: %s", e)
            raise

    # -----------------------------------------------------------------------
    # Detección
    # -----------------------------------------------------------------------
    def _detect(self, frame):
        """Ejecuta YOLO sobre un frame OpenCV y devuelve detecciones filtradas.

        Args:
            frame: numpy array BGR (como lo devuelve cv2.imdecode)

        Returns:
            Lista de Detection para la clase objetivo
        """
        if self._yolo is None:
            return []

        results = self._yolo(frame, verbose=False, conf=self.config.confidence_threshold)

        detections = []
        for r in results:
            if r.boxes is None:
                continue
            for box in r.boxes:
                cls_id = int(box.cls[0])
                cls_name = self._yolo.names[cls_id]
                conf = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                detections.append(Detection(cls_name, conf, x1, y1, x2, y2))

        # Filtrar por clase objetivo
        target_dets = [d for d in detections
                       if d.class_name == self.config.target_class]

        with self._stats_lock:
            self._stats["detections"] = len(target_dets)

        return target_dets

    # -----------------------------------------------------------------------
    # Selección de objetivo
    # -----------------------------------------------------------------------
    def _select_target(self, detections,
                       frame_h: int, frame_w: int):
        """Selecciona el mejor objetivo de entre las detecciones.

        Prioridad:
        1. Si hay un target previo, elegir el más cercano (continuidad)
        2. Si no, elegir el más grande (más cercano al dron)
        """
        if not detections:
            return None

        if self._last_target is not None:
            # Buscar el más cercano al último target conocido
            last_cx = self._last_target.center_x
            last_cy = self._last_target.center_y

            def distance_to_last(d):
                dx = d.center_x - last_cx
                dy = d.center_y - last_cy
                return math.sqrt(dx * dx + dy * dy)

            return min(detections, key=distance_to_last)
        else:
            # Elegir el más grande (mayor área)
            return max(detections, key=lambda d: d.area)

    # -----------------------------------------------------------------------
    # Cálculo de error y PID
    # -----------------------------------------------------------------------
    def _calculate_control(self, target: Detection,
                           frame_h: int, frame_w: int) -> dict:
        """Calcula los comandos RC a partir del error entre el target y el centro.

        Errores (normalizados -1..+1):
            error_x: target a la derecha del centro (+) / izquierda (-)
            error_y: target abajo del centro (+) / arriba (-)
            error_size: target muy lejos (+, bbox chico) / muy cerca (-, bbox grande)

        Mapeo:
            error_x → Yaw (rotar para centrar horizontalmente)
            error_y → Throttle (subir/bajar para centrar verticalmente)
            error_size → Pitch (avanzar/retroceder para mantener distancia)
        """
        now = time.time()

        # Error de posición (normalizado al centro del frame)
        cx = target.center_x / frame_w  # 0..1
        cy = target.center_y / frame_h  # 0..1
        error_x = cx - 0.5              # -0.5..+0.5 (+ = derecha)
        error_y = cy - 0.5              # -0.5..+0.5 (+ = abajo)

        # Error de tamaño (para control de distancia)
        bbox_ratio = target.area / (frame_w * frame_h)
        error_size = self.config.target_bbox_ratio - bbox_ratio
        # + = bbox muy chico = acercarse, - = bbox muy grande = alejarse

        # Aplicar deadband
        if abs(error_x) < self.config.deadband_x:
            error_x = 0
        if abs(error_y) < self.config.deadband_y:
            error_y = 0
        if abs(error_size) < self.config.deadband_size:
            error_size = 0

        # PID → RC
        yaw_output = self._pid_yaw.update(error_x * 100, now)
        throttle_adj = self._pid_throttle.update(-error_y * 100, now)
        pitch_output = self._pid_pitch.update(error_size * 100, now)
        roll_output = 0  # Roll no se usa en tracking básico
        # (podría usarse para strafing lateral avanzado)

        # Throttle = base ± ajuste vertical
        throttle_output = self.config.throttle_base + throttle_adj

        # Guardar errores para debug/gráficas
        self._error_history.append({
            "t": now,
            "ex": error_x, "ey": error_y, "es": error_size,
            "yaw": yaw_output, "pitch": pitch_output,
            "throttle": throttle_output, "roll": roll_output,
        })

        with self._stats_lock:
            self._stats["errors"] = {
                "x": round(error_x, 3),
                "y": round(error_y, 3),
                "size": round(error_size, 3),
            }
            self._stats["rc_output"] = {
                "roll": round(roll_output, 1),
                "pitch": round(pitch_output, 1),
                "throttle": round(throttle_output, 1),
                "yaw": round(yaw_output, 1),
            }
            self._stats["target"] = target.to_dict()

        return {
            "roll": roll_output,
            "pitch": pitch_output,
            "throttle": throttle_output,
            "yaw": yaw_output,
        }

    # -----------------------------------------------------------------------
    # Máquina de estados
    # -----------------------------------------------------------------------
    def _process_frame_internal(self, frame, frame_h: int, frame_w: int):
        """Procesa un frame según el estado actual de la FSM."""
        now = time.time()
        current_state = self.state

        if current_state == AIState.IDLE:
            return

        elif current_state == AIState.SEARCH:
            # Rotar lentamente buscando el objetivo
            detections = self._detect(frame)
            target = self._select_target(detections, frame_h, frame_w)

            if target:
                logger.info("🎯 Objetivo encontrado: %s (conf=%.2f)",
                            target.class_name, target.confidence)
                self._last_target = target
                self._last_detection_time = now
                self.state = AIState.TRACK
            else:
                # Rotar en busca
                speed = self.config.search_yaw_speed * self._search_direction
                self.ctrl.set_rc(yaw=speed, throttle=self.config.throttle_base)

        elif current_state == AIState.TRACK:
            detections = self._detect(frame)
            target = self._select_target(detections, frame_h, frame_w)

            if target:
                self._last_target = target
                self._last_detection_time = now

                # Calcular control PID
                rc = self._calculate_control(target, frame_h, frame_w)
                self.ctrl.set_rc(**rc)
            else:
                # Target no detectado en este frame
                if (now - self._last_detection_time) > self.config.target_lost_timeout:
                    logger.warning("⚠ Objetivo perdido → HOVER")
                    self.state = AIState.HOVER
                    self._hover_start_time = now
                    self.ctrl.hover()

        elif current_state == AIState.HOVER:
            # Mantener posición y seguir buscando
            detections = self._detect(frame)
            target = self._select_target(detections, frame_h, frame_w)

            if target:
                logger.info("🎯 Objetivo re-adquirido")
                self._last_target = target
                self._last_detection_time = now
                self.state = AIState.TRACK
            else:
                self.ctrl.hover()
                # Si pasa mucho tiempo en hover → buscar
                if (now - self._hover_start_time) > self.config.hover_timeout:
                    logger.info("Hover timeout → SEARCH")
                    self.state = AIState.SEARCH
                    self._search_direction *= -1  # Alternar dirección

        elif current_state == AIState.LAND:
            self.ctrl.land()

    # -----------------------------------------------------------------------
    # API Pública
    # -----------------------------------------------------------------------
    def feed_frame(self, jpeg_bytes: bytes):
        """Alimenta un frame JPEG al motor de IA.

        Llamar desde drone_stream.py cada vez que haya un frame nuevo.
        El procesamiento ocurre en el hilo de IA, no bloquea al caller.
        """
        with self._frame_lock:
            self._latest_frame = jpeg_bytes
            self._frame_updated = True

    def start(self):
        """Inicia el motor de IA.

        Carga YOLO y arranca el hilo de procesamiento.
        Empieza en modo SEARCH (buscando objetivo).
        """
        if self._running:
            logger.warning("IA ya está corriendo")
            return

        self._load_yolo()
        self._running = True
        self.state = AIState.SEARCH

        # Reset PIDs
        self._pid_yaw.reset()
        self._pid_throttle.reset()
        self._pid_pitch.reset()
        self._pid_roll.reset()

        self._ai_thread = threading.Thread(
            target=self._ai_loop, daemon=True, name="drone-ai")
        self._ai_thread.start()
        logger.info("🤖 Motor de IA iniciado (target: %s, modelo: %s)",
                     self.config.target_class, self.model_name)

    def stop(self):
        """Detiene el motor de IA. El dron pasa a hover."""
        if not self._running:
            return

        self._running = False
        self.state = AIState.IDLE

        if self._ai_thread and self._ai_thread.is_alive():
            self._ai_thread.join(timeout=2.0)

        self.ctrl.hover()
        logger.info("🤖 Motor de IA detenido")

    def set_target_class(self, class_name: str):
        """Cambia la clase objetivo en tiempo real."""
        self.config.target_class = class_name
        self._last_target = None
        self._pid_yaw.reset()
        self._pid_throttle.reset()
        self._pid_pitch.reset()
        self._pid_roll.reset()
        logger.info("Target cambiado a: %s", class_name)

    def get_stats(self) -> dict:
        """Devuelve estadísticas del motor de IA."""
        with self._stats_lock:
            s = dict(self._stats)
        s["state"] = self.state.name
        return s

    def get_error_history(self):
        """Devuelve el historial de errores PID (últimos 200 puntos)."""
        return list(self._error_history)

    # -----------------------------------------------------------------------
    # Hilo principal de IA
    # -----------------------------------------------------------------------
    def _ai_loop(self):
        """Hilo de procesamiento de IA.

        Lee frames del buffer, los decodifica con OpenCV, ejecuta YOLO
        y actualiza los comandos RC vía DroneController.
        """
        import cv2
        import numpy as np

        fps_counter = 0
        fps_t0 = time.time()

        while self._running:
            # Obtener frame
            with self._frame_lock:
                if not self._frame_updated or self._latest_frame is None:
                    pass
                else:
                    jpeg_data = self._latest_frame
                    self._frame_updated = False
                    frame = jpeg_data  # Placeholder, decodificamos abajo
            
            # Si no hay frame nuevo, esperar brevemente
            try:
                jpeg_data
            except UnboundLocalError:
                time.sleep(0.02)
                continue

            if not self._frame_updated and jpeg_data:
                # Decodificar JPEG → numpy array BGR
                try:
                    nparr = np.frombuffer(jpeg_data, np.uint8)
                    frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                    if frame is None:
                        continue
                except Exception:
                    continue

                frame_h, frame_w = frame.shape[:2]

                # Procesar según estado de la FSM
                self._process_frame_internal(frame, frame_h, frame_w)

                # FPS counter
                fps_counter += 1
                now = time.time()
                if now - fps_t0 >= 1.0:
                    with self._stats_lock:
                        self._stats["fps"] = fps_counter / (now - fps_t0)
                    fps_counter = 0
                    fps_t0 = now

                # Reset para siguiente iteración
                jpeg_data = None
            else:
                time.sleep(0.01)  # No hay frame nuevo, esperar

    # -----------------------------------------------------------------------
    # Utilidades
    # -----------------------------------------------------------------------
    def update_pid_gains(self, axis: str, kp: float = None,
                         ki: float = None, kd: float = None):
        """Actualiza las ganancias PID de un eje en tiempo real.

        Args:
            axis: "yaw", "throttle", "pitch", o "roll"
            kp, ki, kd: nuevas ganancias (None = no cambiar)
        """
        pid_map = {
            "yaw": self._pid_yaw,
            "throttle": self._pid_throttle,
            "pitch": self._pid_pitch,
            "roll": self._pid_roll,
        }
        pid = pid_map.get(axis)
        if pid is None:
            logger.warning("Eje desconocido: %s", axis)
            return

        if kp is not None:
            pid.kp = kp
        if ki is not None:
            pid.ki = ki
        if kd is not None:
            pid.kd = kd

        logger.info("PID %s actualizado: kp=%.3f ki=%.3f kd=%.3f",
                     axis, pid.kp, pid.ki, pid.kd)

    def __repr__(self):
        return (f"<DroneAI state={self.state.name} "
                f"target='{self.config.target_class}' "
                f"model='{self.model_name}'>")


# ===========================================================================
# Tests standalone
# ===========================================================================
if __name__ == "__main__":
    print("=" * 65)
    print(" DroneAI M22 -- Test de modulo")
    print("=" * 65)

    # Test 1: PID Controller
    pid = PIDController(kp=1.0, ki=0.1, kd=0.05)
    t = 0.0
    outputs = []
    for i in range(20):
        error = 50 - i * 5  # Error decrece linealmente
        out = pid.update(error, t)
        outputs.append(out)
        t += 0.02
    assert len(outputs) == 20
    # Salida debe empezar en 0 (primer update) y luego ser positiva
    assert outputs[0] == 0.0  # Primera iteracion siempre 0
    print(f"[OK] PID Controller: {len(outputs)} iteraciones OK")
    print(f"    Rango de salida: [{min(outputs):.1f}, {max(outputs):.1f}]")

    # Test 2: Detection
    det = Detection("person", 0.95, 100, 100, 300, 400)
    assert det.center_x == 200
    assert det.center_y == 250
    assert det.width == 200
    assert det.height == 300
    assert det.area == 60000
    print(f"[OK] Detection: center=({det.center_x}, {det.center_y}), "
          f"area={det.area}")

    # Test 3: TrackingConfig
    config = TrackingConfig()
    d = config.to_dict()
    assert "pid_yaw_kp" in d
    assert "target_class" in d
    print(f"[OK] TrackingConfig: {len(d)} parametros")

    # Test 4: DroneAI instantiation (sin controller real)
    class MockController:
        def set_rc(self, **kwargs): pass
        def hover(self): pass
        def land(self): pass

    ai = DroneAI(MockController(), target_class="person")
    assert ai.state == AIState.IDLE
    assert ai.config.target_class == "person"
    stats = ai.get_stats()
    assert stats["state"] == "IDLE"
    print(f"[OK] DroneAI: {ai}")

    # Test 5: Axis scaling
    from drone_control import DroneController
    assert DroneController._scale_axis(0) == 128
    assert DroneController._scale_axis(-100) == 1
    assert DroneController._scale_axis(100) == 255
    print("[OK] Axis scaling integrado con DroneController")

    print("\n" + "=" * 65)
    print(" Todos los tests pasaron OK")
    print(" Modulo listo: from drone_ai import DroneAI")
    print("=" * 65)
