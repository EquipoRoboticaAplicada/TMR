import time
import threading


# ------------------------------------------------------------------ #
# ImgProcessorJetson                                                   #
# Lee el estado de VisionZED localmente y controla el rover           #
# cuando detecta un objeto (vision_override activo).                  #
# ------------------------------------------------------------------ #

class ImgProcessorJetson:

    def __init__(self, vision_zed):
        """
        vision_zed : instancia de VisionZED (ya iniciada)
        """
        self.vision = vision_zed

        self.stop_event  = threading.Event()
        self.thread      = None

        # Estado de tracking
        self._tracking       = False
        self.vision_override = threading.Event()  # consultado por Route_Command y SenderJetson

        # Parámetros de estabilidad
        self.N_ENTER     = 5
        self.N_EXIT      = 10
        self._seen_count = 0
        self._lost_count = 0

        # Referencia al sender (asignada en start)
        self._sender = None

    def is_tracking(self) -> bool:
        return self._tracking

    def start(self, sender: "SenderJetson"):
        self._sender = sender
        self.thread  = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        return self

    def _run(self):
        AREA_MIN     = 500
        APPROACH_RPM = 25
        Y_TRIGGER    = 0.40
        Y_HYST       = 0.35
        COMMAND_SEND_EVERY = 2

        mode_rotate          = False
        left_rpm = right_rpm = APPROACH_RPM
        dir_left = dir_right = 1
        command_send_counter = 0

        while not self.stop_event.is_set():
            # Si el control manual está activo, pausar el procesamiento de tracking/brazo
            if self._sender is not None and self._sender.manual_override.is_set():
                if self._tracking:
                    self._tracking = False
                    self.vision_override.clear()
                self._seen_count = 0
                time.sleep(0.05)
                continue

            state    = self.vision.get_state()
            detected = state.get("detected", False)
            cx       = state.get("cx")
            cy       = state.get("cy")
            area     = state.get("area", 0)
            frame_w  = state.get("frame_width")
            frame_h  = state.get("frame_height")

            # Filtro por área mínima
            if detected and (area is None or area < AREA_MIN):
                detected = False

            # Contadores de estabilidad
            if detected:
                self._seen_count += 1
                self._lost_count  = 0
            else:
                self._lost_count  += 1
                self._seen_count   = 0

            # Transiciones de tracking
            if (not self._tracking) and (self._seen_count >= self.N_ENTER):
                self._tracking = True
                self.vision_override.set()

            elif self._tracking and (self._lost_count >= self.N_EXIT):
                self._tracking = False
                mode_rotate    = False
                self.vision_override.clear()

            # Lógica de control (solo durante tracking)
            if self._tracking:

                if detected and frame_w and frame_h:

                    turn = calc_turn_x(cx, frame_w)

                    # Si el objeto está centrado
                    if turn == 0.0:

                        self._sender.send_vision(0)

                        self._sender.esp.act_arm()
                        self._sender.esp.wait_for_peso_change()

                    else:
                        # No podemos girar con una sola velocidad.
                        # Por seguridad, detenemos el rover.
                        self._sender.send_vision(0)

                else:
                    # Si se pierde temporalmente el objeto, detenerse
                    self._sender.send_vision(0)

            time.sleep(0.01)

    def stop(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=1.0)


# ------------------------------------------------------------------ #
# SenderJetson                                                         #
# Único punto de escritura al ESP. Gestiona tres fuentes con          #
# prioridad fija:                                                      #
#   1. Manual  — cuando manual_override está activo (DualSense/MQTT)  #
#   2. Visión  — cuando vision_override está activo                   #
#   3. Ruta    — cuando Route_Command envía comandos                  #
# ------------------------------------------------------------------ #

class SenderJetson:

    MAX_RPM          = 10
    MANUAL_MAX_RPM   = 25
    MANUAL_TIMEOUT_S = 0.45   # Watchdog: frena a 0 si dejan de llegar paquetes manuales

    def __init__(self, esp):
        """
        esp : instancia de ESP (connect.py)
        """
        self.esp = esp

        self.lock = threading.Lock()

        self.manual_override   = threading.Event()
        self.manual_max_rpm    = self.MANUAL_MAX_RPM
        self.latest_manual     = None
        self._last_manual_ts   = 0.0
        self._watchdog_tripped = False

        self.latest_vision = None
        self.latest_route  = None

        self._last_sent_manual    = None
        self._last_sent_manual_ts = 0.0

        self.stop_event = threading.Event()
        self.thread = threading.Thread(
            target=self._run,
            daemon=True
        )

    def start(self):
        self.thread.start()
        return self

    def set_manual_mode(self, enabled: bool):
        """
        Activa o desactiva el modo manual (DualSense).
        Al cambiar de modo siempre envía paro ("0", "0") por seguridad.
        """
        now = time.time()
        with self.lock:
            self.latest_vision = None
            self.latest_route  = None
            self._last_sent_manual = None
            if enabled:
                self.manual_override.set()
                self.latest_manual     = ("0", "0")
                self._last_manual_ts   = now
                self._watchdog_tripped = False
            else:
                self.manual_override.clear()
                self.latest_manual     = None
                self._watchdog_tripped = False

        try:
            self.esp.send_uart("0", "0")
        except Exception as e:
            print(f"[SenderJetson] Error deteniendo en cambio de modo: {e}")

    def send_manual(self, left_rpm: int, right_rpm: int, max_rpm: int = None):
        """
        Comando manual de baja latencia con control diferencial (Skid-Steer).
        left_rpm / right_rpm con signo: positivo = adelante, negativo = atrás.
        """
        limit = int(max_rpm) if max_rpm is not None else self.manual_max_rpm
        left_rpm  = max(-limit, min(int(left_rpm),  limit))
        right_rpm = max(-limit, min(int(right_rpm), limit))
        now = time.time()
        with self.lock:
            if not self.manual_override.is_set():
                self.manual_override.set()
            self.latest_manual     = (str(left_rpm), str(right_rpm))
            self._last_manual_ts   = now
            self._watchdog_tripped = False

    def emergency_stop(self):
        """
        Paro de emergencia inmediato: toma control manual y fuerza ambos ESPs a 0 RPM.
        """
        with self.lock:
            self.manual_override.set()
            self.latest_manual     = ("0", "0")
            self.latest_vision     = None
            self.latest_route      = None
            self._last_manual_ts   = time.time()
            self._last_sent_manual = ("0", "0")
        try:
            self.esp.send_uart("0", "0")
        except Exception as e:
            print(f"[SenderJetson] Error en emergency_stop: {e}")

    def send_vision(self, rpm):
        """
        Comando de vision: mismo RPM para ambos lados.
        El valor se pasa como string a send_uart, que agrega el prefijo S.
        """
        rpm = max(-self.MAX_RPM, min(int(rpm), self.MAX_RPM))
        val = str(rpm)
        with self.lock:
            if not self.manual_override.is_set():
                self.latest_vision = (val, val)

    def send_route(self, left_rpm: int, right_rpm: int):
        """
        Comando de ruta con control diferencial.
        left_rpm / right_rpm con signo: positivo = adelante, negativo = atras.
        """
        left_rpm  = max(-self.MAX_RPM, min(int(left_rpm),  self.MAX_RPM))
        right_rpm = max(-self.MAX_RPM, min(int(right_rpm), self.MAX_RPM))
        with self.lock:
            if not self.manual_override.is_set():
                self.latest_route = (str(left_rpm), str(right_rpm))

    def _run(self):
        UART_KEEPALIVE_S = 0.08  # Refresco periódico a los ESPs en modo manual

        while not self.stop_event.is_set():

            payload = None
            now     = time.time()

            with self.lock:
                # Prioridad 1: Modo Manual (con Watchdog de desconexión)
                if self.manual_override.is_set():
                    self.latest_vision = None
                    self.latest_route  = None

                    if (now - self._last_manual_ts) > self.MANUAL_TIMEOUT_S:
                        if not self._watchdog_tripped:
                            self._watchdog_tripped = True
                            self.latest_manual     = ("0", "0")
                            payload                = ("0", "0")
                            self._last_sent_manual = payload
                            self._last_sent_manual_ts = now
                            print("[SenderJetson] ⚠️ Watchdog manual: sin señal >450ms, frenando motores.")
                    elif self.latest_manual is not None:
                        # Enviar de inmediato si cambió el setpoint o si toca refresco UART
                        if (self.latest_manual != self._last_sent_manual or
                                (now - self._last_sent_manual_ts) >= UART_KEEPALIVE_S):
                            payload = self.latest_manual
                            self._last_sent_manual    = payload
                            self._last_sent_manual_ts = now

                # Prioridad 2: Visión
                elif self.latest_vision is not None:
                    payload = self.latest_vision
                    self.latest_vision = None

                # Prioridad 3: Ruta Autónoma por Odometría
                elif self.latest_route is not None:
                    payload = self.latest_route
                    self.latest_route = None

            if payload is not None:
                try:
                    self.esp.send_uart(payload[0], payload[1])
                except Exception as e:
                    print(f"[SenderJetson] Error send_uart: {e}")

            time.sleep(0.005)

    def stop(self):
        self.stop_event.set()

        if self.thread is not None:
            self.thread.join(timeout=1.0)


# -----------------------------------------------------------
#                           Tools                                                             
# -----------------------------------------------------------

def calc_turn_x(cx, frame_width, deadband_px=50):
    error_px = cx - frame_width / 2

    if abs(error_px) < deadband_px:
        return 0.0

    turn = error_px / (frame_width / 2)
    return max(-1.0, min(1.0, turn))


def clamp_rpm(rpm, min_rpm=5):
    if rpm < min_rpm:
        return 0
    return int(rpm)