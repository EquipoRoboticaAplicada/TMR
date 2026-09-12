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
# Único punto de escritura al ESP. Gestiona dos fuentes con           #
# prioridad fija:                                                      #
#   1. Visión  — cuando vision_override está activo                   #
#   2. Ruta    — cuando Route_Command envía comandos                  #
# ------------------------------------------------------------------ #

class SenderJetson:

    MAX_RPM = 30

    def __init__(self, esp):
        """
        esp : instancia de ESP (connect.py)
        """
        self.esp = esp

        self.lock = threading.Lock()

        self.latest_vision = None
        self.latest_route = None

        self.stop_event = threading.Event()
        self.thread = threading.Thread(
            target=self._run,
            daemon=True
        )

    def start(self):
        self.thread.start()
        return self

    def send_vision(self, rpm):
        """
        Comando de vision: mismo RPM para ambos lados.
        El valor se pasa como string a send_uart, que agrega el prefijo S.
        """
        rpm = max(-self.MAX_RPM, min(int(rpm), self.MAX_RPM))
        val = str(rpm)
        with self.lock:
            self.latest_vision = (val, val)

    def send_route(self, left_rpm: int, right_rpm: int):
        """
        Comando de ruta con control diferencial.
        left_rpm / right_rpm con signo: positivo = adelante, negativo = atras.
        """
        left_rpm  = max(-self.MAX_RPM, min(int(left_rpm),  self.MAX_RPM))
        right_rpm = max(-self.MAX_RPM, min(int(right_rpm), self.MAX_RPM))
        with self.lock:
            self.latest_route = (str(left_rpm), str(right_rpm))

    def _run(self):
        while not self.stop_event.is_set():

            payload = None

            with self.lock:
                if self.latest_vision is not None:
                    payload = self.latest_vision
                    self.latest_vision = None

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


def clamp_rpm(rpm, min_rpm=20):
    if rpm < min_rpm:
        return 0
    return int(rpm)