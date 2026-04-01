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

            if self._tracking and (self._lost_count >= self.N_EXIT):
                self._tracking = False
                mode_rotate    = False
                self.vision_override.clear()

            # Lógica de control (solo durante tracking)
            if self._tracking:
                if detected and frame_w and frame_h:
                    centroid = (None, cx, cy)
                    turn = calc_turn_x(centroid, frame_w)

                    if (not mode_rotate) and (cy >= frame_h * Y_TRIGGER):
                        mode_rotate = True
                    elif mode_rotate and (cy < frame_h * Y_HYST):
                        mode_rotate = False

                    if mode_rotate:
                        rot = 20 + 20 * abs(turn)
                        if turn == 0.0:
                            left_rpm = right_rpm = 0
                            dir_left = dir_right = 1
                            self._sender.send_vision(0, 0, dir_left, dir_right)
                            command_send_counter = 0
                        elif turn > 0:
                            left_rpm = right_rpm = rot
                            dir_left, dir_right  = 1, 0
                        else:
                            left_rpm = right_rpm = rot
                            dir_left, dir_right  = 0, 1
                    else:
                        left_rpm = right_rpm = APPROACH_RPM
                        dir_left = dir_right = 1

                command_send_counter += 1
                if command_send_counter >= COMMAND_SEND_EVERY:
                    self._sender.send_vision(
                        clamp_rpm(left_rpm),
                        clamp_rpm(right_rpm),
                        dir_left, dir_right
                    )
                    command_send_counter = 0

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

    MAX_RPM = 67

    def __init__(self, esp):
        """
        esp : instancia de ESP (connect.py)
        """
        self.esp = esp

        self.lock          = threading.Lock()
        self.latest_vision = None   # puesto por send_vision() — ImgProcessorJetson
        self.latest_route  = None   # puesto por send_route()  — Route_Command
        self.stop_event    = threading.Event()
        self.thread        = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()
        return self

    def send_vision(self, left_rpm, right_rpm, dir_left, dir_right):
        """Comando de visión (mayor prioridad)."""
        left_rpm  = max(0, min(int(left_rpm),  self.MAX_RPM))
        right_rpm = max(0, min(int(right_rpm), self.MAX_RPM))
        payload = {
            "left_dir":  f"D{int(dir_left)}",
            "left_rpm":  f"S{left_rpm}",
            "right_dir": f"D{int(dir_right)}",
            "right_rpm": f"S{right_rpm}",
        }
        with self.lock:
            self.latest_vision = payload

    def send_route(self, left_dir: str, left_rpm: str, right_dir: str, right_rpm: str):
        """Comando de ruta (menor prioridad). Formato: 'D1'/'D0' y 'S<n>'."""
        payload = {
            "left_dir":  left_dir,
            "left_rpm":  left_rpm,
            "right_dir": right_dir,
            "right_rpm": right_rpm,
        }
        with self.lock:
            self.latest_route = payload

    def _run(self):
        while not self.stop_event.is_set():
            payload = None

            with self.lock:
                if self.latest_vision is not None:
                    # Prioridad 1: visión
                    payload = self.latest_vision
                    self.latest_vision = None
                elif self.latest_route is not None:
                    # Prioridad 2: ruta autónoma
                    payload = self.latest_route
                    self.latest_route = None

            if payload:
                self.esp.send_uart(
                    payload["left_dir"],
                    payload["left_rpm"],
                    payload["right_dir"],
                    payload["right_rpm"],
                )

            time.sleep(0.005)

    def stop(self):
        self.stop_event.set()
        self.thread.join(timeout=1.0)


# -----------------------------------------------------------
#                           Tools                                                             
# -----------------------------------------------------------

def calc_turn_x(centroid, frame_width, deadband_px=50):
    cx       = centroid[1]
    error_px = cx - frame_width / 2

    if abs(error_px) < deadband_px:
        return 0.0

    turn = error_px / (frame_width / 2)
    return max(-1.0, min(1.0, turn))


def clamp_rpm(rpm, min_rpm=20):
    if rpm < min_rpm:
        return 0
    return int(rpm)