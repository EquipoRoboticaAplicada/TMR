import requests
import time
import threading
import cv2 as cv

# ---------------------------
# Hilo de captura (latest-frame)
# ---------------------------
class FrameGrabber:
    def __init__(self, src, warmup_sec=0.0):
        self.cap = cv.VideoCapture(src)
        self.lock = threading.Lock()
        self.frame = None
        self.ok = False
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

        if not self.cap.isOpened():
            raise RuntimeError("No se pudo abrir el stream")

        # opcional: dejar que el buffer se estabilice
        if warmup_sec > 0:
            t0 = time.time()
            while time.time() - t0 < warmup_sec:
                self.cap.read()

    def start(self):
        self.thread.start()
        return self

    def _run(self):
        while not self.stop_event.is_set():
            ok, f = self.cap.read()
            if not ok:
                self.ok = False
                # evita busy-loop si el stream se corta momentáneamente
                time.sleep(0.01)
                continue

            with self.lock:
                self.frame = f
                self.ok = True

    def read(self):
        with self.lock:
            if self.frame is None:
                return False, None
            return self.ok, self.frame.copy()

    def stop(self):
        self.stop_event.set()
        self.thread.join(timeout=1.0)
        self.cap.release()


# ---------------------------
# Hilo para envíos HTTP
# ---------------------------
class Sender:
    def __init__(self, cmd_url, vision_url):
        self.cmd_url = cmd_url
        self.vision_url = vision_url
        self.session = requests.Session()
        self.lock = threading.Lock()
        self.latest_cmd = None          # dict
        self.latest_vision = None       # dict
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()
        return self

    def send_cmd(self, payload):
        with self.lock:
            self.latest_cmd = payload

    def send_vision(self, payload):
        with self.lock:
            self.latest_vision = payload

    def _run(self):
        while not self.stop_event.is_set():
            cmd = None
            vis = None
            with self.lock:
                cmd = self.latest_cmd
                vis = self.latest_vision
                self.latest_cmd = None
                self.latest_vision = None

            # manda lo más reciente, descarta acumulación
            try:
                if cmd is not None:
                    self.session.post(self.cmd_url, json=cmd, timeout=0.2)
                if vis is not None:
                    self.session.post(self.vision_url, json=vis, timeout=0.2)
            except:
                pass

            time.sleep(0.005)  # reduce CPU, mantiene baja latencia

    def stop(self):
        self.stop_event.set()
        self.thread.join(timeout=1.0)
        self.session.close()


# ---------------------------
# Utilidades 
# ---------------------------


def send_rpms(sender, left_rpm, right_rpm, dir_left, dir_right, ticks_left=0, ticks_right=0):
    # NUEVO: Estructura idéntica a mapeo_trayectoria.py integrando los ticks
    rover_state = {
        "left_side":  {"seq": 0, "dt_ms": 0, "motors": [{"ticks": ticks_left, "m/s": 0.0} for _ in range(3)]},
        "right_side": {"seq": 0, "dt_ms": 0, "motors": [{"ticks": ticks_right, "m/s": 0.0} for _ in range(3)]},
        "last_update": time.time()
    }

    payload = {
        "left_rpm":  f"S{int(left_rpm)}",
        "right_rpm": f"S{int(right_rpm)}",
        "left_dir":  f"D{int(dir_left)}",
        "right_dir": f"D{int(dir_right)}",
        "rover_state": rover_state  # Se adjunta el estado al payload
    }
    sender.send_cmd(payload)

# def send_vision_data(sender, colors, centroids, areas):
#     payload = {
#         "colors": list(colors) if isinstance(colors, set) else colors,
#         "centroids": centroids,
#         "areas": areas,
#         "time": time.time()
#     }
#     sender.send_vision(payload)

def calc_turn_x(centroid, frame_width, deadband_px=10):
    cx = centroid[1]
    center_x = frame_width / 2
    error_px = cx - center_x
    if abs(error_px) < deadband_px:
        return 0.0
    error = error_px / (frame_width / 2)
    Kp = 5
    turn = Kp * error
    return max(-1.0, min(1.0, turn))

def pick_target(centroids, areas, area_min=1500):
    if not centroids or not areas:
        return None
    candidates = [(c, a) for c, a in zip(centroids, areas) if a >= area_min]
    if not candidates:
        return None
    centroid, area = max(candidates, key=lambda x: x[1])
    return centroid, area

def clamp_rpm(rpm, min_rpm=20):
    if rpm < min_rpm:
        return 0
    return int(rpm)
