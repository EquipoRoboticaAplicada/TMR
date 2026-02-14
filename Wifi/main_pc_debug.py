#
# Este código necesita también del código: vision.py
#
# La función es recibir el video de la raspberry y procesarlo con vision, escribir el color que se encontró en la terminal al igual que su centroide y área.
#
# Aquí se incluyen funciones para debuggear, transmite a la página y se abre el cuadro de video con bounding boxes y centroides.

from vision import detect_colors, crosslines
import cv2 as cv
import requests
import time
import threading

PI_IP = "172.32.236.53"

VIDEO_URL  = f"http://{PI_IP}:5000/video_feed"
CMD_URL    = f"http://{PI_IP}:5000/command"
VISION_URL = f"http://{PI_IP}:5000/vision_data"


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
# Hilo opcional para envíos HTTP
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
# Utilidades (tu lógica)
# ---------------------------
def send_rpms(sender, left_rpm, right_rpm, dir_left, dir_right):
    payload = {
        "left_rpm":  f"S{int(left_rpm)}",
        "right_rpm": f"S{int(right_rpm)}",
        "left_dir":  f"D{int(dir_left)}",
        "right_dir": f"D{int(dir_right)}",
    }
    sender.send_cmd(payload)

def send_vision_data(sender, colors, centroids, areas):
    payload = {
        "colors": list(colors) if isinstance(colors, set) else colors,
        "centroids": centroids,
        "areas": areas,
        "time": time.time()
    }
    sender.send_vision(payload)

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


def main():
    # Hilo captura
    grabber = FrameGrabber(VIDEO_URL).start()

    # Hilo envío (opcional pero recomendado)
    sender = Sender(CMD_URL, VISION_URL).start()

    # --- Parámetros de estabilidad ---
    N_ENTER = 5
    N_EXIT  = 10
    AREA_MIN = 500

    seen_count = 0
    lost_count = 0
    tracking = False

    CRUISE_RPM = 30
    APPROACH_RPM = 25

    left_rpm, right_rpm = CRUISE_RPM, CRUISE_RPM
    dir_left, dir_right = 1, 1

    VISION_SEND_EVERY = 5
    COMMAND_SEND_EVERY = 2
    vision_send_counter = 0
    command_send_counter = 0

    mode_rotate = False
    Y_TRIGGER = 0.40
    Y_HYST = 0.35

    while True:
        ok, frame = grabber.read()
        if not ok or frame is None:
            # no bloquea; espera el primer frame real
            time.sleep(0.005)
            continue

        colors, centroids, areas = detect_colors(frame, draw=True)

        target = pick_target(centroids, areas, area_min=AREA_MIN)
        detected = target is not None

        if detected:
            centroid, area = target

        # contadores
        if detected:
            seen_count += 1
            lost_count = 0
        else:
            lost_count += 1
            seen_count = 0

        # transiciones tracking
        if (not tracking) and (seen_count >= N_ENTER):
            tracking = True

        if tracking and (lost_count >= N_EXIT):
            tracking = False
            mode_rotate = False

        if not tracking:
            left_rpm, right_rpm = CRUISE_RPM, CRUISE_RPM
            dir_left, dir_right = 1, 1

        # dentro tracking
        if tracking and detected:
            h, w = frame.shape[:2]
            cy = centroid[2]
            turn = calc_turn_x(centroid, w)

            if (not mode_rotate) and (cy >= h * Y_TRIGGER):
                mode_rotate = True
            elif mode_rotate and (cy < h * Y_HYST):
                mode_rotate = False

            if mode_rotate:
                rot = APPROACH_RPM * abs(turn)
                if abs(turn) < 0.1:
                    left_rpm, right_rpm = 0, 0
                    dir_left, dir_right = 1, 1
                elif turn > 0:
                    left_rpm, right_rpm = rot, rot
                    dir_left, dir_right = 0, 1
                else:
                    left_rpm, right_rpm = rot, rot
                    dir_left, dir_right = 1, 0
            else:
                left_rpm, right_rpm = APPROACH_RPM, APPROACH_RPM
                dir_left, dir_right = 1, 1

        # envío comandos (no bloquea por requests)
        left_rpm_c = clamp_rpm(left_rpm, min_rpm=20)
        right_rpm_c = clamp_rpm(right_rpm, min_rpm=20)

        command_send_counter += 1
        if command_send_counter >= COMMAND_SEND_EVERY:
            send_rpms(sender, left_rpm_c, right_rpm_c, dir_left, dir_right)
            command_send_counter = 0

        vision_send_counter += 1
        if vision_send_counter >= VISION_SEND_EVERY:
            send_vision_data(sender, colors, centroids, areas)
            vision_send_counter = 0

        video = crosslines(frame.copy())
        cv.imshow("VISION ROVER (PC DEBUG)", video)

        if cv.waitKey(1) & 0xFF == 27:
            break

    sender.stop()
    grabber.stop()
    cv.destroyAllWindows()


if __name__ == "__main__":
    main()
