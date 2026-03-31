import requests
import time
import threading
import cv2 as cv
from vision import detect_colors, crosslines

# ---------------------------
# Hilo de captura (latest-frame)
# ---------------------------
class ImgProcessor:
    def __init__(self, PI_IP, warmup_sec=0.0, vision_override=None):
        self.video_url = f"http://{PI_IP}:5000/video_feed"
        self.depth_url = f"http://{PI_IP}:5000/depth_at"

        self.cap = cv.VideoCapture(self.video_url)
        if not self.cap.isOpened():
            raise RuntimeError(f"No se pudo abrir el stream: {self.video_url}")

        self.session = requests.Session()

        self.lock = threading.Lock()
        self.frame = None
        self.ok = False
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.vision_override = vision_override

        self.last_depth = None
        self.last_depth_ts = 0.0

        if warmup_sec > 0:
            t0 = time.time()
            while time.time() - t0 < warmup_sec:
                self.cap.read()

    def start(self):
        self.thread.start()
        return self

    def _run(self):
        """Hilo background: captura el frame más reciente continuamente."""
        while not self.stop_event.is_set():
            ok, f = self.cap.read()
            if not ok:
                self.ok = False
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

    def query_depth(self, x, y, timeout=0.08):
        try:
            r = self.session.post(
                self.depth_url,
                json={"x": int(x), "y": int(y)},
                timeout=timeout
            )
            r.raise_for_status()
            data = r.json()
            if not data.get("ok", False):
                return None
            return data
        except requests.exceptions.RequestException:
            return None

    def run_tracker(self, sender: "Sender"):
        """
        Loop principal de visión y control. Bloqueante hasta pulsar ESC.
        """

        # --- Parámetros de estabilidad ---
        N_ENTER      = 5
        N_EXIT       = 10
        AREA_MIN     = 500
        CRUISE_RPM   = 30
        APPROACH_RPM = 25
        Y_TRIGGER    = 0.40
        Y_HYST       = 0.35
        COMMAND_SEND_EVERY = 2

        seen_count = lost_count = command_send_counter = 0
        tracking = mode_rotate = False
        left_rpm = right_rpm = CRUISE_RPM
        dir_left = dir_right = 1

        while not self.stop_event.is_set():
            ok, frame = self.read()
            if not ok or frame is None:
                time.sleep(0.005)
                continue

            colors, centroids, areas = detect_colors(frame, draw=True)
            target   = pick_target(centroids, areas, area_min=AREA_MIN)
            detected = target is not None

            depth_info = None
            if detected:
                centroid, area = target
                cx = centroid[1]
                cy = centroid[2]

                depth_info = self.query_depth(cx, cy)
                if depth_info is not None:
                    self.last_depth = depth_info.get("distance_m", None)
                    self.last_depth_ts = depth_info.get("timestamp", 0.0)

            if detected:
                centroid, area = target
                seen_count += 1
                lost_count  = 0
            else:
                lost_count  += 1
                seen_count   = 0

            # Transiciones tracking
            if (not tracking) and (seen_count >= N_ENTER):
                tracking = True
                if self.vision_override:
                    self.vision_override.set()

            if tracking and (lost_count >= N_EXIT):
                tracking    = False
                mode_rotate = False
                if self.vision_override:
                    self.vision_override.clear()

            # Lógica de seguimiento y envío de comandos EXCLUSIVO durante el tracking
            if tracking:
                if detected:
                    h, w = frame.shape[:2]
                    cy   = centroid[2]
                    turn = calc_turn_x(centroid, w)

                    if (not mode_rotate) and (cy >= h * Y_TRIGGER):
                        mode_rotate = True
                    elif mode_rotate and (cy < h * Y_HYST):
                        mode_rotate = False

                    if mode_rotate:
                        rot = 20 + 20 * abs(turn)
                        if turn == 0.0:
                            left_rpm = right_rpm = 0
                            dir_left = dir_right = 1
                            sender.send_rpms(0, 0, dir_left, dir_right)
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
                    sender.send_rpms(
                        clamp_rpm(left_rpm),
                        clamp_rpm(right_rpm),
                        dir_left, dir_right
                    )
                    command_send_counter = 0

            video = crosslines(frame.copy())

            if detected:
                centroid, area = target
                cx = centroid[1]
                cy = centroid[2]

                cv.putText(video, f"Area: {int(area)}", (cx + 10, cy + 20),
                           cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                if self.last_depth is not None:
                    depth_text = f"Dist: {self.last_depth:.2f} m"
                else:
                    depth_text = "Dist: invalida"

                cv.putText(video, depth_text, (cx + 10, cy + 45),
                           cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            cv.imshow("VISION ROVER (PC DEBUG)", video)

            if cv.waitKey(1) & 0xFF == 27:  # ESC
                break

        cv.destroyAllWindows()

    def stop(self):
        self.stop_event.set()
        self.thread.join(timeout=1.0)
        self.cap.release()
        self.session.close()


# ---------------------------
# Hilo para envíos HTTP
# ---------------------------
class Sender:
    def __init__(self, PI_IP):
        self.cmd_url = f"http://{PI_IP}:5000/rcv_speed_dir"
        self.session = requests.Session()
        self.lock = threading.Lock()
        self.latest_rpm = None
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.MAX_RPM = 67

    def start(self):
        self.thread.start()
        return self

    def send_rpms(self, left_rpm, right_rpm, dir_left, dir_right):
        left_rpm  = max(0, min(int(left_rpm),  self.MAX_RPM))
        right_rpm = max(0, min(int(right_rpm), self.MAX_RPM))
        payload = {
            "left_rpm":  f"S{left_rpm}",
            "right_rpm": f"S{right_rpm}",
            "left_dir":  f"D{int(dir_left)}",
            "right_dir": f"D{int(dir_right)}",
        }
        with self.lock:
            self.latest_rpm = payload

    def _run(self):
        while not self.stop_event.is_set():
            rpm = None
            with self.lock:
                rpm = self.latest_rpm
                self.latest_rpm = None
            try:
                if rpm is not None:
                    self.session.post(self.cmd_url, json=rpm, timeout=0.2)
            except requests.exceptions.RequestException:
                pass

            time.sleep(0.005)

    def stop(self):
        self.stop_event.set()
        self.thread.join(timeout=1.0)
        self.session.close()


# --------------- Tools -------------------

def calc_turn_x(centroid, frame_width, deadband_px=50):
    cx = centroid[1]
    center_x = frame_width / 2
    error_px = cx - center_x

    if abs(error_px) < deadband_px:
        return 0.0
    
    error = error_px / (frame_width / 2)
    Kp = 1
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