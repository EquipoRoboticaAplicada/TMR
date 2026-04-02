# vision_zed.py

import json
from pathlib import Path
import threading
import time
import cv2 as cv
import numpy as np
import pyzed.sl as sl

color_file = Path(__file__).resolve().parent / "config" / "colors.json"

#============================ZED CAMERA====================================

class ZEDShared:
    def __init__(self,
                 resolution="VGA",
                 fps=15,
                 depth_mode="NEURAL",
                 min_depth=0.2,
                 max_depth=20.0,
                 confidence_threshold=50):

        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread = None

        self.zed = sl.Camera()

        init = sl.InitParameters()
        init.camera_fps = fps
        init.coordinate_units = sl.UNIT.METER
        init.depth_minimum_distance = min_depth
        init.depth_maximum_distance = max_depth

        if resolution == "HD720":
            init.camera_resolution = sl.RESOLUTION.HD720
        elif resolution == "VGA":
            init.camera_resolution = sl.RESOLUTION.VGA
        else:
            init.camera_resolution = sl.RESOLUTION.HD720

        if depth_mode == "ULTRA":
            init.depth_mode = sl.DEPTH_MODE.ULTRA
        elif depth_mode == "QUALITY":
            init.depth_mode = sl.DEPTH_MODE.QUALITY
        elif depth_mode == "NEURAL":
            init.depth_mode = sl.DEPTH_MODE.NEURAL
        else:
            init.depth_mode = sl.DEPTH_MODE.PERFORMANCE

        status = self.zed.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"No se pudo abrir la ZED: {status}")

        self.runtime = sl.RuntimeParameters()
        self.runtime.confidence_threshold = confidence_threshold

        self.image_mat = sl.Mat()
        self.depth_mat = sl.Mat()

        self.last_frame_bgr = None
        self.last_timestamp = 0.0
        self.frame_h = None
        self.frame_w = None

    def start(self):
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        return self

    def _run(self):
        while not self.stop_event.is_set():
            if self.zed.grab(self.runtime) == sl.ERROR_CODE.SUCCESS:
                self.zed.retrieve_image(self.image_mat, sl.VIEW.LEFT)
                self.zed.retrieve_measure(self.depth_mat, sl.MEASURE.DEPTH)

                img = self.image_mat.get_data()
                if img is None:
                    continue

                # Normalmente la ZED devuelve BGRA
                if len(img.shape) == 3 and img.shape[2] == 4:
                    frame_bgr = cv.cvtColor(img, cv.COLOR_BGRA2BGR)
                else:
                    frame_bgr = img.copy()

                with self.lock:
                    self.last_frame_bgr = frame_bgr
                    self.frame_h, self.frame_w = frame_bgr.shape[:2]
                    self.last_timestamp = time.time()
            else:
                time.sleep(0.001)

    def get_frame_copy(self):
        with self.lock:
            if self.last_frame_bgr is None:
                return None
            return self.last_frame_bgr.copy(), self.last_timestamp

    def get_depth_at(self, x: int, y: int):
        with self.lock:
            if self.frame_w is None or self.frame_h is None:
                return None

            x = max(0, min(int(x), self.frame_w - 1))
            y = max(0, min(int(y), self.frame_h - 1))

            err, depth_value = self.depth_mat.get_value(x, y)

        if err != sl.ERROR_CODE.SUCCESS:
            return None

        if isinstance(depth_value, (list, tuple, np.ndarray)):
            if len(depth_value) == 0:
                return None
            depth_value = depth_value[0]

        if depth_value is None:
            return None

        if not np.isfinite(depth_value) or depth_value <= 0:
            return None

        return float(depth_value)

    def get_depth_median_neighborhood(self, x: int, y: int, radius: int = 2):
        with self.lock:
            if self.frame_w is None or self.frame_h is None:
                return None
            vals = []
            for yy in range(y - radius, y + radius + 1):
                for xx in range(x - radius, x + radius + 1):
                    cx = max(0, min(int(xx), self.frame_w - 1))
                    cy = max(0, min(int(yy), self.frame_h - 1))
                    err, d = self.depth_mat.get_value(cx, cy)
                    if err == sl.ERROR_CODE.SUCCESS and d is not None and np.isfinite(d) and d > 0:
                        vals.append(float(d))
        return float(np.median(vals)) if vals else None

    def stop(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=1.0)
        self.zed.close()


#============================VISION====================================

DRAW = {
    "green": (0, 255, 0),
    "blue":  (255, 0, 0),
    "red":   (0, 0, 255)
}

MIN_AREA = 600
KERNEL = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5))


def load_color_ranges():
    try: 
        with open(color_file, "r", encoding="utf-8") as f:
            return json.load(f)
    except FileNotFoundError as e:
        print(f"Error al cargar {color_file}: {e}")
        return {}

def process_mask(mask):
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, KERNEL, iterations=1)
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, KERNEL, iterations=1)
    return mask


def find_and_draw(mask, frame_draw, label, draw=True):
    found = False
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    detected_centroids = []
    areas = []

    for cnt in contours:
        area = cv.contourArea(cnt)
        if area > MIN_AREA:
            found = True
            areas.append(area)

            M = cv.moments(cnt)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                detected_centroids.append((label, cx, cy))

                if draw:
                    x, y, w, h = cv.boundingRect(cnt)
                    color_bgr = DRAW.get(label, (255, 255, 255))
                    cv.rectangle(frame_draw, (x, y), (x + w, y + h), color_bgr, 2)
                    cv.putText(frame_draw, label, (x, y - 8),
                               cv.FONT_HERSHEY_SIMPLEX, 0.7, color_bgr, 2, cv.LINE_AA)
                    cv.circle(frame_draw, (cx, cy), 5, (255, 255, 255), -1)

    return found, detected_centroids, areas


def detect_colors(frame, color_ranges, draw=False):
    colors = []
    centroids = []
    areas = []

    blurred = cv.GaussianBlur(frame, (5, 5), 0)
    hsv = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)

    for color, ranges in color_ranges.items():
        if isinstance(ranges, dict):
            lower = np.array(ranges["lower"], dtype=np.uint8)
            upper = np.array(ranges["upper"], dtype=np.uint8)
            mask = cv.inRange(hsv, lower, upper)

        elif isinstance(ranges, list):
            mask = None
            for r in ranges:
                lower = np.array(r["lower"], dtype=np.uint8)
                upper = np.array(r["upper"], dtype=np.uint8)
                m = cv.inRange(hsv, lower, upper)
                mask = m if mask is None else cv.bitwise_or(mask, m)

            if mask is None:
                continue
        else:
            continue

        mask = process_mask(mask)

        found, c_list, a_list = find_and_draw(mask, frame, color, draw)

        if found:
            colors.extend([color] * len(c_list))
            centroids.extend(c_list)
            areas.extend(a_list)

    return colors, centroids, areas


def pick_target(centroids, areas, area_min=1500):
    if not centroids or not areas:
        return None
    candidates = [(c, a) for c, a in zip(centroids, areas) if a >= area_min]
    if not candidates:
        return None
    centroid, area = max(candidates, key=lambda x: x[1])
    return centroid, area


#============================MIX ZED & VISION====================================

class VisionZED:
    def __init__(self, zed_shared, area_min=500, draw_local=False):
        self.zed = zed_shared
        self.color_ranges = load_color_ranges()
        self.area_min = area_min
        self.draw_local = draw_local

        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread = None

        self.state = {
            "detected": False,
            "label": None,
            "cx": None,
            "cy": None,
            "area": None,
            "distance_m": None,
            "timestamp": 0.0,
            "frame_width": None,
            "frame_height": None
        }

    def start(self):
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        return self

    def _run(self):
        while not self.stop_event.is_set():
            result = self.zed.get_frame_copy()
            if result is None:
                time.sleep(0.005)
                continue

            frame, ts = result
            h, w = frame.shape[:2]

            if self.draw_local:
                draw_frame = frame.copy()
            else:
                draw_frame = frame   # sin copia extra — detect_colors solo lee para HSV

            colors, centroids, areas = detect_colors(draw_frame, self.color_ranges, draw=self.draw_local)
            target = pick_target(centroids, areas, area_min=self.area_min)

            new_state = {
                "detected": False,
                "label": None,
                "cx": None,
                "cy": None,
                "area": None,
                "distance_m": None,
                "timestamp": ts,
                "frame_width": w,
                "frame_height": h
            }

            if target is not None:
                centroid, area = target
                label, cx, cy = centroid
                distance_m = self.zed.get_depth_median_neighborhood(cx, cy, radius=2)

                new_state.update({
                    "detected": True,
                    "label": label,
                    "cx": int(cx),
                    "cy": int(cy),
                    "area": float(area),
                    "distance_m": distance_m,
                    "timestamp": ts,
                    "frame_width": w,
                    "frame_height": h
                })

                if self.draw_local:
                    txt = f"Dist: {distance_m:.2f} m" if distance_m is not None else "Dist: invalida"
                    cv.putText(debug_frame, txt, (cx + 10, cy + 20),
                               cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            with self.lock:
                self.state = new_state

            if self.draw_local:
                cv.imshow("VISION ZED (JETSON)", debug_frame)
                if cv.waitKey(1) & 0xFF == 27:
                    self.stop_event.set()
                    break

    def get_state(self):
        with self.lock:
            return dict(self.state)

    def stop(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=1.0)
        if self.draw_local:
            cv.destroyAllWindows()