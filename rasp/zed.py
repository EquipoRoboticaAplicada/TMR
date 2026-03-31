# zed_shared.py
import threading
import time
import numpy as np
import cv2 as cv
import pyzed.sl as sl


class ZEDShared:
    def __init__(self,
                 resolution="HD720",
                 fps=30,
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
        vals = []
        for yy in range(y - radius, y + radius + 1):
            for xx in range(x - radius, x + radius + 1):
                d = self.get_depth_at(xx, yy)
                if d is not None and np.isfinite(d):
                    vals.append(d)

        if not vals:
            return None

        return float(np.median(vals))

    def stop(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=1.0)
        self.zed.close()
