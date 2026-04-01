# vision_zed.py
import threading
import time
import cv2 as cv

from vision import load_color_ranges, detect_colors, pick_target


class VisionZED:
    def __init__(self, zed_shared, color_file="colors.json", area_min=500, draw_local=False):
        self.zed = zed_shared
        self.color_ranges = load_color_ranges(color_file)
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
            debug_frame = frame.copy()

            colors, centroids, areas = detect_colors(
                debug_frame if self.draw_local else frame.copy(),
                self.color_ranges,
                draw=self.draw_local
            )
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