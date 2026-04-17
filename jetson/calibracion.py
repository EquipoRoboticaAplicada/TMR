import cv2 as cv
import numpy as np
import json
import os
import time
import tempfile

from vision_zed import ZEDShared

# ---------------- CONFIG ----------------
BOX_SIZE = 80

TOL_H = 40
TOL_S = 40
TOL_V = 40

COLORS = ["red", "blue", "green"]
WINDOW_NAME = "Color_Calibration_ZED"

N_SAMPLES = 15          # frames por captura
SAMPLE_DELAY = 0.01     # pausa pequeña entre frames (s)

MOVE_STEP = 10          # pixeles por tecla

# Configuración de la ZED
ZED_RESOLUTION = "VGA"          # "VGA" o "HD720"
ZED_FPS = 15
ZED_DEPTH_MODE = "PERFORMANCE"
ZED_MIN_DEPTH = 0.2
ZED_MAX_DEPTH = 20.0
ZED_CONFIDENCE = 50
# ---------------------------------------


def clamp_roi(cx, cy, half, w, h):
    cx = max(half, min(w - half - 1, cx))
    cy = max(half, min(h - half - 1, cy))
    return cx, cy


def hsv_bounds_with_wrap(h, s, v, tol_h, tol_s, tol_v):
    """Devuelve una lista de 1 o 2 rangos HSV (por wrap-around en H)."""
    s_low = int(max(0, s - tol_s))
    v_low = int(max(0, v - tol_v))
    s_up  = int(min(255, s + tol_s))
    v_up  = int(min(255, v + tol_v))

    h_low = int(h - tol_h)
    h_up  = int(h + tol_h)

    if h_low < 0:
        return [
            {"lower": [0, s_low, v_low], "upper": [int(h_up), s_up, v_up]},
            {"lower": [int(179 + h_low), s_low, v_low], "upper": [179, s_up, v_up]},
        ]
    elif h_up > 179:
        return [
            {"lower": [0, s_low, v_low], "upper": [int(h_up - 179), s_up, v_up]},
            {"lower": [int(h_low), s_low, v_low], "upper": [179, s_up, v_up]},
        ]
    else:
        return [
            {"lower": [int(h_low), s_low, v_low], "upper": [int(h_up), s_up, v_up]}
        ]


def atomic_json_dump(obj, final_path):
    """Escritura atómica para evitar JSON incompleto."""
    os.makedirs(os.path.dirname(final_path), exist_ok=True)
    fd, tmp_path = tempfile.mkstemp(
        dir=os.path.dirname(final_path),
        prefix="colors_",
        suffix=".tmp"
    )
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            json.dump(obj, f, indent=4)
        os.replace(tmp_path, final_path)
    finally:
        if os.path.exists(tmp_path):
            try:
                os.remove(tmp_path)
            except Exception:
                pass


def get_latest_frame(zed, timeout=2.0):
    """Espera hasta obtener un frame válido de la ZED."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        result = zed.get_frame_copy()
        if result is not None:
            frame, ts = result
            if frame is not None and frame.size > 0:
                return frame
        time.sleep(0.01)
    return None


def capture_pixels(zed, cx, cy, half, n_samples, sample_delay):
    """
    Captura N frames del ROI desde la ZED y devuelve TODOS los pixeles HSV
    para usar una mediana robusta.
    """
    all_pixels = []

    for _ in range(n_samples):
        frame = get_latest_frame(zed, timeout=1.0)
        if frame is None:
            continue

        h, w = frame.shape[:2]
        cx2, cy2 = clamp_roi(cx, cy, half, w, h)

        roi = frame[cy2 - half:cy2 + half, cx2 - half:cx2 + half]
        if roi.size == 0:
            continue

        roi_blur = cv.GaussianBlur(roi, (5, 5), 0)
        hsv = cv.cvtColor(roi_blur, cv.COLOR_BGR2HSV)

        pixels = hsv.reshape(-1, 3)
        all_pixels.append(pixels)

        if sample_delay > 0:
            time.sleep(sample_delay)

    if not all_pixels:
        return None

    return np.vstack(all_pixels)


def main():
    zed = None
    try:
        zed = ZEDShared(
            resolution=ZED_RESOLUTION,
            fps=ZED_FPS,
            depth_mode=ZED_DEPTH_MODE,
            min_depth=ZED_MIN_DEPTH,
            max_depth=ZED_MAX_DEPTH,
            confidence_threshold=ZED_CONFIDENCE
        ).start()

        cv.namedWindow(WINDOW_NAME, cv.WINDOW_NORMAL)

        # Ruta destino
        base_dir = os.path.dirname(os.path.abspath(__file__))
        config_dir = os.path.join(base_dir, "config")
        file_path = os.path.join(config_dir, "colors.json")

        calibrated = {}
        color_index = 0

        frame = get_latest_frame(zed, timeout=5.0)
        if frame is None:
            print("No se pudo leer frame inicial desde la ZED")
            return

        h, w = frame.shape[:2]
        half = BOX_SIZE // 2
        cx, cy = w // 2, h // 2
        cx, cy = clamp_roi(cx, cy, half, w, h)

        # Acumulador de pixeles por color
        accum_pixels = []

        while color_index < len(COLORS):
            color_name = COLORS[color_index]

            frame = get_latest_frame(zed, timeout=1.0)
            if frame is None:
                continue

            h, w = frame.shape[:2]
            half = BOX_SIZE // 2
            cx, cy = clamp_roi(cx, cy, half, w, h)

            # Dibujo del ROI
            vis = frame.copy()
            cv.rectangle(vis, (cx - half, cy - half), (cx + half, cy + half), (0, 255, 0), 2)
            cv.putText(
                vis,
                f"Color: {color_name} | Capturas acumuladas: {len(accum_pixels)}",
                (10, 30),
                cv.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2
            )
            cv.putText(
                vis,
                "Flechas/WASD mover | c capturar | n siguiente | r reset | ESC salir",
                (10, 60),
                cv.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2
            )

            cv.imshow(WINDOW_NAME, vis)
            key = cv.waitKey(1) & 0xFF

            # Movimiento (WASD)
            if key == ord("a"):
                cx -= MOVE_STEP
            elif key == ord("d"):
                cx += MOVE_STEP
            elif key == ord("w"):
                cy -= MOVE_STEP
            elif key == ord("s"):
                cy += MOVE_STEP

            # Flechas
            elif key in (81,):
                cx -= MOVE_STEP
            elif key in (83,):
                cx += MOVE_STEP
            elif key in (82,):
                cy -= MOVE_STEP
            elif key in (84,):
                cy += MOVE_STEP

            # Capturar y acumular
            elif key == ord("c"):
                pixels = capture_pixels(zed, cx, cy, half, N_SAMPLES, SAMPLE_DELAY)
                if pixels is None:
                    print("No se pudo capturar pixeles desde la ZED (reintenta).")
                else:
                    accum_pixels.append(pixels)
                    print(f"Captura añadida para {color_name}. Total: {len(accum_pixels)}")

            # Reset del color actual
            elif key == ord("r"):
                accum_pixels = []
                print(f"Reset de capturas para {color_name}")

            # Finalizar color y pasar al siguiente
            elif key == ord("n"):
                if not accum_pixels:
                    print(f"No hay capturas para {color_name}. Presiona 'c' primero.")
                    continue

                pixels_all = np.vstack(accum_pixels)
                median_hsv = np.median(pixels_all, axis=0).astype(int)
                hh, ss, vv = int(median_hsv[0]), int(median_hsv[1]), int(median_hsv[2])

                ranges_list = hsv_bounds_with_wrap(hh, ss, vv, TOL_H, TOL_S, TOL_V)

                if len(ranges_list) == 1:
                    calibrated[color_name] = ranges_list[0]
                else:
                    calibrated[color_name] = ranges_list

                print(f"{color_name} calibrado. Mediana HSV: {median_hsv}")
                print(f"Rangos guardados: {calibrated[color_name]}")

                accum_pixels = []
                color_index += 1

            elif key == 27:
                break

        atomic_json_dump(calibrated, file_path)
        print("Calibración completada")
        print("Guardado en:", file_path)

    finally:
        if zed is not None:
            zed.stop()
        cv.destroyAllWindows()


if __name__ == "__main__":
    main()