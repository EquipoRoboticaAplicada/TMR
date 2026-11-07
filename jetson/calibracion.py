# calibracion.py
"""
Calibración de colores con la ZED.

Flujo:
  1. CÁMARA: arranca en AUTO. Apunta a la escena real (con la iluminación de la
     competencia), deja que se estabilice y presiona L para FIJAR exposición,
     ganancia y balance de blancos. Puedes afinarlos con las barras.
  2. COLORES: para cada color pon el ROI sobre el objeto y presiona C varias
     veces, idealmente con el objeto cerca, lejos, con sombra y con luz directa.
     A la derecha se ve EN VIVO la máscara que saldría con lo acumulado.
     N acepta el color, B regresa al color anterior.
  3. REVISIÓN: a la izquierda corre el mismo detector que usa main.py.
     G guarda colors.json (colores + ajustes de cámara). ESC sale SIN guardar.
"""

import cv2 as cv
import numpy as np
import json
import os
import time
import tempfile
import textwrap
import unicodedata
from datetime import datetime

from vision_zed import (
    ZEDShared, color_file, preprocess_hsv, build_mask, process_mask,
    detect_colors, DRAW,
)

# ---------------- CONFIG ----------------
COLORS = ["red", "blue", "green"]
WINDOW_NAME = "Color_Calibration_ZED"

BOX_SIZE = 60                 # lado del ROI en pixeles (ajustable con + / -)
BOX_MIN, BOX_MAX = 10, 200
MOVE_STEP = 10                # pixeles por tecla

N_FRAMES = 10                 # frames DISTINTOS por captura

# Rangos = percentiles de lo capturado + margen.
# H angosto; S y V solo con mínimo (el máximo siempre es 255), porque un objeto
# más iluminado o más saturado sigue siendo el mismo objeto.
P_LO, P_HI = 2.0, 98.0
MARGIN_H = 5                  # unidades de hue de OpenCV (0-179)
MARGIN_S = 30
V_MIN_FACTOR = 0.5            # V mínimo = 50% del brillo más bajo capturado. Es relativo
                              # porque una sombra ESCALA el brillo (un margen fijo no alcanza)
S_FLOOR = 50                  # nunca aceptar menos saturación (grises, blancos, brillos)
V_FLOOR = 35                  # nunca aceptar pixeles casi negros (ahí el hue no significa nada)
MAX_H_WIDTH = 50              # advertir si el rango de hue sale más ancho que esto
MIN_VALID_PIXELS = 200
OVERLAP_WARN = 0.02           # advertir si >2% de las muestras de un color caen en otro
MAX_SAMPLES_PER_COLOR = 50000

# Cámara
AUTO_SETTLE_S = 2.0           # tiempo para que AUTO converja antes de fijar
DEFAULT_EXPOSURE = 50
DEFAULT_GAIN = 50
DEFAULT_WB = 4600
WB_MIN, WB_MAX = 2800, 6500

# Configuración de la ZED (tiene que coincidir con la de main.py)
ZED_RESOLUTION = "VGA"        # "VGA" o "HD720"
ZED_FPS = 15
ZED_DEPTH_MODE = "PERFORMANCE"
ZED_MIN_DEPTH = 0.2
ZED_MAX_DEPTH = 20.0
ZED_CONFIDENCE = 50
# ---------------------------------------

_rng = np.random.default_rng(0)


# ======================= Estadística de color =======================

def hue_deviation(h, center):
    """Distancia con signo en el círculo de hue (0-179), en [-90, 90)."""
    return ((np.asarray(h, dtype=np.float64) - center + 90.0) % 180.0) - 90.0


def circular_hue_center(h):
    """
    Centro del hue tratándolo como ángulo. La mediana lineal falla con el rojo:
    pixeles en H≈2 y H≈177 dan una mediana de ~90 (cian).
    """
    ang = np.asarray(h, dtype=np.float64) * (np.pi / 90.0)      # 0-180 -> 0-2π
    mean_ang = np.arctan2(np.sin(ang).mean(), np.cos(ang).mean())
    center = (mean_ang * 90.0 / np.pi) % 180.0
    # Refinar con la mediana de las desviaciones (menos sensible a outliers)
    center = (center + np.median(hue_deviation(h, center))) % 180.0
    return center


def hue_interval_to_ranges(lo, hi, s_min, v_min):
    """Convierte un intervalo de hue (puede salirse de 0-179) en 1 o 2 rangos de inRange."""
    lo = int(np.floor(lo))
    hi = int(np.ceil(hi))
    if hi < 0:
        lo, hi = lo + 180, hi + 180
    elif lo > 179:
        lo, hi = lo - 180, hi - 180

    s_min, v_min = int(s_min), int(v_min)
    if hi - lo >= 179:
        return [{"lower": [0, s_min, v_min], "upper": [179, 255, 255]}]
    if lo < 0:
        return [
            {"lower": [0, s_min, v_min],         "upper": [hi, 255, 255]},
            {"lower": [lo + 180, s_min, v_min],  "upper": [179, 255, 255]},
        ]
    if hi > 179:
        return [
            {"lower": [lo, s_min, v_min],        "upper": [179, 255, 255]},
            {"lower": [0, s_min, v_min],         "upper": [hi - 180, 255, 255]},
        ]
    return [{"lower": [lo, s_min, v_min], "upper": [hi, 255, 255]}]


def compute_hsv_ranges(pixels):
    """
    pixels: Nx3 HSV (uint8). Regresa (ranges, stats, warnings).
    ranges es None si no hay suficientes pixeles con color.
    """
    pixels = np.asarray(pixels)
    h, s, v = pixels[:, 0], pixels[:, 1], pixels[:, 2]
    valid = (s >= S_FLOOR) & (v >= V_FLOOR)
    n_valid = int(valid.sum())
    stats = {"n_pixels": int(len(pixels)), "n_valid": n_valid}
    warnings = []

    if n_valid < MIN_VALID_PIXELS:
        warnings.append(f"Muy pocos pixeles con color ({n_valid}). "
                        "¿El ROI está sobre el objeto? ¿Está muy oscuro o deslavado?")
        return None, stats, warnings

    frac = n_valid / len(pixels)
    if frac < 0.6:
        warnings.append(f"Solo {frac:.0%} del ROI tiene color: puede haber fondo o brillos en el ROI.")

    hv, sv, vv = h[valid], s[valid], v[valid]
    center = circular_hue_center(hv)
    d = hue_deviation(hv, center)
    lo = center + np.percentile(d, P_LO) - MARGIN_H
    hi = center + np.percentile(d, P_HI) + MARGIN_H
    s_min = max(S_FLOOR, np.percentile(sv, P_LO) - MARGIN_S)
    v_min = max(V_FLOOR, np.percentile(vv, P_LO) * V_MIN_FACTOR)

    ranges = hue_interval_to_ranges(lo, hi, s_min, v_min)
    width = float(hi - lo)
    stats.update({
        "center_h": round(float(center), 1),
        "h_width":  round(width, 1),
        "s_min":    int(s_min),
        "v_min":    int(v_min),
        "s_median": int(np.median(sv)),
        "v_median": int(np.median(vv)),
    })
    if width > MAX_H_WIDTH:
        warnings.append(f"Rango de hue muy ancho ({width:.0f}). "
                        "Puede haber fondo en el ROI o el color no es uniforme.")
    return ranges, stats, warnings


def fraction_in_ranges(pixels, ranges):
    """Fracción de pixeles HSV (Nx3) que caen dentro de ranges."""
    if pixels is None or len(pixels) == 0:
        return 0.0
    img = np.ascontiguousarray(pixels.reshape(-1, 1, 3).astype(np.uint8))
    mask = build_mask(img, ranges)
    return float(np.count_nonzero(mask)) / len(pixels)


def subsample(pixels, max_n=MAX_SAMPLES_PER_COLOR):
    if len(pixels) <= max_n:
        return pixels
    idx = _rng.choice(len(pixels), size=max_n, replace=False)
    return pixels[idx]


def overlap_warnings(name, pixels, ranges, calibrated, samples):
    """Compara un color nuevo contra los ya calibrados, en ambos sentidos."""
    out = []
    for other, other_ranges in calibrated.items():
        if other == name:
            continue
        a = fraction_in_ranges(pixels, other_ranges)          # name detectado como other
        b = fraction_in_ranges(samples.get(other), ranges)    # other detectado como name
        if a > OVERLAP_WARN or b > OVERLAP_WARN:
            out.append(f"Se traslapa con {other}: {a:.0%} de {name} cae en {other}, "
                       f"{b:.0%} de {other} cae en {name}.")
    return out


# ======================= Utilidades =======================

def clamp_roi(cx, cy, half, w, h):
    cx = max(half, min(w - half - 1, cx))
    cy = max(half, min(h - half - 1, cy))
    return cx, cy


def atomic_json_dump(obj, final_path):
    """Escritura atómica para evitar JSON incompleto."""
    final_path = str(final_path)
    os.makedirs(os.path.dirname(final_path), exist_ok=True)
    fd, tmp_path = tempfile.mkstemp(dir=os.path.dirname(final_path),
                                    prefix="colors_", suffix=".tmp")
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


def capture_pixels(zed, cx, cy, half, n_frames=N_FRAMES, timeout=5.0):
    """
    Junta los pixeles HSV del ROI en n_frames frames DISTINTOS (antes se podía
    leer el mismo frame varias veces). Usa el mismo preprocesamiento que main.py.
    """
    all_pixels = []
    last_ts = None
    t0 = time.time()
    while len(all_pixels) < n_frames and time.time() - t0 < timeout:
        result = zed.get_frame_copy()
        if result is None:
            time.sleep(0.005)
            continue
        frame, ts = result
        if ts == last_ts:
            time.sleep(0.005)
            continue
        last_ts = ts

        h, w = frame.shape[:2]
        cx2, cy2 = clamp_roi(cx, cy, half, w, h)
        hsv = preprocess_hsv(frame)
        roi = hsv[cy2 - half:cy2 + half, cx2 - half:cx2 + half]
        if roi.size:
            all_pixels.append(roi.reshape(-1, 3))

    if not all_pixels:
        return None
    return np.vstack(all_pixels)


def hud_text(text):
    """OpenCV no dibuja acentos ni ¿/¡: los quita."""
    text = unicodedata.normalize("NFKD", text).encode("ascii", "ignore").decode("ascii")
    return text


def put_text(img, text, org, scale=0.55, color=(255, 255, 255), thickness=1):
    text = hud_text(text)
    cv.putText(img, text, org, cv.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thickness + 2, cv.LINE_AA)
    cv.putText(img, text, org, cv.FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv.LINE_AA)


# ======================= Cámara =======================

def lock_camera_from_auto(zed):
    """Deja que AUTO converja, lee lo que eligió y lo fija en manual."""
    print(f"Cámara en AUTO, esperando {AUTO_SETTLE_S:.0f} s a que se estabilice...")
    zed.set_auto_camera()
    time.sleep(AUTO_SETTLE_S)
    cur = zed.get_camera_settings()
    print(f"Valores que eligió AUTO: {cur}")

    def pick(key, default):
        val = cur.get(key)
        if val is None or val < 0:
            print(f"  No se pudo leer {key} en AUTO, uso {default}. Ajústalo con la barra.")
            return default
        return val

    camera = {
        "exposure": pick("exposure", DEFAULT_EXPOSURE),
        "gain": pick("gain", DEFAULT_GAIN),
        "whitebalance_temperature": pick("whitebalance_temperature", DEFAULT_WB),
    }
    camera.update(zed.set_manual_camera(**camera))
    print(f"Cámara FIJADA: {camera}")
    print(f"Leído de vuelta: {zed.get_camera_settings()}")
    return camera


def create_trackbars():
    noop = lambda _v: None
    cv.createTrackbar("Exposure", WINDOW_NAME, DEFAULT_EXPOSURE, 100, noop)
    cv.createTrackbar("Gain", WINDOW_NAME, DEFAULT_GAIN, 100, noop)
    cv.createTrackbar("WB x100K", WINDOW_NAME, DEFAULT_WB // 100, WB_MAX // 100, noop)
    try:
        cv.setTrackbarMin("WB x100K", WINDOW_NAME, WB_MIN // 100)
    except Exception:
        pass


def set_trackbars(camera):
    cv.setTrackbarPos("Exposure", WINDOW_NAME, int(camera["exposure"]))
    cv.setTrackbarPos("Gain", WINDOW_NAME, int(camera["gain"]))
    cv.setTrackbarPos("WB x100K", WINDOW_NAME, int(camera["whitebalance_temperature"]) // 100)


def read_trackbars():
    exp = cv.getTrackbarPos("Exposure", WINDOW_NAME)
    gain = cv.getTrackbarPos("Gain", WINDOW_NAME)
    wb = max(WB_MIN, cv.getTrackbarPos("WB x100K", WINDOW_NAME) * 100)
    return {"exposure": exp, "gain": gain, "whitebalance_temperature": wb}


# ======================= Paneles =======================

def mask_panel(frame, hsv, ranges):
    """Muestra el frame original solo donde la máscara (ya procesada) es 1."""
    mask = process_mask(build_mask(hsv, ranges))
    panel = (frame * 0.15).astype(np.uint8)
    panel[mask > 0] = frame[mask > 0]
    return panel


def review_panel(frame, hsv, calibrated):
    panel = (frame * 0.15).astype(np.uint8)
    for name, ranges in calibrated.items():
        mask = process_mask(build_mask(hsv, ranges))
        panel[mask > 0] = DRAW.get(name, (255, 255, 255))
    return panel


# ======================= Main =======================

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
        zed.set_auto_camera()     # por si la cámara quedó en manual de una sesión anterior

        cv.namedWindow(WINDOW_NAME, cv.WINDOW_NORMAL)
        create_trackbars()

        frame = get_latest_frame(zed, timeout=5.0)
        if frame is None:
            print("No se pudo leer frame inicial desde la ZED")
            return

        h, w = frame.shape[:2]
        box = BOX_SIZE
        cx, cy = w // 2, h // 2

        state = "camera"            # camera -> colors -> review
        camera = None               # ajustes fijos de la cámara
        applied_tb = None           # últimos valores aplicados desde las barras
        color_index = 0
        calibrated = {}             # color -> rangos
        samples = {}                # color -> pixeles (para revisar traslapes)
        stats_all = {}              # color -> estadísticas
        accum_pixels = []
        preview = None              # (ranges, stats, warnings) del color actual
        message = "Apunta a la escena real y presiona L para fijar la camara"

        def clear_colors(reason):
            nonlocal calibrated, samples, stats_all, accum_pixels, preview, color_index, state, message
            if calibrated or accum_pixels:
                print(f"⚠️ {reason}: se borran los colores capturados (ya no corresponden a la imagen).")
                message = f"{reason}: recalibra los colores"
            calibrated, samples, stats_all = {}, {}, {}
            accum_pixels, preview = [], None
            color_index = 0
            if state == "review":
                state = "colors"

        while True:
            frame = get_latest_frame(zed, timeout=1.0)
            if frame is None:
                continue
            h, w = frame.shape[:2]
            half = box // 2
            cx, cy = clamp_roi(cx, cy, half, w, h)
            hsv = preprocess_hsv(frame)

            # --- Barras de cámara (solo cuando ya está fijada) ---
            if camera is not None:
                tb = read_trackbars()
                if tb != applied_tb:
                    camera.update(zed.set_manual_camera(**tb))
                    applied_tb = tb
                    clear_colors("Cambio de ajustes de camara")

            # --- Paneles ---
            if state == "review":
                left = frame.copy()
                detect_colors(left, calibrated, draw=True)
                right = review_panel(frame, hsv, calibrated)
                lines = ["REVISION (mismo detector que main.py)",
                         "G guardar y salir | R recalibrar colores | ESC salir sin guardar"]
            else:
                left = frame.copy()
                cv.rectangle(left, (cx - half, cy - half), (cx + half, cy + half), (0, 255, 0), 2)
                if state == "camera":
                    right = (frame * 0.15).astype(np.uint8)
                    lines = ["CAMARA EN AUTO",
                             "L fijar camara | WASD/flechas mover ROI | ESC salir"]
                else:
                    color_name = COLORS[color_index]
                    if preview is not None and preview[0] is not None:
                        right = mask_panel(frame, hsv, preview[0])
                    else:
                        right = (frame * 0.15).astype(np.uint8)
                    lines = [f"Color: {color_name} ({color_index + 1}/{len(COLORS)}) | capturas: {len(accum_pixels)}",
                             "C capturar | N aceptar | B anterior | R reset | +/- ROI | L refijar camara"]
                    if preview is not None and preview[0] is not None:
                        st = preview[1]
                        lines.append(f"H~{st['center_h']} ancho {st['h_width']} | "
                                     f"Smin {st['s_min']} Vmin {st['v_min']}")
                    if preview is not None:
                        lines += preview[2][:2]

            if camera is not None:
                lines.append(f"Cam: exp {camera.get('exposure')} gain {camera.get('gain')} "
                             f"wb {camera.get('whitebalance_temperature')}")
            if message:
                lines.append(message)

            y = 22
            for i, line in enumerate(lines):
                color = (0, 200, 255) if i >= 2 else (255, 255, 255)
                for chunk in textwrap.wrap(hud_text(line), 70) or [""]:
                    put_text(left, chunk, (10, y), 0.5, color)
                    y += 20

            cv.imshow(WINDOW_NAME, np.hstack([left, right]))
            key = cv.waitKey(1) & 0xFF
            if key == 255:
                continue

            # --- Teclas globales ---
            if key == 27:
                print("Salida SIN guardar.")
                break

            if key == ord("l") and state != "review":
                camera = lock_camera_from_auto(zed)
                set_trackbars(camera)
                applied_tb = read_trackbars()
                clear_colors("Camara refijada")
                if state == "camera":
                    state = "colors"
                message = "Camara fijada. Captura el primer color."
                continue

            # --- Revisión ---
            if state == "review":
                if key == ord("g"):
                    out = {
                        "version": 2,
                        "camera": camera,
                        "colors": calibrated,
                        "meta": {
                            "created": datetime.now().isoformat(timespec="seconds"),
                            "resolution": ZED_RESOLUTION,
                            "fps": ZED_FPS,
                            "params": {
                                "P_LO": P_LO, "P_HI": P_HI, "MARGIN_H": MARGIN_H,
                                "MARGIN_S": MARGIN_S, "V_MIN_FACTOR": V_MIN_FACTOR,
                                "S_FLOOR": S_FLOOR, "V_FLOOR": V_FLOOR,
                            },
                            "stats": stats_all,
                        },
                    }
                    atomic_json_dump(out, color_file)
                    print("Calibración guardada en:", color_file)
                    break
                if key == ord("r"):
                    clear_colors("Recalibracion")
                    state = "colors"
                    message = ""
                continue

            # --- Mover / redimensionar ROI ---
            if key in (ord("a"), 81):
                cx -= MOVE_STEP
            elif key in (ord("d"), 83):
                cx += MOVE_STEP
            elif key in (ord("w"), 82):
                cy -= MOVE_STEP
            elif key in (ord("s"), 84):
                cy += MOVE_STEP
            elif key in (ord("+"), ord("=")):
                box = min(BOX_MAX, box + 10)
            elif key == ord("-"):
                box = max(BOX_MIN, box - 10)

            if state == "camera" and key == ord("c"):
                message = "Primero fija la camara con L"
            if state != "colors":
                continue
            color_name = COLORS[color_index]

            if key == ord("c"):
                pixels = capture_pixels(zed, cx, cy, half)
                if pixels is None:
                    message = "No se pudieron capturar pixeles (reintenta)"
                    continue
                accum_pixels.append(pixels)
                all_px = np.vstack(accum_pixels)
                ranges, stats, warns = compute_hsv_ranges(all_px)
                if ranges is not None:
                    warns = warns + overlap_warnings(color_name, subsample(all_px), ranges,
                                                     calibrated, samples)
                preview = (ranges, stats, warns)
                message = ""
                print(f"[{color_name}] captura {len(accum_pixels)}: {stats}")
                for wtxt in warns:
                    print("   ⚠️", wtxt)

            elif key == ord("r"):
                accum_pixels, preview = [], None
                message = f"Reset de capturas de {color_name}"

            elif key == ord("b"):
                if color_index > 0:
                    color_index -= 1
                    prev = COLORS[color_index]
                    calibrated.pop(prev, None)
                    samples.pop(prev, None)
                    stats_all.pop(prev, None)
                    accum_pixels, preview = [], None
                    message = f"De regreso a {prev}"

            elif key == ord("n"):
                if preview is None or preview[0] is None:
                    message = f"Sin rango valido para {color_name}: captura con C"
                    continue
                ranges, stats, warns = preview
                calibrated[color_name] = ranges
                samples[color_name] = subsample(np.vstack(accum_pixels))
                stats_all[color_name] = stats
                print(f"✅ {color_name}: {ranges}")
                accum_pixels, preview = [], None
                color_index += 1
                message = ""
                if color_index >= len(COLORS):
                    state = "review"

    finally:
        if zed is not None:
            zed.stop()
        cv.destroyAllWindows()


if __name__ == "__main__":
    main()