# debug_local.py
# Ejecutar directamente en la Jetson para visualizar la cámara ZED
# y el estado de detección de VisionZED en tiempo real.
# No interfiere con el sistema principal (solo lectura).
#
# Uso:
#   python debug_local.py
#
# Teclas:
#   ESC / Q  — cerrar
#   R        — imprimir pose en consola (si se pasa odo como argumento)

import cv2
import time
from vision_zed import VisionZED, ZEDShared


def run_debug(zed: ZEDShared, vision: VisionZED, odo=None):
    """
    Muestra una ventana OpenCV con:
      - Frame en vivo de la ZED
      - Bounding box y etiqueta del objeto detectado
      - Distancia al objeto
      - Crosslines centrales
      - HUD con estado de detección, distancia y pose (si odo disponible)

    zed    : instancia de ZEDShared ya iniciada
    vision : instancia de VisionZED ya iniciada
    odo    : instancia de RoverOdometry (opcional) para mostrar pose
    """

    FONT       = cv2.FONT_HERSHEY_SIMPLEX
    COLOR_INFO = (0, 255, 255)   # cyan  — textos de detección
    COLOR_HUD  = (200, 200, 200) # gris  — HUD general
    COLOR_CROSS= (80, 80, 80)    # gris oscuro — crosslines
    COLOR_BOX  = (0, 255, 0)     # verde — bounding box

    print("Debug local iniciado. Presiona ESC o Q para salir.")

    while True:
        # ── Frame ──────────────────────────────────────────────────────
        result = zed.get_frame_copy()
        if result is None:
            time.sleep(0.01)
            continue

        frame, ts = result
        h, w = frame.shape[:2]

        # ── Crosslines ─────────────────────────────────────────────────
        cx_center = w // 2
        cy_center = h // 2
        cv2.line(frame, (cx_center, 0), (cx_center, h), COLOR_CROSS, 1)
        cv2.line(frame, (0, cy_center), (w, cy_center), COLOR_CROSS, 1)

        # ── Estado de visión ───────────────────────────────────────────
        state    = vision.get_state()
        detected = state.get("detected", False)
        cx       = state.get("cx")
        cy       = state.get("cy")
        area     = state.get("area")
        dist     = state.get("distance_m")
        label    = state.get("label", "")

        if detected and cx is not None and cy is not None:
            # Punto central del objeto
            cv2.circle(frame, (cx, cy), 6, COLOR_BOX, -1)

            # Línea desde el centro de la imagen al objeto
            cv2.line(frame, (cx_center, cy_center), (cx, cy), COLOR_BOX, 1)

            # Textos junto al centroide
            dist_txt = f"{dist:.2f} m" if dist is not None else "dist: N/A"
            cv2.putText(frame, f"{label}",        (cx + 10, cy - 10), FONT, 0.6, COLOR_BOX,  2)
            cv2.putText(frame, dist_txt,           (cx + 10, cy + 15), FONT, 0.6, COLOR_INFO, 2)
            cv2.putText(frame, f"area: {int(area)}", (cx + 10, cy + 38), FONT, 0.5, COLOR_INFO, 1)

        # ── HUD superior izquierdo ──────────────────────────────────────
        det_txt   = f"Detectado: {'SI' if detected else 'NO'}"
        det_color = (0, 255, 0) if detected else (0, 0, 255)
        cv2.putText(frame, det_txt, (10, 24), FONT, 0.6, det_color, 2)

        if odo is not None:
            x, y, theta = odo.pose
            v, omega    = odo.velocity
            import math
            hud_lines = [
                f"x: {x:+.3f} m",
                f"y: {y:+.3f} m",
                f"theta: {math.degrees(theta):+.1f} deg",
                f"v: {v:+.4f} m/s",
                f"w: {omega:+.4f} rad/s",
            ]
            for i, line in enumerate(hud_lines):
                cv2.putText(frame, line, (10, 50 + i * 22), FONT, 0.55, COLOR_HUD, 1)

        # ── Mostrar ─────────────────────────────────────────────────────
        cv2.imshow("DEBUG ZED — Jetson", frame)

        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord('q')):   # ESC o Q
            break
        if key == ord('r') and odo is not None:
            import math
            x, y, theta = odo.pose
            print(f"[pose] x={x:.3f}  y={y:.3f}  theta={math.degrees(theta):.1f}°")

    cv2.destroyAllWindows()
    print("Debug local cerrado.")


# ────────────────────────────────────────────────────────────────────
# Entrada standalone: levanta la ZED y VisionZED por su cuenta,
# sin necesitar que main.py esté corriendo.
# ────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    zed = ZEDShared(
        resolution="VGA",
        fps=15,
        depth_mode="PERFORMANCE",
        min_depth=0.2,
        max_depth=20.0,
        confidence_threshold=50
    ).start()

    vision = VisionZED(zed_shared=zed, area_min=500, draw_local=False).start()

    try:
        run_debug(zed, vision, odo=None)
    finally:
        vision.stop()
        zed.stop()