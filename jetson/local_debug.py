
import cv2
import time
import math
from vision_zed import VisionZED, ZEDShared


def run_debug(zed: ZEDShared, vision: VisionZED, odo=None):

    FONT        = cv2.FONT_HERSHEY_SIMPLEX
    COLOR_INFO  = (0, 255, 255)
    COLOR_HUD   = (200, 200, 200)
    COLOR_CROSS = (80, 80, 80)
    COLOR_BOX   = (0, 255, 0)
    WARMUP_TIMEOUT = 10.0   # segundos máximo esperando el primer frame

    # ── Verificar backend de display ───────────────────────────────────
    backend = cv2.getBuildInformation()
    print("[debug] Verificando entorno gráfico...")

    # ── Esperar primer frame antes de abrir ventana ─────────────────────
    print("[debug] Esperando primer frame de la ZED...")
    t0 = time.time()
    while True:
        result = zed.get_frame_copy()
        if result is not None:
            print("[debug] Primer frame recibido. Abriendo ventana...")
            break
        if time.time() - t0 > WARMUP_TIMEOUT:
            print("[debug] ⚠ Timeout esperando frame de la ZED. Abortando debug.")
            return
        time.sleep(0.05)

    # ── Crear ventana explícitamente antes del loop ─────────────────────
    cv2.namedWindow("DEBUG ZED", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("DEBUG ZED", 672, 376)

    print("[debug] Ventana abierta. Presiona ESC o Q para salir.")

    while True:
        result = zed.get_frame_copy()
        if result is None:
            # Si perdemos frames, mostramos el último en lugar de negro
            time.sleep(0.01)
            cv2.waitKey(1)
            continue

        frame, ts = result

        # ── Verificar que el frame no esté vacío ───────────────────────
        if frame is None or frame.size == 0:
            time.sleep(0.01)
            continue

        h, w = frame.shape[:2]

        # ── Crosslines ─────────────────────────────────────────────────
        cx_c = w // 2
        cy_c = h // 2
        cv2.line(frame, (cx_c, 0),  (cx_c, h), COLOR_CROSS, 1)
        cv2.line(frame, (0, cy_c),  (w, cy_c), COLOR_CROSS, 1)

        # ── Estado de visión ───────────────────────────────────────────
        state    = vision.get_state()
        detected = state.get("detected", False)
        cx       = state.get("cx")
        cy       = state.get("cy")
        area     = state.get("area")
        dist     = state.get("distance_m")
        label    = state.get("label", "")

        if detected and cx is not None and cy is not None:
            cv2.circle(frame, (cx, cy), 6, COLOR_BOX, -1)
            cv2.line(frame, (cx_c, cy_c), (cx, cy), COLOR_BOX, 1)
            dist_txt = f"{dist:.2f} m" if dist is not None else "dist: N/A"
            cv2.putText(frame, label,               (cx+10, cy-10), FONT, 0.6, COLOR_BOX,  2)
            cv2.putText(frame, dist_txt,             (cx+10, cy+15), FONT, 0.6, COLOR_INFO, 2)
            cv2.putText(frame, f"area: {int(area)}", (cx+10, cy+38), FONT, 0.5, COLOR_INFO, 1)

        # ── HUD ────────────────────────────────────────────────────────
        det_txt   = f"Detectado: {'SI' if detected else 'NO'}"
        det_color = (0, 255, 0) if detected else (0, 0, 255)
        cv2.putText(frame, det_txt, (10, 24), FONT, 0.6, det_color, 2)

        if odo is not None:
            x, y, theta = odo.pose
            v, omega    = odo.velocity
            for i, line in enumerate([
                f"x: {x:+.3f} m",
                f"y: {y:+.3f} m",
                f"theta: {math.degrees(theta):+.1f} deg",
                f"v: {v:+.4f} m/s",
                f"w: {omega:+.4f} rad/s",
            ]):
                cv2.putText(frame, line, (10, 50 + i*22), FONT, 0.55, COLOR_HUD, 1)

        # ── Mostrar ────────────────────────────────────────────────────
        cv2.imshow("DEBUG ZED", frame)

        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord('q')):
            break
        if key == ord('r') and odo is not None:
            x, y, theta = odo.pose
            print(f"[pose] x={x:.3f}  y={y:.3f}  theta={math.degrees(theta):.1f}°")

    cv2.destroyAllWindows()
    print("[debug] Ventana cerrada.")


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