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

PI_IP = "192.168.1.83"

VIDEO_URL  = f"http://{PI_IP}:5000/video_feed"
CMD_URL    = f"http://{PI_IP}:5000/command"
VISION_URL = f"http://{PI_IP}:5000/vision_data"

DETECTION_FRAMES = 15


def send_rpms(left_rpm, right_rpm, dir_left, dir_right):   
    speed = {
        "left_rpm": f"S{left_rpm}",
        "right_rpm": f"S{right_rpm}",
        "left_dir": f"D{dir_left}",
        "right_dir": f"D{dir_right}"
    }

    try:
        requests.post(
            CMD_URL,
            json=speed,
            timeout=0.2
        )
    except Exception as e:
        print("Error enviando rpms:", e)


def send_vision_data(colors, centroids, areas):
    payload = {
        "colors": colors,
        "centroids": centroids,
        "areas": areas,
        "time": time.time()
    }

    try:
        requests.post(
            VISION_URL,
            json=payload,
            timeout=0.1
        )
    except Exception as e:
        print("Error enviando vision_data:", e)


def calc_turn_x(centroid, frame_width, deadband_px=50):
    cx = centroid[1]
    center_x = frame_width / 2
    error_px = cx - center_x

    if abs(error_px) < deadband_px:
        return 0.0

    error = error_px / (frame_width / 2)   # [-1..1]
    Kp = 0.8                                # ahora es “estable” por tamaño
    turn = Kp * error

    turn = max(-1.0, min(1.0, turn))       # turn normalizado
    return turn


def calc_base_y(centroid, frame_height, y_trigger_ratio=0.40):
    cy = centroid[2]
    y_trigger = frame_height * y_trigger_ratio

    # Si el objeto aún está “arriba” (lejos), avanza
    if cy < y_trigger:
        return 1.0   # avanzar (normalizado)
    else:
        return 0.0   # ya llegó -> no avanzar


def pick_target(centroids, areas, area_min=1500):
    if not centroids or not areas:
        return None  # no hay nada

    # Filtra por área mínima
    candidates = [(c, a) for c, a in zip(centroids, areas) if a >= area_min]
    if not candidates:
        return None

    # Escoge el de mayor área
    centroid, area = max(candidates, key=lambda x: x[1])
    return centroid, area


def main():
    cap = cv.VideoCapture(VIDEO_URL)

    if not cap.isOpened():
        print("No se pudo abrir el stream")
        return

    print("Stream conectado")

    # --- Parámetros de estabilidad ---
    N_ENTER = 5     # frames consecutivos para entrar a tracking (rápido)
    N_EXIT  = 10    # frames consecutivos sin target para salir (más lento)
    AREA_MIN = 1500 # ajusta según tu escena (filtro anti-ruido)

    seen_count = 0
    lost_count = 0
    tracking = False

    SEND_HZ = 15
    send_period = 1.0 / SEND_HZ
    last_send = 0.0

    APPROACH_RPM = 20  # velocidad al acercarse
    CRUISE_RPM = 30    # velocidad default cuando no hay tracking
    left_rpm, right_rpm = CRUISE_RPM, CRUISE_RPM  # velocidad default
    dir_left, dir_right = 1, 1     # dirección default (1=forward, 0=backward)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Frame no recibido")
            break

        # Detección
        colors, centroids, areas = detect_colors(frame, draw=True)

        # Decide si hay un target "confiable"
        target = pick_target(centroids, areas, area_min=AREA_MIN)
        detected = target is not None

        if detected:
            centroid = target[0]

        # --- Lógica de contadores ---
        if detected:
            seen_count += 1
            lost_count = 0
        else:
            lost_count += 1
            seen_count = 0

        # --- Transiciones de modo (tracking ON/OFF) ---
        if (not tracking) and (seen_count >= N_ENTER):
            tracking = True
            print("Tracking ACTIVADO (detección estable)\n")
            # print("Colores:", colors) 
            # print("Centroides:", centroids) 
            # print("Áreas:", areas)
            # Aquí puedes activar “prioridad visión”    


        if tracking and (lost_count >= N_EXIT):
            tracking = False
            print("Tracking DESACTIVADO (detección perdida)")
            # Aquí vuelves al modo default

        if not tracking:
            left_rpm, right_rpm = CRUISE_RPM, CRUISE_RPM
            dir_left, dir_right = 1, 1
            
        # --- Si hay tracking, manda control por visión ---
        if tracking and detected:
            centroid, area = target
            h, w = frame.shape[0], frame.shape[1]

            base = APPROACH_RPM * calc_base_y(centroid, h)   # 0 o APPROACH_RPM
            turn = calc_turn_x(centroid, w)                  # [-1..1]

            if base == 0:
                # "Ya llegué" en Y → alinear sin RPM negativas
                rot = APPROACH_RPM * abs(turn)
                if turn > 0:     # objeto a la derecha → gira a la derecha (ajusta si queda al revés)
                    left_rpm, right_rpm = rot, rot
                    dir_left, dir_right = 1, 0
                else:            # objeto a la izquierda
                    left_rpm, right_rpm = rot, rot
                    dir_left, dir_right = 0, 1
            else:
                dir_left, dir_right = 1, 1  # reset dirección después de enviar
            # else:
            #     # Avanza y corrige
            #     left_rpm  = base - APPROACH_RPM * turn
            #     right_rpm = base + APPROACH_RPM * turn


        # --- Envío de comandos ---
        now = time.time()
        if now - last_send >= send_period:
            send_rpms(left_rpm, right_rpm, dir_left, dir_right)
            last_send = now
        # dir_left, dir_right = 1, 1  # reset dirección después de enviar

        # --- Cálculo de comandos ---
        send_vision_data(colors, centroids, areas)

        # Visualización
        video = crosslines(frame.copy())
        cv.imshow("VISION ROVER (PC DEBUG)", video)

        if cv.waitKey(1) & 0xFF == 27:
            break

    cap.release()
    cv.destroyAllWindows()


if __name__ == "__main__":
    main()
