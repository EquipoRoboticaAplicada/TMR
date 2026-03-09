#
# Este código necesita también del código: vision.py
#
# La función es recibir el video de la raspberry y procesarlo con vision, escribir el color que se encontró en la terminal al igual que su centroide y área.
#
# Aquí se incluyen funciones para debuggear, transmite a la página y se abre el cuadro de video con bounding boxes y centroides.

from vision import detect_colors, crosslines
from  util import FrameGrabber, Sender, pick_target, clamp_rpm, send_rpms, calc_turn_x


import cv2 as cv
import time


PI_IP = "172.32.236.53"

VIDEO_URL  = f"http://{PI_IP}:5000/video_feed"
CMD_URL    = f"http://{PI_IP}:5000/command"
VISION_URL = f"http://{PI_IP}:5000/vision_data"

def main():
    # Inicia el hilo captura
    grabber = FrameGrabber(VIDEO_URL).start()

    # Inicia el hilo envío 
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

    # NUEVO: Variables para simular el avance de los encoders
    ticks_left = 0
    ticks_right = 0

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

        # NUEVO: Simulación de encoders sumando o restando RPMs según la dirección
        if dir_left == 1:
            ticks_left += left_rpm_c
        else:
            ticks_left -= left_rpm_c
            
        if dir_right == 1:
            ticks_right += right_rpm_c
        else:
            ticks_right -= right_rpm_c

        command_send_counter += 1
        if command_send_counter >= COMMAND_SEND_EVERY:
            # Mandamos a llamar la función incluyendo los ticks simulados
            send_rpms(sender, left_rpm_c, right_rpm_c, dir_left, dir_right, ticks_left, ticks_right)
            command_send_counter = 0

        # vision_send_counter += 1
        # if vision_send_counter >= VISION_SEND_EVERY:
        #     send_vision_data(sender, colors, centroids, areas)
        #     vision_send_counter = 0

        video = crosslines(frame.copy())
        cv.imshow("VISION ROVER (PC DEBUG)", video)

        if cv.waitKey(1) & 0xFF == 27:
            break

    sender.stop()
    grabber.stop()
    cv.destroyAllWindows()


if __name__ == "__main__":
    main()   
    