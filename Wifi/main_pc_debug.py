#
# Este código necesita también del código: vision.py
#
# La función es recibir el video de la raspberry y procesarlo con vision, escribir el color que se encontró en la terminal al igual que su centroide y área.
#
# Aquí se incluyen funciones para debuggear, transmite a la página y se abre el cuadro de video con bounding boxes y centroides.

from vision import detect_colors
import cv2 as cv
import requests
import time

PI_IP = "172.32.192.88"

VIDEO_URL  = f"http://{PI_IP}:5000/video_feed"
CMD_URL    = f"http://{PI_IP}:5000/command"
VISION_URL = f"http://{PI_IP}:5000/vision_data"

DETECTION_FRAMES = 15


def send_command(cmd):
    try:
        requests.post(
            CMD_URL,
            json={"cmd": cmd},
            timeout=0.2
        )
    except:
        pass


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
    except:
        pass


def main():
    cap = cv.VideoCapture(VIDEO_URL)

    if not cap.isOpened():
        print("No se pudo abrir el stream")
        return

    print("Stream conectado")
    send_command("signal_on")

    counter = 0
    confirmed = False

    while True:
        ret, frame = cap.read()
        if not ret:
            print("rame no recibido")
            break

        colors, centroids, areas = detect_colors(frame, draw=True)

        if colors:
            counter += 1
        else:
            counter = 0
            if confirmed:
                print("Detección perdida")
                confirmed = False
                send_command("signal_on")

        if counter >= DETECTION_FRAMES and not confirmed:
            print("Detección confirmada")
            print()
            print("Colores:", colors)
            print("Centroides:", centroids)
            print("Áreas:", areas)

            send_vision_data(colors, centroids, areas)
            send_command("signal_off")
            confirmed = True

        cv.imshow("VISION ROVER (PC DEBUG)", frame)

        if cv.waitKey(1) & 0xFF == 27:
            break

    send_command("signal_off")
    cap.release()
    cv.destroyAllWindows()


if __name__ == "__main__":
    main()
