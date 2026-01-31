#
# Este código necesita también del código: vision.py
#
# La función es recibir el video de la raspberry y procesarlo con vision, escribir el color que se encontró en la terminal al igual que su centroide.
#
# Aquí se incluyen funciones para debuggear, transmite a la página y se abre el cuadro de video con bounding boxes y centroides.

from vision import detect_colors
import cv2 as cv
import requests

PI_IP = "172.32.192.88"
VIDEO_URL = f"http://{PI_IP}:5000/video_feed"
CMD_URL   = f"http://{PI_IP}:5000/command"
DATA_URL = f"http://{PI_IP}:5000/detection" # Nuevo para enviar datos de colores 


def send_command(cmd, speed=None):
    payload = {"cmd": cmd}
    if speed is not None:
        payload["speed"] = speed
    requests.post(CMD_URL, json=payload, timeout=0.2)

def main():
    cap = cv.VideoCapture(VIDEO_URL)

    if not cap.isOpened():
        print("No se pudo abrir el stream")
        return

    send_command("signal_on")

def send_detection(color, centroid): # nueva función para enviar detección
    payload = {
        "color": color,
        "centroid": {
            "x": int(centroid[0]),
            "y": int(centroid[1])
        }
    }
    try:
        requests.post(DATA_URL, json=payload, timeout=0.2)
    except:
        pass


    DETECTION_FRAMES = 18
    counter = 0
    color_detected = False

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        colors, centroids = detect_colors(frame, draw=True)

        if colors:
            counter += 1
        else:
            counter = 0
            if color_detected:
                print("Color perdido → reanudar señal")
                send_command("signal_on")
                color_detected = False

        if counter >= DETECTION_FRAMES and not color_detected:
            print("Color confirmado:", colors)
            print("Centroides:", centroids)

            # Enviar datos a la Raspberry, también es nuevo 
            for color in colors:
                if color in centroids:
                    send_detection(color, centroids[color])

            send_command("signal_off")
            color_detected = True


        cv.imshow("VISION ROVER (PC)", frame)

        if cv.waitKey(1) & 0xFF == 27:
            break

    send_command("signal_off")
    cap.release()
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()