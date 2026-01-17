from vision import detect_colors
import cv2 as cv
import requests

PI_IP = "172.32.192.88"   # ← IP de la Raspberry
VIDEO_URL = f"http://{PI_IP}:5000/video_feed"
CMD_URL   = f"http://{PI_IP}:5000/command"

def send_command(cmd, speed=None):
    payload = {"cmd": cmd}
    if speed is not None:
        payload["speed"] = speed
    requests.post(CMD_URL, json=payload, timeout=0.2)

def main():
    cap = cv.VideoCapture(VIDEO_URL)

    if not cap.isOpened():
        print("❌ No se pudo abrir el stream")
        return

    send_command("motor_on", 0.5)

    DETECTION_FRAMES = 18
    counter = 0
    color_detected = False

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        colors = detect_colors(frame, draw=True)

        if colors:
            counter += 1
        else:
            counter = 0
            if color_detected:
                print("Color perdido, reanudando motor")
                send_command("motor_on", 0.5)
                color_detected = False

        if counter >= DETECTION_FRAMES and not color_detected:
            print("Color confirmado:", colors)
            send_command("motor_off")
            color_detected = True

        cv.imshow("VISION ROVER (PC)", frame)

        if cv.waitKey(1) & 0xFF == 27:
            break

    send_command("motor_off")
    cap.release()
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()