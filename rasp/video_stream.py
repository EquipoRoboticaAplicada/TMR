import cv2 as cv
import time

camera = cv.VideoCapture(0, cv.CAP_V4L2)
camera.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*"MJPG"))
camera.set(cv.CAP_PROP_FRAME_WIDTH, 320)
camera.set(cv.CAP_PROP_FRAME_HEIGHT, 240)

# Evita acumulación interna en V4L2 (a veces funciona, a veces no, pero ayuda)
camera.set(cv.CAP_PROP_BUFFERSIZE, 1)

# Ajustes principales
TARGET_FPS = 8
FRAME_PERIOD = 1.0 / TARGET_FPS

# Calidad JPEG (30–50 suele ser buen rango para bajar bytes sin destruir la imagen)
ENCODE_PARAMS = [
    int(cv.IMWRITE_JPEG_QUALITY), 40,
    int(cv.IMWRITE_JPEG_OPTIMIZE), 1
]

def gen_frames():
    last = 0.0
    while True:
        # 1) FPS cap (no generes frames más rápido de lo necesario)
        now = time.time()
        if now - last < FRAME_PERIOD:
            continue
        last = now

        # 2) “Tirar frames viejos”: grab() lee y descarta, retrieve() toma el más reciente
        camera.grab()
        ret, frame = camera.retrieve()
        if not ret:
            continue

        # 3) Comprimir más ligero (pero conservando color)
        ok, buffer = cv.imencode(".jpg", frame, ENCODE_PARAMS)
        if not ok:
            continue

        jpg = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + jpg + b'\r\n')
