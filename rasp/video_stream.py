import cv2 as cv

camera = cv.VideoCapture(0, cv.CAP_V4L2)
camera.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*"MJPG"))
camera.set(cv.CAP_PROP_FRAME_WIDTH, 320)
camera.set(cv.CAP_PROP_FRAME_HEIGHT, 240)

# Muy importante para evitar "cola" de frames (lag acumulado)
camera.set(cv.CAP_PROP_BUFFERSIZE, 1)

# Parámetros de compresión (ajusta calidad: 30–60 suele ser buen rango)
ENCODE_PARAMS = [int(cv.IMWRITE_JPEG_QUALITY), 45, int(cv.IMWRITE_JPEG_OPTIMIZE), 1]

def gen_frames():
    while True:
        # Opcional: drenar cola y quedarte con el frame más reciente
        camera.grab()
        ret, frame = camera.retrieve()
        if not ret:
            continue

        # 1) RASP: BGR -> GRIS (reduce datos antes de comprimir)
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        # 2) Comprimir en JPG (en gris)
        ok, buffer = cv.imencode(".jpg", gray, ENCODE_PARAMS)
        if not ok:
            continue

        jpg_bytes = buffer.tobytes()

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" + jpg_bytes + b"\r\n"
        )
