# video_stream.py
import cv2 as cv
import time

zed_shared = None

TARGET_FPS = 8
FRAME_PERIOD = 1.0 / TARGET_FPS

ENCODE_PARAMS = [
    int(cv.IMWRITE_JPEG_QUALITY), 40,
    int(cv.IMWRITE_JPEG_OPTIMIZE), 1
]

def init_video_stream(zed_instance):
    global zed_shared
    zed_shared = zed_instance

def gen_frames():
    if zed_shared is None:
        raise RuntimeError("video_stream no fue inicializado con ZEDShared")

    last = 0.0
    while True:
        now = time.time()
        if now - last < FRAME_PERIOD:
            continue
        last = now

        result = zed_shared.get_frame_copy()
        if result is None:
            continue

        frame, ts = result

        ok, buffer = cv.imencode(".jpg", frame, ENCODE_PARAMS)
        if not ok:
            continue

        jpg = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + jpg + b'\r\n')
