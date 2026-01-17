from flask import Flask, Response, request, jsonify
import cv2 as cv
from gpiozero import PWMOutputDevice, DigitalOutputDevice

# ----- GPIO -----
ENA = PWMOutputDevice(18)
IN1 = DigitalOutputDevice(23)
IN2 = DigitalOutputDevice(24)

def motor_on(speed=0.5):
    IN1.on()
    IN2.off()
    ENA.value = speed

def motor_off():
    ENA.off()
    IN1.off()
    IN2.off()

# ----- Flask -----
app = Flask(__name__)

camera = cv.VideoCapture(0, cv.CAP_V4L2)
camera.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*"MJPG"))
camera.set(cv.CAP_PROP_FRAME_WIDTH, 320)
camera.set(cv.CAP_PROP_FRAME_HEIGHT, 240)

def gen_frames():
    while True:
        ret, frame = camera.read()
        if not ret:
            break
        _, buffer = cv.imencode('.jpg', frame)
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route("/video_feed")
def video_feed():
    return Response(gen_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/command", methods=["POST"])
def command():
    data = request.json
    cmd = data.get("cmd")

    if cmd == "motor_on":
        speed = data.get("speed", 0.5)
        motor_on(speed)
    elif cmd == "motor_off":
        motor_off()

    return jsonify({"status": "ok", "cmd": cmd})

if __name__ == "__main__":
    motor_off()
    app.run(host="0.0.0.0", port=5000)