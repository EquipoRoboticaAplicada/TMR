# server.py
import threading
from flask import Flask, Response, request, jsonify
from video_stream import gen_frames, init_video_stream
from connect import ESP
from zed import ZEDShared

# ----- Instancias globales inyectadas desde main.py -----
esp: ESP = None
zed: ZEDShared = None

def init_app(esp_instance: ESP, zed_instance: ZEDShared):
    global esp, zed
    esp = esp_instance
    zed = zed_instance
    init_video_stream(zed)

# ================= FLASK =================
app = Flask(__name__)

@app.route("/video_feed")
def video_feed():
    return Response(gen_frames(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/telemetry", methods=["GET"])
def telemetry():
    return jsonify({"rover_state": esp.get_rover_state()})

@app.route("/rcv_speed_dir", methods=["POST"])
def rcv_speed_dir():
    data = request.get_json(force=True) or {}

    left_rpm  = data.get("left_rpm",  "S0")
    right_rpm = data.get("right_rpm", "S0")
    left_dir  = data.get("left_dir",  "D1")
    right_dir = data.get("right_dir", "D1")

    if not valid_S(left_rpm):
        left_rpm = "S0"
    if not valid_S(right_rpm):
        right_rpm = "S0"
    if not valid_D(left_dir):
        left_dir = "D1"
    if not valid_D(right_dir):
        right_dir = "D1"

    with command_lock:
        command_state.update({
            "left_dir":  left_dir,
            "left_rpm":  left_rpm,
            "right_dir": right_dir,
            "right_rpm": right_rpm,
        })

    return jsonify({"status": "ok"})
