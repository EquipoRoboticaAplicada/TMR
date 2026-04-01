# server.py
import threading
from flask import Flask, Response, request, jsonify
from video_stream import gen_frames, init_video_stream
from connect import ESP

esp: ESP = None
vision = None
tracker = None

command_state = {
    "left_dir":  "D1",
    "left_rpm":  "S0",
    "right_dir": "D1",
    "right_rpm": "S0",
}
command_lock = threading.Lock()


def init_app(esp_instance: ESP, zed_instance, vision_instance, tracker_instance):
    global esp, vision, tracker
    esp = esp_instance
    vision = vision_instance
    tracker = tracker_instance
    init_video_stream(zed_instance)


def valid_S(x):
    return isinstance(x, str) and x.startswith("S") and len(x) >= 2


def valid_D(x):
    return isinstance(x, str) and x in ("D0", "D1")


app = Flask(__name__)


@app.route("/video_feed")
def video_feed():
    return Response(gen_frames(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/telemetry", methods=["GET"])
def telemetry():
    return jsonify({"rover_state": esp.get_rover_state()})


@app.route("/vision_state", methods=["GET"])
def vision_state():
    if vision is None:
        return jsonify({"ok": False, "error": "vision no inicializada"}), 503

    v_state = vision.get_state()
    t_state = tracker.get_state() if tracker is not None else {"tracking_active": False}

    return jsonify({
        "ok": True,
        **v_state,
        **t_state
    })


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