# server.py
import threading
from flask import Flask, Response, request, jsonify
from video_stream import gen_frames, init_video_stream
from connect import ESP
from zed import ZEDShared

# ----- Estado de Visión -----
vision_state = {"colors": [], "centroids": [], "areas": [], "time": 0.0}

# ----- Instancias globales inyectadas desde main.py -----
esp: ESP = None
zed: ZEDShared = None

command_state = {
    "left_dir":  "D1",
    "left_rpm":  "S0",
    "right_dir": "D1",
    "right_rpm": "S0",
}
command_lock = threading.Lock()

def init_app(esp_instance: ESP, zed_instance: ZEDShared):
    global esp, zed
    esp = esp_instance
    zed = zed_instance
    init_video_stream(zed)

# ================= FUNCIONES DE CONTROL =================
def valid_S(x):
    return isinstance(x, str) and x.startswith("S") and len(x) >= 2

def valid_D(x):
    return isinstance(x, str) and x in ("D0", "D1")

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

@app.route("/depth_at", methods=["POST"])
def depth_at():
    """
    Recibe x,y del píxel detectado en la laptop y devuelve profundidad en metros.
    """
    data = request.get_json(force=True) or {}

    x = data.get("x", None)
    y = data.get("y", None)

    if x is None or y is None:
        return jsonify({"ok": False, "error": "faltan x,y"}), 400

    try:
        x = int(x)
        y = int(y)
    except Exception:
        return jsonify({"ok": False, "error": "x,y invalidos"}), 400

    result = zed.get_frame_copy()
    if result is None:
        return jsonify({"ok": False, "error": "sin frame disponible"}), 503

    frame, ts = result
    h, w = frame.shape[:2]

    distance_m = zed.get_depth_median_neighborhood(x, y, radius=2)

    return jsonify({
        "ok": True,
        "x": x,
        "y": y,
        "distance_m": distance_m,
        "timestamp": ts,
        "frame_width": w,
        "frame_height": h
    })