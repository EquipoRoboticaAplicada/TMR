# Envío y recepción de datos PC - Raspberry Pi 5

import threading
from flask import Flask, Response, request, jsonify
from video_stream import gen_frames
from connect import ESP

# ----- Estado de Visión -----
vision_state = {"colors": [], "centroids": [], "areas": [], "time": 0.0}

# ----- La instancia ESP se inyecta desde main.py mediante init_app() -----
esp: ESP = None

command_state = {
    "left_dir":  "D1",
    "left_rpm":  "S0",
    "right_dir": "D1",
    "right_rpm": "S0",
}
command_lock = threading.Lock()

def init_app(esp_instance: ESP):
    """
    Recibe la instancia ESP ya conectada desde main.py.
    Debe llamarse antes de arrancar Flask.
    """
    global esp
    esp = esp_instance

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
    """Devuelve el estado actual de los motores (odometría) a la PC."""
    # print({"rover_state":esp.get_rover_state()}) # Debug
    return jsonify({"rover_state":esp.get_rover_state()})

@app.route("/rcv_speed_dir", methods=["POST"])
def rcv_speed_dir():
    """Recibe comandos desde la PC y los guarda en command_state."""
    data = request.get_json(force=True) or {}

    left_rpm  = data.get("left_rpm",  "S0")
    right_rpm = data.get("right_rpm", "S0")
    left_dir  = data.get("left_dir",  "D1")
    right_dir = data.get("right_dir", "D1")

    if not valid_S(left_rpm):  left_rpm  = "S0"
    if not valid_S(right_rpm): right_rpm = "S0"
    if not valid_D(left_dir):  left_dir  = "D1"
    if not valid_D(right_dir): right_dir = "D1"

    with command_lock:
        command_state.update({
            "left_dir":  left_dir,
            "left_rpm":  left_rpm,
            "right_dir": right_dir,
            "right_rpm": right_rpm,
        })
    return jsonify({"status": "ok"})