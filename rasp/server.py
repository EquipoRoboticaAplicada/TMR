from flask import Flask, Response, request, jsonify
from video_stream import gen_frames
import time
import serial
        
# ---- UART ESP32 ----
UART_LEFT  = "/dev/ttyACM0"
UART_RIGHT = None
# UART_RIGHT = "/dev/ttyUSB0"  # ejemplo: "/dev/ttyACM1" o "/dev/ttyUSB0"
BAUDRATE = 115200

ser_left = None
ser_right = None

def open_serial(port):
    if not port:
        return None
    try:
        s = serial.Serial(port, BAUDRATE, timeout=0.05)
        print(f"UART abierto: {port}")
        return s
    except Exception as e:
        print(f"Error abriendo UART {port}: {e}")
        return None

ser_left = open_serial(UART_LEFT)
ser_right = open_serial(UART_RIGHT)

def valid_S(x):
    return isinstance(x, str) and x.startswith("S") and len(x) >= 2

def valid_D(x):
    return isinstance(x, str) and x in ("D0", "D1")

def send_uart(left_dir, left_rpm, right_dir, right_rpm):
    """
    Envía comandos ya formateados tipo 'D1' y 'S20' (con newline).
    """
    if ser_left:
        ser_left.write((left_dir + "\n").encode())
        ser_left.write((left_rpm + "\n").encode())

    if ser_right:
        ser_right.write((right_dir + "\n").encode())
        ser_right.write((right_rpm + "\n").encode())

# ----- Estado de Visión -----
vision_state = {"colors": [], "centroids": [], "areas": [], "time": 0.0}

# ---- Estado de Control -----
control_state = {
    "left_rpm":  "S0",
    "right_rpm": "S0",
    "left_dir":  "D1",
    "right_dir": "D1",
    "time": 0.0
}

# ----- Flask -----
app = Flask(__name__)

@app.route("/video_feed")
def video_feed():
    return Response(gen_frames(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/command", methods=["POST"])
def command():
    global control_state

    data = request.get_json(force=True) or {}

    # La PC manda strings tipo "S20" y "D1"
    left_rpm  = data.get("left_rpm",  "S0")
    right_rpm = data.get("right_rpm", "S0")
    left_dir  = data.get("left_dir",  "D1")
    right_dir = data.get("right_dir", "D1")

    # Validación rápida (evita basura)
    if not valid_S(left_rpm):   left_rpm = "S0"
    if not valid_S(right_rpm):  right_rpm = "S0"
    if not valid_D(left_dir):   left_dir = "D1"
    if not valid_D(right_dir):  right_dir = "D1"

    # Guarda estado
    control_state = {
        "left_rpm": left_rpm,
        "right_rpm": right_rpm,
        "left_dir": left_dir,
        "right_dir": right_dir,
        "time": time.time()
    }

    # Reenvía por UART a los ESP32
    try:
        send_uart(left_dir, left_rpm, right_dir, right_rpm)
    except Exception as e:
        print("Error UART:", e)

    # Debug
    # print("CMD recibido:", control_state)

    return jsonify({"status": "ok", "control_state": control_state})

@app.route("/vision_data", methods=["POST"])
def vision_data():
    global vision_state

    data = request.get_json(force=True) or {}
    vision_state["colors"] = data.get("colors", [])
    vision_state["centroids"] = data.get("centroids", [])
    vision_state["areas"] = data.get("areas", [])
    vision_state["time"] = data.get("time", time.time())

    return jsonify({"status": "ok"})

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, threaded=True)
