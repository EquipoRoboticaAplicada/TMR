from flask import Flask, Response, request, jsonify
from video_stream import gen_frames
import time
import serial
import threading

# ---- UART ESP32 ----
UART_LEFT  = "/dev/ttyACM0"
UART_RIGHT = None
# UART_RIGHT = "/dev/ttyUSB0"  # ejemplo: "/dev/ttyACM1" o "/dev/ttyUSB0"
BAUDRATE = 115200

ser_left = None
ser_right = None
serial_lock = threading.Lock() # Lock para proteger el acceso al estado

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

# ----- NUEVO ESTADO DEL ROVER (Reemplazo de control_state) -----
rover_state = {
    "left_side":  {"seq": 0, "dt_ms": 0, "motors": [{"ticks": 0, "m/s": 0.0} for _ in range(3)]},
    "right_side": {"seq": 0, "dt_ms": 0, "motors": [{"ticks": 0, "m/s": 0.0} for _ in range(3)]},
    "last_update": 0.0
}

# ----- Estado de Visión -----
vision_state = {"colors": [], "centroids": [], "areas": [], "time": 0.0}


# ================= LÓGICA DE LECTURA SERIAL (ODOMETRÍA) =================
def parse_esp_line(line):
    """
    Parsea la línea de telemetría de los motores y actualiza rover_state.
    Formato esperado: ESP_L/R, seq, dt_ms, ticks0, v0, ticks1, v1, ticks2, v2
    """
    global rover_state
    try:
        parts = line.strip().split(',')
        if len(parts) < 9:
            return

        header = parts[0]
        seq    = int(parts[1])
        dt_ms  = float(parts[2])

        m_data = [
            {"ticks": int(parts[3]), "m/s": float(parts[4])},
            {"ticks": int(parts[5]), "m/s": float(parts[6])},
            {"ticks": int(parts[7]), "m/s": float(parts[8])},
        ]

        with serial_lock:
            if header == "ESP_L":
                rover_state["left_side"].update({"seq": seq, "dt_ms": dt_ms, "motors": m_data})
            elif header == "ESP_R":
                rover_state["right_side"].update({"seq": seq, "dt_ms": dt_ms, "motors": m_data})
            
            rover_state["last_update"] = time.time()

    except Exception as e:
        pass # Ignorar errores de parseo por ruido en serial

def read_serial_thread(ser_obj):
    """Hilo que escucha constantemente un puerto Serial para recibir la odometría."""
    while ser_obj and ser_obj.is_open:
        try:
            raw = ser_obj.readline()
            if not raw:
                continue
            
            line = raw.decode('utf-8', errors='ignore').strip()
            if line.startswith("ESP"):
                parse_esp_line(line)

        except Exception as e:
            print(f"Error leyendo serial: {e}")
            break

# Iniciar los hilos de lectura serial si los puertos están abiertos
if ser_left:
    threading.Thread(target=read_serial_thread, args=(ser_left,), daemon=True).start()
if ser_right:
    threading.Thread(target=read_serial_thread, args=(ser_right,), daemon=True).start()


# ================= FUNCIONES DE CONTROL =================
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


# ================= FLASK =================
app = Flask(__name__)

@app.route("/video_feed")
def video_feed():
    return Response(gen_frames(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/telemetry", methods=["GET"])
def telemetry():
    """
    NUEVO ENDPOINT: Devuelve el estado actual de los motores (odometría) a la PC.
    """
    with serial_lock:
        return jsonify({
            "status": "ok", 
            "rover_state": rover_state
        })

@app.route("/command", methods=["POST"])
def command():
    """
    Maneja los comandos enviados desde la PC.
    """
    data = request.get_json(force=True) or {}

    left_rpm  = data.get("left_rpm",  "S0")
    right_rpm = data.get("right_rpm", "S0")
    left_dir  = data.get("left_dir",  "D1")
    right_dir = data.get("right_dir", "D1")

    if not valid_S(left_rpm):   left_rpm = "S0"
    if not valid_S(right_rpm):  right_rpm = "S0"
    if not valid_D(left_dir):   left_dir = "D1"
    if not valid_D(right_dir):  right_dir = "D1"

    try:
        send_uart(left_dir, left_rpm, right_dir, right_rpm)
    except Exception as e:
        print("Error UART:", e)

    # Ahora retornamos el rover_state para que la PC pueda ver la odometría como respuesta al comando
    with serial_lock:
        return jsonify({"status": "ok", "rover_state": rover_state})

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