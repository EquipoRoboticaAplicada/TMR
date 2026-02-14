from flask import Flask, Response, request, jsonify
# Asegúrate de que video_stream.py existe en la misma carpeta
from video_stream import gen_frames 
import time
import serial
import threading
import glob

# ================= CONFIGURACIÓN SERIAL =================
BAUDRATE = 115200

# Variables globales para los puertos seriales (se asignan dinámicamente)
ser_left = None
ser_right = None

# ================= ESTADOS GLOBALES =================

# 1. Estado de Visión (Existente)
vision_state = {"colors": [], "centroids": [], "areas": [], "time": 0.0}

# 2. Estado de Control (Existente)
control_state = {
    "left_rpm":  "S0",
    "right_rpm": "S0",
    "left_dir":  "D1",
    "right_dir": "D1",
    "time": 0.0
}

# 3. Estado de Odometría (NUEVO - 6 Ruedas)
# Estructura: 2 lados, cada uno con 3 motores
rover_state = {
    "left_side":  {"seq": 0, "dt_ms": 0, "motors": [{"ticks":0, "rpm":0}, {"ticks":0, "rpm":0}, {"ticks":0, "rpm":0}]},
    "right_side": {"seq": 0, "dt_ms": 0, "motors": [{"ticks":0, "rpm":0}, {"ticks":0, "rpm":0}, {"ticks":0, "rpm":0}]},
    "last_update": 0
}

# ================= LÓGICA DE ODOMETRÍA Y SERIAL =================

def parse_esp_line(line):
    """
    Parsea la línea de telemetría de 6 motores.
    Formato esperado: ID, seq, dt, t1, r1, t2, r2, t3, r3
    Ej: ESP_L,100,50,120,30.5,115,30.2,122,30.8
    """
    global rover_state
    try:
        parts = line.strip().split(',')
        if len(parts) < 9:
            return

        header = parts[0] # ESP_L o ESP_R
        seq = int(parts[1])
        dt_ms = int(parts[2])
        
        # Extraer datos de los 3 motores del lado correspondiente
        m_data = [
            {"ticks": int(parts[3]), "rpm": float(parts[4])},
            {"ticks": int(parts[5]), "rpm": float(parts[6])},
            {"ticks": int(parts[7]), "rpm": float(parts[8])}
        ]

        # Actualizar el estado global
        if header == "ESP_L":
            rover_state["left_side"]["seq"] = seq
            rover_state["left_side"]["dt_ms"] = dt_ms
            rover_state["left_side"]["motors"] = m_data
        elif header == "ESP_R":
            rover_state["right_side"]["seq"] = seq
            rover_state["right_side"]["dt_ms"] = dt_ms
            rover_state["right_side"]["motors"] = m_data
        
        rover_state["last_update"] = time.time()

    except ValueError:
        pass # Error de parseo numérico
    except Exception as e:
        print(f"Error parseando línea: {line} -> {e}")

def read_serial_thread(ser_obj):
    """Hilo que escucha constantemente un puerto Serial."""
    while ser_obj and ser_obj.is_open:
        try:
            if ser_obj.in_waiting > 0:
                line = ser_obj.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    # Intentamos parsear como odometría
                    if line.startswith("ESP"):
                        parse_esp_line(line)
                    # Aquí podrías agregar logs si la ESP manda prints de debug
        except Exception as e:
            print(f"Error leyendo serial: {e}")
            break
        time.sleep(0.001) # Pequeña pausa para no saturar CPU

def auto_connect_esps():
    """
    Busca puertos USB, escucha unos segundos para identificar 'ESP_L' o 'ESP_R'
    y asigna las variables globales ser_left y ser_right.
    """
    global ser_left, ser_right
    
    # Buscar puertos candidatos
    ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
    print(f"🔍 Buscando ESPs en: {ports}")

    for port in ports:
        try:
            s = serial.Serial(port, BAUDRATE, timeout=2.0)
            print(f"Proando {port}...")
            
            # Escuchar brevemente para identificar
            start_time = time.time()
            identified = False
            
            while time.time() - start_time < 3.0: # 3 seg max para handshake
                if s.in_waiting:
                    line = s.readline().decode('utf-8', errors='ignore').strip()
                    if line.startswith("ESP_L"):
                        if ser_left is None:
                            ser_left = s
                            print(f"✅ LADO IZQUIERDO detectado en {port}")
                            # Iniciar hilo de lectura dedicado
                            threading.Thread(target=read_serial_thread, args=(s,), daemon=True).start()
                            identified = True
                            break
                    elif line.startswith("ESP_R"):
                        if ser_right is None:
                            ser_right = s
                            print(f"✅ LADO DERECHO detectado en {port}")
                            # Iniciar hilo de lectura dedicado
                            threading.Thread(target=read_serial_thread, args=(s,), daemon=True).start()
                            identified = True
                            break
            
            if not identified:
                print(f"⚠️ No se identificó ESP en {port} (cerrando).")
                s.close()

        except Exception as e:
            print(f"Error conectando a {port}: {e}")

# ================= FUNCIONES AUXILIARES DE CONTROL =================

def valid_S(x):
    return isinstance(x, str) and x.startswith("S") and len(x) >= 2

def valid_D(x):
    return isinstance(x, str) and x in ("D0", "D1")

def send_uart(left_dir, left_rpm, right_dir, right_rpm):
    """
    Envía comandos a los puertos detectados.
    Mantiene el protocolo original: D{x}\n luego S{x}\n
    """
    try:
        if ser_left and ser_left.is_open:
            ser_left.write((left_dir + "\n").encode())
            ser_left.write((left_rpm + "\n").encode())
        
        if ser_right and ser_right.is_open:
            ser_right.write((right_dir + "\n").encode())
            ser_right.write((right_rpm + "\n").encode())
            
    except Exception as e:
        print(f"Error enviando UART: {e}")

# ================= FLASK APP =================
app = Flask(__name__)

@app.route("/video_feed")
def video_feed():
    """Stream de video MJPEG (Lógica original)."""
    return Response(gen_frames(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/telemetry", methods=["GET"])
def telemetry():
    """(NUEVO) Devuelve el estado de odometría de los 6 motores."""
    return jsonify({
        "status": "ok",
        "rover_state": rover_state,
        "control_feedback": control_state
    })

@app.route("/command", methods=["POST"])
def command():
    """Recibe comandos de la PC y los envía a las ESPs."""
    global control_state

    data = request.get_json(force=True) or {}

    left_rpm  = data.get("left_rpm",  "S0")
    right_rpm = data.get("right_rpm", "S0")
    left_dir  = data.get("left_dir",  "D1")
    right_dir = data.get("right_dir", "D1")

    # Validación
    if not valid_S(left_rpm):   left_rpm = "S0"
    if not valid_S(right_rpm):  right_rpm = "S0"
    if not valid_D(left_dir):   left_dir = "D1"
    if not valid_D(right_dir):  right_dir = "D1"

    control_state = {
        "left_rpm": left_rpm,
        "right_rpm": right_rpm,
        "left_dir": left_dir,
        "right_dir": right_dir,
        "time": time.time()
    }

    # Enviar a hardware
    send_uart(left_dir, left_rpm, right_dir, right_rpm)

    return jsonify({"status": "ok", "control_state": control_state})

@app.route("/vision_data", methods=["POST"])
def vision_data():
    """Recibe datos de procesamiento de imagen (Lógica original)."""
    global vision_state

    data = request.get_json(force=True) or {}
    vision_state["colors"] = data.get("colors", [])
    vision_state["centroids"] = data.get("centroids", [])
    vision_state["areas"] = data.get("areas", [])
    vision_state["time"] = data.get("time", time.time())

    return jsonify({"status": "ok"})

# ================= MAIN =================
if __name__ == "__main__":
    print("🚀 Iniciando Servidor Rover (Video + Odometría 6 Ruedas)...")
    
    # 1. Intentar conectar ESPs automáticamente
    auto_connect_esps()
    
    if ser_left is None and ser_right is None:
        print("⚠️ ADVERTENCIA: No se detectaron ESPs. El servidor correrá solo video.")

    # 2. Arrancar Flask
    app.run(host="0.0.0.0", port=5000, threaded=True)