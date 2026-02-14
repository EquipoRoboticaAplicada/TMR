import time
import serial
import threading
from flask import Flask, request, jsonify
import glob

# ================= CONFIGURACIÓN =================
BAUDRATE = 115200

# Diccionario global para guardar el estado completo del rover
# M1, M2, M3 = Izquierda | M4, M5, M6 = Derecha
rover_state = {
    "left_side":  {"seq": 0, "dt_ms": 0, "motors": [{"ticks":0, "rpm":0}, {"ticks":0, "rpm":0}, {"ticks":0, "rpm":0}]},
    "right_side": {"seq": 0, "dt_ms": 0, "motors": [{"ticks":0, "rpm":0}, {"ticks":0, "rpm":0}, {"ticks":0, "rpm":0}]},
    "last_update": 0
}

# Referencias a los objetos Serial
serial_left = None
serial_right = None

app = Flask(__name__)

# ================= LÓGICA DE PARSEO =================
def parse_esp_line(line):
    """
    Formato esperado: ID, seq, dt, t1, r1, t2, r2, t3, r3
    Ej: ESP_L,100,50,10,2.5,10,2.5,10,2.5
    """
    global rover_state
    try:
        parts = line.strip().split(',')
        if len(parts) < 9:
            return

        header = parts[0] # ESP_L o ESP_R
        seq = int(parts[1])
        dt_ms = int(parts[2])
        
        # Extraer datos de los 3 motores
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
        pass
    except Exception as e:
        print(f"Error parsing: {e}")

# ================= GESTIÓN SERIAL =================
def read_from_port(ser_obj):
    """Función que corre en un hilo por cada puerto conectado"""
    while ser_obj and ser_obj.is_open:
        try:
            if ser_obj.in_waiting:
                line = ser_obj.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    parse_esp_line(line)
        except Exception as e:
            print(f"Error en lectura serial: {e}")
            break
        # Pequeña pausa para no saturar CPU si no hay datos
        time.sleep(0.001) 

def auto_connect_esps():
    """Busca puertos USB, escucha una línea para identificar quién es quién"""
    global serial_left, serial_right
    
    # Buscar posibles puertos (ttyUSB* o ttyACM*)
    ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
    print(f"🔍 Puertos encontrados: {ports}")

    for port in ports:
        try:
            # Intentar abrir puerto
            s = serial.Serial(port, BAUDRATE, timeout=2.0)
            print(f"Abriendo {port} para identificación...")
            
            # Esperar unos segundos a recibir datos para ver el Header
            start_time = time.time()
            identified = False
            
            while time.time() - start_time < 3.0: # 3 segundos max para identificarse
                if s.in_waiting:
                    line = s.readline().decode('utf-8', errors='ignore').strip()
                    if line.startswith("ESP_L"):
                        if serial_left is None:
                            serial_left = s
                            print(f"✅ LADO IZQUIERDO detectado en {port}")
                            threading.Thread(target=read_from_port, args=(s,), daemon=True).start()
                            identified = True
                            break
                    elif line.startswith("ESP_R"):
                        if serial_right is None:
                            serial_right = s
                            print(f"✅ LADO DERECHO detectado en {port}")
                            threading.Thread(target=read_from_port, args=(s,), daemon=True).start()
                            identified = True
                            break
            
            if not identified:
                print(f"⚠️ No se pudo identificar dispositivo en {port} (cerrando).")
                s.close()

        except Exception as e:
            print(f"Error al intentar conectar con {port}: {e}")

# ================= FLASK =================
@app.route("/telemetry", methods=["GET"])
def get_telemetry():
    """Devuelve el estado completo de los 6 motores"""
    return jsonify(rover_state)

@app.route("/command", methods=["POST"])
def command():
    """
    Recibe comando general y lo divide para las dos ESPs.
    Ejemplo JSON entrada: {"left_rpm": 50, "right_rpm": 50, "left_dir": 1, ...}
    """
    data = request.get_json(force=True) or {}
    
    # Aquí construimos el comando para Arduino.
    # Sugerencia: Enviar "CMD,dir,rpm" a cada ESP
    # Asumimos que la ESP aplica la misma velocidad a sus 3 motores (skid steer básico)
    
    cmd_l = f"CMD,{data.get('left_dir', 1)},{data.get('left_rpm', 0)}\n"
    cmd_r = f"CMD,{data.get('right_dir', 1)},{data.get('right_rpm', 0)}\n"

    if serial_left and serial_left.is_open:
        serial_left.write(cmd_l.encode())
    
    if serial_right and serial_right.is_open:
        serial_right.write(cmd_r.encode())

    return jsonify({"status": "sent", "cmds": [cmd_l.strip(), cmd_r.strip()]})

# ================= MAIN =================
if __name__ == "__main__":
    print("🚀 Iniciando Servidor de Odometría 6-Ruedas...")
    
    # 1. Intentar autoconectar
    auto_connect_esps()
    
    if serial_left is None and serial_right is None:
        print("⚠️ ADVERTENCIA: No se detectaron ESPs. El servidor correrá pero sin datos.")

    # 2. Iniciar Flask
    app.run(host="0.0.0.0", port=5000)