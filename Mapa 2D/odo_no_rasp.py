import time
import serial
import serial.tools.list_ports
import threading
import platform

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

        header = parts[0]  # ESP_L o ESP_R
        seq    = int(parts[1])
        dt_ms  = int(parts[2])

        m_data = [
            {"ticks": int(parts[3]), "rpm": float(parts[4])},
            {"ticks": int(parts[5]), "rpm": float(parts[6])},
            {"ticks": int(parts[7]), "rpm": float(parts[8])}
        ]

        if header == "ESP_L":
            rover_state["left_side"]["seq"]    = seq
            rover_state["left_side"]["dt_ms"]  = dt_ms
            rover_state["left_side"]["motors"] = m_data
        elif header == "ESP_R":
            rover_state["right_side"]["seq"]    = seq
            rover_state["right_side"]["dt_ms"]  = dt_ms
            rover_state["right_side"]["motors"] = m_data

        rover_state["last_update"] = time.time()

    except ValueError:
        pass
    except Exception as e:
        print(f"Error parseando línea: {line} -> {e}")


def read_serial_thread(ser_obj):
    """Hilo que escucha constantemente un puerto Serial."""
    while ser_obj and ser_obj.is_open:
        try:
            raw = ser_obj.readline()
            if not raw:
                continue

            line = raw.decode('utf-8', errors='ignore').strip()
            if not line:
                continue

            print(f"[RAW SERIAL] {line}")

            if line.startswith("ESP"):
                parse_esp_line(line)

        except serial.SerialException as e:
            print(f"Error serial (desconexión?): {e}")
            break
        except Exception as e:
            print(f"Error leyendo serial: {e}")


def get_available_ports():
    """
    Devuelve la lista de puertos COM disponibles en Windows
    (o /dev/ttyUSB* y /dev/ttyACM* en Linux/Mac como fallback).
    """
    system = platform.system()

    if system == "Windows":
        # serial.tools.list_ports funciona en todas las plataformas
        # pero en Windows es la única forma fiable de listar COMs
        ports = [p.device for p in serial.tools.list_ports.comports()]
    else:
        # Fallback para Linux / macOS (Raspberry Pi, etc.)
        import glob
        ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')

    return ports


def auto_connect_esps():
    """
    Busca puertos disponibles, escucha unos segundos para identificar
    'ESP_L' o 'ESP_R' y asigna las variables globales ser_left / ser_right.
    """
    global ser_left, ser_right

    ports = get_available_ports()
    print(f"🔍 Buscando ESPs en: {ports}")

    if not ports:
        print("⚠️  No se encontraron puertos seriales.")
        return

    for port in ports:
        try:
            s = serial.Serial(port, BAUDRATE, timeout=0.1)
            print(f"Probando {port}...")

            start_time  = time.time()
            identified  = False

            while time.time() - start_time < 3.0:
                if s.in_waiting:
                    raw  = s.readline()
                    line = raw.decode('utf-8', errors='ignore').strip()

                    if line.startswith("ESP_L"):
                        if ser_left is None:
                            ser_left = s
                            print(f"✅ LADO IZQUIERDO detectado en {port}")
                            threading.Thread(
                                target=read_serial_thread,
                                args=(s,),
                                daemon=True
                            ).start()
                            identified = True
                            break

                    elif line.startswith("ESP_R"):
                        if ser_right is None:
                            ser_right = s
                            print(f"✅ LADO DERECHO detectado en {port}")
                            threading.Thread(
                                target=read_serial_thread,
                                args=(s,),
                                daemon=True
                            ).start()
                            identified = True
                            break

            if not identified:
                print(f"⚠️  No se identificó ESP en {port} (cerrando).")
                s.close()

        except serial.SerialException as e:
            print(f"No se pudo abrir {port}: {e}")
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
    Protocolo original: D{x}\\n luego S{x}\\n
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


# ================= MAIN =================
if __name__ == "__main__":
    print("🚀 Iniciando Servidor Rover (Video + Odometría 6 Ruedas)...")
    print(f"   Sistema operativo detectado: {platform.system()}")

    # 1. Intentar conectar ESPs automáticamente
    auto_connect_esps()

    if ser_left is None and ser_right is None:
        print("⚠️  ADVERTENCIA: No se detectaron ESPs. El servidor correrá solo video.")
    else:
        # Mantener el proceso principal vivo mientras los hilos leen serial
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n🛑 Cerrando conexiones...")
            if ser_left  and ser_left.is_open:  ser_left.close()
            if ser_right and ser_right.is_open: ser_right.close()
            print("✅ Conexiones cerradas. Hasta luego.")