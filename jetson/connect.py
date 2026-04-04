import threading
import platform
import serial
import time
import copy

class ESP: 

    BAUDRATE = 115200

    def __init__(self):
        
        # Puertos seriales 
        self._ser_left  = None # Motores izquierda
        self._ser_right = None # Motores derecha
        self._lock      = threading.Lock()

        # Estado del rover (odometría)
        self._rover_state = {
            "left_side":  {"seq": 0, "motors": [{"rpm": 0.0, "m/s": 0.0} for _ in range(3)]},
            "right_side": {"seq": 0, "motors": [{"rpm": 0.0, "m/s": 0.0} for _ in range(3)]},
            "last_update": 0.0
        }

    def connect(self):
        """Busca y conecta automáticamente los ESP32 por puerto serial."""
        ports = self._get_available_ports()

        if not ports:
            print("⚠️  No se encontraron puertos seriales.\n")
            return 
        else:
            print(f"🔍 Buscando ESPs en: {ports}")
            for port in ports:
                self._try_connect_port(port)
        

        if self._ser_left is None and self._ser_right is None:
            print("⚠️  No se detectaron ESPs.\n")
            return

    def _get_available_ports(self) -> list:
        try: 
            if platform.system() == "Windows":
                return [p.device for p in serial.tools.list_ports.comports()]
            import glob
            return glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
        except Exception as e: 
            print(f"connect.py error: (_get_available_ports()); {e}\n")
            return None

    def _try_connect_port(self, port: str):
        try:
            s = serial.Serial(port, self.BAUDRATE, timeout=0.1)
            print(f"Probando {port}...")

            deadline = time.time() + 3.0
            identified = False

            while time.time() < deadline:
                if s.in_waiting:
                    line = s.readline().decode('utf-8', errors='ignore').strip()

                    if line.startswith("ESP_L") and self._ser_left is None:
                        self._ser_left = s
                        print(f"✅ LADO IZQUIERDO detectado en {port}")
                        threading.Thread(
                            target=self._read_serial_thread,
                            args=(s, "left"),
                            daemon=True
                        ).start()
                        identified = True
                        break

                    elif line.startswith("ESP_R") and self._ser_right is None:
                        self._ser_right = s
                        print(f"✅ LADO DERECHO detectado en {port}")
                        threading.Thread(
                            target=self._read_serial_thread,
                            args=(s, "right"),
                            daemon=True
                        ).start()
                        identified = True
                        break

            if not identified:
                print(f"⚠️  No se identificó ESP en {port} (cerrando).\n")
                s.close()

        except serial.SerialException as e:
            print(f"No se pudo abrir {port}: {e}")
        except Exception as e:
            print(f"Error conectando a {port}: {e}")

    def _read_serial_thread(self, ser_obj, side: str):
        while True:
            try:
                if ser_obj is None or not ser_obj.is_open:
                    break

                raw = ser_obj.readline()
                if not raw:
                    continue

                line = raw.decode("utf-8", errors="ignore").strip()

                if line.startswith("ESP_L") or line.startswith("ESP_R"):
                    self._parse_esp_line(line)

            except serial.SerialException as e:
                print(f"Error serial ({side}): {e}")

                with self._lock:
                    try:
                        if ser_obj and ser_obj.is_open:
                            ser_obj.close()
                    except Exception:
                        pass

                    if side == "left" and self._ser_left is ser_obj:
                        self._ser_left = None
                    elif side == "right" and self._ser_right is ser_obj:
                        self._ser_right = None

                break

            except Exception as e:
                print(f"Error inesperado leyendo serial ({side}): {e}")
                break

    def _parse_esp_line(self, line: str):
        """
        Formato: ESP_L/R, seq, rpm0, v0, rpm1, v1, rpm2, v2
        """
        if not line:
            return

        try:
            # print(line)
            parts = line.strip().split(',')

            if len(parts) != 8:
                return

            try:
                header = parts[0]
                seq    = int(parts[1])
                m_data = [
                    {"rpm": float(parts[2]), "m/s": float(parts[3])},
                    {"rpm": float(parts[4]), "m/s": float(parts[5])},
                    {"rpm": float(parts[6]), "m/s": float(parts[7])},
                ]
            except (ValueError, IndexError):
                print("[_parse_esp_line] Error.\n")
                return

            with self._lock:
                if header == "ESP_L":
                    self._rover_state["left_side"].update(
                        {"seq": seq, "motors": m_data}
                    )
                elif header == "ESP_R":
                    self._rover_state["right_side"].update(
                        {"seq": seq, "motors": m_data}
                    )
                else:
                    print("Header: ESP_L/R, no reconocido.\n")
                    return  # Header desconocido, ignorar

                self._rover_state["last_update"] = time.time()

        except ValueError as e:
            print(f"[parse] ValueError en: {repr(line)} → {e}")
        except Exception as e:
            print(f"[parse] Error inesperado: {repr(line)} → {e}")

    def get_rover_state(self) -> dict:
        """
        Devuelve una copia segura del rover_state actual.
        Usar siempre esta función desde server.py, nunca acceder a _rover_state directamente.
        """
        with self._lock:
            # print((self._rover_state)) # Debug
            return copy.deepcopy(self._rover_state)

    def send_uart(self, left_dir, left_rpm, right_dir, right_rpm):
        for val in (left_dir, left_rpm, right_dir, right_rpm):
            assert isinstance(val, str), f"Tipo inválido: {val!r}"

        with self._lock:
            left  = self._ser_left
            right = self._ser_right
            # print(f"Enviando UART → L: {left_rpm} | R: {right_rpm}")

            try:
                if left and left.is_open:
                    left.write((left_dir  + "\n").encode())
                    left.write((left_rpm  + "\n").encode())
            except serial.SerialException as e:
                print(f"[send_uart] Error escribiendo a ESP: {e}")
                self._ser_left = None

            try:
                if right and right.is_open:
                    right.write((right_dir + "\n").encode())
                    right.write((right_rpm + "\n").encode())
            except serial.SerialException as e:
                print(f"[send_uart] Error escribiendo a ESP_R: {e}")
                self._ser_right = None

    def close(self):
        with self._lock:
            if self._ser_left and self._ser_left.is_open:
                self._ser_left.close()
            if self._ser_right and self._ser_right.is_open:
                self._ser_right.close()
            self._ser_left = None
            self._ser_right = None
        print("🛑 Conexiones seriales cerradas.\n")