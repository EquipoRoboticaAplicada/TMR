import threading
import platform
import serial
import serial.tools.list_ports
import time
import copy

class ESP: 

    BAUDRATE = 115200

    def __init__(self):
        
        # Puertos seriales 
        self._ser_left      = None # Motores izquierda
        self._ser_right     = None # Motores derecha
        self._ser_sensores  = None # Sensores
        self._lock          = threading.Lock()

        # Estado del rover (odometría)
        self._rover_state = {
            "left_side":  {"seq": 0, "motors": [{"rpm": 0.0, "m/s": 0.0} for _ in range(2)]},
            "right_side": {"seq": 0, "motors": [{"rpm": 0.0, "m/s": 0.0} for _ in range(2)]},
            "last_update": 0.0
        }

        # Estado del rover (sensores)
        self._sensor_state = {
            "sensores": {"pitch": 0.0, "heading": 0.0, "velocity": 0.0, "terrain_text": None, "peso": 0.0},
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
            s = serial.Serial(
                port,
                self.BAUDRATE,
                timeout  = 0.5,
                rtscts   = False,
                dsrdtr   = False,
            )
            time.sleep(0.1)
            s.reset_input_buffer()
            print(f"Probando {port}...")

            deadline = time.time() + 6.0
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

                    elif line.startswith("sensores") and self._ser_sensores is None:
                        self._ser_sensores = s
                        print(f"✅ SENSORES detectados en {port}")
                        threading.Thread(
                            target=self._read_serial_thread,
                            args=(s, "sensores"),
                            daemon=True
                        ).start()
                        identified = True
                        break
                time.sleep(0.05)

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
                    self._parse_esp_line_m(line)
                elif line.startswith("sensores"):
                    self._parse_esp_line_s(line)

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
                    elif side == "sensores" and self._ser_sensores is ser_obj:
                        self._ser_sensores = None

                break

            except Exception as e:
                print(f"Error inesperado leyendo serial ({side}): {e}")
                break

    def _parse_esp_line_s(self, line: str):
        """
        Formato: sensores, pitch, heading, velocity, terrain_text, peso
        """

        if not line:
            return

        try:
            try:
                data = line.split(",")
                if len(data) != 6:   
                    return
                header = data[0]
                _, p, h, v, t, w = data
                
                if header != "sensores":
                    ValueError(f"Header desconocido: {header}")

            except (ValueError, IndexError):
                    print("[_parse_esp_line_s] Error.\n")
                    return

            with self._lock:
                pitch = float(p)
                heading = float(h)
                velocity = float(v)
                terrain_text = t
                peso = float(w)
                
                self._sensor_state["sensores"].update(
                    {"pitch": pitch, "heading": heading, "velocity": velocity, "terrain_text": terrain_text, "peso": peso}
                )
                print(f"[Sensores] Pitch: {pitch:.2f}°, Heading: {heading:.2f}°, Velocidad: {velocity:.2f} m/s, Terreno: {terrain_text}, Peso: {peso:.2f} kg\n") # DEBUG
                self._sensor_state["last_update"] = time.time()
        except ValueError as e:
            print(f"[parse] ValueError en: {repr(line)} → {e}")
        except Exception as e:
            print(f"[parse] Error inesperado: {repr(line)} → {e}")

    def _parse_esp_line_m(self, line: str):
        """
        Formato: ESP_L/R, seq, rpm0, v0, rpm1, v1, rpm2, v2
        """
        if not line:
            return

        try:
            # print(line) # DEBUG
            parts = line.strip().split(',')

            if len(parts) != 8:
                return

            try:
                header = parts[0]
                seq    = int(parts[1])
                m_data = [
                    {"rpm": float(parts[2]), "m/s": float(parts[3])},
                   # {"rpm": float(parts[4]), "m/s": float(parts[5])},
                    # {"rpm": float(parts[6]), "m/s": float(parts[7])}, # Solo 2 motores por lado, no 3. Encoders muertos. 
                ]

            except (ValueError, IndexError):
                print("[_parse_esp_line_m] Error.\n")
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
            # print((self._rover_state)) # DEBUG
            return copy.deepcopy(self._rover_state)

    def get_sensor_state(self) -> dict:
        """
        Devuelve una copia segura del sensor_state actual (IMU + sensores).
        Usar siempre esta función desde IMU.py, nunca acceder a _sensor_state directamente.
        """
        with self._lock:
            return copy.deepcopy(self._sensor_state)

    def send_uart(self, left_dir, left_rpm, right_dir, right_rpm):
        for val in (left_dir, left_rpm, right_dir, right_rpm):
            assert isinstance(val, str), f"Tipo inválido: {val!r}"

        with self._lock:
            left  = self._ser_left
            right = self._ser_right
            # print(f"Enviando UART → L: {left_rpm} | R: {right_rpm}") # DEBUG

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

    def wait_for_peso_change(self, timeout: float = 10.0, poll_interval: float = 0.05) -> bool:
        """
        Bloquea el hilo llamante hasta que el valor de 'peso' en _sensor_state cambie.
        Retorna True si detectó un cambio, False si se agotó el timeout.
        """
        with self._lock:
            initial_peso = self._sensor_state["sensores"]["peso"]

        deadline = time.time() + timeout
        while time.time() < deadline:
            time.sleep(poll_interval)
            with self._lock:
                current_peso = self._sensor_state["sensores"]["peso"]
            if current_peso < initial_peso*0.9 or current_peso > initial_peso*1.1:  
                return True

        print("[wait_for_peso_change] Timeout: no se detectó cambio en 'peso'.\n")
        return False

    def act_arm(self):
        try: 
            with self._lock:
                if self._ser_sensores and self._ser_sensores.is_open:
                    self._ser_sensores.write("B0\n".encode())
        except serial.SerialException as e:
            print(f"[act_arm] Error escribiendo a ESP_S: {e}")

    def close(self):
        with self._lock:
            if self._ser_left and self._ser_left.is_open:
                self._ser_left.close()
            if self._ser_right and self._ser_right.is_open:
                self._ser_right.close()
            if self._ser_sensores and self._ser_sensores.is_open:
                self._ser_sensores.close()
            self._ser_left = None
            self._ser_right = None
            self._ser_sensores = None
        print("🛑 Conexiones seriales cerradas.\n")