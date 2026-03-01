"""
odo_no_rasp.py
Módulo de odometría para rover diferencial de 6 ruedas.
Maneja la conexión serial con los ESP32 y el estado del rover.
"""

import time
import math
import serial
import serial.tools.list_ports
import threading
import platform


# ================= CLASE PRINCIPAL =================

class RoverOdometry:
    """
    Gestiona la conexión serial con los ESP32 y calcula la odometría
    del rover en tiempo real mediante modelo diferencial.

    Parámetros físicos:
        track_width_m : distancia entre ruedas izquierda y derecha (m)
        baudrate      : velocidad de comunicación serial
    """

    BAUDRATE = 115200

    def __init__(self, track_width_m: float = 0.80):
        # --- Parámetros físicos ---
        self.L = track_width_m          # Distancia entre llantas [m]

        # --- Puertos seriales ---
        self._ser_left  = None
        self._ser_right = None
        self._lock      = threading.Lock()

        # --- Estado crudo de los motores (datos recibidos del ESP) ---
        #     Estructura: 2 lados × 3 motores → {"ticks": int, "m/s": float}
        self._rover_state = {
            "left_side":  {
                "seq": 0, "dt_ms": 0,
                "motors": [{"ticks": 0, "m/s": 0.0}] * 3
            },
            "right_side": {
                "seq": 0, "dt_ms": 0,
                "motors": [{"ticks": 0, "m/s": 0.0}] * 3
            },
            "last_update": 0.0
        }

        # --- Pose del rover (odometría integrada) ---
        self._x     = 0.0   # metros
        self._y     = 0.0   # metros
        self._theta = 0.0   # radianes

        self._last_pose_update = time.time()

    # ------------------------------------------------------------------ #
    #  Conexión serial                                                     #
    # ------------------------------------------------------------------ #

    def connect(self):
        """Busca y conecta automáticamente los ESP32 por puerto serial."""
        ports = self._get_available_ports()
        print(f"🔍 Buscando ESPs en: {ports}")

        if not ports:
            print("⚠️  No se encontraron puertos seriales.")
            return

        for port in ports:
            self._try_connect_port(port)

        if self._ser_left is None and self._ser_right is None:
            print("⚠️  No se detectaron ESPs.")

    def _get_available_ports(self) -> list:
        if platform.system() == "Windows":
            return [p.device for p in serial.tools.list_ports.comports()]
        import glob
        return glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')

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
                print(f"⚠️  No se identificó ESP en {port} (cerrando).")
                s.close()

        except serial.SerialException as e:
            print(f"No se pudo abrir {port}: {e}")
        except Exception as e:
            print(f"Error conectando a {port}: {e}")

    def _read_serial_thread(self, ser_obj, side: str):
        """Hilo dedicado de lectura serial para un lado del rover."""
        while ser_obj and ser_obj.is_open:
            try:
                raw = ser_obj.readline()
                if not raw:
                    continue

                line = raw.decode('utf-8', errors='ignore').strip()
                # print(line, '\n') # DEBUG

                if line.startswith("ESP"):
                    self._parse_esp_line(line)
                    self._update_pose()   # integrar posición con cada nueva trama

            except serial.SerialException as e:
                print(f"Error serial ({side}): {e}")
                break
            except Exception as e:
                print(f"Error leyendo serial ({side}): {e}")

    # ------------------------------------------------------------------ #
    #  Parseo de tramas ESP                                                #
    # ------------------------------------------------------------------ #

    def _parse_esp_line(self, line: str):
        """
        Parsea la trama de telemetría.
        Formato: ESP_L/R, seq, dt_ms, ticks0, v0, ticks1, v1, ticks2, v2
        """
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

            with self._lock:
                if header == "ESP_L":
                    self._rover_state["left_side"].update(
                        {"seq": seq, "dt_ms": dt_ms, "motors": m_data}
                    )
                elif header == "ESP_R":
                    self._rover_state["right_side"].update(
                        {"seq": seq, "dt_ms": dt_ms, "motors": m_data}
                    )
                self._rover_state["last_update"] = time.time()

        except ValueError as e:
            print(f"[parse] ValueError en: {repr(line)} → {e}")  
            pass
        except Exception as e:
            print(f"Error parseando: {line} → {e}")

    # ------------------------------------------------------------------ #
    #  Odometría diferencial                                               #
    # ------------------------------------------------------------------ #

    def _update_pose(self):
        """
        Integra la posición del rover usando el modelo diferencial:

            v     = (v_l + v_r) / 2          [m/s]
            omega = (v_r - v_l) / L          [rad/s]
            x    += v · cos(θ) · dt
            y    += v · sin(θ) · dt
            θ    += omega · dt
        """
        now = time.time()
        dt  = now - self._last_pose_update

        if dt <= 0:
            return

        with self._lock:
            v_l = self._rover_state["left_side"]["motors"][0]["m/s"]
            v_r = self._rover_state["right_side"]["motors"][0]["m/s"]
            # print(v_l, '\t')
            # print(v_r, '\n')

        v     = (v_l + v_r) / 2.0
        omega = (v_r - v_l) / self.L

        self._x     += v * math.cos(self._theta) * dt
        self._y     += v * math.sin(self._theta) * dt
        self._theta += omega * dt

        self._last_pose_update = now

    # ------------------------------------------------------------------ #
    #  API pública                                                         #
    # ------------------------------------------------------------------ #

    @property
    def pose(self) -> tuple:
        """Retorna (x, y, theta) — posición en metros y orientación en rad."""
        # print(f"X: {self._x}, Y: {self._y}, Theta: {self._theta}") # DEBUG
        return (self._x, self._y, self._theta)

    @property
    def velocity(self) -> tuple:
        """Retorna (v_lineal [m/s], omega [rad/s]) instantáneos."""
        with self._lock:
            v_l = self._rover_state["left_side"]["motors"][0]["m/s"]
            v_r = self._rover_state["right_side"]["motors"][0]["m/s"]
        return (v_l + v_r) / 2.0, (v_r - v_l) / self.L

    def motor_speed(self, side: str, motor_index: int = 0) -> float:
        """
        Retorna la velocidad en m/s de un motor específico.
        side: "left_side" o "right_side"
        motor_index: 0 (adelante), 1 (en medio), 2 (atrás)
        """
        with self._lock:
            return self._rover_state[side]["motors"][motor_index]["m/s"]

    def reset_pose(self):
        """Resetea la posición y orientación a cero."""
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._last_pose_update = time.time()

    def send_command(self, left_dir: str, left_rpm: str,
                     right_dir: str, right_rpm: str):
        """
        Envía comandos de dirección y velocidad a los ESP.
        Formato: D0/D1 (dirección) y S{valor} (RPM)
        """
        try:
            if self._ser_left and self._ser_left.is_open:
                self._ser_left.write((left_dir  + "\n").encode())
                self._ser_left.write((left_rpm  + "\n").encode())

            if self._ser_right and self._ser_right.is_open:
                self._ser_right.write((right_dir + "\n").encode())
                self._ser_right.write((right_rpm + "\n").encode())

        except Exception as e:
            print(f"Error enviando comando UART: {e}")

    def close(self):
        """Cierra las conexiones seriales."""
        if self._ser_left  and self._ser_left.is_open:
            self._ser_left.close()
        if self._ser_right and self._ser_right.is_open:
            self._ser_right.close()
        print("🛑 Conexiones seriales cerradas.")