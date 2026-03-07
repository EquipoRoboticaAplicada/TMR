import threading
import platform
import serial
import time

class ESP: 

    BAUDRATE = 115200

    def __init__(self):
        
        # --- Puertos seriales ---
        self._ser_left  = None
        self._ser_right = None
        self._lock      = threading.Lock()

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

    def close(self):
        """Cierra las conexiones seriales."""
        if self._ser_left  and self._ser_left.is_open:
            self._ser_left.close()
        if self._ser_right and self._ser_right.is_open:
            self._ser_right.close()
        print("🛑 Conexiones seriales cerradas.")