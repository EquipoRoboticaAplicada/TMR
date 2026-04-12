# imu_reader.py
import serial
import threading
import time

class IMUReader:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        # En la Jetson suele ser /dev/ttyUSB0, /dev/ttyACM0 o un puerto THS
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.running = False
        
        self.heading = 0.0
        self.pitch = 0.0
        self.velocity = 0.0
        self.terrain = "FLAT"
        self.peso = 0.0
        
        self.lock = threading.Lock()
        self.thread = None

    def start(self):
        try:
            self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=1)
            self.running = True
            self.thread = threading.Thread(target=self._read_loop, daemon=True)
            self.thread.start()
            print(f"[IMU] Conectado exitosamente en {self.port}")
        except Exception as e:
            print(f"[IMU] Advertencia: No se pudo conectar en {self.port}: {e}")

    def _read_loop(self):
        while self.running:
            if self.serial_conn and self.serial_conn.in_waiting > 0:
                try:
                    line = self.serial_conn.readline().decode('utf-8').strip()
                    if line:
                        data = line.split(',')
                        if len(data) == 5:
                            with self.lock:
                                self.pitch = float(data[0])
                                self.heading = float(data[1])
                                self.velocity = float(data[2])
                                self.terrain = data[3]
                                self.peso = float(data[4])
                except Exception:
                    pass
            time.sleep(0.01)

    def get_heading(self):
        with self.lock:
            return self.heading

    def stop(self):
        self.running = False
        if self.serial_conn:
            self.serial_conn.close()