import requests
import time
import threading

# ---------------------------
# Hilo para recibos HTTP
# ---------------------------
class Receiver:
    def __init__(self, PI_IP, poll_hz: float = 20.0):
        self.telemetry      = f"http://{PI_IP}:5000/telemetry"
        self.odometry       = f"http://{PI_IP}:5000/odometry"
        self.lock           = threading.Lock()
        self.stop_event     = threading.Event()
        self.thread         = threading.Thread(target=self._run, daemon=True)
        self._poll_interval = 1.0 / poll_hz

        self._rover_state = {
            "left_side":  {"seq": 0, "motors": [{"rpm": 0.0, "m/s": 0.0} for _ in range(3)]},
            "right_side": {"seq": 0, "motors": [{"rpm": 0.0, "m/s": 0.0} for _ in range(3)]},
            "last_update": 0.0
        }

    def start(self):
        self.thread.start()
        return self

    @property
    def rover_state(self) -> dict:
        import copy
        with self.lock:
            return copy.deepcopy(self._rover_state)

    def read_odometry(self, timeout: float = 2) -> dict | None:
        try:
            r = requests.get(self.telemetry, timeout=timeout)
            r.raise_for_status()
            data = r.json()
            return data.get("rover_state")
        except requests.exceptions.Timeout:
            print("[read_odometry] Timeout al conectar con el servidor.")
        except requests.exceptions.ConnectionError:
            print("[read_odometry] No se pudo conectar con el servidor.")
        except Exception as e:
            print(f"[read_odometry] Error inesperado: {e}")
        return None

    def _run(self):
        while not self.stop_event.is_set():
            state = self.read_odometry()
            if state is not None:
                with self.lock:
                    self._rover_state = state
            time.sleep(self._poll_interval)

    def stop(self):
        self.stop_event.set()
        self.thread.join(timeout=1.0)