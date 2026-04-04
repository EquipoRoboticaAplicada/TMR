import requests
import threading
import time


class Receiver:
    """
    Corre en la PC/laptop. Hace polling al endpoint /odometry del servidor
    Flask en la Jetson para obtener pose y velocidad en tiempo real.

    Interfaz pública:
        .pose     → (x, y, theta)   metros / radianes
        .velocity → (v, omega)      m/s / rad/s
        .is_stale → bool            True si no hay datos frescos
        .reset_pose()               envía reset de odometría a la Jetson
    """

    STALE_TIMEOUT = 3.0
    MAX_BACKOFF   = 30.0

    def __init__(self, PI_IP: str, poll_hz: float = 20.0):
        self._base_url      = f"http://{PI_IP}:5000"
        self._pose_url      = f"{self._base_url}/odometry"
        self._reset_url     = f"{self._base_url}/pose/reset"
        self._poll_interval = 1.0 / poll_hz

        self._lock        = threading.Lock()
        self._x           = 0.0
        self._y           = 0.0
        self._theta       = 0.0
        self._v           = 0.0
        self._omega       = 0.0
        self._last_update = 0.0

        self._stop_event  = threading.Event()
        self._thread      = threading.Thread(target=self._run, daemon=True)

    # ------------------------------------------------------------------ #

    def start(self):
        self._thread.start()
        return self

    def stop(self):
        self._stop_event.set()
        self._thread.join(timeout=2.0)

    # ------------------------------------------------------------------ #
    #  Hilo de polling                                                     #
    # ------------------------------------------------------------------ #

    def _run(self):
        session = requests.Session()
        backoff = 1.0

        while not self._stop_event.is_set():
            t0 = time.time()
            try:
                r = session.get(self._pose_url, timeout=1.0)
                r.raise_for_status()
                data = r.json()

                with self._lock:
                    self._x           = _safe_float(data.get("x"),     0.0)
                    self._y           = _safe_float(data.get("y"),     0.0)
                    self._theta       = _safe_float(data.get("theta"), 0.0)
                    self._v           = _safe_float(data.get("v"),     0.0)
                    self._omega       = _safe_float(data.get("omega"), 0.0)
                    self._last_update = time.time()
                backoff = 1.0

            except requests.exceptions.Timeout:
                print("[Receiver] Timeout — servidor no responde.")

            except requests.exceptions.ConnectionError:
                print(f"[Receiver] Sin conexión. Reintentando en {backoff:.0f}s...")
                time.sleep(backoff)
                backoff = min(backoff * 2, self.MAX_BACKOFF)
                continue

            except requests.exceptions.HTTPError as e:
                print(f"[Receiver] HTTP {e.response.status_code}.")

            except Exception as e:
                print(f"[Receiver] Error inesperado: {e}")

            elapsed = time.time() - t0
            sleep   = self._poll_interval - elapsed
            if sleep > 0:
                time.sleep(sleep)

        session.close()

    # ------------------------------------------------------------------ #
    #  API pública                                                         #
    # ------------------------------------------------------------------ #

    @property
    def pose(self) -> tuple:
        with self._lock:
            return (self._x, self._y, self._theta)

    @property
    def velocity(self) -> tuple:
        with self._lock:
            return (self._v, self._omega)

    @property
    def is_stale(self) -> bool:
        with self._lock:
            return (time.time() - self._last_update) > self.STALE_TIMEOUT

    def reset_pose(self):
        try:
        
            r = requests.post(self._reset_url, timeout=2.0)
            r.raise_for_status()

            with self._lock:
                self._x=0.0
                self._y=0.0
                self._theta=0.0
                self._v=0.0
                self._omega=0.0
            print("[Receiver] Pose reseteada.")
        except requests.exceptions.HTTPError as e:
            print(f"[Receiver] Reset rechazado ({e.response.status_code}).")
        except Exception as e:
            print(f"[Receiver] Error inesperado en reset: {e}")
        


# ------------------------------------------------------------------ #

def _safe_float(val, default: float = 0.0) -> float:
    try:
        return float(val)
    except (TypeError, ValueError):
        return default