import requests
import time
import threading


class Receiver:
    """
    Corre en la PC. Hace polling al endpoint /pose del servidor Flask
    en la Jetson y expone los datos con la misma interfaz que RoverOdometry,
    para que map.py pueda usarlos sin cambios.

    Interfaz pública:
        .pose     → (x, y, theta)   igual que RoverOdometry.pose
        .velocity → (v, omega)      igual que RoverOdometry.velocity
    """

    def __init__(self, PI_IP: str, poll_hz: float = 20.0):
        self._url           = f"http://{PI_IP}:5000/odometry"
        self._poll_interval = 1.0 / poll_hz
        self._lock          = threading.Lock()
        self._stop_event    = threading.Event()
        self._thread        = threading.Thread(target=self._run, daemon=True)

        # Estado interno — mismas claves que publica /pose
        self._x     = 0.0
        self._y     = 0.0
        self._theta = 0.0
        self._v     = 0.0
        self._omega = 0.0

    def start(self):
        self._thread.start()
        return self

    # ------------------------------------------------------------------ #
    #  Hilo de polling                                                     #
    # ------------------------------------------------------------------ #

    def _run(self):
        session = requests.Session()
        while not self._stop_event.is_set():
            t0 = time.time()
            try:
                r    = session.get(self._url, timeout=1.0)
                r.raise_for_status()
                data = r.json()
                with self._lock:
                    self._x     = data.get("x",     0.0)
                    self._y     = data.get("y",     0.0)
                    self._theta = data.get("theta", 0.0)
                    self._v     = data.get("v",     0.0)
                    self._omega = data.get("omega", 0.0)
            except requests.exceptions.Timeout:
                print("[Receiver] Timeout al conectar con el servidor.")
            except requests.exceptions.ConnectionError:
                print("[Receiver] No se pudo conectar con el servidor.")
            except Exception as e:
                print(f"[Receiver] Error inesperado: {e}")

            # Regulación de frecuencia
            elapsed = time.time() - t0
            sleep   = self._poll_interval - elapsed
            if sleep > 0:
                time.sleep(sleep)

        session.close()

    def reset_pose(self):
        try:
            requests.post(f"{self._url}/reset", timeout=1.0)
        except Exception as e:
            print(f"[Receiver] Error al resetear pose: {e}")

    # ------------------------------------------------------------------ #
    #  API pública — misma interfaz que RoverOdometry                     #
    # ------------------------------------------------------------------ #

    @property
    def pose(self) -> tuple:
        """Retorna (x, y, theta) — metros y radianes."""
        with self._lock:
            return (self._x, self._y, self._theta)

    @property
    def velocity(self) -> tuple:
        """Retorna (v [m/s], omega [rad/s])."""
        with self._lock:
            return (self._v, self._omega)

    def stop(self):
        self._stop_event.set()
        self._thread.join(timeout=1.0)