import requests
import threading
import time

import cv2
import numpy as np


class Receiver:
    """
    Corre en la PC. Hace polling al endpoint /odometry del servidor Flask
    en la Jetson para telemetría de pose, y consume el stream MJPEG de
    /video_feed en un hilo separado.

    Interfaz pública:
        .pose        → (x, y, theta)      metros / radianes
        .velocity    → (v, omega)         m/s / rad/s
        .get_frame() → np.ndarray | None  último frame BGR, o None si no hay
        .is_stale    → bool               True si no hay datos frescos de pose
    """

    STALE_TIMEOUT = 3.0    # segundos sin respuesta de pose = conexión perdida
    MAX_BACKOFF   = 30.0   # tope de backoff exponencial en errores de conexión

    def __init__(self, PI_IP: str, poll_hz: float = 20.0):
        self._base_url      = f"http://{PI_IP}:5000"
        self._pose_url      = f"{self._base_url}/odometry"
        self._reset_url     = f"{self._base_url}/pose/reset"   # ruta independiente
        self._video_url     = f"{self._base_url}/video_feed"
        self._poll_interval = 1.0 / poll_hz

        # --- Pose ---
        self._pose_lock   = threading.Lock()
        self._x           = 0.0
        self._y           = 0.0
        self._theta       = 0.0
        self._v           = 0.0
        self._omega       = 0.0
        self._last_update = 0.0

        # --- Video ---
        self._frame_lock  = threading.Lock()
        self._last_frame: np.ndarray | None = None

        # --- Control de hilos ---
        self._stop_event   = threading.Event()
        self._pose_thread  = threading.Thread(target=self._run_pose,  daemon=True)
        self._video_thread = threading.Thread(target=self._run_video, daemon=True)

    # ------------------------------------------------------------------ #
    #  Arranque y parada                                                   #
    # ------------------------------------------------------------------ #

    def start(self):
        self._pose_thread.start()
        self._video_thread.start()
        return self

    def stop(self):
        self._stop_event.set()
        self._pose_thread.join(timeout=2.0)
        self._video_thread.join(timeout=2.0)

    # ------------------------------------------------------------------ #
    #  Hilo de pose (polling HTTP)                                         #
    # ------------------------------------------------------------------ #

    def _run_pose(self):
        session = requests.Session()
        backoff = 1.0
        while not self._stop_event.is_set():
            t0 = time.time()
            try:
                r = session.get(self._pose_url, timeout=1.0)
                r.raise_for_status()
                data = r.json()
                with self._pose_lock:
                    self._x           = _safe_float(data.get("x"),     0.0)
                    self._y           = _safe_float(data.get("y"),     0.0)
                    self._theta       = _safe_float(data.get("theta"), 0.0)
                    self._v           = _safe_float(data.get("v"),     0.0)
                    self._omega       = _safe_float(data.get("omega"), 0.0)
                    self._last_update = time.time()
                backoff = 1.0

            except requests.exceptions.Timeout:
                print("[Receiver/pose] Timeout — servidor no responde.")

            except requests.exceptions.ConnectionError:
                print(f"[Receiver/pose] Sin conexión. Reintentando en {backoff:.0f}s...")
                time.sleep(backoff)
                backoff = min(backoff * 2, self.MAX_BACKOFF)
                continue

            except requests.exceptions.HTTPError as e:
                print(f"[Receiver/pose] HTTP {e.response.status_code}.")

            except Exception as e:
                print(f"[Receiver/pose] Error inesperado: {e}")

            elapsed = time.time() - t0
            sleep   = self._poll_interval - elapsed
            if sleep > 0:
                time.sleep(sleep)

        session.close()

    # ------------------------------------------------------------------ #
    #  Hilo de video (MJPEG streaming)                                     #
    # ------------------------------------------------------------------ #

    def _run_video(self):
        """
        Consume el stream MJPEG de /video_feed.
        Cada parte del multipart tiene la forma:
            --frame\r\n
            Content-Type: image/jpeg\r\n\r\n
            <bytes JPEG>
            \r\n
        Se acumulan chunks hasta encontrar el marcador de fin de frame,
        se decodifica con OpenCV y se expone como el último frame disponible.
        """
        backoff = 1.0
        while not self._stop_event.is_set():
            try:
                with requests.get(self._video_url, stream=True, timeout=5.0) as resp:
                    resp.raise_for_status()
                    backoff = 1.0
                    buf = b""

                    for chunk in resp.iter_content(chunk_size=4096):
                        if self._stop_event.is_set():
                            break

                        buf += chunk

                        # Buscar un frame JPEG completo entre SOI (FFD8) y EOI (FFD9)
                        soi = buf.find(b'\xff\xd8')
                        eoi = buf.find(b'\xff\xd9')

                        if soi == -1 or eoi == -1 or eoi <= soi:
                            # Frame incompleto — seguir acumulando.
                            # Descartar basura previa al SOI para no crecer sin límite.
                            if soi > 0:
                                buf = buf[soi:]
                            elif soi == -1:
                                buf = b""
                            continue

                        jpg_bytes = buf[soi: eoi + 2]
                        buf = buf[eoi + 2:]       # descartar el frame procesado

                        arr   = np.frombuffer(jpg_bytes, dtype=np.uint8)
                        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                        if frame is None:
                            continue              # JPEG corrupto, ignorar

                        with self._frame_lock:
                            self._last_frame = frame

            except requests.exceptions.ConnectionError:
                print(f"[Receiver/video] Sin conexión al stream. Reintentando en {backoff:.0f}s...")
                time.sleep(backoff)
                backoff = min(backoff * 2, self.MAX_BACKOFF)

            except requests.exceptions.Timeout:
                print("[Receiver/video] Timeout en el stream de video.")
                time.sleep(backoff)
                backoff = min(backoff * 2, self.MAX_BACKOFF)

            except requests.exceptions.HTTPError as e:
                print(f"[Receiver/video] HTTP {e.response.status_code} en /video_feed.")
                time.sleep(backoff)
                backoff = min(backoff * 2, self.MAX_BACKOFF)

            except Exception as e:
                print(f"[Receiver/video] Error inesperado: {e}")
                time.sleep(1.0)

    # ------------------------------------------------------------------ #
    #  API pública                                                         #
    # ------------------------------------------------------------------ #

    @property
    def pose(self) -> tuple:
        """Retorna (x, y, theta) — metros y radianes."""
        with self._pose_lock:
            return (self._x, self._y, self._theta)

    @property
    def velocity(self) -> tuple:
        """Retorna (v [m/s], omega [rad/s])."""
        with self._pose_lock:
            return (self._v, self._omega)

    @property
    def is_stale(self) -> bool:
        """True si no se recibió ningún dato de pose en los últimos STALE_TIMEOUT s."""
        with self._pose_lock:
            return (time.time() - self._last_update) > self.STALE_TIMEOUT

    def get_frame(self) -> np.ndarray | None:
        """
        Retorna una copia del último frame BGR recibido, o None si
        el stream aún no ha enviado ningún frame o está desconectado.
        """
        with self._frame_lock:
            if self._last_frame is None:
                return None
            return self._last_frame.copy()

    def reset_pose(self):
        """Envía comando de reset de odometría a la Jetson."""
        try:
            r = requests.post(self._reset_url, timeout=1.0)
            r.raise_for_status()
        except requests.exceptions.HTTPError as e:
            print(f"[Receiver] Reset rechazado ({e.response.status_code}). ¿Existe /pose/reset en Flask?")
        except Exception as e:
            print(f"[Receiver] Error al resetear pose: {e}")


# ------------------------------------------------------------------ #
#  Helpers                                                             #
# ------------------------------------------------------------------ #

def _safe_float(val, default: float = 0.0) -> float:
    """Convierte a float de forma segura; retorna default si val es None o inválido."""
    try:
        return float(val)
    except (TypeError, ValueError):
        return default