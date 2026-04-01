import time
import math
import threading
from connect import ESP


class RoverOdometry:
    """
    Calcula la odometría del rover en tiempo real mediante modelo diferencial.
    Obtiene rover_state directamente desde la instancia ESP (local).

    Parámetros:
        esp           : instancia de ESP (connect.py)
        track_width_m : distancia entre ruedas izquierda y derecha [m]
        update_hz     : frecuencia de actualización del hilo de odometría [Hz]
    """

    _EMPTY_STATE = {
        "left_side":  {"seq": 0, "motors": [{"rpm": 0.0, "m/s": 0.0} for _ in range(3)]},
        "right_side": {"seq": 0, "motors": [{"rpm": 0.0, "m/s": 0.0} for _ in range(3)]},
        "last_update": 0.0
    }

    def __init__(self, esp: ESP, track_width_m: float = 0.80, update_hz: float = 50.0):
        self.esp    = esp
        self.L      = track_width_m
        self._dt_target = 1.0 / update_hz

        # --- Pose integrada ---
        self._x     = 0.0   # metros
        self._y     = 0.0   # metros
        self._theta = 0.0   # radianes
        self._pose_lock = threading.Lock()

        self._last_pose_update = time.time()

        self._state      = dict(self._EMPTY_STATE)
        self._state_lock = threading.Lock()

        # Hilo de actualización periódica
        self._stop_event = threading.Event()
        self._thread     = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    # ------------------------------------------------------------------ #
    #  Hilo interno                                                        #
    # ------------------------------------------------------------------ #

    def _loop(self):
        """Llama a _update_pose() a la frecuencia configurada."""
        while not self._stop_event.is_set():
            t0 = time.time()
            self._update_pose()
            elapsed = time.time() - t0
            sleep   = self._dt_target - elapsed
            if sleep > 0:
                time.sleep(sleep)

    # ------------------------------------------------------------------ #
    #  Odometría diferencial                                               #
    # ------------------------------------------------------------------ #

    def _update_pose(self):
        """
        1. Obtiene el rover_state más reciente desde ESP.
        2. Integra la posición con modelo diferencial:

            v     = (v_l + v_r) / 2      [m/s]
            omega = (v_r - v_l) / L      [rad/s]
            x    += v · cos(θ) · dt
            y    += v · sin(θ) · dt
            θ    += omega · dt
        """
        new_state = self.esp.get_rover_state()
        with self._state_lock:
            self._state = new_state

        now = time.time()
        dt  = now - self._last_pose_update
        if dt <= 0:
            return
        self._last_pose_update = now

        with self._state_lock:
            v_l = self._state["left_side"]["motors"][0]["m/s"]
            v_r = self._state["right_side"]["motors"][0]["m/s"]

        v     = (v_l + v_r) / 2.0
        omega = (v_r - v_l) / self.L

        with self._pose_lock:
            self._x     += v * math.cos(self._theta) * dt
            self._y     += v * math.sin(self._theta) * dt
            self._theta += omega * dt

    # ------------------------------------------------------------------ #
    #  API pública                                                         #
    # ------------------------------------------------------------------ #

    @property
    def pose(self) -> tuple:
        """Retorna (x, y, theta) — posición en metros y orientación en rad."""
        with self._pose_lock:
            return (self._x, self._y, self._theta)

    @property
    def velocity(self) -> tuple:
        """Retorna (v_lineal [m/s], omega [rad/s]) instantáneos."""
        with self._state_lock:
            v_l = self._state["left_side"]["motors"][0]["m/s"]
            v_r = self._state["right_side"]["motors"][0]["m/s"]
        return (v_l + v_r) / 2.0, (v_r - v_l) / self.L

    def motor_speed(self, side: str, motor_index: int = 0) -> float:
        """
        Retorna la velocidad en m/s de un motor específico.
        side        : "left_side" o "right_side"
        motor_index : 0 (adelante), 1 (en medio), 2 (atrás)
        """
        with self._state_lock:
            return self._state[side]["motors"][motor_index]["m/s"]

    def reset_pose(self):
        """Resetea la posición y orientación a cero."""
        with self._pose_lock:
            self._x     = 0.0
            self._y     = 0.0
            self._theta = 0.0
        self._last_pose_update = time.time()

    def stop(self):
        """Detiene el hilo de odometría."""
        self._stop_event.set()
        self._thread.join(timeout=1.0)