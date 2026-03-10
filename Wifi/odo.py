import time
import math
import threading
from util import read_odometry

# ================= CLASE PRINCIPAL =================

class RoverOdometry:
    """
    Calcula la odometría del rover en tiempo real mediante modelo diferencial.
    Obtiene rover_state consultando el endpoint /telemetry del servidor Flask
    en la Raspberry Pi vía read_odometry().

    Parámetros:
        PI_IP         : IP de la Raspberry Pi (ej: "192.168.1.100")
        track_width_m : distancia entre ruedas izquierda y derecha (m)
    """

    _EMPTY_STATE = {
        "left_side":  {"seq": 0, "dt_ms": 0.0, "motors": [{"ticks": 0, "m/s": 0.0} for _ in range(3)]},
        "right_side": {"seq": 0, "dt_ms": 0.0, "motors": [{"ticks": 0, "m/s": 0.0} for _ in range(3)]},
        "last_update": 0.0
    }

    def __init__(self, PI_IP: str, track_width_m: float = 0.80):
        self.ODO_URL = f"http://{PI_IP}:5000/telemetry"   # ← endpoint correcto
        self.L       = track_width_m

        # --- Pose del rover (odometría integrada) ---
        self._x     = 0.0   # metros
        self._y     = 0.0   # metros
        self._theta = 0.0   # radianes
        self._pose_lock = threading.Lock()

        self._last_pose_update = time.time()

        # Estado crudo: se actualiza en cada llamada a _update_pose()
        self._state      = dict(self._EMPTY_STATE)
        self._state_lock = threading.Lock()

    # ------------------------------------------------------------------ #
    #  Odometría diferencial                                               #
    # ------------------------------------------------------------------ #

    def _update_pose(self):
        """
        1. Consulta /telemetry para obtener el rover_state más reciente.
        2. Integra la posición usando modelo diferencial:

            v     = (v_l + v_r) / 2          [m/s]
            omega = (v_r - v_l) / L          [rad/s]
            x    += v · cos(θ) · dt
            y    += v · sin(θ) · dt
            θ    += omega · dt
        """
        # 1. Obtener estado fresco del servidor
        new_state = read_odometry(self.ODO_URL)
        if new_state is None:
            return  # Sin datos: conservar pose actual, no integrar

        with self._state_lock:
            self._state = new_state

        # 2. Integrar pose
        now = time.time()
        dt  = now - self._last_pose_update

        if dt <= 0:
            return

        with self._state_lock:
            v_l = self._state["left_side"]["motors"][0]["m/s"]
            v_r = self._state["right_side"]["motors"][0]["m/s"]

        v     = (v_l + v_r) / 2.0
        omega = (v_r - v_l) / self.L

        with self._pose_lock:
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
        side: "left_side" o "right_side"
        motor_index: 0 (adelante), 1 (en medio), 2 (atrás)
        """
        with self._state_lock:
            return self._state[side]["motors"][motor_index]["m/s"]

    def reset_pose(self):
        """Resetea la posición y orientación a cero."""
        with self._pose_lock:
            self._x = 0.0
            self._y = 0.0
            self._theta = 0.0
        self._last_pose_update = time.time()