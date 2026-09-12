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
        1. Obtiene el rover_state mas reciente desde ESP.
        2. Integra la posicion con modelo diferencial (punto medio).
        """
        new_state = self.esp.get_rover_state()

        with self._state_lock:
            self._state = new_state
            m_l = self._state["left_side"]["motors"]
            m_r = self._state["right_side"]["motors"]

            # Promedio ponderado si hay 3 motores, fallback al primero disponible
            if len(m_l) >= 3:
                v_l = m_l[0]["m/s"] * 0.1 + m_l[1]["m/s"] * 0.8 + m_l[2]["m/s"] * 0.1
            elif len(m_l) > 0:
                v_l = m_l[0]["m/s"]
            else:
                v_l = 0.0

            if len(m_r) >= 3:
                v_r = m_r[0]["m/s"] * 0.1 + m_r[1]["m/s"] * 0.8 + m_r[2]["m/s"] * 0.1
            elif len(m_r) > 0:
                v_r = m_r[0]["m/s"]
            else:
                v_r = 0.0

        now = time.time()
        with self._pose_lock:
            dt = now - self._last_pose_update
            if dt <= 0:
                return
            self._last_pose_update = now

            v = (v_l + v_r) / 2.0
            omega = (v_r - v_l) / self.L

            delta_theta = omega * dt
            theta_mid = self._theta + delta_theta / 2.0
            self._x     += v * math.cos(theta_mid) * dt
            self._y     += v * math.sin(theta_mid) * dt
            self._theta = math.atan2(
                math.sin(self._theta + delta_theta),
                math.cos(self._theta + delta_theta)
            )

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
        """Retorna (v_lineal [m/s], omega [rad/s]) instantaneos."""
        with self._state_lock:
            m_l = self._state["left_side"]["motors"]
            m_r = self._state["right_side"]["motors"]
            v_l = m_l[0]["m/s"] if len(m_l) > 0 else 0.0
            v_r = m_r[0]["m/s"] if len(m_r) > 0 else 0.0
        return (v_l + v_r) / 2.0, (v_r - v_l) / self.L

    def motor_speed(self, side: str, motor_index: int = 0) -> float:
        """
        Retorna la velocidad en m/s de un motor especifico.
        side        : "left_side" o "right_side"
        motor_index : 0 (adelante), 1 (en medio), 2 (atras)
        """
        with self._state_lock:
            motors = self._state[side]["motors"]
            if 0 <= motor_index < len(motors):
                return motors[motor_index]["m/s"]
            return 0.0

    def reset_pose(self):
        with self._pose_lock:
            self._x = self._y = self._theta = 0.0
            self._last_pose_update = time.time()   # ← bajo el mismo lock

    def stop(self):
        try:
            self.esp.send_uart("0", "0")
        except Exception:
            pass
        self._stop_event.set()
        self._thread.join(timeout=1.0)