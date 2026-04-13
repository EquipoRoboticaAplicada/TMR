import threading
import time

# ===== CONSTANTES DE CONTROL =====
KP      = 0.5   # Constante proporcional. Súbela si corrige muy lento, bájala si oscila.
MAX_RPM = 80
MIN_RPM = 20

# ===== CONFIGURACIÓN DEL MAPA =====
GRID_SIZE = 50
CELL_SIZE = 10


class IMU:
    """
    Módulo IMU que consume los datos de sensores ya parseados por ESP (connect.py).
    Ya no abre ni gestiona ningún puerto serial; sólo lee de ESP.get_sensor_state().

    Uso:
        from connect import ESP
        from IMU import IMU

        esp = ESP()
        esp.connect()

        imu = IMU(esp)
        imu.start()          # arranca el hilo de polling/mapping
        ...
        imu.stop()
    """

    def __init__(self, esp, poll_hz: float = 30.0):
        """
        Parameters
        ----------
        esp      : instancia de ESP (connect.py) ya conectada.
        poll_hz  : frecuencia de muestreo del hilo interno (Hz).
        """
        self._esp      = esp
        self._interval = 1.0 / poll_hz
        self._running  = False
        self._lock     = threading.Lock()
        self._thread   = None

        # ===== ESTADO SENSOR =====
        self._pitch        = 0.0
        self._heading      = 0.0
        self._velocity     = 0.0
        self._terrain_text = "NONE"
        self._peso         = 0.0

        # ===== MAPA TOPOGRÁFICO =====
        self.terrain_map     = [[None for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
        self._mapping_active = False
        self._row            = 0
        self._col            = 0
        self._direction      = 1

    # ------------------------------------------------------------------
    # Ciclo de vida
    # ------------------------------------------------------------------

    def start(self):
        """Arranca el hilo de polling que mantiene los datos sincronizados con ESP."""
        if self._thread and self._thread.is_alive():
            return
        self._running = True
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

    def stop(self):
        """Detiene el hilo de polling de forma limpia."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)

    # ------------------------------------------------------------------
    # Hilo interno (reemplaza el antiguo serial_thread)
    # ------------------------------------------------------------------

    def _poll_loop(self):
        """
        Lee ESP.get_sensor_state() a la frecuencia indicada y actualiza
        el estado interno + el mapa topográfico si el mapping está activo.
        """
        while self._running:
            try:
                state    = self._esp.get_sensor_state()
                sensores = state.get("sensores", {})

                pitch        = sensores.get("pitch",        0.0)
                heading      = sensores.get("heading",      0.0)
                velocity     = sensores.get("velocity",     0.0)
                terrain_text = sensores.get("terrain_text", "NONE")
                peso         = sensores.get("peso",         0.0)

                with self._lock:
                    self._pitch        = pitch
                    self._heading      = heading
                    self._velocity     = velocity
                    self._terrain_text = terrain_text
                    self._peso         = peso

                    if self._mapping_active and self._row < GRID_SIZE:
                        val = self._map_pitch(pitch)
                        self.terrain_map[self._row][self._col] = val

                        self._col += self._direction
                        if self._col >= GRID_SIZE or self._col < 0:
                            self._row      += 1
                            self._direction *= -1
                            self._col       += self._direction

            except Exception as e:
                print(f"[IMU._poll_loop] Error inesperado: {e}")

            time.sleep(self._interval)

    # ------------------------------------------------------------------
    # Control del mapa
    # ------------------------------------------------------------------

    def start_mapping(self):
        """Activa el llenado del mapa topográfico."""
        with self._lock:
            self._mapping_active = True

    def stop_mapping(self):
        """Pausa el llenado del mapa topográfico."""
        with self._lock:
            self._mapping_active = False

    def reset_mapping(self):
        """Borra el mapa y reinicia la posición de escaneo."""
        with self._lock:
            self.terrain_map = [[None for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
            self._row, self._col, self._direction = 0, 0, 1

    # ------------------------------------------------------------------
    # API pública de datos
    # ------------------------------------------------------------------

    def obtener_datos_imu(self):
        """Devuelve (pitch, heading, velocity, terrain_text, peso) de forma segura."""
        with self._lock:
            return (
                self._pitch,
                self._heading,
                self._velocity,
                self._terrain_text,
                self._peso,
            )

    def calcular_correccion_motores(self, target_heading: float, base_rpm: int = 40):
        """
        Calcula los RPM para los motores izquierdo y derecho con un controlador P.

        Parameters
        ----------
        target_heading : heading deseado en grados (0-360).
        base_rpm       : velocidad base de ambas llantas.

        Returns
        -------
        (rpm_izq, rpm_der, error_grados)
        """
        with self._lock:
            current_heading = self._heading

        # Diferencia más corta en [-180, 180]
        error_grados = (target_heading - current_heading + 180) % 360 - 180

        correccion = int(KP * error_grados)

        rpm_izq = max(MIN_RPM, min(MAX_RPM, base_rpm + correccion))
        rpm_der = max(MIN_RPM, min(MAX_RPM, base_rpm - correccion))

        return rpm_izq, rpm_der, error_grados

    # ------------------------------------------------------------------
    # Helpers estáticos (antes funciones sueltas de módulo)
    # ------------------------------------------------------------------

    @staticmethod
    def _limit_pitch(p: float) -> float:
        return max(-40.0, min(40.0, p))

    @staticmethod
    def _map_pitch(p: float) -> int:
        p = IMU._limit_pitch(p)
        return int(max(0, min(100, 50 + (p / 40) * 50)))

    @staticmethod
    def pitch_to_color(v: int) -> tuple:
        """Convierte un valor [0, 100] en un color RGB (rojo=plano, verde=inclinado)."""
        if v <= 50:
            r = 255
            g = int(255 * (v / 50))
        else:
            r = int(255 * (1 - (v - 50) / 50))
            g = 255
        return (r, g, 0)