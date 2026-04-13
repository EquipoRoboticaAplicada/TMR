# command.py
import math
import time


class HeadingKalman:
    """
    Filtro de Kalman 1-D para fusión de orientación.

    Estado:     x = θ_fused  (radianes)
    Predicción: usa ω_odo * dt  (velocidad angular de encoders)
    Corrección: usa θ_imu_aligned  (IMU alineado al sistema de la odometría)

    Ruidos ajustables:
        Q  — ruido de proceso: cuánto confiamos en la odometría entre ciclos.
             Sube Q si los encoders derivan rápido (suelo irregular, slippage).
        R  — ruido de medición: cuánto confiamos en el IMU.
             Sube R si el IMU tiene vibraciones o interferencia magnética.
    """

    def __init__(self, q: float = 0.005, r: float = 0.05):
        self.Q      = q     # varianza ruido proceso   [rad²/ciclo]
        self.R      = r     # varianza ruido medición  [rad²]
        self.x      = None  # estado estimado (None = sin inicializar)
        self.P      = 1.0   # covarianza del error
        self._last_t = None

    def init(self, theta_0: float):
        """Inicializa el filtro con la orientación actual de la odometría."""
        self.x       = theta_0
        self.P       = 1.0
        self._last_t = time.time()

    def update(self, omega_odo: float, theta_imu: float) -> float:
        """
        Ejecuta un ciclo predict → correct.

        Parameters
        ----------
        omega_odo : velocidad angular [rad/s] de encoders  (rover_odometry.velocity[1])
        theta_imu : orientación del IMU alineada al sistema de la odometría [rad]

        Returns
        -------
        θ estimado fusionado [rad]
        """
        now = time.time()
        dt  = now - self._last_t
        self._last_t = now

        # ── PREDICT ─────────────────────────────────────────────────────
        # Propaga el ángulo con la velocidad angular de los encoders.
        x_pred = normalize_angle(self.x + omega_odo * dt)
        P_pred = self.P + self.Q

        # ── CORRECT ─────────────────────────────────────────────────────
        # Innovación: diferencia angular más corta entre IMU y predicción.
        innov = normalize_angle(theta_imu - x_pred)

        # Ganancia de Kalman
        #   K → 0  (P_pred << R): predicción confiable, ignorar IMU
        #   K → 1  (P_pred >> R): predicción incierta, seguir IMU
        K = P_pred / (P_pred + self.R)

        self.x = normalize_angle(x_pred + K * innov)
        self.P = (1.0 - K) * P_pred

        return self.x


class Route_Command:

    def __init__(self, sender, vision_override_event, path=None, esp=None):
        self.default_route   = [(0, 3), (3, 3)]
        self.path            = path or self.default_route
        self.sender          = sender
        self.vision_override = vision_override_event
        self.imu_reader      = esp
        self.current_index   = 0

        self.heading_offset  = None          # calculado en el primer ciclo con IMU
        self._kf             = HeadingKalman(q=0.005, r=0.05)

    def reset_path(self):
        self.current_index = 0

    def set_path(self, path):
        self.path = path

    def follow_path(self, rover_odometry):
        DIST_TOLERANCE  = 0.2        # m
        ANGLE_TOLERANCE = 0.1 / 2    # rad — zona muerta para ir recto
        BASE_RPM        = 40
        KP              = 35.0

        FWD  = "D1"
        BWD  = "D0"
        SRPM = f"S{BASE_RPM}"
        STOP = "S0"

        def go_forward():
            self.sender.send_route(FWD, SRPM, FWD, SRPM)

        def turn_left_tank():
            self.sender.send_route(BWD, SRPM, FWD, SRPM)

        def turn_right_tank():
            self.sender.send_route(FWD, SRPM, BWD, SRPM)

        def stop():
            self.sender.send_route(FWD, STOP, FWD, STOP)

        was_tracking = False

        while self.current_index < len(self.path):
            target_x, target_y = self.path[self.current_index]

            # --- PRIORIDAD DE VISIÓN ---
            if self.vision_override.is_set():
                was_tracking = True
                time.sleep(0.1)
                continue

            if was_tracking:
                was_tracking = False
                stop()
                print("Seguimiento terminado. Calculando el próximo punto más cercano...")
                target_x, target_y = self.path[self.current_index]
                print(f"Resumiendo ruta hacia el punto: {self.path[self.current_index]}")

            # --- ODOMETRÍA ---
            current_x, current_y, current_theta = rover_odometry.pose
            _, omega_odo = rover_odometry.velocity   # [m/s, rad/s]
            fused_theta  = current_theta

            # --- FUSIÓN KALMAN (solo si hay IMU) ---
            if self.imu_reader is not None:
                state           = self.imu_reader.get_sensor_state()
                imu_heading_deg = state.get("sensores", {}).get("heading", 0.0)
                imu_theta       = math.radians(imu_heading_deg)

                # Alineación: calcular offset entre sistemas de referencia.
                # Se hace una sola vez cuando el filtro aún no está inicializado,
                # usando la pose actual de la odometría como verdad de arranque.
                if self.heading_offset is None:
                    self.heading_offset = normalize_angle(current_theta - imu_theta)
                    self._kf.init(current_theta)

                # Llevar el IMU al mismo sistema de referencia que la odometría.
                imu_theta_aligned = normalize_angle(imu_theta + self.heading_offset)

                # Ciclo del filtro de Kalman.
                fused_theta = self._kf.update(omega_odo, imu_theta_aligned)

                print(
                    f"IMU: {imu_heading_deg:.1f}°  "
                    f"Odo: {math.degrees(current_theta):.1f}°  "
                    f"Fused: {math.degrees(fused_theta):.1f}°  "
                    f"K: {self._kf.P / (self._kf.P + self._kf.R):.3f}"
                )
            else:
                print(f"Odo: {math.degrees(current_theta):.1f}° (sin IMU)")

            # --- GEOMETRÍA DE RUTA ---
            dx           = target_x - current_x
            dy           = target_y - current_y
            distance     = math.hypot(dx, dy)
            target_angle = math.atan2(dy, dx)
            angle_error  = normalize_angle(target_angle - fused_theta)

            # --- LÓGICA DE MOVIMIENTO ---
            if distance < DIST_TOLERANCE:
                print(f"Punto alcanzado: {self.path[self.current_index]} :)")
                self.current_index += 1
                continue

            if abs(angle_error) > math.radians(45):
                # Error grande → giro en eje propio
                if angle_error > 0:
                    turn_left_tank()
                else:
                    turn_right_tank()

            elif abs(angle_error) <= ANGLE_TOLERANCE:
                # Dentro de zona muerta → recto
                go_forward()

            else:
                # Control proporcional suave
                correccion = int(KP * angle_error)
                MAX_RPM = 67
                MIN_RPM = 15
                rpm_izq = max(MIN_RPM, min(MAX_RPM, BASE_RPM - correccion))
                rpm_der = max(MIN_RPM, min(MAX_RPM, BASE_RPM + correccion))
                self.sender.send_route(FWD, f"S{rpm_izq}", FWD, f"S{rpm_der}")

            time.sleep(0.1)

        print("Ruta completada. Deteniendo rover.")
        stop()


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))