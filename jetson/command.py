# command.py
import math
import time


class Route_Command:

    def __init__(self, sender, vision_override_event, path=None, esp=None):
        self.default_route   = [(0, 3), (3, 3)]
        self.path            = path or self.default_route
        self.sender          = sender
        self.vision_override = vision_override_event
        self.imu_reader      = esp
        self.current_index   = 0

        self.heading_offset  = None

        # ── Filtro Complementario ────────────────────────────────────────────
        # alpha controla cuánto peso tiene cada fuente en la fusión:
        #
        #   alpha → 1.0 : confía casi totalmente en la odometría
        #                 (precisa a corto plazo, pero acumula drift con el tiempo)
        #   alpha → 0.0 : confía casi totalmente en el IMU
        #                 (estable a largo plazo, pero puede tener ruido puntual)
        #
        # Valor recomendado: 0.80 – 0.90
        #   → la odometría domina el control ciclo a ciclo,
        #     mientras el IMU corrige la deriva acumulada.
        self.cf_alpha        = 1   # peso de la odometría  (1-alpha = peso IMU)

        # Estimación fusionada persistente entre iteraciones del bucle
        self._fused_theta    = None   # se inicializa en la primera lectura

    # ────────────────────────────────────────────────────────────────────────
    def reset_path(self):
        self.current_index = 0

    def set_path(self, path):
        self.path = path

    # ── Filtro Complementario ────────────────────────────────────────────────
    def _fuse_heading(self, odom_theta: float) -> float:
        """Fusiona la orientación de odometría con el heading del IMU mediante
        un filtro complementario.

        Fórmula aplicada:
            delta      = normalize(imu_theta - fused_prev)   ← diferencia angular corta
            fused_new  = normalize(fused_prev
                                   + alpha       * normalize(odom_theta - fused_prev)
                                   + (1 - alpha) * delta)

        Separar la contribución de cada fuente como *correcciones sobre el estado
        anterior* (en vez de promediar ángulos crudos) evita el salto discontinuo
        que ocurre cuando los ángulos cruzan el límite ±π.

        Returns:
            fused_theta (float): orientación fusionada en radianes, en [-π, π].
        """
        # Inicialización: el primer ciclo arranca desde la odometría
        if self._fused_theta is None:
            self._fused_theta = odom_theta

        # 1) Leer IMU (si no hay reader disponible, usar solo odometría)
        imu_theta = odom_theta          # valor de respaldo
        if self.imu_reader is not None:
            try:
                state           = self.imu_reader.get_sensor_state()
                imu_heading_deg = state.get("sensores", {}).get("heading", 0.0)
                imu_theta       = math.radians(imu_heading_deg)
            except Exception as exc:
                # Sensor temporalmente no disponible → mantener la estimación actual
                print(f"[CF] Advertencia: no se pudo leer el IMU ({exc}). "
                      "Usando solo odometría.")

        # 2) Calcular correcciones angulares cortas (resuelven el cruce ±π)
        odom_delta = normalize_angle(odom_theta - self._fused_theta)
        imu_delta  = normalize_angle(imu_theta  - self._fused_theta)

        # 3) Aplicar el filtro complementario
        #       alpha        → contribución de la odometría
        #       (1 - alpha)  → contribución del IMU
        correction        = self.cf_alpha * odom_delta + (1.0 - self.cf_alpha) * imu_delta
        self._fused_theta = normalize_angle(self._fused_theta + correction)

        print(f"[CF] Odom θ: {odom_theta:.2f} rad, IMU θ: {imu_theta:.2f} rad, "
              f"Fused θ: {self._fused_theta:.2f} rad")

        return self._fused_theta

    # ────────────────────────────────────────────────────────────────────────
    def follow_path(self, rover_odometry):
        DIST_TOLERANCE  = 0.2   # m   — distancia para considerar que se llegó al punto
        ANGLE_TOLERANCE = 0.1   # rad — error angular antes de avanzar recto
        BASE_RPM        = 35

        FWD  = "D1"
        BWD  = "D0"
        SRPM = f"S{BASE_RPM}"
        STOP = "S0"

        def go_forward():
            self.sender.send_route(FWD, SRPM, FWD, SRPM)

        def turn_left():
            # Rueda derecha adelante, rueda izquierda atrás ===> gira a la izquierda
            self.sender.send_route(BWD, SRPM, FWD, f"S{65}")

        def turn_right():
            # Rueda izquierda adelante, rueda derecha atrás ===> gira a la derecha
            self.sender.send_route(FWD, f"S{65}", BWD, SRPM)

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

            # --- RECÁLCULO DEL PUNTO MÁS CERCANO tras retomar control ---
            if was_tracking:
                was_tracking = False
                stop()          # detener antes de recalcular para evitar inestabilidad
                current_x, current_y, _ = rover_odometry.pose
                print("Seguimiento terminado. Calculando el próximo punto más cercano...")

                min_dist = float('inf')
                # best_index = current_index

                # for i in range(current_index, len(self.path)):
                #     wx, wy = self.path[i]
                #     dist = math.hypot(wx - current_x, wy - current_y)
                #     if dist < min_dist:
                #         min_dist   = dist
                #         best_index = i

                # current_index      = best_index
                target_x, target_y = self.path[self.current_index]
                print(f"Resumiendo ruta hacia el punto: {self.path[self.current_index]}")

            # --- LÓGICA NORMAL DE SEGUIMIENTO ---
            current_x, current_y, current_theta = rover_odometry.pose

            # ── FUSIÓN: filtro complementario odometría + IMU ────────────────
            fused_theta = self._fuse_heading(current_theta)
            # ─────────────────────────────────────────────────────────────────

            dx       = target_x - current_x
            dy       = target_y - current_y
            distance = math.hypot(dx, dy)

            target_angle = math.atan2(dy, dx)
            angle_error  = normalize_angle(target_angle - fused_theta)  # ← fusionado

            if distance < DIST_TOLERANCE:
                print(f"Se ha llegado al punto: {self.path[self.current_index]} :)")
                self.current_index += 1
                continue

            if abs(angle_error) > ANGLE_TOLERANCE:
                if angle_error > 0:
                    turn_left()
                else:
                    turn_right()
            else:
                go_forward()

            time.sleep(0.1)

        print("Ruta completada. Deteniendo rover.")
        stop()


# ────────────────────────────────────────────────────────────────────────────
def normalize_angle(angle: float) -> float:
    """Normaliza un ángulo al rango [-π, π]."""
    return math.atan2(math.sin(angle), math.cos(angle))