import math
import time


class Route_Command:
    def __init__(self, sender, vision_override_event, path=None):
        self.path = path or [(3,0),(3,3),(0,3),(0,0),(3,3)]
        self.sender          = sender
        self.vision_override = vision_override_event

    def follow_path(self, rover_odometry):
        DIST_TOLERANCE  = 0.2   # m   — distancia para considerar que se llegó al punto
        ANGLE_TOLERANCE = 0.1   # rad — error angular antes de avanzar recto
        BASE_RPM        = 30

        FWD  = "D1"
        BWD  = "D0"
        SRPM = f"S{BASE_RPM}"
        STOP = "S0"

        def go_forward():
            self.sender.send_route(FWD, SRPM, FWD, SRPM)

        def turn_left():
            # Rueda derecha adelante, rueda izquierda atrás ===> gira a la izquierda
            self.sender.send_route(BWD, SRPM, FWD, SRPM)

        def turn_right():
            # Rueda izquierda adelante, rueda derecha atrás ===> gira a la derecha
            self.sender.send_route(FWD, SRPM, BWD, SRPM)

        def stop():
            self.sender.send_route(FWD, STOP, FWD, STOP)

        current_index = 0
        was_tracking  = False

        while current_index < len(self.path):
            target_x, target_y = self.path[current_index]

            # --- PRIORIDAD DE VISIÓN ---
            if self.vision_override.is_set():
                was_tracking = True
                time.sleep(0.1)
                continue

            # --- RECÁLCULO DEL PUNTO MÁS CERCANO tras retomar control ---
            if was_tracking:
                was_tracking = False
                # stop()  # detener antes de recalcular para evitar inestabilidad
                current_x, current_y, _ = rover_odometry.pose
                print("Seguimiento terminado. Calculando el próximo punto más cercano...")

                min_dist   = float('inf')
                best_index = current_index

                for i in range(current_index, len(self.path)):
                    wx, wy = self.path[i]
                    dist = math.hypot(wx - current_x, wy - current_y)
                    if dist < min_dist:
                        min_dist   = dist
                        best_index = i

                current_index      = best_index
                target_x, target_y = self.path[current_index]
                print(f"Resumiendo ruta hacia el punto: {self.path[current_index]}")

            # --- LÓGICA NORMAL DE SEGUIMIENTO ---
            current_x, current_y, current_theta = rover_odometry.pose

            dx       = target_x - current_x
            dy       = target_y - current_y
            distance = math.hypot(dx, dy)

            target_angle = math.atan2(dy, dx)
            angle_error  = normalize_angle(target_angle - current_theta)

            if distance < DIST_TOLERANCE:
                print(f"Se ha llegado al punto: {self.path[current_index]} :)")
                current_index += 1
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


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))