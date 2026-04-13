# command.py
import math
import time

class Route_Command:

    def __init__(self, sender, vision_override_event, path=None, esp=None):
        self.default_route   = [(0,3), (3,3)]
        self.path            = path or self.default_route
        self.sender          = sender
        self.vision_override = vision_override_event
        self.imu_reader             = esp
        self.current_index   = 0
        
        self.heading_offset  = None 

    def reset_path(self):
        self.current_index = 0

    def set_path(self, path):
        self.path = path

    def follow_path(self, rover_odometry):
        DIST_TOLERANCE  = 0.2   # m
        ANGLE_TOLERANCE = 0.1/2   #  Zona muerta donde va derecho
        BASE_RPM        = 40
        
        ALPHA_IMU       = 0.8 
        
        # Constante Proporcional (Kp) - Ajusta este valor si gira muy lento o muy agresivo
        KP = 35.0  

        FWD  = "D1"
        BWD  = "D0"
        SRPM = f"S{BASE_RPM}"
        STOP = "S0"

        def go_forward():
            self.sender.send_route(FWD, SRPM, FWD, SRPM)

        # Giros sobre su propio eje (ahora solo se usan si el error es gigantesco)
        def turn_left_tank():
            self.sender.send_route(BWD, SRPM, FWD, SRPM)

        def turn_right_tank():
            self.sender.send_route(FWD, SRPM, BWD, SRPM)

        def stop():
            self.sender.send_route(FWD, STOP, FWD, STOP)

        was_tracking  = False

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
                current_x, current_y, _ = rover_odometry.pose
                print("Seguimiento terminado. Calculando el próximo punto más cercano...")

                target_x, target_y = self.path[self.current_index]
                print(f"Resumiendo ruta hacia el punto: {self.path[self.current_index]}")

            # --- ODOMETRÍA E IMU ---
            current_x, current_y, current_theta = rover_odometry.pose
            fused_theta = current_theta

            if self.imu_reader is not None:
                state = self.imu_reader.get_sensor_state()
                imu_heading_deg = state.get("sensores", {}).get("heading", 0.0)
                imu_theta = math.radians(imu_heading_deg) 
                
                if self.heading_offset is None:
                    self.heading_offset = normalize_angle(current_theta - imu_theta)
                
                imu_theta_aligned = normalize_angle(imu_theta + self.heading_offset)
                angle_diff = normalize_angle(imu_theta_aligned - current_theta)
                fused_theta = normalize_angle(current_theta + (ALPHA_IMU * angle_diff))
                print(f"IMU Heading: {imu_heading_deg:.2f}°, Odometry Theta: {math.degrees(current_theta):.2f}°, Fused Theta: {math.degrees(fused_theta):.2f}°")

            dx       = target_x - current_x
            dy       = target_y - current_y
            distance = math.hypot(dx, dy)

            target_angle = math.atan2(dy, dx)
            angle_error  = normalize_angle(target_angle - fused_theta)

            # --- LÓGICA DE MOVIMIENTO ---
            if distance < DIST_TOLERANCE:
                print(f"Se ha llegado al punto: {self.path[self.current_index]} :)")
                self.current_index += 1
                continue

            # 1. Si el error es masivo (> 45 grados), giramos en nuestro propio eje
            if abs(angle_error) > math.radians(45):
                if angle_error > 0:
                    turn_left_tank()
                else:
                    turn_right_tank()
            
            # 2. Si estamos apuntando casi perfecto (dentro de tolerancia), vamos derecho
            elif abs(angle_error) <= ANGLE_TOLERANCE:
                go_forward()
            
            # 3. Control Proporcional para correcciones suaves en movimiento
            else:
                # El error positivo requiere giro a la izquierda (rueda derecha más rápida)
                correccion = int(KP * angle_error)

                rpm_izq = BASE_RPM - correccion
                rpm_der = BASE_RPM + correccion

                # Saturación (Clamp) para no enviar velocidades inválidas o detener el motor
                # Ajusta MIN_RPM según el torque mínimo que requiere tu motor para moverse
                MAX_RPM = 80
                MIN_RPM = 15 
                
                rpm_izq = max(MIN_RPM, min(MAX_RPM, rpm_izq))
                rpm_der = max(MIN_RPM, min(MAX_RPM, rpm_der))

                self.sender.send_route(FWD, f"S{rpm_izq}", FWD, f"S{rpm_der}")

            time.sleep(0.1)

        print("Ruta completada. Deteniendo rover.")
        stop()

def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))