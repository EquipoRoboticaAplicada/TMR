import math
import time

class Route_Command:
    def __init__(self, ESP_send, vision_override_event):
        self.path =  [
            (3, 0),    
            (3, 3),   
            (0, 3),    
            (0, 0),     
            (3, 3)      
        ]
        self.send_motor_rpm = ESP_send
        self.vision_override = vision_override_event

    def set_path(self):
        return self.path

    def follow_path(self, rover_odometry):
        DIST_TOLERANCE = 0.2  # m (que tan cerca se considera el arribo al punto)
        ANGLE_TOLERANCE = 0.1 # radianes (que tan cerca del angulo objetivo antes de avanzar)  
        BASE_RPM = 30 
        
        current_index = 0
        was_tracking = False

        while current_index < len(self.path):
            target_x, target_y = self.path[current_index]
            
            # --- REVISIÓN DE PRIORIDAD DE VISIÓN ---
            if self.vision_override.is_set():
                was_tracking = True 
                time.sleep(0.1) 
                continue
            
            # --- RECÁLCULO DEL PUNTO MÁS CERCANO ---


            if was_tracking:
                was_tracking = False
                current_x, current_y, _ = rover_odometry.pose
                print("Seguimiento terminado. Calculando el próximo punto más cercano...")
                
                min_dist = float('inf')
                best_index = current_index
                
                # Buscar solo entre los puntos restantes
                # for i in range(current_index, len(self.path)):
                #     wx, wy = self.path[i]
                #     dist = math.hypot(wx - current_x, wy - current_y)
                #     if dist < min_dist:
                #         min_dist = dist
                #         best_index = i
                        
                current_index = best_index
                target_x, target_y = self.path[current_index]
                print(f"Resumiendo ruta hacia el punto: {self.path[current_index]}")

            # --- LÓGICA NORMAL DE SEGUIMIENTO DE RUTA ---
            current_x, current_y, current_theta = rover_odometry.pose
            
            dx = target_x - current_x   
            dy = target_y - current_y   
            distance = math.hypot(dx, dy)  
            
            target_angle = math.atan2(dy, dx) 
            angle_error = normalize_angle(target_angle - current_theta)  
            
            if distance < DIST_TOLERANCE:
                print(f"Se ha llegado al punto: {self.path[current_index]} :)")
                current_index += 1  
                continue 
                
            if abs(angle_error) > ANGLE_TOLERANCE:
                if angle_error > 0:
                    self.send_motor_rpm.send_uart(BASE_RPM, BASE_RPM, 0, 1)  # Derecha      
                else:
                    self.send_motor_rpm.send_uart(BASE_RPM, BASE_RPM, 1, 0)  # Izquierda
            else:
                self.send_motor_rpm.send_uart(BASE_RPM, BASE_RPM, 1, 1)  # En frente
                
            time.sleep(0.1) 

        print("Ruta completada. Deteniendo rover.")  
        self.send_motor_rpm.send_uart(0, 0, 1, 1)     

def normalize_angle(angle):
    while angle > math.pi: angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle