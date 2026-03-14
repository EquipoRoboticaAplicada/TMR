# command.py

# Descripción del programa
#---------------------------------------------------------------------------------------------------------------------------------------------

# Este programa define una ruta que el rover deberá seguir (dicha ruta se defin en "set_path()")

# Se hace una normalización de unidades, como por ejemplo usar radianes en lugar ángulos (revisar si les gusta esta parte)

# Se manda el request a server.py para obtener los valores de posición actual del rover calculados por odometría 

# Se mandan comandos para que el rover se mueva de acuerdo a la ruta establacida, calculando errores del punto deseado hasta su punto actual  

#---------------------------------------------------------------------------------------------------------------------------------------------

import math
import time

class Route_Command:
    
    # Acepta la instancia del sender creado en main.py
    def __init__(self, sender_instance):
        self.path =  [
            (10, 0),    
            (10, 10),   
            (0, 10),    
            (0, 0),     
            (5, 5)      
        ]
        self.send_motor_rpm = sender_instance

    def set_path(self):
        return self.path

    def follow_path(self, rover_odometry):
        DIST_TOLERANCE = 0.2  
        ANGLE_TOLERANCE = 0.1 
        BASE_RPM = 30 
        
        for target in self.path:
            target_x, target_y = target
            print(f"Moving to waypoint: {target}")
            
            reached = False
            while not reached:
                current_x, current_y, current_theta = rover_odometry.pose
                
                dx = target_x - current_x   
                dy = target_y - current_y   
                distance = math.hypot(dx, dy)  
                
                target_angle = math.atan2(dy, dx) 
                angle_error = normalize_angle(target_angle - current_theta)  
                
                if distance < DIST_TOLERANCE:
                    reached = True
                    print("Se ha llegado al punto :)")
                    break
                    
                if abs(angle_error) > ANGLE_TOLERANCE:
                    if angle_error > 0:
                        self.send_motor_rpm.send_rpms(BASE_RPM,BASE_RPM,0,1)        
                    else:
                        
                        self.send_motor_rpm.send_rpms(BASE_RPM,BASE_RPM,1,0)  
                else:
                    
                    self.send_motor_rpm.send_rpms(BASE_RPM,BASE_RPM,1,1) 
                    
                time.sleep(0.1) 

        print("Ruta completada :D")  
      
        self.send_motor_rpm.send_rpms(0, 0, 1, 1)

def normalize_angle(angle):
    while angle > math.pi: angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle