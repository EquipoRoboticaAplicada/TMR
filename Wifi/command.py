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
from util import Sender


PI_IP = "172.32.236.53"
CMD_URL = f"http://{PI_IP}:5000/command"

class Route_Command:

    send_motor_rpm=Sender.start(PI_IP)

    def __init__(self):
        self.path =  [
            (10, 0),    
            (10, 10),   
            (0, 10),    
            (0, 0),     
            (5, 5)      
        ]

    # Ruta eligida
    def set_path(self):
        return self.path


    def follow_path(self, rover_odometry):
       
        # Tolerancias
        DIST_TOLERANCE = 0.2  # m (que tan cerca se considera el arrivo al punto)
        ANGLE_TOLERANCE = 0.1 # radianes (que tan cerca del angulo objetivo antes de avanzar)  
        
        BASE_RPM = 30 
        
        for target in self.path:
            target_x, target_y = target
            print(f"Moving to waypoint: {target}")
            
            reached = False
            while not reached:
                # 1. Del request server.py, se obtiene la odometria que se está calculando en tiempo real (posisión del rover)
                current_x, current_y, current_theta = rover_odometry.pose
                
                # 2. Cálculo de errores 
                dx = target_x - current_x   # Error en x
                dy = target_y - current_y   # Error en y
                distance = math.hypot(dx, dy)  # Distancia por recorrer (hipotenusa con librería math)
                
                target_angle = math.atan2(dy, dx) # Ángulo deseado 
                angle_error = normalize_angle(target_angle - current_theta)  # Error del angulo
                
                # 3. Revisión si es que se llegó al punto deseado
                if distance < DIST_TOLERANCE:
                    reached = True
                    print("Se ha llegado al punto :)")
                    break
                    
                # 4. Lógica de Movimiento (Rotar, luego moverse)
                if abs(angle_error) > ANGLE_TOLERANCE:
                    # Rotate in place
                    if angle_error > 0:
                        # Girar a la izquierda: Izquierda hacia atrás (0), derecha hacia adelante (1)
                        self.send_motor_rpm.send_rpms(BASE_RPM,BASE_RPM,0,1)        
                    else:
                        # Girar a la derecha: Izquierda hacia adelante (1), Derecha hacia atrás (0)
                        self.send_motor_rpm(BASE_RPM,BASE_RPM,1,0)  
                else:
                    # Moverse hacia adelante (Si la orientación es correcta)
                    # Izquierda hacia adelante (1), derecha hacia adelante (1)
                    self.send_motor_rpm(BASE_RPM,BASE_RPM,1,1) 
                    
                time.sleep(0.1) # Control loop delay (10Hz)

        # Se detienen los motores cuando se completa la ruta
        print("Ruta completada :D")  
        self.send_motor_rpm(0, 0, 1, 1)     # Se requerirá modificar después para que esté continuamente recorriendo el escenario

# Normalizar angulos entre -pi y pi
def normalize_angle(angle):
    while angle > math.pi: angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle