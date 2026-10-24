import time                                                                                                                                                      
import math                                                                                                                                                      
import threading                                                                                                                                                 
from connect import ESP                                                                                                                                          
from odo import RoverOdometry                                                                                                                                    
import server                                                                                                                                                    
                                                                                                                                                                    
def simulate_esp_inputs(esp: ESP):                                                                                                                               
    """Genera tramas seriales simuladas y las inyecta en los parsers de ESP."""                                                                                  
    seq = 0                                                                                                                                                      
    t = 0.0                                                                                                                                                      
    print("🚀 Iniciando generador de tramas simuladas...")                                                                                                       
                                                                                                                                                                    
    while True:                                                                                                                                                  
        seq += 1                                                                                                                                                 
        t += 0.1                                                                                                                                                 
                                                                                                                                                                    
        # 1. Simulación de dinámica de motores (RPM y m/s)                                                                                                       
        rpm_l = 60.0 + 15.0 * math.sin(t * 0.5)                                                                                                                  
        rpm_r = 60.0 + 15.0 * math.cos(t * 0.5)                                                                                                                  
        v_l = (rpm_l / 60.0) * (2 * math.pi * 0.10)   # asumiendo radio aprox                                                                                    
        v_r = (rpm_r / 60.0) * (2 * math.pi * 0.10)                                                                                                              
                                                                                                                                                                    
        # Formato esperado por connect.py: ESP_L/R,seq,rpm0,v0,rpm1,v1,rpm2,v2                                                                                   
        line_l = f"ESP_L,{seq},{rpm_l:.2f},{v_l:.3f},{rpm_l:.2f},{v_l:.3f},{rpm_l:.2f},{v_l:.3f}"                                                                
        line_r = f"ESP_R,{seq},{rpm_r:.2f},{v_r:.3f},{rpm_r:.2f},{v_r:.3f},{rpm_r:.2f},{v_r:.3f}"                                                                
                                                                                                                                                                    
        # 2. Simulación de sensores (pitch, heading, velocidad, terreno, peso)                                                                                   
        pitch = 3.0 * math.sin(t * 0.3)                                                                                                                          
        heading = (t * 5.0) % 360.0                                                                                                                              
        velocity = (v_l + v_r) / 2.0                                                                                                                             
        peso = 15.2 + 0.3 * math.sin(t * 0.1)                                                                                                                    
                                                                                                                                                                    
        # Formato: sensores,pitch,heading,velocity,terrain_text,peso                                                                                             
        line_s = f"sensores,{pitch:.2f},{heading:.2f},{velocity:.2f},REGOLITO,{peso:.2f}"                                                                        
                                                                                                                                                                    
        # Inyectar directamente a los métodos de parseo de connect.py                                                                                            
        esp._parse_esp_line_m(line_l)                                                                                                                            
        esp._parse_esp_line_m(line_r)                                                                                                                            
        esp._parse_esp_line_s(line_s)                                                                                                                            
                                                                                                                                                                    
        time.sleep(0.05)  # 20 Hz                                                                                                                                
                                                                                                                                                                    
if __name__ == "__main__":                                                                                                                                       
    # Instanciamos la clase ESP real de connect.py                                                                                                               
    esp = ESP()                                                                                                                                                  
                                                                                                                                                                    
    # Odometría que integra los motores de ESP                                                                                                                   
    odo = RoverOdometry(esp=esp)                                                                                                                                 
                                                                                                                                                                    
    # Inicializar server MQTT (apunta a localhost)                                                                                                               
    server.init_app(                                                                                                                                             
        esp_instance=esp,                                                                                                                                        
        zed_instance=None,                                                                                                                                       
        vision_instance=None,                                                                                                                                    
        tracker_instance=None,                                                                                                                                   
        odo_instance=odo,                                                                                                                                        
        cmd_instance=None                                                                                                                                        
    )                                                                                                                                                            
                                                                                                                                                                    
    # Iniciar hilo de inyección de tramas seriales                                                                                                               
    threading.Thread(target=simulate_esp_inputs, args=(esp,), daemon=True).start()                                                                               
                                                                                                                                                                    
    # Iniciar publicación MQTT hacia el broker de HMI                                                                                                            
    print("📡 Publicando telemetría hacia localhost:1883...")                                                                                                    
    server.run(broker_ip="127.0.0.1", broker_port=1883)