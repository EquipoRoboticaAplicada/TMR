import threading
import time
from mapeo_trayectoria import RoverOdometry, RoverMap
from connect import ESP

def odometry_loop():
    last_izq = ""
    last_der = ""

    while True:
        izq = esp.lectura_odometria_izq
        der = esp.lectura_odometria_der

        # Solo procesar si llegó dato nuevo
        if izq != last_izq:
            odo._parse_esp_line(izq)
            last_izq = izq

        if der != last_der:
            odo._parse_esp_line(der)
            last_der = der

        odo._update_pose()
        time.sleep(0.005)   # 200 Hz máximo — cede CPU al thread serial y a pygame

if __name__ == "__main__":
    esp = ESP()
    esp.connect()

    odo = RoverOdometry()
    mapa = RoverMap(odometry=odo)

    threading.Thread(target=odometry_loop, daemon=True).start()

    mapa.run()