import threading
from mapeo_trayectoria import RoverOdometry, RoverMap
from connect import ESP

def odometry_loop():
    while True:
        odo._parse_esp_line(esp.lectura_odometria_izq)
        odo._parse_esp_line(esp.lectura_odometria_der)
        odo._update_pose()

if __name__ == "__main__":
    esp = ESP()
    esp.connect()

    odo = RoverOdometry()
    mapa = RoverMap(odometry=odo)

    threading.Thread(target=odometry_loop, daemon=True).start()

    mapa.run()