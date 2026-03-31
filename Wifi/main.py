import threading
from odo import RoverOdometry
from map import RoverMap

Jetson_IP = "192.168.0.190"

if __name__ == "__main__":    
    odo  = RoverOdometry(Jetson_IP)
    mapa = RoverMap(odometry=odo)

    try:
        mapa.run()
    finally:
        odo.stop()