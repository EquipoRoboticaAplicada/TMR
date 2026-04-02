from map import RoverMap
from receiver import Receiver

Jetson_IP = "192.168.1.82"

if __name__ == "__main__":    
    receiver  = Receiver(Jetson_IP)
    rec = receiver.start()
    mapa = RoverMap(receiver=rec)

    try:
        mapa.run()
    finally:
        receiver.stop()