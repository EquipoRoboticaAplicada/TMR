import sys
from map import RoverMap
from receiver import Receiver

DEFAULT_IP = "192.168.1.82"

if __name__ == "__main__":
    jetson_ip = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_IP

    receiver = Receiver(jetson_ip).start()
    mapa = RoverMap(receiver=receiver)

    try:
        mapa.run()
    finally:
        receiver.stop()