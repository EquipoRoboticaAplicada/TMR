import threading
from util import ImgProcessor, Sender
from odo import RoverOdometry
from map import RoverMap
from command import Route_Command

PI_IP = "172.32.236.53"

if __name__ == "__main__":
    # 1. ComunicaciónXN
    sender  = Sender(PI_IP).start()
    grabber = ImgProcessor(PI_IP).start()

    # 2. Odometría
    odo  = RoverOdometry(PI_IP)
    mapa = RoverMap(odometry=odo)

    # 3. Visión + control en hilo secundario
    #    run_tracker no llama cv.imshow para no competir con pygame en el main thread
    threading.Thread(
        target=grabber.run_tracker,
        args=(sender,),
        daemon=True
    ).start()



    # 4. Mapa pygame — bloqueante en main thread
    #    _update_pose() se llama dentro de mapa.run() en cada frame
    try:
        mapa.run()
    finally:
        grabber.stop()
        sender.stop()
        odo.stop()

    # 5. Ruta predefinida para el rover

    route=Route_Command()