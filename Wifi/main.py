import threading
from util import ImgProcessor, Sender
from odo import RoverOdometry
from map import RoverMap
from command import Route_Command

Jetson_IP = "192.168.0.190"

if __name__ == "__main__":
    # 1. Comunicación
    sender  = Sender(Jetson_IP).start()
    
    # 2. Bandera de prioridad para visión
    vision_override = threading.Event()

    # 3. Pasar la bandera a ImgProcessor
    grabber = ImgProcessor(Jetson_IP, vision_override=vision_override).start()

    # 4. Odometría
    odo  = RoverOdometry(Jetson_IP)
    mapa = RoverMap(odometry=odo)

    # 5. Visión + control en hilo secundario
    threading.Thread(
        target=grabber.run_tracker,
        args=(sender,),
        daemon=True
    ).start()

    # 6. Ruta predefinida para el rover (en hilo secundario)
    route = Route_Command(sender, vision_override)
    
    threading.Thread(
        target=route.follow_path,
        args=(odo,),
        daemon=True
    ).start()

    # 7. Mapa pygame — bloqueante en main thread
    try:
        mapa.run()
    finally:
        grabber.stop()
        sender.stop()
        odo.stop()