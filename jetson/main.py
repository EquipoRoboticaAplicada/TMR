# main.py
from connect import ESP
import server

from vision_zed import VisionZED, ZEDShared
from util import SenderJetson, ImgProcessorJetson
from command import Route_Command
from odo import RoverOdometry
from local_debug import run_debug
import threading


if __name__ == "__main__":
    # 1. Conexión serial con los ESP32
    esp = ESP()
    esp.connect()

    # 2. Odometría (inicia su propio hilo interno)
    odo = RoverOdometry(esp=esp)

    # 3. Cámara ZED
    zed = ZEDShared().start()

    # 4. Pipeline de visión
    vision = VisionZED(zed_shared=zed).start()

    # 5. Sender: único punto de escritura al ESP
    sender_local = SenderJetson(esp=esp).start()

    # 6. Tracker: detecta objetos y toma control cuando corresponde
    tracker = ImgProcessorJetson(vision)
    tracker.start(sender_local)

    # 7. Servidor Flask en hilo secundario (OpenCV necesita el hilo principal)
    server.init_app(esp, zed, vision, tracker, odo)
    threading.Thread(target=server.run, daemon=True).start()

    # 8. Ruta autónoma
    rvr_cmd = Route_Command(
        sender=sender_local,
        vision_override_event=tracker.vision_override
    )
    threading.Thread(
        target=rvr_cmd.follow_path,
        args=(odo,),
        daemon=True
    ).start()

    # 9. Debug local — corre en el hilo principal para que OpenCV funcione
    try:
        run_debug(zed, vision, odo)
    finally:
        tracker.stop()
        sender_local.stop()
        vision.stop()
        zed.stop()
        odo.stop()
        esp.close()