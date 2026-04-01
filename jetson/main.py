from connect import ESP
import server
from vision_zed import VisionZED, ZEDShared
from util import SenderJetson, ImgProcessorJetson
from command import Route_Command
from odo import RoverOdometry
import threading
import socket


def get_local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    finally:
        s.close()
    return ip


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

    # 7. Servidor Flask (telemetría + stream de video)
    server.init_app(esp, zed, vision, tracker)

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

    jetson_IP = get_local_ip()

    try:
        print(f"Iniciando servidor Flask en http://{jetson_IP}:5000")
        server.app.run(
            host=jetson_IP,
            port=5000,
            threaded=True,
            use_reloader=False
        )
    finally:
        tracker.stop()
        sender_local.stop()
        vision.stop()
        zed.stop()
        odo.stop()
        esp.close()