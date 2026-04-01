# main.py
from connect import ESP
import server
from zed import ZEDShared
from vision_zed import VisionZED
from util import SenderJetson, ImgProcessorJetson


def get_remote_command():
    with server.command_lock:
        return dict(server.command_state)

def command_loop():
    


if __name__ == "__main__":
    esp = ESP()
    esp.connect()

    zed = ZEDShared(
        resolution="HD720",
        fps=30,
        depth_mode="PERFORMANCE",
        min_depth=0.2,
        max_depth=20.0,
        confidence_threshold=50
    ).start()

    vision = VisionZED(
        zed_shared=zed,
        color_file="colors.json",
        area_min=500,
        draw_local=False
    ).start()

    tracker = ImgProcessorJetson(vision)

    sender_local = SenderJetson(
        esp=esp,
        get_remote_command_fn=get_remote_command,
        is_tracking_fn=tracker.is_tracking
    ).start()

    tracker.start(sender_local)

    server.init_app(esp, zed, vision, tracker)

    try:
        print("Iniciando servidor Flask en http://0.0.0.0:5000")
        server.app.run(
            host="0.0.0.0",
            port=5000,
            threaded=True,
            use_reloader=False
        )
    finally:
        tracker.stop()
        sender_local.stop()
        vision.stop()
        zed.stop()
        esp.close()