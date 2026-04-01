# main.py
from connect import ESP
import server
from zed import ZEDShared

def get_remote_command():
    with server.command_lock:
        return dict(server.command_state)


if __name__ == "__main__":
    esp = ESP()
    esp.connect()

    zed=ZEDShared(resolution="HD720",fps=30,depth_mode="NEURAL",min_depth=0.2,max_depth=20,confidence_threshold=50).start()


    server.init_app(esp,zed)

    # Este sí puede ser daemon
    threading.Thread(target=control_loop, args=(esp,), daemon=True).start()

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