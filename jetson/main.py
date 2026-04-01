import threading
import time
from connect import ESP
import command
from zed import ZEDShared

def control_loop(esp: ESP):
    """Consume command_state y envía comandos UART a los ESP32."""
    while True:
        with server.command_lock:
            cmd = dict(server.command_state)

        esp.send_uart(
            cmd["left_dir"], cmd["left_rpm"],
            cmd["right_dir"], cmd["right_rpm"]
        )
        time.sleep(0.02)  # 50 Hz

if __name__ == "__main__":
    esp = ESP()
    esp.connect()

    zed=ZEDShared(resolution="HD720",fps=30,depth_mode="NEURAL",min_depth=0.2,max_depth=20,confidence_threshold=50).start()

    rvr_cmd = command.Route_Command(esp, vision_override_event=threading.Event())

    # Este sí puede ser daemon
    threading.Thread(target=control_loop, args=(esp,), daemon=True).start()

    try:
        print("Iniciando servidor Flask en http://0.0.0.0:5000")
        # Flask se queda en el hilo principal
        server.app.run(
            host="0.0.0.0",
            port=5000,
            threaded=True,
            use_reloader=False
        )
    finally:
        zed.stop()
        esp.close()