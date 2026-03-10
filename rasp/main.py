import threading
import time
from connect import ESP
import server

def control_loop(esp: ESP):
    """Consume command_state y envía comandos UART a los ESP32."""
    while True:
        with server.command_lock:
            cmd = dict(server.command_state)
        esp.send_uart(cmd["left_dir"], cmd["left_rpm"],
                      cmd["right_dir"], cmd["right_rpm"])
        time.sleep(0.02)    # 50 Hz

def flask_loop():
    """Corre el servidor Flask en su propio hilo."""
    server.app.run(host="0.0.0.0", port=5000, threaded=True)

if __name__ == "__main__":
    # 1. Conexión serial — fuente única de verdad
    esp = ESP()
    esp.connect()

    # 2. Inyectar ESP en server.py antes de que lleguen requests
    server.init_app(esp)

    # 3. Hilos secundarios
    threading.Thread(target=control_loop,  args=(esp,),  daemon=True).start()
    threading.Thread(target=flask_loop,                  daemon=True).start()