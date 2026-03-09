import threading
import time
from mapeo_trayectoria import RoverOdometry, RoverMap
from connect import ESP
import server

def odometry_loop(odo: RoverOdometry):
    """Integra la pose del rover a alta frecuencia."""
    while True:
        odo._update_pose()
        time.sleep(0.005)   # 200 Hz

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

    # 3. Odometría y visualización
    odo  = RoverOdometry(esp)
    mapa = RoverMap(odometry=odo)

    # 4. Hilos secundarios
    threading.Thread(target=odometry_loop, args=(odo,),  daemon=True).start()
    threading.Thread(target=control_loop,  args=(esp,),  daemon=True).start()
    threading.Thread(target=flask_loop,                  daemon=True).start()

    # 5. Bucle principal: pygame (bloqueante)
    mapa.run()