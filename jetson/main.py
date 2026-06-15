# main.py
from connect import ESP
import server
from vision_zed import VisionZED, ZEDShared
from util import SenderJetson, ImgProcessorJetson
from command import Route_Command
from odo import RoverOdometry
from local_debug import run_debug
import threading
import time

DRAW_LOCAL = False  # ← cambiar a False para desactivar la ventana de debug


# Modificación en jetson/main.py

if __name__ == "__main__":
    # 1. Conexión serial con los ESP32 (Protegida contra hardware faltante)
    esp = ESP()
    try:
        esp.connect()
        esp.send_uart("D1", "S0", "D1", "S0")  # fuerza parada inicial
        print("✅ ESP32 conectado exitosamente por Serial.")
    except Exception as e:
        print(f"⚠️ Alerta: No se pudo conectar al ESP32 ({e}). Iniciando en modo seguro sin serial.")

    # 2. Odometría (Evitamos que bloquee el flujo si no hay ESP)
    try:
        odo = RoverOdometry(esp=esp)
    except Exception as e:
        print(f"⚠️ Alerta: Error al iniciar Odometría física ({e}). Usando Mock de datos base.")
        # Creamos una clase dummy rápida para que server.py no truene al leer propiedades
        class DummyOdo:
            @property
            def pose(self): return (0.0, 0.0, 0.0)
            @property
            def velocity(self): return (0.0, 0.0)
            def reset_pose(self): pass
        odo = DummyOdo()

    # 3. Cámara ZED
    zed = ZEDShared().start()

    # 4. Pipeline de visión
    vision = VisionZED(zed_shared=zed).start()

    # 5. Sender: único punto de escritura al ESP
    sender_local = SenderJetson(esp=esp).start()

    # 6. Tracker: detecta objetos y toma control cuando corresponde
    tracker = ImgProcessorJetson(vision)
    tracker.start(sender_local)

    # 7. Ruta autónoma
    rvr_cmd = Route_Command(
        sender=sender_local,
        vision_override_event=tracker.vision_override,
        esp=esp
    )

    # 8. Servidor MQTT en hilo secundario
    server.init_app(esp, zed, vision, tracker, odo, rvr_cmd)
    
    # Asegúrate de que esta sea la IP real de tu laptop en este instante
    LAPTOP_BROKER_IP = "172.32.149.2" 
    
    print(f"📡 Lanzando hilo de red hacia el Broker en {LAPTOP_BROKER_IP}...")
    threading.Thread(
        target=server.run, 
        kwargs={"broker_ip": LAPTOP_BROKER_IP, "broker_port": 1883}, 
        daemon=True
    ).start()

    # Le damos un pequeño respiro (0.5s) al socket para establecerse antes de lanzar la ruta
    time.sleep(0.5)

    threading.Thread(
        target=rvr_cmd.follow_path,
        args=(odo,),
        daemon=True
    ).start()

    # El resto del código (DRAW_LOCAL e hilos de cierre) se queda exactamente igual...