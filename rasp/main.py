from mapeo_trayectoria import RoverOdometry, RoverMap
from connect import ESP

if __name__ == "__main__":
    # 1. Crear odometría con la trocha real del rover
    odo = RoverOdometry(track_width_m=0.80)

    # 2. Conectar ESPs (bloquea ~3s por puerto mientras hace handshake)
    ESP.connect()

    # 3. Lanzar la ventana (bloqueante hasta cerrar)
    #    Presiona R para resetear la posición en cualquier momento
    mapa = RoverMap(
        odometry = odo,
        width    = 1200,
        height   = 1000,
        scale    = 50,     # 1 metro = 50 píxeles
        fps      = 60
    )
    mapa.run()