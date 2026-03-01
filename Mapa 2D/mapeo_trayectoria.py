"""
mapeo_trayectoria.py
Visualización 2D en tiempo real del trayecto del rover usando pygame.
Depende de odo_no_rasp.py (clase RoverOdometry).
"""

import pygame
import math
from odo_no_rasp import RoverOdometry


# ================= CLASE DE VISUALIZACIÓN =================

class RoverMap:
    """
    Renderiza en tiempo real la trayectoria del rover en una ventana pygame.

    Parámetros:
        odometry   : instancia de RoverOdometry ya conectada
        width/height: tamaño de la ventana en píxeles
        scale      : píxeles por metro
        fps        : fotogramas por segundo del bucle de render
    """

    # Colores
    BG_COLOR      = (30,  30,  30)
    PATH_COLOR    = (0,  255,   0)
    ROVER_COLOR   = (255,  0,   0)
    GRID_COLOR    = (60,  60,  60)
    ORIGIN_COLOR  = (100, 100, 255)
    TEXT_COLOR    = (200, 200, 200)

    def __init__(self, odometry: RoverOdometry,
                 width: int = 1200, height: int = 1000,
                 scale: float = 50.0, fps: int = 60):

        self.odometry = odometry
        self.width    = width
        self.height   = height
        self.scale    = scale          # px / metro
        self.fps      = fps

        # Centro de pantalla = origen del mundo
        self.origin_x = width  // 2
        self.origin_y = height // 2

        self._path   = []
        self._running = False

        pygame.init()
        self._screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Trayectoria del ROVER")
        self._clock  = pygame.time.Clock()
        self._font   = pygame.font.SysFont("monospace", 14)

    # ------------------------------------------------------------------ #
    #  Conversión de coordenadas mundo → pantalla                         #
    # ------------------------------------------------------------------ #

    def _world_to_screen(self, x: float, y: float) -> tuple:
        """
        Convierte metros (x derecha, y arriba) a píxeles pygame
        (eje Y invertido: arriba en mundo = arriba en pantalla).
        """
        sx = int(self.origin_x + x * self.scale)
        sy = int(self.origin_y - y * self.scale)   # Y invertido
        return sx, sy

    # ------------------------------------------------------------------ #
    #  Dibujo                                                              #
    # ------------------------------------------------------------------ #

    def _draw_grid(self):
        """Grilla de fondo cada 1 metro."""
        step = int(self.scale)   # px entre líneas = 1 metro

        # Líneas verticales
        for xi in range(self.origin_x % step, self.width, step):
            pygame.draw.line(self._screen, self.GRID_COLOR, (xi, 0), (xi, self.height))

        # Líneas horizontales
        for yi in range(self.origin_y % step, self.height, step):
            pygame.draw.line(self._screen, self.GRID_COLOR, (0, yi), (self.width, yi))

        # Ejes principales
        pygame.draw.line(self._screen, self.ORIGIN_COLOR,
                         (self.origin_x, 0), (self.origin_x, self.height), 1)
        pygame.draw.line(self._screen, self.ORIGIN_COLOR,
                         (0, self.origin_y), (self.width, self.origin_y), 1)

    def _draw_path(self):
        if len(self._path) < 2:
            return
        points_px = [self._world_to_screen(px, py) for px, py in self._path]
        pygame.draw.lines(self._screen, self.PATH_COLOR, False, points_px, 2)

    def _draw_rover(self, x: float, y: float, theta: float):
        """Dibuja el rover como triángulo apuntando hacia theta."""
        rover_m = 0.3   # tamaño en metros

        # Vértices en coordenadas locales (frente, atrás-izq, atrás-der)
        local_pts = [
            ( rover_m,         0.0),
            (-rover_m / 2,  rover_m / 2),
            (-rover_m / 2, -rover_m / 2),
        ]

        cos_t, sin_t = math.cos(theta), math.sin(theta)
        screen_pts = []

        for lx, ly in local_pts:
            # Rotar
            rx = lx * cos_t - ly * sin_t
            ry = lx * sin_t + ly * cos_t
            # Trasladar al mundo y convertir a pantalla
            screen_pts.append(self._world_to_screen(x + rx, y + ry))

        pygame.draw.polygon(self._screen, self.ROVER_COLOR, screen_pts)

    def _draw_hud(self, x: float, y: float, theta: float,
                  v: float, omega: float):
        """Muestra telemetría en la esquina superior izquierda."""
        lines = [
            f"x     : {x:+.3f} m",
            f"y     : {y:+.3f} m",
            f"θ     : {math.degrees(theta):+.1f}°",
            f"v     : {v:+.4f} m/s",
            f"ω     : {omega:+.4f} rad/s",
            f"puntos: {len(self._path)}",
        ]
        for i, text in enumerate(lines):
            surf = self._font.render(text, True, self.TEXT_COLOR)
            self._screen.blit(surf, (10, 10 + i * 18))

    # ------------------------------------------------------------------ #
    #  Bucle principal                                                     #
    # ------------------------------------------------------------------ #

    def run(self):
        """Inicia el bucle de renderizado. Bloqueante hasta cerrar la ventana."""
        self._running = True

        while self._running:
            self._clock.tick(self.fps)

            # --- Eventos ---
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self._running = False

                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_r:          # R → resetear pose
                        self.odometry.reset_pose()
                        self._path.clear()
                        print("🔄 Pose reseteada.")

            # --- Obtener estado del rover ---
            x, y, theta = self.odometry.pose
            # print(f"X: {x}, Y: {y}, Theta: {theta}") # DEBUG
            v, omega     = self.odometry.velocity

            # Guardar punto en la trayectoria (evitar duplicados estáticos)
            if not self._path or (x, y) != self._path[-1]:
                self._path.append((x, y))

            # --- Render ---
            self._screen.fill(self.BG_COLOR)
            self._draw_grid()
            self._draw_path()
            self._draw_rover(x, y, theta)
            self._draw_hud(x, y, theta, v, omega)

            pygame.display.flip()

        self._quit()

    def _quit(self):
        self.odometry.close()
        pygame.quit()
        print("✅ Visualización cerrada.")


# ================= MAIN =================

if __name__ == "__main__":
    # 1. Crear odometría con la trocha real del rover
    odo = RoverOdometry(track_width_m=0.80)

    # 2. Conectar ESPs (bloquea ~3s por puerto mientras hace handshake)
    odo.connect()

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