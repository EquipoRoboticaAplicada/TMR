import pygame
import math
from collections import deque
from receiver import Receiver


class RoverMap:
    """
    Renderiza en tiempo real la trayectoria del rover y el video feed
    de la cámara ZED (picture-in-picture) en una ventana pygame.
    """

    # Colores
    BG_COLOR     = (30,  30,  30)
    PATH_COLOR   = (0,  255,   0)
    ROVER_COLOR  = (255,  0,   0)
    GRID_COLOR   = (60,  60,  60)
    ORIGIN_COLOR = (100, 100, 255)
    TEXT_COLOR   = (200, 200, 200)
    STALE_COLOR  = (255,  80,  80)
    PIP_BORDER   = (80,  80,  80)
    NO_SIGNAL_BG = (50,  20,  20)

    # PiP
    PIP_W        = 426    # ancho del recuadro de video en pantalla (px)
    PIP_H        = 240    # alto  del recuadro de video en pantalla (px)
    PIP_MARGIN   = 12     # distancia al borde de la ventana (px)

    def __init__(self, receiver: Receiver,
                 width: int = 1200, height: int = 1000,
                 scale: float = 50.0, fps: int = 60):

        self.receiver  = receiver
        self.width     = width
        self.height    = height
        self.scale     = scale
        self.fps       = fps
        self.origin_x  = width  // 2
        self.origin_y  = height // 2

        self._path    = deque(maxlen=5000)
        self._running = False

        # Posición del recuadro PiP (esquina inferior derecha)
        self._pip_x = width  - self.PIP_W - self.PIP_MARGIN
        self._pip_y = height - self.PIP_H - self.PIP_MARGIN

        ok, failed = pygame.init()
        if failed > 0:
            print(f"[RoverMap] ⚠ {failed} módulo(s) de pygame fallaron al inicializar.")

        self._screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Trayectoria del ROVER")
        self._clock  = pygame.time.Clock()
        self._font   = pygame.font.SysFont("monospace", 14)

    # ------------------------------------------------------------------ #
    #  Conversión coordenadas mundo → pantalla                            #
    # ------------------------------------------------------------------ #

    def _world_to_screen(self, x: float, y: float) -> tuple:
        sx = int(self.origin_x + x * self.scale)
        sy = int(self.origin_y - y * self.scale)
        return sx, sy

    # ------------------------------------------------------------------ #
    #  Dibujo del mapa                                                     #
    # ------------------------------------------------------------------ #

    def _draw_grid(self):
        step = int(self.scale)
        for xi in range(self.origin_x % step, self.width, step):
            pygame.draw.line(self._screen, self.GRID_COLOR, (xi, 0), (xi, self.height))
        for yi in range(self.origin_y % step, self.height, step):
            pygame.draw.line(self._screen, self.GRID_COLOR, (0, yi), (self.width, yi))
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
        rover_m = 0.3
        local_pts = [
            ( rover_m,        0.0),
            (-rover_m / 2,  rover_m / 2),
            (-rover_m / 2, -rover_m / 2),
        ]
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        screen_pts = []
        for lx, ly in local_pts:
            rx = lx * cos_t - ly * sin_t
            ry = lx * sin_t + ly * cos_t
            screen_pts.append(self._world_to_screen(x + rx, y + ry))
        pygame.draw.polygon(self._screen, self.ROVER_COLOR, screen_pts)

    def _draw_hud(self, x: float, y: float, theta: float,
                  v: float, omega: float):
        stale = self.receiver.is_stale
        signal_text  = "⚠ SIN SEÑAL" if stale else "SEÑAL: OK"
        signal_color = self.STALE_COLOR if stale else self.TEXT_COLOR
        lines = [
            (signal_text,                            signal_color),
            (f"x     : {x:+.3f} m",          self.TEXT_COLOR),
            (f"y     : {y:+.3f} m",          self.TEXT_COLOR),
            (f"θ     : {math.degrees(theta):+.1f}°", self.TEXT_COLOR),
            (f"v     : {v:+.4f} m/s",        self.TEXT_COLOR),
            (f"ω     : {omega:+.4f} rad/s",  self.TEXT_COLOR),
            (f"puntos: {len(self._path)}",   self.TEXT_COLOR),
        ]
        for i, (text, color) in enumerate(lines):
            surf = self._font.render(text, True, color)
            self._screen.blit(surf, (10, 10 + i * 18))

    # ------------------------------------------------------------------ #
    #  Bucle principal                                                     #
    # ------------------------------------------------------------------ #

    def run(self):
        """Inicia el bucle de renderizado. Bloqueante hasta cerrar la ventana."""
        self._running = True
        try:
            while self._running:
                self._clock.tick(self.fps)

                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        self._running = False
                    elif event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_r:
                            self.receiver.reset_pose()
                            self._path.clear()
                            print("[RoverMap] Path limpiado.")

                x, y, theta = self.receiver.pose
                v, omega    = self.receiver.velocity

                if not self._path or (x, y) != self._path[-1]:
                    self._path.append((x, y))

                self._screen.fill(self.BG_COLOR)
                self._draw_grid()
                self._draw_path()
                self._draw_rover(x, y, theta)
                self._draw_hud(x, y, theta, v, omega)

                pygame.display.flip()

        except Exception as e:
            print(f"[RoverMap] Error en visualización: {e}")
        finally:
            self._quit()

    def _quit(self):
        pygame.quit()
        print("Visualización cerrada.")