"""
rover_viz.py — Visualizador pygame de la pose y trayectoria del rover.

Marco del mundo = marco de la odometría (odo.py): origen donde arrancó el
rover, +x hacia donde miraba, +y a su izquierda, θ antihorario.
En pantalla: +x a la derecha, +y hacia arriba.

Teclas:
    ESPACIO / Enter    arrancar la ruta (mientras espera)
    ESC / Q / cerrar   cancelar o PARO durante la ruta; salir al terminar
    S                  guardar captura PNG en el directorio actual

Debe correr en el hilo principal (pygame/SDL lo requiere): el script de prueba
llama a update() + tick() desde su loop de monitoreo.
"""
import os
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

import math
import time

import pygame

# ── Colores ─────────────────────────────────────────────────────────────────
BG      = (17, 19, 23)
PANEL   = (26, 29, 35)
GRID    = (36, 40, 48)
AXIS    = (84, 90, 104)
TEXT    = (222, 226, 232)
MUTED   = (130, 137, 150)
PATH    = (74, 128, 214)
TARGET  = (255, 193, 7)
REACHED = (76, 196, 120)
TRAIL   = (0, 206, 209)
ROVER   = (236, 236, 236)
FUSED   = (214, 96, 214)
GOOD    = (76, 196, 120)
BAD     = (232, 78, 78)
WAIT    = (255, 193, 7)

# ── Layout ──────────────────────────────────────────────────────────────────
PANEL_W     = 300
MAP_MARGIN  = 30
GRID_STEPS  = (0.05, 0.1, 0.25, 0.5, 1.0, 2.0, 5.0, 10.0)   # m
MIN_GRID_PX = 60      # separación mínima entre líneas de cuadrícula
MIN_SPAN_M  = 2.0     # vista mínima de 2 m aunque la ruta sea corta
TRAIL_STEP  = 0.005   # m — distancia mínima entre puntos de la trayectoria

KEYS_BY_MODE = {
    "wait": ["ESPACIO/Enter  arrancar", "ESC/Q          cancelar", "S              captura"],
    "run":  ["ESC/Q/cerrar   PARO", "S              captura"],
    "done": ["ESC/Q/cerrar   salir", "S              captura"],
}


class RoverViz:

    def __init__(self, path, tolerance_m, track_width_m, size=(1100, 720), title="follow_path"):
        """
        path          : lista de waypoints [(x, y), ...] en metros
        tolerance_m   : radio para considerar alcanzado un punto (se dibuja como círculo)
        track_width_m : distancia entre ruedas, para dibujar el rover a escala
        """
        pygame.init()
        pygame.display.set_caption(f"{title} — pose del rover")
        self.screen = pygame.display.set_mode(size, pygame.RESIZABLE)
        self.clock  = pygame.time.Clock()
        self.font   = pygame.font.SysFont("consolas", 16)
        self.font_s = pygame.font.SysFont("consolas", 13)
        self.font_b = pygame.font.SysFont("consolas", 20, bold=True)

        self.title = title
        self.path  = [tuple(p) for p in path]
        self.tol   = tolerance_m
        self.track = track_width_m

        self.pose         = (0.0, 0.0, 0.0)
        self.velocity     = (0.0, 0.0)
        self.fused_theta  = None
        self.target_index = 0

        self.trail    = []
        self.distance = 0.0
        self._trail_bounds = None    # [minx, miny, maxx, maxy]
        self.t0    = None
        self.t_end = None

        self.set_waiting()

    # ------------------------------------------------------------------ #
    #  Estado                                                              #
    # ------------------------------------------------------------------ #

    def set_waiting(self):
        self.mode = "wait"
        self.status = "Esperando arranque"
        self.status_color = WAIT
        self.hint = "Empuja el rover a mano para checar la odometría antes de arrancar."

    def set_running(self):
        """Llamar justo después de odo.reset_pose(): limpia la trayectoria y arranca el reloj."""
        self.mode = "run"
        self.status = "En ruta"
        self.status_color = TRAIL
        self.hint = None
        self._clear_trail()
        self.t0 = time.time()
        self.t_end = None

    def set_finished(self, text, ok):
        self.mode = "done"
        self.status = text
        self.status_color = GOOD if ok else BAD
        self.hint = "Rover detenido."
        if self.t0 is not None and self.t_end is None:
            self.t_end = time.time()

    def update(self, pose, target_index, velocity=(0.0, 0.0), fused_theta=None):
        self.pose = tuple(pose)
        self.target_index = target_index
        self.velocity = tuple(velocity)
        self.fused_theta = fused_theta

        x, y = self.pose[0], self.pose[1]
        if self.trail:
            px, py = self.trail[-1]
            step = math.hypot(x - px, y - py)
            if step < TRAIL_STEP:
                return
            self.distance += step
        self.trail.append((x, y))
        if self._trail_bounds is None:
            self._trail_bounds = [x, y, x, y]
        else:
            b = self._trail_bounds
            b[0], b[1] = min(b[0], x), min(b[1], y)
            b[2], b[3] = max(b[2], x), max(b[3], y)

    def _clear_trail(self):
        self.trail = []
        self.distance = 0.0
        self._trail_bounds = None

    # ------------------------------------------------------------------ #
    #  Loop                                                                #
    # ------------------------------------------------------------------ #

    def tick(self, fps=30):
        """Procesa eventos, dibuja un frame y limita los fps.
        Retorna "quit", "start" o None."""
        action = None
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                action = "quit"
            elif ev.type == pygame.KEYDOWN:
                if ev.key in (pygame.K_ESCAPE, pygame.K_q):
                    action = "quit"
                elif ev.key in (pygame.K_SPACE, pygame.K_RETURN, pygame.K_KP_ENTER):
                    if action is None:
                        action = "start"
                elif ev.key == pygame.K_s:
                    self.save_screenshot()
        self.draw()
        pygame.display.flip()
        self.clock.tick(fps)
        return action

    def hold(self):
        """Mantiene la ventana con el resultado hasta que el usuario la cierre."""
        while self.tick() != "quit":
            pass

    def close(self):
        pygame.quit()

    def save_screenshot(self, filename=None):
        path = os.path.abspath(filename or time.strftime("follow_path_%Y%m%d_%H%M%S.png"))
        pygame.image.save(self.screen, path)
        print(f"📸 Captura guardada: {path}")
        return path

    # ------------------------------------------------------------------ #
    #  Dibujo                                                              #
    # ------------------------------------------------------------------ #

    def draw(self):
        self.screen.fill(BG)
        w, h = self.screen.get_size()
        self.map_rect = pygame.Rect(PANEL_W, 0, max(w - PANEL_W, 100), h)
        self._fit_view()
        self._draw_grid()
        self._draw_path()
        self._draw_trail()
        self._draw_rover()
        self._draw_panel()

    # ── Transformación mundo ↔ pantalla ──
    def _fit_view(self):
        pts = [(0.0, 0.0), self.pose[:2], *self.path]
        if self._trail_bounds is not None:
            b = self._trail_bounds
            pts += [(b[0], b[1]), (b[2], b[3])]
        pad = max(0.5, self.track)
        minx = min(p[0] for p in pts) - pad
        maxx = max(p[0] for p in pts) + pad
        miny = min(p[1] for p in pts) - pad
        maxy = max(p[1] for p in pts) + pad
        span_x = max(maxx - minx, MIN_SPAN_M)
        span_y = max(maxy - miny, MIN_SPAN_M)
        r = self.map_rect
        self.scale = max(min((r.w - 2 * MAP_MARGIN) / span_x,
                             (r.h - 2 * MAP_MARGIN) / span_y), 1.0)   # px por metro
        self.cx = (minx + maxx) / 2.0
        self.cy = (miny + maxy) / 2.0

    def w2s(self, x, y):
        r = self.map_rect
        return (round(r.centerx + (x - self.cx) * self.scale),
                round(r.centery - (y - self.cy) * self.scale))

    def s2w(self, sx, sy):
        r = self.map_rect
        return (self.cx + (sx - r.centerx) / self.scale,
                self.cy - (sy - r.centery) / self.scale)

    # ── Capas ──
    def _draw_grid(self):
        r = self.map_rect
        step = next((s for s in GRID_STEPS if s * self.scale >= MIN_GRID_PX), GRID_STEPS[-1])
        x_min, y_max = self.s2w(r.left, r.top)
        x_max, y_min = self.s2w(r.right, r.bottom)

        i = math.ceil(x_min / step)
        while i * step <= x_max:
            sx, _ = self.w2s(i * step, 0.0)
            pygame.draw.line(self.screen, AXIS if i == 0 else GRID, (sx, r.top), (sx, r.bottom))
            self._text(f"{round(i * step, 3):g}", (sx + 3, r.bottom - 16), MUTED, self.font_s)
            i += 1

        j = math.ceil(y_min / step)
        while j * step <= y_max:
            _, sy = self.w2s(0.0, j * step)
            pygame.draw.line(self.screen, AXIS if j == 0 else GRID, (r.left, sy), (r.right, sy))
            self._text(f"{round(j * step, 3):g}", (r.left + 4, sy - 15), MUTED, self.font_s)
            j += 1

        self._text(f"cuadro = {step:g} m", (r.right - 120, r.top + 8), MUTED, self.font_s)

        # Ejes del marco de odometría en el origen
        ox, oy = self.w2s(0.0, 0.0)
        pygame.draw.line(self.screen, AXIS, (ox, oy), (ox + 40, oy), 2)
        pygame.draw.line(self.screen, AXIS, (ox, oy), (ox, oy - 40), 2)
        self._text("+x", (ox + 44, oy - 8), AXIS, self.font_s)
        self._text("+y", (ox - 8, oy - 58), AXIS, self.font_s)

    def _draw_path(self):
        pts = [self.w2s(0.0, 0.0)] + [self.w2s(x, y) for x, y in self.path]
        if len(pts) >= 2:
            pygame.draw.lines(self.screen, PATH, False, pts, 2)

        tol_px = max(int(self.tol * self.scale), 2)
        for i, (sx, sy) in enumerate(pts[1:]):
            if i < self.target_index:
                color = REACHED
            elif i == self.target_index:
                color = TARGET
            else:
                color = PATH
            pygame.draw.circle(self.screen, color, (sx, sy), tol_px, 1)
            pygame.draw.circle(self.screen, color, (sx, sy), 5)
            self._text(f"P{i}", (sx + 8, sy - 20), color, self.font_s)

    def _draw_trail(self):
        if len(self.trail) >= 2:
            pygame.draw.lines(self.screen, TRAIL, False, [self.w2s(x, y) for x, y in self.trail], 2)

    def _draw_rover(self):
        x, y, theta = self.pose
        center = self.w2s(x, y)
        radius = max(int(self.track / 2.0 * self.scale), 8)
        pygame.draw.circle(self.screen, ROVER, center, radius, 2)
        self._arrow(center, theta, max(radius * 1.6, 26), ROVER, 3)
        if self.fused_theta is not None:
            self._arrow(center, self.fused_theta, max(radius * 1.25, 20), FUSED, 2)

    def _arrow(self, origin, theta, length, color, width):
        ox, oy = origin
        dx, dy = math.cos(theta), -math.sin(theta)   # y de pantalla invertida
        tip = (ox + dx * length, oy + dy * length)
        pygame.draw.line(self.screen, color, origin, tip, width)
        head = []
        for a in (0.45, -0.45):
            hx = dx * math.cos(a) - dy * math.sin(a)
            hy = dx * math.sin(a) + dy * math.cos(a)
            head.append((tip[0] - 11 * hx, tip[1] - 11 * hy))
        pygame.draw.polygon(self.screen, color, [tip, *head])

    def _draw_panel(self):
        h = self.screen.get_height()
        pygame.draw.rect(self.screen, PANEL, (0, 0, PANEL_W, h))
        left, width = 16, PANEL_W - 32
        y = 16

        y = self._text(self.title, (left, y), TEXT, self.font_b) + 4
        for line in self._wrap(self.status, self.font_b, width):
            y = self._text(line, (left, y), self.status_color, self.font_b)
        if self.hint:
            for line in self._wrap(self.hint, self.font_s, width):
                y = self._text(line, (left, y), MUTED, self.font_s)
        y += 12

        x, yy, theta = self.pose
        v, omega = self.velocity
        if self.t0 is None:
            elapsed = 0.0
        else:
            elapsed = (self.t_end or time.time()) - self.t0
        # Al terminar, la odometría ya no se actualiza: no mostrar velocidades viejas
        stale = self.mode == "done"
        rows = [
            ("t",         f"{elapsed:7.1f} s"),
            ("x",         f"{x:+8.3f} m"),
            ("y",         f"{yy:+8.3f} m"),
            ("θ",         f"{math.degrees(theta):+8.2f}°"),
            ("v",         "       — m/s" if stale else f"{v:+8.3f} m/s"),
            ("ω",         "       — rad/s" if stale else f"{omega:+8.3f} rad/s"),
            ("recorrido", f"{self.distance:8.2f} m"),
        ]
        if self.fused_theta is not None:
            rows.append(("θ fusión", f"{math.degrees(self.fused_theta):+8.2f}°"))
        for label, value in rows:
            self._text(label, (left, y), MUTED, self.font)
            y = self._text(value, (left + 110, y), TEXT, self.font)
        y += 12

        n, idx = len(self.path), self.target_index
        y = self._text(f"Puntos alcanzados: {min(idx, n)}/{n}", (left, y), TEXT, self.font)
        if idx < n:
            tx, ty = self.path[idx]
            y = self._text(f"Objetivo P{idx} ({tx:.2f}, {ty:.2f})", (left, y), TARGET, self.font)
            y = self._text(f"a {math.hypot(tx - x, ty - yy):.3f} m", (left, y), TARGET, self.font)
        y += 16

        legend = [
            (PATH,    "Ruta planeada"),
            (TRAIL,   "Trayectoria (odometría)"),
            (TARGET,  "Objetivo actual"),
            (REACHED, "Punto alcanzado"),
            (ROVER,   "Rumbo odometría"),
        ]
        if self.fused_theta is not None:
            legend.append((FUSED, "Rumbo fusionado (IMU)"))
        for color, label in legend:
            pygame.draw.rect(self.screen, color, (left, y + 4, 14, 8))
            y = self._text(label, (left + 22, y), MUTED, self.font_s) + 2

        keys = KEYS_BY_MODE[self.mode]
        y = h - 16 - len(keys) * (self.font_s.get_height() + 2)
        for k in keys:
            y = self._text(k, (left, y), MUTED, self.font_s) + 2

    # ── Texto ──
    def _text(self, text, pos, color, font):
        """Dibuja texto y regresa la coordenada y de la siguiente línea."""
        self.screen.blit(font.render(text, True, color), pos)
        return pos[1] + font.get_height() + 2

    @staticmethod
    def _wrap(text, font, width):
        lines, current = [], ""
        for word in text.split():
            candidate = f"{current} {word}".strip()
            if font.size(candidate)[0] <= width or not current:
                current = candidate
            else:
                lines.append(current)
                current = word
        if current:
            lines.append(current)
        return lines
