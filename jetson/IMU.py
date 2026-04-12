#import pygame
import serial
import threading

# ===== SERIAL =====
puerto = 'COM5'
ser = serial.Serial(puerto, 115200, timeout=1)
ser.flushInput()

# ===== VARIABLES GLOBALES DEL SENSOR =====
pitch = 0.0
heading = 0.0
velocity = 0.0
peso = 0.0
terrain_text = "NONE"

running = True
lock = threading.Lock()

# ===== CONSTANTES DE CONTROL =====
KP = 0.8  # Constante proporcional. Súbela si corrige muy lento, bájala si oscila.
MAX_RPM = 80
MIN_RPM = 15

# ===== CONFIGURACIÓN =====
GRID_SIZE = 50
CELL_SIZE = 10

terrain_map = [[None for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]

row = 0
col = 0
direction = 1

# estados
mapping_active = False
running = True

# datos
pitch = 0
heading = 0
velocity = 0
peso = 0
terrain_text = "NONE"

# estado botón medir
medicion_activa = False
tiempo_boton = 0

lock = threading.Lock()

# # ===== PYGAME =====
# pygame.init()

# MARGIN_LEFT = 60
# MARGIN_BOTTOM = 60

# WIDTH = GRID_SIZE * CELL_SIZE + MARGIN_LEFT + 150
# HEIGHT = GRID_SIZE * CELL_SIZE + MARGIN_BOTTOM + 140

# screen = pygame.display.set_mode((WIDTH, HEIGHT))
# pygame.display.set_caption("Topographic Map")

# font = pygame.font.SysFont("Arial", 14)
# big_font = pygame.font.SysFont("Arial", 18)

# clock = pygame.time.Clock()

# # ===== BOTONES =====
# start_button = pygame.Rect(WIDTH - 140, HEIGHT - 60, 120, 40)
# reset_button = pygame.Rect(WIDTH - 140, HEIGHT - 110, 120, 40)
# measure_button = pygame.Rect(WIDTH - 140, HEIGHT - 160, 120, 40)

# ===== FUNCIONES =====
def limit_pitch(p):
    return max(-40, min(40, p))

def map_pitch(p):
    p = limit_pitch(p)
    return int(max(0, min(100, 50 + (p / 40) * 50)))

def pitch_to_color(v):
    if v <= 50:
        r = 255
        g = int(255 * (v / 50))
    else:
        r = int(255 * (1 - (v - 50) / 50))
        g = 255
    return (r, g, 0)

# ===== THREAD SERIAL =====
def serial_thread():
    global row, col, direction
    global pitch, heading, velocity, terrain_text, peso

    while running:
        try:
            line = ser.readline().decode().strip()
            if not line:
                continue

            data = line.split(",")
            if len(data) != 5:
                continue

            p, h, v, t, w = data

            with lock:
                pitch = float(p)
                heading = float(h)
                velocity = float(v)
                terrain_text = t
                peso = float(w)

                if mapping_active and row < GRID_SIZE:
                    val = map_pitch(pitch)

                    terrain_map[row][col] = val
                    col += direction

                    if col >= GRID_SIZE or col < 0:
                        row += 1
                        direction *= -1
                        col += direction

        except:
            pass

thread = threading.Thread(target=serial_thread)
thread.start()

# # ===== LOOP =====
# while running:

#     for event in pygame.event.get():
#         if event.type == pygame.QUIT:
#             running = False

#         if event.type == pygame.MOUSEBUTTONDOWN:

#             # START / STOP
#             if start_button.collidepoint(event.pos):
#                 mapping_active = not mapping_active

#             # RESET
#             if reset_button.collidepoint(event.pos):
#                 with lock:
#                     for r in range(GRID_SIZE):
#                         for c in range(GRID_SIZE):
#                             terrain_map[r][c] = None
#                     row, col, direction = 0, 0, 1

#             # ===== MEDIR PESO =====
#             if measure_button.collidepoint(event.pos):
#                 ser.write(b'\r\n')   # 🔥 ENTER correcto
#                 ser.flush()

#                 medicion_activa = True
#                 tiempo_boton = pygame.time.get_ticks()

#     # ===== FONDO =====
#     screen.fill((40, 0, 70))

#     # ===== MAPA =====
#     with lock:
#         for r in range(GRID_SIZE):
#             for c in range(GRID_SIZE):
#                 val = terrain_map[r][c]
#                 color = (100, 100, 150) if val is None else pitch_to_color(val)

#                 x = MARGIN_LEFT + c * CELL_SIZE
#                 y = r * CELL_SIZE

#                 pygame.draw.rect(screen, color, (x, y, CELL_SIZE, CELL_SIZE))

#     # ===== EJES =====
#     for i in range(0, GRID_SIZE + 1, 5):
#         meters = i * 0.2

#         y = i * CELL_SIZE
#         screen.blit(font.render(f"{meters:.0f}", True, (255,255,255)), (10, y))

#         x = MARGIN_LEFT + i * CELL_SIZE
#         screen.blit(font.render(f"{meters:.0f}", True, (255,255,255)),
#                     (x, GRID_SIZE * CELL_SIZE + 5))

#     # ===== ESCALA =====
#     scale_x = GRID_SIZE * CELL_SIZE + MARGIN_LEFT + 30
#     scale_y = 20
#     scale_height = 300

#     for i in range(scale_height):
#         v = 100 - int((i / scale_height) * 100)
#         pygame.draw.line(screen, pitch_to_color(v),
#                          (scale_x, scale_y + i),
#                          (scale_x + 20, scale_y + i))

#     for v in [0, 50, 100]:
#         y = scale_y + scale_height - int((v / 100) * scale_height)
#         screen.blit(font.render(f"{v}", True, (255,255,255)),
#                     (scale_x + 30, y - 5))

#     # ===== INFO =====
#     info_y = scale_y + scale_height + 20

#     with lock:
#         screen.blit(big_font.render(f"Pitch: {pitch:.2f}", True, (255,255,255)),
#                     (scale_x - 20, info_y))

#         screen.blit(big_font.render(f"Heading: {heading:.2f}", True, (255,255,255)),
#                     (scale_x - 20, info_y + 30))

#         screen.blit(big_font.render(f"Vel: {velocity:.2f}", True, (255,255,255)),
#                     (scale_x - 20, info_y + 60))

#         screen.blit(big_font.render(f"Terrain: {terrain_text}", True, (255,255,0)),
#                     (scale_x - 20, info_y + 90))

#         screen.blit(big_font.render(f"Peso: {peso:.2f} g", True, (0,255,255)),
#                     (scale_x - 20, info_y + 120))

#     # ===== BOTONES =====

#     # START / STOP
#     if mapping_active:
#         color = (0, 180, 0)
#         text = "STOP"
#     else:
#         color = (180, 0, 0)
#         text = "START"

#     pygame.draw.rect(screen, color, start_button)
#     screen.blit(font.render(text, True, (255,255,255)),
#                 (start_button.x + 25, start_button.y + 10))

#     # RESET
#     pygame.draw.rect(screen, (50, 50, 200), reset_button)
#     screen.blit(font.render("RESET", True, (255,255,255)),
#                 (reset_button.x + 25, reset_button.y + 10))

#     # ===== BOTÓN MEDIR (con animación) =====
#     if medicion_activa and pygame.time.get_ticks() - tiempo_boton > 500:
#         medicion_activa = False

#     color_medir = (255,150,0) if medicion_activa else (0,150,200)

#     pygame.draw.rect(screen, color_medir, measure_button)
#     screen.blit(font.render("MEDIR", True, (255,255,255)),
#                 (measure_button.x + 25, measure_button.y + 10))

#     pygame.display.flip()
#     clock.tick(30)

# ===== FUNCIONES DE CONTROL DE MOTORES =====

def obtener_datos_imu():
    """Devuelve los datos actuales del IMU de forma segura."""
    with lock:
        return pitch, heading, velocity, terrain_text, peso

def calcular_correccion_motores(target_heading, base_rpm=40):
    """
    Calcula los RPM para los motores izquierdo y derecho basándose en 
    el error entre el heading actual y el target_heading (ambos en grados).
    """
    with lock:
        current_heading = heading

    # 1. Calcular el error. 
    # El IMU da valores de 0 a 360. Necesitamos la diferencia más corta.
    error_grados = target_heading - current_heading
    
    # Normalizar el error a un rango de -180 a 180 grados
    error_grados = (error_grados + 180) % 360 - 180

    # 2. Aplicar Controlador Proporcional (P)
    correccion = int(KP * error_grados)

    # 3. Calcular RPMs
    # Si el error es positivo (ej. target=90, current=0), debemos girar a la derecha.
    # Girar a la derecha = llanta izquierda más rápida, llanta derecha más lenta.
    # (Ajusta los signos "+" y "-" dependiendo de la orientación física de tus motores)
    rpm_izq = base_rpm + correccion
    rpm_der = base_rpm - correccion

    # 4. Saturación (Clamp) para evitar pedirle al ESP32 velocidades imposibles
    rpm_izq = max(MIN_RPM, min(MAX_RPM, rpm_izq))
    rpm_der = max(MIN_RPM, min(MAX_RPM, rpm_der))

    return rpm_izq, rpm_der, error_grados

def detener_imu():
    """Detiene el hilo y cierra el puerto serial de manera segura."""
    global running
    running = False

# ===== CIERRE =====
running = False
thread.join()
ser.close()
#pygame.quit() DESCOMENTAR ESTO PARA EL MAPA 
ser.close()