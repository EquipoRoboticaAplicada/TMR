from vision import detect_colors, crosslines
import cv2 as cv
import requests
import time
import threading
import math

# ================= CONFIGURACIÓN =================
PI_IP = "172.32.236.53"  # <--- Asegúrate que esta sea la IP correcta de la Raspberry

VIDEO_URL     = f"http://{PI_IP}:5000/video_feed"
CMD_URL       = f"http://{PI_IP}:5000/command"
VISION_URL    = f"http://{PI_IP}:5000/vision_data"
TELEMETRY_URL = f"http://{PI_IP}:5000/telemetry"

# ================= HILOS DE COMUNICACIÓN =================

# 1. Hilo de Captura de Video (Existente)
class FrameGrabber:
    def __init__(self, src, warmup_sec=0.0):
        self.cap = cv.VideoCapture(src)
        self.lock = threading.Lock()
        self.frame = None
        self.ok = False
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

        if not self.cap.isOpened():
            print("⚠️ No se pudo abrir el stream de video (FrameGrabber)")
            # No lanzamos error fatal para permitir probar telemetría sin video
        
        if warmup_sec > 0:
            time.sleep(warmup_sec)

    def start(self):
        self.thread.start()
        return self

    def _run(self):
        while not self.stop_event.is_set():
            if self.cap.isOpened():
                ok, f = self.cap.read()
                if not ok:
                    self.ok = False
                    time.sleep(0.01)
                    continue
                with self.lock:
                    self.frame = f
                    self.ok = True
            else:
                time.sleep(0.1)

    def read(self):
        with self.lock:
            if self.frame is None:
                return False, None
            return self.ok, self.frame.copy()

    def stop(self):
        self.stop_event.set()
        self.thread.join(timeout=1.0)
        if self.cap.isOpened():
            self.cap.release()

# 2. Hilo de Envío de Comandos (Existente)
class Sender:
    def __init__(self, cmd_url, vision_url):
        self.cmd_url = cmd_url
        self.vision_url = vision_url
        self.session = requests.Session()
        self.lock = threading.Lock()
        self.latest_cmd = None
        self.latest_vision = None
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()
        return self

    def send_cmd(self, payload):
        with self.lock:
            self.latest_cmd = payload

    def send_vision(self, payload):
        with self.lock:
            self.latest_vision = payload

    def _run(self):
        while not self.stop_event.is_set():
            cmd, vis = None, None
            with self.lock:
                cmd = self.latest_cmd
                vis = self.latest_vision
                self.latest_cmd = None
                self.latest_vision = None

            try:
                if cmd:
                    self.session.post(self.cmd_url, json=cmd, timeout=0.2)
                if vis:
                    self.session.post(self.vision_url, json=vis, timeout=0.2)
            except Exception:
                pass # Ignorar errores de red para no saturar consola

            time.sleep(0.01)

    def stop(self):
        self.stop_event.set()
        self.thread.join(timeout=1.0)
        self.session.close()

# 3. Hilo de Recepción de Telemetría (NUEVO)
class TelemetryReceiver:
    def __init__(self, url):
        self.url = url
        self.session = requests.Session()
        self.lock = threading.Lock()
        self.data = None
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.last_print = 0

    def start(self):
        self.thread.start()
        return self

    def _run(self):
        while not self.stop_event.is_set():
            try:
                resp = self.session.get(self.url, timeout=0.2)
                if resp.status_code == 200:
                    json_data = resp.json()
                    with self.lock:
                        self.data = json_data.get("rover_state", {})
            except Exception:
                pass # Error de conexión momentáneo
            
            time.sleep(0.05) # Consultar a 20Hz aprox

    def get_state(self):
        with self.lock:
            return self.data

    def stop(self):
        self.stop_event.set()
        self.thread.join(timeout=1.0)
        self.session.close()

# ================= UTILIDADES =================
def send_rpms(sender, left_rpm, right_rpm, dir_left, dir_right):
    payload = {
        "left_rpm":  f"S{int(left_rpm)}",
        "right_rpm": f"S{int(right_rpm)}",
        "left_dir":  f"D{int(dir_left)}",
        "right_dir": f"D{int(dir_right)}",
    }
    sender.send_cmd(payload)

def send_vision_data(sender, colors, centroids, areas):
    payload = {
        "colors": list(colors) if isinstance(colors, set) else colors,
        "centroids": centroids,
        "areas": areas,
        "time": time.time()
    }
    sender.send_vision(payload)

def calc_turn_x(centroid, frame_width, deadband_px=10):
    cx = centroid[1]
    center_x = frame_width / 2
    error_px = cx - center_x
    if abs(error_px) < deadband_px: return 0.0
    error = error_px / (frame_width / 2)
    Kp = 5
    return max(-1.0, min(1.0, Kp * error))

def pick_target(centroids, areas, area_min=1500):
    if not centroids or not areas: return None
    candidates = [(c, a) for c, a in zip(centroids, areas) if a >= area_min]
    if not candidates: return None
    return max(candidates, key=lambda x: x[1]) # Retorna (centroid, area)

def clamp_rpm(rpm, min_rpm=20):
    return int(rpm) if rpm >= min_rpm else 0

# ================= MAIN =================
def main():
    print("🚀 Iniciando Cliente PC (Video + Odometría)...")

    # 1. Iniciar Hilos
    grabber = FrameGrabber(VIDEO_URL).start()
    sender = Sender(CMD_URL, VISION_URL).start()
    telemetry = TelemetryReceiver(TELEMETRY_URL).start()

    # Variables de Control
    tracking = False
    seen_count = 0
    lost_count = 0
    mode_rotate = False
    
    # Parametros
    CRUISE_RPM = 30
    APPROACH_RPM = 25
    AREA_MIN = 500
    N_ENTER = 5
    N_EXIT = 10
    
    left_rpm, right_rpm = CRUISE_RPM, CRUISE_RPM
    dir_left, dir_right = 1, 1

    # Timers de envío
    cmd_timer = 0
    vis_timer = 0
    
    # Loop Principal
    while True:
        # A. Leer Telemetría (Odometría)
        odom_state = telemetry.get_state()
        if odom_state:
            # Aquí tienes acceso a los datos crudos para tu algoritmo de odometría
            left_side = odom_state.get("left_side", {})
            right_side = odom_state.get("right_side", {})
            
            # Ejemplo: Imprimir RPM promedio de cada lado
            motors_l = left_side.get("motors", [])
            motors_r = right_side.get("motors", [])
            
            if motors_l and motors_r:
                rpm_l_avg = sum(m["rpm"] for m in motors_l) / 3
                rpm_r_avg = sum(m["rpm"] for m in motors_r) / 3
                
                # Imprimir datos para debug (comenta si satura)
                # print(f"📡 ODOM | L_RPM: {rpm_l_avg:.1f} | R_RPM: {rpm_r_avg:.1f} | Seq: {left_side.get('seq')}")

        # B. Leer Video
        ok, frame = grabber.read()
        if not ok or frame is None:
            time.sleep(0.01)
            continue

        # C. Procesamiento de Visión
        colors, centroids, areas = detect_colors(frame, draw=True)
        target = pick_target(centroids, areas, area_min=AREA_MIN)
        detected = target is not None

        if detected:
            centroid, area = target
            seen_count += 1
            lost_count = 0
        else:
            lost_count += 1
            seen_count = 0

        # Lógica de estados (Tracking)
        if not tracking and seen_count >= N_ENTER:
            tracking = True
            print("🟢 TRACKING ACTIVADO")
        
        if tracking and lost_count >= N_EXIT:
            tracking = False
            mode_rotate = False
            print("🔴 TRACKING PERDIDO")

        # Control de Motores
        if not tracking:
            left_rpm, right_rpm = CRUISE_RPM, CRUISE_RPM
            dir_left, dir_right = 1, 1
        else:
            # Tracking activo
            h, w = frame.shape[:2]
            cy = centroid[2]
            turn = calc_turn_x(centroid, w)

            # Histéresis rotación
            Y_TRIGGER = 0.40
            Y_HYST = 0.35
            
            if not mode_rotate and cy >= h * Y_TRIGGER:
                mode_rotate = True
            elif mode_rotate and cy < h * Y_HYST:
                mode_rotate = False

            if mode_rotate:
                rot = APPROACH_RPM * abs(turn)
                if abs(turn) < 0.1:
                    left_rpm, right_rpm = 0, 0
                elif turn > 0: # Gira derecha
                    left_rpm, right_rpm = rot, rot
                    dir_left, dir_right = 0, 1
                else: # Gira izquierda
                    left_rpm, right_rpm = rot, rot
                    dir_left, dir_right = 1, 0
            else:
                left_rpm, right_rpm = APPROACH_RPM, APPROACH_RPM
                dir_left, dir_right = 1, 1

        # D. Enviar Comandos
        cmd_timer += 1
        if cmd_timer >= 2:
            send_rpms(sender, clamp_rpm(left_rpm), clamp_rpm(right_rpm), dir_left, dir_right)
            cmd_timer = 0

        vis_timer += 1
        if vis_timer >= 5:
            send_vision_data(sender, colors, centroids, areas)
            vis_timer = 0

        # E. Mostrar Video
        video = crosslines(frame.copy())
        
        # Opcional: Escribir info de odometría en la pantalla
        if odom_state:
             cv.putText(video, "ODOM OK", (10, 230), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        else:
             cv.putText(video, "NO ODOM", (10, 230), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        cv.imshow("VISION ROVER (PC DEBUG)", video)

        if cv.waitKey(1) & 0xFF == 27:
            break

    # Cleanup
    sender.stop()
    grabber.stop()
    telemetry.stop()
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()