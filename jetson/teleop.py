"""
teleop.py

Módulo de teleoperación de baja latencia con control DualSense (PS5) para el Rover Lunar.
Soporta motores de giro diferencial por eje planetario (ESP_L y ESP_R) sin alterar
la odometría (RoverOdometry) ni la navegación autónoma (Route_Command).

Modos de ejecución:
  1. Remoto por MQTT (Estación Base -> Jetson, optimizado para larga distancia):
       python teleop.py --broker 172.32.166.50
  2. Local con ESP32 por USB (y ruta autónoma + visualizador opcional):
       python teleop.py --local
  3. Simulación local en Laptop (sin hardware, con visualizador 2D):
       python teleop.py --sim
  4. Diagnóstico de botones y ejes del DualSense:
       python teleop.py --diag
"""

import argparse
import json
import math
import os
import sys
import threading
import time

# Permitir importar módulos hermanos cuando se ejecuta desde cualquier directorio
sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))


def apply_deadzone(value: float, deadzone: float = 0.10) -> float:
    """Aplica zona muerta re-escalando suavemente el rango restante a [0.0, 1.0]."""
    if abs(value) < deadzone:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    scaled = (abs(value) - deadzone) / (1.0 - deadzone)
    return sign * min(1.0, max(0.0, scaled))


def compute_differential_rpm(
    throttle: float,
    steer: float,
    max_rpm: int,
    steer_expo: float = 0.35,
) -> tuple[int, int]:
    """
    Convierte aceleración lineal (throttle ∈ [-1, 1]) y giro (steer ∈ [-1, 1])
    en setpoints diferenciales (left_rpm, right_rpm) para los 2 ESP32.

    Convención del rover (ver command.py):
      - Avanzar recto      : L = +RPM, R = +RPM
      - Giro a la derecha  : L = +RPM, R = -RPM  (steer > 0)
      - Giro a la izquierda: L = -RPM, R = +RPM  (steer < 0)

    steer_expo mezcla respuesta lineal y cúbica para dar precisión fina cerca
    del centro sin perder giro máximo en los extremos.
    """
    curved_steer = (1.0 - steer_expo) * steer + steer_expo * (steer ** 3)

    left_norm  = max(-1.0, min(1.0, throttle + curved_steer))
    right_norm = max(-1.0, min(1.0, throttle - curved_steer))

    left_rpm  = int(round(left_norm  * max_rpm))
    right_rpm = int(round(right_norm * max_rpm))
    return left_rpm, right_rpm


class SlewRateLimiter:
    """
    Limita el cambio máximo de RPM por segundo para proteger las cajas planetarias
    contra inversiones bruscas de par, manteniendo respuesta inmediata al frenar a 0.
    """

    def __init__(self, max_delta_rpm_per_sec: float = 120.0):
        self.max_rate = max_delta_rpm_per_sec
        self._l = 0.0
        self._r = 0.0
        self._last_t = time.time()

    def reset(self):
        self._l = 0.0
        self._r = 0.0
        self._last_t = time.time()

    def step(self, target_l: int, target_r: int) -> tuple[int, int]:
        now = time.time()
        dt = max(0.001, min(0.2, now - self._last_t))
        self._last_t = now

        # Si el objetivo es 0, permitir frenado inmediato por seguridad
        if target_l == 0 and target_r == 0:
            self._l = 0.0
            self._r = 0.0
            return 0, 0

        max_step = self.max_rate * dt
        for attr, target in (("_l", float(target_l)), ("_r", float(target_r))):
            curr = getattr(self, attr)
            diff = target - curr
            if abs(diff) <= max_step:
                setattr(self, attr, target)
            else:
                setattr(self, attr, curr + math.copysign(max_step, diff))

        return int(round(self._l)), int(round(self._r))


class DualSenseReader:
    """
    Lee un control DualSense (PS5) o compatible vía pygame.joystick.
    Incluye detección segura de gatillos L2/R2 en Linux/Windows (evita el bug
    de gatillo en 0.0 antes de tocarse por primera vez).
    """

    def __init__(
        self,
        deadzone: float = 0.10,
        drive_mode: str = "arcade_triggers",
        l2_axis: int = None,
        r2_axis: int = None,
    ):
        self.deadzone   = deadzone
        self.drive_mode = drive_mode  # "arcade_triggers", "arcade_stick", "tank"
        self.joystick   = None
        self._pygame    = None

        self._l2_axis = l2_axis
        self._r2_axis = r2_axis
        self._trigger_calibrated = {}  # axis_idx -> bool (True cuando ya reportó reposo -1.0)
        self._prev_buttons = {}

    def connect(self) -> bool:
        try:
            import pygame
            self._pygame = pygame
        except ImportError:
            print("❌ pygame no está instalado. Instálalo con: pip install pygame")
            return False

        if not self._pygame.get_init():
            self._pygame.init()
        if not self._pygame.joystick.get_init():
            self._pygame.joystick.init()

        count = self._pygame.joystick.get_count()
        if count == 0:
            return False

        self.joystick = self._pygame.joystick.Joystick(0)
        self.joystick.init()
        name = self.joystick.get_name()
        num_axes = self.joystick.get_numaxes()
        num_btns = self.joystick.get_numbuttons()

        # Auto-detección de ejes de gatillos L2 / R2 si no se especificaron
        if self._l2_axis is None or self._r2_axis is None:
            # En SDL2/pygame estándar con 6 ejes: L2=4 (o 2 en hid-playstation legacy), R2=5
            self._l2_axis = 4 if num_axes >= 6 else 2
            self._r2_axis = 5 if num_axes >= 6 else 3

        print(f"🎮 Control conectado: '{name}' ({num_axes} ejes, {num_btns} botones)")
        return True

    def _read_trigger(self, axis_idx: int) -> float:
        """Retorna valor del gatillo normalizado en [0.0, 1.0]."""
        if self.joystick is None or axis_idx >= self.joystick.get_numaxes():
            return 0.0
        raw = self.joystick.get_axis(axis_idx)

        # En Linux, antes de tocar el gatillo por primera vez, el kernel reporta 0.0
        # en lugar de -1.0. Solo activamos el mapeo [-1, 1] -> [0, 1] cuando sale de 0.0.
        if not self._trigger_calibrated.get(axis_idx, False):
            if abs(raw) > 0.05:
                self._trigger_calibrated[axis_idx] = True
            else:
                return 0.0

        norm = (raw + 1.0) * 0.5
        return 0.0 if norm < self.deadzone else min(1.0, (norm - self.deadzone) / (1.0 - self.deadzone))

    def poll(self) -> dict:
        """
        Lee el estado actual del control y retorna:
          - throttle, steer (para modo arcade)
          - left_tank, right_tank (para modo tanque)
          - edges de botones presionados en este ciclo
        """
        if self._pygame is None:
            return {}

        self._pygame.event.pump()

        if self._pygame.joystick.get_count() == 0:
            if self.joystick is not None:
                print("⚠️ Control DualSense desconectado.")
                self.joystick = None
            return {"connected": False}

        if self.joystick is None:
            self.connect()
            if self.joystick is None:
                return {"connected": False}

        num_axes = self.joystick.get_numaxes()
        lx = apply_deadzone(self.joystick.get_axis(0) if num_axes > 0 else 0.0, self.deadzone)
        ly = apply_deadzone(-self.joystick.get_axis(1) if num_axes > 1 else 0.0, self.deadzone)

        # En DualSense con 6 ejes, el stick derecho vertical suele ser eje 3 (o 4)
        ry_axis = 3 if num_axes >= 6 else 2
        ry = apply_deadzone(-self.joystick.get_axis(ry_axis) if num_axes > ry_axis else 0.0, self.deadzone)

        l2 = self._read_trigger(self._l2_axis)
        r2 = self._read_trigger(self._r2_axis)

        # Si los gatillos se presionan usan R2 - L2; si están sueltos permite usar Stick Izquierdo Y
        trigger_throttle = r2 - l2
        if self.drive_mode == "arcade_stick":
            throttle = ly
        elif abs(trigger_throttle) > 0.01:
            throttle = trigger_throttle
        else:
            throttle = ly

        steer = lx

        # Detectar flancos de subida (button_down) para acciones discretas
        pressed_edges = set()
        num_btns = self.joystick.get_numbuttons()
        for b in range(num_btns):
            curr = bool(self.joystick.get_button(b))
            prev = self._prev_buttons.get(b, False)
            if curr and not prev:
                pressed_edges.add(b)
            self._prev_buttons[b] = curr

        return {
            "connected": True,
            "throttle": throttle,
            "steer": steer,
            "left_tank": ly,
            "right_tank": ry,
            "l2": l2,
            "r2": r2,
            "lx": lx,
            "ly": ly,
            "pressed": pressed_edges,
        }


class DualSenseTeleop:
    """
    Gestor de teleoperación con DualSense.
    Puede enviar comandos directamente a instancias locales (SenderJetson / Route_Command / ESP)
    o publicar por MQTT con mínima latencia (QoS 0 + filtro delta + heartbeat).
    """

    # Índices de botones compatibles con DualSense en SDL2 / Linux hid-playstation
    BTN_CROSS    = {0}          # X / Cruz: Accionar brazo
    BTN_CIRCLE   = {1}          # O / Círculo: Paro de Emergencia (E-Stop)
    BTN_TRIANGLE = {2, 3}       # Triángulo: Reanudar ruta autónoma (resume_route)
    BTN_L1       = {4}          # L1: Bajar tope de RPM (-5 RPM)
    BTN_R1       = {5}          # R1: Subir tope de RPM (+5 RPM)
    BTN_SHARE    = {8}          # Share / Create: Resetear pose (0,0,0)
    BTN_OPTIONS  = {9, 6, 7}    # Options: Alternar Modo Manual <-> Standby

    def __init__(
        self,
        sender=None,
        rvr_cmd=None,
        odo=None,
        esp=None,
        mqtt_client=None,
        max_rpm: int = 20,
        drive_mode: str = "arcade_triggers",
        deadzone: float = 0.10,
        rate_hz: float = 40.0,
        heartbeat_s: float = 0.15,
    ):
        self.sender      = sender
        self.rvr_cmd     = rvr_cmd
        self.odo         = odo
        self.esp         = esp
        self.mqtt_client = mqtt_client

        self.max_rpm     = max_rpm
        self.drive_mode  = drive_mode
        self.rate_hz     = rate_hz
        self.heartbeat_s = heartbeat_s

        self.reader      = DualSenseReader(deadzone=deadzone, drive_mode=drive_mode)
        self.limiter     = SlewRateLimiter(max_delta_rpm_per_sec=150.0)

        self.manual_active = False
        self._seq          = 0
        self._last_sent    = (None, None)
        self._last_tx_ts   = 0.0

        self.stop_event = threading.Event()
        self.thread     = None

    def start(self):
        """Inicia el bucle de teleoperación en un hilo daemon."""
        self.thread = threading.Thread(target=self.run_loop, daemon=True)
        self.thread.start()
        return self

    def stop(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=1.0)

    # ------------------------------------------------------------------ #
    #  Acciones de estado y transmisión                                   #
    # ------------------------------------------------------------------ #

    def set_manual_mode(self, enabled: bool):
        self.manual_active = enabled
        self.limiter.reset()
        self._last_sent = (None, None)

        if self.sender is not None:
            self.sender.set_manual_mode(enabled)
        if self.rvr_cmd is not None and enabled:
            self.rvr_cmd.pause_route()
        if self.mqtt_client is not None:
            self.mqtt_client.publish(
                "rover/commands",
                json.dumps({"action": "set_mode", "manual": enabled}),
                qos=1,
            )

        estado = "🎮 MANUAL (DualSense activo)" if enabled else "⏸️ STANDBY (Rover detenido, esperando orden)"
        print(f"\n[Teleop] Modo cambiado a: {estado}")

    def trigger_emergency_stop(self):
        self.manual_active = True
        self.limiter.reset()
        self._last_sent = (0, 0)

        if self.sender is not None:
            self.sender.emergency_stop()
        if self.rvr_cmd is not None:
            self.rvr_cmd.pause_route()
        if self.mqtt_client is not None:
            self.mqtt_client.publish(
                "rover/commands",
                json.dumps({"action": "emergency_stop"}),
                qos=1,
            )
        print("\n🛑 [Teleop] ¡PARO DE EMERGENCIA ACTIVADO! Motores en 0 RPM.")

    def trigger_resume_route(self):
        if self.manual_active:
            self.set_manual_mode(False)
        if self.rvr_cmd is not None:
            self.rvr_cmd.resume_route()
        if self.mqtt_client is not None:
            self.mqtt_client.publish(
                "rover/commands",
                json.dumps({"action": "resume_route"}),
                qos=1,
            )
        print("\n▶️ [Teleop] Reanudando navegación autónoma por odometría.")

    def trigger_reset_pose(self):
        if self.odo is not None:
            self.odo.reset_pose()
        if self.rvr_cmd is not None:
            self.rvr_cmd.reset_path()
        if self.mqtt_client is not None:
            self.mqtt_client.publish(
                "rover/commands",
                json.dumps({"action": "reset_pose"}),
                qos=1,
            )
        print("\n🔄 [Teleop] Pose de odometría y ruta reiniciadas a (0, 0, 0).")

    def trigger_arm(self):
        if self.esp is not None:
            self.esp.act_arm()
        if self.mqtt_client is not None:
            self.mqtt_client.publish(
                "rover/commands",
                json.dumps({"action": "act_arm"}),
                qos=1,
            )
        print("\n🦾 [Teleop] Comando de brazo enviado (B0).")

    def _dispatch_rpm(self, left_rpm: int, right_rpm: int):
        """
        Envía setpoints (left_rpm, right_rpm) aplicando filtro Delta + Heartbeat
        para minimizar la latencia y evitar saturar el enlace de radio/Wi-Fi.
        """
        now = time.time()
        changed = (left_rpm, right_rpm) != self._last_sent
        heartbeat_due = (now - self._last_tx_ts) >= self.heartbeat_s

        if not (changed or heartbeat_due):
            return

        self._seq += 1
        self._last_sent = (left_rpm, right_rpm)
        self._last_tx_ts = now

        if self.sender is not None:
            self.sender.send_manual(left_rpm, right_rpm, max_rpm=self.max_rpm)

        if self.mqtt_client is not None:
            # Payload ultracompacto con QoS 0 para mínima latencia de red
            pkt = json.dumps(
                {"l": left_rpm, "r": right_rpm, "s": self._seq, "max": self.max_rpm},
                separators=(",", ":"),
            )
            self.mqtt_client.publish("rover/teleop", pkt, qos=0)

    # ------------------------------------------------------------------ #
    #  Bucle principal de lectura                                         #
    # ------------------------------------------------------------------ #

    def run_loop(self):
        interval = 1.0 / self.rate_hz
        self.reader.connect()

        was_connected = False
        while not self.stop_event.is_set():
            t0 = time.time()
            state = self.reader.poll()

            if not state.get("connected", False):
                if was_connected and self.manual_active:
                    # Si el control se desconecta en modo manual, frenar de inmediato
                    self._dispatch_rpm(0, 0)
                was_connected = False
                time.sleep(0.2)
                continue

            was_connected = True
            pressed = state["pressed"]

            # 1. Procesar botones discretos
            if pressed & self.BTN_CIRCLE:
                self.trigger_emergency_stop()
            elif pressed & self.BTN_OPTIONS:
                self.set_manual_mode(not self.manual_active)
            elif pressed & self.BTN_TRIANGLE:
                self.trigger_resume_route()
            elif pressed & self.BTN_SHARE:
                self.trigger_reset_pose()
            elif pressed & self.BTN_CROSS:
                self.trigger_arm()
            elif pressed & self.BTN_L1:
                self.max_rpm = max(5, self.max_rpm - 5)
                print(f"\n🔽 [Teleop] Límite MAX_RPM ajustado a: {self.max_rpm} RPM")
            elif pressed & self.BTN_R1:
                self.max_rpm = min(60, self.max_rpm + 5)
                print(f"\n🔼 [Teleop] Límite MAX_RPM ajustado a: {self.max_rpm} RPM")

            # 2. Si el operador mueve los gatillos/sticks estando en Standby,
            #    o si el modo manual está activo, calcular y enviar RPMs
            if self.manual_active:
                if self.drive_mode == "tank":
                    raw_l = int(round(state["left_tank"] * self.max_rpm))
                    raw_r = int(round(state["right_tank"] * self.max_rpm))
                else:
                    raw_l, raw_r = compute_differential_rpm(
                        throttle=state["throttle"],
                        steer=state["steer"],
                        max_rpm=self.max_rpm,
                    )

                l_rpm, r_rpm = self.limiter.step(raw_l, raw_r)
                self._dispatch_rpm(l_rpm, r_rpm)

            elapsed = time.time() - t0
            if interval - elapsed > 0:
                time.sleep(interval - elapsed)


# ------------------------------------------------------------------ #
#  Modos CLI (Diagnóstico, Remoto MQTT y Local/Simulación)           #
# ------------------------------------------------------------------ #

def run_diag():
    """Imprime en tiempo real los ejes y botones del DualSense para verificación rápida."""
    reader = DualSenseReader(deadzone=0.05)
    if not reader.connect():
        print("⚠️ Conecta tu control DualSense por USB o Bluetooth e intenta de nuevo.")
        return 1

    print("\n─── Diagnóstico DualSense (Ctrl+C para salir) ───")
    try:
        while True:
            st = reader.poll()
            if st.get("connected"):
                l_rpm, r_rpm = compute_differential_rpm(st["throttle"], st["steer"], max_rpm=25)
                btns = sorted(list(st["pressed"])) if st["pressed"] else "-"
                print(
                    f"\rL2:{st['l2']:.2f} R2:{st['r2']:.2f} | "
                    f"LX:{st['lx']:+.2f} LY:{st['ly']:+.2f} | "
                    f"Throttle:{st['throttle']:+.2f} Steer:{st['steer']:+.2f} -> "
                    f"ESP_L:{l_rpm:+3d} RPM  ESP_R:{r_rpm:+3d} RPM | Btns:{btns}   ",
                    end="",
                    flush=True,
                )
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nDiagnóstico finalizado.")
    return 0


def run_remote_mqtt(args):
    """Ejecuta el cliente de teleoperación en la Laptop enviando por MQTT a la Jetson."""
    import paho.mqtt.client as mqtt

    client = mqtt.Client(
        callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
        client_id="BaseStation_DualSense",
    )
    # Last Will: si la laptop pierde conexión o se cierra el script, frena el rover
    client.will_set(
        "rover/commands",
        json.dumps({"action": "emergency_stop"}),
        qos=1,
        retain=False,
    )

    print(f"📡 Conectando al Broker MQTT en {args.broker}:{args.port}...")
    client.connect(args.broker, args.port, keepalive=15)
    client.loop_start()

    teleop = DualSenseTeleop(
        mqtt_client=client,
        max_rpm=args.max_rpm,
        drive_mode=args.mode,
    )
    # Arrancar directamente en modo manual cuando se invoca el cliente remoto
    teleop.set_manual_mode(True)

    print("\n══════════ CONTROLES DUALSENSE (REMOTO MQTT) ══════════")
    print("  • R2 / L2              : Acelerar adelante / Reversa")
    print("  • Stick Izquierdo (X)  : Giro diferencial (Izquierda / Derecha)")
    print("  • Botón Options        : Alternar Modo MANUAL <-> STANDBY")
    print("  • Botón Triángulo (△)  : Reanudar ruta autónoma por odometría")
    print("  • Botón Círculo (O)    : PARO DE EMERGENCIA (0 RPM)")
    print("  • Botón Cruz (X)       : Accionar brazo (B0)")
    print("  • L1 / R1              : Bajar / Subir límite MAX_RPM")
    print("═══════════════════════════════════════════════════════\n")

    try:
        teleop.run_loop()
    except KeyboardInterrupt:
        print("\nCerrando teleoperación remota...")
    finally:
        teleop.set_manual_mode(False)
        time.sleep(0.1)
        client.loop_stop()
        client.disconnect()
    return 0


class SimulatedESP:
    """
    Simulador físico ligero de los 2 ESP32 (ESP_L y ESP_R) que responde a send_uart()
    y alimenta get_rover_state() / get_sensor_state() para probar teleop + odometría sin hardware.
    """

    WHEEL_CIRC_M = math.pi * 0.17

    def __init__(self):
        from connect import ESP
        self._esp = ESP()
        self._sp_l = 0.0
        self._sp_r = 0.0
        self._seq  = 0
        self._stop = threading.Event()
        self._thread = None

    def connect(self):
        self._thread = threading.Thread(target=self._sim_loop, daemon=True)
        self._thread.start()
        print("🧪 SimulatedESP conectado (cinemática diferencial activa a 20 Hz).")

    def send_uart(self, left_value: str, right_value: str):
        self._sp_l = float(left_value)
        self._sp_r = float(right_value)

    def get_rover_state(self) -> dict:
        return self._esp.get_rover_state()

    def get_sensor_state(self) -> dict:
        return self._esp.get_sensor_state()

    def act_arm(self):
        print("🦾 [SimulatedESP] Brazo accionado (B0).")

    def _sim_loop(self):
        while not self._stop.is_set():
            self._seq += 1
            v_l = (self._sp_l / 60.0) * self.WHEEL_CIRC_M
            v_r = (self._sp_r / 60.0) * self.WHEEL_CIRC_M
            line_l = f"ESP_L,{self._seq},{self._sp_l:.2f},{v_l:.4f},{self._sp_l:.2f},{v_l:.4f},{self._sp_l:.2f},{v_l:.4f}"
            line_r = f"ESP_R,{self._seq},{self._sp_r:.2f},{v_r:.4f},{self._sp_r:.2f},{v_r:.4f},{self._sp_r:.2f},{v_r:.4f}"
            self._esp._parse_esp_line_m(line_l)
            self._esp._parse_esp_line_m(line_r)
            time.sleep(0.05)

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)


def run_local_or_sim(args):
    """
    Ejecuta el bucle completo localmente (con ESP real o SimulatedESP) junto con
    RoverOdometry, Route_Command y la ventana 2D RoverViz si está disponible.
    """
    from odo import RoverOdometry
    from util import SenderJetson
    from command import Route_Command

    if args.sim:
        esp = SimulatedESP()
    else:
        from connect import ESP
        esp = ESP()

    esp.connect()
    esp.send_uart("0", "0")

    odo = RoverOdometry(esp=esp)
    sender = SenderJetson(esp=esp).start()
    square_path = [(1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.0, 0.0)]
    rvr_cmd = Route_Command(
        sender=sender,
        vision_override_event=threading.Event(),
        path=square_path,
        esp=esp,
    )

    teleop = DualSenseTeleop(
        sender=sender,
        rvr_cmd=rvr_cmd,
        odo=odo,
        esp=esp,
        max_rpm=args.max_rpm,
        drive_mode=args.mode,
    )
    teleop.set_manual_mode(True)
    teleop.start()

    # Hilo de ruta autónoma (inicia en Standby porque manual_mode=True)
    threading.Thread(target=rvr_cmd.follow_path, args=(odo,), daemon=True).start()

    viz = None
    if not args.no_viz:
        try:
            sys.path.insert(0, os.path.join(os.path.dirname(__file__), "test_scripts"))
            from rover_viz import RoverViz
            viz = RoverViz(path=square_path, tolerance_m=0.05, track_width_m=odo.L)
        except Exception as e:
            print(f"⚠️ Visualizador no disponible ({e}). Corriendo en consola.")

    print("\n══════════ TELEOP LOCAL + ODOMETRÍA ACTIVA ══════════")
    print("  • R2 / L2 + Stick Izq  : Conducir manualmente")
    print("  • Botón Options        : Alternar MANUAL <-> STANDBY")
    print("  • Botón Triángulo (△)  : Reanudar ruta cuadrada autónoma")
    print("  • Botón Círculo (O)    : PARO DE EMERGENCIA")
    print("═════════════════════════════════════════════════════\n")

    try:
        while True:
            if viz is not None:
                if not viz.process_events():
                    break
                viz.update(
                    pose=odo.pose,
                    target_index=rvr_cmd.current_index,
                    velocity=odo.velocity,
                )
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nDeteniendo prueba local...")
    finally:
        teleop.stop()
        sender.stop()
        esp.send_uart("0", "0")
        odo.stop()
        esp.close()
        if viz is not None:
            viz.close()
    return 0


def parse_args(argv=None):
    p = argparse.ArgumentParser(description="Teleoperación con DualSense (PS5) para Rover Lunar.")
    group = p.add_mutually_exclusive_group()
    group.add_argument("--broker", type=str, default="127.0.0.1", help="IP del broker MQTT para modo remoto (default: 127.0.0.1).")
    group.add_argument("--local", action="store_true", help="Conectar directamente a los ESP32 por USB local.")
    group.add_argument("--sim", action="store_true", help="Ejecutar con SimulatedESP y visualizador 2D.")
    group.add_argument("--diag", action="store_true", help="Modo diagnóstico para ver ejes y botones del DualSense.")

    p.add_argument("--port", type=int, default=1883, help="Puerto del broker MQTT (default: 1883).")
    p.add_argument("--max-rpm", type=int, default=20, help="RPM máximo en modo manual (default: 20).")
    p.add_argument(
        "--mode",
        choices=["arcade_triggers", "arcade_stick", "tank"],
        default="arcade_triggers",
        help="Mapeo de conducción (default: arcade_triggers).",
    )
    p.add_argument("--no-viz", action="store_true", help="Desactivar ventana pygame en modo --local / --sim.")
    return p.parse_args(argv)


if __name__ == "__main__":
    args = parse_args()
    if args.diag:
        sys.exit(run_diag())
    elif args.local or args.sim:
        sys.exit(run_local_or_sim(args))
    else:
        sys.exit(run_remote_mqtt(args))
