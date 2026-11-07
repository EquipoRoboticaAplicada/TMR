"""
test_follow_path_laptop.py

Prueba de navegación autónoma por odometría (Route_Command.follow_path)
con los ESP32 conectados directamente a la laptop por USB.

Omite a propósito la cámara ZED, el pipeline de visión y MQTT: no importa
vision_zed (pyzed) ni server (paho), así que corre sin esos SDKs instalados.

Uso (desde jetson/):
    python test_scripts/test_follow_path_laptop.py                      # cuadrado de 1 m
    python test_scripts/test_follow_path_laptop.py --path line          # 1 m recto
    python test_scripts/test_follow_path_laptop.py --path "1,0;1,1"     # ruta propia
    python test_scripts/test_follow_path_laptop.py --alpha 0.85         # fusiona IMU
    python test_scripts/test_follow_path_laptop.py --no-viz             # sin ventana

Visualizador (pygame, rover_viz.py): muestra la ruta planeada, la trayectoria
por odometría y la pose del rover. ESPACIO arranca, ESC/Q o cerrar la ventana
es PARO, S guarda una captura. Si pygame no está instalado, sigue sin ventana.

Seguridad:
    - Aborta si no detecta ESP_L y ESP_R, o si dejan de mandar telemetría.
    - Detiene el rover al terminar, al llegar a --timeout, con Ctrl+C o al
      cerrar la ventana / presionar ESC.
"""
import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import argparse
import math
import threading
import time
from types import SimpleNamespace

from connect import ESP
from odo import RoverOdometry
from util import SenderJetson
from command import Route_Command

# Si la salida se redirige a archivo/pipe en Windows (cp1252), los emojis de los
# prints truenan con UnicodeEncodeError. Con "replace" nunca rompen el flujo.
try:
    sys.stdout.reconfigure(errors="replace")
except AttributeError:
    pass

PRESETS = {
    "line":   [(1.0, 0.0)],
    "square": [(1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.0, 0.0)],
}

TELEMETRY_WAIT_S  = 3.0   # espera inicial de líneas ESP_L/ESP_R antes de arrancar
TELEMETRY_STALE_S = 1.0   # sin datos de motores por más de esto → abortar
STATUS_EVERY_S    = 0.5   # cada cuánto imprimir la pose
LOOP_SLEEP_S      = 0.05  # periodo del monitoreo cuando no hay ventana

# Solo para dibujar: mantener igual a DIST_TOLERANCE de follow_path (command.py)
WAYPOINT_TOLERANCE_M = 0.01


# ------------------------------------------------------------------ #
#  Argumentos                                                          #
# ------------------------------------------------------------------ #

def parse_path(text: str) -> list:
    if text in PRESETS:
        return PRESETS[text]
    try:
        points = []
        for pair in text.split(";"):
            x, y = pair.split(",")
            points.append((float(x), float(y)))
        return points
    except ValueError:
        raise argparse.ArgumentTypeError(
            f"Ruta inválida: {text!r}. Usa {list(PRESETS)} o 'x,y;x,y;...'"
        )


def parse_alpha(text: str) -> float:
    value = float(text)
    if not 0.0 <= value <= 1.0:
        raise argparse.ArgumentTypeError("--alpha debe estar entre 0.0 y 1.0")
    return value


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description="Prueba de follow_path con ESP32 por serial (sin ZED ni MQTT)."
    )
    p.add_argument(
        "--path", type=parse_path, default=PRESETS["square"],
        help=f"Preset {list(PRESETS)} o puntos 'x,y;x,y;...' en metros. Default: square.",
    )
    p.add_argument(
        "--alpha", type=parse_alpha, default=1.0,
        help="cf_alpha de Route_Command. 1.0 = solo odometría (default); "
             "<1.0 mezcla el heading del IMU y requiere el ESP de sensores.",
    )
    p.add_argument(
        "--timeout", type=float, default=60.0,
        help="Segundos máximos de ruta antes de detener el rover. Default: 60.",
    )
    p.add_argument(
        "--yes", action="store_true",
        help="Arrancar sin esperar confirmación (Enter o ESPACIO en la ventana).",
    )
    p.add_argument(
        "--no-viz", dest="viz", action="store_false",
        help="No abrir el visualizador pygame.",
    )
    return p.parse_args(argv)


# ------------------------------------------------------------------ #
#  Helpers                                                             #
# ------------------------------------------------------------------ #

def wait_for_telemetry(esp, timeout: float) -> bool:
    """Espera a que llegue al menos una línea ESP_L/ESP_R parseada."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if esp.get_rover_state()["last_update"] > 0:
            return True
        time.sleep(0.05)
    return False


def print_status(odo, rvr_cmd, elapsed: float):
    x, y, theta = odo.pose
    idx = rvr_cmd.current_index
    if idx < len(rvr_cmd.path):
        tx, ty = rvr_cmd.path[idx]
        target = f"→ P{idx} ({tx:.2f}, {ty:.2f}) a {math.hypot(tx - x, ty - y):.2f} m"
    else:
        target = "→ ruta completa"
    print(f"[t={elapsed:5.1f}s] pose x={x:+.3f} y={y:+.3f} θ={math.degrees(theta):+7.2f}°  {target}")


def open_viz(path, track_width_m):
    """Abre la ventana pygame. Regresa None si pygame no está disponible."""
    try:
        from rover_viz import RoverViz
    except ImportError as e:
        print(f"⚠️  Visualizador desactivado ({e}). Instálalo con: pip install pygame")
        return None
    return RoverViz(path=path, tolerance_m=WAYPOINT_TOLERANCE_M, track_width_m=track_width_m)


def feed_viz(viz, odo, rvr_cmd, alpha):
    viz.update(
        pose=odo.pose,
        target_index=rvr_cmd.current_index,
        velocity=odo.velocity,
        fused_theta=rvr_cmd._fused_theta if alpha < 1.0 else None,
    )


def fail(ctx, msg: str, rc: int = 1) -> int:
    """Reporta un aborto en consola y en la ventana (si hay)."""
    print(f"❌ {msg}")
    if ctx.viz is not None:
        ctx.viz.set_finished(msg, ok=False)
    return rc


def shutdown(esp, sender, odo):
    """Detiene el rover. El stop a motores va antes de cualquier cosa que pueda fallar."""
    # 1) Cortar el único escritor al ESP para que follow_path ya no mande comandos
    if sender is not None:
        sender.stop()
    # 2) Stop directo a ambos lados (repetido por si se pierde una línea)
    for _ in range(3):
        try:
            esp.send_uart("0", "0")
        except Exception as e:
            print(f"[shutdown] Error mandando stop: {e}")
        time.sleep(0.05)
    print("🛑 Rover detenido.")
    # 3) Detener hilo de odometría y cerrar puertos
    if odo is not None:
        odo.stop()
    esp.close()


# ------------------------------------------------------------------ #
#  Prueba                                                              #
# ------------------------------------------------------------------ #

def run(args) -> int:
    esp = ESP()
    ctx = SimpleNamespace(odo=None, sender=None, viz=None)

    try:
        rc = _run_route(args, esp, ctx)
    except KeyboardInterrupt:
        print("\nCtrl+C recibido.")
        if ctx.viz is not None:
            ctx.viz.set_finished("PARO: Ctrl+C", ok=False)
        rc = 130
    finally:
        shutdown(esp, ctx.sender, ctx.odo)

    # El rover ya está detenido: la ventana se queda con el resultado
    if ctx.viz is not None:
        try:
            print("Ventana abierta con el resultado (ESC/Q o cerrar para salir, S para captura).")
            ctx.viz.hold()
        except KeyboardInterrupt:
            pass
        finally:
            ctx.viz.close()
    return rc


def _run_route(args, esp, ctx) -> int:
    # 1. Conexión serial
    esp.connect()
    missing = [name for name, ser in (("ESP_L", esp._ser_left), ("ESP_R", esp._ser_right))
               if ser is None]
    if missing:
        return fail(ctx, f"No se detectó {', '.join(missing)}. Revisa los cables USB y el monitor serial.")
    if args.alpha < 1.0 and esp._ser_sensores is None:
        return fail(ctx, "--alpha < 1.0 necesita el ESP de sensores (IMU) y no se detectó. "
                         "Sin él, heading se queda en 0.0 y la fusión distorsiona el ángulo.")
    esp.send_uart("0", "0")

    if not wait_for_telemetry(esp, TELEMETRY_WAIT_S):
        return fail(ctx, f"Los ESP de motores no mandaron telemetría en {TELEMETRY_WAIT_S:.0f} s.")

    # 2. Odometría, sender y ruta (sin visión: el override nunca se activa)
    odo = ctx.odo = RoverOdometry(esp=esp)
    ctx.sender = SenderJetson(esp=esp).start()
    rvr_cmd = Route_Command(
        sender=ctx.sender,
        vision_override_event=threading.Event(),
        path=list(args.path),
        esp=esp,
    )
    rvr_cmd.cf_alpha = args.alpha

    print("\n──────── Prueba follow_path ────────")
    print(f"Ruta     : {rvr_cmd.path}")
    print(f"cf_alpha : {args.alpha}" + ("  (solo odometría)" if args.alpha == 1.0 else "  (fusión con IMU)"))
    print(f"Timeout  : {args.timeout:.0f} s")
    print("Origen   : pose actual del rover, mirando hacia +x")
    print("────────────────────────────────────")

    viz = ctx.viz = open_viz(rvr_cmd.path, odo.L) if args.viz else None

    # 3. Confirmación de arranque
    if not args.yes:
        if viz is not None:
            print("En la ventana: ESPACIO/Enter arranca, ESC cancela.")
            while True:
                feed_viz(viz, odo, rvr_cmd, args.alpha)
                action = viz.tick()
                if action == "start":
                    break
                if action == "quit":
                    return fail(ctx, "Cancelado desde la ventana.")
        else:
            try:
                input("Coloca el rover en el origen y presiona Enter para arrancar (Ctrl+C cancela)... ")
            except EOFError:
                return fail(ctx, "Cancelado.")

    # 4. Odometría desde cero justo al arrancar
    odo.reset_pose()
    if viz is not None:
        viz.set_running()
    route_thread = threading.Thread(target=rvr_cmd.follow_path, args=(odo,), daemon=True)
    t0 = time.time()
    route_thread.start()

    # 5. Monitoreo con watchdogs. Corre en el hilo principal (pygame lo requiere)
    #    y sin bloqueos largos para que Ctrl+C responda en Windows.
    last_status = 0.0
    while route_thread.is_alive():
        now     = time.time()
        elapsed = now - t0

        if elapsed > args.timeout:
            print_status(odo, rvr_cmd, elapsed)
            return fail(ctx, f"Timeout de {args.timeout:.0f} s alcanzado.")
        if esp._ser_left is None or esp._ser_right is None:
            return fail(ctx, "Se perdió la conexión serial con un ESP de motores.")
        age = now - esp.get_rover_state()["last_update"]
        if age > TELEMETRY_STALE_S:
            return fail(ctx, f"Telemetría de motores congelada ({age:.1f} s sin datos). "
                             "La odometría está ciega.")

        if now - last_status >= STATUS_EVERY_S:
            print_status(odo, rvr_cmd, elapsed)
            last_status = now

        if viz is not None:
            feed_viz(viz, odo, rvr_cmd, args.alpha)
            if viz.tick() == "quit":
                return fail(ctx, "PARO desde la ventana.")
        else:
            time.sleep(LOOP_SLEEP_S)

    print("✅ Ruta terminada.")
    print_status(odo, rvr_cmd, time.time() - t0)
    if viz is not None:
        feed_viz(viz, odo, rvr_cmd, args.alpha)
        viz.set_finished("Ruta terminada", ok=True)
    return 0


if __name__ == "__main__":
    sys.exit(run(parse_args()))
