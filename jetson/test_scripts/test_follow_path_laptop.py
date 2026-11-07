"""
test_follow_path_laptop.py

Prueba de navegación autónoma por odometría (Route_Command.follow_path)
con los ESP32 conectados directamente a la laptop por USB.

Omite a propósito la cámara ZED, el pipeline de visión y MQTT: no importa
vision_zed (pyzed) ni server (paho), así que corre sin esos SDKs instalados.

Uso (desde jetson/):
    python test_scripts/test_follow_path_laptop.py                      # 1 m recto
    python test_scripts/test_follow_path_laptop.py --path square        # cuadrado de 1 m
    python test_scripts/test_follow_path_laptop.py --path "1,0;1,1"     # ruta propia
    python test_scripts/test_follow_path_laptop.py --alpha 0.85         # fusiona IMU

Seguridad:
    - Aborta si no detecta ESP_L y ESP_R, o si dejan de mandar telemetría.
    - Detiene el rover al terminar, al llegar a --timeout o con Ctrl+C.
"""
import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import argparse
import math
import threading
import time

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
        "--path", type=parse_path, default=PRESETS["line"],
        help=f"Preset {list(PRESETS)} o puntos 'x,y;x,y;...' en metros. Default: line.",
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
        help="Arrancar sin esperar Enter.",
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
    esp    = ESP()
    odo    = None
    sender = None

    try:
        # 1. Conexión serial
        esp.connect()
        missing = [name for name, ser in (("ESP_L", esp._ser_left), ("ESP_R", esp._ser_right))
                   if ser is None]
        if missing:
            print(f"❌ No se detectó {', '.join(missing)}. Revisa los cables USB y el monitor serial.")
            return 1
        if args.alpha < 1.0 and esp._ser_sensores is None:
            print("❌ --alpha < 1.0 necesita el ESP de sensores (IMU) y no se detectó.\n"
                  "   Sin él, heading se queda en 0.0 y la fusión distorsiona el ángulo.")
            return 1
        esp.send_uart("0", "0")

        if not wait_for_telemetry(esp, TELEMETRY_WAIT_S):
            print(f"❌ Los ESP de motores no mandaron telemetría en {TELEMETRY_WAIT_S:.0f} s.")
            return 1

        # 2. Odometría, sender y ruta (sin visión: el override nunca se activa)
        odo    = RoverOdometry(esp=esp)
        sender = SenderJetson(esp=esp).start()
        rvr_cmd = Route_Command(
            sender=sender,
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

        if not args.yes:
            try:
                input("Coloca el rover en el origen y presiona Enter para arrancar (Ctrl+C cancela)... ")
            except EOFError:
                print("Cancelado.")
                return 1

        # 3. Odometría desde cero justo al arrancar
        odo.reset_pose()
        route_thread = threading.Thread(target=rvr_cmd.follow_path, args=(odo,), daemon=True)
        t0 = time.time()
        route_thread.start()

        # 4. Monitoreo con watchdogs (loop con sleep para que Ctrl+C responda en Windows)
        last_status = 0.0
        while route_thread.is_alive():
            now     = time.time()
            elapsed = now - t0

            if elapsed > args.timeout:
                print(f"⏱️  Timeout de {args.timeout:.0f} s alcanzado.")
                print_status(odo, rvr_cmd, elapsed)
                return 1
            if esp._ser_left is None or esp._ser_right is None:
                print("❌ Se perdió la conexión serial con un ESP de motores.")
                return 1
            age = now - esp.get_rover_state()["last_update"]
            if age > TELEMETRY_STALE_S:
                print(f"❌ Telemetría de motores congelada ({age:.1f} s sin datos). La odometría está ciega.")
                return 1

            if now - last_status >= STATUS_EVERY_S:
                print_status(odo, rvr_cmd, elapsed)
                last_status = now
            time.sleep(0.05)

        print("✅ Ruta terminada.")
        print_status(odo, rvr_cmd, time.time() - t0)
        return 0

    except KeyboardInterrupt:
        print("\nCtrl+C recibido.")
        return 130
    finally:
        shutdown(esp, sender, odo)


if __name__ == "__main__":
    sys.exit(run(parse_args()))
