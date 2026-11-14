"""
enlace.py — Comunicación con los ESP32 que corren der_sintonizacion / izq_sintonizacion.

Cada ESP32 maneja un lado del rover (3 motores). Este módulo:
  - encuentra los puertos USB y distingue ESP_L / ESP_R,
  - lee en un hilo la salida CSV del firmware (OUT CSV) y sus mensajes "# ...",
  - manda comandos con un candado (escriben varios hilos),
  - mantiene vivo el watchdog del firmware con un latido: si la laptop se
    cuelga o se desconecta el cable, el firmware pone SP = 0 solo.

No calcula ni ajusta ganancias: solo transporta comandos y datos.
"""
import re
import threading
import time
from dataclasses import dataclass

import serial
import serial.tools.list_ports

BAUD        = 115200
LADOS       = ("L", "R")
NOMBRE_LADO = {"L": "Izquierdo", "R": "Derecho"}
MOTORES     = [f"{lado}{i}" for lado in LADOS for i in range(3)]   # L0 L1 L2 R0 R1 R2
JUEGOS      = ("REF", "PERT")   # un juego de ganancias por criterio de sintonía
NOMBRE_JUEGO = {"REF": "Referencia", "PERT": "Perturbación"}
CAMPOS      = ("Kp", "Ti", "Td", "N")

WD_MS    = 500    # watchdog del firmware mientras la herramienta está conectada
LATIDO_S = 0.15   # periodo del latido ("# HB": el firmware lo cuenta como comando y lo ignora)

_RE_PID_ACK = re.compile(r"^# PID (REF|PERT) ([0-2]) (\S+) (\S+) (\S+) (\S+)$")
_RE_ZM_ACK  = re.compile(r"^# ZM ([0-2]) (\S+)$")
_RE_CSV     = re.compile(r"^\d+,[0-2],(?:M|\d+),")
_RE_PLOT    = re.compile(r"^SP\d:")


@dataclass
class Muestra:
    t_pc: float        # time.monotonic() al recibirla en la laptop
    t_esp_ms: int      # millis() del ESP32
    lado: str          # "L" | "R"
    motor: int         # 0..2
    reg: str           # "M" (manual) o porcentaje del juego REF en la mezcla, "0".."100"
    sp: float
    pv: float
    e: float
    P: float
    I: float
    D: float
    u: float

    @property
    def id(self) -> str:
        return f"{self.lado}{self.motor}"


def parsear_csv(linea: str, lado: str, t_pc: float):
    """Fila de OUT CSV: t_ms,motor,reg,SP,PV,e,P,I,D,u → Muestra (o None)."""
    partes = linea.split(",")
    if len(partes) != 10:
        return None
    try:
        return Muestra(t_pc, int(partes[0]), lado, int(partes[1]), partes[2],
                       *(float(x) for x in partes[3:]))
    except ValueError:
        return None


def fmt_num(v: float) -> str:
    """Número para mandar al firmware sin perder precisión ni usar notación rara."""
    return f"{v:.6g}"


# ---------------------------------------------------------------------- #
#  Descubrimiento de puertos                                              #
# ---------------------------------------------------------------------- #

def puertos_usb() -> list:
    """Puertos seriales USB (los ESP32 usan CP210x/CH340). Ignora Bluetooth y COM viejos."""
    return [p.device for p in serial.tools.list_ports.comports() if p.vid is not None]


def abrir_puerto(nombre: str):
    s = serial.Serial()
    s.port          = nombre
    s.baudrate      = BAUD
    s.timeout       = 0.05
    s.write_timeout = 0.5
    # Con DTR y RTS en bajo el circuito de auto-reset del ESP32 no se dispara al abrir
    s.dtr = False
    s.rts = False
    s.open()
    return s


def identificar(transporte, timeout: float = 6.0, silencio_s: float = 1.5):
    """Escucha el puerto y regresa ("L"|"R"|None, descripción).

    El firmware en OUT ESP manda tramas ESP_L/ESP_R. Si está en CSV/PLOT/OFF
    (sesión anterior sin reinicio), se le pide OUT ESP una vez para reconocerlo.
    """
    buf, pedido = b"", False
    t0 = ultimo = time.monotonic()
    while time.monotonic() - t0 < timeout:
        chunk = transporte.read(max(1, transporte.in_waiting))
        ahora = time.monotonic()
        if chunk:
            ultimo = ahora
            buf += chunk
        while b"\n" in buf:
            raw, buf = buf.split(b"\n", 1)
            linea = raw.decode("utf-8", errors="ignore").strip()
            if linea.startswith("ESP_L"):
                return "L", "ESP_L"
            if linea.startswith("ESP_R"):
                return "R", "ESP_R"
            if linea.startswith("sensores"):
                return None, "ESP de sensores"
            if not pedido and (_RE_CSV.match(linea) or _RE_PLOT.match(linea) or linea.startswith("# ")):
                transporte.write(b"OUT ESP\n")
                pedido = True
        if not pedido and ahora - ultimo > silencio_s:
            transporte.write(b"OUT ESP\n")
            pedido = True
    return None, "sin tramas ESP_L/ESP_R"


def buscar_esps(puertos=None, abrir=abrir_puerto):
    """Prueba los puertos en paralelo. Regresa ({lado: (transporte, puerto)}, [avisos])."""
    puertos = list(puertos) if puertos else puertos_usb()
    resultados, avisos, lock = {}, [], threading.Lock()

    def probar(puerto):
        try:
            t = abrir(puerto)
        except Exception as e:
            with lock:
                avisos.append(f"{puerto}: no se pudo abrir ({e})")
            return
        lado, desc = identificar(t)
        with lock:
            if lado is None or lado in resultados:
                avisos.append(f"{puerto}: {desc if lado is None else 'lado ' + lado + ' repetido'}")
                t.close()
            else:
                resultados[lado] = (t, puerto)

    hilos = [threading.Thread(target=probar, args=(p,), daemon=True) for p in puertos]
    for h in hilos:
        h.start()
    for h in hilos:
        h.join()
    if not puertos:
        avisos.append("No hay puertos USB seriales")
    return resultados, avisos


# ---------------------------------------------------------------------- #
#  Enlace con un ESP32                                                    #
# ---------------------------------------------------------------------- #

class EnlaceESP:

    def __init__(self, transporte, lado: str, puerto: str, al_recibir_muestra, al_recibir_mensaje):
        """
        al_recibir_muestra(Muestra)   — se llama desde el hilo lector
        al_recibir_mensaje(lado, txt) — mensajes "# ..." del firmware y avisos del enlace
        """
        self.t       = transporte
        self.lado    = lado
        self.puerto  = puerto
        self._on_muestra = al_recibir_muestra
        self._on_msg     = al_recibir_mensaje

        self._lock_tx = threading.Lock()
        self._vivo    = threading.Event()
        self._vivo.set()

        self.ultima_muestra_t = 0.0
        self.acks_pid = {}      # (juego, motor) → (Kp, Ti, Td, N) confirmados por el firmware
        self.acks_zm  = {}      # motor → zona muerta [PWM] confirmada por el firmware
        self.error    = None

        self._hilo_rx     = threading.Thread(target=self._leer, daemon=True)
        self._hilo_latido = threading.Thread(target=self._latir, daemon=True)

    def iniciar(self):
        self._hilo_rx.start()
        self._hilo_latido.start()
        return self

    @property
    def conectado(self) -> bool:
        return self._vivo.is_set() and self.error is None

    def edad_datos(self) -> float:
        """Segundos desde la última fila CSV (inf si nunca llegó una)."""
        if self.ultima_muestra_t == 0.0:
            return float("inf")
        return time.monotonic() - self.ultima_muestra_t

    # ---- Envío ----
    def enviar(self, linea: str) -> bool:
        if not self._vivo.is_set():
            return False
        try:
            with self._lock_tx:
                self.t.write((linea + "\n").encode("utf-8"))
            return True
        except Exception as e:
            self._fallar(f"error escribiendo: {e}")
            return False

    def configurar(self, ganancias=None, cfg=None):
        """Deja el firmware listo para la herramienta y, si se dan, le carga las ganancias.

        ganancias: {"L0": {"REF": {"Kp":..,"Ti":..,"Td":..,"N":..}, "PERT": {...}, "ZM": pwm}, ...}
        cfg: {"UMAX":.., "TS":.., "BANDA":.., "TMEZ":.., "DMODE": "E"|"PV"}
        """
        for linea in ("STOP", f"WD {WD_MS}", "VER *"):
            self.enviar(linea)
        if ganancias is not None and cfg is not None:
            self.enviar_ganancias(ganancias, cfg)
        self.enviar("OUT CSV")

    def enviar_ganancias(self, ganancias: dict, cfg: dict):
        self.enviar(f"UMAX {fmt_num(cfg['UMAX'])}")
        self.enviar(f"TS {int(cfg['TS'])}")
        self.enviar(f"BANDA {fmt_num(cfg['BANDA'])}")
        self.enviar(f"TMEZ {int(cfg['TMEZ'])}")
        self.enviar(f"DMODE {cfg['DMODE']}")
        for i in range(3):
            g_motor = ganancias[f"{self.lado}{i}"]
            self.acks_zm.pop(i, None)
            self.enviar(f"ZM {i} {fmt_num(g_motor['ZM'])}")
            for juego in JUEGOS:
                self.acks_pid.pop((juego, i), None)
                g = g_motor[juego]
                self.enviar(f"PID {juego} {i} " + " ".join(fmt_num(g[c]) for c in CAMPOS))

    def verificar_ganancias(self, ganancias: dict) -> dict:
        """{(motor_id, juego o "ZM"): True | False | None (aún sin confirmación)}"""
        res = {}
        for i in range(3):
            mid = f"{self.lado}{i}"
            zm = self.acks_zm.get(i)      # el firmware la confirma con 1 decimal
            res[(mid, "ZM")] = None if zm is None else abs(zm - ganancias[mid]["ZM"]) <= 0.051
            for juego in JUEGOS:
                ack = self.acks_pid.get((juego, i))
                if ack is None:
                    res[(mid, juego)] = None
                    continue
                esperado = [ganancias[mid][juego][c] for c in CAMPOS]
                # El firmware confirma Kp/Ti/Td con 4 decimales y N con 1
                tol = (1e-4, 1e-4, 1e-4, 0.051)
                res[(mid, juego)] = all(abs(a - b) <= t + 1e-4 * abs(b)
                                        for a, b, t in zip(ack, esperado, tol))
        return res

    # ---- Hilos ----
    def _leer(self):
        buf = b""
        while self._vivo.is_set():
            try:
                chunk = self.t.read(max(1, self.t.in_waiting))
            except Exception as e:
                self._fallar(f"error leyendo: {e}")
                return
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                self._procesar(raw.decode("utf-8", errors="ignore").strip(), time.monotonic())

    def _procesar(self, linea: str, t_pc: float):
        if not linea:
            return
        if linea[0].isdigit():
            m = parsear_csv(linea, self.lado, t_pc)
            if m is not None:
                self.ultima_muestra_t = t_pc
                self._on_muestra(m)
            return
        if linea.startswith("# "):
            ack = _RE_PID_ACK.match(linea)
            if ack:
                try:
                    self.acks_pid[(ack.group(1), int(ack.group(2)))] = tuple(float(ack.group(k)) for k in range(3, 7))
                except ValueError:
                    pass
            ack = _RE_ZM_ACK.match(linea)
            if ack:
                try:
                    self.acks_zm[int(ack.group(1))] = float(ack.group(2))
                except ValueError:
                    pass
            self._on_msg(self.lado, linea[2:])
            return
        if linea.startswith(("ESP_L", "ESP_R", "t_ms,")) or _RE_PLOT.match(linea):
            return   # tramas de otros modos de salida y encabezado CSV
        self._on_msg(self.lado, f"» {linea}")

    def _latir(self):
        while self._vivo.is_set():
            self.enviar("# HB")
            time.sleep(LATIDO_S)

    def _fallar(self, motivo: str):
        if self.error is None:
            self.error = motivo
            self._on_msg(self.lado, f"[enlace] {motivo}")
        self._vivo.clear()

    def cerrar(self, restaurar: bool = True):
        """restaurar=True: STOP, watchdog apagado y salida ESP, para que connect.py
        lo reconozca sin reiniciar la placa."""
        if restaurar and self._vivo.is_set():
            for linea in ("STOP", "WD 0", "OUT ESP"):
                self.enviar(linea)
            time.sleep(0.05)
        self._vivo.clear()
        try:
            self.t.close()
        except Exception:
            pass
