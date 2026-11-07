"""
experimento.py — Datos en vivo de los 6 motores, experimentos que dispara el
usuario y su registro en disco.

Un experimento es un perfil fijo que el usuario lanza a mano:

        valor inicial (t_antes) → escalón (t_escalon) → valor inicial (t_despues) → STOP

  - tipo "PWM": lazo abierto (comando MAN del firmware), para identificar cada motor.
    La subida da la respuesta de aceleración y la bajada la de frenado.
  - tipo "SP":  lazo cerrado (setpoint), para revisar la sintonía con las ganancias
    cargadas. La subida usa el juego A y la bajada el juego F.

Cada experimento se guarda en registros/ como:
  - <fecha>_<tipo>_<motores>.csv  : una fila por muestra y motor (los 6 motores)
  - <fecha>_<tipo>_<motores>.json : parámetros, comandos enviados con su tiempo,
                                    ganancias y configuración usadas, y estado final
"""
import csv
import json
import threading
import time
from collections import deque
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path

from enlace import LADOS, MOTORES, Muestra, fmt_num

MAX_MUESTRAS   = 3000   # por motor en memoria (5 min a 100 ms)
DURACION_MAX_S = 60.0   # tope de un experimento, por si hay un error de dedo
DATOS_VIEJOS_S = 1.0    # sin datos de un lado por más de esto → abortar


# ---------------------------------------------------------------------- #
#  Datos en vivo                                                          #
# ---------------------------------------------------------------------- #

class Datos:
    """Últimas muestras de cada motor. Lo alimentan los hilos lectores."""

    def __init__(self):
        self._lock    = threading.Lock()
        self._buf     = {m: deque(maxlen=MAX_MUESTRAS) for m in MOTORES}
        self.ultima   = {m: None for m in MOTORES}
        self.registro = None   # Registro activo (o None)

    def agregar(self, m: Muestra):
        with self._lock:
            self._buf[m.id].append(m)
            self.ultima[m.id] = m
            reg = self.registro
        if reg is not None:
            reg.escribir(m)

    def recientes(self, motor_id: str, desde_t: float) -> list:
        """Muestras de un motor con t_pc >= desde_t, en orden."""
        with self._lock:
            buf = list(self._buf[motor_id])
        i = len(buf)
        while i > 0 and buf[i - 1].t_pc >= desde_t:
            i -= 1
        return buf[i:]


# ---------------------------------------------------------------------- #
#  Experimento                                                            #
# ---------------------------------------------------------------------- #

@dataclass
class Experimento:
    tipo: str          # "PWM" | "SP"
    motores: list      # ["L0", "R2", ...]
    v0: float          # valor inicial y final (PWM o rpm)
    v1: float          # valor del escalón
    t_antes: float     # s
    t_escalon: float   # s
    t_despues: float   # s
    notas: str = ""

    @property
    def duracion(self) -> float:
        return self.t_antes + self.t_escalon + self.t_despues

    @property
    def lados(self) -> list:
        return [l for l in LADOS if any(m[0] == l for m in self.motores)]

    def validar(self) -> list:
        errores = []
        if self.tipo not in ("PWM", "SP"):
            errores.append(f"tipo desconocido: {self.tipo}")
        if not self.motores:
            errores.append("elige al menos un motor")
        if any(m not in MOTORES for m in self.motores):
            errores.append("motor desconocido")
        if min(self.t_antes, self.t_escalon, self.t_despues) < 0 or self.t_escalon <= 0:
            errores.append("los tiempos deben ser >= 0 y el del escalón > 0")
        if self.duracion > DURACION_MAX_S:
            errores.append(f"la duración total ({self.duracion:.1f} s) pasa de {DURACION_MAX_S:.0f} s")
        if self.tipo == "PWM" and max(abs(self.v0), abs(self.v1)) > 255:
            errores.append("el PWM debe estar entre -255 y 255")
        return errores

    def etiqueta_motores(self) -> str:
        s = set(self.motores)
        if s == set(MOTORES):
            return "todos"
        if s == {"L0", "L1", "L2"}:
            return "izq"
        if s == {"R0", "R1", "R2"}:
            return "der"
        return "-".join(m for m in MOTORES if m in s)


# ---------------------------------------------------------------------- #
#  Registro en disco                                                      #
# ---------------------------------------------------------------------- #

class Registro:
    COLUMNAS = ["t_s", "lado", "motor", "id", "t_esp_ms", "reg", "SP", "PV", "e", "P", "I", "D", "u"]

    def __init__(self, carpeta: Path, exp: Experimento, meta: dict):
        carpeta.mkdir(parents=True, exist_ok=True)
        base = f"{datetime.now():%Y%m%d_%H%M%S}_{exp.tipo}_{exp.etiqueta_motores()}"
        self.ruta_csv  = carpeta / f"{base}.csv"
        self.ruta_json = carpeta / f"{base}.json"
        self.meta      = {"experimento": asdict(exp), "inicio": datetime.now().isoformat(timespec="seconds"), **meta}
        self.eventos   = []
        self.n         = 0
        self.t0        = None
        self._lock     = threading.Lock()
        self._f        = open(self.ruta_csv, "w", newline="", encoding="utf-8")
        self._w        = csv.writer(self._f)
        self._w.writerow(self.COLUMNAS)

    def iniciar(self, t0: float):
        self.t0 = t0

    def escribir(self, m: Muestra):
        with self._lock:
            if self._f is None or self.t0 is None or m.t_pc < self.t0:
                return
            self._w.writerow([f"{m.t_pc - self.t0:.4f}", m.lado, m.motor, m.id, m.t_esp_ms, m.reg,
                              m.sp, m.pv, m.e, m.P, m.I, m.D, m.u])
            self.n += 1

    def evento(self, t_pc: float, lado: str, comando: str):
        with self._lock:
            self.eventos.append({"t_s": round(t_pc - self.t0, 4), "lado": lado, "comando": comando})

    def cerrar(self, estado: str):
        with self._lock:
            if self._f is None:
                return
            self._f.close()
            self._f = None
            meta = {**self.meta, "estado": estado, "muestras": self.n,
                    "archivo_csv": self.ruta_csv.name, "eventos": self.eventos}
        self.ruta_json.write_text(json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8")


# ---------------------------------------------------------------------- #
#  Ejecución                                                              #
# ---------------------------------------------------------------------- #

class Ejecutor(threading.Thread):
    """Corre un experimento en su propio hilo. Al terminar (o abortar) manda
    STOP a todos los lados conectados y llama a al_terminar(ok, estado, ruta_csv)."""

    def __init__(self, exp: Experimento, enlaces: dict, datos: Datos, carpeta: Path,
                 meta: dict, al_terminar):
        super().__init__(daemon=True)
        self.exp         = exp
        self.enlaces     = enlaces
        self.datos       = datos
        self.carpeta     = carpeta
        self.meta        = meta
        self.al_terminar = al_terminar
        self._abortar    = threading.Event()
        self.motivo      = None
        self.t0          = None

    def abortar(self, motivo: str):
        if self.motivo is None:
            self.motivo = motivo
        self._abortar.set()

    @property
    def transcurrido(self) -> float:
        return 0.0 if self.t0 is None else time.monotonic() - self.t0

    def run(self):
        exp = self.exp
        conectados = [l for l in LADOS if l in self.enlaces and self.enlaces[l].conectado]
        faltan = [l for l in exp.lados if l not in conectados]
        if faltan:
            self.al_terminar(False, f"no hay conexión con el lado {', '.join(faltan)}", None)
            return

        reg = Registro(self.carpeta, exp, self.meta)
        estado = "completo"
        try:
            self.t0 = time.monotonic()
            reg.iniciar(self.t0)
            self.datos.registro = reg
            self._enviar(conectados, "STOP", reg)     # partida limpia: SP = 0 y PID reiniciado

            pasos = [(0.0, exp.v0),
                     (exp.t_antes, exp.v1),
                     (exp.t_antes + exp.t_escalon, exp.v0)]
            completo = True
            for t_rel, valor in pasos:
                if not self._esperar_hasta(t_rel):
                    completo = False
                    break
                self._aplicar(valor, reg)
            if completo:
                completo = self._esperar_hasta(exp.duracion)
            if not completo:
                estado = f"abortado: {self.motivo}"
        except Exception as e:
            estado = f"error: {e}"
        finally:
            self._enviar(conectados, "STOP", reg)
            self.datos.registro = None
            reg.cerrar(estado)
        self.al_terminar(estado == "completo", estado, reg.ruta_csv)

    def _esperar_hasta(self, t_rel: float) -> bool:
        while True:
            if self._abortar.is_set():
                return False
            for lado in self.exp.lados:
                enl = self.enlaces[lado]
                if not enl.conectado:
                    self.abortar(f"se perdió la conexión con el lado {lado}")
                    return False
                if enl.edad_datos() > DATOS_VIEJOS_S:
                    self.abortar(f"sin datos del lado {lado} por más de {DATOS_VIEJOS_S:.0f} s")
                    return False
            if time.monotonic() - self.t0 >= t_rel:
                return True
            time.sleep(0.005)

    def _aplicar(self, valor: float, reg: Registro):
        cmd = "MAN" if self.exp.tipo == "PWM" else "SP"
        for lado in self.exp.lados:     # ambos lados seguidos: llegan con ~1 ms de diferencia
            idx = sorted(int(m[1]) for m in self.exp.motores if m[0] == lado)
            selectores = ["*"] if len(idx) == 3 else [str(i) for i in idx]
            for sel in selectores:
                linea = f"{cmd} {sel} {fmt_num(valor)}"
                self.enlaces[lado].enviar(linea)
                reg.evento(time.monotonic(), lado, linea)

    def _enviar(self, lados, linea: str, reg: Registro):
        for lado in lados:
            self.enlaces[lado].enviar(linea)
            if reg.t0 is not None:
                reg.evento(time.monotonic(), lado, linea)
