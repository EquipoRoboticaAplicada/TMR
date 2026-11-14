"""
experimento.py — Datos en vivo de los 6 motores, experimentos que dispara el
usuario y su registro en disco.

Un experimento es un perfil fijo que el usuario lanza a mano: una lista de
niveles, cada uno con su duración, y STOP al terminar.

  - tipo "PWM": lazo abierto (comando MAN del firmware), para identificar cada
    motor. Una escalera de niveles de PWM con la misma duración cada uno, como
    en los sketches der_escalon / izq_escalon. El PWM se aplica directo: sin
    PID y sin sumar la zona muerta. Mientras dura, el ESP32 muestrea al periodo
    que pide el experimento (ts_ms) y al terminar regresa al suyo.
  - tipo "SP":  lazo cerrado (setpoint), para revisar la sintonía con las
    ganancias cargadas: inicial → escalón → inicial. En cada cambio de setpoint
    actúa el juego REF y, cuando la velocidad llega, la mezcla pasa al juego
    PERT (columna "reg" = porcentaje de REF).

Cada experimento se guarda en registros/ como:
  - <fecha>_<tipo>_<motores>.csv  : una fila por muestra y motor
  - <fecha>_<tipo>_<motores>.json : parámetros, comandos enviados con su tiempo,
                                    ganancias y configuración usadas, y estado final
  - <fecha>_<tipo>_<motores>_<motor>.txt (solo PWM): t[s], PWM y RPM de cada
    motor probado, separados por tabulador. Es el formato que imprime
    der_escalon: sirve tal cual para Control Station y para rango_lineal.py.
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

MAX_MUESTRAS   = 3000    # por motor en memoria (1 min con una muestra cada 20 ms)
DURACION_MAX_S = 300.0   # tope de un experimento, por si hay un error de dedo
DATOS_VIEJOS_S = 1.0     # sin datos de un lado por más de esto → abortar


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

def perfil_escalon(v0: float, v1: float, t_antes: float, t_escalon: float, t_despues: float) -> list:
    """inicial → escalón → inicial. Un tramo de duración 0 simplemente no existe."""
    return [(d, v) for d, v in ((t_antes, v0), (t_escalon, v1), (t_despues, v0)) if d != 0]


def perfil_escalera(niveles: list, t_nivel: float) -> list:
    """Todos los niveles, en orden, con la misma duración cada uno."""
    return [(t_nivel, n) for n in niveles]


@dataclass
class Experimento:
    tipo: str            # "PWM" | "SP"
    motores: list        # ["L0", "R2", ...]
    perfil: list         # [(duración [s], valor), ...] en orden; valor en PWM o en rpm
    ts_ms: int = None    # solo PWM: periodo de muestreo del ESP32 durante la prueba (None = no tocarlo)
    notas: str = ""

    @property
    def duracion(self) -> float:
        return sum(d for d, _v in self.perfil)

    @property
    def lados(self) -> list:
        return [l for l in LADOS if any(m[0] == l for m in self.motores)]

    def pasos(self) -> list:
        """[(instante en que empieza el nivel [s], valor), ...]"""
        t, pasos = 0.0, []
        for d, v in self.perfil:
            pasos.append((t, v))
            t += d
        return pasos

    def resumen(self) -> str:
        return " → ".join(f"{v:g}" for _d, v in self.perfil)

    def validar(self) -> list:
        errores = []
        if self.tipo not in ("PWM", "SP"):
            errores.append(f"tipo desconocido: {self.tipo}")
        if not self.motores:
            errores.append("elige al menos un motor")
        if any(m not in MOTORES for m in self.motores):
            errores.append("motor desconocido")
        if not self.perfil:
            errores.append("el perfil no tiene ningún nivel")
        elif any(d <= 0 for d, _v in self.perfil):
            errores.append("la duración de cada nivel debe ser > 0")
        elif self.duracion > DURACION_MAX_S:
            errores.append(f"la duración total ({self.duracion:.1f} s) pasa de {DURACION_MAX_S:.0f} s")
        if self.tipo == "PWM" and any(abs(v) > 255 for _d, v in self.perfil):
            errores.append("el PWM debe estar entre -255 y 255")
        if self.ts_ms is not None and not 10 <= self.ts_ms <= 1000:
            errores.append("el muestreo de la prueba debe estar entre 10 y 1000 ms")
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
        self.base = carpeta / f"{datetime.now():%Y%m%d_%H%M%S}_{exp.tipo}_{exp.etiqueta_motores()}"
        self.ruta_csv  = self.base.with_name(self.base.name + ".csv")
        self.ruta_json = self.base.with_name(self.base.name + ".json")
        self.rutas_txt = []
        self.meta      = {"experimento": asdict(exp), "inicio": datetime.now().isoformat(timespec="seconds"), **meta}
        self.eventos   = []
        self.n         = 0
        self.t0        = None
        self._lock     = threading.Lock()
        self._f        = open(self.ruta_csv, "w", newline="", encoding="utf-8")
        self._w        = csv.writer(self._f)
        self._w.writerow(self.COLUMNAS)
        # Lazo abierto: (t_esp_ms, PWM, RPM) de cada motor probado, para su .txt
        self._lazo_abierto = {m: [] for m in exp.motores} if exp.tipo == "PWM" else {}

    def iniciar(self, t0: float):
        self.t0 = t0

    def escribir(self, m: Muestra):
        with self._lock:
            if self._f is None or self.t0 is None or m.t_pc < self.t0:
                return
            self._w.writerow([f"{m.t_pc - self.t0:.4f}", m.lado, m.motor, m.id, m.t_esp_ms, m.reg,
                              m.sp, m.pv, m.e, m.P, m.I, m.D, m.u])
            self.n += 1
            if m.reg == "M" and m.id in self._lazo_abierto:   # "M": el PWM lo fija el experimento
                self._lazo_abierto[m.id].append((m.t_esp_ms, m.u, m.pv))

    def evento(self, t_pc: float, lado: str, comando: str):
        with self._lock:
            self.eventos.append({"t_s": round(t_pc - self.t0, 4), "lado": lado, "comando": comando})

    def cerrar(self, estado: str):
        with self._lock:
            if self._f is None:
                return
            self._f.close()
            self._f = None
            for motor, filas in self._lazo_abierto.items():
                if not filas:
                    continue
                ruta = self.base.with_name(f"{self.base.name}_{motor}.txt")
                t0 = filas[0][0]          # reloj del ESP32: el periodo entre filas es el real
                ruta.write_text("".join(f"{(t - t0) / 1000.0:.3f}\t{u:g}\t{pv:.3f}\n" for t, u, pv in filas),
                                encoding="utf-8")
                self.rutas_txt.append(ruta)
            meta = {**self.meta, "estado": estado, "muestras": self.n,
                    "archivo_csv": self.ruta_csv.name, "archivos_txt": [r.name for r in self.rutas_txt],
                    "eventos": self.eventos}
        self.ruta_json.write_text(json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8")


# ---------------------------------------------------------------------- #
#  Ejecución                                                              #
# ---------------------------------------------------------------------- #

class Ejecutor(threading.Thread):
    """Corre un experimento en su propio hilo. Al terminar (o abortar) manda
    STOP a todos los lados conectados y llama a
    al_terminar(ok, estado, ruta_csv, rutas_txt)."""

    def __init__(self, exp: Experimento, enlaces: dict, datos: Datos, carpeta: Path,
                 meta: dict, al_terminar, ts_normal_ms: int = None):
        """ts_normal_ms: periodo de muestreo que tienen los ESP32, para dejarlo
        igual después de una prueba que pide el suyo (exp.ts_ms)."""
        super().__init__(daemon=True)
        self.exp          = exp
        self.enlaces      = enlaces
        self.datos        = datos
        self.carpeta      = carpeta
        self.meta         = meta
        self.al_terminar  = al_terminar
        self.ts_normal_ms = ts_normal_ms
        self._abortar     = threading.Event()
        self.motivo       = None
        self.t0           = None

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
            self.al_terminar(False, f"no hay conexión con el lado {', '.join(faltan)}", None, [])
            return

        reg = Registro(self.carpeta, exp, self.meta)
        cambiar_muestreo = exp.ts_ms is not None and self.ts_normal_ms is not None
        estado = "completo"
        try:
            self.t0 = time.monotonic()
            reg.iniciar(self.t0)
            self.datos.registro = reg
            self._enviar(conectados, "STOP", reg)     # partida limpia: SP = 0 y PID reiniciado
            if cambiar_muestreo:
                self._muestreo_de_prueba(reg)

            completo = True
            for t_rel, valor in exp.pasos():
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
            if cambiar_muestreo:
                for linea in (f"TS {self.ts_normal_ms}", "VER *"):
                    self._enviar(exp.lados, linea, reg)
            self.datos.registro = None
            reg.cerrar(estado)
        self.al_terminar(estado == "completo", estado, reg.ruta_csv, reg.rutas_txt)

    def _motores_de(self, lado: str) -> list:
        return sorted(int(m[1]) for m in self.exp.motores if m[0] == lado)

    def _muestreo_de_prueba(self, reg: Registro):
        """Pone a los lados de la prueba a muestrear al periodo que pide. Si de un
        lado solo se prueba un motor, el ESP32 reporta nada más ese: con los tres
        no le alcanza el serial para mandar todas las muestras a 10 ms."""
        for lado in self.exp.lados:
            self._enviar([lado], f"TS {self.exp.ts_ms}", reg)
            idx = self._motores_de(lado)
            if len(idx) == 1:
                self._enviar([lado], f"VER {idx[0]}", reg)

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
            idx = self._motores_de(lado)
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
