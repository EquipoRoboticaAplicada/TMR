"""
sintonia.py — Cálculo de las ganancias PID a partir del modelo de cada motor.

Modelo de la planta (primer orden más tiempo muerto, el que entrega Control
Station con la prueba de escalón en lazo abierto):

        G(s) = K · e^(-θ·s) / (τ·s + 1)        K [rpm/PWM], τ [s], θ [s]

Para cada motor se calculan dos juegos de ganancias, cada uno minimizando un
criterio integral del error:

    REF  : ITAE ante un escalón de referencia (setpoint)
    PERT : IAE  ante un escalón de perturbación en la entrada de la planta

El firmware interpola entre los dos en tiempo real (ver der_sintonizacion.ino).

Por qué un solver numérico y no las correlaciones de tablas (López / Rovira):
esas fórmulas suponen un controlador continuo y 0.1 <= θ/τ <= 1. Aquí la
constante de tiempo del motor (decenas de ms) es del orden del tiempo de
muestreo del PID, así que el muestreo cuenta tanto como la planta. El solver
simula el lazo tal como corre en el ESP32 (mismo PID discreto, misma medición
de velocidad promediada en cada muestra) y minimiza el criterio sobre esa
simulación. Las correlaciones solo se usan como punto de partida.

Este módulo no depende del resto de la herramienta: solo de numpy.
"""
import math
from dataclasses import dataclass

import numpy as np

JUEGOS = ("REF", "PERT")
CRITERIO = {"REF": "ITAE", "PERT": "IAE"}
ESCENARIO = {"REF": "referencia", "PERT": "perturbacion"}
ESTRUCTURAS = ("PI", "PID")

N_DEFECTO = 10.0          # filtro de la derivada que se deja en las ganancias calculadas
MARGEN_DEFECTO = 2.0      # margen de ganancia mínimo que se le exige a cada sintonía


@dataclass(frozen=True)
class Modelo:
    K: float       # ganancia [rpm/PWM]
    tau: float     # constante de tiempo [s]
    theta: float   # tiempo muerto [s]

    def problema(self):
        """Texto con lo que le falta al modelo, o None si se puede usar."""
        if not all(math.isfinite(v) for v in (self.K, self.tau, self.theta)):
            return "valores no numéricos"
        if self.K <= 0:
            return "K debe ser > 0"
        if self.tau <= 0:
            return "τ debe ser > 0"
        if self.theta < 0:
            return "θ debe ser >= 0"
        return None


@dataclass(frozen=True)
class Resultado:
    juego: str        # "REF" | "PERT"
    criterio: str     # "ITAE" | "IAE"
    g: dict           # {"Kp":.., "Ti":.., "Td":.., "N":..}
    costo: float      # valor del criterio (escalón unitario)
    margen: float     # margen de ganancia: cuánto puede crecer K antes de oscilar
    limitado: bool    # True si quedó en el margen mínimo: el óptimo puro del criterio pedía más ganancia


# ---------------------------------------------------------------------- #
#  Lazo cerrado discreto                                                  #
# ---------------------------------------------------------------------- #

class Lazo:
    """PID del firmware + planta, avanzando una muestra (ts) a la vez.

    Estado x = [y, pv, I, D, x_prev, u[k-1], ..., u[k-d-1]]
      y      velocidad real de la planta en el instante de muestreo
      pv     velocidad medida: promedio de y en la última muestra (el encoder
             cuenta pulsos durante el periodo, no mide la velocidad instantánea)
      I, D   términos del PID en unidades de PWM
      x_prev error (o -pv) de la muestra anterior, para la derivada

    Todo es lineal (sin saturación): el óptimo del criterio no depende del
    tamaño del escalón y la estabilidad se puede leer de los valores propios.
    """

    def __init__(self, modelo: Modelo, g: dict, ts: float, dmode: str = "E", k_mult: float = 1.0):
        self.ts, self.tau = ts, modelo.tau
        self.K = modelo.K * k_mult
        self.d = int(modelo.theta / ts + 1e-9)                 # muestras enteras de tiempo muerto
        self.delta = min(max(modelo.theta - self.d * ts, 0.0), ts)   # fracción restante
        self.E1 = math.exp(-self.delta / self.tau)
        self.E2 = math.exp(-(ts - self.delta) / self.tau)

        kp, ti, td, n = g["Kp"], g["Ti"], g["Td"], g["N"]
        tf = td / n if n > 0 else 0.0
        self.kp = kp
        self.ki = kp / ti if ti > 0 else 0.0
        self.cD1 = tf / (tf + ts)
        self.cD2 = kp * td / (tf + ts)
        self.sobre_pv = (dmode == "PV")
        self.n = 6 + self.d

    def paso(self, x, sp: float, dist: float):
        """Una muestra: (estado siguiente, u aplicada). Mismo orden que pasoControl()."""
        y, pv, I, D, x_prev = x[0], x[1], x[2], x[3], x[4]
        e = sp - pv
        x_act = -pv if self.sobre_pv else e
        D = self.cD1 * D + self.cD2 * (x_act - x_prev)
        I = I + self.ki * e * self.ts
        u = self.kp * e + I + D

        # Planta durante la muestra: por el tiempo muerto, primero sigue actuando
        # u[k-d-1] (durante delta) y luego u[k-d]. Solución exacta de cada tramo.
        hist = [u] + list(x[5:])                 # hist[j] = u[k-j]
        ka = self.K * (hist[self.d + 1] + dist)
        kb = self.K * (hist[self.d] + dist)
        y1 = ka + (y - ka) * self.E1
        a1 = ka * self.delta + self.tau * (y - ka) * (1.0 - self.E1)
        y2 = kb + (y1 - kb) * self.E2
        a2 = kb * (self.ts - self.delta) + self.tau * (y1 - kb) * (1.0 - self.E2)
        pv_sig = (a1 + a2) / self.ts
        return [y2, pv_sig, I, D, x_act] + hist[:self.d + 1], u

    def matriz(self):
        """A tal que x[k+1] = A·x[k] cuando SP = 0 y no hay perturbación."""
        A = np.empty((self.n, self.n))
        for i in range(self.n):
            x = [0.0] * self.n
            x[i] = 1.0
            A[:, i] = self.paso(x, 0.0, 0.0)[0]
        return A

    def radio_espectral(self) -> float:
        """Mayor |valor propio| del lazo cerrado: < 1 estable."""
        return float(np.max(np.abs(np.linalg.eigvals(self.matriz()))))

    def respuesta(self, sp: float, dist: float, n: int):
        """pv[0..n-1] partiendo del reposo con SP y perturbación constantes.

        Da lo mismo que llamar n veces a paso(), pero duplicando el tramo ya
        calculado en cada vuelta (x[k+m] = A^m·x[k]): unas 12 operaciones de
        matrices en vez de miles de pasos, que es lo que hace rápido al solver.
        """
        ns = self.n
        Az = np.zeros((ns + 1, ns + 1))            # estado aumentado z = [x, 1]
        Az[:ns, :ns] = self.matriz()
        Az[:ns, ns] = self.paso([0.0] * ns, sp, dist)[0]   # efecto de las entradas constantes
        Az[ns, ns] = 1.0
        Z = np.zeros((ns + 1, 1))
        Z[ns, 0] = 1.0
        with np.errstate(all="ignore"):
            while Z.shape[1] < n:
                Z = np.hstack([Z, Az @ Z])
                Az = Az @ Az
        return Z[1, :n]


def simular(modelo: Modelo, g: dict, ts: float, escenario: str, duracion: float, dmode: str = "E"):
    """Respuesta del lazo a un escalón unitario. Regresa (t, sp, pv, u) como listas.

    escenario "referencia":   SP pasa de 0 a 1 en t = 0
    escenario "perturbacion": entra 1 PWM en la entrada de la planta en t = 0 (SP = 0)
    """
    lazo = Lazo(modelo, g, ts, dmode)
    sp, dist = (1.0, 0.0) if escenario == "referencia" else (0.0, 1.0)
    x = [0.0] * lazo.n
    t, sps, pvs, us = [], [], [], []
    for k in range(max(1, int(round(duracion / ts)))):
        pv = x[1]
        x, u = lazo.paso(x, sp, dist)
        t.append(k * ts)
        sps.append(sp)
        pvs.append(pv)
        us.append(u)
    return t, sps, pvs, us


def horizonte(modelo: Modelo, ts: float) -> float:
    """Duración de la simulación con la que se evalúa el criterio [s]."""
    n = min(max(int((40.0 * (modelo.tau + modelo.theta) + 40.0 * ts) / ts), 100), 4000)
    return n * ts


def criterio(modelo: Modelo, g: dict, ts: float, juego: str, dmode: str = "E") -> float:
    """ITAE (REF) o IAE (PERT) de la respuesta al escalón unitario. inf si es inestable."""
    sp, dist = (1.0, 0.0) if juego == "REF" else (0.0, 1.0)
    con_tiempo = (CRITERIO[juego] == "ITAE")
    n = int(round(horizonte(modelo, ts) / ts))
    e = np.abs(sp - Lazo(modelo, g, ts, dmode).respuesta(sp, dist, n))
    if not np.all(np.isfinite(e)) or e.max() > 1e3 * max(1.0, modelo.K):
        return math.inf
    total = float(np.sum(e * (np.arange(n) * ts) if con_tiempo else e)) * ts
    # Lo que quede de error al final se cobra como si durara otro horizonte:
    # así no gana una sintonía que simplemente no alcanzó a llegar.
    dur = n * ts
    return total + float(e[-1]) * (1.5 * dur * dur if con_tiempo else dur)


def margen_ganancia(modelo: Modelo, g: dict, ts: float, dmode: str = "E") -> float:
    """Factor por el que puede multiplicarse la ganancia del lazo (K real mayor
    que la del modelo, por ejemplo) antes de que el lazo se vuelva inestable."""
    def estable(m):
        return Lazo(modelo, g, ts, dmode, k_mult=m).radio_espectral() < 1.0

    if estable(1.0):
        lo, hi = 1.0, 2.0
        while estable(hi):
            lo, hi = hi, hi * 2.0
            if hi > 64.0:
                return math.inf
    else:
        lo, hi = 0.5, 1.0
        while not estable(lo):
            lo, hi = lo / 2.0, lo
            if lo < 1.0 / 64.0:
                return 0.0
    for _ in range(24):
        mid = math.sqrt(lo * hi)
        if estable(mid):
            lo = mid
        else:
            hi = mid
    return lo


# ---------------------------------------------------------------------- #
#  Punto de partida: correlaciones de tablas                              #
# ---------------------------------------------------------------------- #

def correlacion(modelo: Modelo, juego: str, estructura: str, theta_extra: float = 0.0) -> dict:
    """Fórmulas de mínimo error integral para un PID ideal continuo.

    PERT: López et al. (IAE, perturbación).  REF: Rovira et al. (ITAE, referencia).
    theta_extra se suma al tiempo muerto (el retardo que mete el muestreo).
    """
    K, tau = modelo.K, modelo.tau
    r = min(max((modelo.theta + theta_extra) / tau, 0.05), 2.0)
    if juego == "PERT":
        if estructura == "PI":
            kp = (0.984 / K) * r ** -0.986
            ti = tau / (0.608 * r ** -0.707)
            td = 0.0
        else:
            kp = (1.435 / K) * r ** -0.921
            ti = tau / (0.878 * r ** -0.749)
            td = tau * 0.482 * r ** 1.137
    else:
        if estructura == "PI":
            kp = (0.586 / K) * r ** -0.916
            ti = tau / (1.030 - 0.165 * r)
            td = 0.0
        else:
            kp = (0.965 / K) * r ** -0.855
            ti = tau / (0.796 - 0.147 * r)
            td = tau * 0.308 * r ** 0.929
    return {"Kp": kp, "Ti": ti, "Td": td, "N": N_DEFECTO}


# ---------------------------------------------------------------------- #
#  Solver                                                                 #
# ---------------------------------------------------------------------- #

def _nelder_mead(f, x0, paso=0.35, max_iter=300, tol=1e-6):
    """Mínimo de f sin derivadas (símplex de Nelder-Mead). Regresa (x, f(x))."""
    n = len(x0)
    simplex = [list(x0)] + [[x0[j] + (paso if j == i else 0.0) for j in range(n)] for i in range(n)]
    fs = [f(x) for x in simplex]
    for _ in range(max_iter):
        orden = sorted(range(n + 1), key=lambda i: fs[i])
        simplex = [simplex[i] for i in orden]
        fs = [fs[i] for i in orden]
        ancho = max(abs(simplex[-1][j] - simplex[0][j]) for j in range(n))
        if math.isfinite(fs[-1]) and fs[-1] - fs[0] <= tol * abs(fs[0]) and ancho < 1e-3:
            break
        c = [sum(x[j] for x in simplex[:-1]) / n for j in range(n)]
        peor = simplex[-1]
        xr = [c[j] + (c[j] - peor[j]) for j in range(n)]
        fr = f(xr)
        if fr < fs[0]:
            xe = [c[j] + 2.0 * (c[j] - peor[j]) for j in range(n)]
            fe = f(xe)
            simplex[-1], fs[-1] = (xe, fe) if fe < fr else (xr, fr)
        elif fr < fs[-2]:
            simplex[-1], fs[-1] = xr, fr
        else:
            base = xr if fr < fs[-1] else peor
            xc = [c[j] + 0.5 * (base[j] - c[j]) for j in range(n)]
            fc = f(xc)
            if fc < min(fr, fs[-1]):
                simplex[-1], fs[-1] = xc, fc
            else:
                mejor = simplex[0]
                simplex = [mejor] + [[mejor[j] + 0.5 * (x[j] - mejor[j]) for j in range(n)] for x in simplex[1:]]
                fs = [fs[0]] + [f(x) for x in simplex[1:]]
    i = min(range(n + 1), key=lambda k: fs[k])
    return simplex[i], fs[i]


def calcular(modelo: Modelo, juego: str, ts: float, estructura: str = "PI", dmode: str = "E",
             margen_min: float = MARGEN_DEFECTO, n_filtro: float = N_DEFECTO) -> Resultado:
    """Ganancias que minimizan el criterio del juego, con margen de ganancia >= margen_min.

    ts en segundos. margen_min <= 1 deja el óptimo del criterio sin restricción
    (solo se exige que el lazo sea estable).
    """
    problema = modelo.problema()
    if problema:
        raise ValueError(problema)
    if juego not in JUEGOS or estructura not in ESTRUCTURAS:
        raise ValueError(f"juego o estructura desconocidos: {juego}, {estructura}")
    if ts <= 0:
        raise ValueError("el tiempo de muestreo debe ser > 0")
    margen_min = max(margen_min, 1.0)
    con_d = (estructura == "PID")

    def ganancias(x):
        return {"Kp": math.exp(x[0]), "Ti": math.exp(x[1]),
                "Td": math.exp(x[2]) if con_d else 0.0, "N": n_filtro}

    def recortar(g):
        """Baja Kp lo necesario para cumplir el margen (el margen es proporcional a 1/Kp)."""
        if Lazo(modelo, g, ts, dmode, k_mult=margen_min).radio_espectral() < 1.0:
            return g, False
        m = margen_ganancia(modelo, g, ts, dmode)
        if m <= 0.0:
            return None, True
        return {**g, "Kp": g["Kp"] * m / margen_min * 0.999}, True

    def costo(x):
        if any(abs(v) > 50.0 for v in x):
            return math.inf
        g, recortado = recortar(ganancias(x))
        if g is None:
            return math.inf
        c = criterio(modelo, g, ts, juego, dmode)
        # Empuje suave hacia adentro de la región permitida, para que el símplex
        # no se quede vagando donde Kp ya está recortada.
        return c * (1.0 + 1e-3 * (x[0] - math.log(g["Kp"]))) if recortado else c

    # Punto de partida: correlación con el retardo del muestreo sumado al tiempo muerto
    g0 = correlacion(modelo, juego, estructura, theta_extra=ts)
    x0 = [math.log(g0["Kp"]), math.log(g0["Ti"])] + ([math.log(max(g0["Td"], 1e-4))] if con_d else [])
    mejor_x, mejor_c = None, math.inf
    for d_kp, d_ti in ((0.0, 0.0), (-0.7, 0.0), (0.0, 0.7), (0.7, -0.5)):
        inicio = list(x0)
        inicio[0] += d_kp
        inicio[1] += d_ti
        x, c = _nelder_mead(costo, inicio)
        if c < mejor_c:
            mejor_x, mejor_c = x, c
    if mejor_x is None or not math.isfinite(mejor_c):
        raise ValueError("no se encontró una sintonía estable para este modelo y tiempo de muestreo")

    g, _recortado = recortar(ganancias(mejor_x))
    g = {"Kp": float(f"{g['Kp']:.4g}"), "Ti": round(g["Ti"], 4), "Td": round(g["Td"], 4), "N": n_filtro}
    if g["Ti"] <= 0.0:
        g["Ti"] = 1e-4
    margen = margen_ganancia(modelo, g, ts, dmode)
    return Resultado(juego=juego, criterio=CRITERIO[juego], g=g,
                     costo=criterio(modelo, g, ts, juego, dmode), margen=margen,
                     limitado=margen_min > 1.0 and margen <= margen_min * 1.02)


def calcular_motor(modelo: Modelo, ts: float, estructura: str = "PI", dmode: str = "E",
                   margen_min: float = MARGEN_DEFECTO) -> dict:
    """Los dos juegos de un motor: {"REF": Resultado, "PERT": Resultado}."""
    return {j: calcular(modelo, j, ts, estructura, dmode, margen_min) for j in JUEGOS}
