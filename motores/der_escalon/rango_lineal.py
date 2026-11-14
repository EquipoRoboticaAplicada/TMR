#!/usr/bin/env python3
"""
rango_lineal.py — Rango de mayor linealidad de una prueba escalonada en lazo abierto.

Lee los archivos de la prueba (columnas t[s], PWM, RPM, como los imprime
der_escalon / izq_escalon), saca la curva estática de cada uno (la velocidad en
que se asienta el motor en cada nivel de PWM) y busca el tramo de PWM más ancho
en el que esa curva es una recta. Ese tramo es donde conviene hacer la prueba
de escalón que se le da a Control Station: ahí la ganancia K es constante.

Uso (solo necesita Python, sin librerías extra):
    python rango_lineal.py                    # todos los .txt de prueba_csv/
    python rango_lineal.py motor0.txt         # archivos o carpetas concretos
    python rango_lineal.py --tol 2            # tolerancia de linealidad en %

Criterio: el error de linealidad de un tramo es la mayor desviación de sus
puntos respecto a su recta de mínimos cuadrados, como porcentaje de lo que
cambia el PV en el tramo. Gana el tramo más ancho cuyo error no pasa de --tol.
"""
import argparse
import re
import statistics
import sys
from pathlib import Path

AQUI = Path(__file__).resolve().parent
MIN_MUESTRAS = 10   # un nivel con menos muestras que esto no se toma en cuenta


def leer(ruta: Path, col_pv: int):
    """Columnas PWM y PV del archivo. Ignora líneas vacías, con '#' o sin números."""
    pwm, pv = [], []
    for linea in ruta.read_text(encoding="utf-8", errors="ignore").splitlines():
        linea = linea.strip()
        if not linea or linea.startswith("#"):
            continue
        partes = re.split(r"[\t; ]+", linea)
        if len(partes) < 3:
            partes = linea.split(",")                        # CSV con comas
        else:
            partes = [p.replace(",", ".") for p in partes]   # coma decimal
        try:
            pwm.append(float(partes[1]))
            pv.append(float(partes[col_pv - 1]))
        except (ValueError, IndexError):
            continue
    return pwm, pv


def curva_estatica(pwm, pv, fraccion_final: float):
    """Un punto por nivel de PWM: {pwm, sube, baja, media}, ordenados por PWM.

    El PV de cada nivel es el promedio de su tramo final, cuando la velocidad
    ya se asentó. Si la prueba pasa por un nivel subiendo y bajando, se guardan
    los dos valores y su promedio.
    """
    tramos = []                       # (pwm, pv asentado) en el orden de la prueba
    i = 0
    while i < len(pwm):
        j = i
        while j + 1 < len(pwm) and pwm[j + 1] == pwm[i]:
            j += 1
        n = j - i + 1
        if n >= MIN_MUESTRAS:
            tramos.append((pwm[i], statistics.fmean(pv[i + int(n * (1.0 - fraccion_final)): j + 1])))
        i = j + 1

    por_nivel = {}
    for k, (nivel, valor) in enumerate(tramos):
        if k > 0:
            sube = nivel > tramos[k - 1][0]
        else:
            sube = len(tramos) < 2 or tramos[1][0] > nivel
        por_nivel.setdefault(nivel, {"sube": [], "baja": []})["sube" if sube else "baja"].append(valor)

    puntos = []
    for nivel in sorted(por_nivel):
        d = por_nivel[nivel]
        puntos.append({"pwm": nivel,
                       "sube": statistics.fmean(d["sube"]) if d["sube"] else None,
                       "baja": statistics.fmean(d["baja"]) if d["baja"] else None,
                       "media": statistics.fmean(d["sube"] + d["baja"])})
    return puntos


def ajustar(puntos):
    """Recta de mínimos cuadrados PV = k·PWM + c de un tramo y su error de linealidad."""
    xs = [p["pwm"] for p in puntos]
    ys = [p["media"] for p in puntos]
    mx, my = statistics.fmean(xs), statistics.fmean(ys)
    sxx = sum((x - mx) ** 2 for x in xs)
    k = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    c = my - k * mx
    residuos = [y - (k * x + c) for x, y in zip(xs, ys)]
    desv_max = max(abs(r) for r in residuos)
    cambio = abs(k) * (xs[-1] - xs[0])            # lo que cambia el PV en el tramo
    syy = sum((y - my) ** 2 for y in ys)
    return {"k": k, "c": c, "desv_max": desv_max,
            "error": 100.0 * desv_max / cambio if cambio > 0 else float("inf"),
            "r2": 1.0 - sum(r * r for r in residuos) / syy if syy > 0 else 0.0}


def buscar_rango(puntos, tol: float, min_niveles: int):
    """(i, j, ajuste, cumple): el tramo puntos[i..j] más ancho con error <= tol.

    Si ninguno cumple, regresa el de menor error entre los de tamaño mínimo.
    """
    candidatos = []
    for i in range(len(puntos)):
        for j in range(i + min_niveles - 1, len(puntos)):
            candidatos.append((i, j, ajustar(puntos[i:j + 1])))
    if not candidatos:
        return None
    buenos = [c for c in candidatos if c[2]["error"] <= tol]
    if buenos:
        i, j, a = max(buenos, key=lambda c: (puntos[c[1]]["pwm"] - puntos[c[0]]["pwm"], -c[2]["error"]))
        return i, j, a, True
    i, j, a = min((c for c in candidatos if c[1] - c[0] + 1 == min_niveles), key=lambda c: c[2]["error"])
    return i, j, a, False


def num(v, ancho=8, dec=2):
    return f"{v:{ancho}.{dec}f}" if v is not None else " " * (ancho - 1) + "-"


def analizar(ruta: Path, args):
    """Imprime el análisis de un archivo y regresa su fila para el resumen (o None)."""
    print(f"\n=== {ruta.name} " + "=" * max(4, 60 - len(ruta.name)))
    pwm, pv = leer(ruta, args.pv)
    puntos = curva_estatica(pwm, pv, args.final)
    if len(puntos) < args.min_niveles:
        print(f"  Solo tiene {len(puntos)} nivel(es) de PWM con datos: hacen falta al menos {args.min_niveles}.")
        return None
    i, j, a, cumple = buscar_rango(puntos, args.tol, args.min_niveles)

    con_bajada = any(p["baja"] is not None for p in puntos)
    print(f"  {len(pwm)} muestras, {len(puntos)} niveles de PWM "
          f"({puntos[0]['pwm']:g} a {puntos[-1]['pwm']:g})" + (", con subida y bajada" if con_bajada else ""))
    print("\n  Curva estática: PV asentado en cada nivel, y ganancia K desde el nivel anterior")
    print("  (con * las ganancias de los intervalos que quedan dentro del rango lineal):")
    print("     PWM" + ("   PV sube   PV baja  PV media" if con_bajada else "   PV [rpm]") + "   K [rpm/PWM]")
    for n, p in enumerate(puntos):
        fila = f"  {p['pwm']:6g}"
        fila += (num(p["sube"], 10) + num(p["baja"], 10) + num(p["media"], 10)) if con_bajada else num(p["media"], 11)
        if n > 0:
            k_local = (p["media"] - puntos[n - 1]["media"]) / (p["pwm"] - puntos[n - 1]["pwm"])
            fila += f"   {k_local:10.4f}" + ("*" if i < n <= j else " ")
        else:
            fila += " " * 14
        print(fila + ("    <-- lineal" if i <= n <= j else ""))

    lo, hi = puntos[i], puntos[j]
    titulo = "RANGO DE MAYOR LINEALIDAD" if cumple else f"NINGÚN TRAMO CUMPLE LA TOLERANCIA DE {args.tol:g} %; EL MÁS LINEAL ES"
    print(f"\n  {titulo}: PWM {lo['pwm']:g} a {hi['pwm']:g}   (PV {lo['media']:.2f} a {hi['media']:.2f} rpm)")
    print(f"    recta:  PV = {a['k']:.4f} * PWM {'+' if a['c'] >= 0 else '-'} {abs(a['c']):.2f}"
          f"     K = {a['k']:.4f} rpm/PWM     R2 = {a['r2']:.5f}")
    print(f"    error de linealidad: {a['error']:.2f} % del cambio de PV en el tramo "
          f"(desviación máxima {a['desv_max']:.2f} rpm)")
    vecinos = []
    if i > 0:
        vecinos.append(f"desde PWM {puntos[i - 1]['pwm']:g} sube a {ajustar(puntos[i - 1:j + 1])['error']:.1f} %")
    if j < len(puntos) - 1:
        vecinos.append(f"hasta PWM {puntos[j + 1]['pwm']:g} sube a {ajustar(puntos[i:j + 2])['error']:.1f} %")
    if vecinos:
        print("    si se amplía el tramo: " + "; ".join(vecinos))

    # Zona muerta: el PWM en que la recta del rango lineal cruza PV = 0. Sumarle ese
    # PWM a la salida del PID deja al motor respondiendo como una recta desde cero.
    if a["k"] != 0:
        zm = max(0.0, -a["c"] / a["k"])
        print(f"  Zona muerta estimada (ZM del sintonizador): PWM {zm:.1f}"
              "   (donde la recta del rango lineal cruza PV = 0)")
        quietos = [p["pwm"] for p in puntos if p["pwm"] > 0 and abs(p["media"]) < 0.05]
        if quietos and puntos[-1]["pwm"] > max(quietos):
            siguiente = min(p["pwm"] for p in puntos if p["pwm"] > max(quietos))
            print(f"    desde reposo el motor arrancó entre PWM {max(quietos):g} y {siguiente:g}: para vencer la "
                  "fricción estática hace falta más que para seguir girando")

    # Escalón para Control Station: dentro del rango, con un nivel de margen por lado si caben
    a_lo, a_hi = (puntos[i + 1], puntos[j - 1]) if j - i >= 3 else (lo, hi)
    print(f"  Escalón sugerido para Control Station: PWM {a_lo['pwm']:g} -> {a_hi['pwm']:g} -> {a_lo['pwm']:g}"
          f"   (en el sketch de escalón: <motor> {a_lo['pwm']:g} {a_hi['pwm']:g} {a_lo['pwm']:g})")
    return ruta.name, lo, hi, a, cumple


def reunir_archivos(rutas):
    """Archivos a analizar: los dados, o los .txt de las carpetas dadas."""
    archivos = []
    for r in rutas:
        r = Path(r)
        if r.is_dir():
            archivos += sorted(p for p in r.iterdir() if p.suffix.lower() == ".txt")
        elif r.is_file():
            archivos.append(r)
        else:
            print(f"No existe: {r}", file=sys.stderr)
    return archivos


def main():
    p = argparse.ArgumentParser(description="Rango de mayor linealidad de una prueba escalonada en lazo abierto.")
    p.add_argument("rutas", nargs="*", default=[str(AQUI / "prueba_csv")],
                   help="archivos de la prueba o carpetas con ellos (por defecto: prueba_csv/)")
    p.add_argument("--tol", type=float, default=1.0,
                   help="error de linealidad máximo, en %% del cambio de PV del tramo (por defecto 1)")
    p.add_argument("--min-niveles", type=int, default=3, help="niveles de PWM mínimos del tramo (por defecto 3)")
    p.add_argument("--pv", type=int, default=3, help="columna del PV, contando desde 1 (por defecto 3)")
    p.add_argument("--final", type=float, default=0.4,
                   help="fracción final de cada nivel que se promedia como PV asentado (por defecto 0.4)")
    args = p.parse_args()
    if args.min_niveles < 3:
        p.error("--min-niveles debe ser al menos 3: dos puntos siempre forman una recta")
    if not 0.0 < args.final <= 1.0 or args.tol <= 0 or args.pv < 3:
        p.error("--final debe estar entre 0 y 1, --tol debe ser > 0 y --pv >= 3")
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(errors="replace")     # consolas que no tienen todos los caracteres

    archivos = reunir_archivos(args.rutas)
    if not archivos:
        sys.exit("No hay archivos que analizar.")
    filas = [f for f in (analizar(a, args) for a in archivos) if f is not None]

    if len(filas) > 1:
        print(f"\n=== RESUMEN (tolerancia {args.tol:g} %) " + "=" * 34)
        ancho = max(len(f[0]) for f in filas)
        print(f"  {'archivo':<{ancho}}   rango PWM     PV [rpm]         K [rpm/PWM]   error")
        for nombre, lo, hi, a, cumple in filas:
            rango = f"{lo['pwm']:g} a {hi['pwm']:g}"
            pvs = f"{lo['media']:.2f} a {hi['media']:.2f}"
            print(f"  {nombre:<{ancho}}   {rango:<12}  {pvs:<15}  {a['k']:.4f}        {a['error']:.2f} %"
                  + ("" if cumple else "   (no cumple la tolerancia)"))
        # Tramo que es lineal en todos a la vez, por si se quiere el mismo escalón para todos
        comun_lo = max(f[1]["pwm"] for f in filas)
        comun_hi = min(f[2]["pwm"] for f in filas)
        if comun_lo < comun_hi:
            print(f"  Rango lineal común a todos: PWM {comun_lo:g} a {comun_hi:g}")
        else:
            print("  Los rangos lineales de los archivos no se traslapan.")
    print()


if __name__ == "__main__":
    main()
