"""
sintonizador.py — Herramienta para sintonizar los PID de los 6 motores con el
rover ya armado.

Requiere en los ESP32 el firmware motores/der_sintonizacion y
motores/izq_sintonizacion (115200 baud, comandos PID/MAN/SP/STOP/OUT/...).

Uso (desde motores/sintonizador/):
    python sintonizador.py                       # busca los dos ESP32 en los puertos USB
    python sintonizador.py --puertos COM5 COM7   # probar solo esos puertos
    python sintonizador.py --ganancias otra.json

Qué hace:
  - Al conectar, carga a los dos ESP32 las ganancias de ganancias.json.
  - Guarda el modelo de cada motor (K, τ, θ de la prueba de escalón en lazo
    abierto) y, cuando ustedes lo piden, calcula con él los dos juegos de
    ganancias: Referencia (ITAE) y Perturbación (IAE). Ver sintonia.py.
    El firmware mezcla los dos juegos en tiempo real.
  - Guarda también la zona muerta de cada motor (ZM, en PWM): el firmware se
    la suma a la salida del PID para saltársela.
  - Muestra en vivo SP, PV y PWM de los 6 motores.
  - Corre los experimentos que ustedes lanzan (una escalera de niveles de PWM
    en lazo abierto, o un escalón de setpoint en lazo cerrado) en un motor, un
    lado o los 6 a la vez, y guarda cada uno en registros/ (CSV + JSON con las
    ganancias usadas; en lazo abierto, además un .txt por motor con t, PWM y
    RPM, listo para Control Station).
  - Nunca cambia ganancias por su cuenta: las calculadas quedan en la tabla
    hasta que ustedes pulsan "Enviar a los ESP".

Seguridad:
  - PARO (botón o tecla Esc) manda STOP a los dos lados.
  - Watchdog del firmware a 500 ms con latido desde la laptop: si la
    herramienta se cierra o se cuelga, o se desconecta un cable, los motores se detienen.
  - Cada experimento tiene duración fija y termina con STOP.
"""
import argparse
import json
import math
import os
import queue
import re
import subprocess
import sys
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure

import sintonia
from enlace import (CAMPOS, JUEGOS, LADOS, MOTORES, NOMBRE_JUEGO, NOMBRE_LADO, WD_MS, EnlaceESP,
                    abrir_puerto, buscar_esps)
from experimento import (DATOS_VIEJOS_S, Datos, Ejecutor, Experimento, perfil_escalera,
                         perfil_escalon)

AQUI = Path(__file__).resolve().parent

GANANCIA_DEFECTO = {"Kp": 0.5, "Ti": 0.5, "Td": 0.0, "N": 10.0}   # = der.ino / izq.ino
CAMPOS_MODELO    = ("K", "tau", "theta")
MODELO_DEFECTO   = {"K": 0.0, "tau": 0.0, "theta": 0.0}           # K = 0: motor sin identificar
CONFIG_DEFECTO   = {"UMAX": 255, "TS": 100, "BANDA": 0.5, "TMEZ": 500, "DMODE": "E"}
LIMITES_CFG      = {"UMAX": (0, 255), "TS": (10, 1000), "BANDA": (0, 1e9), "TMEZ": (0, 60000)}

VENTANA_S   = 20      # segundos visibles en las gráficas
POLL_MS     = 50      # cola de eventos
GRAFICA_MS  = 200     # refresco de gráficas
PUNTOS_GRAFICA = 600  # puntos que se dibujan por curva, como mucho (aprox.)
COLORES     = ("#1f77b4", "#d62728", "#2ca02c")   # motor 0, 1, 2

TEXTO_PERFIL = {
    "PWM": "Cada nivel se sostiene el tiempo indicado, en orden, y al final STOP. El PWM se aplica directo: sin PID "
           "y sin sumar la zona muerta. Durante la prueba el ESP muestrea al periodo indicado y después regresa a "
           "su TS. Además del registro se guarda un .txt por motor (t, PWM, RPM), igual al de der_escalon.",
    "SP":  "Perfil: inicial (t antes) → escalón (t escalón) → inicial (t después) → STOP. Los motores no elegidos "
           "se quedan en SP = 0. Con |SP| ≤ BANDA el motor queda en reposo (PWM 0, frena solo): para ver cómo "
           "responde el lazo al bajar de velocidad usa un SP inicial distinto de 0.",
}

BLANCO, AMARILLO, ROJO_CLARO = "white", "#fff3b0", "#ffcdd2"
VERDE, NARANJA, ROJO, GRIS = "#2e7d32", "#ef6c00", "#c62828", "#757575"


# ---------------------------------------------------------------------- #
#  Archivo de ganancias                                                   #
# ---------------------------------------------------------------------- #

def ganancias_defecto():
    """Lo que se le manda a los ESP por motor: los dos juegos y la zona muerta (ZM, en PWM)."""
    return {m: {**{j: dict(GANANCIA_DEFECTO) for j in JUEGOS}, "ZM": 0.0} for m in MOTORES}


def modelos_defecto():
    return {m: dict(MODELO_DEFECTO) for m in MOTORES}


def cargar_ganancias(ruta: Path):
    """Regresa (ganancias, modelos, config). Crea el archivo con valores por defecto
    si no existe. Lo que falte en el archivo (por ejemplo, uno anterior sin
    modelos ni juegos REF/PERT) se queda con su valor por defecto."""
    if not ruta.exists():
        guardar_ganancias(ruta, ganancias_defecto(), modelos_defecto(), dict(CONFIG_DEFECTO))
    data = json.loads(ruta.read_text(encoding="utf-8"))
    cfg = {**CONFIG_DEFECTO, **{k: v for k, v in data.get("config", {}).items() if k in CONFIG_DEFECTO}}
    g, modelos = ganancias_defecto(), modelos_defecto()
    for m in MOTORES:
        dm = data.get("motores", {}).get(m, {})
        modelos[m].update({c: float(v) for c, v in dm.get("modelo", {}).items() if c in CAMPOS_MODELO})
        g[m]["ZM"] = float(dm.get("ZM", 0.0))
        for j in JUEGOS:
            g[m][j].update({c: float(v) for c, v in dm.get(j, {}).items() if c in CAMPOS})
    return g, modelos, cfg


def guardar_ganancias(ruta: Path, g: dict, modelos: dict, cfg: dict):
    """JSON legible: una línea por motor y juego, para editarlo a mano o verlo en git diff."""
    def obj(d):
        return "{" + ", ".join(f'"{k}": {json.dumps(v)}' for k, v in d.items()) + "}"
    lineas = [
        "{",
        '  "_comentario": "modelo: G(s) = K*exp(-theta*s)/(tau*s + 1), K en rpm/PWM, tau y theta en s; '
        'K = 0 es motor sin identificar. ZM = zona muerta en PWM que el PID se salta (0 = sin compensar). '
        'PID en forma ideal u = Kp*[e + (1/Ti)*integral(e) + Td*de/dt], '
        'filtro Td/N. Ti = 0 sin integral, Td = 0 sin derivada, N = 0 sin filtro. '
        'REF = juego para cambios de referencia (ITAE), PERT = juego contra perturbaciones (IAE); '
        'el firmware los mezcla. L = izquierdo (ESP_L), R = derecho (ESP_R). '
        'TS y TMEZ en ms, BANDA en rpm, UMAX en PWM.",',
        f'  "config": {obj(cfg)},',
        '  "motores": {',
    ]
    for k, m in enumerate(MOTORES):
        fin = "," if k < len(MOTORES) - 1 else ""
        lineas.append(f'    "{m}": {{"modelo": {obj(modelos[m])}, "ZM": {json.dumps(g[m]["ZM"])},')
        lineas.append(f'           "REF":    {obj(g[m]["REF"])},')
        lineas.append(f'           "PERT":   {obj(g[m]["PERT"])}}}{fin}')
    lineas += ["  }", "}", ""]
    ruta.write_text("\n".join(lineas), encoding="utf-8")


# ---------------------------------------------------------------------- #
#  Interfaz                                                               #
# ---------------------------------------------------------------------- #

class App:

    def __init__(self, root, ruta_ganancias: Path, carpeta_registros: Path,
                 puertos=None, abrir=abrir_puerto):
        self.root    = root
        self.ruta_g  = ruta_ganancias
        self.carpeta = carpeta_registros
        self.puertos = puertos
        self.abrir   = abrir

        self.cola      = queue.Queue()
        self.datos     = Datos()
        self.enlaces   = {}
        self.t_conexion = {}
        self.ejecutor  = None
        self.buscando  = False

        self.confirmadas = {}     # (motor, juego, campo) o (motor, "ZM") → valor confirmado por el firmware
        self.cfg_enviada = {}     # config que tienen los ESP
        self.g_enviadas  = None   # ganancias que tienen los ESP (para el registro)
        self.verif       = None   # (ganancias, t_envio) pendiente de confirmar

        self.calculando  = False  # hay un cálculo de ganancias corriendo en otro hilo

        self._construir()
        try:
            g, modelos, cfg = cargar_ganancias(self.ruta_g)
        except Exception as e:
            messagebox.showerror("Ganancias", f"No se pudo leer {self.ruta_g.name}:\n{e}\nSe usan valores por defecto.")
            g, modelos, cfg = ganancias_defecto(), modelos_defecto(), dict(CONFIG_DEFECTO)
        self._poner_tabla(g, modelos, cfg)
        self.log(f"Modelos y ganancias de {self.ruta_g}")

        root.protocol("WM_DELETE_WINDOW", self.cerrar)
        root.bind_all("<Escape>", lambda _e: self.paro())
        self.conectar()
        root.after(POLL_MS, self._tick)
        root.after(GRAFICA_MS, self._tick_grafica)

    # ================================================================== #
    #  Construcción                                                        #
    # ================================================================== #

    def _construir(self):
        r = self.root
        # ---- Barra superior ----
        barra = ttk.Frame(r, padding=(8, 6))
        barra.pack(side="top", fill="x")
        self.lbl_lado = {}
        for lado in LADOS:
            lbl = tk.Label(barra, text=f"{NOMBRE_LADO[lado]}: —", font=("Segoe UI", 10, "bold"), fg=GRIS)
            lbl.pack(side="left", padx=(0, 20))
            self.lbl_lado[lado] = lbl
        self.btn_reconectar = ttk.Button(barra, text="Reconectar", command=self.conectar)
        self.btn_reconectar.pack(side="left")
        tk.Button(barra, text="PARO  (Esc)", command=self.paro, bg=ROJO, fg="white",
                  activebackground="#8e0000", activeforeground="white",
                  font=("Segoe UI", 12, "bold"), padx=22, pady=2).pack(side="right")

        # ---- Consola (abajo) ----
        self._construir_consola(r)

        # ---- Cuerpo: gráficas | controles ----
        cuerpo = ttk.Panedwindow(r, orient="horizontal")
        cuerpo.pack(side="top", fill="both", expand=True, padx=6)
        izq = ttk.Frame(cuerpo)
        der = ttk.Frame(cuerpo, padding=(6, 0))
        cuerpo.add(izq, weight=3)
        cuerpo.add(der, weight=2)
        self._construir_graficas(izq)
        self._construir_ganancias(der)
        self._construir_experimento(der)

    def _construir_graficas(self, padre):
        fig = Figure(figsize=(8, 6), dpi=100, layout="constrained")
        self.fig = fig
        self.ax = {}
        for col, lado in enumerate(LADOS):
            ax_v = fig.add_subplot(2, 2, 1 + col)
            ax_u = fig.add_subplot(2, 2, 3 + col, sharex=ax_v)
            ax_v.set_title(f"{NOMBRE_LADO[lado]} (ESP_{lado})", fontsize=10)
            ax_v.set_ylabel("velocidad [rpm eje]")
            ax_u.set_ylabel("PWM")
            ax_u.set_xlabel("t [s]")
            for ax in (ax_v, ax_u):
                ax.grid(True, alpha=0.3)
                ax.set_xlim(-VENTANA_S, 0)
                ax.tick_params(labelsize=8)
            self.ax[(lado, "v")] = ax_v
            self.ax[(lado, "u")] = ax_u

        self.lin = {}
        self.lineas_ax = {k: [] for k in self.ax}
        for m in MOTORES:
            lado, i = m[0], int(m[1])
            c = COLORES[i]
            self.lin[(m, "pv")], = self.ax[(lado, "v")].plot([], [], color=c, lw=1.6, label=f"{m} PV")
            self.lin[(m, "sp")], = self.ax[(lado, "v")].plot([], [], color=c, lw=1.0, ls="--", alpha=0.8, label=f"{m} SP")
            self.lin[(m, "u")],  = self.ax[(lado, "u")].plot([], [], color=c, lw=1.4, label=m)
            self.lineas_ax[(lado, "v")] += [self.lin[(m, "pv")], self.lin[(m, "sp")]]
            self.lineas_ax[(lado, "u")].append(self.lin[(m, "u")])
        for lado in LADOS:
            self.ax[(lado, "v")].legend(fontsize=7, ncol=3, loc="upper left")

        self.canvas = FigureCanvasTkAgg(fig, master=padre)
        barra = ttk.Frame(padre)
        barra.pack(side="bottom", fill="x")
        tb = NavigationToolbar2Tk(self.canvas, barra, pack_toolbar=False)
        tb.update()
        tb.pack(side="left")
        self.var_pausa = tk.BooleanVar(value=False)
        ttk.Checkbutton(barra, text="Pausar gráficas (para hacer zoom)", variable=self.var_pausa).pack(side="left", padx=10)
        self.canvas.get_tk_widget().pack(side="top", fill="both", expand=True)

    def _construir_ganancias(self, padre):
        lf = ttk.LabelFrame(padre, text="Modelo y ganancias PID de cada motor   "
                                        "u = Kp·[ e + (1/Ti)∫e dt + Td·de/dt ]", padding=6)
        lf.pack(side="top", fill="x")

        # Una fila por motor, de izquierda a derecha: modelo → juego REF → juego PERT → en vivo
        tabla = ttk.Frame(lf)
        tabla.pack(side="top", fill="x")
        negrita = ("Segoe UI", 9, "bold")
        COL_MODELO, COL_ZM, COL_JUEGO, COL_VIVO = 1, 4, {"REF": 6, "PERT": 11}, 16
        ttk.Label(tabla, text="Modelo de la planta", font=negrita).grid(row=0, column=COL_MODELO, columnspan=4)
        ttk.Label(tabla, text="Referencia · ITAE", font=negrita).grid(row=0, column=COL_JUEGO["REF"], columnspan=4)
        ttk.Label(tabla, text="Perturbación · IAE", font=negrita).grid(row=0, column=COL_JUEGO["PERT"], columnspan=4)
        ttk.Label(tabla, text="En vivo", font=negrita).grid(row=0, column=COL_VIVO, columnspan=4)
        encabezados = ["Motor", "K", "τ [s]", "θ [s]", "ZM", "", "Kp", "Ti [s]", "Td [s]", "N", "",
                       "Kp", "Ti [s]", "Td [s]", "N", "", "% Ref", "SP", "PV", "PWM"]
        for c, txt in enumerate(encabezados):
            ttk.Label(tabla, text=txt, foreground=GRIS).grid(row=1, column=c, padx=2)
        for c in (5, 10, 15):
            ttk.Separator(tabla, orient="vertical").grid(row=0, column=c, rowspan=8, sticky="ns", padx=3)

        self.ent = {}       # (motor, juego, campo) → (Entry, StringVar)
        self.ent_mod = {}   # (motor, campo del modelo) → (Entry, StringVar)
        self.ent_zm = {}    # motor → (Entry, StringVar) de su zona muerta
        self.lbl_vivo = {}
        for fila, m in enumerate(MOTORES, start=2):
            tk.Label(tabla, text=m, fg=COLORES[int(m[1])], font=negrita).grid(row=fila, column=0)
            for cc, campo in enumerate(CAMPOS_MODELO):
                var = tk.StringVar()
                e = tk.Entry(tabla, textvariable=var, width=6, justify="right", relief="solid", bd=1)
                e.grid(row=fila, column=COL_MODELO + cc, padx=1, pady=1)
                self.ent_mod[(m, campo)] = (e, var)
                var.trace_add("write", lambda *_a, k=(m, campo): self._colorear_modelo(k))
            var = tk.StringVar()
            e = tk.Entry(tabla, textvariable=var, width=4, justify="right", relief="solid", bd=1)
            e.grid(row=fila, column=COL_ZM, padx=1, pady=1)
            self.ent_zm[m] = (e, var)
            var.trace_add("write", lambda *_a, k=m: self._colorear_zm(k))
            for juego in JUEGOS:
                for cc, campo in enumerate(CAMPOS):
                    var = tk.StringVar()
                    e = tk.Entry(tabla, textvariable=var, width=6, justify="right", relief="solid", bd=1)
                    e.grid(row=fila, column=COL_JUEGO[juego] + cc, padx=1, pady=1)
                    clave = (m, juego, campo)
                    self.ent[clave] = (e, var)
                    var.trace_add("write", lambda *_a, k=clave: self._colorear(k))
            for cc, (campo, ancho) in enumerate((("reg", 4), ("sp", 6), ("pv", 6), ("u", 4))):
                lbl = ttk.Label(tabla, text="—", width=ancho, anchor="e", font=("Consolas", 9))
                lbl.grid(row=fila, column=COL_VIVO + cc, padx=1)
                self.lbl_vivo[(m, campo)] = lbl
        ttk.Label(lf, foreground=GRIS, wraplength=760, justify="left",
                  text="Modelo: G(s) = K·e^(−θs) / (τs + 1) de la prueba de escalón, con K en rpm/PWM "
                       "(K = 0: motor sin identificar). ZM: zona muerta en PWM; el PID se la salta "
                       "sumándola a su salida (0 = sin compensar)."
                  ).pack(side="top", anchor="w", pady=(2, 0))

        cfg_f = ttk.Frame(lf)
        cfg_f.pack(side="top", fill="x", pady=(6, 0))
        self.ent_cfg = {}
        for nombre, etiqueta in (("UMAX", "UMAX [PWM]"), ("TS", "TS [ms]"), ("BANDA", "BANDA [rpm]"),
                                 ("TMEZ", "Mezcla [ms]")):
            ttk.Label(cfg_f, text=etiqueta).pack(side="left")
            var = tk.StringVar()
            e = tk.Entry(cfg_f, textvariable=var, width=6, justify="right", relief="solid", bd=1)
            e.pack(side="left", padx=(2, 12))
            self.ent_cfg[nombre] = (e, var)
            var.trace_add("write", lambda *_a, k=nombre: self._colorear_cfg(k))
        ttk.Label(cfg_f, text="Derivada sobre").pack(side="left")
        self.var_dmode = tk.StringVar(value="E")
        cb = ttk.Combobox(cfg_f, textvariable=self.var_dmode, values=("E", "PV"), width=4, state="readonly")
        cb.pack(side="left", padx=2)

        # ---- Cálculo de ganancias desde el modelo (usa TS y la derivada de arriba) ----
        calc = ttk.Frame(lf)
        calc.pack(side="top", fill="x", pady=(8, 0))
        self.btn_calcular = ttk.Button(calc, text="Calcular ganancias con el modelo", command=self.calcular_ganancias)
        self.btn_calcular.pack(side="left")
        ttk.Label(calc, text="Estructura").pack(side="left", padx=(10, 2))
        self.var_estructura = tk.StringVar(value="PI")
        ttk.Combobox(calc, textvariable=self.var_estructura, values=sintonia.ESTRUCTURAS,
                     width=4, state="readonly").pack(side="left")
        ttk.Label(calc, text="Margen de ganancia ≥").pack(side="left", padx=(10, 2))
        self.var_margen = tk.StringVar(value=f"{sintonia.MARGEN_DEFECTO:g}")
        ttk.Entry(calc, textvariable=self.var_margen, width=5, justify="right").pack(side="left")
        ttk.Label(calc, text="(1 = óptimo sin restricción)", foreground=GRIS).pack(side="left", padx=4)
        self.lbl_calc = ttk.Label(lf, foreground=GRIS, wraplength=760, justify="left",
                                  text="Llena la tabla con los dos juegos de cada motor que tenga modelo, para el TS de arriba. "
                                       "No envía nada a los ESP.")
        self.lbl_calc.pack(side="top", anchor="w", pady=(2, 0))

        botones = ttk.Frame(lf)
        botones.pack(side="top", fill="x", pady=(8, 0))
        ttk.Button(botones, text="Enviar a los ESP", command=self.enviar_ganancias).pack(side="left")
        ttk.Button(botones, text="Guardar archivo", command=self.guardar_archivo).pack(side="left", padx=4)
        ttk.Button(botones, text="Recargar archivo", command=self.recargar_archivo).pack(side="left")
        ttk.Button(botones, text="Guardar en memoria de los ESP…", command=self.guardar_en_esp).pack(side="left", padx=4)
        self.lbl_g = ttk.Label(lf, text="", foreground=GRIS)
        self.lbl_g.pack(side="top", anchor="w", pady=(4, 0))
        ttk.Label(lf, foreground=GRIS, wraplength=760, justify="left",
                  text="Ganancias y ZM: blanco = confirmado por el ESP · amarillo = sin enviar · rojo = inválido o no coincide.\n"
                       "% Ref: cuánto pesa el juego Referencia. Sube a 100 cuando el SP cambia más que BANDA; "
                       "cuando el error entra en BANDA baja a 0 (juego Perturbación) en el tiempo de Mezcla.\n"
                       "Derivada sobre: E = el error, PV = la medición."
                  ).pack(side="top", anchor="w")

    def _construir_experimento(self, padre):
        lf = ttk.LabelFrame(padre, text="Experimento", padding=6)
        lf.pack(side="top", fill="x", pady=(8, 0))

        fila = ttk.Frame(lf)
        fila.pack(side="top", fill="x")
        self.var_tipo = tk.StringVar(value="PWM")
        ttk.Radiobutton(fila, text="Escalones de PWM (lazo abierto: identificar)", value="PWM",
                        variable=self.var_tipo, command=self._cambiar_tipo).pack(side="left")
        ttk.Radiobutton(fila, text="Escalón de setpoint (lazo cerrado: validar)", value="SP",
                        variable=self.var_tipo, command=self._cambiar_tipo).pack(side="left", padx=10)

        fila = ttk.Frame(lf)
        fila.pack(side="top", fill="x", pady=(6, 0))
        self.var_mot = {}
        for lado in LADOS:
            ttk.Label(fila, text=f"{NOMBRE_LADO[lado]}:").pack(side="left")
            for i in range(3):
                m = f"{lado}{i}"
                v = tk.BooleanVar(value=True)
                ttk.Checkbutton(fila, text=m, variable=v).pack(side="left")
                self.var_mot[m] = v
            ttk.Label(fila, text="  ").pack(side="left")
        for txt, sel in (("Todos", MOTORES), ("Izq", MOTORES[:3]), ("Der", MOTORES[3:]), ("Ninguno", [])):
            ttk.Button(fila, text=txt,
                       command=lambda s=sel: [self.var_mot[m].set(m in s) for m in MOTORES]).pack(side="left", padx=1)

        # Parámetros: un marco por tipo de experimento; solo se muestra el del tipo elegido
        params = ttk.Frame(lf)
        params.pack(side="top", fill="x", pady=(6, 0))
        self.marco_tipo = {"PWM": ttk.Frame(params), "SP": ttk.Frame(params)}

        # ---- Lazo abierto: escalera de niveles de PWM, como der_escalon / izq_escalon ----
        pwm = self.marco_tipo["PWM"]
        ttk.Label(pwm, text="Niveles de PWM").grid(row=0, column=0, sticky="e", padx=(0, 2))
        self.var_niveles = tk.StringVar(value="100 150 100")
        ttk.Entry(pwm, textvariable=self.var_niveles, width=52).grid(row=0, column=1, columnspan=5, sticky="w")
        ttk.Label(pwm, text="(en orden, separados por espacio)", foreground=GRIS).grid(row=0, column=6, sticky="w", padx=6)
        ttk.Label(pwm, text="t por nivel [s]").grid(row=1, column=0, sticky="e", padx=(0, 2), pady=(4, 0))
        self.var_t_nivel = tk.StringVar(value="2")
        ttk.Entry(pwm, textvariable=self.var_t_nivel, width=7, justify="right").grid(row=1, column=1, sticky="w", pady=(4, 0))
        ttk.Label(pwm, text="Muestreo [ms]").grid(row=1, column=2, sticky="e", padx=(12, 2), pady=(4, 0))
        self.var_ts_prueba = tk.StringVar(value="10")
        ttk.Entry(pwm, textvariable=self.var_ts_prueba, width=5, justify="right").grid(row=1, column=3, sticky="w", pady=(4, 0))
        self.lbl_escalera = ttk.Label(pwm, text="", foreground=GRIS)
        self.lbl_escalera.grid(row=1, column=4, columnspan=3, sticky="w", padx=(12, 0), pady=(4, 0))
        for var in (self.var_niveles, self.var_t_nivel):
            var.trace_add("write", lambda *_a: self._resumir_escalera())

        # ---- Lazo cerrado: inicial → escalón → inicial ----
        sp = self.marco_tipo["SP"]
        self.var_exp = {}
        campos = [("v0", "SP inicial y final [rpm]", "0"), ("v1", "SP del escalón [rpm]", "10"),
                  ("t_antes", "t antes [s]", "1"), ("t_escalon", "t escalón [s]", "3"), ("t_despues", "t después [s]", "3")]
        for k, (clave, etiqueta, defecto) in enumerate(campos):
            r, c = divmod(k, 3)
            ttk.Label(sp, text=etiqueta).grid(row=r, column=c * 2, sticky="e", padx=(12 if c else 0, 2), pady=(4 if r else 0, 0))
            var = tk.StringVar(value=defecto)
            ttk.Entry(sp, textvariable=var, width=7, justify="right").grid(row=r, column=c * 2 + 1, sticky="w", pady=(4 if r else 0, 0))
            self.var_exp[clave] = var

        fila = ttk.Frame(lf)
        fila.pack(side="top", fill="x", pady=(4, 0))
        ttk.Label(fila, text="Notas").pack(side="left")
        self.var_notas = tk.StringVar()
        ttk.Entry(fila, textvariable=self.var_notas).pack(side="left", fill="x", expand=True, padx=(4, 0))

        fila = ttk.Frame(lf)
        fila.pack(side="top", fill="x", pady=(8, 0))
        self.btn_iniciar = ttk.Button(fila, text="▶ Iniciar experimento", command=self.iniciar_experimento)
        self.btn_iniciar.pack(side="left")
        self.lbl_prog = ttk.Label(fila, text="", font=("Segoe UI", 9, "bold"))
        self.lbl_prog.pack(side="left", padx=10)
        ttk.Button(fila, text="Abrir registros", command=self.abrir_registros).pack(side="right")
        self.lbl_ultimo = ttk.Label(lf, text="", foreground=GRIS)
        self.lbl_ultimo.pack(side="top", anchor="w", pady=(4, 0))
        self.lbl_perfil = ttk.Label(lf, foreground=GRIS, wraplength=760, justify="left")
        self.lbl_perfil.pack(side="top", anchor="w")
        self._cambiar_tipo()
        self._resumir_escalera()

    def _construir_consola(self, padre):
        lf = ttk.LabelFrame(padre, text="Consola (mensajes de los ESP y comandos directos)", padding=4)
        lf.pack(side="bottom", fill="x", padx=6, pady=6)
        fila = ttk.Frame(lf)
        fila.pack(side="bottom", fill="x", pady=(4, 0))
        self.var_destino = tk.StringVar(value="Ambos")
        ttk.Combobox(fila, textvariable=self.var_destino, values=("Ambos", "Izquierdo", "Derecho"),
                     width=10, state="readonly").pack(side="left")
        self.var_cmd = tk.StringVar()
        e = ttk.Entry(fila, textvariable=self.var_cmd)
        e.pack(side="left", fill="x", expand=True, padx=4)
        e.bind("<Return>", lambda _e: self.enviar_directo())
        ttk.Button(fila, text="Enviar", command=self.enviar_directo).pack(side="left")
        ttk.Label(fila, text="ej. EST, CFG, REG REF *", foreground=GRIS).pack(side="left", padx=6)

        self.txt = tk.Text(lf, height=8, font=("Consolas", 9), state="disabled", wrap="none")
        sb = ttk.Scrollbar(lf, command=self.txt.yview)
        self.txt.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self.txt.pack(side="left", fill="both", expand=True)

    # ================================================================== #
    #  Tabla de ganancias                                                  #
    # ================================================================== #

    def _poner_tabla(self, g: dict, modelos: dict, cfg: dict):
        for (m, c), (_e, var) in self.ent_mod.items():
            var.set(f"{modelos[m][c]:g}")
        for m, (_e, var) in self.ent_zm.items():
            var.set(f"{g[m]['ZM']:g}")
        for (m, j, c), (_e, var) in self.ent.items():
            var.set(f"{g[m][j][c]:g}")
        for nombre, (_e, var) in self.ent_cfg.items():
            var.set(f"{cfg[nombre]:g}")
        self.var_dmode.set(cfg["DMODE"])

    def _leer_tabla(self):
        """(ganancias, cfg) de la tabla. ValueError con el primer campo inválido."""
        g = {m: {j: {} for j in JUEGOS} for m in MOTORES}
        for (m, j, c), (_e, var) in self.ent.items():
            v = _a_float(var.get())
            if v is None or v < 0:
                raise ValueError(f"{m} {NOMBRE_JUEGO[j]} {c}: '{var.get()}' no es un número >= 0")
            g[m][j][c] = v
        cfg = self._leer_cfg()
        for m, (_e, var) in self.ent_zm.items():
            v = _a_float(var.get())
            if v is None or v < 0:
                raise ValueError(f"Zona muerta de {m}: '{var.get()}' no es un número >= 0")
            if v > 0 and v >= cfg["UMAX"]:
                raise ValueError(f"Zona muerta de {m}: debe ser menor que UMAX ({cfg['UMAX']:g})")
            g[m]["ZM"] = v
        return g, cfg

    def _leer_cfg(self):
        cfg = {}
        for nombre, (_e, var) in self.ent_cfg.items():
            v = _a_float(var.get())
            lo, hi = LIMITES_CFG[nombre]
            if v is None or not lo <= v <= hi:
                raise ValueError(f"{nombre}: '{var.get()}' fuera de rango [{lo:g}, {hi:g}]")
            cfg[nombre] = int(v) if nombre in ("TS", "TMEZ") else v
        cfg["DMODE"] = self.var_dmode.get()
        return cfg

    def _leer_modelos(self):
        """{motor: {"K":.., "tau":.., "theta":..}} de la tabla. ValueError con el primer campo inválido."""
        modelos = {m: {} for m in MOTORES}
        for (m, c), (_e, var) in self.ent_mod.items():
            v = _a_float(var.get())
            if v is None or v < 0:
                raise ValueError(f"Modelo de {m}, {c}: '{var.get()}' no es un número >= 0")
            modelos[m][c] = v
        return modelos

    def _colorear_modelo(self, clave):
        e, var = self.ent_mod[clave]
        v = _a_float(var.get())
        e.configure(bg=ROJO_CLARO if v is None or v < 0 else BLANCO)

    def _colorear_zm(self, m):
        e, var = self.ent_zm[m]
        v = _a_float(var.get())
        if v is None or v < 0:
            e.configure(bg=ROJO_CLARO)
        elif (m, "ZM") in self.confirmadas and math.isclose(v, self.confirmadas[(m, "ZM")], abs_tol=1e-9):
            e.configure(bg=BLANCO)
        else:
            e.configure(bg=AMARILLO)

    def _colorear(self, clave):
        e, var = self.ent[clave]
        v = _a_float(var.get())
        if v is None or v < 0:
            e.configure(bg=ROJO_CLARO)
        elif clave in self.confirmadas and math.isclose(v, self.confirmadas[clave], rel_tol=1e-9, abs_tol=1e-12):
            e.configure(bg=BLANCO)
        else:
            e.configure(bg=AMARILLO)

    def _colorear_cfg(self, nombre):
        e, var = self.ent_cfg[nombre]
        v = _a_float(var.get())
        if v is None:
            e.configure(bg=ROJO_CLARO)
        elif nombre in self.cfg_enviada and v == self.cfg_enviada[nombre]:
            e.configure(bg=BLANCO)
        else:
            e.configure(bg=AMARILLO)

    def _recolorear_todo(self):
        for k in self.ent:
            self._colorear(k)
        for m in self.ent_zm:
            self._colorear_zm(m)
        for k in self.ent_cfg:
            self._colorear_cfg(k)

    def _sin_confirmar(self, con_config: bool = False) -> bool:
        """True si alguna ganancia o zona muerta (o la configuración) no está confirmada por el ESP."""
        celdas = list(self.ent.values()) + list(self.ent_zm.values())
        if con_config:
            celdas += list(self.ent_cfg.values())
        return any(e.cget("bg") != BLANCO for e, _v in celdas)

    # ================================================================== #
    #  Cálculo de ganancias desde el modelo                                #
    # ================================================================== #

    def calcular_ganancias(self):
        """Calcula los juegos Referencia (ITAE) y Perturbación (IAE) de cada motor
        con modelo y los deja en la tabla. No envía nada a los ESP."""
        if self.calculando:
            return
        try:
            modelos = self._leer_modelos()
            cfg = self._leer_cfg()
            margen = _a_float(self.var_margen.get())
            if margen is None or margen < 1:
                raise ValueError(f"Margen de ganancia: '{self.var_margen.get()}' debe ser un número >= 1")
        except ValueError as e:
            messagebox.showerror("Calcular ganancias", str(e))
            return
        con_modelo = {m: sintonia.Modelo(**modelos[m]) for m in MOTORES
                      if modelos[m]["K"] > 0 and modelos[m]["tau"] > 0}
        if not con_modelo:
            messagebox.showinfo("Calcular ganancias",
                                "Ningún motor tiene modelo.\nLlena K, τ y θ de los motores que quieras calcular "
                                "(los que entrega Control Station con la prueba de escalón).")
            return

        estructura = self.var_estructura.get()
        self.calculando = True
        self.btn_calcular.state(["disabled"])
        self.lbl_calc.configure(text="Calculando…", foreground=NARANJA)

        def trabajo():
            resultados, errores, hechos = {}, {}, {}
            for m, modelo in con_modelo.items():
                try:
                    if modelo not in hechos:      # motores con el mismo modelo: un solo cálculo
                        hechos[modelo] = sintonia.calcular_motor(modelo, cfg["TS"] / 1000.0, estructura,
                                                                 cfg["DMODE"], margen)
                    resultados[m] = hechos[modelo]
                except Exception as e:
                    errores[m] = str(e)
            self.cola.put(("sintonia", resultados, errores, con_modelo, cfg["TS"], estructura))
        threading.Thread(target=trabajo, daemon=True).start()

    def _fin_calculo(self, resultados, errores, modelos, ts_ms, estructura):
        self.calculando = False
        self.btn_calcular.state(["!disabled"])
        for m, res in resultados.items():
            for j in JUEGOS:
                for c in CAMPOS:
                    self.ent[(m, j, c)][1].set(f"{res[j].g[c]:g}")
            partes = []
            for j in JUEGOS:
                r = res[j]
                partes.append(f"{NOMBRE_JUEGO[j]} Kp={r.g['Kp']:g} Ti={r.g['Ti']:g} Td={r.g['Td']:g} "
                              f"({r.criterio} = {r.costo:.3g}, margen ×{r.margen:.2f}"
                              f"{', limitada por el margen mínimo' if r.limitado else ''})")
            self.log(f"{m}: " + " · ".join(partes))
        for m, err in errores.items():
            self.log(f"{m}: no se pudo calcular ({err})")

        rapidos = [m for m in resultados if modelos[m].tau < ts_ms / 1000.0]
        if rapidos:
            self.log(f"Aviso: TS = {ts_ms} ms es mayor que τ de {', '.join(rapidos)}: ahí el muestreo limita al lazo "
                     "más que el motor. Con TS de 10 a 20 ms la respuesta mejora mucho (cambia TS y vuelve a calcular).")
        if resultados:
            self.lbl_calc.configure(foreground=VERDE, text=f"✓ {estructura} calculado para {', '.join(resultados)} con "
                                                           f"TS = {ts_ms} ms. Revisa y pulsa «Enviar a los ESP».")
        else:
            self.lbl_calc.configure(foreground=ROJO, text="✗ No se pudo calcular (ver consola)")

    def enviar_ganancias(self):
        try:
            g, cfg = self._leer_tabla()
        except ValueError as e:
            messagebox.showerror("Ganancias", str(e))
            return
        activos = [l for l in LADOS if l in self.enlaces and self.enlaces[l].conectado]
        if not activos:
            self.log("No hay ESP conectados: no se envió nada")
            return
        for lado in activos:
            self.enlaces[lado].enviar_ganancias(g, cfg)
        self._registrar_envio(g, cfg)
        self.log(f"Ganancias enviadas a: {', '.join(NOMBRE_LADO[l] for l in activos)}")

    def _registrar_envio(self, g, cfg):
        self.cfg_enviada = dict(cfg)
        self.g_enviadas = g
        self.verif = (g, time.monotonic())
        self.lbl_g.configure(text="Esperando confirmación de los ESP…", foreground=GRIS)
        self._recolorear_todo()

    def _revisar_confirmacion(self):
        if self.verif is None:
            return
        g, t_envio = self.verif
        espera = time.monotonic() - t_envio
        if espera < 0.6:
            return
        pendientes, malos = [], []
        for lado, enl in self.enlaces.items():
            if not enl.conectado:
                continue
            for (m, j), ok in enl.verificar_ganancias(g).items():
                # Lo que cubre cada acuse del ESP: la zona muerta, o las 4 ganancias de un juego
                valores = {(m, "ZM"): g[m]["ZM"]} if j == "ZM" else {(m, j, c): g[m][j][c] for c in CAMPOS}
                if ok:
                    self.confirmadas.update(valores)
                elif ok is None:
                    pendientes.append(f"{m} {j}")
                else:
                    malos.append(f"{m} {j}")
                    for clave in valores:
                        self.confirmadas.pop(clave, None)
        self._recolorear_todo()
        for m_j in malos:
            m, j = m_j.split()
            for e in ([self.ent_zm[m][0]] if j == "ZM" else [self.ent[(m, j, c)][0] for c in CAMPOS]):
                e.configure(bg=ROJO_CLARO)
        if malos:
            self.lbl_g.configure(text=f"✗ El ESP confirmó otros valores en: {', '.join(malos)}", foreground=ROJO)
            self.verif = None
        elif pendientes and espera < 3.0:
            return
        elif pendientes:
            self.lbl_g.configure(text=f"✗ Sin confirmación de: {', '.join(pendientes)} "
                                      "(¿firmware de sintonización actualizado en los ESP?)", foreground=ROJO)
            self.verif = None
        else:
            lados = ", ".join(NOMBRE_LADO[l] for l, e in self.enlaces.items() if e.conectado)
            self.lbl_g.configure(text=f"✓ Ganancias confirmadas por: {lados}", foreground=VERDE)
            self.verif = None

    def guardar_archivo(self):
        try:
            g, cfg = self._leer_tabla()
            modelos = self._leer_modelos()
        except ValueError as e:
            messagebox.showerror("Ganancias", str(e))
            return
        guardar_ganancias(self.ruta_g, g, modelos, cfg)
        self.log(f"Modelos y ganancias guardados en {self.ruta_g}")

    def recargar_archivo(self):
        try:
            g, modelos, cfg = cargar_ganancias(self.ruta_g)
        except Exception as e:
            messagebox.showerror("Ganancias", f"No se pudo leer {self.ruta_g.name}:\n{e}")
            return
        self._poner_tabla(g, modelos, cfg)
        self.log(f"Modelos y ganancias recargados de {self.ruta_g} (falta 'Enviar a los ESP')")

    def guardar_en_esp(self):
        activos = [e for e in self.enlaces.values() if e.conectado]
        if not activos:
            self.log("No hay ESP conectados")
            return
        aviso = ""
        if self._sin_confirmar(con_config=True):
            aviso = "\n\nOjo: la tabla tiene cambios sin enviar o sin confirmar. Se guarda lo que tiene cada ESP ahora."
        if not messagebox.askyesno("Guardar en los ESP",
                                   "Se guardan en la memoria de los ESP32 conectados las ganancias, la zona "
                                   "muerta y la configuración que tienen ahora. Las cargarán al reiniciar, "
                                   "también cuando se usen con connect.py." + aviso):
            return
        for enl in activos:
            enl.enviar("WD 0")          # que el watchdog no quede guardado encendido
            enl.enviar("GUARDAR")
            enl.enviar(f"WD {WD_MS}")
        self.log("GUARDAR enviado (revisa la confirmación de cada ESP en la consola)")

    # ================================================================== #
    #  Experimentos                                                        #
    # ================================================================== #

    def _cambiar_tipo(self):
        """Muestra los parámetros y la explicación del tipo de experimento elegido."""
        tipo = self.var_tipo.get()
        for marco in self.marco_tipo.values():
            marco.pack_forget()
        self.marco_tipo[tipo].pack(side="top", fill="x")
        self.lbl_perfil.configure(text=TEXTO_PERFIL[tipo])

    def _resumir_escalera(self):
        niveles = _a_niveles(self.var_niveles.get())
        t_nivel = _a_float(self.var_t_nivel.get())
        if not niveles or t_nivel is None or t_nivel <= 0:
            self.lbl_escalera.configure(text="niveles o tiempo inválidos", foreground=ROJO)
        else:
            self.lbl_escalera.configure(foreground=GRIS, text=f"{len(niveles)} nivel(es) · {len(niveles) * t_nivel:g} s en total")

    def _perfil_de_la_pantalla(self):
        """(perfil, muestreo de la prueba en ms o None) según el tipo elegido. ValueError si algo está mal."""
        if self.var_tipo.get() == "PWM":
            niveles = _a_niveles(self.var_niveles.get())
            t_nivel = _a_float(self.var_t_nivel.get())
            ts = _a_float(self.var_ts_prueba.get())
            if not niveles:
                raise ValueError("Niveles de PWM: escribe uno o más números separados por espacio, ej. 100 150 100")
            if t_nivel is None:
                raise ValueError("t por nivel: no es un número")
            if ts is None or ts != int(ts):
                raise ValueError("Muestreo: debe ser un número entero de milisegundos")
            return perfil_escalera(niveles, t_nivel), int(ts)
        valores = {k: _a_float(v.get()) for k, v in self.var_exp.items()}
        malos = [k for k, v in valores.items() if v is None]
        if malos:
            raise ValueError(f"Valor inválido en: {', '.join(malos)}")
        if valores["t_escalon"] <= 0:
            raise ValueError("t escalón debe ser > 0")
        return perfil_escalon(**valores), None

    def iniciar_experimento(self):
        if self.ejecutor is not None and self.ejecutor.is_alive():
            return
        try:
            perfil, ts_prueba = self._perfil_de_la_pantalla()
        except ValueError as e:
            messagebox.showerror("Experimento", str(e))
            return
        exp = Experimento(tipo=self.var_tipo.get(),
                          motores=[m for m in MOTORES if self.var_mot[m].get()],
                          perfil=perfil, ts_ms=ts_prueba, notas=self.var_notas.get().strip())
        errores = exp.validar()
        faltan = [NOMBRE_LADO[l] for l in exp.lados if not (l in self.enlaces and self.enlaces[l].conectado)]
        if faltan:
            errores.append(f"no hay conexión con: {', '.join(faltan)}")
        if errores:
            messagebox.showerror("Experimento", "\n".join(errores))
            return
        if exp.tipo == "SP" and self._sin_confirmar():
            self.log("Aviso: la tabla tiene cambios sin enviar; el experimento usa las ganancias que ya tienen los ESP")
        umax = self.cfg_enviada.get("UMAX", 255)
        if exp.tipo == "PWM" and max(abs(v) for _d, v in exp.perfil) > umax:
            self.log(f"Aviso: el firmware limita el PWM a UMAX = {umax:g}")
        ts_normal = self.cfg_enviada.get("TS")
        if exp.ts_ms is not None and ts_normal is None:
            self.log("Aviso: aún no se le ha enviado la configuración a los ESP, así que la prueba corre con el "
                     "muestreo que tengan y no con el de la prueba")

        try:
            modelos = self._leer_modelos()
        except ValueError:
            modelos = None
        meta = {"ganancias": self.g_enviadas, "modelos": modelos, "config": self.cfg_enviada,
                "puertos": {l: e.puerto for l, e in self.enlaces.items()}}
        self.ejecutor = Ejecutor(exp, dict(self.enlaces), self.datos, self.carpeta, meta,
                                 lambda ok, estado, ruta, txt: self.cola.put(("exp_fin", ok, estado, ruta, txt)),
                                 ts_normal_ms=ts_normal)
        self.ejecutor.start()
        self.btn_iniciar.state(["disabled"])
        self.btn_reconectar.state(["disabled"])
        self.log(f"Experimento {exp.tipo} en {exp.etiqueta_motores()}: {exp.resumen()}, {exp.duracion:.1f} s")

    def _fin_experimento(self, ok, estado, ruta, rutas_txt):
        self.btn_iniciar.state(["!disabled"])
        self.btn_reconectar.state(["!disabled"])
        self.lbl_prog.configure(text=("✓ Terminado" if ok else f"✗ {estado}"), foreground=VERDE if ok else ROJO)
        if ruta is not None:
            self.lbl_ultimo.configure(text=f"Último registro: {ruta.name}")
            self.log(f"Experimento {estado}. Registro: {ruta}")
            if rutas_txt:
                self.log("Datos por motor (t, PWM, RPM) para Control Station: " + ", ".join(r.name for r in rutas_txt))
        else:
            self.log(f"Experimento no iniciado: {estado}")

    def abrir_registros(self):
        self.carpeta.mkdir(parents=True, exist_ok=True)
        try:
            if sys.platform == "win32":
                os.startfile(self.carpeta)
            else:
                subprocess.Popen(["xdg-open" if sys.platform.startswith("linux") else "open", str(self.carpeta)])
        except Exception as e:
            self.log(f"No se pudo abrir {self.carpeta}: {e}")

    # ================================================================== #
    #  Conexión, PARO y consola                                            #
    # ================================================================== #

    def conectar(self):
        if self.buscando or (self.ejecutor is not None and self.ejecutor.is_alive()):
            return
        for enl in self.enlaces.values():
            enl.cerrar(restaurar=True)
        self.enlaces.clear()
        self.buscando = True
        for lado in LADOS:
            self.lbl_lado[lado].configure(text=f"{NOMBRE_LADO[lado]}: buscando…", fg=GRIS)
        self.log("Buscando ESP32…" + (f" en {' '.join(self.puertos)}" if self.puertos else ""))

        def trabajo():
            res, avisos = buscar_esps(self.puertos, abrir=self.abrir)
            self.cola.put(("conexion", res, avisos))
        threading.Thread(target=trabajo, daemon=True).start()

    def _conectados(self, res, avisos):
        self.buscando = False
        for a in avisos:
            self.log(a)
        try:
            g, cfg = self._leer_tabla()
        except ValueError as e:
            g = cfg = None
            self.log(f"Tabla inválida ({e}): se conecta sin mandar ganancias")
        for lado, (t, puerto) in res.items():
            enl = EnlaceESP(t, lado, puerto, self.datos.agregar,
                            lambda l, txt: self.cola.put(("msg", l, txt))).iniciar()
            self.enlaces[lado] = enl
            self.t_conexion[lado] = time.monotonic()
            enl.configurar(g, cfg)
            self.log(f"{NOMBRE_LADO[lado]} conectado en {puerto}")
        for lado in LADOS:
            if lado not in res:
                self.lbl_lado[lado].configure(text=f"{NOMBRE_LADO[lado]}: no encontrado", fg=ROJO)
        if res and g is not None:
            self._registrar_envio(g, cfg)

    def paro(self):
        for enl in self.enlaces.values():
            enl.enviar("STOP")
        if self.ejecutor is not None and self.ejecutor.is_alive():
            self.ejecutor.abortar("PARO")
        self.log("PARO: STOP enviado a todos los lados")

    def enviar_directo(self):
        cmd = self.var_cmd.get().strip()
        if not cmd:
            return
        destino = {"Ambos": LADOS, "Izquierdo": ("L",), "Derecho": ("R",)}[self.var_destino.get()]
        enviados = [l for l in destino if l in self.enlaces and self.enlaces[l].enviar(cmd)]
        if not enviados:
            self.log("Sin conexión con el destino elegido")
            return
        self.log(f"→ [{'+'.join(enviados)}] {cmd}")
        if cmd.upper().startswith("OUT") and cmd.upper().split()[-1] != "CSV":
            self.log("Aviso: la herramienta necesita OUT CSV para recibir datos (manda OUT CSV para regresar)")
        self.var_cmd.set("")

    def log(self, texto: str, lado: str = None):
        prefijo = f"[{lado}] " if lado else "[herramienta] "
        self.txt.configure(state="normal")
        self.txt.insert("end", f"{time.strftime('%H:%M:%S')} {prefijo}{texto}\n")
        lineas = int(self.txt.index("end-1c").split(".")[0])
        if lineas > 1500:
            self.txt.delete("1.0", f"{lineas - 1500}.0")
        self.txt.configure(state="disabled")
        self.txt.see("end")

    def cerrar(self):
        if self.ejecutor is not None and self.ejecutor.is_alive():
            self.ejecutor.abortar("ventana cerrada")
            self.ejecutor.join(timeout=1.0)
        for enl in self.enlaces.values():
            enl.cerrar(restaurar=True)
        self.root.destroy()

    # ================================================================== #
    #  Ciclos de refresco                                                  #
    # ================================================================== #

    def _tick(self):
        try:
            while True:
                ev = self.cola.get_nowait()
                if ev[0] == "msg":
                    self.log(ev[2], lado=ev[1])
                elif ev[0] == "conexion":
                    self._conectados(ev[1], ev[2])
                elif ev[0] == "exp_fin":
                    self._fin_experimento(*ev[1:])
                elif ev[0] == "sintonia":
                    self._fin_calculo(*ev[1:])
        except queue.Empty:
            pass

        self._revisar_confirmacion()
        self._actualizar_estado()
        if self.ejecutor is not None and self.ejecutor.is_alive():
            self.lbl_prog.configure(text=f"Corriendo {self.ejecutor.transcurrido:4.1f} / "
                                         f"{self.ejecutor.exp.duracion:.1f} s", foreground=NARANJA)
        self.root.after(POLL_MS, self._tick)

    def _actualizar_estado(self):
        for lado in LADOS:
            enl = self.enlaces.get(lado)
            if enl is None:
                continue
            edad = enl.edad_datos()
            nombre = NOMBRE_LADO[lado]
            if not enl.conectado:
                txt, color = f"{nombre}: desconectado ({enl.error})", ROJO
            elif not enl.acks_pid and time.monotonic() - self.t_conexion[lado] > 3.0:
                txt, color = f"{nombre}: {enl.puerto} no confirma ganancias (¿firmware de sintonización actualizado?)", ROJO
            elif edad < 0.5:
                txt, color = f"{nombre}: {enl.puerto} ✓", VERDE
            elif edad < DATOS_VIEJOS_S:
                txt, color = f"{nombre}: {enl.puerto} datos con retraso", NARANJA
            else:
                txt, color = f"{nombre}: {enl.puerto} sin datos", ROJO
            self.lbl_lado[lado].configure(text=txt, fg=color)

        for m in MOTORES:
            s = self.datos.ultima[m]
            if s is None or time.monotonic() - s.t_pc > DATOS_VIEJOS_S:
                vals = ("—", "—", "—", "—")
            else:
                vals = (s.reg, f"{s.sp:.2f}", f"{s.pv:.2f}", f"{s.u:.0f}")
            for campo, v in zip(("reg", "sp", "pv", "u"), vals):
                self.lbl_vivo[(m, campo)].configure(text=v)

    def _tick_grafica(self):
        espera = GRAFICA_MS
        if not self.var_pausa.get():
            ahora = time.monotonic()
            for m in MOTORES:
                pts = self.datos.recientes(m, ahora - VENTANA_S)
                pts = pts[::max(1, len(pts) // PUNTOS_GRAFICA)]   # con muestreo rápido sobran puntos para la pantalla
                t = [p.t_pc - ahora for p in pts]
                self.lin[(m, "pv")].set_data(t, [p.pv for p in pts])
                self.lin[(m, "sp")].set_data(t, [p.sp for p in pts])
                self.lin[(m, "u")].set_data(t, [p.u for p in pts])
            for (lado, tipo), ax in self.ax.items():
                ax.set_xlim(-VENTANA_S, 0)
                _escalar_y(ax, self.lineas_ax[(lado, tipo)], 2.0 if tipo == "v" else 20.0)
            self.canvas.draw()
            # Si dibujar tarda (laptop lenta, muchos datos), el refresco se espacia: la
            # ventana debe pasar más tiempo atendiendo botones y teclas (PARO) que dibujando.
            espera = max(GRAFICA_MS, int(2000 * (time.monotonic() - ahora)))
        self.root.after(espera, self._tick_grafica)


def _a_float(texto: str):
    try:
        v = float(texto.replace(",", "."))
    except ValueError:
        return None
    return v if math.isfinite(v) else None


def _a_niveles(texto: str):
    """Lista de números de un texto separado por espacios, comas o punto y coma.
    None si algo no es un número; lista vacía si no hay nada."""
    niveles = [_a_float(p) for p in re.split(r"[\s,;]+", texto.strip()) if p]
    return None if any(v is None for v in niveles) else niveles


def _escalar_y(ax, lineas, span_min: float):
    """Límites Y a partir de los datos visibles, con margen y un rango mínimo."""
    ys = [y for linea in lineas for y in linea.get_ydata()]
    if ys:
        lo, hi = min(ys), max(ys)
        margen = 0.08 * (hi - lo)
        lo, hi = lo - margen, hi + margen
    else:
        lo = hi = 0.0
    if hi - lo < span_min:
        c = (lo + hi) / 2.0
        lo, hi = c - span_min / 2.0, c + span_min / 2.0
    ax.set_ylim(lo, hi)


# ---------------------------------------------------------------------- #

def main(argv=None, abrir=abrir_puerto, al_iniciar=None):
    p = argparse.ArgumentParser(description="Sintonizador PID de los 6 motores del rover.")
    p.add_argument("--puertos", nargs="+", help="probar solo estos puertos (ej. COM5 COM7)")
    p.add_argument("--ganancias", default=str(AQUI / "ganancias.json"), help="archivo de ganancias")
    p.add_argument("--registros", default=str(AQUI / "registros"), help="carpeta de registros")
    args = p.parse_args(argv)

    if sys.platform == "win32":
        try:   # texto nítido en pantallas con escalado
            import ctypes
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except Exception:
            pass

    root = tk.Tk()
    root.title("Sintonizador PID — rover (6 motores)")
    root.geometry("1560x940")
    root.minsize(1250, 780)
    app = App(root, Path(args.ganancias), Path(args.registros), puertos=args.puertos, abrir=abrir)
    if al_iniciar is not None:
        al_iniciar(app)
    root.mainloop()
    return app


if __name__ == "__main__":
    main()
