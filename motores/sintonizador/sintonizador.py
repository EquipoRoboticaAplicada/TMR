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
  - Muestra en vivo SP, PV y PWM de los 6 motores.
  - Corre los experimentos que ustedes lanzan (escalón de PWM en lazo abierto
    o de setpoint en lazo cerrado) en un motor, un lado o los 6 a la vez, y
    guarda cada uno en registros/ (CSV + JSON con las ganancias usadas).
  - No calcula ni ajusta ganancias por su cuenta.

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
import subprocess
import sys
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure

from enlace import (CAMPOS, JUEGOS, LADOS, MOTORES, NOMBRE_LADO, WD_MS, EnlaceESP,
                    abrir_puerto, buscar_esps)
from experimento import DATOS_VIEJOS_S, Datos, Ejecutor, Experimento

AQUI = Path(__file__).resolve().parent

GANANCIA_DEFECTO = {"Kp": 0.5, "Ti": 0.5, "Td": 0.0, "N": 10.0}   # = der.ino / izq.ino
CONFIG_DEFECTO   = {"UMAX": 255, "TS": 100, "BANDA": 0.5, "DMODE": "E"}

VENTANA_S   = 20      # segundos visibles en las gráficas
POLL_MS     = 50      # cola de eventos
GRAFICA_MS  = 200     # refresco de gráficas
COLORES     = ("#1f77b4", "#d62728", "#2ca02c")   # motor 0, 1, 2

BLANCO, AMARILLO, ROJO_CLARO = "white", "#fff3b0", "#ffcdd2"
VERDE, NARANJA, ROJO, GRIS = "#2e7d32", "#ef6c00", "#c62828", "#757575"


# ---------------------------------------------------------------------- #
#  Archivo de ganancias                                                   #
# ---------------------------------------------------------------------- #

def ganancias_defecto():
    return {m: {j: dict(GANANCIA_DEFECTO) for j in JUEGOS} for m in MOTORES}


def cargar_ganancias(ruta: Path):
    """Regresa (ganancias, config). Crea el archivo con valores por defecto si no existe."""
    if not ruta.exists():
        guardar_ganancias(ruta, ganancias_defecto(), dict(CONFIG_DEFECTO))
    data = json.loads(ruta.read_text(encoding="utf-8"))
    cfg = {**CONFIG_DEFECTO, **data.get("config", {})}
    g = ganancias_defecto()
    for m in MOTORES:
        for j in JUEGOS:
            g[m][j].update({c: float(v) for c, v in data["motores"][m][j].items() if c in CAMPOS})
    return g, cfg


def guardar_ganancias(ruta: Path, g: dict, cfg: dict):
    """JSON legible: una línea por motor y juego, para editarlo a mano o verlo en git diff."""
    def obj(d):
        return "{" + ", ".join(f'"{k}": {json.dumps(v)}' for k, v in d.items()) + "}"
    lineas = [
        "{",
        '  "_comentario": "PID en forma ideal u = Kp*[e + (1/Ti)*integral(e) + Td*de/dt], filtro Td/N. '
        'Ti = 0 sin integral, Td = 0 sin derivada, N = 0 sin filtro. A = aceleracion, F = frenado. '
        'L = izquierdo (ESP_L), R = derecho (ESP_R). TS en ms, BANDA en rpm, UMAX en PWM.",',
        f'  "config": {obj(cfg)},',
        '  "motores": {',
    ]
    for k, m in enumerate(MOTORES):
        fin = "," if k < len(MOTORES) - 1 else ""
        lineas.append(f'    "{m}": {{"A": {obj(g[m]["A"])},')
        lineas.append(f'           "F": {obj(g[m]["F"])}}}{fin}')
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

        self.confirmadas = {}     # (motor, juego, campo) → valor confirmado por el firmware
        self.cfg_enviada = {}     # config que tienen los ESP
        self.g_enviadas  = None   # ganancias que tienen los ESP (para el registro)
        self.verif       = None   # (ganancias, t_envio) pendiente de confirmar

        self.valores_tipo = {"PWM": {"v0": "0", "v1": "120"}, "SP": {"v0": "0", "v1": "10"}}

        self._construir()
        try:
            g, cfg = cargar_ganancias(self.ruta_g)
        except Exception as e:
            messagebox.showerror("Ganancias", f"No se pudo leer {self.ruta_g.name}:\n{e}\nSe usan valores por defecto.")
            g, cfg = ganancias_defecto(), dict(CONFIG_DEFECTO)
        self._poner_tabla(g, cfg)
        self.log(f"Ganancias de {self.ruta_g}")

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
        lf = ttk.LabelFrame(padre, text="Ganancias PID (forma ideal)   u = Kp·[ e + (1/Ti)∫e dt + Td·de/dt ]", padding=6)
        lf.pack(side="top", fill="x")

        tabla = ttk.Frame(lf)
        tabla.pack(side="top", fill="x")
        ttk.Label(tabla, text="Aceleración (A)", font=("Segoe UI", 9, "bold")).grid(row=0, column=1, columnspan=4)
        ttk.Label(tabla, text="Frenado (F)", font=("Segoe UI", 9, "bold")).grid(row=0, column=6, columnspan=4)
        ttk.Label(tabla, text="En vivo", font=("Segoe UI", 9, "bold")).grid(row=0, column=11, columnspan=4)
        encabezados = ["Motor", "Kp", "Ti [s]", "Td [s]", "N", "", "Kp", "Ti [s]", "Td [s]", "N", "",
                       "Reg", "SP", "PV", "u"]
        for c, txt in enumerate(encabezados):
            ttk.Label(tabla, text=txt, foreground=GRIS).grid(row=1, column=c, padx=2)
        ttk.Separator(tabla, orient="vertical").grid(row=0, column=5, rowspan=8, sticky="ns", padx=4)
        ttk.Separator(tabla, orient="vertical").grid(row=0, column=10, rowspan=8, sticky="ns", padx=4)

        self.ent = {}
        self.lbl_vivo = {}
        for fila, m in enumerate(MOTORES, start=2):
            tk.Label(tabla, text=m, fg=COLORES[int(m[1])], font=("Segoe UI", 9, "bold")).grid(row=fila, column=0)
            for jj, juego in enumerate(JUEGOS):
                for cc, campo in enumerate(CAMPOS):
                    var = tk.StringVar()
                    e = tk.Entry(tabla, textvariable=var, width=7, justify="right", relief="solid", bd=1)
                    e.grid(row=fila, column=1 + jj * 5 + cc, padx=1, pady=1)
                    clave = (m, juego, campo)
                    self.ent[clave] = (e, var)
                    var.trace_add("write", lambda *_a, k=clave: self._colorear(k))
            for cc, campo in enumerate(("reg", "sp", "pv", "u")):
                lbl = ttk.Label(tabla, text="—", width=6, anchor="e", font=("Consolas", 9))
                lbl.grid(row=fila, column=11 + cc, padx=1)
                self.lbl_vivo[(m, campo)] = lbl

        cfg_f = ttk.Frame(lf)
        cfg_f.pack(side="top", fill="x", pady=(6, 0))
        self.ent_cfg = {}
        for nombre, etiqueta in (("UMAX", "UMAX [PWM]"), ("TS", "TS [ms]"), ("BANDA", "BANDA [rpm]")):
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
        ttk.Label(cfg_f, text="(E = error, PV = medición)", foreground=GRIS).pack(side="left")

        botones = ttk.Frame(lf)
        botones.pack(side="top", fill="x", pady=(8, 0))
        ttk.Button(botones, text="Enviar a los ESP", command=self.enviar_ganancias).pack(side="left")
        ttk.Button(botones, text="Guardar archivo", command=self.guardar_archivo).pack(side="left", padx=4)
        ttk.Button(botones, text="Recargar archivo", command=self.recargar_archivo).pack(side="left")
        ttk.Button(botones, text="Guardar en memoria de los ESP…", command=self.guardar_en_esp).pack(side="left", padx=4)
        self.lbl_g = ttk.Label(lf, text="", foreground=GRIS)
        self.lbl_g.pack(side="top", anchor="w", pady=(4, 0))
        ttk.Label(lf, foreground=GRIS, text="Blanco = confirmado por el ESP · amarillo = sin enviar · rojo = inválido o no coincide").pack(side="top", anchor="w")

    def _construir_experimento(self, padre):
        lf = ttk.LabelFrame(padre, text="Experimento", padding=6)
        lf.pack(side="top", fill="x", pady=(8, 0))

        fila = ttk.Frame(lf)
        fila.pack(side="top", fill="x")
        self.var_tipo = tk.StringVar(value="PWM")
        self._tipo_actual = "PWM"
        ttk.Radiobutton(fila, text="Escalón de PWM (lazo abierto: identificar)", value="PWM",
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

        params = ttk.Frame(lf)
        params.pack(side="top", fill="x", pady=(6, 0))
        self.var_exp = {}
        campos = [("v0", "Valor inicial y final", "0"), ("v1", "Valor del escalón", "120"),
                  ("t_antes", "t antes [s]", "1"), ("t_escalon", "t escalón [s]", "3"), ("t_despues", "t después [s]", "3")]
        self.lbl_unidad = []
        for k, (clave, etiqueta, defecto) in enumerate(campos):
            r, c = divmod(k, 3)
            ttk.Label(params, text=etiqueta).grid(row=r, column=c * 3, sticky="e", padx=(8 if c else 0, 2))
            var = tk.StringVar(value=defecto)
            ttk.Entry(params, textvariable=var, width=7, justify="right").grid(row=r, column=c * 3 + 1, sticky="w")
            if clave in ("v0", "v1"):
                lbl = ttk.Label(params, text="PWM", foreground=GRIS)
                lbl.grid(row=r, column=c * 3 + 2, sticky="w")
                self.lbl_unidad.append(lbl)
            self.var_exp[clave] = var
        ttk.Label(params, text="Notas").grid(row=2, column=0, sticky="e", pady=(4, 0))
        self.var_notas = tk.StringVar()
        ttk.Entry(params, textvariable=self.var_notas, width=60).grid(row=2, column=1, columnspan=8, sticky="we", pady=(4, 0))

        fila = ttk.Frame(lf)
        fila.pack(side="top", fill="x", pady=(8, 0))
        self.btn_iniciar = ttk.Button(fila, text="▶ Iniciar experimento", command=self.iniciar_experimento)
        self.btn_iniciar.pack(side="left")
        self.lbl_prog = ttk.Label(fila, text="", font=("Segoe UI", 9, "bold"))
        self.lbl_prog.pack(side="left", padx=10)
        ttk.Button(fila, text="Abrir registros", command=self.abrir_registros).pack(side="right")
        self.lbl_ultimo = ttk.Label(lf, text="", foreground=GRIS)
        self.lbl_ultimo.pack(side="top", anchor="w", pady=(4, 0))
        ttk.Label(lf, foreground=GRIS, wraplength=620, justify="left",
                  text="Perfil: inicial (t antes) → escalón (t escalón) → inicial (t después) → STOP. "
                       "Los motores no elegidos se quedan en SP = 0. Se registran los 6 motores.").pack(side="top", anchor="w")

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
        ttk.Label(fila, text="ej. EST, CFG, REG A *", foreground=GRIS).pack(side="left", padx=6)

        self.txt = tk.Text(lf, height=8, font=("Consolas", 9), state="disabled", wrap="none")
        sb = ttk.Scrollbar(lf, command=self.txt.yview)
        self.txt.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self.txt.pack(side="left", fill="both", expand=True)

    # ================================================================== #
    #  Tabla de ganancias                                                  #
    # ================================================================== #

    def _poner_tabla(self, g: dict, cfg: dict):
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
                raise ValueError(f"{m} {j} {c}: '{var.get()}' no es un número >= 0")
            g[m][j][c] = v
        cfg = {}
        limites = {"UMAX": (0, 255), "TS": (10, 1000), "BANDA": (0, 1e9)}
        for nombre, (_e, var) in self.ent_cfg.items():
            v = _a_float(var.get())
            lo, hi = limites[nombre]
            if v is None or not lo <= v <= hi:
                raise ValueError(f"{nombre}: '{var.get()}' fuera de rango [{lo:g}, {hi:g}]")
            cfg[nombre] = int(v) if nombre == "TS" else v
        cfg["DMODE"] = self.var_dmode.get()
        return g, cfg

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
        for k in self.ent_cfg:
            self._colorear_cfg(k)

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
                if ok:
                    for c in CAMPOS:
                        self.confirmadas[(m, j, c)] = g[m][j][c]
                elif ok is None:
                    pendientes.append(f"{m} {j}")
                else:
                    malos.append(f"{m} {j}")
                    for c in CAMPOS:
                        self.confirmadas.pop((m, j, c), None)
                        self.ent[(m, j, c)][0].configure(bg=ROJO_CLARO)
        self._recolorear_todo()
        for m_j in malos:
            m, j = m_j.split()
            for c in CAMPOS:
                self.ent[(m, j, c)][0].configure(bg=ROJO_CLARO)
        if malos:
            self.lbl_g.configure(text=f"✗ El ESP confirmó otros valores en: {', '.join(malos)}", foreground=ROJO)
            self.verif = None
        elif pendientes and espera < 3.0:
            return
        elif pendientes:
            self.lbl_g.configure(text=f"✗ Sin confirmación de: {', '.join(pendientes)} "
                                      "(¿firmware de sintonización cargado?)", foreground=ROJO)
            self.verif = None
        else:
            lados = ", ".join(NOMBRE_LADO[l] for l, e in self.enlaces.items() if e.conectado)
            self.lbl_g.configure(text=f"✓ Ganancias confirmadas por: {lados}", foreground=VERDE)
            self.verif = None

    def guardar_archivo(self):
        try:
            g, cfg = self._leer_tabla()
        except ValueError as e:
            messagebox.showerror("Ganancias", str(e))
            return
        guardar_ganancias(self.ruta_g, g, cfg)
        self.log(f"Ganancias guardadas en {self.ruta_g}")

    def recargar_archivo(self):
        try:
            g, cfg = cargar_ganancias(self.ruta_g)
        except Exception as e:
            messagebox.showerror("Ganancias", f"No se pudo leer {self.ruta_g.name}:\n{e}")
            return
        self._poner_tabla(g, cfg)
        self.log(f"Ganancias recargadas de {self.ruta_g} (falta 'Enviar a los ESP')")

    def guardar_en_esp(self):
        activos = [e for e in self.enlaces.values() if e.conectado]
        if not activos:
            self.log("No hay ESP conectados")
            return
        aviso = ""
        if any(e.cget("bg") != BLANCO for e, _v in list(self.ent.values()) + list(self.ent_cfg.values())):
            aviso = "\n\nOjo: la tabla tiene cambios sin enviar o sin confirmar. Se guarda lo que tiene cada ESP ahora."
        if not messagebox.askyesno("Guardar en los ESP",
                                   "Se guardan en la memoria de los ESP32 conectados las ganancias y la "
                                   "configuración que tienen ahora. Las cargarán al reiniciar, también "
                                   "cuando se usen con connect.py." + aviso):
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
        nuevo = self.var_tipo.get()
        if nuevo == self._tipo_actual:
            return
        self.valores_tipo[self._tipo_actual] = {k: self.var_exp[k].get() for k in ("v0", "v1")}
        for k, v in self.valores_tipo[nuevo].items():
            self.var_exp[k].set(v)
        for lbl in self.lbl_unidad:
            lbl.configure(text="PWM" if nuevo == "PWM" else "rpm")
        self._tipo_actual = nuevo

    def iniciar_experimento(self):
        if self.ejecutor is not None and self.ejecutor.is_alive():
            return
        valores = {k: _a_float(v.get()) for k, v in self.var_exp.items()}
        malos = [k for k, v in valores.items() if v is None]
        if malos:
            messagebox.showerror("Experimento", f"Valor inválido en: {', '.join(malos)}")
            return
        exp = Experimento(tipo=self.var_tipo.get(),
                          motores=[m for m in MOTORES if self.var_mot[m].get()],
                          notas=self.var_notas.get().strip(), **valores)
        errores = exp.validar()
        faltan = [NOMBRE_LADO[l] for l in exp.lados if not (l in self.enlaces and self.enlaces[l].conectado)]
        if faltan:
            errores.append(f"no hay conexión con: {', '.join(faltan)}")
        if errores:
            messagebox.showerror("Experimento", "\n".join(errores))
            return
        if any(e.cget("bg") != BLANCO for e, _v in self.ent.values()):
            self.log("Aviso: la tabla tiene cambios sin enviar; el experimento usa las ganancias que ya tienen los ESP")
        umax = self.cfg_enviada.get("UMAX", 255)
        if exp.tipo == "PWM" and max(abs(exp.v0), abs(exp.v1)) > umax:
            self.log(f"Aviso: el firmware limita el PWM a UMAX = {umax:g}")

        meta = {"ganancias": self.g_enviadas, "config": self.cfg_enviada,
                "puertos": {l: e.puerto for l, e in self.enlaces.items()}}
        self.ejecutor = Ejecutor(exp, dict(self.enlaces), self.datos, self.carpeta, meta,
                                 lambda ok, estado, ruta: self.cola.put(("exp_fin", ok, estado, ruta)))
        self.ejecutor.start()
        self.btn_iniciar.state(["disabled"])
        self.btn_reconectar.state(["disabled"])
        self.log(f"Experimento {exp.tipo} en {exp.etiqueta_motores()}: {exp.v0:g} → {exp.v1:g} → {exp.v0:g}, "
                 f"{exp.duracion:.1f} s")

    def _fin_experimento(self, ok, estado, ruta):
        self.btn_iniciar.state(["!disabled"])
        self.btn_reconectar.state(["!disabled"])
        self.lbl_prog.configure(text=("✓ Terminado" if ok else f"✗ {estado}"), foreground=VERDE if ok else ROJO)
        if ruta is not None:
            self.lbl_ultimo.configure(text=f"Último registro: {ruta.name}")
            self.log(f"Experimento {estado}. Registro: {ruta}")
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
                txt, color = f"{nombre}: {enl.puerto} sin respuesta a comandos (¿firmware original?)", ROJO
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
        if not self.var_pausa.get():
            ahora = time.monotonic()
            for m in MOTORES:
                pts = self.datos.recientes(m, ahora - VENTANA_S)
                t = [p.t_pc - ahora for p in pts]
                self.lin[(m, "pv")].set_data(t, [p.pv for p in pts])
                self.lin[(m, "sp")].set_data(t, [p.sp for p in pts])
                self.lin[(m, "u")].set_data(t, [p.u for p in pts])
            for (lado, tipo), ax in self.ax.items():
                ax.set_xlim(-VENTANA_S, 0)
                _escalar_y(ax, self.lineas_ax[(lado, tipo)], 2.0 if tipo == "v" else 20.0)
            self.canvas.draw_idle()
        self.root.after(GRAFICA_MS, self._tick_grafica)


def _a_float(texto: str):
    try:
        v = float(texto.replace(",", "."))
    except ValueError:
        return None
    return v if math.isfinite(v) else None


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
