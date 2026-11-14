/*
  ===========================================================================
   SINTONIZACIÓN PID DE VELOCIDAD — LADO DERECHO (ESP_R)
  ===========================================================================
  Basado en der.ino: mismos pines, encoder, datos del motor, PWM y trama de
  telemetría, así que connect.py lo sigue reconociendo como "ESP_R" y acepta
  los mismos comandos S<rpm>.

  Plataforma: ESP32 (Arduino framework, core 3.x) + 3 puentes IBT-2

  Conexiones (ajusta si tu cableado difiere):
    Motor   IN2(LPWM)  IN1(RPWM)   ENC_B   ENC_A
      0        14         13          4       5
      1        27         26         16      17
      2        23         25         18      19

  Nota: R_EN y L_EN de cada IBT-2 se manejan como salidas digitales
  simples (sin PWM) y se ponen en HIGH en setup() para activar los
  drivers.

  ---------------------------------------------------------------------------
   CONTROLADOR: PID en estructura IDEAL (ISA / estándar), uno por motor
  ---------------------------------------------------------------------------
        u(t) = Kp · [ e(t) + (1/Ti)·∫e dt + Td·de/dt ]

    con filtro de primer orden en la derivada:  Td·s / (1 + (Td/N)·s)

      Kp : ganancia proporcional   [PWM/rpm]
      Ti : tiempo integral         [s]    Ti = 0 → sin acción integral
      Td : tiempo derivativo       [s]    Td = 0 → sin acción derivativa
      N  : filtro de la derivada   [-]    N  = 0 → derivada sin filtro

    e = SP - PV en rpm del eje de salida; u en PWM (-255..255).
    Discretización con el Ts real medido: integral por Euler hacia atrás,
    derivada por diferencia hacia atrás con su filtro.

  Dos juegos de ganancias por motor, uno por criterio de sintonía. Los
  calcula sintonizador.py a partir del modelo K, τ, θ de cada motor:
      REF  = seguimiento de REFERENCIA    (ITAE ante cambios de setpoint)
      PERT = rechazo de PERTURBACIONES    (IAE con el setpoint fijo)

    Las ganancias que actúan son una mezcla de los dos juegos, que se
    recalcula en cada muestra con un peso alfa (porcentaje del juego REF):
      - cuando el setpoint cambia más que BANDA, alfa = 100 %: actúa REF;
      - cuando la velocidad llega (|e| <= BANDA), alfa baja en rampa hasta
        0 % en TMEZ milisegundos: queda PERT regulando;
      - REG REF / REG PERT dejan fijo un juego, para probar cada uno.
    Se interpolan las ganancias en forma paralela (Kp, Kp/Ti, Kp·Td) y la
    constante del filtro. El término integral se guarda en unidades de PWM:
    al cambiar la mezcla o las ganancias en vivo la integral no salta; solo
    cambian P y D, que son chicos cerca del setpoint. Al pasar de MAN a AUTO
    la integral absorbe la diferencia, así que la salida continúa desde el
    PWM manual.

    Anti-windup: la integral se congela si empujaría la salida más allá de
    UMAX, y se limita a ±UMAX.

  Velocidad: las cuentas del encoder se dividen entre el tiempo real del
  primer al último flanco de la muestra, no entre el periodo de muestreo. Así
  la medición no se cuantiza y TS se puede bajar a 10-20 ms, que es lo que
  piden motores cuya constante de tiempo es de decenas de ms.

  ---------------------------------------------------------------------------
   COMANDOS POR SERIAL (115200, terminar con Enter, mayúsculas o minúsculas)
  ---------------------------------------------------------------------------
    m = motor 0, 1, 2 o * (los tres)      J = juego REF, PERT o * (los dos)

    S<rpm>                    setpoint a los 3 motores (formato de connect.py)
    SP m <rpm>                setpoint a un motor
    PID J m Kp Ti Td [N]      ganancias ideales de un juego
    KP|TI|TD|N J m <valor>    un solo parámetro
    REG AUTO|REF|PERT m       mezcla automática, o dejar fijo un juego
    MAN m <pwm>               lazo abierto: PWM fijo (para identificar la planta)
    AUTO m                    regresar al PID sin salto en la salida
    UMAX <0-255>              límite de |PWM| (PID y MAN)
    BANDA <rpm>               cambio de SP que activa REF y error que cuenta como "llegó"
    TMEZ <ms>                 duración de la rampa de REF a PERT (0 = cambio directo)
    TS <ms>                   tiempo de muestreo (10-1000)
    DMODE E|PV                derivada sobre el error (ideal) o sobre la medición
    WD <ms>                   watchdog: SP = 0 si no llegan comandos (0 = apagado)
    OUT ESP|PLOT|CSV|OFF      formato de salida por muestra
    VER m                     motor(es) que se imprimen en PLOT / CSV
    EST                       estado actual de los 3 motores
    CFG                       configuración actual en líneas copiables
    STOP  (o X)               PARO: SP = 0, AUTO, reinicia estados PID
    RESET                     reinicia integral y derivada
    GUARDAR | CARGAR | FABRICA   memoria no volátil (sobrevive a reinicios)
    AYUDA (o H)               esta lista

  Los mensajes del firmware empiezan con "# ": connect.py los ignora y las
  líneas de CFG se pueden pegar tal cual (el "# " inicial se descarta).
*/

#include <Preferences.h>
#include <stdarg.h>

// ===========================================================================
//  LADO — lo único que cambia entre der_sintonizacion e izq_sintonizacion
// ===========================================================================
const char* ESP_ID = "ESP_R"; // connect.py identifica el puerto buscando "ESP_L"/"ESP_R" al inicio de línea
const char* LADO   = "DERECHO";

// ---------- Pines (uno por motor) ----------
const int IN1[3] = {14, 27, 23};   // IN2 originales pasaron a ser IN1
const int IN2[3] = {13, 26, 25};   // IN1 originales pasaron a ser IN2
const int ENC_A[3] = {5, 17, 19};
const int ENC_B[3] = {4, 16, 18};

// Pines de habilitación de cada IBT-2 (salidas digitales, HIGH = activo)
const int R_EN[3] = {22, 33, 2};
const int L_EN[3] = {21, 32, 15};

// ===========================================================================
//  HARDWARE — igual en ambos lados
// ===========================================================================
// ---------- PWM (LEDC) ----------
const int PWM_FREQ = 20000;   // 20 kHz, fuera del rango audible
const int PWM_RES  = 8;       // 8 bits -> 0-255
const int PWM_MAX  = 255;

// ---------- Datos del motor ----------
const float GEAR_RATIO      = 262;   // relación de reducción de la caja
const int   PULSES_PER_REV  = 16;    // cuentas del encoder por vuelta del motor
const float PULSES_PER_OUTPUT_REV = PULSES_PER_REV * GEAR_RATIO;

// Diámetro de rueda: se usa para calcular m/s de los 3 motores.
// Ajusta al diámetro real.
const float WHEEL_DIAM_M = 0.17f;
const float WHEEL_CIRC_M = 3.14159265f * WHEEL_DIAM_M;

// ===========================================================================
//  TIPOS (antes de cualquier función, por los prototipos que genera Arduino)
// ===========================================================================
enum Juego : uint8_t { REF = 0, PERT = 1 };
const char* JUEGO_NOMBRE[2] = {"REF", "PERT"};

enum Forzado : uint8_t { REG_AUTO = 0, REG_SOLO_REF = 1, REG_SOLO_PERT = 2 };

enum Salida : uint8_t { OUT_ESP = 0, OUT_PLOT = 1, OUT_CSV = 2, OUT_OFF = 3 };
const char* SALIDA_NOMBRE[4] = {"ESP", "PLOT", "CSV", "OFF"};

struct Ganancias {
  float Kp;   // [PWM/rpm]
  float Ti;   // [s]  0 = sin integral
  float Td;   // [s]  0 = sin derivada
  float N;    // [-]  0 = derivada sin filtro
};

// Ganancias de un juego en forma paralela: así se pueden interpolar término a
// término (un juego con Ti = 0 queda simplemente con ki = 0).
struct Paralela {
  float kp;   // [PWM/rpm]
  float ki;   // Kp/Ti  [PWM/(rpm·s)]
  float kd;   // Kp·Td  [PWM·s/rpm]
  float tf;   // Td/N   [s], constante del filtro de la derivada
};

struct Config {
  uint32_t  version;
  Ganancias g[3][2];   // [motor][REF/PERT]
  float     uMax;      // límite de |u| [PWM]
  float     banda;     // cambio de SP que activa REF y error que cuenta como "llegó" [rpm]
  uint16_t  tsMs;      // tiempo de muestreo [ms]
  uint16_t  wdMs;      // watchdog de comandos [ms], 0 = apagado
  uint16_t  tMezMs;    // rampa de REF a PERT [ms], 0 = cambio directo
  uint8_t   dSobrePV;  // 0 = derivada sobre el error (ideal), 1 = sobre la medición
};

struct Motor {
  float   sp = 0.0f;          // setpoint [rpm eje de salida]
  float   pv = 0.0f;          // velocidad medida [rpm eje de salida]
  float   e  = 0.0f;
  float   P = 0.0f, I = 0.0f, D = 0.0f;   // términos del PID [PWM]
  float   u  = 0.0f;          // salida aplicada [PWM]
  float   ePrev = 0.0f, pvPrev = 0.0f;
  bool    inicializado = false;   // false → la siguiente muestra no calcula derivada
  bool    manual = false;         // true → lazo abierto con uManual
  float   uManual = 0.0f;
  bool    bumpless = false;       // pendiente: inicializar integral al volver a AUTO

  // Mezcla de los dos juegos
  float   alfa = 0.0f;            // peso del juego REF: 1 = solo REF, 0 = solo PERT
  float   spAncla = 0.0f;         // setpoint del último cambio de referencia detectado
  bool    asentado = true;        // la velocidad ya llegó tras ese cambio
  Forzado forzado = REG_AUTO;

  // Medición de velocidad
  long     cuenta = 0;
  long     cuentaPrev = 0;
  uint32_t tFlanco = 0;           // instante del último flanco del encoder [us]
  uint32_t tFlancoPrev = 0;
  bool     huboFlanco = false;    // la muestra anterior tuvo flancos (hay referencia de tiempo)
};

// ===========================================================================
//  CONFIGURACIÓN Y ESTADO
// ===========================================================================
// Defaults = ganancias de prueba de der.ino (forma paralela Kp=0.5, Ki=1.0,
// Kd=0) convertidas a forma ideal: Ti = Kp/Ki = 0.5 s, Td = Kd/Kp = 0.
const Ganancias GANANCIAS_DEFECTO = {0.5f, 0.5f, 0.0f, 10.0f};
const float     UMAX_DEFECTO      = 255.0f;
const float     BANDA_DEFECTO     = 0.5f;   // rpm
const uint16_t  TS_DEFECTO        = 100;    // ms, igual que der.ino
const uint16_t  TMEZ_DEFECTO      = 500;    // ms

const uint32_t CFG_VERSION = 0x50494402;   // cambiar si se modifica struct Config
const char*    NVS_NS      = "pid_tuning";
const char*    NVS_KEY     = "cfg";

Config      cfg;
bool        cfgDesdeNVS = false;
Preferences prefs;

Motor   mot[3];
Salida  salida  = OUT_ESP;     // ESP por defecto: compatible con connect.py
uint8_t verMask = 0b111;       // motores que se imprimen en PLOT / CSV

uint32_t      tProgramadaUs  = 0;      // instante en que tocaba la última muestra
uint32_t      tMuestraUs     = 0;      // instante en que se leyeron los encoders
unsigned long lastCmdMs      = 0;
bool          wdDisparado    = false;
unsigned int  seq            = 0;

char   inBuf[128];
size_t inLen      = 0;
bool   inOverflow = false;

// ---------- Encoder (cuadratura completa x4) — misma lógica que der.ino ----------
// Además de la cuenta se guarda el instante del último flanco: con él la
// velocidad se calcula sobre el tiempo real entre flancos (ver medirVelocidad).
volatile long     encoderCount[3] = {0, 0, 0};
volatile uint8_t  lastEncoded[3]  = {0, 0, 0};
volatile uint32_t lastEdgeUs[3]   = {0, 0, 0};

void IRAM_ATTR isrEncoder(void* arg) {
  int i = (int)(intptr_t)arg;

  uint8_t MSB = digitalRead(ENC_A[i]);
  uint8_t LSB = digitalRead(ENC_B[i]);
  uint8_t encoded = (MSB << 1) | LSB;
  uint8_t sum = (lastEncoded[i] << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) { encoderCount[i]++; lastEdgeUs[i] = micros(); }
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) { encoderCount[i]--; lastEdgeUs[i] = micros(); }

  lastEncoded[i] = encoded;
}

// ===========================================================================
//  UTILIDADES
// ===========================================================================
// Mensaje informativo: siempre empieza con "# " para que connect.py lo ignore
void msg(const char* fmt, ...) {
  char buf[200];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  Serial.print("# ");
  Serial.println(buf);
}

void setMotor(int idx, int pwm) {
  if (pwm >= 0) {
    ledcWrite(IN1[idx], pwm);
    ledcWrite(IN2[idx], 0);
  } else {
    ledcWrite(IN1[idx], 0);
    ledcWrite(IN2[idx], -pwm);
  }
}

void cargarDefectos() {
  cfg.version = CFG_VERSION;
  for (int i = 0; i < 3; i++) {
    cfg.g[i][REF]  = GANANCIAS_DEFECTO;
    cfg.g[i][PERT] = GANANCIAS_DEFECTO;
  }
  cfg.uMax     = UMAX_DEFECTO;
  cfg.banda    = BANDA_DEFECTO;
  cfg.tsMs     = TS_DEFECTO;
  cfg.wdMs     = 0;
  cfg.tMezMs   = TMEZ_DEFECTO;
  cfg.dSobrePV = 0;
}

bool cargarNVS() {
  if (!prefs.begin(NVS_NS, true)) return false;   // aún no hay nada guardado
  bool ok = false;
  if (prefs.getBytesLength(NVS_KEY) == sizeof(Config)) {
    Config tmp;
    prefs.getBytes(NVS_KEY, &tmp, sizeof(tmp));
    if (tmp.version == CFG_VERSION) {
      cfg = tmp;
      ok = true;
    }
  }
  prefs.end();
  return ok;
}

void reiniciarEstado(Motor& m) {
  m.I = 0.0f;
  m.D = 0.0f;
  m.bumpless = false;
  m.inicializado = false;
}

// ===========================================================================
//  PID
// ===========================================================================
// Velocidad en rpm del eje de salida. Las cuentas de la muestra se dividen
// entre el tiempo real del primer al último flanco, no entre el periodo de
// muestreo: contando pulsos, a TS = 10 ms la lectura saltaría de 1.4 en 1.4 rpm.
float medirVelocidad(Motor& m, uint32_t dtMuestraUs) {
  long delta   = m.cuenta - m.cuentaPrev;
  m.cuentaPrev = m.cuenta;
  if (delta == 0) {
    m.huboFlanco = false;
    return 0.0f;
  }
  // Saliendo del reposo no hay flanco anterior de referencia: esa muestra se
  // calcula con el periodo de muestreo.
  uint32_t dtUs = m.huboFlanco ? (m.tFlanco - m.tFlancoPrev) : dtMuestraUs;
  if (dtUs == 0) dtUs = dtMuestraUs;
  m.tFlancoPrev = m.tFlanco;
  m.huboFlanco  = true;
  return ((float)delta * 60.0e6f) / ((float)dtUs * PULSES_PER_OUTPUT_REV);
}

// Peso del juego REF. Un cambio de setpoint lo lleva a 1; cuando la velocidad
// llega (|e| <= BANDA) baja en rampa hasta 0 en TMEZ, y queda regulando PERT.
void actualizarMezcla(Motor& m, float dt) {
  if (m.forzado != REG_AUTO) {
    m.alfa     = (m.forzado == REG_SOLO_REF) ? 1.0f : 0.0f;
    m.spAncla  = m.sp;
    m.asentado = true;
    return;
  }

  if (fabsf(m.sp - m.spAncla) > cfg.banda) {   // cambió la referencia
    m.spAncla  = m.sp;
    m.alfa     = 1.0f;
    m.asentado = false;
  }
  if (!m.asentado && fabsf(m.e) <= cfg.banda) m.asentado = true;

  if (m.asentado && m.alfa > 0.0f) {
    float baja = (cfg.tMezMs > 0) ? dt * 1000.0f / (float)cfg.tMezMs : 1.0f;
    m.alfa = (m.alfa > baja) ? m.alfa - baja : 0.0f;
  }
}

Paralela aParalela(const Ganancias& g) {
  Paralela p;
  p.kp = g.Kp;
  p.ki = (g.Ti > 0.0f) ? g.Kp / g.Ti : 0.0f;
  p.kd = g.Kp * g.Td;
  p.tf = (g.N > 0.0f) ? g.Td / g.N : 0.0f;
  return p;
}

// Ganancias que actúan en esta muestra: alfa·REF + (1 - alfa)·PERT
Paralela mezclar(int i, float alfa) {
  Paralela r = aParalela(cfg.g[i][REF]);
  Paralela p = aParalela(cfg.g[i][PERT]);
  Paralela k;
  k.kp = p.kp + alfa * (r.kp - p.kp);
  k.ki = p.ki + alfa * (r.ki - p.ki);
  k.kd = p.kd + alfa * (r.kd - p.kd);
  k.tf = p.tf + alfa * (r.tf - p.tf);
  return k;
}

void pasoControl(int i, float dt, uint32_t dtMuestraUs) {
  Motor& m = mot[i];

  m.pv = medirVelocidad(m, dtMuestraUs);
  m.e  = m.sp - m.pv;

  if (!m.inicializado) {         // sin patada derivativa al arrancar / tras RESET
    m.ePrev  = m.e;
    m.pvPrev = m.pv;
    m.D      = 0.0f;
    m.inicializado = true;
  }

  actualizarMezcla(m, dt);
  Paralela k = mezclar(i, m.alfa);

  // P
  m.P = k.kp * m.e;

  // D filtrada:  D_k = Tf/(Tf+dt)·D_k-1 + Kp·Td/(Tf+dt)·Δx,  Tf = Td/N
  float dx = cfg.dSobrePV ? -(m.pv - m.pvPrev) : (m.e - m.ePrev);
  m.D = (k.tf / (k.tf + dt)) * m.D + (k.kd / (k.tf + dt)) * dx;
  m.ePrev  = m.e;
  m.pvPrev = m.pv;

  if (m.manual) {
    // Lazo abierto: P y D se calculan solo para mostrarlos
    m.u = constrain(m.uManual, -cfg.uMax, cfg.uMax);
  } else {
    // Bumpless MAN → AUTO: la integral absorbe la diferencia
    if (m.bumpless) {
      m.I = m.u - m.P - m.D;
      m.bumpless = false;
    }

    // I:  I_k = I_k-1 + (Kp/Ti)·e·dt, con anti-windup condicional
    if (k.ki > 0.0f) {
      float dI = k.ki * m.e * dt;
      float uPrueba = m.P + m.I + dI + m.D;
      bool  satura  = (uPrueba > cfg.uMax && dI > 0.0f) || (uPrueba < -cfg.uMax && dI < 0.0f);
      if (!satura) m.I += dI;
      m.I = constrain(m.I, -cfg.uMax, cfg.uMax);
    } else {
      m.I = 0.0f;
    }

    m.u = constrain(m.P + m.I + m.D, -cfg.uMax, cfg.uMax);
  }

  setMotor(i, (int)lroundf(m.u));
}

// ===========================================================================
//  SALIDA POR MUESTRA
// ===========================================================================
// A 115200 baudios caben ~11 500 caracteres por segundo y una fila CSV son
// unos 60: con TS corto no alcanza para imprimir todas las muestras de los 3
// motores, así que se imprime una de cada n. El control corre en todas.
uint32_t muestrasPorImpresion() {
  if (salida != OUT_CSV && salida != OUT_PLOT) return 1;
  uint32_t visibles = 0;
  for (int i = 0; i < 3; i++) if (verMask & (1 << i)) visibles++;
  uint32_t periodoMinMs = (visibles * 20 + 2) / 3;   // ~6.7 ms por motor visible
  uint32_t n = (periodoMinMs + cfg.tsMs - 1) / cfg.tsMs;
  return (n > 0) ? n : 1;
}

void imprimirMuestra(unsigned long t) {
  if (seq++ % muestrasPorImpresion() != 0) return;
  switch (salida) {
    case OUT_ESP: {
      // Trama de der.ino para connect.py: ESP_ID,seq,rpm0,v0,rpm1,v1,rpm2,v2
      Serial.print(ESP_ID); Serial.print(",");
      Serial.print(seq - 1);
      for (int i = 0; i < 3; i++) {
        float v_mps = (mot[i].pv / 60.0f) * WHEEL_CIRC_M;
        Serial.print(",");
        Serial.print(mot[i].pv, 2);
        Serial.print(",");
        Serial.print(v_mps, 4);
      }
      Serial.println();
      break;
    }
    case OUT_PLOT: {
      // Formato "etiqueta:valor" del Serial Plotter de Arduino IDE
      bool primero = true;
      for (int i = 0; i < 3; i++) {
        if (!(verMask & (1 << i))) continue;
        Serial.printf("%sSP%d:%.2f,PV%d:%.2f,U%d:%.1f",
                      primero ? "" : ",", i, mot[i].sp, i, mot[i].pv, i, mot[i].u);
        primero = false;
      }
      Serial.println();
      break;
    }
    case OUT_CSV: {
      // reg: "M" en manual, o el porcentaje del juego REF en la mezcla (0-100)
      for (int i = 0; i < 3; i++) {
        if (!(verMask & (1 << i))) continue;
        const Motor& m = mot[i];
        char reg[5];
        if (m.manual) strcpy(reg, "M");
        else          snprintf(reg, sizeof(reg), "%d", (int)lroundf(m.alfa * 100.0f));
        Serial.printf("%lu,%d,%s,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f,%.1f\n",
                      t, i, reg, m.sp, m.pv, m.e, m.P, m.I, m.D, m.u);
      }
      break;
    }
    case OUT_OFF:
      break;
  }
}

// ===========================================================================
//  REPORTES
// ===========================================================================
void imprimirGanancias(uint8_t motores, uint8_t juegos) {
  for (int i = 0; i < 3; i++) {
    if (!(motores & (1 << i))) continue;
    for (int r = 0; r < 2; r++) {
      if (!(juegos & (1 << r))) continue;
      const Ganancias& g = cfg.g[i][r];
      Serial.printf("# PID %s %d %.4f %.4f %.4f %.1f\n", JUEGO_NOMBRE[r], i, g.Kp, g.Ti, g.Td, g.N);
    }
  }
}

void imprimirVer() {
  if (verMask == 0b111) msg("VER *");
  else for (int i = 0; i < 3; i++) if (verMask & (1 << i)) msg("VER %d", i);
}

void imprimirConfig() {
  msg("===== Configuración — lado %s (%s) =====", LADO, ESP_ID);
  msg("origen: %s", cfgDesdeNVS ? "memoria no volátil (GUARDAR)" : "valores por defecto");
  msg("      J    m   Kp      Ti[s]   Td[s]   N");
  imprimirGanancias(0b111, 0b11);
  msg("UMAX %.0f", cfg.uMax);
  msg("BANDA %.2f", cfg.banda);
  msg("TMEZ %u", cfg.tMezMs);
  msg("TS %u", cfg.tsMs);
  msg("DMODE %s", cfg.dSobrePV ? "PV" : "E");
  msg("WD %u", cfg.wdMs);
  msg("OUT %s", SALIDA_NOMBRE[salida]);
  imprimirVer();
}

void imprimirEstado() {
  for (int i = 0; i < 3; i++) {
    const Motor& m = mot[i];
    const char* forz = (m.forzado == REG_AUTO) ? "auto" : "fijo";
    msg("m%d %s REF=%.0f%%(%s) SP=%.2f PV=%.2f e=%.2f P=%.1f I=%.1f D=%.1f u=%.0f enc=%ld",
        i, m.manual ? "MAN " : "AUTO", m.alfa * 100.0f, forz,
        m.sp, m.pv, m.e, m.P, m.I, m.D, m.u, m.cuenta);
  }
  if (cfg.wdMs > 0) msg("watchdog %u ms%s", cfg.wdMs, wdDisparado ? " (DISPARADO: SP = 0)" : "");
}

void imprimirAyuda() {
  msg("---------- Comandos (m = 0,1,2,*   J = REF,PERT,*) ----------");
  msg("S<rpm>                   setpoint a los 3 motores (connect.py), ej. S10");
  msg("SP m <rpm>               setpoint a un motor, ej. SP 0 8");
  msg("PID J m Kp Ti Td [N]     ganancias ideales, ej. PID REF * 6 0.03 0 10");
  msg("KP|TI|TD|N J m <valor>   un parámetro, ej. TI PERT 1 0.03");
  msg("REG AUTO|REF|PERT m      mezcla automática o dejar fijo un juego");
  msg("MAN m <pwm>              lazo abierto, PWM fijo -255..255");
  msg("AUTO m                   regresar al PID sin salto");
  msg("UMAX <0-255>             límite de |PWM|");
  msg("BANDA <rpm>              cambio de SP que activa REF y error que cuenta como llegada");
  msg("TMEZ <ms>                rampa de REF a PERT al llegar (0 = directo)");
  msg("TS <ms>                  tiempo de muestreo, 10-1000");
  msg("DMODE E|PV               derivada sobre error o sobre medición");
  msg("WD <ms>                  watchdog de comandos (0 = apagado)");
  msg("OUT ESP|PLOT|CSV|OFF     formato de salida");
  msg("VER m                    motor(es) en PLOT/CSV");
  msg("EST | CFG                estado | configuración");
  msg("STOP o X                 PARO");
  msg("RESET                    reinicia integral y derivada");
  msg("GUARDAR|CARGAR|FABRICA   memoria no volátil");
  msg("Ti = 0: sin integral   Td = 0: sin derivada   N = 0: derivada sin filtro");
  msg("PLOT/CSV no los reconoce connect.py: usa OUT ESP para conectarlo.");
}

// ===========================================================================
//  COMANDOS
// ===========================================================================
bool es(const char* a, const char* b) { return strcmp(a, b) == 0; }

bool aNumero(const char* s, float& out) {
  char* fin;
  out = strtof(s, &fin);
  return fin != s && *fin == '\0';
}

// Máscara de motores: "0","1","2" o "*"; 0 si es inválido
uint8_t parseMotores(const char* s) {
  if (s[0] == '*' && s[1] == '\0') return 0b111;
  if (s[0] >= '0' && s[0] <= '2' && s[1] == '\0') return 1 << (s[0] - '0');
  return 0;
}

// Máscara de juegos: "REF" (bit 0), "PERT" (bit 1) o "*"; 0 si es inválido
uint8_t parseJuegos(const char* s) {
  if (es(s, "REF"))  return 0b01;
  if (es(s, "PERT")) return 0b10;
  if (es(s, "*"))    return 0b11;
  return 0;
}

// PID J m Kp Ti Td [N]
void cmdPID(char** tok, int n) {
  uint8_t js = 0, ms = 0;
  float kp, ti, td, nf = 0.0f;
  bool ok = (n == 6 || n == 7)
         && (js = parseJuegos(tok[1])) && (ms = parseMotores(tok[2]))
         && aNumero(tok[3], kp) && aNumero(tok[4], ti) && aNumero(tok[5], td)
         && (n == 6 || aNumero(tok[6], nf));
  if (!ok) { msg("Uso: PID <REF|PERT|*> <0|1|2|*> <Kp> <Ti> <Td> [N]"); return; }
  if (kp < 0 || ti < 0 || td < 0 || nf < 0) { msg("Kp, Ti, Td y N deben ser >= 0"); return; }

  for (int i = 0; i < 3; i++) {
    if (!(ms & (1 << i))) continue;
    for (int r = 0; r < 2; r++) {
      if (!(js & (1 << r))) continue;
      Ganancias& g = cfg.g[i][r];
      g.Kp = kp; g.Ti = ti; g.Td = td;
      if (n == 7) g.N = nf;
    }
  }
  imprimirGanancias(ms, js);
}

// KP|TI|TD|N J m <valor>
void cmdParametro(const char* nombre, char** tok, int n) {
  uint8_t js = 0, ms = 0;
  float v;
  bool ok = (n == 4) && (js = parseJuegos(tok[1])) && (ms = parseMotores(tok[2])) && aNumero(tok[3], v);
  if (!ok) { msg("Uso: %s <REF|PERT|*> <0|1|2|*> <valor>", nombre); return; }
  if (v < 0) { msg("%s debe ser >= 0", nombre); return; }

  for (int i = 0; i < 3; i++) {
    if (!(ms & (1 << i))) continue;
    for (int r = 0; r < 2; r++) {
      if (!(js & (1 << r))) continue;
      Ganancias& g = cfg.g[i][r];
      if      (es(nombre, "KP")) g.Kp = v;
      else if (es(nombre, "TI")) g.Ti = v;
      else if (es(nombre, "TD")) g.Td = v;
      else                       g.N  = v;
    }
  }
  imprimirGanancias(ms, js);
}

void cmdStop() {
  for (int i = 0; i < 3; i++) {
    Motor& m = mot[i];
    m.manual  = false;
    m.uManual = 0.0f;
    m.sp      = 0.0f;
    m.u       = 0.0f;
    reiniciarEstado(m);
    setMotor(i, 0);
  }
  msg("STOP: SP = 0 en los 3 motores, modo AUTO, estados PID reiniciados");
}

void procesarLinea(char* linea) {
  // Las líneas de CFG empiezan con "# ": se aceptan pegadas tal cual
  bool comentario = false;
  while (*linea == ' ' || *linea == '\t') linea++;
  if (*linea == '#') { comentario = true; linea++; }

  for (char* p = linea; *p; ++p) *p = (char)toupper((unsigned char)*p);

  const int MAX_TOK = 8;
  char* tok[MAX_TOK];
  int n = 0;
  for (char* t = strtok(linea, " \t,"); t && n < MAX_TOK; t = strtok(NULL, " \t,")) tok[n++] = t;
  if (n == 0) return;

  lastCmdMs = millis();
  if (wdDisparado) {
    wdDisparado = false;
    msg("Watchdog rearmado");
  }

  float v;
  uint8_t ms;
  const char* c = tok[0];

  // ---- Setpoint de connect.py: "S10", "S-5" (también "S 10") ----
  if (c[0] == 'S' && n == 1 && aNumero(c + 1, v)) {
    for (int i = 0; i < 3; i++) mot[i].sp = v;
    return;   // silencioso: connect.py lo manda a 10 Hz
  }
  if (es(c, "S") && n == 2 && aNumero(tok[1], v)) {
    for (int i = 0; i < 3; i++) mot[i].sp = v;
    msg("SP * = %.2f rpm", v);
    return;
  }

  if (es(c, "SP")) {
    if (n != 3 || !(ms = parseMotores(tok[1])) || !aNumero(tok[2], v)) { msg("Uso: SP <0|1|2|*> <rpm>"); return; }
    for (int i = 0; i < 3; i++) if (ms & (1 << i)) mot[i].sp = v;
    msg("SP %s = %.2f rpm", tok[1], v);
  }
  else if (es(c, "PID")) cmdPID(tok, n);
  else if (es(c, "KP") || es(c, "TI") || es(c, "TD") || es(c, "N")) cmdParametro(c, tok, n);
  else if (es(c, "REG")) {
    Forzado f;
    if      (n == 3 && es(tok[1], "AUTO")) f = REG_AUTO;
    else if (n == 3 && es(tok[1], "REF"))  f = REG_SOLO_REF;
    else if (n == 3 && es(tok[1], "PERT")) f = REG_SOLO_PERT;
    else { msg("Uso: REG <AUTO|REF|PERT> <0|1|2|*>"); return; }
    if (!(ms = parseMotores(tok[2]))) { msg("Uso: REG <AUTO|REF|PERT> <0|1|2|*>"); return; }
    for (int i = 0; i < 3; i++) if (ms & (1 << i)) mot[i].forzado = f;
    msg("REG %s %s", tok[1], tok[2]);
  }
  else if (es(c, "MAN")) {
    if (n != 3 || !(ms = parseMotores(tok[1])) || !aNumero(tok[2], v)) { msg("Uso: MAN <0|1|2|*> <pwm -255..255>"); return; }
    v = constrain(v, (float)-PWM_MAX, (float)PWM_MAX);
    for (int i = 0; i < 3; i++) {
      if (!(ms & (1 << i))) continue;
      mot[i].manual   = true;
      mot[i].uManual  = v;
      mot[i].bumpless = false;
    }
    msg("MAN %s: PWM = %.0f (lazo abierto, limitado por UMAX = %.0f)", tok[1], v, cfg.uMax);
  }
  else if (es(c, "AUTO")) {
    if (n != 2 || !(ms = parseMotores(tok[1]))) { msg("Uso: AUTO <0|1|2|*>"); return; }
    for (int i = 0; i < 3; i++) {
      if (!(ms & (1 << i)) || !mot[i].manual) continue;
      mot[i].manual   = false;
      mot[i].bumpless = true;
    }
    msg("AUTO %s: PID activo", tok[1]);
  }
  else if (es(c, "UMAX")) {
    if (n != 2 || !aNumero(tok[1], v) || v < 0 || v > PWM_MAX) { msg("Uso: UMAX <0-255>"); return; }
    cfg.uMax = v;
    msg("UMAX %.0f", v);
  }
  else if (es(c, "BANDA")) {
    if (n != 2 || !aNumero(tok[1], v) || v < 0) { msg("Uso: BANDA <rpm >= 0>"); return; }
    cfg.banda = v;
    msg("BANDA %.2f", v);
  }
  else if (es(c, "TMEZ")) {
    if (n != 2 || !aNumero(tok[1], v) || v < 0 || v > 60000) { msg("Uso: TMEZ <0-60000 ms>"); return; }
    cfg.tMezMs = (uint16_t)v;
    msg("TMEZ %u", cfg.tMezMs);
  }
  else if (es(c, "TS")) {
    if (n != 2 || !aNumero(tok[1], v) || v < 10 || v > 1000) { msg("Uso: TS <10-1000 ms>"); return; }
    cfg.tsMs = (uint16_t)v;
    msg("TS %u", cfg.tsMs);
  }
  else if (es(c, "DMODE")) {
    if      (n == 2 && es(tok[1], "E"))  cfg.dSobrePV = 0;
    else if (n == 2 && es(tok[1], "PV")) cfg.dSobrePV = 1;
    else { msg("Uso: DMODE <E|PV>"); return; }
    msg("DMODE %s", tok[1]);
  }
  else if (es(c, "WD")) {
    if (n != 2 || !aNumero(tok[1], v) || v < 0 || v > 60000) { msg("Uso: WD <0-60000 ms> (0 = apagado)"); return; }
    cfg.wdMs = (uint16_t)v;
    msg("WD %u", cfg.wdMs);
  }
  else if (es(c, "OUT")) {
    int s = -1;
    for (int k = 0; k < 4 && n == 2; k++) if (es(tok[1], SALIDA_NOMBRE[k])) s = k;
    if (s < 0) { msg("Uso: OUT <ESP|PLOT|CSV|OFF>"); return; }
    salida = (Salida)s;
    msg("OUT %s", SALIDA_NOMBRE[s]);
    if (salida == OUT_CSV) Serial.println("t_ms,motor,reg,SP,PV,e,P,I,D,u");
  }
  else if (es(c, "VER")) {
    if (n != 2 || !(ms = parseMotores(tok[1]))) { msg("Uso: VER <0|1|2|*>"); return; }
    verMask = ms;
    imprimirVer();
  }
  else if (es(c, "EST"))                       imprimirEstado();
  else if (es(c, "CFG") || es(c, "?"))         imprimirConfig();
  else if (es(c, "STOP") || es(c, "X"))        cmdStop();
  else if (es(c, "RESET")) {
    for (int i = 0; i < 3; i++) reiniciarEstado(mot[i]);
    msg("RESET: integral y derivada en 0");
  }
  else if (es(c, "GUARDAR")) {
    if (!prefs.begin(NVS_NS, false)) { msg("Error abriendo la memoria no volátil"); return; }
    size_t w = prefs.putBytes(NVS_KEY, &cfg, sizeof(cfg));
    prefs.end();
    if (w == sizeof(cfg)) {
      cfgDesdeNVS = true;
      msg("GUARDAR: configuración guardada, se cargará al reiniciar");
    } else {
      msg("Error guardando la configuración");
    }
  }
  else if (es(c, "CARGAR")) {
    if (cargarNVS()) {
      cfgDesdeNVS = true;
      msg("CARGAR: configuración guardada restaurada");
      imprimirConfig();
    } else {
      msg("CARGAR: no hay configuración guardada");
    }
  }
  else if (es(c, "FABRICA")) {
    if (prefs.begin(NVS_NS, false)) {
      prefs.clear();
      prefs.end();
    }
    cargarDefectos();
    cfgDesdeNVS = false;
    msg("FABRICA: valores por defecto, memoria borrada");
    imprimirConfig();
  }
  else if (es(c, "AYUDA") || es(c, "H") || es(c, "HELP")) imprimirAyuda();
  else if (!comentario) msg("Comando desconocido: %s (AYUDA para ver la lista)", c);
}

void leerSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (inOverflow) msg("Línea demasiado larga, descartada");
      else if (inLen > 0) {
        inBuf[inLen] = '\0';
        procesarLinea(inBuf);
      }
      inLen = 0;
      inOverflow = false;
    } else if (inLen < sizeof(inBuf) - 1) {
      inBuf[inLen++] = c;
    } else {
      inOverflow = true;
    }
  }
}

void revisarWatchdog() {
  if (cfg.wdMs == 0 || wdDisparado) return;
  if (millis() - lastCmdMs > cfg.wdMs) {
    for (int i = 0; i < 3; i++) {
      mot[i].sp = 0.0f;
      mot[i].uManual = 0.0f;
    }
    wdDisparado = true;
    msg("WATCHDOG: %u ms sin comandos, SP = 0 (y PWM manual = 0)", cfg.wdMs);
  }
}

// ===========================================================================
//  SETUP / LOOP
// ===========================================================================
void setup() {
  Serial.setTxBufferSize(2048);   // los reportes largos no bloquean el lazo
  Serial.begin(115200);
  delay(300);

  for (int i = 0; i < 3; i++) {
    ledcAttach(IN1[i], PWM_FREQ, PWM_RES);
    ledcAttach(IN2[i], PWM_FREQ, PWM_RES);
    ledcWrite(IN1[i], 0);
    ledcWrite(IN2[i], 0);

    // Habilita el driver hasta que el PWM ya está en 0 para que el
    // motor no arranque con un pulso inesperado.
    pinMode(R_EN[i], OUTPUT);
    pinMode(L_EN[i], OUTPUT);
    digitalWrite(R_EN[i], HIGH);
    digitalWrite(L_EN[i], HIGH);

    pinMode(ENC_A[i], INPUT_PULLUP);
    pinMode(ENC_B[i], INPUT_PULLUP);
    attachInterruptArg(digitalPinToInterrupt(ENC_A[i]), isrEncoder, (void*)(intptr_t)i, CHANGE);
    attachInterruptArg(digitalPinToInterrupt(ENC_B[i]), isrEncoder, (void*)(intptr_t)i, CHANGE);
  }

  cargarDefectos();
  cfgDesdeNVS = cargarNVS();

  lastCmdMs     = millis();
  tProgramadaUs = micros();
  tMuestraUs    = tProgramadaUs;

  msg("Sintonización PID — lado %s (%s). Escribe AYUDA para ver los comandos.", LADO, ESP_ID);
  imprimirConfig();
}

void loop() {
  leerSerial();
  revisarWatchdog();

  uint32_t tsUs = (uint32_t)cfg.tsMs * 1000UL;
  uint32_t ahora = micros();
  if ((uint32_t)(ahora - tProgramadaUs) < tsUs) return;

  // Periodo fijo, sin deriva. Si el lazo se atrasó más de una muestra (un
  // reporte largo, GUARDAR) se reanuda desde ahora en vez de recuperarlas.
  tProgramadaUs += tsUs;
  if ((uint32_t)(ahora - tProgramadaUs) >= tsUs) tProgramadaUs = ahora;

  long     count[3];
  uint32_t tFlanco[3];
  noInterrupts();
  uint32_t t = micros();
  for (int i = 0; i < 3; i++) {
    count[i]   = encoderCount[i];
    tFlanco[i] = lastEdgeUs[i];
  }
  interrupts();

  uint32_t dtUs = t - tMuestraUs;   // tiempo real desde la muestra anterior
  tMuestraUs = t;
  if (dtUs == 0) return;
  float dt = dtUs * 1.0e-6f;

  for (int i = 0; i < 3; i++) {
    mot[i].cuenta  = count[i];
    mot[i].tFlanco = tFlanco[i];
    pasoControl(i, dt, dtUs);
  }
  imprimirMuestra(millis());
}
