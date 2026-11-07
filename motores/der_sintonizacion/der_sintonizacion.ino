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

  Dos juegos de ganancias por motor:
      A = ACELERACIÓN : hay que AUMENTAR |velocidad| en el sentido de giro
      F = FRENADO     : hay que REDUCIR |velocidad| (incluye invertir sentido)

    El régimen se decide en cada muestra. Con |e| dentro de BANDA se conserva
    el régimen anterior para no conmutar en estado estable. El término
    integral se guarda en unidades de PWM: al cambiar de juego o de ganancias
    en vivo la integral no salta; solo cambian P y D, que son chicos cerca del
    setpoint. Al pasar de MAN a AUTO la integral absorbe la diferencia, así
    que la salida continúa desde el PWM manual.

    Anti-windup: la integral se congela si empujaría la salida más allá de
    UMAX, y se limita a ±UMAX.

  ---------------------------------------------------------------------------
   COMANDOS POR SERIAL (115200, terminar con Enter, mayúsculas o minúsculas)
  ---------------------------------------------------------------------------
    m = motor 0, 1, 2 o * (los tres)      J = juego A, F o * (los dos)

    S<rpm>                    setpoint a los 3 motores (formato de connect.py)
    SP m <rpm>                setpoint a un motor
    PID J m Kp Ti Td [N]      ganancias ideales de un juego
    KP|TI|TD|N J m <valor>    un solo parámetro
    REG AUTO|A|F m            régimen automático, o forzar juego A / F
    MAN m <pwm>               lazo abierto: PWM fijo (para identificar la planta)
    AUTO m                    regresar al PID sin salto en la salida
    UMAX <0-255>              límite de |PWM| (PID y MAN)
    BANDA <rpm>               histéresis para cambiar de régimen
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
enum Regimen : uint8_t { ACEL = 0, FREN = 1 };
const char REG_CHAR[2] = {'A', 'F'};

enum Forzado : uint8_t { REG_AUTO = 0, REG_SOLO_A = 1, REG_SOLO_F = 2 };

enum Salida : uint8_t { OUT_ESP = 0, OUT_PLOT = 1, OUT_CSV = 2, OUT_OFF = 3 };
const char* SALIDA_NOMBRE[4] = {"ESP", "PLOT", "CSV", "OFF"};

struct Ganancias {
  float Kp;   // [PWM/rpm]
  float Ti;   // [s]  0 = sin integral
  float Td;   // [s]  0 = sin derivada
  float N;    // [-]  0 = derivada sin filtro
};

struct Config {
  uint32_t  version;
  Ganancias g[3][2];   // [motor][ACEL/FREN]
  float     uMax;      // límite de |u| [PWM]
  float     banda;     // histéresis del régimen [rpm]
  uint16_t  tsMs;      // tiempo de muestreo [ms]
  uint16_t  wdMs;      // watchdog de comandos [ms], 0 = apagado
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
  Regimen reg = ACEL;
  Forzado forzado = REG_AUTO;
  long    cuenta = 0;
  long    cuentaPrev = 0;
};

// ===========================================================================
//  CONFIGURACIÓN Y ESTADO
// ===========================================================================
// Defaults = ganancias de prueba de der.ino (forma paralela Kp=0.5, Ki=1.0,
// Kd=0) convertidas a forma ideal: Ti = Kp/Ki = 0.5 s, Td = Kd/Kp = 0.
const Ganancias GANANCIAS_DEFECTO = {0.5f, 0.5f, 0.0f, 10.0f};
const float     UMAX_DEFECTO      = 255.0f;
const float     BANDA_DEFECTO     = 0.5f;   // rpm (~3.5 cuentas a 100 ms)
const uint16_t  TS_DEFECTO        = 100;    // ms, igual que der.ino

const uint32_t CFG_VERSION = 0x50494401;   // cambiar si se modifica struct Config
const char*    NVS_NS      = "pid_tuning";
const char*    NVS_KEY     = "cfg";

Config      cfg;
bool        cfgDesdeNVS = false;
Preferences prefs;

Motor   mot[3];
Salida  salida  = OUT_ESP;     // ESP por defecto: compatible con connect.py
uint8_t verMask = 0b111;       // motores que se imprimen en PLOT / CSV

unsigned long lastSampleTime = 0;
unsigned long lastCmdMs      = 0;
bool          wdDisparado    = false;
unsigned int  seq            = 0;

char   inBuf[128];
size_t inLen      = 0;
bool   inOverflow = false;

// ---------- Encoder (cuadratura completa x4) — idéntico a der.ino ----------
volatile long    encoderCount[3] = {0, 0, 0};
volatile uint8_t lastEncoded[3]  = {0, 0, 0};

void IRAM_ATTR isrEncoder(void* arg) {
  int i = (int)(intptr_t)arg;

  uint8_t MSB = digitalRead(ENC_A[i]);
  uint8_t LSB = digitalRead(ENC_B[i]);
  uint8_t encoded = (MSB << 1) | LSB;
  uint8_t sum = (lastEncoded[i] << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderCount[i]++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderCount[i]--;

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
    cfg.g[i][ACEL] = GANANCIAS_DEFECTO;
    cfg.g[i][FREN] = GANANCIAS_DEFECTO;
  }
  cfg.uMax     = UMAX_DEFECTO;
  cfg.banda    = BANDA_DEFECTO;
  cfg.tsMs     = TS_DEFECTO;
  cfg.wdMs     = 0;
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
// ACEL si hay que aumentar |velocidad| en el sentido de giro, FREN si hay que
// reducirla. Dentro de la banda se conserva el régimen anterior.
Regimen elegirRegimen(const Motor& m) {
  if (m.forzado == REG_SOLO_A) return ACEL;
  if (m.forzado == REG_SOLO_F) return FREN;

  float dir;
  if (fabsf(m.pv) > cfg.banda)      dir = (m.pv > 0.0f) ? 1.0f : -1.0f;   // sentido de giro real
  else if (fabsf(m.sp) > cfg.banda) dir = (m.sp > 0.0f) ? 1.0f : -1.0f;   // casi detenido: sentido pedido
  else return m.reg;                                                       // detenido y SP ≈ 0

  float eSentido = m.e * dir;   // > 0: aumentar |vel|   < 0: reducirla
  if (eSentido >  cfg.banda) return ACEL;
  if (eSentido < -cfg.banda) return FREN;
  return m.reg;
}

void pasoControl(int i, float dt) {
  Motor& m = mot[i];

  // Velocidad en rpm del eje de salida
  long delta   = m.cuenta - m.cuentaPrev;
  m.cuentaPrev = m.cuenta;
  m.pv = ((float)delta / dt) / PULSES_PER_OUTPUT_REV * 60.0f;
  m.e  = m.sp - m.pv;

  if (!m.inicializado) {         // sin patada derivativa al arrancar / tras RESET
    m.ePrev  = m.e;
    m.pvPrev = m.pv;
    m.D      = 0.0f;
    m.inicializado = true;
  }

  m.reg = elegirRegimen(m);
  const Ganancias& g = cfg.g[i][m.reg];

  // P
  m.P = g.Kp * m.e;

  // D filtrada:  D_k = Tf/(Tf+dt)·D_k-1 + Kp·Td/(Tf+dt)·Δx,  Tf = Td/N
  float Tf = (g.N > 0.0f) ? g.Td / g.N : 0.0f;
  float dx = cfg.dSobrePV ? -(m.pv - m.pvPrev) : (m.e - m.ePrev);
  m.D = (Tf / (Tf + dt)) * m.D + (g.Kp * g.Td / (Tf + dt)) * dx;
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
    if (g.Ti > 0.0f) {
      float dI = (g.Kp / g.Ti) * m.e * dt;
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
char codigoModo(const Motor& m) {
  return m.manual ? 'M' : REG_CHAR[m.reg];
}

void imprimirMuestra(unsigned long t) {
  switch (salida) {
    case OUT_ESP: {
      // Trama de der.ino para connect.py: ESP_ID,seq,rpm0,v0,rpm1,v1,rpm2,v2
      Serial.print(ESP_ID); Serial.print(",");
      Serial.print(seq);
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
      for (int i = 0; i < 3; i++) {
        if (!(verMask & (1 << i))) continue;
        const Motor& m = mot[i];
        Serial.printf("%lu,%d,%c,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.1f\n",
                      t, i, codigoModo(m), m.sp, m.pv, m.e, m.P, m.I, m.D, m.u);
      }
      break;
    }
    case OUT_OFF:
      break;
  }
  seq++;
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
      Serial.printf("# PID %c %d %.4f %.4f %.4f %.1f\n", REG_CHAR[r], i, g.Kp, g.Ti, g.Td, g.N);
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
  msg("      J m     Kp      Ti[s]   Td[s]   N");
  imprimirGanancias(0b111, 0b11);
  msg("UMAX %.0f", cfg.uMax);
  msg("BANDA %.2f", cfg.banda);
  msg("TS %u", cfg.tsMs);
  msg("DMODE %s", cfg.dSobrePV ? "PV" : "E");
  msg("WD %u", cfg.wdMs);
  msg("OUT %s", SALIDA_NOMBRE[salida]);
  imprimirVer();
}

void imprimirEstado() {
  for (int i = 0; i < 3; i++) {
    const Motor& m = mot[i];
    const char* forz = (m.forzado == REG_AUTO) ? "auto" : "forzado";
    msg("m%d %s reg=%c(%s) SP=%.2f PV=%.2f e=%.2f P=%.1f I=%.1f D=%.1f u=%.0f enc=%ld",
        i, m.manual ? "MAN " : "AUTO", REG_CHAR[m.reg], forz,
        m.sp, m.pv, m.e, m.P, m.I, m.D, m.u, m.cuenta);
  }
  if (cfg.wdMs > 0) msg("watchdog %u ms%s", cfg.wdMs, wdDisparado ? " (DISPARADO: SP = 0)" : "");
}

void imprimirAyuda() {
  msg("---------- Comandos (m = 0,1,2,*   J = A,F,*) ----------");
  msg("S<rpm>                   setpoint a los 3 motores (connect.py), ej. S10");
  msg("SP m <rpm>               setpoint a un motor, ej. SP 0 8");
  msg("PID J m Kp Ti Td [N]     ganancias ideales, ej. PID A * 0.8 0.4 0.02 10");
  msg("KP|TI|TD|N J m <valor>   un parámetro, ej. TI F 1 0.3");
  msg("REG AUTO|A|F m           régimen automático o forzar juego A / F");
  msg("MAN m <pwm>              lazo abierto, PWM fijo -255..255");
  msg("AUTO m                   regresar al PID sin salto");
  msg("UMAX <0-255>             límite de |PWM|");
  msg("BANDA <rpm>              histéresis del cambio de régimen");
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

// Máscara de juegos: "A" (bit 0), "F" (bit 1) o "*"; 0 si es inválido
uint8_t parseJuegos(const char* s) {
  if (s[1] != '\0') return 0;
  if (s[0] == 'A') return 0b01;
  if (s[0] == 'F') return 0b10;
  if (s[0] == '*') return 0b11;
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
  if (!ok) { msg("Uso: PID <A|F|*> <0|1|2|*> <Kp> <Ti> <Td> [N]"); return; }
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
  if (!ok) { msg("Uso: %s <A|F|*> <0|1|2|*> <valor>", nombre); return; }
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
    else if (n == 3 && es(tok[1], "A"))    f = REG_SOLO_A;
    else if (n == 3 && es(tok[1], "F"))    f = REG_SOLO_F;
    else { msg("Uso: REG <AUTO|A|F> <0|1|2|*>"); return; }
    if (!(ms = parseMotores(tok[2]))) { msg("Uso: REG <AUTO|A|F> <0|1|2|*>"); return; }
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
  Serial.setTxBufferSize(1024);   // los reportes largos no bloquean el lazo
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

  lastCmdMs      = millis();
  lastSampleTime = millis();

  msg("Sintonización PID — lado %s (%s). Escribe AYUDA para ver los comandos.", LADO, ESP_ID);
  imprimirConfig();
}

void loop() {
  leerSerial();
  revisarWatchdog();

  unsigned long now = millis();
  if (now - lastSampleTime >= cfg.tsMs) {
    float dt = (now - lastSampleTime) / 1000.0f;
    lastSampleTime = now;

    long count[3];
    noInterrupts();
    count[0] = encoderCount[0];
    count[1] = encoderCount[1];
    count[2] = encoderCount[2];
    interrupts();

    for (int i = 0; i < 3; i++) {
      mot[i].cuenta = count[i];
      pasoControl(i, dt);
    }
    imprimirMuestra(now);
  }
}
