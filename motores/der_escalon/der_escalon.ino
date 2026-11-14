/*
  ===========================================================================
   PRUEBA DE ESCALÓN EN LAZO ABIERTO — LADO DERECHO
  ===========================================================================
  Para identificar el modelo de primer orden de cada motor con Control
  Station. Aquí NO hay PID: el PWM se aplica directo al IBT-2 siguiendo una
  secuencia de escalones y se imprime la velocidad medida.

  Basado en der.ino: mismos pines, encoder, datos del motor y PWM.
  Plataforma: ESP32 (Arduino framework, core 3.x) + 3 puentes IBT-2

  Conexiones (las mismas de der.ino):
    Motor   IN1   IN2   ENC_A   ENC_B   R_EN   L_EN
      0      14    13      5       4      22     21
      1      27    26     17      16      33     32
      2      23    25     19      18       2     15

  ---------------------------------------------------------------------------
   USO — Monitor Serie a 115200 baudios, SIN marca de tiempo
  ---------------------------------------------------------------------------
    0 | 1 | 2          corre la secuencia en ese motor
    *                  corre la secuencia en los 3 motores a la vez
    0 100 180 100      cambia la secuencia (niveles de PWM, -255..255) y la
                       corre en el motor 0; se conserva para las siguientes
    T 1500             duración de cada nivel en ms
    ?                  muestra la secuencia actual
    Durante la prueba, cualquier cosa que se envíe (un Enter basta) = PARO

  El primer nivel se sostiene T_PREVIO_MS antes de empezar a registrar, para
  que los datos arranquen en estado estable (Control Station lo necesita).
  Al terminar el último nivel el PWM regresa a 0.

  ---------------------------------------------------------------------------
   SALIDA — una fila cada TS_MS, columnas separadas por tabulador
  ---------------------------------------------------------------------------
    un motor:   t[s]   PWM   RPM
    con *:      t[s]   PWM   RPM0   RPM1   RPM2

    t   : tiempo desde el inicio del registro
    PWM : salida aplicada a partir de ese instante (variable manipulada)
    RPM : velocidad del eje de salida (variable de proceso)

  Las líneas que empiezan con "#" son mensajes: al pasar los datos al TXT
  copia solo las filas numéricas. La ganancia del modelo queda en rpm/PWM,
  las mismas unidades que usa el PID de der.ino.
*/

// ===========================================================================
//  LADO — lo único que cambia entre der_escalon e izq_escalon
// ===========================================================================
const char* LADO = "DERECHO";

// ---------- Pines (uno por motor) ----------
const int IN1[3] = {14, 27, 23};
const int IN2[3] = {13, 26, 25};
const int ENC_A[3] = {5, 17, 19};
const int ENC_B[3] = {4, 16, 18};

// Pines de habilitación de cada IBT-2 (salidas digitales, HIGH = activo)
const int R_EN[3] = {22, 33, 2};
const int L_EN[3] = {21, 32, 15};

// ===========================================================================
//  PRUEBA — ajusta aquí los valores por defecto
// ===========================================================================
// Secuencia de niveles de PWM. Conviene que el primero no sea 0 para no meter
// la zona muerta del motor en el modelo.
const int      NIVELES_DEFECTO[]  = {25, 50, 75, 100, 125, 150, 175, 200, 225, 250, 225, 200, 175, 150, 125, 100, 75, 50, 25};
const uint32_t T_NIVEL_MS_DEFECTO = 5000;   // duración de cada nivel
const uint32_t T_PREVIO_MS        = 1000;   // primer nivel sin registrar (asentamiento)
const uint32_t TS_MS              = 10;     // tiempo de muestreo

// ===========================================================================
//  HARDWARE — igual que der.ino
// ===========================================================================
// ---------- PWM (LEDC) ----------
const int PWM_FREQ = 20000;   // 20 kHz, fuera del rango audible
const int PWM_RES  = 8;       // 8 bits -> 0-255
const int PWM_MAX  = 255;

// ---------- Datos del motor ----------
const float GEAR_RATIO      = 262;   // relación de reducción de la caja
const int   PULSES_PER_REV  = 16;    // cuentas del encoder por vuelta del motor
const float PULSES_PER_OUTPUT_REV = PULSES_PER_REV * GEAR_RATIO;

// ---------- Encoder (cuadratura completa x4, misma lógica que der.ino) ----------
// Además de la cuenta se guarda el instante del último flanco: con él la
// velocidad se calcula sobre el tiempo real entre flancos (ver medirRPM).
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
//  ESTADO
// ===========================================================================
const int      MAX_NIVELES = 20;
const uint32_t TS_US       = TS_MS * 1000UL;

int      niveles[MAX_NIVELES];
int      nNiveles = 0;
uint32_t tNivelMs = T_NIVEL_MS_DEFECTO;

bool     corriendo = false;
uint8_t  motores   = 0;        // máscara de motores en prueba (bit i = motor i)
uint32_t muestra   = 0;        // muestras tomadas desde que arrancó la prueba
uint32_t tUltimaUs = 0;
uint32_t tInicioMs = 0;

// Medición de velocidad
float    rpm[3]         = {0, 0, 0};
long     cuentaPrev[3]  = {0, 0, 0};
uint32_t tFlancoPrev[3] = {0, 0, 0};
bool     huboFlanco[3]  = {false, false, false};
uint32_t tMuestraPrev   = 0;

char   inBuf[128];
size_t inLen = 0;

// ===========================================================================
//  MOTOR Y MEDICIÓN
// ===========================================================================
void setMotor(int idx, int pwm) {
  if (pwm >= 0) {
    ledcWrite(IN1[idx], pwm);
    ledcWrite(IN2[idx], 0);
  } else {
    ledcWrite(IN1[idx], 0);
    ledcWrite(IN2[idx], -pwm);
  }
}

// Velocidad en rpm del eje de salida. Los motores responden en decenas de
// ms, así que se muestrea rápido; a 10 ms contar pulsos por muestra daría
// escalones de ~1.4 rpm. Por eso las cuentas se dividen entre el tiempo
// real del primer al último flanco, no entre el periodo de muestreo.
void medirRPM() {
  long     cuenta[3];
  uint32_t tFlanco[3];

  noInterrupts();
  uint32_t ahora = micros();
  for (int i = 0; i < 3; i++) {
    cuenta[i]  = encoderCount[i];
    tFlanco[i] = lastEdgeUs[i];
  }
  interrupts();

  for (int i = 0; i < 3; i++) {
    long delta = cuenta[i] - cuentaPrev[i];
    cuentaPrev[i] = cuenta[i];

    if (delta == 0) {
      rpm[i] = 0.0f;
      huboFlanco[i] = false;
      continue;
    }

    // Arrancando desde reposo no hay flanco anterior de referencia: esa
    // muestra se calcula con el periodo de muestreo.
    uint32_t dtUs = huboFlanco[i] ? (tFlanco[i] - tFlancoPrev[i]) : (ahora - tMuestraPrev);
    if (dtUs == 0) dtUs = ahora - tMuestraPrev;

    rpm[i] = ((float)delta * 60.0e6f) / ((float)dtUs * PULSES_PER_OUTPUT_REV);
    tFlancoPrev[i] = tFlanco[i];
    huboFlanco[i]  = true;
  }
  tMuestraPrev = ahora;
}

// ===========================================================================
//  PRUEBA
// ===========================================================================
void imprimirSecuencia() {
  Serial.print("# PWM:");
  for (int j = 0; j < nNiveles; j++) Serial.printf(" %d", niveles[j]);
  Serial.printf("  |  %lu ms por nivel  |  Ts = %lu ms\n", (unsigned long)tNivelMs, (unsigned long)TS_MS);
}

void iniciarPrueba(uint8_t mascara) {
  motores = mascara;

  Serial.printf("# Lado %s, motor%s", LADO, mascara == 0b111 ? "es" : "");
  for (int i = 0; i < 3; i++) if (motores & (1 << i)) Serial.printf(" %d", i);
  Serial.println();
  imprimirSecuencia();
  Serial.print("# t[s]\tPWM");
  for (int i = 0; i < 3; i++) if (motores & (1 << i)) Serial.printf("\tRPM%d", i);
  Serial.println();

  noInterrupts();
  for (int i = 0; i < 3; i++) cuentaPrev[i] = encoderCount[i];
  interrupts();
  for (int i = 0; i < 3; i++) huboFlanco[i] = false;

  for (int i = 0; i < 3; i++) setMotor(i, (motores & (1 << i)) ? niveles[0] : 0);

  muestra      = 0;
  tInicioMs    = millis();
  tUltimaUs    = micros();
  tMuestraPrev = tUltimaUs;
  corriendo    = true;
}

void terminarPrueba(const char* mensaje) {
  for (int i = 0; i < 3; i++) setMotor(i, 0);
  corriendo = false;
  inLen = 0;
  Serial.println(mensaje);
}

void pasoPrueba() {
  // Cualquier byte recibido durante la prueba = PARO. Los primeros 200 ms se
  // descartan: ahí llega el salto de línea del comando que la inició.
  while (Serial.available()) {
    Serial.read();
    if (millis() - tInicioMs > 200) {
      terminarPrueba("# ABORTADO: PWM = 0");
      return;
    }
  }

  if ((uint32_t)(micros() - tUltimaUs) < TS_US) return;
  tUltimaUs += TS_US;   // periodo fijo, sin deriva acumulada
  muestra++;

  medirRPM();

  // Tiempo desde el inicio del registro; negativo durante el asentamiento
  long tMs = (long)(muestra * TS_MS) - (long)T_PREVIO_MS;
  int  idx = (tMs < 0) ? 0 : (int)(tMs / (long)tNivelMs);
  if (idx >= nNiveles) {
    terminarPrueba("# FIN");
    return;
  }

  int pwm = niveles[idx];
  for (int i = 0; i < 3; i++) if (motores & (1 << i)) setMotor(i, pwm);
  if (tMs < 0) return;

  Serial.printf("%.3f\t%d", tMs / 1000.0f, pwm);
  for (int i = 0; i < 3; i++) if (motores & (1 << i)) Serial.printf("\t%.2f", rpm[i]);
  Serial.println();
}

// ===========================================================================
//  COMANDOS
// ===========================================================================
bool aEntero(const char* s, long& out) {
  char* fin;
  out = strtol(s, &fin, 10);
  return fin != s && *fin == '\0';
}

void imprimirAyuda() {
  Serial.printf("# Prueba de escalon en lazo abierto, lado %s\n", LADO);
  Serial.println("#   0 | 1 | 2 | *      corre la secuencia en ese motor (* = los 3)");
  Serial.println("#   0 100 180 100      cambia la secuencia de PWM y la corre en el motor 0");
  Serial.println("#   T 1500             duracion de cada nivel en ms");
  Serial.println("#   durante la prueba, un Enter = PARO");
  imprimirSecuencia();
}

void procesarLinea(char* linea) {
  char* tok[MAX_NIVELES + 2];
  int n = 0;
  for (char* t = strtok(linea, " \t,"); t && n < MAX_NIVELES + 2; t = strtok(NULL, " \t,")) tok[n++] = t;
  if (n == 0) return;

  long v;
  char c = (char)toupper((unsigned char)tok[0][0]);
  bool unaLetra = (tok[0][1] == '\0');

  // T <ms>
  if (unaLetra && c == 'T') {
    if (n != 2 || !aEntero(tok[1], v) || v < (long)TS_MS || v > 60000) {
      Serial.println("# Uso: T <ms> (hasta 60000)");
      return;
    }
    tNivelMs = (uint32_t)v;
    imprimirSecuencia();
    return;
  }

  // <motor> [pwm pwm ...]
  uint8_t mascara = 0;
  if (unaLetra && c == '*')             mascara = 0b111;
  if (unaLetra && c >= '0' && c <= '2') mascara = 1 << (c - '0');
  if (mascara == 0) {
    imprimirAyuda();
    return;
  }

  if (n > 1) {
    if (n - 1 > MAX_NIVELES) {
      Serial.printf("# Maximo %d niveles\n", MAX_NIVELES);
      return;
    }
    int nuevos[MAX_NIVELES];
    for (int j = 1; j < n; j++) {
      if (!aEntero(tok[j], v) || v < -PWM_MAX || v > PWM_MAX) {
        Serial.printf("# Nivel invalido: %s (PWM entre -%d y %d)\n", tok[j], PWM_MAX, PWM_MAX);
        return;
      }
      nuevos[j - 1] = (int)v;
    }
    nNiveles = n - 1;
    for (int j = 0; j < nNiveles; j++) niveles[j] = nuevos[j];
  }

  iniciarPrueba(mascara);
}

void leerComandos() {
  while (Serial.available() && !corriendo) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (inLen > 0) {
        inBuf[inLen] = '\0';
        inLen = 0;
        procesarLinea(inBuf);
      }
    } else if (inLen < sizeof(inBuf) - 1) {
      inBuf[inLen++] = c;
    }
  }
}

// ===========================================================================
//  SETUP / LOOP
// ===========================================================================
void setup() {
  Serial.setTxBufferSize(2048);   // imprimir no frena el muestreo
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

  nNiveles = sizeof(NIVELES_DEFECTO) / sizeof(NIVELES_DEFECTO[0]);
  if (nNiveles > MAX_NIVELES) nNiveles = MAX_NIVELES;
  for (int j = 0; j < nNiveles; j++) niveles[j] = constrain(NIVELES_DEFECTO[j], -PWM_MAX, PWM_MAX);

  imprimirAyuda();
}

void loop() {
  if (corriendo) pasoPrueba();
  else           leerComandos();
}
