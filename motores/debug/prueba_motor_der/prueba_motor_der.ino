void IRAM_ATTR encoderISR(void* arg);
const char* ESP_ID = "ESP_R";

// =====================================================
// CONFIG
// =====================================================

// Para IBT-2: IN1 = RPWM, IN2 = LPWM
// Orden: 0=adelante, 1=en medio, 2=atrás
const int IN1[3] = {13, 26, 23};
const int IN2[3] = {14, 27, 25};

const int ENC_A[3] = {4, 16, 19};
const int ENC_B[3] = {5, 17, 18};

#define PWM_FREQ       20000
#define PWM_RESOLUTION 10
const int PWM_MAX = (1 << PWM_RESOLUTION) - 1;

const float PWM_MIN        = 20.0f;
const float GEAR_RATIO     = 270.0f;
const int   PULSES_PER_REV = 16;
const int   CPR_OUTPUT     = (int)(PULSES_PER_REV * 4 * GEAR_RATIO); // 17280

const unsigned long SAMPLE_MS = 100;

// ─── PARÁMETROS DE PROTECCIÓN IBT-2 ─────────────────
// Tasa máxima de cambio del setpoint (RPM/segundo).
// Con SAMPLE_MS=100 ms: delta máximo = 6 RPM por ciclo.
// Tiempo de 0 → 67 RPM: ~1.1 s.  Ajustar según inercia.
const float RAMP_RATE_RPM_PER_S = 60.0f;

// Tiempo de espera DESPUÉS de confirmar que el motor
// está detenido, antes de aplicar la nueva dirección.
// 500 ms permite que cese cualquier corriente residual.
const unsigned long DIR_CHANGE_HOLD_MS = 500;

// Umbral de RPM para considerar el motor "detenido"
// antes de ejecutar el cambio de dirección.
const float STOP_THRESHOLD_RPM = 3.0f;
// ─────────────────────────────────────────────────────

const bool ENABLE_CMD_TIMEOUT = true;
const unsigned long CMD_TIMEOUT_MS = 3000;

const bool PLOTTER_MODE = false;
const int PLOT_MOTOR_IDX = 0;

const float WHEEL_DIAM_M = 0.17f;
const float WHEEL_CIRC_M = 3.14159265f * WHEEL_DIAM_M;

// PID
float Kp[3] = {1.0f, 1.0f, 1.0f};
float Ki[3] = {1.4f, 1.4f, 1.4f};
float Kd[3] = {0.0f, 0.0f, 0.0f};

const float INTEGRAL_MAX = 200.0f;

unsigned long lastCmdMs = 0;
unsigned int seq = 0;

// =====================================================
// ESTADO
// =====================================================
struct PIDState {
  // ── Setpoints ──
  float setpointRPM = 0.0f;   // magnitud pedida externamente (objetivo final)
  float rampedRPM   = 0.0f;   // setpoint real con rampa aplicada (alimenta al PID)

  // ── Variables de lazo ──
  float desiredRPM  = 0.0f;
  float currentRPM  = 0.0f;
  float controlRPM  = 0.0f;
  float error       = 0.0f;
  float errorSum    = 0.0f;
  float errorPrev   = 0.0f;
  float pidOutput   = 0.0f;
  float pwmPercent  = 0.0f;

  // ── Dirección ──
  bool direction    = true;   // true=adelante (RPWM), false=atrás (LPWM)

  // ── Máquina de estados para cambio de dirección seguro ──
  bool  awaitingDirChange  = false; // hay un cambio pendiente de dirección
  bool  pendingDir         = true;  // dirección deseada cuando cambie
  unsigned long inhibitUntilMs = 0; // no aplicar PWM hasta este timestamp
};

PIDState motor[3];

volatile long   ticks[3] = {0, 0, 0};
volatile int8_t lastEncState[3] = {0, 0, 0};

unsigned long lastSampleTime = 0;
String inputBuffer = "";

// =====================================================
// ISR ENCODER X4 (sin cambios)
// =====================================================
void IRAM_ATTR encoderISR(void* arg) {
  int i = (int)(intptr_t)arg;

  bool A = digitalRead(ENC_A[i]);
  bool B = digitalRead(ENC_B[i]);

  int8_t state = ((int8_t)A << 1) | (int8_t)B;
  int8_t prev  = lastEncState[i];
  lastEncState[i] = state;

  static const int8_t lookup[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
  };

  ticks[i] += lookup[(prev << 2) | state];
}

// =====================================================
// UTILIDADES
// =====================================================

float calcularRPMFirmada(long dticks, float dt) {
  if (dt <= 0.0f || CPR_OUTPUT == 0) return 0.0f;
  float rpmNatural = (dticks / (float)CPR_OUTPUT) * (60.0f / dt);
  return -rpmNatural; // inversión de signo para el lado derecho
}

float calcularVelocidadMPSDesdeRPM(float rpmSigned) {
  return WHEEL_CIRC_M * rpmSigned / 60.0f;
}

// Resetea solo las variables del lazo PID sin tocar
// setpointRPM ni rampedRPM ni dirección.
void resetPIDState(PIDState &m) {
  m.desiredRPM  = 0.0f;
  m.controlRPM  = 0.0f;
  m.error       = 0.0f;
  m.errorSum    = 0.0f;
  m.errorPrev   = 0.0f;
  m.pidOutput   = 0.0f;
  m.pwmPercent  = 0.0f;
}

// ─── 1. RAMPA DE SETPOINT ────────────────────────────
// Mueve rampedRPM hacia setpointRPM a máximo RAMP_RATE_RPM_PER_S.
// Llama una vez por ciclo de muestreo antes de computePID.
void applyRamp(PIDState &m, float dt) {
  float maxDelta = RAMP_RATE_RPM_PER_S * dt;

  if (m.rampedRPM < m.setpointRPM) {
    m.rampedRPM = min(m.rampedRPM + maxDelta, m.setpointRPM);
  } else if (m.rampedRPM > m.setpointRPM) {
    m.rampedRPM = max(m.rampedRPM - maxDelta, m.setpointRPM);
  }
}

// ─── 2. GESTIÓN DE CAMBIO DE DIRECCIÓN SEGURO ────────
// Retorna true si el motor debe mantener PWM=0 (en frenado o inhibición).
bool handleDirChange(PIDState &m, unsigned long now) {
  if (!m.awaitingDirChange) return false;

  // Paso A: esperar a que la velocidad real Y la rampa lleguen a ~0
  if (fabsf(m.currentRPM) > STOP_THRESHOLD_RPM || m.rampedRPM > STOP_THRESHOLD_RPM) {
    // Asegurarse de que el setpoint siga en 0 (podría haber llegado
    // un comando S durante el frenado)
    m.setpointRPM = 0.0f;
    return true;  // seguir frenando
  }

  // Paso B: motor detenido — aplicar nueva dirección y arrancar inhibición
  m.direction          = m.pendingDir;
  m.awaitingDirChange  = false;
  m.rampedRPM          = 0.0f;  // garantizar que parte desde 0
  m.inhibitUntilMs     = now + DIR_CHANGE_HOLD_MS;
  resetPIDState(m);

  return true;  // continuar en inhibición hasta que expire el timer
}

// ─── 3. PID ──────────────────────────────────────────
float computePID(PIDState &m, float dt, float Kp, float Ki, float Kd, float integralMax) {
  if (dt <= 0.0f) return m.pidOutput;

  // El lazo usa rampedRPM, no setpointRPM directamente.
  m.desiredRPM = m.rampedRPM;
  m.controlRPM = fabsf(m.currentRPM);
  m.error      = m.desiredRPM - m.controlRPM;

  float P = Kp * m.error;

  float errorDiff = (m.error - m.errorPrev) / dt;
  float D = Kd * errorDiff;
  m.errorPrev = m.error;

  // Anti-windup condicional
  float I_prev    = Ki * m.errorSum;
  float tentative = P + I_prev + D;

  bool sat_high = (tentative >= 100.0f) && (m.error > 0.0f);
  bool sat_low  = (tentative <=   0.0f) && (m.error < 0.0f);

  if (!sat_high && !sat_low) {
    m.errorSum += m.error * dt;
    m.errorSum  = constrain(m.errorSum, -integralMax, integralMax);
  }
  float I = Ki * m.errorSum;

  float u = P + I + D;
  if (u < 0.0f) u = 0.0f;
  m.pidOutput = constrain(u, 0.0f, 100.0f);

  return m.pidOutput;
}

// ─── 4. APLICAR PWM AL DRIVER ────────────────────────
// El IBT-2/BTS7960B nunca debe tener RPWM y LPWM altos
// simultáneamente. Se zerean ambos antes de escribir
// la dirección activa — garantiza ausencia de shoot-through.
void setMotorPins(int in1Pin, int in2Pin, float percent, bool forward) {
  percent = constrain(percent, 0.0f, 100.0f);
  int pwmValue = 0;

  if (percent >= 0.1f) {
    float percentReal = map((long)(percent * 10), 0, 1000,
                            (long)(PWM_MIN * 10), 1000) / 10.0f;
    pwmValue = (int)((percentReal / 100.0f) * PWM_MAX);
    pwmValue = constrain(pwmValue, 0, PWM_MAX);
  }

  // Primero apagar ambas salidas
  ledcWrite(in1Pin, 0);
  ledcWrite(in2Pin, 0);

  if (pwmValue == 0) return;

  if (forward) {
    ledcWrite(in1Pin, pwmValue);  // RPWM
  } else {
    ledcWrite(in2Pin, pwmValue);  // LPWM
  }
}

// ─── 5. STOP SUAVE ───────────────────────────────────
// Baja setpointRPM a 0 en todos los motores.
// La rampa (applyRamp) se encarga de frenar gradualmente.
// No se corta el PWM de golpe.
void stopAllMotors() {
  for (int i = 0; i < 3; i++) {
    motor[i].setpointRPM    = 0.0f;
    motor[i].awaitingDirChange = false; // cancelar cambio pendiente
  }
}

// =====================================================
// IMPRESIÓN
// =====================================================
void imprimirPlotter() {
  int i = PLOT_MOTOR_IDX;

  // Mostrar la rampa real como SP para ver la curva de aceleración
  float spSigned = motor[i].direction ? motor[i].rampedRPM : -motor[i].rampedRPM;

  Serial.print("SP:");    Serial.print(spSigned, 2);
  Serial.print("\tPV:");  Serial.print(motor[i].currentRPM, 2);
  Serial.print("\tERR:"); Serial.print(motor[i].error, 2);
  Serial.print("\tPWM:"); Serial.print(motor[i].pwmPercent, 2);
  Serial.print("\tRPM0:"); Serial.print(motor[0].currentRPM, 2);
  Serial.print("\tRPM1:"); Serial.print(motor[1].currentRPM, 2);
  Serial.print("\tRPM2:"); Serial.println(motor[2].currentRPM, 2);
}

void imprimirCSV(float v_mps[3]) {
  Serial.print(ESP_ID);
  Serial.print(",");
  Serial.print(seq++);

  for (int i = 0; i < 2; i++) {
    Serial.print(","); Serial.print(motor[i].currentRPM, 2);
    Serial.print(","); Serial.print(v_mps[i], 4);
  }
  Serial.println();
}

// =====================================================
// COMANDOS SERIAL
// =====================================================
void handleLine(String line) {
  line.trim();
  line.toUpperCase();
  if (line.length() < 2) return;

  // ── STOP (freno suave, no corte brusco) ──
  if (line == "STOP") {
    stopAllMotors();
    lastCmdMs = millis();
    return;
  }

  // ── Dirección: inicia máquina de estados de cambio seguro ──
  if (line[0] == 'D') {
    bool dir = (line.substring(1).toInt() == 0);
    unsigned long tNow = millis();

    for (int i = 0; i < 3; i++) {
      if (motor[i].direction != dir) {
        if (!motor[i].awaitingDirChange) {
          // Iniciar secuencia de cambio: frena primero
          motor[i].setpointRPM     = 0.0f;
          motor[i].pendingDir      = dir;
          motor[i].awaitingDirChange = true;
        } else {
          // Ya hay un cambio en curso: actualizar la dir destino
          motor[i].pendingDir = dir;
        }
      }
      // Si la dirección ya es la misma, no hacer nada
    }
    lastCmdMs = tNow;
    return;
  }

  // ── Velocidad ──
  if (line[0] == 'S') {
    float sp = constrain(line.substring(1).toFloat(), 0.0f, 67.0f);
    for (int i = 0; i < 3; i++) {
      // Si hay un cambio de dirección en curso, no aplicar
      // el nuevo SP hasta que se complete (la rampa y el
      // handleDirChange lo harán en el loop).
      if (!motor[i].awaitingDirChange) {
        motor[i].setpointRPM = sp;
      }
      // Si sp==0, limpiar integrador de todas formas
      if (sp == 0.0f) {
        motor[i].errorSum  = 0.0f;
        motor[i].errorPrev = 0.0f;
        motor[i].pidOutput = 0.0f;
      }
    }
    lastCmdMs = millis();
    return;
  }

  // ── Ganancias PID ──
  // Formato: KP<i>:<val>  |  KI<i>:<val>  |  KD<i>:<val>
  //   i = 0,1,2 → motor específico   |   A → los 3
  if (line.length() >= 5 &&
      (line.startsWith("KP") || line.startsWith("KI") || line.startsWith("KD"))) {

    char gain    = line[1];
    char target  = line[2];
    int  colonIdx = line.indexOf(':');
    if (colonIdx != 3) return;

    float val = line.substring(colonIdx + 1).toFloat();

    int mStart = 0, mEnd = 3;
    if (target >= '0' && target <= '2') {
      mStart = target - '0';
      mEnd   = mStart + 1;
    } else if (target != 'A') {
      return;
    }

    for (int i = mStart; i < mEnd; i++) {
      if      (gain == 'P') Kp[i] = val;
      else if (gain == 'I') Ki[i] = val;
      else if (gain == 'D') Kd[i] = val;
      // Reiniciar integrador al cambiar ganancias
      motor[i].errorSum  = 0.0f;
      motor[i].errorPrev = 0.0f;
    }
  }
}

void readSerialLines() {
  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        handleLine(inputBuffer);
        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
      if (inputBuffer.length() > 64) inputBuffer = "";
    }
  }
}

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(300);

  for (int i = 0; i < 3; i++) {
    ledcAttach(IN1[i], PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(IN2[i], PWM_FREQ, PWM_RESOLUTION);
    ledcWrite(IN1[i], 0);
    ledcWrite(IN2[i], 0);

    pinMode(ENC_A[i], INPUT_PULLUP);
    pinMode(ENC_B[i], INPUT_PULLUP);

    bool A = digitalRead(ENC_A[i]);
    bool B = digitalRead(ENC_B[i]);
    lastEncState[i] = ((int8_t)A << 1) | (int8_t)B;

    attachInterruptArg(digitalPinToInterrupt(ENC_A[i]), encoderISR, (void*)(intptr_t)i, CHANGE);
    attachInterruptArg(digitalPinToInterrupt(ENC_B[i]), encoderISR, (void*)(intptr_t)i, CHANGE);

    motor[i].direction   = true;
    motor[i].setpointRPM = 0.0f;
    motor[i].rampedRPM   = 0.0f;
  }

  lastCmdMs      = millis();
  lastSampleTime = millis();
}

// =====================================================
// LOOP
// =====================================================
void loop() {
  readSerialLines();

  // Timeout de comando: baja setpoint gradualmente (no corte brusco)
  if (ENABLE_CMD_TIMEOUT && (millis() - lastCmdMs > CMD_TIMEOUT_MS)) {
    for (int i = 0; i < 3; i++) {
      motor[i].setpointRPM = 0.0f;
      // La rampa frenará suavemente; no se toca rampedRPM aquí.
    }
  }

  unsigned long now = millis();
  if (now - lastSampleTime >= SAMPLE_MS) {
    uint32_t dt_ms = now - lastSampleTime;
    float    dt    = (float)dt_ms / 1000.0f;

    long dticks[3];

    portDISABLE_INTERRUPTS();
    dticks[0] = ticks[0]; ticks[0] = 0;
    dticks[1] = ticks[1]; ticks[1] = 0;
    dticks[2] = ticks[2]; ticks[2] = 0;
    portENABLE_INTERRUPTS();

    float v_mps[3];

    for (int i = 0; i < 3; i++) {
      // 1. Medir velocidad real
      motor[i].currentRPM = calcularRPMFirmada(dticks[i], dt);

      // 2. Aplicar rampa de setpoint (aceleración/frenado gradual)
      applyRamp(motor[i], dt);

      // 3. Gestionar cambio de dirección pendiente
      //    Retorna true si el motor debe mantener PWM=0.
      bool inDirChange = handleDirChange(motor[i], now);

      // 4. Verificar inhibición post-cambio
      bool inhibited = inDirChange || (now < motor[i].inhibitUntilMs);

      if (inhibited) {
        // Congelar y resetear el integrador durante toda la fase
        // de frenado e inhibición para evitar kick al retomar.
        motor[i].errorSum   = 0.0f;
        motor[i].errorPrev  = 0.0f;
        motor[i].pwmPercent = 0.0f;
      } else {
        motor[i].pwmPercent = computePID(motor[i], dt,
                                         Kp[i], Ki[i], Kd[i],
                                         INTEGRAL_MAX);
      }

      // Motor 2 usa el PID del motor 1 (encoder no funcional en motor 2)
      if (i == 2)
        setMotorPins(IN1[2], IN2[2], motor[1].pwmPercent, motor[1].direction);
      else
        setMotorPins(IN1[i], IN2[i], motor[i].pwmPercent, motor[i].direction);

      v_mps[i] = calcularVelocidadMPSDesdeRPM(motor[i].currentRPM);
    }

    if (PLOTTER_MODE) imprimirPlotter();
    else              imprimirCSV(v_mps);

    lastSampleTime = now;
  }
}
