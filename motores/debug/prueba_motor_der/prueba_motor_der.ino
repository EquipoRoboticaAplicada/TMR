void IRAM_ATTR encoderISR(void* arg);
const char* ESP_ID = "ESP_R";

// =====================================================
// CONFIG
// =====================================================

// Para IBT-2: IN1 = RPWM, IN2 = LPWM
// Orden: 0=adelante, 1=en medio, 2=atrás
const int IN1[3] = {14, 27, 23};
const int IN2[3] = {13, 26, 25};

const int ENC_A[3] = {4, 16, 18};
const int ENC_B[3] = {5, 17, 19};

#define PWM_FREQ       20000
#define PWM_RESOLUTION 10
const int PWM_MAX = (1 << PWM_RESOLUTION) - 1;

const float PWM_MIN        = 20.0f;
const float GEAR_RATIO     = 270.0f;
const int   PULSES_PER_REV = 16;
const int   CPR_OUTPUT     = (int)(PULSES_PER_REV * 4 * GEAR_RATIO); // 3600

const unsigned long SAMPLE_MS = 100;
const unsigned long DIR_CHANGE_HOLD_MS = 120;

// ================= MODO TIMEOUT =================
// true  -> modo normal, timeout activo
// false -> modo debug, timeout desactivado
const bool ENABLE_CMD_TIMEOUT = true;
const unsigned long CMD_TIMEOUT_MS = 3000;

// true  -> imprime para Serial Plotter
// false -> imprime CSV para integración con Jetson/RPi
const bool PLOTTER_MODE = false;

// Motor principal para graficar SP/PV/ERR/PWM
const int PLOT_MOTOR_IDX = 0;

const float WHEEL_DIAM_M = 0.17f;
const float WHEEL_CIRC_M = 3.14159265f * WHEEL_DIAM_M;

// PID
float Kp[3] = {0.0f, 0.0f, 0.0f}; 
float Ki[3] = {1.0f, 1.0f, 1.0f};
float Kd[3] = {0.0f, 0.0f, 0.0f};

const float INTEGRAL_MAX = 200.0f;

unsigned long lastCmdMs = 0;
unsigned int seq = 0;

// =====================================================
// ESTADO
// =====================================================
struct PIDState {
  float setpointRPM = 0.0f;      // magnitud pedida (siempre >= 0)
  float desiredRPM  = 0.0f;      // referencia del lazo (magnitud)
  float currentRPM  = 0.0f;      // velocidad firmada medida (+ adelante, - atrás)
  float controlRPM  = 0.0f;      // velocidad alineada al sentido comandado
  float error       = 0.0f;
  float errorSum    = 0.0f;
  float errorPrev   = 0.0f;
  float pidOutput   = 0.0f;
  float pwmPercent  = 0.0f;
  bool  direction   = true;      // true=D1=adelante, false=D0=atrás
  unsigned long inhibitUntilMs = 0;
};

PIDState motor[3];

volatile long   ticks[3] = {0, 0, 0};
volatile int8_t lastEncState[3] = {0, 0, 0};

unsigned long lastSampleTime = 0;
String inputBuffer = "";

// =====================================================
// ISR ENCODER X4
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

// Para el lado derecho, el signo natural del encoder estaba invertido
// respecto a la convención global.
// Resultado final deseado:
//   adelante físico  -> RPM positiva
//   atrás físico     -> RPM negativa
float calcularRPMFirmada(long dticks, float dt) {
  if (dt <= 0.0f || CPR_OUTPUT == 0) return 0.0f;
  float rpmNatural = (dticks / (float)CPR_OUTPUT) * (60.0f / dt);
  return -rpmNatural;
}

float calcularVelocidadMPSDesdeRPM(float rpmSigned) {
  return WHEEL_CIRC_M * rpmSigned / 60.0f;
}

float computePID(PIDState &m, float dt, float Kp, float Ki, float Kd, float integralMax) {
  if (dt <= 0.0f) return m.pidOutput;

  // El setpoint del lazo se mantiene como magnitud positiva
  m.desiredRPM = m.setpointRPM;

  // Alinear la medición al sentido comandado:
  // D1 -> controlRPM = currentRPM
  // D0 -> controlRPM = -currentRPM
  // Así el lazo siempre trabaja con magnitudes positivas.
  m.controlRPM = fabsf(m.currentRPM);

  // Error del lazo
  m.error = m.desiredRPM - m.controlRPM;

  float P = Kp * m.error;
  
  // ── calcular D primero para tenerlo disponible ──────────────
  float errorDiff = (m.error - m.errorPrev) / dt;
  float D = Kd * errorDiff;
  m.errorPrev = m.error;
  
  // ── I con anti-windup (ahora sí tiene D disponible) ─────────
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

void setMotorPins(int in1Pin, int in2Pin, float percent, bool forward) {
  percent = constrain(percent, 0.0f, 100.0f);

  int pwmValue = 0;

  if (percent >= 0.1f) {
    float percentReal = map((long)(percent * 10), 0, 1000,
                            (long)(PWM_MIN * 10), 1000) / 10.0f;
    pwmValue = (int)((percentReal / 100.0f) * PWM_MAX);
    pwmValue = constrain(pwmValue, 0, PWM_MAX);
  }

  ledcWrite(in1Pin, 0);
  ledcWrite(in2Pin, 0);

  if (pwmValue == 0) return;

  if (forward) {
    ledcWrite(in1Pin, pwmValue);
  } else {
    ledcWrite(in2Pin, pwmValue);
  }
}

void stopAllMotors() {
  for (int i = 0; i < 3; i++) {
    motor[i].setpointRPM = 0.0f;
    motor[i].desiredRPM  = 0.0f;
    motor[i].controlRPM  = 0.0f;
    motor[i].error       = 0.0f;
    motor[i].errorSum    = 0.0f;
    motor[i].errorPrev   = 0.0f;
    motor[i].pidOutput   = 0.0f;
    motor[i].pwmPercent  = 0.0f;
    ledcWrite(IN1[i], 0);
    ledcWrite(IN2[i], 0);
  }
}

void imprimirPlotter() {
  int i = PLOT_MOTOR_IDX;

  // SP firmado para que el plotter refleje la convención física real
  float spSigned = motor[i].direction ? motor[i].setpointRPM : -motor[i].setpointRPM;

  Serial.print("SP:");
  Serial.print(spSigned, 2);

  Serial.print("\tPV:");
  Serial.print(motor[i].currentRPM, 2);

  Serial.print("\tERR:");
  Serial.print(motor[i].error, 2);

  Serial.print("\tPWM:");
  Serial.print(motor[i].pwmPercent, 2);

  Serial.print("\tRPM0:");
  Serial.print(motor[0].currentRPM, 2);

  Serial.print("\tRPM1:");
  Serial.print(motor[1].currentRPM, 2);

  Serial.print("\tRPM2:");
  Serial.println(motor[2].currentRPM, 2);
}

void imprimirCSV(float v_mps[3]) {
  Serial.print(ESP_ID);
  Serial.print(",");
  Serial.print(seq++);

  for (int i = 0; i < 2; i++) { // dos motores funcionando, los dos de enfrente 
    Serial.print(",");
    Serial.print(motor[i].currentRPM, 2);
    Serial.print(",");
    Serial.print(v_mps[i], 4);
  }
  Serial.println();
}

// =====================================================
// COMANDOS SERIAL
// Mantiene formato tipo S30, D1
// =====================================================
void handleLine(String line) {
  line.trim();
  line.toUpperCase();
  if (line.length() < 2) return;

  if (line[0] == 'D') {
    bool dir = (line.substring(1).toInt() == 1);
    unsigned long tNow = millis();

    for (int i = 0; i < 3; i++) {
      if (motor[i].direction != dir) {
        motor[i].direction = dir;

        motor[i].desiredRPM  = 0.0f;
        motor[i].controlRPM  = 0.0f;
        motor[i].error       = 0.0f;
        motor[i].errorSum    = 0.0f;
        motor[i].errorPrev   = 0.0f;
        motor[i].pidOutput   = 0.0f;
        motor[i].pwmPercent  = 0.0f;

        motor[i].inhibitUntilMs = tNow + DIR_CHANGE_HOLD_MS;
      }
    }

    lastCmdMs = tNow;
    return;
  }

  if (line[0] == 'S') {
      float sp = constrain(line.substring(1).toFloat(), 0.0f, 67.0f);
      for (int i = 0; i < 3; i++) {
          motor[i].setpointRPM = sp;
          if (sp == 0.0f) {
              motor[i].errorSum  = 0.0f;
              motor[i].errorPrev = 0.0f;
              motor[i].pidOutput = 0.0f;
          }
      }
      lastCmdMs = millis();
  }

  if (line == "STOP") {
    stopAllMotors();
    lastCmdMs = millis();
    return;
  }

      // ---- Comandos PID ----
  // Formato:  KP<i>:<val>  |  KI<i>:<val>  |  KD<i>:<val>
  //   i = 0, 1, 2  → motor específico
  //   i = A        → aplica a los 3 motores
  // Ejemplos:  KP0:1.5   KIA:0.1   KD2:0.05
  // PID?  → imprime valores actuales

  if (line.length() >= 5 &&
      (line.startsWith("KP") || line.startsWith("KI") || line.startsWith("KD"))) {

    char gain   = line[1];          // 'P', 'I' o 'D'
    char target = line[2];          // '0', '1', '2' o 'A'
    int  colonIdx = line.indexOf(':');

    if (colonIdx != 3) {
      return;
    }

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

    return;
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

    motor[i].direction = true;
    motor[i].setpointRPM = 0.0f;
  }

  lastCmdMs = millis();
  lastSampleTime = millis();
}

// =====================================================
// LOOP
// =====================================================
void loop() {
  readSerialLines();

  if (ENABLE_CMD_TIMEOUT && (millis() - lastCmdMs > CMD_TIMEOUT_MS)) {
    for (int i = 0; i < 3; i++) {
      motor[i].setpointRPM = 0.0f;
      motor[i].desiredRPM  = 0.0f;
      motor[i].controlRPM  = 0.0f;
      motor[i].errorSum    = 0.0f;
    }
  }

  unsigned long now = millis();
  if (now - lastSampleTime >= SAMPLE_MS) {
    uint32_t dt_ms = now - lastSampleTime;
    float dt = fabs(dt_ms) / 1000.0f;

    long dticks[3];

    portDISABLE_INTERRUPTS();
    dticks[0] = ticks[0]; ticks[0] = 0;
    dticks[1] = ticks[1]; ticks[1] = 0;
    dticks[2] = ticks[2]; ticks[2] = 0;
    portENABLE_INTERRUPTS();

    float v_mps[3];

    for (int i = 0; i < 3; i++) {
      motor[i].currentRPM = calcularRPMFirmada(dticks[i], dt);

      if (now < motor[i].inhibitUntilMs) 
        motor[i].pwmPercent = 0.0f;
      else 
        motor[i].pwmPercent = computePID(motor[i], dt, Kp[i], Ki[i], Kd[i], INTEGRAL_MAX);

      if (i == 2)
        setMotorPins(IN1[2], IN2[2], motor[1].pwmPercent, motor[1].direction); // Encoder de motor 2 no funcional
      else 
        setMotorPins(IN1[i], IN2[i], motor[i].pwmPercent, motor[i].direction);
        
      v_mps[i] = calcularVelocidadMPSDesdeRPM(motor[i].currentRPM);
    }

    if (PLOTTER_MODE) {
      imprimirPlotter();
    } else {
      imprimirCSV(v_mps);
    }

    lastSampleTime = now;
  }
}
