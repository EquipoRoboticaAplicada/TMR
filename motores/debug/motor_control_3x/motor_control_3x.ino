// ============================================================
//  Control de velocidad en lazo cerrado — 3 motores 12 V
//  Plataforma : ESP32
//  Driver     : IBT-2  (IN1 = RPWM, IN2 = LPWM)
//  Encoder    : cuadratura x4 (dos canales por motor)
//  Control    : PID discrecional, muestreo cada 100 ms
//
//  Comandos por Serial (115200 baud, terminado en '\n'):
//    S<rpm>   — setpoint global, ej. "S30\n"  → 30 RPM
//    S0\n     — detener
// ============================================================

// ── Pines ───────────────────────────────────────────────────
//                  Motor 0   Motor 1   Motor 2
const int IN1[3] = { 13, 26, 25 };   // RPWM
const int IN2[3] = { 14, 27, 23 };   // LPWM
const int ENC_A[3] = {  4, 16, 18 };
const int ENC_B[3] = {  5, 17, 19 };

// ── PWM ─────────────────────────────────────────────────────
#define PWM_FREQ       20000   // Hz
#define PWM_RESOLUTION    10   // bits → máximo 1023
const int PWM_MAX = (1 << PWM_RESOLUTION) - 1;   // 1023

// ── Encoder / mecánica ──────────────────────────────────
const float GEAR_RATIO     = 270.0f;
const int   CPR_MOTOR      = 64;       // directo del datasheet, x4 ya incluido
const float CPR_OUTPUT     = CPR_MOTOR * GEAR_RATIO;  // 64 × 270 = 17,280 / CPR = Counts per revolution
const float TOP_SPEED      = 40.0f; // el motor va a 40.7rpm sin carga, falta ajustar el valor máximo con carga ej. +-25rpm

const int8_t ENC_SIGN[3]   = { +1, +1, +1 };
const int8_t MOTOR_SIGN[3] = { +1, +1, +1 };

// ── PID (mismos ganancias para los 3 motores; ajusta según tu planta) ──
float Kp = 1.0f;
float Ki = 0.8f;
float Kd = 0.0f;
const float INTEGRAL_MAX = 50.0f;   // anti-windup

// ── Muestreo ────────────────────────────────────────────────
const unsigned long SAMPLE_MS = 100;   // período de control

// ── Pausa al invertir dirección ─────────────────────────────
const unsigned long DIR_HOLD_MS = 120;

// ============================================================
//  Estructura de estado por motor
// ============================================================
struct Motor {
  float setpointRPM = 0.0f;   // con signo: + = adelante, - = atrás
  float currentRPM  = 0.0f;   // con signo: viene directo de los ticks

  float integral    = 0.0f;
  float errorPrev   = 0.0f;
  float pwmPercent  = 0.0f;   // magnitud 0–100%

  unsigned long inhibitUntilMs = 0;
  bool  prevForward = true;    // para detectar el cruce de dirección
};

Motor motor[3];

// ── Variables de encoder (accedidas desde ISR) ──────────────
volatile long   ticks[3]        = { 0, 0, 0 };
volatile int8_t lastEncState[3] = { 0, 0, 0 };

// ── Tiempo ──────────────────────────────────────────────────
unsigned long lastSampleMs = 0;

// ── Buffer de Serial ────────────────────────────────────────
String serialBuf = "";

// ============================================================
//  ISR de encoder — decodificación cuadratura x4
//  Tabla de lookup: estado_previo(2 bits) | estado_actual(2 bits) → ±1 / 0
// ============================================================
void IRAM_ATTR encoderISR(void* arg) {
  int i = (int)(intptr_t)arg;

  int8_t state = ((int8_t)digitalRead(ENC_A[i]) << 1) | (int8_t)digitalRead(ENC_B[i]);
  int8_t prev  = lastEncState[i];
  lastEncState[i] = state;

  static const int8_t lut[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
  };

  ticks[i] += lut[(prev << 2) | state];
}

// ============================================================
//  Cálculo de RPM
// ============================================================
float calcRPM(long dt_ticks, float dt_s) {
  if (dt_s <= 0.0f || CPR_OUTPUT == 0.0f) return 0.0f;
  return (dt_ticks / CPR_OUTPUT) * (60.0f / dt_s);
}

// ============================================================
//  Calcula la salida PID y devuelve pwmPercent [0–100]
// ============================================================
float computePID(Motor &m, float dt) {
  if (dt <= 0.0f) return 0.0f;

  if (fabs(m.setpointRPM) < 0.1f) {
    m.integral = 0.0f;
    m.errorPrev = 0.0f;
    return 0.0f;
  }

  float dir = (m.setpointRPM >= 0.0f) ? 1.0f : -1.0f;

  // Velocidad medida en la dirección deseada
  float measuredAlongDir = m.currentRPM * dir;

  // Error de magnitud
  float error = fabs(m.setpointRPM) - measuredAlongDir;

  float P = Kp * error;

  // Anti-windup condicional para no integrar si ya está en saturación
//  float pidRPM_preview = Kp * error + Ki * (m.integral + error * dt) + D;
//  float pwm_preview = pidRPM_preview / TOP_SPEED * 100.0f;
//  bool saturating = (pwm_preview > 100.0f && error > 0.0f) ||
//                    (pwm_preview < 0.0f  && error < 0.0f);
//  if (!saturating) {
//      m.integral += error * dt;
//      m.integral = constrain(m.integral, -INTEGRAL_MAX, INTEGRAL_MAX);
//  }

  m.integral += error * dt;
  m.integral = constrain(m.integral, -INTEGRAL_MAX, INTEGRAL_MAX);
  float I = Ki * m.integral;

  float D = Kd * (error - m.errorPrev) / dt;
  m.errorPrev = error;

  float pidRPM = P + I + D;

  float pwmMag = (pidRPM / TOP_SPEED) * 100.0f;

  pwmMag = constrain(pwmMag, 0.0f, 100.0f);

  return dir * pwmMag;
}

// ============================================================
//  Escribe PWM al driver IBT-2
// ============================================================
void writeMotor(int idx, float signedPercent) {
  signedPercent *= MOTOR_SIGN[idx];
  
  bool forward = (signedPercent >= 0.0f);
  float magPercent = fabs(signedPercent);

  ledcWrite(IN1[idx], 0);
  ledcWrite(IN2[idx], 0);

  if (magPercent < 0.1f) return;

  int pwmVal = (int)((magPercent / 100.0f) * PWM_MAX);

  if (forward) ledcWrite(IN1[idx], pwmVal);
  else         ledcWrite(IN2[idx], pwmVal);
}

// ============================================================
//  Parseo de comandos Serial
//    S<rpm>  → setpoint RPM 
// ============================================================
void handleCommand(String line) {
  line.trim(); line.toUpperCase();
  if (line.length() < 2 || line[0] != 'S') return;

  float sp = constrain(line.substring(1).toFloat(), -TOP_SPEED, TOP_SPEED);

  for (int i = 0; i < 3; i++) {
    bool newForward = (sp >= 0.0f);
    bool dirChanged = (newForward != motor[i].prevForward) && (sp != 0.0f);

    if (dirChanged) {
      // Reset del PID al cruzar dirección
      motor[i].integral    = 0.0f;
      motor[i].errorPrev   = 0.0f;
      motor[i].pwmPercent  = 0.0f;
      motor[i].inhibitUntilMs = millis() + DIR_HOLD_MS;
    }

    motor[i].setpointRPM = sp;
    motor[i].prevForward = newForward;
  }
}

void readSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBuf.length() > 0) {
        handleCommand(serialBuf);
        serialBuf = "";
      }
    } else {
      serialBuf += c;
      if (serialBuf.length() > 64) serialBuf = "";   // overflow guard
    }
  }
}

// ============================================================
//  Setup
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  for (int i = 0; i < 3; i++) {
    // Motor PWM
    ledcAttach(IN1[i], PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(IN2[i], PWM_FREQ, PWM_RESOLUTION);
    ledcWrite(IN1[i], 0);
    ledcWrite(IN2[i], 0);

    // Encoder
    pinMode(ENC_A[i], INPUT_PULLUP);
    pinMode(ENC_B[i], INPUT_PULLUP);
    lastEncState[i] = ((int8_t)digitalRead(ENC_A[i]) << 1) | (int8_t)digitalRead(ENC_B[i]);

    attachInterruptArg(digitalPinToInterrupt(ENC_A[i]), encoderISR, (void*)(intptr_t)i, CHANGE);
    attachInterruptArg(digitalPinToInterrupt(ENC_B[i]), encoderISR, (void*)(intptr_t)i, CHANGE);
  }

  lastSampleMs = millis();
}

// ============================================================
//  Loop
// ============================================================
void loop() {
  readSerial();

  unsigned long now = millis();
  if (now - lastSampleMs < SAMPLE_MS) return;

  float dt = (now - lastSampleMs) / 1000.0f;
  lastSampleMs = now;

  // Leer y reiniciar ticks de forma atómica
  long dt_ticks[3];
  portDISABLE_INTERRUPTS();
  for (int i = 0; i < 3; i++) { dt_ticks[i] = ticks[i]; ticks[i] = 0; }
  portENABLE_INTERRUPTS();

  // Control loop
  for (int i = 0; i < 3; i++) {
    motor[i].currentRPM = ENC_SIGN[i] * calcRPM(dt_ticks[i], dt);

    if (now < motor[i].inhibitUntilMs) {
      // En pausa por cambio de dirección
      motor[i].pwmPercent = 0.0f;
      writeMotor(i, motor[i].pwmPercent);
    } else {
      motor[i].pwmPercent = computePID(motor[i], dt);
      writeMotor(i, motor[i].pwmPercent);
    }
  }

  // ── Telemetría por Serial ────────────────────────────────
  //  Formato: M0 SP=XX.X PV=XX.X PWM=XX.X | M1 ... | M2 ...
  for (int i = 0; i < 3; i++) {
    Serial.print("M"); Serial.print(i);
    Serial.print(" SP=");  Serial.print(motor[i].setpointRPM, 1);
    Serial.print(" PV=");  Serial.print(motor[i].currentRPM,  1);
    Serial.print(" PWM="); Serial.print(motor[i].pwmPercent,  1);
  }
  Serial.println();
}
