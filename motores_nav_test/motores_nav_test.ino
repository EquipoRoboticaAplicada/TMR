const char* ESP_ID = "ESP_2WD";

// Definición de pines para 2 motores (Izquierdo = 0, Derecho = 1)
// Ajusta estos pines a tu cableado real con los drivers (ej. IBT-2 o L298N)
const int IN1[2] = {27, 25}; // RPWM o Adelante
const int IN2[2] = {14, 26}; // LPWM o Atrás

// Pines de los encoders (usando pines sin conflicto del código original)
const int ENC_A[2] = {32, 34};
const int ENC_B[2] = {33, 35};

#define PWM_FREQ       20000
#define PWM_RESOLUTION 10
const int PWM_MAX = (1 << PWM_RESOLUTION) - 1;
const float PWM_MIN         = 20.0;
const float GEAR_RATIO      = 56.25;
const int   PULSES_PER_REV  = 16;
const int   CPR_OUTPUT      = PULSES_PER_REV * 4 * GEAR_RATIO;

const unsigned long SAMPLE_MS = 100;

const float WHEEL_DIAM_M = 0.062f;
const float WHEEL_CIRC_M = 3.14159265f * WHEEL_DIAM_M;
uint32_t seq = 0;

// Ganancias PID (Individuales por si un motor responde distinto)
float Kp[2] = {0.0, 0.0};
float Ki[2] = {1.0, 1.0};
float Kd[2] = {0.0, 0.0};
const float INTEGRAL_MAX = 200.0;

// Failsafe
const unsigned long CMD_TIMEOUT_MS = 400;
unsigned long lastCmdMs = 0;

struct PIDState {
  float setpointRPM = 0.0;
  float currentRPM  = 0.0;
  float error       = 0.0;
  float errorSum    = 0.0;
  float errorPrev   = 0.0;
  float pidOutput   = 0.0;
  float pwmPercent  = 0.0;
  bool  direction   = true;
};

PIDState motor[2];
volatile long ticks[2] = {0, 0};
long last_ticks[2]     = {0, 0}; // Para calcular el delta sin perder el absoluto
volatile int8_t lastEncState[2] = {0, 0};
unsigned long lastSampleTime = 0;
String inputBuffer = "";

// ---------------- ISR Encoder x4 (lookup table) ----------------
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

// ---------------- Utilidades ----------------
float calcularRPM(long dticks, float dt) {
  if (dt <= 0.0f || CPR_OUTPUT == 0) return 0.0f;
  return (dticks / (float)CPR_OUTPUT) * (60.0f / dt);
}

float computePID(PIDState &m, float dt, float Kp, float Ki, float Kd, float integralMax) {
  if (dt <= 0) return m.pidOutput;
  m.error = m.setpointRPM - fabsf(m.currentRPM);

  float P = Kp * m.error;

  m.errorSum += m.error * dt;
  m.errorSum  = constrain(m.errorSum, -integralMax, integralMax);
  float I = Ki * m.errorSum;

  float errorDiff = (m.error - m.errorPrev) / dt;
  float D = Kd * errorDiff;
  m.errorPrev = m.error;

  m.pidOutput = P + I + D;
  m.pidOutput = constrain(m.pidOutput, 0.0f, 100.0f);

  return m.pidOutput;
}

float calcularVelocidadMPS(long dticks, float dt_s) {
  if (dt_s <= 0.0f) return 0.0f;
  float rev    = (float)dticks / (float)CPR_OUTPUT;
  float dist_m = rev * WHEEL_CIRC_M;
  return dist_m / dt_s;
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

  if (forward) ledcWrite(in1Pin, pwmValue);
  else         ledcWrite(in2Pin, pwmValue);
}

// ---------------- Parseo de comandos ----------------
void handleLine(String line) {
  line.trim();
  line.toUpperCase();
  if (line.length() < 3) return;

  // Comandos Izquierdos (DL/SL) y Derechos (DR/SR)
  if (line.startsWith("DL")) {
    motor[0].direction = (line.substring(2).toInt() == 1);
    lastCmdMs = millis();
  } else if (line.startsWith("DR")) {
    motor[1].direction = (line.substring(2).toInt() == 1);
    lastCmdMs = millis();
  } else if (line.startsWith("SL")) {
    motor[0].setpointRPM = constrain(line.substring(2).toFloat(), 0.0f, 67.0f);
    lastCmdMs = millis();
  } else if (line.startsWith("SR")) {
    motor[1].setpointRPM = constrain(line.substring(2).toFloat(), 0.0f, 67.0f);
    lastCmdMs = millis();
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

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  delay(300);
  for (int i = 0; i < 2; i++) {
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

    motor[i].direction    = true;
    motor[i].setpointRPM  = 0.0f;
  }

  lastCmdMs      = millis();
  lastSampleTime = millis();
}

// ---------------- Loop ----------------
void loop() {
  readSerialLines();

  // Failsafe
  if (millis() - lastCmdMs > CMD_TIMEOUT_MS) {
    for (int i = 0; i < 2; i++) {
      motor[i].setpointRPM = 0.0f;
      motor[i].errorSum    = 0.0f;
    }
  }

  unsigned long now = millis();
  if (now - lastSampleTime >= SAMPLE_MS) {

    uint32_t dt_ms = now - lastSampleTime;
    float    dt    = dt_ms / 1000.0f;

    long current_ticks[2];

    // Leer ticks de forma segura sin reiniciarlos a 0
    portDISABLE_INTERRUPTS();
    current_ticks[0] = ticks[0];
    current_ticks[1] = ticks[1];
    portENABLE_INTERRUPTS();

    long dticks[2];
    float v_mps[2];

    for (int i = 0; i < 2; i++) {
      // Calcular delta para el PID de velocidad
      dticks[i] = current_ticks[i] - last_ticks[i];
      last_ticks[i] = current_ticks[i];

      motor[i].currentRPM = calcularRPM(dticks[i], dt);
      motor[i].pwmPercent = computePID(motor[i], dt, Kp[i], Ki[i], Kd[i], INTEGRAL_MAX);
      setMotorPins(IN1[i], IN2[i], motor[i].pwmPercent, motor[i].direction);
      
      v_mps[i] = calcularVelocidadMPS(dticks[i], dt);
    }

    // Paquete UART adaptado: ID, seq, dt_ms, ticksAbs_L, rpm_L, ticksAbs_R, rpm_R
    Serial.print(ESP_ID);
    Serial.print(","); Serial.print(seq++); 
    Serial.print(","); Serial.print(dt_ms);

    for (int i = 0; i < 2; i++) {
      Serial.print(","); Serial.print(current_ticks[i]);
      Serial.print(","); Serial.print(motor[i].currentRPM, 2);
    }
    Serial.println();

    lastSampleTime = now;
  }
}
