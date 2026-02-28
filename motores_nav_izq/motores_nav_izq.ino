void IRAM_ATTR encoderISR(void* arg);
const char* ESP_ID = "ESP_L";

// Para IBT-2: IN1 = RPWM, IN2 = LPWM
// Orden: 0=adelante, 1=en medio, 2=atrás
const int IN1[3] = {27, 26, 25};  // RPWM
const int IN2[3] = {14, 13, 23};  // LPWM

// FIX #1: Los pines de encoder YA NO comparten pines con IN1/IN2.
// Antes: ENC_A = {32, 25, 26} → pines 25 y 26 colisionaban con IN1[2] e IN1[1].
// Ajusta estos valores a los pines físicos reales que uses para encoders.
const int ENC_A[3] = {32, 34, 36};  // ← CORRECCIÓN: pines sin conflicto
const int ENC_B[3] = {33, 35, 39};  // ← CORRECCIÓN: pines sin conflicto

#define PWM_FREQ       20000
#define PWM_RESOLUTION 10
const int PWM_MAX = (1 << PWM_RESOLUTION) - 1;

const float PWM_MIN         = 20.0;
const float GEAR_RATIO      = 56.25;
const int   PULSES_PER_REV  = 16;
const int   CPR_OUTPUT      = PULSES_PER_REV * 4 * GEAR_RATIO; // 3600

const unsigned long SAMPLE_MS = 100;

const float WHEEL_DIAM_M = 0.062f;
const float WHEEL_CIRC_M = 3.14159265f * WHEEL_DIAM_M;

uint32_t seq = 0;

// PID gains
float Kp[3] = {0.0, 0.0, 0.0};
float Ki[3] = {1.0, 1.0, 1.0};
float Kd[3] = {0.0, 0.0, 0.0};
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

PIDState motor[3];

volatile long  ticks[3]        = {0, 0, 0};
// FIX #2: estado previo por encoder para decodificación x4 confiable
volatile int8_t lastEncState[3] = {0, 0, 0};

unsigned long lastSampleTime = 0;
String inputBuffer = "";

// ---------------- ISR Encoder x4 (lookup table) ----------------
// FIX #2: reemplaza la lógica frágil "if (A == B)" por tabla de estados.
void IRAM_ATTR encoderISR(void* arg) {
  int i = (int)(intptr_t)arg;

  bool A = digitalRead(ENC_A[i]);
  bool B = digitalRead(ENC_B[i]);

  int8_t state = ((int8_t)A << 1) | (int8_t)B;
  int8_t prev  = lastEncState[i];
  lastEncState[i] = state;

  // Secuencia forward:  00→10→11→01→00  (+1 cada transición válida)
  // Secuencia backward: 00→01→11→10→00  (-1 cada transición válida)
  static const int8_t lookup[16] = {
     0,  1, -1,  0,
    -1,  0,  0,  1,
     1,  0,  0, -1,
     0, -1,  1,  0
  };

  ticks[i] += lookup[(prev << 2) | state];
}

// ---------------- Utilidades ----------------

// FIX #4: protección contra dt == 0 para evitar inf/NaN en serial
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

// FIX #3: velocidad con signo derivado de los ticks (no de la dirección comandada).
// Ticks positivos → avance, negativos → retroceso, independientemente del comando.
float calcularVelocidadMPS(long dticks, float dt_s) {
  if (dt_s <= 0.0f) return 0.0f;
  //  float rev    = (float)dticks / (float)CPR_OUTPUT;  // signed
  //  float dist_m = rev * WHEEL_CIRC_M;                 // signed
  //  return dist_m / dt_s;                              // m/s signed
  float rpm = calcularRPM(dticks, dt_s);
  float m_s = WHEEL_CIRC_M * rpm / 60;
  return m_s;
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
  if (line.length() < 2) return;

  if (line[0] == 'D') {
    bool dir = (line.substring(1).toInt() == 1);
    for (int i = 0; i < 3; i++) motor[i].direction = dir;
    lastCmdMs = millis();
    return;
  }

  if (line[0] == 'S') {
    float sp = constrain(line.substring(1).toFloat(), 0.0f, 67.0f);
    for (int i = 0; i < 3; i++) motor[i].setpointRPM = sp;
    lastCmdMs = millis();
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

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  delay(300);

  for (int i = 0; i < 3; i++) {
    // Configurar PWM de motores
    ledcAttach(IN1[i], PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(IN2[i], PWM_FREQ, PWM_RESOLUTION);
    ledcWrite(IN1[i], 0);
    ledcWrite(IN2[i], 0);

    // Configurar encoders (pines ahora sin conflicto con IN1/IN2)
    pinMode(ENC_A[i], INPUT_PULLUP);
    pinMode(ENC_B[i], INPUT_PULLUP);

    // Inicializar estado previo del encoder
    bool A = digitalRead(ENC_A[i]);
    bool B = digitalRead(ENC_B[i]);
    lastEncState[i] = ((int8_t)A << 1) | (int8_t)B;

    // Interrupción en ambos canales para x4
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
//    if (millis() - lastCmdMs > CMD_TIMEOUT_MS) {
//      for (int i = 0; i < 3; i++) {
//        motor[i].setpointRPM = 0.0f;
//      motor[i].errorSum    = 0.0f;
//      }
//    }

   motor[0].setpointRPM = 40; 

  unsigned long now = millis();
  if (now - lastSampleTime >= SAMPLE_MS) {

    uint32_t dt_ms = now - lastSampleTime;
    float    dt    = fabs(dt_ms) / 1000.0f;

    long dticks[3];

    portDISABLE_INTERRUPTS();
    dticks[0] = ticks[0]; ticks[0] = 0;
    dticks[1] = ticks[1]; ticks[1] = 0;
    dticks[2] = ticks[2]; ticks[2] = 0;
    portENABLE_INTERRUPTS();

    float v_mps[3];

    for (int i = 0; i < 3; i++) {
      motor[i].currentRPM = calcularRPM(dticks[i], dt);
      motor[i].pwmPercent = computePID(motor[i], dt, Kp[i], Ki[i], Kd[i], INTEGRAL_MAX);
      setMotorPins(IN1[i], IN2[i], motor[i].pwmPercent, motor[i].direction);

      // FIX #3: velocidad con signo real de encoder, no de dirección comandada
      v_mps[i] = calcularVelocidadMPS(dticks[i], dt);
    }
    
     // Paquete UART: ID, seq, dt_ms, dticks0, v0, dticks1, v1, dticks2, v2
      Serial.print(ESP_ID); Serial.print(",");
      Serial.print(seq++);  Serial.print(",");
      Serial.print(fabs(dt_ms));
  
      for (int i = 0; i < 3; i++) {
        Serial.print(",");
        Serial.print(dticks[i]);
        Serial.print(",");
        Serial.print(v_mps[i], 4);
      }
      Serial.println();

      // Diagnóstico
//    Serial.print("SP:");
//    Serial.print(motor[0].setpointRPM);
//    Serial.print("\t");
//    Serial.print("PV:");
//    Serial.println(calcularRPM(dticks[0], dt));

    lastSampleTime = now;
  }
}
