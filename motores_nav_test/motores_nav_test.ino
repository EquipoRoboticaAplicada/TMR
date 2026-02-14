// =====================================================
// Control 1 Motor DC con Encoder JGB37-3530 + PID
// UART por USB (Serial) desde Raspberry Pi
// Commands:
//   D1  -> forward
//   D0  -> reverse
//   S20.0 -> setpoint RPM
// =====================================================

#define IN1 27
#define IN2 14

#define ENC_A 32
#define ENC_B 33

#define PWM_FREQ 20000
#define PWM_RESOLUTION 10
const int PWM_MAX = (1 << PWM_RESOLUTION) - 1;

const float PWM_MIN = 20.0;
const float GEAR_RATIO = 56.25;
const int   PULSES_PER_REV = 16;
const int   CPR_OUTPUT = PULSES_PER_REV * 4 * GEAR_RATIO; // 3600

const unsigned long SAMPLE_MS = 100;

// Wheel parameters
const float WHEEL_DIAM_M = 0.062f;
const float WHEEL_CIRC_M = 3.14159265f * WHEEL_DIAM_M;

uint32_t seq = 0;

// PID gains
float Kp_R = 0.0, Ki_R = 1.0, Kd_R = 0.0;
const float INTEGRAL_MAX_R = 200.0;

// Failsafe: si no llegan comandos, frena
const unsigned long CMD_TIMEOUT_MS = 400;
unsigned long lastCmdMs = 0;

struct PIDState {
  float setpointRPM = 0.0;
  float currentRPM  = 0.0;

  float error       = 0.0;
  float errorSum    = 0.0;
  float errorPrev   = 0.0;

  float pidOutput   = 0.0;   // 0..100%
  float pwmPercent  = 0.0;   // 0..100%

  bool  direction   = true;  // true=forward, false=reverse
};

PIDState motor;

volatile long ticksR = 0;
unsigned long lastSampleTime = 0;

String inputBuffer = "";

// ---------------- ISR Encoder (x4) ----------------
void IRAM_ATTR ISR_R_A() {
  bool A = digitalRead(ENC_A);
  bool B = digitalRead(ENC_B);
  if (A == B) ticksR--;
  else        ticksR++;
}

void IRAM_ATTR ISR_R_B() {
  bool A = digitalRead(ENC_A);
  bool B = digitalRead(ENC_B);
  if (A != B) ticksR--;
  else        ticksR++;
}

// ---------------- Utilidades ----------------
float calcularRPM(long ticks, float dt) {
  return (ticks / (float)CPR_OUTPUT) * (60.0 / dt);
}

float computePID(PIDState &m, float dt, float Kp, float Ki, float Kd, float integralMax) {
  if (dt <= 0) return m.pidOutput;

  m.error = m.setpointRPM - abs(m.currentRPM);

  float P = Kp * m.error;

  m.errorSum += m.error * dt;
  m.errorSum = constrain(m.errorSum, -integralMax, integralMax);
  float I = Ki * m.errorSum;

  float errorDiff = (m.error - m.errorPrev) / dt;
  float D = Kd * errorDiff;
  m.errorPrev = m.error;

  m.pidOutput = P + I + D;
  m.pidOutput = constrain(m.pidOutput, 0.0, 100.0);

  return m.pidOutput;
}

float calcularVelocidadMPS(long dticks, float dt_s, bool forward) {
  if (dt_s <= 0) return 0.0f;

  // Revoluciones de la rueda en este intervalo
  float rev = (float)dticks / (float)CPR_OUTPUT;

  // Distancia recorrida en este intervalo
  float dist_m = rev * WHEEL_CIRC_M;

  // Velocidad
  float v = dist_m / dt_s;

  // Signo por dirección (si quieres v signed)
  return forward ? v : -v;
}

void setMotorPins(int in1Pin, int in2Pin, float percent, bool forward) {
  percent = constrain(percent, 0.0, 100.0);

  int pwmValue = 0;

  if (percent >= 0.1) {
    float percentReal = map((long)(percent * 10), 0, 1000, (long)(PWM_MIN * 10), 1000) / 10.0;
    pwmValue = (int)((percentReal / 100.0) * PWM_MAX);
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

  // Dirección: D0 / D1
  if (line[0] == 'D') {
    int v = line.substring(1).toInt();
    motor.direction = (v == 1);
    lastCmdMs = millis();
    return;
  }

  // Setpoint: Sxx.x
  if (line[0] == 'S') {
    float sp = line.substring(1).toFloat();
    sp = constrain(sp, 0.0, 67.0);
    motor.setpointRPM = sp;
    lastCmdMs = millis();
    return;
  }

  // Puedes agregar más comandos aquí si ocupas
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
      if (inputBuffer.length() > 64) inputBuffer = ""; // evita overflow
    }
  }
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  delay(300);

  ledcAttach(IN1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(IN2, PWM_FREQ, PWM_RESOLUTION);
  ledcWrite(IN1, 0);
  ledcWrite(IN2, 0);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), ISR_R_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), ISR_R_B, CHANGE);

  motor.direction = true;
  motor.setpointRPM = 0.0;

  lastCmdMs = millis();
  lastSampleTime = millis();
}

// ---------------- Loop ----------------
void loop() {
  readSerialLines();

  // Failsafe: si no llegan comandos, frena
  if (millis() - lastCmdMs > CMD_TIMEOUT_MS) {
    motor.setpointRPM = 0.0;
    motor.errorSum = 0.0;  // evita windup si se queda sin comando
  }

  unsigned long now = millis();
  if (now - lastSampleTime >= SAMPLE_MS) {

    portDISABLE_INTERRUPTS();
    long tR = ticksR; ticksR = 0;
    portENABLE_INTERRUPTS();

    float dt = (now - lastSampleTime) / 1000.0;

    motor.currentRPM = calcularRPM(tR, dt);
    motor.pwmPercent = computePID(motor, dt, Kp_R, Ki_R, Kd_R, INTEGRAL_MAX_R);

    setMotorPins(IN1, IN2, motor.pwmPercent, motor.direction);

//    // Debug (opcional)
//    Serial.print("SP:");  Serial.print(motor.setpointRPM, 1);
//    Serial.print(" PV:"); Serial.print(motor.currentRPM, 1);
//    Serial.print(" PWM:");Serial.println(motor.pwmPercent, 1);

    float v_mps = calcularVelocidadMPS(tR, dt, motor.direction);
    // --- Telemetría para la Raspberry (una línea fácil de parsear) ---
    // Formato: ODOM,seq,t_ms,dt_ms,dticks,dir,rpm,v_mps
    Serial.print("ODOM,");
    Serial.print(seq++); Serial.print(","); // cantidad de paquetes enviados
    Serial.print(now); Serial.print(","); // tiempo actual
    Serial.print((unsigned long)(dt * 1000.0f)); Serial.print(","); // diferencia del tiempo previo al actual
    Serial.print(tR); Serial.print(","); // ticks actuales
    Serial.print(motor.direction ? 1 : 0); Serial.print(","); // dirección 
    Serial.print(motor.currentRPM, 2); Serial.print(",");
    Serial.println(v_mps, 4);

    lastSampleTime = now;
  }
}

//el formato que te di solamente es para un motor, lo que era para pruebas, en realidad vamos a tener 6 motores con sus respectivos encoders y 2 ESP32 (cada una va a controlar 3 motores). Como sugirirías compartir todas esa información para los 6 motores.
//
//Formato esperado: ID, seq, dt, t1, r1, t2, r2, t3, r3
//    Ej: ESP_L,100,50,10,2.5,10,2.5,10,2.5
