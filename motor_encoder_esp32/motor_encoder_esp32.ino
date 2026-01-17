// =====================================================
// Control 2 Motores DC con 2 Encoders JGB37-3530
// 2 PID independientes - ESP32
// Drivers: 2x IBT-4 (cada motor usa IN1/IN2)
// =====================================================

// -------------------- Pines IBT-4 --------------------
// Motor Izquierdo (LEFT)
#define L_IN1 25   // PWM Forward
#define L_IN2 26   // PWM Reverse

// Motor Derecho (RIGHT)
#define R_IN1 27   // PWM Forward
#define R_IN2 14   // PWM Reverse  (ejemplo; cambia si lo necesitas)

// -------------------- Pines Encoders -----------------
// Encoder Izquierdo
#define L_ENC_A 18
#define L_ENC_B 19

// Encoder Derecho
#define R_ENC_A 32
#define R_ENC_B 33

// -------------------- PWM ESP32 ----------------------
#define PWM_FREQ 20000
#define PWM_RESOLUTION 10
const int PWM_MAX = (1 << PWM_RESOLUTION) - 1;  // 255 si RES=8

// Entrada externa (ejemplo)
#define RASP_INPUT 4   // cambié a 4 porque 32/33 ya se usan; ajusta a tu caso

// -------------------- Parámetros motor/encoder --------
const float PWM_MIN = 20.0;   // % mínimo donde el motor gira
const float GEAR_RATIO = 56.25;
const int   PULSES_PER_REV = 16;
const int   CPR_OUTPUT = PULSES_PER_REV * 4 * GEAR_RATIO; // 3600

const unsigned long SAMPLE_MS = 100;

// -------------------- PID (Ganancias) ----------------
// Puedes dejarlas iguales o ajustarlas por separado
float Kp_L = 0.0, Ki_L = 1.0, Kd_L = 0.0;
float Kp_R = 0.0, Ki_R = 1.0, Kd_R = 0.0;

const float INTEGRAL_MAX_L = 200.0;
const float INTEGRAL_MAX_R = 200.0;

// -------------------- Estados por motor ----------------
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

PIDState leftMotor;
PIDState rightMotor;

// -------------------- Ticks encoders ------------------
volatile long ticksL = 0;
volatile long ticksR = 0;

unsigned long lastSampleTime = 0;

// =====================================================
// ISR Encoder LEFT (x4)
// =====================================================
void IRAM_ATTR ISR_L_A() {
  bool A = digitalRead(L_ENC_A);
  bool B = digitalRead(L_ENC_B);
  if (A == B) ticksL--;
  else        ticksL++;
}

void IRAM_ATTR ISR_L_B() {
  bool A = digitalRead(L_ENC_A);
  bool B = digitalRead(L_ENC_B);
  if (A != B) ticksL--;
  else        ticksL++;
}

// =====================================================
// ISR Encoder RIGHT (x4)
// =====================================================
void IRAM_ATTR ISR_R_A() {
  bool A = digitalRead(R_ENC_A);
  bool B = digitalRead(R_ENC_B);
  if (A == B) ticksR--;
  else        ticksR++;
}

void IRAM_ATTR ISR_R_B() {
  bool A = digitalRead(R_ENC_A);
  bool B = digitalRead(R_ENC_B);
  if (A != B) ticksR--;
  else        ticksR++;
}

// =====================================================
// Utilidades
// =====================================================
float calcularRPM(long ticks, float dt) {
  return (ticks / (float)CPR_OUTPUT) * (60.0 / dt);
}

// PID genérico (independiente por motor)
float computePID(PIDState &m, float dt, float Kp, float Ki, float Kd, float integralMax) {
  if (dt <= 0) return m.pidOutput;

  // error con RPM medida en valor absoluto (igual que tu versión)
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

// Control motor para IBT-4 (IN1/IN2)
void setMotorPins(int in1Pin, int in2Pin, float percent, bool forward) {
  percent = constrain(percent, 0.0, 100.0);

  int pwmValue = 0;

  if (percent >= 0.1) {
    // 0-100% lógico -> PWM_MIN-100% físico
    float percentReal = map((long)(percent * 10), 0, 1000, (long)(PWM_MIN * 10), 1000) / 10.0;
    pwmValue = (int)((percentReal / 100.0) * PWM_MAX);
    pwmValue = constrain(pwmValue, 0, PWM_MAX);
  }

  // Apaga ambos siempre
  ledcWrite(in1Pin, 0);
  ledcWrite(in2Pin, 0);

  if (pwmValue == 0) return;

  if (forward) ledcWrite(in1Pin, pwmValue);
  else         ledcWrite(in2Pin, pwmValue);
}

// ============================================
// Variables globales necesarias
// ============================================
String inputBuffer = "";  // Buffer para comandos seriales

// ============================================
// Función: Procesar comandos seriales
// ============================================
void handleCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  
  if (cmd.length() == 0) return;
  
  cmd.replace(",", " ");
  cmd.replace("  ", " ");
  
  // Comando S (Setpoint RPM)
  int sIdx = cmd.indexOf('S');
  if (sIdx != -1) {
    int endIdx = cmd.indexOf(' ', sIdx);
    if (endIdx == -1) endIdx = cmd.length();
    
    String sStr = cmd.substring(sIdx + 1, endIdx);
    sStr.trim();
    
    float newSetpoint = constrain(sStr.toFloat(), 0.0, 67.0);
    
    // Actualiza AMBOS motores con el mismo setpoint
    if (abs(newSetpoint - leftMotor.setpointRPM) > 0.1) {
      leftMotor.setpointRPM = newSetpoint;
      rightMotor.setpointRPM = newSetpoint;
    }
  }
  
  // Comando D (Dirección)
  int dIdx = cmd.indexOf('D');
  if (dIdx != -1) {
    String dStr = cmd.substring(dIdx + 1);
    dStr.trim();
    int dirValue = dStr.toInt();
    bool newDir = (dirValue == 1);
    
    // Actualiza AMBOS motores con la misma dirección
    if (newDir != leftMotor.direction) {
      leftMotor.direction = newDir;
      rightMotor.direction = newDir;
    }
  }
}

// =====================================================
// Setup
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  // PWM attach (Core 3.x: ledcAttach(pin,freq,res))
  ledcAttach(L_IN1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(L_IN2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(R_IN1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(R_IN2, PWM_FREQ, PWM_RESOLUTION);

  ledcWrite(L_IN1, 0); ledcWrite(L_IN2, 0);
  ledcWrite(R_IN1, 0); ledcWrite(R_IN2, 0);

  // Encoder pins
  pinMode(L_ENC_A, INPUT_PULLUP);
  pinMode(L_ENC_B, INPUT_PULLUP);
  pinMode(R_ENC_A, INPUT_PULLUP);
  pinMode(R_ENC_B, INPUT_PULLUP);

  // Entrada externa
  pinMode(RASP_INPUT, INPUT_PULLDOWN);

  // Interrupts encoders
  attachInterrupt(digitalPinToInterrupt(L_ENC_A), ISR_L_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(L_ENC_B), ISR_L_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_ENC_A), ISR_R_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_ENC_B), ISR_R_B, CHANGE);

  // Direcciones iniciales (ajusta)
  leftMotor.direction  = true;
  rightMotor.direction = true;

  lastSampleTime = millis();
}

// =====================================================
// Loop
// =====================================================
void loop() {
//  // Ejemplo: mismo setpoint para ambos motores desde una entrada
//  if (digitalRead(RASP_INPUT) == HIGH) {
//    leftMotor.setpointRPM  = 45;
//    rightMotor.setpointRPM = 45;
//  } else {
//    leftMotor.setpointRPM  = 0;
//    rightMotor.setpointRPM = 0;
//  }

  // Procesa entrada serial
  while (Serial.available()) {
    char c = (char)Serial.read();
    
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        handleCommand(inputBuffer);
        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
    }
  }

  unsigned long now = millis();
  if (now - lastSampleTime >= SAMPLE_MS) {

    // Lee ambos encoders de forma atómica
    portDISABLE_INTERRUPTS();
    long tL = ticksL; ticksL = 0;
    long tR = ticksR; ticksR = 0;
    portENABLE_INTERRUPTS();

    float dt = (now - lastSampleTime) / 1000.0;

    // RPM por motor
    leftMotor.currentRPM  = calcularRPM(tL, dt);
    rightMotor.currentRPM = calcularRPM(tR, dt);

    // PID independiente
    leftMotor.pwmPercent  = computePID(leftMotor,  dt, Kp_L, Ki_L, Kd_L, INTEGRAL_MAX_L);
    rightMotor.pwmPercent = computePID(rightMotor, dt, Kp_R, Ki_R, Kd_R, INTEGRAL_MAX_R);

    // Aplicar PWM a cada IBT-4
    setMotorPins(L_IN1, L_IN2, leftMotor.pwmPercent,  leftMotor.direction);
    setMotorPins(R_IN1, R_IN2, rightMotor.pwmPercent, rightMotor.direction);

    // Serial Plotter (una sola línea)
    Serial.print("SP_L:");   Serial.print(leftMotor.setpointRPM, 2);
    Serial.print(" PV_L:");  Serial.print(leftMotor.currentRPM, 2);
//    Serial.print(" PWM_L:"); Serial.print(leftMotor.pwmPercent, 1);

    Serial.print(" SP_R:");  Serial.print(rightMotor.setpointRPM, 2);
    Serial.print(" PV_R:");  Serial.print(rightMotor.currentRPM, 2);
//    Serial.print(" PWM_R:"); Serial.println(rightMotor.pwmPercent, 1);

    lastSampleTime = now;
  }
}
