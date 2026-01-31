// =====================================================
// Control 23 Motores DC con 3 Encoders JGB37-3530
// 3 PID independientes - ESP32 der
// Drivers: IBT-4 (cada motor usa IN1/IN2)
// =====================================================

// Motores
#define IN1 27   // PWM Forward
#define IN2 14   // PWM Reverse  (ejemplo; cambia si lo necesitas)

// -------------------- Pines Encoders -----------------

// Encoders
#define ENC_A 32
#define ENC_B 33

// -------------------- PWM ESP32 ----------------------
#define PWM_FREQ 20000
#define PWM_RESOLUTION 10
const int PWM_MAX = (1 << PWM_RESOLUTION) - 1;  // 255 si RES=8

// -------------------- Parámetros motor/encoder --------
const float PWM_MIN = 20.0;   // % mínimo donde el motor gira
const float GEAR_RATIO = 56.25;
const int   PULSES_PER_REV = 16;
const int   CPR_OUTPUT = PULSES_PER_REV * 4 * GEAR_RATIO; // 3600

const unsigned long SAMPLE_MS = 100;

// -------------------- PID (Ganancias) ----------------
// Puedes dejarlas iguales o ajustarlas por separado
float Kp_R = 0.0, Ki_R = 1.0, Kd_R = 0.0;

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

  bool  direction   = false;  // true=forward, false=reverse
};

PIDState rightMotor;

// -------------------- Ticks encoders ------------------
volatile long ticksL = 0;
volatile long ticksR = 0;

unsigned long lastSampleTime = 0;

// =====================================================
// ISR Encoder RIGHT (x4)
// =====================================================
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
    if (abs(newSetpoint - rightMotor.setpointRPM) > 0.1) {
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
    if (newDir != rightMotor.direction) {
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
  ledcAttach(IN1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(IN2, PWM_FREQ, PWM_RESOLUTION);

  ledcWrite(IN1, 0); ledcWrite(IN2, 0);

  // Encoder pins
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  // Entrada externa
  pinMode(RASP_INPUT, INPUT_PULLDOWN);

  // Interrupts encoders
  attachInterrupt(digitalPinToInterrupt(ENC_A), ISR_R_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), ISR_R_B, CHANGE);

  // Direcciones iniciales (ajusta)
  rightMotor.direction = true;

  lastSampleTime = millis();
}

// =====================================================
// Loop
// =====================================================
void loop() {


//  // Procesa entrada serial
//  while (Serial.available()) {
//    char c = (char)Serial.read();
//    
//    if (c == '\n' || c == '\r') {
//      if (inputBuffer.length() > 0) {
//        handleCommand(inputBuffer);
//        inputBuffer = "";
//      }
//    } else {
//      inputBuffer += c;
//    }
//  }

  unsigned long now = millis();
  if (now - lastSampleTime >= SAMPLE_MS) {

    // Lee ambos encoders de forma atómica
    portDISABLE_INTERRUPTS();
    long tR = ticksR; ticksR = 0;
    portENABLE_INTERRUPTS();

    float dt = (now - lastSampleTime) / 1000.0;

    // RPM por motor
    rightMotor.currentRPM = calcularRPM(tR, dt);

    // PID independiente
    rightMotor.pwmPercent = computePID(rightMotor, dt, Kp_R, Ki_R, Kd_R, INTEGRAL_MAX_R);

    // Aplicar PWM a cada IBT-4
    setMotorPins(IN1, IN2, rightMotor.pwmPercent, rightMotor.direction);

    // Serial Plotter (una sola línea)
    Serial.print(" SP_R:");  Serial.print(rightMotor.setpointRPM, 2);
    Serial.print(" PV_R:");  Serial.print(rightMotor.currentRPM, 2);
    Serial.print(" PWM_R:"); Serial.println(rightMotor.pwmPercent, 1);

    lastSampleTime = now;
  }
}
