// =====================================================
// CONTROLADOR 3 MOTORES (ROVER 6WD - Lado Independiente)
// =====================================================

// !!! IMPORTANTE: CAMBIA ESTO SEGÚN EL LADO !!!
// Usa "ESP_L" para la izquierda, "ESP_R" para la derecha
#define DEVICE_ID "ESP_L" 

// --- DEFINICIÓN DE PINES (Ajusta según tu conexión) ---
// MOTOR 1
#define M1_IN1 27
#define M1_IN2 14
#define M1_ENC_A 32
#define M1_ENC_B 33

// MOTOR 2
#define M2_IN1 25
#define M2_IN2 26
#define M2_ENC_A 4
#define M2_ENC_B 16 // Ojo: RX2 suele ser 16, revisa tu board

// MOTOR 3
#define M3_IN1 18
#define M3_IN2 19
#define M3_ENC_A 21
#define M3_ENC_B 22

// --- CONFIGURACIÓN PWM ---
#define PWM_FREQ 20000
#define PWM_RES 10
#define PWM_MAX ((1 << PWM_RES) - 1)
const float PWM_MIN = 20.0; // Mínimo para mover motor

// --- PARÁMETROS FÍSICOS ---
const float GEAR_RATIO = 56.25;
const int PULSES_PER_REV = 16;
const int CPR = PULSES_PER_REV * 4 * GEAR_RATIO; // ~3600 ticks/vuelta

// --- ESTRUCTURA DE UN MOTOR ---
struct Motor {
  // Hardware Pins
  int pinIN1, pinIN2;
  int pinEncA, pinEncB;
  int pwmCh1, pwmCh2; // Canales LEDC
  
  // Encoder state
  volatile long ticks;
  long lastTicks; // Para calcular delta

  // PID state
  float setpointRPM;
  float currentRPM;
  float errorSum;
  float errorPrev;
  float outputPercent;
  
  // Constantes PID (puedes ajustarlas individualmente si un motor es diferente)
  float Kp, Ki, Kd;
};

// Inicializamos los 3 motores
Motor m1 = {M1_IN1, M1_IN2, M1_ENC_A, M1_ENC_B, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0.5, 1.0, 0.0};
Motor m2 = {M2_IN1, M2_IN2, M2_ENC_A, M2_ENC_B, 2, 3, 0, 0, 0, 0, 0, 0, 0, 0.5, 1.0, 0.0};
Motor m3 = {M3_IN1, M3_IN2, M3_ENC_A, M3_ENC_B, 4, 5, 0, 0, 0, 0, 0, 0, 0, 0.5, 1.0, 0.0};

// Globales de Control
bool globalDirection = true; // 1 = Forward, 0 = Reverse
uint32_t seq = 0;
unsigned long lastSampleTime = 0;
unsigned long lastCmdMs = 0;
const unsigned long SAMPLE_MS = 50; // 20Hz refresco
const unsigned long TIMEOUT_MS = 500;

String inputBuffer = "";

// ================= INTERRUPCIONES (ISRs) =================
// Deben ser funciones estáticas void. Es tedioso pero seguro.

// Motor 1
void IRAM_ATTR ISR_M1_A() { if(digitalRead(M1_ENC_A) == digitalRead(M1_ENC_B)) m1.ticks--; else m1.ticks++; }
void IRAM_ATTR ISR_M1_B() { if(digitalRead(M1_ENC_A) != digitalRead(M1_ENC_B)) m1.ticks--; else m1.ticks++; }

// Motor 2
void IRAM_ATTR ISR_M2_A() { if(digitalRead(M2_ENC_A) == digitalRead(M2_ENC_B)) m2.ticks--; else m2.ticks++; }
void IRAM_ATTR ISR_M2_B() { if(digitalRead(M2_ENC_A) != digitalRead(M2_ENC_B)) m2.ticks--; else m2.ticks++; }

// Motor 3
void IRAM_ATTR ISR_M3_A() { if(digitalRead(M3_ENC_A) == digitalRead(M3_ENC_B)) m3.ticks--; else m3.ticks++; }
void IRAM_ATTR ISR_M3_B() { if(digitalRead(M3_ENC_A) != digitalRead(M3_ENC_B)) m3.ticks--; else m3.ticks++; }

// ================= LOGICA PID Y MOTORES =================

void setupMotor(Motor &m, void (*isrA)(), void (*isrB)()) {
  pinMode(m.pinEncA, INPUT_PULLUP);
  pinMode(m.pinEncB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(m.pinEncA), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(m.pinEncB), isrB, CHANGE);

  ledcAttach(m.pinIN1, m.pwmCh1);
  ledcAttach(m.pinIN2, m.pwmCh2);
  ledcSetup(m.pwmCh1, PWM_FREQ, PWM_RES);
  ledcSetup(m.pwmCh2, PWM_FREQ, PWM_RES);
  ledcWrite(m.pwmCh1, 0);
  ledcWrite(m.pwmCh2, 0);
}

void driveMotor(Motor &m, bool forward) {
  int pwmVal = 0;
  // Convertir porcentaje 0-100 a valor PWM
  if (m.outputPercent > 0.1) {
    // Mapeo simple
    pwmVal = map((long)(m.outputPercent*100), 0, 10000, 0, PWM_MAX);
    pwmVal = constrain(pwmVal, 0, PWM_MAX);
  }

  if (pwmVal == 0) {
    ledcWrite(m.pwmCh1, 0);
    ledcWrite(m.pwmCh2, 0);
    return;
  }

  if (forward) {
    ledcWrite(m.pwmCh1, pwmVal);
    ledcWrite(m.pwmCh2, 0);
  } else {
    ledcWrite(m.pwmCh1, 0);
    ledcWrite(m.pwmCh2, pwmVal);
  }
}

void updatePID(Motor &m, float dt) {
  if (dt <= 0) return;

  float error = m.setpointRPM - m.currentRPM; // currentRPM ya es positivo
  
  // P
  float P = m.Kp * error;

  // I
  m.errorSum += error * dt;
  m.errorSum = constrain(m.errorSum, -200.0, 200.0); // Anti-windup
  float I = m.Ki * m.errorSum;

  // D
  float errorDiff = (error - m.errorPrev) / dt;
  float D = m.Kd * errorDiff;
  m.errorPrev = error;

  m.outputPercent = P + I + D;
  m.outputPercent = constrain(m.outputPercent, 0.0, 100.0);
}

// ================= COMUNICACIÓN =================

void parseCommand(String line) {
  // Formato recibido desde Python: CMD,dir,rpm
  // Ejemplo: CMD,D1,20.5  o  CMD,1,20.5
  
  // Quitamos CMD,
  int firstComma = line.indexOf(',');
  if (firstComma == -1) return;
  
  String params = line.substring(firstComma + 1);
  int secondComma = params.indexOf(',');
  if (secondComma == -1) return;

  String sDir = params.substring(0, secondComma);
  String sRpm = params.substring(secondComma + 1);

  // Parsear Dirección
  // El python manda "D1" o "1". Manejamos ambos.
  if (sDir.startsWith("D")) sDir.remove(0, 1);
  int dirVal = sDir.toInt();
  globalDirection = (dirVal == 1);

  // Parsear RPM
  float rpmVal = sRpm.toFloat();
  rpmVal = constrain(rpmVal, 0.0, 80.0); // Limite de seguridad

  // Actualizar Setpoints de LOS 3 MOTORES
  m1.setpointRPM = rpmVal;
  m2.setpointRPM = rpmVal;
  m3.setpointRPM = rpmVal;
  
  lastCmdMs = millis();
}

// ================= MAIN =================

void setup() {
  Serial.begin(115200);
  
  setupMotor(m1, ISR_M1_A, ISR_M1_B);
  setupMotor(m2, ISR_M2_A, ISR_M2_B);
  setupMotor(m3, ISR_M3_A, ISR_M3_B);

  lastSampleTime = millis();
  lastCmdMs = millis();
}

void loop() {
  // 1. Leer Serial
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      inputBuffer.trim();
      if (inputBuffer.startsWith("CMD")) {
        parseCommand(inputBuffer);
      }
      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }

  // 2. Failsafe (Detener si no hay señal)
  if (millis() - lastCmdMs > TIMEOUT_MS) {
    m1.setpointRPM = 0; m2.setpointRPM = 0; m3.setpointRPM = 0;
    m1.errorSum = 0; m2.errorSum = 0; m3.errorSum = 0;
  }

  // 3. Ciclo de Control y Telemetría
  unsigned long now = millis();
  if (now - lastSampleTime >= SAMPLE_MS) {
    float dt = (now - lastSampleTime) / 1000.0;
    
    // --- ATOMIC BLOCK START ---
    portDISABLE_INTERRUPTS();
    long t1 = m1.ticks;
    long t2 = m2.ticks;
    long t3 = m3.ticks;
    portENABLE_INTERRUPTS();
    // --- ATOMIC BLOCK END ---

    // Calcular Delta Ticks para RPM
    long dt1 = t1 - m1.lastTicks; m1.lastTicks = t1;
    long dt2 = t2 - m2.lastTicks; m2.lastTicks = t2;
    long dt3 = t3 - m3.lastTicks; m3.lastTicks = t3;

    // Calcular RPM (Absoluto para el PID)
    m1.currentRPM = abs((dt1 / (float)CPR) * (60.0 / dt));
    m2.currentRPM = abs((dt2 / (float)CPR) * (60.0 / dt));
    m3.currentRPM = abs((dt3 / (float)CPR) * (60.0 / dt));

    // Ejecutar PID
    updatePID(m1, dt);
    updatePID(m2, dt);
    updatePID(m3, dt);

    // Mover motores
    driveMotor(m1, globalDirection);
    driveMotor(m2, globalDirection);
    driveMotor(m3, globalDirection);

    // --- ENVIAR TELEMETRÍA ---
    // Formato: HEADER, seq, dt_ms, t1, r1, t2, r2, t3, r3
    // Nota: Enviamos ticks crudos acumulados (t1, t2, t3) para que la odometría de Python no pierda posición
    Serial.print(DEVICE_ID); Serial.print(",");
    Serial.print(seq++); Serial.print(",");
    Serial.print((int)(dt * 1000)); Serial.print(",");
    
    Serial.print(t1); Serial.print(",");
    Serial.print(m1.currentRPM, 1); Serial.print(",");
    
    Serial.print(t2); Serial.print(",");
    Serial.print(m2.currentRPM, 1); Serial.print(",");
    
    Serial.print(t3); Serial.print(",");
    Serial.println(m3.currentRPM, 1);

    lastSampleTime = now;
  }
}
