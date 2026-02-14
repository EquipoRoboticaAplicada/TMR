const char* ESP_ID = "ESP_L"; 

// Para IBT-2: IN1 = RPWM, IN2 = LPWM
// Orden: 0=adelante, 1=en medio, 2=atrás
const int IN1[3] = {27, 26, 25};  // RPWM
const int IN2[3] = {14, 13, 23};  // LPWM

const int ENC_A[3] = {32, 25, 26};  // cambia por tus pines reales
const int ENC_B[3] = {33, 34, 35};  // cambia por tus pines reales

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
float Kp[3] = {0.0, 0.0, 0.0};
float Ki[3] = {1.0, 1.0, 1.0};
float Kd[3] = {0.0, 0.0, 0.0};
const float INTEGRAL_MAX = 200.0;

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

PIDState motor[3];

volatile long ticks[3] = {0, 0, 0};
unsigned long lastSampleTime = 0;

String inputBuffer = "";

// ---------------- ISR Encoder (x4) ----------------
void IRAM_ATTR encoderISR(void* arg) {
  int i = (int)(intptr_t)arg;  // índice 0..2

  bool A = digitalRead(ENC_A[i]);
  bool B = digitalRead(ENC_B[i]);

  // Misma lógica que ya usabas (x4)
  // Nota: funciona tanto si la interrupción viene de A como de B.
  if (A == B) ticks[i]--;
  else        ticks[i]++;
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
    motor[0].direction = (v == 1);
    motor[1].direction = (v == 1);
    motor[2].direction = (v == 1);
    lastCmdMs = millis();
    return;
  }

  // Setpoint: Sxx.x
  if (line[0] == 'S') {
    float sp = line.substring(1).toFloat();
    sp = constrain(sp, 0.0, 67.0);
    motor[0].setpointRPM = sp;
    motor[1].setpointRPM = sp;
    motor[2].setpointRPM = sp;
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

  for (int i = 0; i < 3; i++) {
    
    ledcAttach(IN1[i], PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(IN2[i], PWM_FREQ, PWM_RESOLUTION);
    ledcWrite(IN1[i], 0);
    ledcWrite(IN2[i], 0);
    
    pinMode(ENC_A[i], INPUT_PULLUP);
    pinMode(ENC_B[i], INPUT_PULLUP);
  
    attachInterruptArg(digitalPinToInterrupt(ENC_A[i]), encoderISR, (void*)(intptr_t)i, CHANGE);
    attachInterruptArg(digitalPinToInterrupt(ENC_B[i]), encoderISR, (void*)(intptr_t)i, CHANGE);
  
    motor[i].direction = true;
    motor[i].setpointRPM = 0.0;
  
  }

  lastCmdMs = millis();
  lastSampleTime = millis();
}

// ---------------- Loop ----------------
void loop() {
  readSerialLines();

  // Failsafe: si no llegan comandos, frena
  if (millis() - lastCmdMs > CMD_TIMEOUT_MS) {
    for(int i = 0; i < 3; i++) {
      motor[i].setpointRPM = 0.0;
      motor[i].errorSum = 0.0;  // evita windup si se queda sin comando 
    }
  }

  unsigned long now = millis();
  if (now - lastSampleTime >= SAMPLE_MS) {
  
    uint32_t dt_ms = now - lastSampleTime;
    float dt = dt_ms / 1000.0f;
  
    long dticks[3];
  
    // Lee y resetea ticks de los 3 motores de forma atómica
    portDISABLE_INTERRUPTS();
    dticks[0] = ticks[0]; ticks[0] = 0;
    dticks[1] = ticks[1]; ticks[1] = 0;
    dticks[2] = ticks[2]; ticks[2] = 0;
    portENABLE_INTERRUPTS();
  
    float v_mps[3];
  
    // Calcula PID + aplica PWM para cada motor
    for (int i = 0; i < 3; i++) {
      motor[i].currentRPM = calcularRPM(dticks[i], dt);
      motor[i].pwmPercent = computePID(motor[i], dt, Kp[i], Ki[i], Kd[i], INTEGRAL_MAX);
      setMotorPins(IN1[i], IN2[i], motor[i].pwmPercent, motor[i].direction);
  
      v_mps[i] = calcularVelocidadMPS(dticks[i], dt, motor[i].direction);
    }
  
    // UART packet: ID, seq, dt, t1, r1, t2, r2, t3, r3
    Serial.print(ESP_ID); Serial.print(",");
    Serial.print(seq++);  Serial.print(",");
    Serial.print(dt_ms);
  
    for (int i = 0; i < 3; i++) {
      Serial.print(",");
      Serial.print(dticks[i]);
      Serial.print(",");
      Serial.print(v_mps[i], 4);
    }
    Serial.println();
  

//    // Debug (opcional)
//    Serial.print("SP:");  Serial.print(motor.setpointRPM, 1);
//    Serial.print(" PV:"); Serial.print(motor.currentRPM, 1);
//    Serial.print(" PWM:");Serial.println(motor.pwmPercent, 1);

    // --- Telemetría para la Raspberry (una línea fácil de parsear) ---
    // Formato: ODOM,seq,t_ms,dt_ms,dticks,dir,rpm,v_mps
//    Serial.print("ODOM,");
//    Serial.print(seq++); Serial.print(","); // cantidad de paquetes enviados
//    Serial.print(now); Serial.print(","); // tiempo actual
//    Serial.print((unsigned long)(dt * 1000.0f)); Serial.print(","); // diferencia del tiempo previo al actual
//    Serial.print(tR); Serial.print(","); // ticks actuales
//    Serial.print(motor.direction ? 1 : 0); Serial.print(","); // dirección 
//    Serial.print(motor.currentRPM, 2); Serial.print(",");
//    Serial.println(v_mps, 4);

    lastSampleTime = now;
  }
}
