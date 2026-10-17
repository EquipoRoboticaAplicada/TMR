/*
  Control de velocidad de 3 motores DC con encoder mediante puentes IBT-2
  Plataforma: ESP32 (Arduino framework, core 3.x)
  LADO DERECHO (ESP_R)

  Conexiones para LADO DERECHO (Base):
    Motor   IN1(RPWM)  IN2(LPWM)   ENC_A   ENC_B
      0        14         13          4       5
      1        27         26         16      17
      2        23         25         18      19

  Comandos por Serial (buffer de línea):
    S<valor>    -> fija el setpoint en RPM de los 3 motores, con signo
                   (+ = avanzar adelante, - = retroceder).

  Trama de salida (cada SAMPLE_TIME ms), formato que espera connect.py:
    ESP_R,seq,rpm0,v0,rpm1,v1,rpm2,v2
*/

const char* ESP_ID = "ESP_R"; // connect.py identifica el puerto buscando "ESP_L"/"ESP_R"

// ---------- Pines LADO DERECHO ----------
const int IN1[3]   = {14, 27, 23};
const int IN2[3]   = {13, 26, 25};
const int ENC_A[3] = {4, 16, 18};
const int ENC_B[3] = {5, 17, 19};

// Pines de habilitación de cada IBT-2 (salidas digitales, HIGH = activo)
const int R_EN[3] = {21, 32, 15};
const int L_EN[3] = {22, 33, 2};

// ---------- PWM (LEDC) ----------
const int PWM_FREQ = 20000;   // 20 kHz, fuera del rango audible
const int PWM_RES  = 8;       // 8 bits -> 0-255

// ---------- Datos del motor ----------
const float GEAR_RATIO      = 262;   // relación de reducción de la caja
const int   PULSES_PER_REV  = 16;    // cuentas del encoder por vuelta del motor
const float PULSES_PER_OUTPUT_REV = PULSES_PER_REV * GEAR_RATIO;

// Diámetro de rueda: se usa para calcular m/s de los 3 motores.
const float WHEEL_DIAM_M = 0.17f;
const float WHEEL_CIRC_M = 3.14159265f * WHEEL_DIAM_M;

// ---------- Encoder (cuadratura completa x4, arreglado a 3 motores) ----------
volatile long   encoderCount[3] = {0, 0, 0};
volatile uint8_t lastEncoded[3] = {0, 0, 0};

void IRAM_ATTR isrEncoder(void* arg) {
  int i = (int)(intptr_t)arg;

  uint8_t MSB = digitalRead(ENC_A[i]);
  uint8_t LSB = digitalRead(ENC_B[i]);
  uint8_t encoded = (MSB << 1) | LSB;
  uint8_t sum = (lastEncoded[i] << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderCount[i]++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderCount[i]--;

  lastEncoded[i] = encoded;
}

// ---------- Variables de control PID (una por motor) ----------
double SP[3]         = {0.0, 0.0, 0.0};  // Setpoint en RPM del eje de salida
double PV[3]         = {0.0, 0.0, 0.0};  // Valor medido (RPM del eje de salida)
double errorPID[3]   = {0.0, 0.0, 0.0};
double lastError[3]  = {0.0, 0.0, 0.0};
double integral[3]   = {0.0, 0.0, 0.0};
double derivative[3] = {0.0, 0.0, 0.0};

// ---------- Ganancias PID ----------
double Kp[3] = {0.0, 0.0, 0.0};
double Ki[3] = {1.0, 1.0, 1.0};
double Kd[3] = {0.0, 0.0, 0.0};

int pwmOut[3] = {0, 0, 0};

// ---------- Tiempo de muestreo ----------
const unsigned long SAMPLE_TIME = 100; // ms
unsigned long lastSampleTime = 0;
long lastEncoderCount[3] = {0, 0, 0};

// ---------- Estado de comandos seriales ----------
unsigned long lastCmdMs = 0;
unsigned int seq = 0;
String inputBuffer = "";

// ---------------- Parseo de comandos ----------------
void handleLine(String line) {
  line.trim();
  line.toUpperCase();
  if (line.length() < 2) return;

  if (line[0] == 'S') {
    float value = line.substring(1).toFloat(); // viene con signo (+ adelante, - atras)
    for (int i = 0; i < 3; i++) SP[i] = value;
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

// ---------------- Control de motor ----------------
void setMotor(int idx, int pwm) {
  if (pwm >= 0) {
    ledcWrite(IN1[idx], pwm);
    ledcWrite(IN2[idx], 0);
  } else {
    ledcWrite(IN1[idx], 0);
    ledcWrite(IN2[idx], -pwm);
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);

  for (int i = 0; i < 3; i++) {
    ledcAttach(IN1[i], PWM_FREQ, PWM_RES);
    ledcAttach(IN2[i], PWM_FREQ, PWM_RES);
    ledcWrite(IN1[i], 0);
    ledcWrite(IN2[i], 0);

    pinMode(R_EN[i], OUTPUT);
    pinMode(L_EN[i], OUTPUT);
    digitalWrite(R_EN[i], HIGH);
    digitalWrite(L_EN[i], HIGH);

    pinMode(ENC_A[i], INPUT_PULLUP);
    pinMode(ENC_B[i], INPUT_PULLUP);
    attachInterruptArg(digitalPinToInterrupt(ENC_A[i]), isrEncoder, (void*)(intptr_t)i, CHANGE);
    attachInterruptArg(digitalPinToInterrupt(ENC_B[i]), isrEncoder, (void*)(intptr_t)i, CHANGE);
  }

  lastCmdMs      = millis();
  lastSampleTime = millis();
}

void loop() {
  readSerialLines();

  unsigned long now = millis();
  if (now - lastSampleTime >= SAMPLE_TIME) {
    float dt = (now - lastSampleTime) / 1000.0;
    lastSampleTime = now;

    long count[3];
    noInterrupts();
    count[0] = encoderCount[0];
    count[1] = encoderCount[1];
    count[2] = encoderCount[2];
    interrupts();

    for (int i = 0; i < 3; i++) {
      long deltaCount = count[i] - lastEncoderCount[i];
      lastEncoderCount[i] = count[i];

      // Velocidad en RPM del eje de salida
      double pulsesPerSec = deltaCount / dt;
      PV[i] = (pulsesPerSec / PULSES_PER_OUTPUT_REV) * 60.0;

      // ---- PID ----
      errorPID[i] = SP[i] - PV[i];
      integral[i] += errorPID[i] * dt;
      derivative[i] = (errorPID[i] - lastError[i]) / dt;
      lastError[i] = errorPID[i];

      double output = Kp[i] * errorPID[i] + Ki[i] * integral[i] + Kd[i] * derivative[i];

      // Anti-windup simple
      if (output > 255) {
        output = 255;
        integral[i] -= errorPID[i] * dt;
      } else if (output < -255) {
        output = -255;
        integral[i] -= errorPID[i] * dt;
      }

      pwmOut[i] = (int)output;
      setMotor(i, pwmOut[i]);
    }

    // ---- Trama de salida: ESP_R,seq,rpm0,v0,rpm1,v1,rpm2,v2
    Serial.print(ESP_ID); Serial.print(",");
    Serial.print(seq++);
    for (int i = 0; i < 3; i++) {
      float v_mps = (PV[i] / 60.0) * WHEEL_CIRC_M;
      Serial.print(",");
      Serial.print(PV[i], 2);
      Serial.print(",");
      Serial.print(v_mps, 4);
    }
    Serial.println();
  }
}
