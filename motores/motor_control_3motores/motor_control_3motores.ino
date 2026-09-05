/*
  Control de velocidad de 3 motores DC con encoder mediante puentes IBT-2
  Plataforma: ESP32 (Arduino framework, core 3.x)

  Fusión de dos programas:
    - Control PID (motor_control_encoder_ibt2_2): salida PID bidireccional
      (-255..255), ISR de cuadratura x4 y anti-windup, ahora en arreglos [3].
    - Comunicación serial (motores_der.ino): buffer de línea + parseo de
      comandos "D"/"S" y trama de salida "ID,seq,...".

  Conexiones (ajusta si tu cableado difiere):
    Motor   IN1(RPWM)  IN2(LPWM)   ENC_A   ENC_B
      0        14         13          4       5
      1        27         26         16      17
      2        23         25         18      19

  Nota: al igual que en motores_der.ino, se asume que R_EN/L_EN de los
  IBT-2 están habilitados por hardware (atados a VCC). Si tu montaje usa
  pines de enable independientes, agrégalos y ponlos en HIGH en setup(),
  como hacía motor_control_encoder_ibt2_2 con R_EN/L_EN.

  --------------------------------------------------------------------
  Comandos por Serial (buffer de línea de motores_der.ino, dirección
  como en motor_control_encoder_ibt2_2):
    S<valor>    -> fija el setpoint en RPM de los 3 motores, con signo
                   (+ = un sentido, - = el sentido contrario). No hay
                   comando de dirección separado ni valores binarios 0/1;
                   el signo del propio valor es lo que define el sentido,
                   igual que en motor_control_encoder_ibt2_2.

  A partir de ahí el control es exactamente el de
  motor_control_encoder_ibt2_2 (PID con salida con signo, sin mapeo a
  porcentaje ni PWM mínimo).

  Trama de salida (cada SAMPLE_TIME ms), formato exacto que espera
  connect.py (_parse_esp_line_m, que exige 4 campos separados por coma):
    ESP_R,seq,rpm0,v0
  ESP_ID se dejó como "ESP_R" porque connect.py identifica el puerto
  buscando líneas que empiecen con "ESP_L" o "ESP_R". El PID de los 3
  motores sigue corriendo igual; sólo se reporta la telemetría (rpm y
  m/s) del motor 0, que es lo único que connect.py lee.
*/

const char* ESP_ID = "ESP_R"; // connect.py identifica el puerto buscando "ESP_L"/"ESP_R" al inicio de línea

// ---------- Pines (uno por motor) ----------
const int IN1[3] = {14, 27, 23};   // RPWM
const int IN2[3] = {13, 26, 25};   // LPWM
const int ENC_A[3] = {4, 16, 18};
const int ENC_B[3] = {5, 17, 19};

// ---------- PWM (LEDC) ----------
// Mismos parámetros que motor_control_encoder_ibt2_2 para conservar el
// rango de salida del PID (0-255) tal cual.
const int PWM_FREQ = 20000;   // 20 kHz, fuera del rango audible
const int PWM_RES  = 8;       // 8 bits -> 0-255

// ---------- Datos del motor ----------
const float GEAR_RATIO      = 262;   // relación de reducción de la caja
const int   PULSES_PER_REV  = 16;    // cuentas del encoder por vuelta del motor
const float PULSES_PER_OUTPUT_REV = PULSES_PER_REV * GEAR_RATIO;

// Diámetro de rueda: sólo se usa para reportar m/s del motor 0, como
// espera connect.py (_parse_esp_line_m). Ajusta al diámetro real.
const float WHEEL_DIAM_M = 0.17f;
const float WHEEL_CIRC_M = 3.14159265f * WHEEL_DIAM_M;

// ---------- Encoder (cuadratura completa x4, arreglado a 3 motores) ----------
// Misma lógica de decodificación (MSB/LSB + patrón "sum") que
// motor_control_encoder_ibt2_2, usando attachInterruptArg para saber
// a qué motor pertenece cada interrupción (como en motores_der.ino).
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

// ---------- Ganancias PID (una por motor, mismo valor por defecto que
// motor_control_encoder_ibt2_2; re-sintonizar si hace falta) ----------
double Kp[3] = {0.0, 0.0, 0.0};
double Ki[3] = {1.0, 1.0, 1.0};
double Kd[3] = {0.0, 0.0, 0.0};

int pwmOut[3] = {0, 0, 0};

// ---------- Tiempo de muestreo ----------
const unsigned long SAMPLE_TIME = 100; // ms
unsigned long lastSampleTime = 0;
long lastEncoderCount[3] = {0, 0, 0};

// ---------- Estado de comandos seriales (motores_der.ino) ----------
unsigned long lastCmdMs = 0;
unsigned int seq = 0;
String inputBuffer = "";

// ---------------- Parseo de comandos (buffer de motores_der.ino, valor
// con signo como en motor_control_encoder_ibt2_2) ----------------
void handleLine(String line) {
  line.trim();
  line.toUpperCase();
  if (line.length() < 2) return;

  if (line[0] == 'S') {
    float value = line.substring(1).toFloat(); // ya viene con signo
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

// ---------------- Control (misma lógica que motor_control_encoder_ibt2_2) ----------------
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

    pinMode(ENC_A[i], INPUT_PULLUP);
    pinMode(ENC_B[i], INPUT_PULLUP);
    attachInterruptArg(digitalPinToInterrupt(ENC_A[i]), isrEncoder, (void*)(intptr_t)i, CHANGE);
    attachInterruptArg(digitalPinToInterrupt(ENC_B[i]), isrEncoder, (void*)(intptr_t)i, CHANGE);
  }

  lastCmdMs      = millis();
  lastSampleTime = millis();

  // Serial.println("ID,seq,SP0,PV0,ERR0,PWM0,SP1,PV1,ERR1,PWM1,SP2,PV2,ERR2,PWM2");
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

      // ---- PID (idéntico a motor_control_encoder_ibt2_2) ----
      errorPID[i] = SP[i] - PV[i];
      integral[i] += errorPID[i] * dt;
      derivative[i] = (errorPID[i] - lastError[i]) / dt;
      lastError[i] = errorPID[i];

      double output = Kp[i] * errorPID[i] + Ki[i] * integral[i] + Kd[i] * derivative[i];

      // Anti-windup simple: satura la salida y limita el término integral
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

    // ---- Trama de salida: formato que espera connect.py (_parse_esp_line_m)
    // ESP_ID,seq,rpm0,v0 — SIEMPRE 4 campos (el parser descarta cualquier
    // línea que no tenga exactamente 4). Sólo se reporta el motor 0; el
    // control PID de los 3 motores sigue corriendo internamente igual.
    float v0_mps = (PV[0] / 60.0) * WHEEL_CIRC_M;

    Serial.print(ESP_ID); Serial.print(",");
    Serial.print(seq++); Serial.print(",");
    Serial.print(PV[0]); Serial.print(",");
    Serial.println(v0_mps, 4);
  }
}
