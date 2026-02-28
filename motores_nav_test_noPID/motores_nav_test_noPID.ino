// =====================================================
// CONTROLADOR 2 MOTORES PROTOTIPO (SIN PID - VELOCIDAD CONSTANTE)
// =====================================================

const char* ESP_ID = "ESP_PROTO";

// --- PINES MOTOR IZQUIERDO (Original Motor 0) ---
const int IN1_L = 27; // RPWM
const int IN2_L = 14; // LPWM
const int ENC_A_L = 32;
const int ENC_B_L = 33;

// --- PINES MOTOR DERECHO (Original Motor 1) ---
const int IN1_R = 26; // RPWM
const int IN2_R = 13; // LPWM
const int ENC_A_R = 34;
const int ENC_B_R = 35;

// --- CONFIGURACIÓN PWM Y VELOCIDAD ---
#define PWM_FREQ       20000
#define PWM_RESOLUTION 10
const int PWM_MAX = (1 << PWM_RESOLUTION) - 1;

// ---> AJUSTA AQUÍ LA VELOCIDAD CONSTANTE (0.0 a 100.0 %) <---
const float FIXED_PWM_PERCENT = 40.0; 

// --- ESTADOS DE CONTROL ---
bool enable_motors = false; // Toggle general (GO / STOP)
bool dir_L = true;          // true = adelante, false = atrás
bool dir_R = true;          // true = adelante, false = atrás

// --- VARIABLES DE ODOMETRÍA (TICKS ABSOLUTOS) ---
volatile long ticks_L = 0;
volatile long ticks_R = 0;
volatile int8_t lastEnc_L = 0;
volatile int8_t lastEnc_R = 0;

uint32_t seq = 0;
unsigned long lastSampleTime = 0;
const unsigned long SAMPLE_MS = 100; // Refresco de 10Hz

String inputBuffer = "";

// ---------------- ISR Encoder x4 (lookup table robusta) ----------------
static const int8_t lookup[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

void IRAM_ATTR isr_L() {
  bool A = digitalRead(ENC_A_L);
  bool B = digitalRead(ENC_B_L);
  int8_t state = ((int8_t)A << 1) | (int8_t)B;
  ticks_L += lookup[(lastEnc_L << 2) | state];
  lastEnc_L = state;
}

void IRAM_ATTR isr_R() {
  bool A = digitalRead(ENC_A_R);
  bool B = digitalRead(ENC_B_R);
  int8_t state = ((int8_t)A << 1) | (int8_t)B;
  ticks_R += lookup[(lastEnc_R << 2) | state];
  lastEnc_R = state;
}

// ---------------- Utilidades de Control ----------------
void setMotorOpenLoop(int pin1, int pin2, float percent, bool forward) {
  if (percent < 1.0f) { // Apagar si el PWM es menor al 1%
    ledcWrite(pin1, 0);
    ledcWrite(pin2, 0);
    return;
  }
  
  int pwmValue = (int)((percent / 100.0f) * PWM_MAX);
  pwmValue = constrain(pwmValue, 0, PWM_MAX);

  if (forward) {
    ledcWrite(pin1, pwmValue);
    ledcWrite(pin2, 0);
  } else {
    ledcWrite(pin1, 0);
    ledcWrite(pin2, pwmValue);
  }
}

// ---------------- Parseo de comandos Seriales ----------------
void handleLine(String line) {
  line.trim();
  line.toUpperCase();
  
  // Comandos ultra simples
  if (line == "STOP") enable_motors = false;
  else if (line == "GO") enable_motors = true;
  else if (line == "DL1") dir_L = true;   // Dirección Izquierda Adelante
  else if (line == "DL0") dir_L = false;  // Dirección Izquierda Atrás
  else if (line == "DR1") dir_R = true;   // Dirección Derecha Adelante
  else if (line == "DR0") dir_R = false;  // Dirección Derecha Atrás
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

  // Configurar PWM
  ledcAttach(IN1_L, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(IN2_L, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(IN1_R, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(IN2_R, PWM_FREQ, PWM_RESOLUTION);

  // Configurar Encoders
  pinMode(ENC_A_L, INPUT_PULLUP); pinMode(ENC_B_L, INPUT_PULLUP);
  pinMode(ENC_A_R, INPUT_PULLUP); pinMode(ENC_B_R, INPUT_PULLUP);

  // Estado inicial encoders
  lastEnc_L = ((int8_t)digitalRead(ENC_A_L) << 1) | (int8_t)digitalRead(ENC_B_L);
  lastEnc_R = ((int8_t)digitalRead(ENC_A_R) << 1) | (int8_t)digitalRead(ENC_B_R);

  // Interrupciones
  attachInterrupt(digitalPinToInterrupt(ENC_A_L), isr_L, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_L), isr_L, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A_R), isr_R, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_R), isr_R, CHANGE);

  lastSampleTime = millis();
}

// ---------------- Loop ----------------
void loop() {
  readSerialLines();

  // Mover motores a velocidad constante si están habilitados
  if (enable_motors) {
    setMotorOpenLoop(IN1_L, IN2_L, FIXED_PWM_PERCENT, dir_L);
    setMotorOpenLoop(IN1_R, IN2_R, FIXED_PWM_PERCENT, dir_R);
  } else {
    setMotorOpenLoop(IN1_L, IN2_L, 0.0, dir_L);
    setMotorOpenLoop(IN1_R, IN2_R, 0.0, dir_R);
  }

  // Telemetría de Odometría a 10Hz
  unsigned long now = millis();
  if (now - lastSampleTime >= SAMPLE_MS) {
    uint32_t dt_ms = now - lastSampleTime;

    // Bloque atómico seguro para leer ticks
    portDISABLE_INTERRUPTS();
    long t_L = ticks_L;
    long t_R = ticks_R;
    portENABLE_INTERRUPTS();

    // Paquete UART simple: ID, seq, dt_ms, ticks_Izquierdos, ticks_Derechos
    Serial.print(ESP_ID); Serial.print(",");
    Serial.print(seq++); Serial.print(",");
    Serial.print(dt_ms); Serial.print(",");
    Serial.print(t_L); Serial.print(",");
    Serial.println(t_R);

    lastSampleTime = now;
  }
}
