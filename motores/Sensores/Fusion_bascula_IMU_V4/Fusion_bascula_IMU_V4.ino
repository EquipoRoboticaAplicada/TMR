#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303DLH_Mag.h>
#include <Adafruit_LSM303_Accel.h>
#include "HX711.h"
#include <math.h>
#include <ESP32Servo.h>

/* ===== I2C ===== */
#define SDA_PIN 21
#define SCL_PIN 22

/* ===== HX711 ===== */
#define DT 26
#define SCK 25
HX711 celda;
float calibration_factor = 687;

/* ===== Sensores IMU ===== */
Adafruit_LSM303DLH_Mag_Unified   mag   = Adafruit_LSM303DLH_Mag_Unified(12345);
Adafruit_LSM303_Accel_Unified    accel = Adafruit_LSM303_Accel_Unified(54321);

/* ===== Calibración magnetómetro (hard/soft iron) ===== */
const float MAG_MIN_X = -40.18,  MAG_MAX_X = 49.45;
const float MAG_MIN_Y = -82.55,  MAG_MAX_Y = 30.73;
const float OFFSET_X  = (MAG_MAX_X + MAG_MIN_X) / 2.0f;
const float OFFSET_Y  = (MAG_MAX_Y + MAG_MIN_Y) / 2.0f;
const float RANGE_X   = (MAG_MAX_X - MAG_MIN_X) / 2.0f;
const float RANGE_Y   = (MAG_MAX_Y - MAG_MIN_Y) / 2.0f;
const float AVG_RANGE = (RANGE_X + RANGE_Y) / 2.0f;
const float SCALE_X   = AVG_RANGE / RANGE_X;
const float SCALE_Y   = AVG_RANGE / RANGE_Y;

/* ===== Filtro EMA ===== */
const float alpha = 0.15;

/* ===== Estado IMU (variables de filtro, globales) ===== */
float pitchF     = 0;
float axRealF    = 0;
float velocity   = 0;
float headingSin = 0;
float headingCos = 0;

/* ===== Estado de la báscula ===== */
float peso_actual = 0;

/* ===== Tiempo ===== */
unsigned long lastTime  = 0;
unsigned long lastPrint = 0;

/* ===== Watchdog I2C ===== */
static uint8_t zeroCount = 0;

/* ===== Servos Brazo ===== */
Servo servo1;       // Pin 10 
Servo servo2;       // Pin 9 
Servo servoGripper; // Pin 13 (Añadido para el control del gripper)

int posicionActual = 180; // Empezamos en el descanso con ambos servos apuntando hacia arriba (180°)

/* ============================================================
   Struct para agrupar los resultados del pipeline IMU
   ============================================================ */
struct DatosIMU {
  float pitch;
  float headingRaw;
  float heading;
  float velocity;
  String terrain;
};

/* ============================================================
   Funciones auxiliares existentes
   ============================================================ */
void imuInit() {
  accel.begin();
  mag.begin();
}

void i2cReset() {
  Wire.end();
  delay(50);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  delay(50);
  imuInit();
}

String detectTerrain(float pitch) {
  if      (pitch >  15) return "UP";
  else if (pitch < -15) return "DOWN";
  else                  return "FLAT";
}

void arm_down(){
  int destino = 0; 
  
  while (posicionActual != destino) {
    if (posicionActual < destino) {
      posicionActual++;
    } else {
      posicionActual--;
    }

    servo1.write(posicionActual);
    servo2.write(180 - posicionActual);
    delay(15);
  }
}

void arm_up(){
  int destino = 180; 
  
  while (posicionActual != destino) {
    if (posicionActual < destino) {
      posicionActual++;
    } else {
      posicionActual--;
    }

    servo1.write(posicionActual);
    servo2.write(180 - posicionActual);
    delay(15);
  }
}

/* ============================================================
   FUNCIÓN: leerBascula
   Lee la celda de carga cuando llega un comando serial.
   Actualiza la variable global peso_actual.
   ============================================================ */
void leerBascula() {
  if (!Serial.available()) return;

  // Vaciar buffer serial
  while (Serial.available()) Serial.read();

  if (!celda.is_ready()) return;

  float peso = celda.get_units(10);

  if (peso < 0.0) {
    celda.tare();
    peso = celda.get_units(10);
  } else if (abs(peso) < 0.5) {
    peso = 0.0;   // filtrar ruido residual
  }

  peso_actual = peso;
}

/* ============================================================
   FUNCIÓN: leerAcelerometro
   Lee los ejes X,Y,Z del acelerómetro.
   Devuelve false si la lectura es inválida (todos ceros).
   ============================================================ */
bool leerAcelerometro(float &ax, float &ay, float &az) {
  sensors_event_t accel_event;
  accel.getEvent(&accel_event);

  ax = accel_event.acceleration.x;
  ay = accel_event.acceleration.y;
  az = accel_event.acceleration.z;

  // Watchdog: detectar sensor colgado
  if (ax == 0 && ay == 0 && az == 0) {
    zeroCount++;
    if (zeroCount >= 3) {
      i2cReset();
      zeroCount = 0;
    }
    return false;  // datos inválidos
  }

  zeroCount = 0;
  return true;
}

/* ============================================================
   FUNCIÓN: calcularPitch
   Calcula el pitch en grados a partir de los ejes del acelerómetro
   y lo suaviza con un filtro EMA sobre la variable global pitchF.
   ============================================================ */
float calcularPitch(float ax, float ay, float az) {
  float pitch = atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  pitchF = alpha * pitch + (1 - alpha) * pitchF;
  return pitch;
}

/* ============================================================
   FUNCIÓN: calcularHeading
   Lee el magnetómetro, aplica calibración hard/soft iron y
   filtra el heading sobre sin/cos para evitar el salto 359°→0°.
   Devuelve el heading crudo (headingRaw) y el filtrado (heading).
   ============================================================ */
void calcularHeading(float &headingRaw, float &heading) {
  sensors_event_t mag_event;
  mag.getEvent(&mag_event);

  // Calibración hard/soft iron
  float cal_x = (mag_event.magnetic.x - OFFSET_X) * SCALE_X;
  float cal_y = (mag_event.magnetic.y - OFFSET_Y) * SCALE_Y;

  headingRaw = atan2(cal_y, cal_x) * (180.0f / M_PI);
  if (headingRaw < 0) headingRaw += 360.0f;

  // Filtro EMA sobre sin/cos (evita discontinuidad angular)
  float headingRad = headingRaw * PI / 180.0f;
  headingSin = alpha * sin(headingRad) + (1 - alpha) * headingSin;
  headingCos = alpha * cos(headingRad) + (1 - alpha) * headingCos;

  heading = atan2(headingSin, headingCos) * (180.0f / M_PI);
  if (heading < 0) heading += 360.0f;
}

/* ============================================================
   FUNCIÓN: actualizarVelocidad
   Estima la velocidad integrando la aceleración lineal (sin
   componente de gravedad). Aplica amortiguamiento cuando la
   aceleración es casi nula para evitar drift.
   ============================================================ */
void actualizarVelocidad(float ax, float pitchRad, float dt) {
  float ax_real = ax - 9.81f * sin(pitchRad);
  axRealF = alpha * ax_real + (1 - alpha) * axRealF;

  velocity += axRealF * dt;
  if (abs(axRealF) < 0.05) velocity *= 0.9;  // amortiguamiento anti-drift
}

/* ============================================================
   FUNCIÓN: procesarIMU
   Orquesta el pipeline completo de la IMU:
     1. Lee acelerómetro (con watchdog)
     2. Calcula pitch y lo filtra
     3. Calcula heading y lo filtra
     4. Actualiza velocidad
     5. Detecta terreno
   Devuelve un struct DatosIMU con todos los resultados,
   y valid=false si el acelerómetro devolvió datos inválidos.
   ============================================================ */
bool procesarIMU(DatosIMU &datos, float dt) {
  float ax, ay, az;

  if (!leerAcelerometro(ax, ay, az)) return false;

  float pitch    = calcularPitch(ax, ay, az);
  float pitchRad = pitchF * PI / 180.0;

  float headingRaw, heading;
  calcularHeading(headingRaw, heading);

  actualizarVelocidad(ax, pitchRad, dt);

  datos.pitch      = pitchF;
  datos.headingRaw = headingRaw;
  datos.heading    = heading;
  datos.velocity   = velocity;
  datos.terrain    = detectTerrain(pitchF);

  return true;
}

/* ============================================================
   FUNCIÓN: imprimirDatos
   Envía los datos por serial en formato CSV cada 100 ms.
   Formato: sensores,pitch,headingRaw,heading,velocity,terrain,peso
   ============================================================ */
void imprimirDatos(const DatosIMU &datos) {
  if (millis() - lastPrint < 100) return;
  lastPrint = millis();

  Serial.print("sensores");        Serial.print(",");
  Serial.print(datos.pitch);       Serial.print(",");
  Serial.print(datos.headingRaw);  Serial.print(",");
  Serial.print(datos.heading);     Serial.print(",");
  Serial.print(datos.velocity);    Serial.print(",");
  Serial.print(datos.terrain);     Serial.print(",");
  Serial.print(peso_actual);       Serial.println();
}

/* ============================================================
   SETUP
   ============================================================ */
void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  imuInit();

  celda.begin(DT, SCK);
  celda.set_scale(calibration_factor);
  celda.tare();

   // ----- Configuración de Servos para ESP32 -----
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  servoGripper.setPeriodHertz(50);

  servo1.attach(32, 500, 2500); 
  servo2.attach(33, 500, 2500);  
  servoGripper.attach(14, 500, 2500); 

  // Posición inicial
  servo1.write(posicionActual);
  servo2.write(180 - posicionActual);
  servoGripper.write(140); // El gripper empieza abierto a 140°

  lastTime = millis();
}

/* ============================================================
   LOOP  —  pipeline en 4 pasos claros
   ============================================================ */
void loop() {

    if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim(); // Se limpian posibles retornos de carro invisibles

    if (input == "B0") {
      arm_down();
      delay(500); 
      servoGripper.write(175);
      arm_up();
      delay(500);
      servoGripper.write(140);
      delay(500);
      
      // 1. Leer báscula si llega un comando serial
      leerBascula();
      
      lastTime = millis();

    } else {
      // --- RUTINA ORIGINAL DE LA BÁSCULA ---
      midiendo       = true;
      contador       = 0;
      suma           = 0;
      segundaMedicion = false;
    }

  // 2. Calcular dt para la integración de velocidad
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0f;
  lastTime = now;

  // 3. Procesar IMU (pitch, heading, velocidad, terreno)
  DatosIMU datos;
  if (!procesarIMU(datos, dt)) return;  // sensor colgado, skip

  // 4. Transmitir datos por serial (sin bloquear)
  imprimirDatos(datos);

  delay(20);
}
