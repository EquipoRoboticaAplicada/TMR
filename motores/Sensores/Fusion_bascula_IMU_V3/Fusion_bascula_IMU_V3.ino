#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303DLH_Mag.h>
#include <Adafruit_LSM303_Accel.h>
#include "HX711.h"

/* ===== I2C ===== */
#define SDA_PIN 21
#define SCL_PIN 22

/* ===== HX711 ===== */
#define DT 26
#define SCK 25
HX711 celda;

/* ===== Sensor ===== */
Adafruit_LSM303DLH_Mag_Unified   mag   = Adafruit_LSM303DLH_Mag_Unified(12345);
Adafruit_LSM303_Accel_Unified    accel = Adafruit_LSM303_Accel_Unified(54321);

/* ===== Offsets ===== */
float offsetX = -49.54;
float offsetY = -37.23;
float offsetZ = -55.32;

/* ===== Filtro ===== */
const float alpha = 0.15;

/* ===== Variables ===== */
float pitchF   = 0;
float axRealF  = 0;
float velocity = 0;

/* ===== Peso ===== */
float peso_actual = 0;

/* ===== Máquina de estado báscula ===== */
bool  midiendo       = false;
int   muestras       = 30;
int   contador       = 0;
float suma           = 0;
unsigned long lastSampleTime = 0;
const int sampleInterval     = 15;
bool  segundaMedicion = false;
float peso1           = 0;

/* ===== Tiempo ===== */
unsigned long lastTime  = 0;
unsigned long lastPrint = 0;

/* ===== Watchdog I2C ===== */
static uint8_t zeroCount = 0;

/* ========================= */
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
  //Serial.println("I2C reiniciado.");
}

/* ========================= */
String detectTerrain(float pitch) {
  if      (pitch >  15) return "UP";
  else if (pitch < -15) return "DOWN";
  else                  return "FLAT";
}

/* ========================= */
void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  imuInit();

  celda.begin(DT, SCK);
  celda.set_scale(682.f);
  celda.tare();

  lastTime = millis();
  //Serial.println("Presiona ENTER para medir peso");
}

/* ========================= */
void loop() {

  /* ===== DETECTAR ENTER ===== */
  if (Serial.available()) {
    while (Serial.available()) Serial.read();
    midiendo       = true;
    contador       = 0;
    suma           = 0;
    segundaMedicion = false;
  }

  /* ===== MAQUINA DE ESTADO BASCULA ===== */
  if (midiendo) {
    unsigned long now = millis();
    if (now - lastSampleTime >= sampleInterval) {
      lastSampleTime = now;
      suma += celda.get_units(1);
      contador++;

      if (contador >= muestras) {
        if (!segundaMedicion) {
          peso1           = suma / muestras;
          suma            = 0;
          contador        = 0;
          segundaMedicion = true;
        } else {
          float peso2   = suma / muestras;
          peso_actual   = (peso1 + peso2) / 2.0;
          //Serial.print("Peso actualizado: ");
          Serial.println(peso_actual, 2);
          midiendo = false;
        }
      }
    }
  }

  /* ===== IMU ===== */
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  sensors_event_t accel_event;
  sensors_event_t mag_event;

  // Leer con verificación de fallo I2C
  bool accelOk = accel.getEvent(&accel_event);
  bool magOk   = mag.getEvent(&mag_event);

  float ax = accel_event.acceleration.x;
  float ay = accel_event.acceleration.y;
  float az = accel_event.acceleration.z;

  // Detectar congelamiento: 3 lecturas seguidas en cero
  if (ax == 0 && ay == 0 && az == 0) {
    zeroCount++;
    if (zeroCount >= 3) {
      i2cReset();
      zeroCount = 0;
    }
    return; // no calcular con datos inválidos
  }
  zeroCount = 0;

  float pitch    = atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  float pitchRad = pitch * PI / 180.0;

  float heading  = atan2(mag_event.magnetic.y, mag_event.magnetic.x) * 180.0 / PI;
  if (heading < 0) heading += 360;

  pitchF   = alpha * pitch + (1 - alpha) * pitchF;

  float ax_real = ax - 9.81 * sin(pitchRad);
  axRealF  = alpha * ax_real + (1 - alpha) * axRealF;

  velocity += axRealF * dt;
  if (abs(axRealF) < 0.05) velocity *= 0.9;

  String terrain = detectTerrain(pitchF);

  /* ===== PRINT SIN BLOQUEAR ===== */
  Serial.println("Sensores");
  
    if (millis() - lastPrint >= 100) {
    lastPrint = millis();
    Serial.print(pitchF);    Serial.print(",");
    Serial.print(heading);   Serial.print(",");
    Serial.print(velocity);  Serial.print(",");
    Serial.print(terrain);   Serial.print(",");
    Serial.println(peso_actual);

  }
  delay(20);
}