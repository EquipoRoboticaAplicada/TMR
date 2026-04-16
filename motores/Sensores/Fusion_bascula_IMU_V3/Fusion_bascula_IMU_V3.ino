#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303DLH_Mag.h>
#include <Adafruit_LSM303_Accel.h>
#include "HX711.h"
#include <ESP32Servo.h> // Librería necesaria para el control de servos en placas ESP32

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

/* ===== Variables IMU ===== */
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

/* ===== Servos Brazo ===== */
Servo servo1;       // Pin 10 
Servo servo2;       // Pin 9 
Servo servoGripper; // Pin 13 (Añadido para el control del gripper)

int posicionActual = 180; // Empezamos en el descanso con ambos servos apuntando hacia arriba (180°)

/* ========================= bv*/
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

/* ========================= */
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

  // ----- Configuración de Servos para ESP32 -----
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  servoGripper.setPeriodHertz(50);

  servo1.attach(10, 500, 2500); 
  servo2.attach(9, 500, 2500);  
  servoGripper.attach(13, 500, 2500); 

  // Posición inicial
  servo1.write(posicionActual);
  servo2.write(180 - posicionActual);
  servoGripper.write(140); // El gripper empieza abierto a 140°

  lastTime = millis();
}

/* ========================= */
void loop() {

  /* ===== DETECTAR COMANDOS SERIALES ===== */
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim(); // Se limpian posibles retornos de carro invisibles

    if (input == "B0") {
      arm_down();
      delay(500); // Pequeña pausa para asegurar que el movimiento se complete
      // Después de que ambos servos llegan a 0°, se cierra el gripper
      servoGripper.write(175);
      arm_up();
      
      // Actualizar lastTime para evitar picos de velocidad de la IMU 
      // generados por el tiempo pausado durante la instrucción delay(15)
      lastTime = millis();

    } else {
      // --- RUTINA ORIGINAL DE LA BÁSCULA ---
      midiendo       = true;
      contador       = 0;
      suma           = 0;
      segundaMedicion = false;
    }
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
  bool accelOk = accel.getEvent(&accel_event);
  bool magOk   = mag.getEvent(&mag_event);
  
  float ax = accel_event.acceleration.x;
  float ay = accel_event.acceleration.y;
  float az = accel_event.acceleration.z;
  
  if (ax == 0 && ay == 0 && az == 0) {
    zeroCount++;
    if (zeroCount >= 3) {
      i2cReset();
      zeroCount = 0;
    }
    return;
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
  if (millis() - lastPrint >= 100) {
    lastPrint = millis();
    Serial.print("sensores");     Serial.print(",");
    Serial.print(pitchF);         Serial.print(",");
    Serial.print(heading);        Serial.print(",");
    Serial.print(velocity);       Serial.print(",");
    Serial.print(terrain);        Serial.print(",");
    Serial.print(peso_actual);    Serial.println();
  }
  delay(20);
}