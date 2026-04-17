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
//#define IMU_DESCONECTADO //  ← comentar esta línea para re-activar el IMU

/* ===== HX711 ===== */
#define DT 26
#define SCK 25
HX711 celda;
float calibration_factor = 687;

/* ===== Sensores IMU ===== */
Adafruit_LSM303DLH_Mag_Unified   mag   = Adafruit_LSM303DLH_Mag_Unified(12345);
Adafruit_LSM303_Accel_Unified    accel = Adafruit_LSM303_Accel_Unified(54321);

/*
Mag Minimums: -69.73  -59.09  -101.43
Mag Maximums: 65.27  55.82  48.06
*/

/* ===== Offsets ===== */
const float MAG_MIN_X = -69.73; // -40.18
const float MAG_MAX_X =  65.27; // 49.45
const float MAG_MIN_Y = -59.09; // -82.55
const float MAG_MAX_Y =   55.82; // 30.73
float offsetZ = -55.32;// -55.32 -23.325

// Para calcular offsets para arreglar distorsion de hard iron & soft iron
const float OFFSET_X  = (MAG_MAX_X + MAG_MIN_X) / 2.0f;
const float OFFSET_Y  = (MAG_MAX_Y + MAG_MIN_Y) / 2.0f;
const float RANGE_X   = (MAG_MAX_X - MAG_MIN_X) / 2.0f;
const float RANGE_Y   = (MAG_MAX_Y - MAG_MIN_Y) / 2.0f;
const float AVG_RANGE = (RANGE_X + RANGE_Y) / 2.0f;
const float SCALE_X   = AVG_RANGE / RANGE_X;
const float SCALE_Y   = AVG_RANGE / RANGE_Y;

/* ===== Filtro EMA ===== */
const float alpha = 0.15;

/* ===== Estado IMU ===== */
float pitchF     = 0;
float axRealF    = 0;
float velocity   = 0;
float headingSin = 0;
float headingCos = 0;

/* ===== Peso ===== */
float peso_actual = 0;

/* ===== Tiempo ===== */
unsigned long lastTime  = 0;
unsigned long lastPrint = 0;

/* ===== Watchdog I2C ===== */
static uint8_t zeroCount = 0;

/* ===== Servos Brazo ===== */ /////////////////nuevooooooooooooooooooooooooooooooo
Servo servo1;
Servo servo2;
Servo servoGripper;
Servo servoBox; 
int posicionActual = 180;

/* ============================================================
   Struct: resultados del pipeline IMU
   ============================================================ */
struct DatosIMU {
  float pitch;
  float headingRaw;
  float heading;
  float velocity;
  String terrain;
}; //                     ---------------------------nuevo---------------------------

/* ============================================================
   Funciones de bajo nivel
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
  //Serial.println("I2C reiniciado.");
}

String detectTerrain(float pitch) {
  // -3.30 en cero
  if      (pitch >  5) return "UP";
  else if (pitch < -5) return "DOWN";
  else                  return "FLAT";
}

void arm_down() { // ---------------------------------------nuevo------------------
  int destino = 20;
  while (posicionActual != destino) {
    posicionActual += (posicionActual < destino) ? 1 : -1;
    servo1.write(posicionActual);
    servo2.write(180 - posicionActual);
    delay(15);
  }
} // -----------------------------------------------------------nuevo----------------------

void arm_up() { // -------------------------------------- nuevo--------------
  int destino = 180;
  while (posicionActual != destino) {
    posicionActual += (posicionActual < destino) ? 1 : -1;
    servo1.write(posicionActual);
    servo2.write(180 - posicionActual);
    delay(15);
  }
} // ----------------------------------------------------  nuevo -----------------

void box_open() {
  servoBox.write(110); 
  delay(5000); 
  servoBox.write(20);
}

/* ============================================================
   FUNCIÓN: leerPesoDirecto
   Lee la celda sin depender del buffer serial.
   Retorna el peso medido (o el último valor si la celda no está lista).
   FIX BUG 3: reemplaza leerBascula() que era código muerto
              porque el buffer ya estaba consumido.
   ============================================================ */
float leerPesoDirecto() {
  if (!celda.is_ready()) return peso_actual;  // conservar último valor

  float peso = celda.get_units(10);

  if (peso < 0.0) {
    celda.tare();
    peso = celda.get_units(10);
  } else if (abs(peso) < 0.5) {
    peso = 0.0;  // filtrar ruido residual
  }

  return peso;
}

/* ============================================================
   FUNCIÓN: manejarComandoSerial
   Lee un comando del puerto serial y ejecuta la acción
   correspondiente. Retorna true si se procesó un comando.
   FIX BUG 2: elimina referencias a variables no declaradas
              (midiendo, contador, suma, segundaMedicion).
   ============================================================ */
bool manejarComandoSerial() {
  if (!Serial.available()) return false;

  String input = Serial.readStringUntil('\n');
  input.trim();

  if (input == "B0") {
    // Secuencia: bajar → cerrar gripper → MEDIR → subir → abrir
    // FIX BUG 4: peso se lee con gripper cerrado, ANTES de abrir
    arm_down();
    delay(500);
    servoGripper.write(175);    // cerrar gripper
    delay(300);
    arm_up();
    delay(500);
    servoGripper.write(135);    // abrir gripper
    delay(500);
    peso_actual = leerPesoDirecto();  // medir mientras sostiene el objeto
    lastTime = millis();        // resetear dt para no acumular el tiempo del movimiento
  } else if (input == "BOPEN") {
    box_open();  
  } else {
    // Cualquier otro comando → leer peso directamente
    peso_actual = leerPesoDirecto();
  }

  return true;
}

/* ============================================================
   FUNCIÓN: leerAcelerometro
   Lee los ejes X,Y,Z. Devuelve false si la lectura es inválida.
   ============================================================ */
bool leerAcelerometro(float &ax, float &ay, float &az) {
  sensors_event_t accel_event;
  accel.getEvent(&accel_event);

  ax = accel_event.acceleration.x;
  ay = accel_event.acceleration.y;
  az = accel_event.acceleration.z;

  if (ax == 0 && ay == 0 && az == 0) {
    zeroCount++;
    if (zeroCount >= 3) {
      i2cReset();
      zeroCount = 0;
    }
    return false;
  }

  zeroCount = 0;
  return true;
}

/* ============================================================
   FUNCIÓN: calcularPitch
   Calcula y filtra el pitch con EMA. Actualiza pitchF global.
   ============================================================ */
float calcularPitch(float ax, float ay, float az) {
  float pitch = atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  pitchF = alpha * pitch + (1 - alpha) * pitchF;
  return pitch;
}

/* ============================================================
   FUNCIÓN: calcularHeading
   Lee el magnetómetro, calibra y filtra. Actualiza headingSin/Cos.
   ============================================================ */
void calcularHeading(float &headingRaw, float &heading) {
  sensors_event_t mag_event;
  mag.getEvent(&mag_event);

  float cal_x = (mag_event.magnetic.x - OFFSET_X) * SCALE_X;
  float cal_y = (mag_event.magnetic.y - OFFSET_Y) * SCALE_Y;

  headingRaw = atan2(cal_y, cal_x) * (180.0f / M_PI);
  if (headingRaw < 0) headingRaw += 360.0f;

  float headingRad = headingRaw * PI / 180.0f;
  headingSin = alpha * sin(headingRad) + (1 - alpha) * headingSin;
  headingCos = alpha * cos(headingRad) + (1 - alpha) * headingCos;

  heading = atan2(headingSin, headingCos) * (180.0f / M_PI);
  if (heading < 0) heading += 360.0f;
}

/* ============================================================
   FUNCIÓN: actualizarVelocidad
   Integra aceleración lineal real. Aplica anti-drift.
   ============================================================ */
void actualizarVelocidad(float ax, float pitchRad, float dt) {
  float ax_real = ax - 9.81f * sin(pitchRad);
  axRealF = alpha * ax_real + (1 - alpha) * axRealF;

  velocity += axRealF * dt;
  if (abs(axRealF) < 0.05) velocity *= 0.9;
}

/* ============================================================
   FUNCIÓN: procesarIMU
   Orquesta el pipeline IMU completo.
   Devuelve false si el sensor está colgado.
   ============================================================ */
bool procesarIMU(DatosIMU &datos, float dt) {
    #ifdef IMU_DESCONECTADO
    datos.pitch      = 0.0f;
    datos.headingRaw = 0.0f;
    datos.heading    = 0.0f;
    datos.velocity   = 0.0f;
    datos.terrain    = "FLAT";
    return true;
  #endif
  float ax, ay, az;
  if (!leerAcelerometro(ax, ay, az)) return false;

  float pitch    = calcularPitch(ax, ay, az);
  float pitchRad = pitchF * PI / 180.0f;

  float headingRaw, heading;
  calcularHeading(headingRaw, heading);

  actualizarVelocidad(ax, pitchRad, dt);

  datos.pitch      = pitchF + 2.7; // offset 
  datos.headingRaw = headingRaw;
  datos.heading    = heading;
  datos.velocity   = velocity;
  datos.terrain    = detectTerrain(pitchF);

  return true;
}

/* ============================================================
   FUNCIÓN: imprimirDatos
   CSV por serial cada 100 ms, sin bloquear.
   Formato: sensores,pitch,headingRaw,heading,velocity,terrain,peso
   ============================================================ */
void imprimirDatos(const DatosIMU &datos) {
  if (millis() - lastPrint < 100) return;
  lastPrint = millis();

  Serial.print("sensores");        Serial.print(",");
  Serial.print(datos.pitch);       Serial.print(",");
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

  #ifndef IMU_DESCONECTADO
    imuInit();
  #endif

  celda.begin(DT, SCK);
  celda.set_scale(calibration_factor);
  celda.tare();

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  servoGripper.setPeriodHertz(50);
  servoBox.setPeriodHertz(50);
  
  servo1.attach(33, 500, 2500); // Servo de afuera 
  servo2.attach(32, 500, 2500); // Servo de adentro
  servoGripper.attach(14, 500, 2500);
  servoBox.attach(27, 500, 2500);

  servo1.write(posicionActual);
  servo2.write(180 - posicionActual);
  servoGripper.write(135);
  servoBox.write(20);

  lastTime = millis();
}

/* ============================================================
   LOOP — pipeline en 4 pasos claros
   FIX BUG 1: if(Serial.available()) ahora cierra correctamente
              su llave antes del pipeline IMU.
   ============================================================ */
void loop() {

  // 1. Procesar comando serial si hay uno disponible
  manejarComandoSerial();  // <-- llave del if está dentro de la función

  // 2. Calcular dt para integración de velocidad
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0f;
  lastTime = now;

  // 3. Procesar IMU (pitch, heading, velocidad, terreno)
  DatosIMU datos;
  if (!procesarIMU(datos, dt)) return;  // sensor colgado → skip

  // 4. Transmitir datos por serial sin bloquear
  imprimirDatos(datos);


  delay(20);
}