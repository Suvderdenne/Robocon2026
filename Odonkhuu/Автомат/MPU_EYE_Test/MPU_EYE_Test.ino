#include <Wire.h>

// E18-D80NK мэдрэгчүүд
const int sensorPins[] = {22, 23, 24, 25, 26, 27, 28, 29};
const int N = sizeof(sensorPins) / sizeof(sensorPins[0]);

// Товчнууд (Pin 31 = reset)
const int buttonPins[] = {31, 32, 33, 34, 35, 36, 37, 38};
const int BN = sizeof(buttonPins) / sizeof(buttonPins[0]);

// MPU-6050
const int MPU = 0x68;
float gyroZoffset = 0;
float heading = 0;
float pitchOffset = 0;
unsigned long lastMicros = 0;

const float NALUU_LIMIT = 25.0;

bool lastResetBtn = HIGH;

int16_t readGyroZ();
void readAccel(float &ax, float &ay, float &az);
void calibrate();

void setup() {
  Serial.begin(9600);
  Wire.begin();

  for (int i = 0; i < N; i++)  pinMode(sensorPins[i],  INPUT_PULLUP);
  for (int i = 0; i < BN; i++) pinMode(buttonPins[i], INPUT_PULLUP);

  // MPU сэрээх
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Serial.println(F("=== Тест эхэлж байна ==="));
  Serial.println(F("РОБОТЫГ ХАВТГАЙ ГАЗАР ХӨДӨЛГӨӨНГҮЙ БАЙЛГА (1 сек)"));
  delay(800);

  calibrate();

  Serial.println(F("Pin 31 товч ЭСВЭЛ 'A' илгээвэл reset хийнэ"));
  Serial.println(F("=================================================================="));

  lastMicros = micros();
}

void loop() {
  // ----- Reset шалгах -----
  // 1) Pin 31 товч (HIGH → LOW шилжилт)
  bool btn = digitalRead(buttonPins[0]);
  if (lastResetBtn == HIGH && btn == LOW) {
    Serial.println(F(">>> RESET (товч) — хөдөлгөөнгүй байлга..."));
    calibrate();
    Serial.println(F(">>> Бэлэн!"));
  }
  lastResetBtn = btn;

  // 2) Serial 'A'
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'A' || c == 'a') {
      Serial.println(F(">>> RESET ('A') — хөдөлгөөнгүй байлга..."));
      calibrate();
      Serial.println(F(">>> Бэлэн!"));
    }
  }

  // ----- Чиг шинэчлэх -----
  unsigned long now = micros();
  float dt = (now - lastMicros) / 1000000.0;
  lastMicros = now;

  float gz_dps = (readGyroZ() - gyroZoffset) / 131.0;
  if (abs(gz_dps) > 0.8) heading -= gz_dps * dt;
  while (heading < 0)    heading += 360;
  while (heading >= 360) heading -= 360;

  const char* zug;
  if      (heading <  45 || heading >= 315) zug = "ХОЙД  ";
  else if (heading < 135)                   zug = "ЗҮҮН  ";
  else if (heading < 225)                   zug = "ӨМНӨ  ";
  else                                      zug = "БАРУУН";

  // ----- Налуу -----
  float ax, ay, az;
  readAccel(ax, ay, az);
  float pitch = atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / PI - pitchOffset;
  bool naluu = abs(pitch) > NALUU_LIMIT;

  // ----- Хэвлэх -----
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 200) {
    lastPrint = millis();

    // Мэдрэгчүүд
    Serial.print(F("S:"));
    for (int i = 0; i < N; i++)
      Serial.print(digitalRead(sensorPins[i]) == LOW ? F("X") : F("."));

    // Товчнууд
    Serial.print(F(" B:"));
    for (int i = 0; i < BN; i++)
      Serial.print(digitalRead(buttonPins[i]) == LOW ? F("X") : F("."));

    // Чиг + налуу
    Serial.print(F(" | "));
    Serial.print(heading, 0);
    Serial.print(F("° "));
    Serial.print(zug);

    Serial.print(F(" | "));
    Serial.print(pitch, 0);
    Serial.print(F("° "));
    Serial.println(naluu ? F("НАЛУУ") : F("ХАВТГАЙ"));
  }
}

void calibrate() {
  // Gyro Z offset
  long sum = 0;
  const int samples = 300;
  for (int i = 0; i < samples; i++) {
    sum += readGyroZ();
    delay(2);
  }
  gyroZoffset = (float)sum / samples;
  heading = 0;

  // Pitch offset (одоогийн налууг "хавтгай" гэж тогтоох)
  float ax, ay, az;
  readAccel(ax, ay, az);
  pitchOffset = atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

  lastMicros = micros();
}

int16_t readGyroZ() {
  Wire.beginTransmission(MPU);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, (uint8_t)2, (uint8_t)true);
  return (int16_t)((Wire.read() << 8) | Wire.read());
}

void readAccel(float &ax, float &ay, float &az) {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, (uint8_t)6, (uint8_t)true);
  int16_t rawX = (Wire.read() << 8) | Wire.read();
  int16_t rawY = (Wire.read() << 8) | Wire.read();
  int16_t rawZ = (Wire.read() << 8) | Wire.read();
  ax = rawX / 16384.0;
  ay = rawY / 16384.0;
  az = rawZ / 16384.0;
}