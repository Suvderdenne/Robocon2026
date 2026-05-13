/*
 * 1 ширхэг 24V BLDC мотор (42GM-4260) — Arduino UNO
 *
 * Утас:
 *   🔴 Улаан  → 24V тэжээлийн +
 *   ⚫ Хар    → 24V тэжээлийн − (UNO GND-той нийтлэг!)
 *   🔵 Хөх    → Pin 9  (SPEED, PWM)
 *   ⚪ Цагаан → Pin 7  (DIR)
 *   🟢 Ногоон → Pin 8  (BRAKE)
 *   🟡 Шар    → холбохгүй
 *
 * Цикл: 3 сек урагш → 1.5 сек зогсох → 3 сек хойш → 1.5 сек зогсох ...
 */

const int SPEED_PIN = 9;     // Хөх — PWM
const int DIR_PIN   = 7;     // Цагаан
const int BRAKE_PIN = 8;     // Ногоон

const int SPEED   = 100;     // 0-255 хооронд
const int RUN_MS  = 3000;
const int STOP_MS = 1500;

const bool INVERT = false;   // Чиглэл эсрэг бол true болго

// ═══════════════════════════════════════════
// BLDC мотор удирдах
// speed: -255..+255  (тэмдэг = чиглэл, хэмжээ = хурд)
// ═══════════════════════════════════════════
void driveBLDC(int speed) {
  if (INVERT) speed = -speed;
  speed = constrain(speed, -150, 150);

  if (speed == 0) {
    analogWrite(SPEED_PIN, 0);
    digitalWrite(BRAKE_PIN, LOW);     // тормоз тавив
    return;
  }

  digitalWrite(BRAKE_PIN, HIGH);      // тормоз салгав
  digitalWrite(DIR_PIN, (speed > 0) ? LOW : HIGH);
  analogWrite(SPEED_PIN, abs(speed));
}

void setup() {
  pinMode(SPEED_PIN, OUTPUT);
  pinMode(DIR_PIN,   OUTPUT);
  pinMode(BRAKE_PIN, OUTPUT);

  driveBLDC(0);   // Эхэндээ зогссон

  Serial.begin(9600);
  Serial.println("BLDC тест эхэллээ");
  delay(1500);
}

void loop() {
  // ▶ Урагш
  Serial.println(">> Урагш");
  driveBLDC(+SPEED);
  delay(RUN_MS);

  // ⏸ Зогсох
  Serial.println(">> Зогсох");
  driveBLDC(0);
  delay(STOP_MS);

  // ◀ Хойш
  Serial.println(">> Хойш");
  driveBLDC(-SPEED);
  delay(RUN_MS);

  // ⏸ Зогсох
  Serial.println(">> Зогсох");
  driveBLDC(0);
  delay(STOP_MS);
}