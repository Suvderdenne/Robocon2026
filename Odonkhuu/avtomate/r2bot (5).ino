/*
 * ═══════════════════════════════════════════════════════════════════════════
 * R2bot — 8-direction Stick + 2-motor D-Pad + 4 туслах digital мотор
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *   START          → Toggle ON/OFF
 *
 *   🕹️ Зүүн STICK    → 8 чиглэл (F/FR/R/BR/B/BL/L/FL)
 *   🕹️ Баруун STICK  → spin (4 мотор АДИЛ ЧИГЛЭЛД)
 *
 *   D-Pad ⬆⬇        → forward/backward (M1+M3)
 *   D-Pad ⬅➡        → strafe          (M2+M4)
 *
 *   △ +20  × −20  ○ MAX(255)  □ MIN(50)
 *
 * ━━━ ТУСЛАХ МОТОРУУД (digital) ━━━
 *   L1 → M7  (D23/D25)  cycle:  STOP→CW→STOP→CCW→STOP...
 *   L2 → M8  (D27/D29)  cycle:  STOP→UP→STOP→DOWN→STOP...
 *   R1 → M9  (D31/D30)  hold:   дарвал асна, авбал унтрана (пневмо)
 *   SELECT+START → M10 (D33/D32) cycle, LED on D34
 *
 *   M5 (D10/D11) ба M6 (D12/D13) — зарлагдсан ч АШИГЛАХГҮЙ
 *
 * ━━━ 🌀 PAIR DRIVE COMBO ━━━
 *   L1 + L2 → toggle pair (M1+M3 ↔ M2+M4)   ⚠️ M7, M8 нэг удаа cycle хийх боломжтой
 *   R1 + R2 → all 4 motors                  ⚠️ M9 хэсэг хугацаанд асна
 *
 * ━━━ 🧪 TEST COMBO ━━━
 *   SELECT + ×    → 4 мотор дарааллаар (M10-той зөрөөгүй болгож × руу нүүсэн)
 *   SELECT + △    → diagonal pair test
 *   ○             → test/cancel
 *
 * ━━━ 🔧 LIVE CALIBRATION ━━━
 *   L1+⬆/⬇ → M1 ±5    R1+⬆/⬇ → M2 ±5
 *   L2+⬆/⬇ → M4 ±5    R2+⬆/⬇ → M3 ±5
 *   SELECT+○ → reset     SELECT+□ → print
 *
 * ═══════════════════════════════════════════════════════════════════════════
 */

#include <PS2X_lib.h>
#include <Servo.h>
#include <math.h>

// ═══════════════════════════════════════════
// 🔄 MOTOR DIRECTION INVERT
// ═══════════════════════════════════════════
const bool MOTOR_INVERT_M1  = false;   // FL
const bool MOTOR_INVERT_M2  = false;   // FR
const bool MOTOR_INVERT_M3  = false;   // RR
const bool MOTOR_INVERT_M4  = false;   // RL

// Туслах моторын INVERT (digital pins)
const bool MOTOR_INVERT_M7  = false;   // L1 cycle (хайрцгийн мотор)
const bool MOTOR_INVERT_M8  = false;   // L2 cycle (дээш доош өргөгч)
const bool MOTOR_INVERT_M9  = false;   // R1 hold (пневмо)
const bool MOTOR_INVERT_M10 = false;   // SELECT+START cycle (жадны гар)

// ═══════════════════════════════════════════
// 🔧 MOTOR CALIBRATION OFFSETS (live editable)
// ═══════════════════════════════════════════
int MOTOR_OFFSET_M1 = 0;
int MOTOR_OFFSET_M2 = 0;
int MOTOR_OFFSET_M3 = 0;
int MOTOR_OFFSET_M4 = 0;

const int OFFSET_MIN  = -30;
const int OFFSET_MAX  = +30;
const int OFFSET_STEP = 5;

// ═══════════════════════════════════════════
// 🎯 8 ЧИГЛЭЛИЙН МОТОР PATTERN ХҮСНЭГТ
// ═══════════════════════════════════════════
// Sector ID: 1=F  2=FR  3=R  4=BR  5=B  6=BL  7=L  8=FL
int DIRECTION_TABLE[9][4] = {
//      m1     m2     m3     m4
  {     0,     0,     0,     0 },  // 0 = NONE
  {  +100,     0,  -100,     0 },  // 1 = F
  {  +100,  +100,  -100,  -100 },  // 2 = FR
  {     0,  +100,     0,  -100 },  // 3 = R
  {  -100,  +100,  +100,  -100 },  // 4 = BR
  {  -100,     0,  +100,     0 },  // 5 = B
  {  -100,  -100,  +100,  +100 },  // 6 = BL
  {     0,  -100,     0,  +100 },  // 7 = L
  {  +100,  -100,  -100,  +100 },  // 8 = FL
};

// ═══════════════════════════════════════════
// PIN CONFIGURATION
// ═══════════════════════════════════════════
// ▼ Үндсэн 4 мотор — PWM ▼
#define M1_RPWM   2
#define M1_LPWM   3
#define M2_RPWM   4
#define M2_LPWM   5
#define M3_RPWM   6
#define M3_LPWM   7
#define M4_RPWM   8
#define M4_LPWM   9
// ▼ M5/M6 — PWM (зарлагдсан ч АШИГЛАХГҮЙ) ▼
#define M5_RPWM   10
#define M5_LPWM   11
#define M6_RPWM   12
#define M6_LPWM   13
// ▼ M7-M10 — DIGITAL (PWM биш) ▼
#define M7_RPWM   23     // L1 cycle
#define M7_LPWM   25
#define M8_RPWM   27     // L2 cycle
#define M8_LPWM   29
#define M9_RPWM   31     // R1 hold (пневмо)
#define M9_LPWM   30
#define M10_RPWM  33     // SELECT+START cycle
#define M10_LPWM  32
#define M10_LED   34     // M10 LED indicator
// ▼ M11 — 270° SERVO (D37, SELECT+R1) ▼
#define M11_PIN   37
const int M11_HOME_US     = 500;    // 0° байрлал (home)
const int M11_ROTATED_US  = 2500;   // 270° байрлал

// ▼ PS2 controller pins ▼
#define PS2_DAT  24
#define PS2_CMD  22
#define PS2_SEL  28
#define PS2_CLK  26
#define PRESSURES   false
#define RUMBLE      false

// ═══════════════════════════════════════════
// CONSTANTS
// ═══════════════════════════════════════════
const int DEADZONE        = 40;
const int MIN_STICK_SPEED = 30;
const int ROTATION_SPEED  = 30;
const int SPEED_MIN       = 0;
const int SPEED_MAX       = 255;
const int DEFAULT_SPEED   = 80;
const int SPEED_STEP      = 20;
const int TEST_SPEED      = 150;
const int M5_SPEED        = 230;   // АШИГЛАХГҮЙ
const int M6_SPEED        = 230;   // АШИГЛАХГҮЙ
const unsigned long PS2_READ_INTERVAL  = 50;
const unsigned long TEST_PHASE_MS      = 1000;

// ═══════════════════════════════════════════
// ENUMS
// ═══════════════════════════════════════════
enum Direction {
  DIR_NONE = 0, DIR_UP = 1, DIR_DOWN = 2, DIR_LEFT = 3, DIR_RIGHT = 4
};

enum Sector {
  SEC_NONE = 0, SEC_F = 1, SEC_FR = 2, SEC_R = 3, SEC_BR = 4,
  SEC_B = 5, SEC_BL = 6, SEC_L = 7, SEC_FL = 8
};

enum PairMode {
  PAIR_ALL = 0, PAIR_M1_M3 = 1, PAIR_M2_M4 = 2
};

enum TestMode {
  TEST_OFF = 0, TEST_SEQUENCE = 1, TEST_PAIRS = 2
};

// ═══════════════════════════════════════════
// GLOBAL STATE
// ═══════════════════════════════════════════
PS2X ps2x;
int   ps2_error    = 1;
byte  ps2_type     = 0;

bool   robotEnabled    = false;
int    currentSpeed    = DEFAULT_SPEED;
int    directionState  = DIR_NONE;
String currentState    = "STOPPED";
bool   wasMoving       = false;

PairMode pairMode       = PAIR_ALL;

TestMode testMode       = TEST_OFF;
int      testStep       = 0;
int      testPhase      = 0;
unsigned long testStart = 0;

int       lastSector       = -1;
int       activeSector     = 0;

// M5/M6 — declared but not used
bool      m5Toggled        = false;
int       m6State          = 0;

// M7, M8, M10 циклийн state (4-state: 0=STOP, 1=CW, 2=STOP, 3=CCW)
int       m7State          = 0;   // L1 cycle
int       m8State          = 0;   // L2 cycle
int       m10State         = 0;   // SELECT+START cycle
// M9 нь зөвхөн hold-on/release-off — state хадгалах хэрэггүй

// M11 — Servo (270°), SELECT+R1 toggle
Servo     m11Servo;
bool      m11Rotated       = false;   // false = HOME (0°), true = 270°

// ═══════════════════════════════════════════
// SAFE PIN INIT
// ═══════════════════════════════════════════
void safePinInit(int pin) {
  digitalWrite(pin, LOW);
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

// ═══════════════════════════════════════════
// 🔧 OFFSET / 🔄 INVERT
// ═══════════════════════════════════════════
int applyOffset(int speed, int offset) {
  if (speed == 0) return 0;
  if (speed > 0)  return constrain(speed + offset, 0, SPEED_MAX);
  else            return constrain(speed - offset, -SPEED_MAX, 0);
}

int applyInvert(int speed, bool invert) {
  return invert ? -speed : speed;
}

// ═══════════════════════════════════════════
// LOW-LEVEL MOTOR (PWM — M1-M6)
// ═══════════════════════════════════════════
void driveMotor(int rpwm, int lpwm, int speed) {
  speed = constrain(speed, -SPEED_MAX, SPEED_MAX);
  if (speed > 0) {
    analogWrite(rpwm, speed);
    analogWrite(lpwm, 0);
  } else if (speed < 0) {
    analogWrite(rpwm, 0);
    analogWrite(lpwm, -speed);
  } else {
    analogWrite(rpwm, 0);
    analogWrite(lpwm, 0);
  }
}

// ═══════════════════════════════════════════
// LOW-LEVEL DIGITAL MOTOR (M7-M10) — non-PWM pins
// ═══════════════════════════════════════════
//   direction:  +1 = forward (full power)
//               -1 = reverse (full power)
//                0 = stop
void driveDigitalMotor(int rpwm, int lpwm, int direction, bool invert) {
  if (invert) direction = -direction;
  if (direction > 0) {
    digitalWrite(rpwm, HIGH);
    digitalWrite(lpwm, LOW);
  } else if (direction < 0) {
    digitalWrite(rpwm, LOW);
    digitalWrite(lpwm, HIGH);
  } else {
    digitalWrite(rpwm, LOW);
    digitalWrite(lpwm, LOW);
  }
}

// ═══════════════════════════════════════════
// 🎯 4-МОТОРЫГ ХАМТ УДИРДАХ (Wheels)
// ═══════════════════════════════════════════
void writeFourMotors(int m1, int m2, int m3, int m4) {
  switch (pairMode) {
    case PAIR_M1_M3: m2 = 0; m4 = 0; break;
    case PAIR_M2_M4: m1 = 0; m3 = 0; break;
    case PAIR_ALL:
    default: break;
  }
  m1 = applyOffset(m1, MOTOR_OFFSET_M1);
  m2 = applyOffset(m2, MOTOR_OFFSET_M2);
  m3 = applyOffset(m3, MOTOR_OFFSET_M3);
  m4 = applyOffset(m4, MOTOR_OFFSET_M4);
  m1 = applyInvert(m1, MOTOR_INVERT_M1);
  m2 = applyInvert(m2, MOTOR_INVERT_M2);
  m3 = applyInvert(m3, MOTOR_INVERT_M3);
  m4 = applyInvert(m4, MOTOR_INVERT_M4);
  driveMotor(M1_RPWM, M1_LPWM, m1);
  driveMotor(M2_RPWM, M2_LPWM, m2);
  driveMotor(M3_RPWM, M3_LPWM, m3);
  driveMotor(M4_RPWM, M4_LPWM, m4);
}

void driveSingleMotor(int motorIndex, int speed) {
  bool invert = false;
  switch (motorIndex) {
    case 0: invert = MOTOR_INVERT_M1; break;
    case 1: invert = MOTOR_INVERT_M2; break;
    case 2: invert = MOTOR_INVERT_M3; break;
    case 3: invert = MOTOR_INVERT_M4; break;
  }
  if (invert) speed = -speed;
  switch (motorIndex) {
    case 0: driveMotor(M1_RPWM, M1_LPWM, speed); break;
    case 1: driveMotor(M2_RPWM, M2_LPWM, speed); break;
    case 2: driveMotor(M3_RPWM, M3_LPWM, speed); break;
    case 3: driveMotor(M4_RPWM, M4_LPWM, speed); break;
  }
}

const char* motorName(int idx) {
  switch (idx) {
    case 0: return "M1 (FL)"; case 1: return "M2 (FR)";
    case 2: return "M3 (RR)"; case 3: return "M4 (RL)";
  }
  return "?";
}

// ═══════════════════════════════════════════
// 🎯 8-SECTOR DRIVE
// ═══════════════════════════════════════════
void driveSector(int sector, int speed) {
  if (sector < 1 || sector > 8) {
    stopAllRaw();
    activeSector = 0;
    return;
  }
  int m1 = (DIRECTION_TABLE[sector][0] * speed) / 100;
  int m2 = (DIRECTION_TABLE[sector][1] * speed) / 100;
  int m3 = (DIRECTION_TABLE[sector][2] * speed) / 100;
  int m4 = (DIRECTION_TABLE[sector][3] * speed) / 100;
  writeFourMotors(m1, m2, m3, m4);
  activeSector = sector;
}

const char* sectorName(int sector) {
  switch (sector) {
    case 0: return "NONE"; case 1: return "F";  case 2: return "FR";
    case 3: return "R";    case 4: return "BR"; case 5: return "B";
    case 6: return "BL";   case 7: return "L";  case 8: return "FL";
  }
  return "?";
}

// ═══════════════════════════════════════════
// 🌀 SPIN, D-PAD HELPERS
// ═══════════════════════════════════════════
void spinAll(int speed) { writeFourMotors(speed, speed, speed, speed); }
void driveTwoFwdBwd(int speed) { writeFourMotors(speed, 0, -speed, 0); }
void driveTwoStrafe(int speed) { writeFourMotors(0, speed, 0, -speed); }

void stopAllRaw() {
  driveMotor(M1_RPWM, M1_LPWM, 0);
  driveMotor(M2_RPWM, M2_LPWM, 0);
  driveMotor(M3_RPWM, M3_LPWM, 0);
  driveMotor(M4_RPWM, M4_LPWM, 0);
}

void stopAll() {
  stopAllRaw();
  currentState = "STOPPED";
}

void moveForward()   { driveTwoFwdBwd( currentSpeed); currentState = "FWD_2M"; }
void moveBackward()  { driveTwoFwdBwd(-currentSpeed); currentState = "BWD_2M"; }
void strafeLeft()    { driveTwoStrafe(-currentSpeed); currentState = "STRAFE_L_2M"; }
void strafeRight()   { driveTwoStrafe( currentSpeed); currentState = "STRAFE_R_2M"; }

// ═══════════════════════════════════════════
// ⚙️ M5 / 🔁 M6 — ЗАРЛАГДСАН ГЭХДЭЭ АШИГЛАХГҮЙ
// (handleM5, handleM6 ДУУДАГДАХГҮЙ — processInput-д комментолсон)
// ═══════════════════════════════════════════
void driveM5(int speed) {
  speed = constrain(speed, -SPEED_MAX, SPEED_MAX);
  if (speed > 0)      { analogWrite(M5_RPWM, speed);  analogWrite(M5_LPWM, 0); }
  else if (speed < 0) { analogWrite(M5_RPWM, 0);      analogWrite(M5_LPWM, -speed); }
  else                { analogWrite(M5_RPWM, 0);      analogWrite(M5_LPWM, 0); }
}
void stopM5() { m5Toggled = false; driveM5(0); }

void driveM6(int speed) {
  speed = constrain(speed, -SPEED_MAX, SPEED_MAX);
  if (speed > 0)      { analogWrite(M6_RPWM, speed);  analogWrite(M6_LPWM, 0); }
  else if (speed < 0) { analogWrite(M6_RPWM, 0);      analogWrite(M6_LPWM, -speed); }
  else                { analogWrite(M6_RPWM, 0);      analogWrite(M6_LPWM, 0); }
}
void stopM6() { m6State = 0; driveM6(0); }

// ═══════════════════════════════════════════
// 📦 M7 — ХАЙРЦГИЙН ЭРГЭДЭГ МОТОР (D23/D25, L1)
// ═══════════════════════════════════════════
//   L1 → 4 төлөв цикл: STOP → CW → STOP → CCW → STOP → ...
//   ⚠️ L1+L2 эсвэл L1+⬆⬇ combo үед cycle хийхгүй
// ═══════════════════════════════════════════
void driveM7(int direction) {
  driveDigitalMotor(M7_RPWM, M7_LPWM, direction, MOTOR_INVERT_M7);
}
void stopM7() { m7State = 0; driveM7(0); }

void handleM7() {
  bool comboActive = ps2x.Button(PSB_L2)
                  || ps2x.Button(PSB_PAD_UP)
                  || ps2x.Button(PSB_PAD_DOWN);
  if (ps2x.ButtonPressed(PSB_L1) && !comboActive) {
    m7State = (m7State + 1) % 4;
    Serial.print(F("📦 M7 cycle: "));
    switch (m7State) {
      case 0: Serial.println(F("STOPPED")); break;
      case 1: Serial.println(F("CW"));      break;
      case 2: Serial.println(F("STOPPED")); break;
      case 3: Serial.println(F("CCW"));     break;
    }
  }
  int dir = 0;
  if      (m7State == 1) dir = +1;
  else if (m7State == 3) dir = -1;
  driveM7(dir);
}

// ═══════════════════════════════════════════
// ⬆⬇ M8 — ДЭЭШ ДООШ ӨРГӨГЧ МОТОР (D27/D29, L2)
// ═══════════════════════════════════════════
//   L2 → 4 төлөв цикл: STOP → UP → STOP → DOWN → STOP → ...
//   ⚠️ L1+L2 эсвэл L2+⬆⬇ combo үед cycle хийхгүй
// ═══════════════════════════════════════════
void driveM8(int direction) {
  driveDigitalMotor(M8_RPWM, M8_LPWM, direction, MOTOR_INVERT_M8);
}
void stopM8() { m8State = 0; driveM8(0); }

void handleM8() {
  bool comboActive = ps2x.Button(PSB_L1)
                  || ps2x.Button(PSB_R2)         // R2+L2 = test mode
                  || ps2x.Button(PSB_PAD_UP)
                  || ps2x.Button(PSB_PAD_DOWN);
  if (ps2x.ButtonPressed(PSB_L2) && !comboActive) {
    m8State = (m8State + 1) % 4;
    Serial.print(F("⬆⬇ M8 cycle: "));
    switch (m8State) {
      case 0: Serial.println(F("STOPPED")); break;
      case 1: Serial.println(F("UP"));      break;
      case 2: Serial.println(F("STOPPED")); break;
      case 3: Serial.println(F("DOWN"));    break;
    }
  }
  int dir = 0;
  if      (m8State == 1) dir = +1;
  else if (m8State == 3) dir = -1;
  driveM8(dir);
}

// ═══════════════════════════════════════════
// 💨 M9 — ХИЙН ЦЕЛИНДЕР (D31/D30, R1 hold)
// ═══════════════════════════════════════════
//   R1 дараастай үед: тог өгөнө (forward direction)
//   R1 тавихад: унтарна
//   ⚠️ SELECT+R1 нь M11 servo-д очдог тул SELECT held үед M9 идэвхгүй
// ═══════════════════════════════════════════
void driveM9(int direction) {
  driveDigitalMotor(M9_RPWM, M9_LPWM, direction, MOTOR_INVERT_M9);
}
void stopM9() { driveM9(0); }

void handleM9() {
  // R1 held + SELECT NOT held → M9 on
  // (SELECT+R1 нь M11 servo, тэгэхээр M9 идэвхгүй болгож байна)
  bool m9On = ps2x.Button(PSB_R1) && !ps2x.Button(PSB_SELECT);
  driveM9(m9On ? +1 : 0);
}

// ═══════════════════════════════════════════
// 🎯 M10 — ЖАДНЫ ГАР (D33/D32, SELECT+START) + LED on D34
// ═══════════════════════════════════════════
void driveM10(int direction) {
  driveDigitalMotor(M10_RPWM, M10_LPWM, direction, MOTOR_INVERT_M10);
  digitalWrite(M10_LED, (direction != 0) ? HIGH : LOW);
}
void stopM10() { m10State = 0; driveM10(0); }

void handleM10() {
  if (ps2x.Button(PSB_SELECT) && ps2x.ButtonPressed(PSB_START)) {
    m10State = (m10State + 1) % 4;
    Serial.print(F("🎯 M10 cycle: "));
    switch (m10State) {
      case 0: Serial.println(F("STOPPED (LED off)")); break;
      case 1: Serial.println(F("CW (LED on)"));       break;
      case 2: Serial.println(F("STOPPED (LED off)")); break;
      case 3: Serial.println(F("CCW (LED on)"));      break;
    }
  }
  int dir = 0;
  if      (m10State == 1) dir = +1;
  else if (m10State == 3) dir = -1;
  driveM10(dir);
}

// ═══════════════════════════════════════════
// 🤖 M11 — 270° SERVO (D37, SELECT+R1)
// ═══════════════════════════════════════════
//   2 төлөвт toggle:
//     1-р дарах: 270° руу эргэнэ
//     2-р дарах: HOME (0°) руу буцаана
//   SELECT+R1 — нэг удаа хамт дарах бүрд
// ═══════════════════════════════════════════
void driveM11(bool rotated) {
  if (rotated) m11Servo.writeMicroseconds(M11_ROTATED_US);
  else         m11Servo.writeMicroseconds(M11_HOME_US);
}

void stopM11() {
  m11Rotated = false;
  driveM11(false);   // HOME руу буцаана
}

void handleM11() {
  // SELECT held + R1 rising edge → toggle servo
  if (ps2x.Button(PSB_SELECT) && ps2x.ButtonPressed(PSB_R1)) {
    m11Rotated = !m11Rotated;
    Serial.print(F("🤖 M11 servo: "));
    Serial.println(m11Rotated ? F("ROTATED 270°") : F("HOME 0°"));
    driveM11(m11Rotated);
  }
}

// Бүх туслах моторыг зэрэг зогсоох helpers
void stopAllAux() {
  stopM5(); stopM6();
  stopM7(); stopM8();
  stopM9(); stopM10();
  stopM11();
}
void driveAllAuxZero() {
  driveM5(0); driveM6(0);
  driveM7(0); driveM8(0);
  driveM9(0); driveM10(0);
  // M11 нь өөрөө байрлал хадгалдаг тул HOME руу буцаахгүй —
  // зөвхөн START toggle буюу stopAllAux үед буцаана
}

// ═══════════════════════════════════════════
// PS2 INITIALIZATION
// ═══════════════════════════════════════════
bool initPS2() {
  for (int attempt = 1; attempt <= 5; attempt++) {
    ps2_error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, PRESSURES, RUMBLE);
    if (ps2_error == 0) {
      Serial.print(F("✅ PS2 connected (attempt "));
      Serial.print(attempt); Serial.println(F(")"));
      ps2_type = ps2x.readType();
      return true;
    }
    Serial.print(F("⚠️ PS2 attempt ")); Serial.print(attempt);
    Serial.println(F(" failed, retrying..."));
    delay(400);
  }
  Serial.println(F("❌ PS2 init failed"));
  return false;
}

// ═══════════════════════════════════════════
// SPEED ADJUSTMENT
// ═══════════════════════════════════════════
void handleSpeedButtons() {
  bool calibCombo = ps2x.Button(PSB_L1) || ps2x.Button(PSB_L2) ||
                    ps2x.Button(PSB_R1) || ps2x.Button(PSB_R2) ||
                    ps2x.Button(PSB_SELECT);
  if (calibCombo) return;

  if (ps2x.ButtonPressed(PSB_TRIANGLE)) {
    currentSpeed = min(SPEED_MAX, currentSpeed + SPEED_STEP);
    Serial.print(F("⚡ Speed +")); Serial.print(SPEED_STEP);
    Serial.print(F(" → ")); Serial.println(currentSpeed);
  }
  if (ps2x.ButtonPressed(PSB_CROSS)) {
    currentSpeed = max(SPEED_MIN, currentSpeed - SPEED_STEP);
    Serial.print(F("🐢 Speed −")); Serial.print(SPEED_STEP);
    Serial.print(F(" → ")); Serial.println(currentSpeed);
  }
  if (ps2x.ButtonPressed(PSB_CIRCLE)) {
    currentSpeed = SPEED_MAX;
    Serial.println(F("🔥 MAX SPEED (255)"));
  }
  if (ps2x.ButtonPressed(PSB_SQUARE)) {
    currentSpeed = 50;
    Serial.println(F("🐌 MIN SPEED (50)"));
  }
}

// ═══════════════════════════════════════════
// 🔧 LIVE CALIBRATION
// ═══════════════════════════════════════════
void adjustOffset(int* offset, int delta, const char* name) {
  *offset = constrain(*offset + delta, OFFSET_MIN, OFFSET_MAX);
  Serial.print(F("🔧 ")); Serial.print(name);
  Serial.print(F(" offset → ")); Serial.println(*offset);
}

void printAllOffsets() {
  Serial.println(F("\n━━━ Current Offsets ━━━"));
  Serial.print(F("  M1 (FL): ")); Serial.println(MOTOR_OFFSET_M1);
  Serial.print(F("  M2 (FR): ")); Serial.println(MOTOR_OFFSET_M2);
  Serial.print(F("  M3 (RR): ")); Serial.println(MOTOR_OFFSET_M3);
  Serial.print(F("  M4 (RL): ")); Serial.println(MOTOR_OFFSET_M4);
  Serial.println();
}

bool handleCalibration() {
  bool L1 = ps2x.Button(PSB_L1);
  bool L2 = ps2x.Button(PSB_L2);
  bool R1 = ps2x.Button(PSB_R1);
  bool R2 = ps2x.Button(PSB_R2);
  bool SEL = ps2x.Button(PSB_SELECT);

  bool up   = ps2x.ButtonPressed(PSB_PAD_UP);
  bool down = ps2x.ButtonPressed(PSB_PAD_DOWN);

  if (L1 && !L2 && !R1 && !R2) {
    if (up)   { adjustOffset(&MOTOR_OFFSET_M1,  OFFSET_STEP, "M1"); return true; }
    if (down) { adjustOffset(&MOTOR_OFFSET_M1, -OFFSET_STEP, "M1"); return true; }
  }
  if (R1 && !L1 && !L2 && !R2) {
    if (up)   { adjustOffset(&MOTOR_OFFSET_M2,  OFFSET_STEP, "M2"); return true; }
    if (down) { adjustOffset(&MOTOR_OFFSET_M2, -OFFSET_STEP, "M2"); return true; }
  }
  if (R2 && !L1 && !L2 && !R1) {
    if (up)   { adjustOffset(&MOTOR_OFFSET_M3,  OFFSET_STEP, "M3"); return true; }
    if (down) { adjustOffset(&MOTOR_OFFSET_M3, -OFFSET_STEP, "M3"); return true; }
  }
  if (L2 && !L1 && !R1 && !R2) {
    if (up)   { adjustOffset(&MOTOR_OFFSET_M4,  OFFSET_STEP, "M4"); return true; }
    if (down) { adjustOffset(&MOTOR_OFFSET_M4, -OFFSET_STEP, "M4"); return true; }
  }

  if (SEL && ps2x.ButtonPressed(PSB_CIRCLE)) {
    MOTOR_OFFSET_M1 = MOTOR_OFFSET_M2 = MOTOR_OFFSET_M3 = MOTOR_OFFSET_M4 = 0;
    Serial.println(F("🔄 ALL OFFSETS RESET → 0"));
    return true;
  }
  if (SEL && ps2x.ButtonPressed(PSB_SQUARE)) {
    printAllOffsets();
    return true;
  }
  return false;
}

// ═══════════════════════════════════════════
// 🌀 PAIR MODE COMBOS
// ═══════════════════════════════════════════
bool handlePairCombos() {
  if (ps2x.Button(PSB_L1) && ps2x.ButtonPressed(PSB_L2)) {
    pairMode = (pairMode == PAIR_M1_M3) ? PAIR_M2_M4 : PAIR_M1_M3;
    Serial.print(F("🌀 PAIR MODE: "));
    Serial.println(pairMode == PAIR_M1_M3 ? F("M1 + M3") : F("M2 + M4"));
    stopAll();
    return true;
  }
  if (ps2x.Button(PSB_R1) && ps2x.ButtonPressed(PSB_R2)) {
    pairMode = PAIR_ALL;
    Serial.println(F("🌀 PAIR MODE: ALL 4 motors"));
    stopAll();
    return true;
  }
  return false;
}

// ═══════════════════════════════════════════
// 🧪 MOTOR TEST COMBOS (TEST_SEQUENCE → L2+R2 руу нүүсэн)
// ═══════════════════════════════════════════
bool handleTestCombos() {
  // TEST SEQUENCE — L2 + R2 (зэрэг дарах)
  // L2 held + R2 pressed, эсвэл R2 held + L2 pressed — аль аль ажиллана
  bool l2heldR2pressed = ps2x.Button(PSB_L2) && ps2x.ButtonPressed(PSB_R2);
  bool r2heldL2pressed = ps2x.Button(PSB_R2) && ps2x.ButtonPressed(PSB_L2);
  if (l2heldR2pressed || r2heldL2pressed) {
    testMode  = TEST_SEQUENCE;
    testStep  = 0;
    testPhase = 0;
    testStart = millis();
    Serial.println(F("\n🧪 MOTOR TEST: M1 → M2 → M3 → M4"));
    Serial.print(F("Step 1/4 — ")); Serial.print(motorName(0));
    Serial.println(F(" forward"));
    return true;
  }
  // PAIR TEST — SELECT + △
  if (ps2x.Button(PSB_SELECT) && ps2x.ButtonPressed(PSB_TRIANGLE)) {
    testMode  = TEST_PAIRS;
    testStep  = 0;
    testPhase = 0;
    testStart = millis();
    Serial.println(F("\n🧪 PAIR TEST"));
    Serial.println(F("Step 1/2 — M1+M3 forward"));
    return true;
  }
  return false;
}

// ═══════════════════════════════════════════
// 🧪 TEST RUNNER
// ═══════════════════════════════════════════
void runMotorTest() {
  if (testMode == TEST_OFF) return;

  unsigned long elapsed = millis() - testStart;

  if (elapsed >= TEST_PHASE_MS) {
    testPhase++;
    testStart = millis();

    if (testPhase >= 2) {
      testPhase = 0;
      testStep++;

      int totalSteps = (testMode == TEST_SEQUENCE) ? 4 : 2;
      if (testStep >= totalSteps) {
        stopAll();
        Serial.println(F("✅ TEST COMPLETE\n"));
        testMode = TEST_OFF;
        return;
      }

      Serial.print(F("Step "));
      Serial.print(testStep + 1); Serial.print('/'); Serial.print(totalSteps);
      Serial.print(F(" — "));
      if (testMode == TEST_SEQUENCE) Serial.print(motorName(testStep));
      else Serial.print(testStep == 0 ? F("M1+M3") : F("M2+M4"));
      Serial.println(F(" forward"));
    } else {
      Serial.println(F("   ↻ backward"));
    }
  }

  int speed = (testPhase == 0) ? TEST_SPEED : -TEST_SPEED;

  if (testMode == TEST_SEQUENCE) {
    for (int m = 0; m < 4; m++) {
      driveSingleMotor(m, (m == testStep) ? speed : 0);
    }
  } else {
    if (testStep == 0) {
      driveSingleMotor(0, speed); driveSingleMotor(1, 0);
      driveSingleMotor(2, speed); driveSingleMotor(3, 0);
    } else {
      driveSingleMotor(0, 0); driveSingleMotor(1, speed);
      driveSingleMotor(2, 0); driveSingleMotor(3, speed);
    }
  }
  currentState = "TESTING";
}

// ═══════════════════════════════════════════
// D-PAD HANDLER
// ═══════════════════════════════════════════
bool handleDPad() {
  bool calibCombo = ps2x.Button(PSB_L1) || ps2x.Button(PSB_L2) ||
                    ps2x.Button(PSB_R1) || ps2x.Button(PSB_R2);
  if (calibCombo) {
    directionState = DIR_NONE;
    return false;
  }

  if (ps2x.Button(PSB_PAD_UP))    { directionState = DIR_UP;    moveForward();  return true; }
  if (ps2x.Button(PSB_PAD_DOWN))  { directionState = DIR_DOWN;  moveBackward(); return true; }
  if (ps2x.Button(PSB_PAD_LEFT))  { directionState = DIR_LEFT;  strafeLeft();   return true; }
  if (ps2x.Button(PSB_PAD_RIGHT)) { directionState = DIR_RIGHT; strafeRight();  return true; }

  directionState = DIR_NONE;
  return false;
}

// ═══════════════════════════════════════════
// STICK HANDLER
// ═══════════════════════════════════════════
bool handleSticks() {
  int LX = ps2x.Analog(PSS_LX) - 128;
  int LY = 128 - ps2x.Analog(PSS_LY);
  int RX = ps2x.Analog(PSS_RX) - 128;

  if (abs(LX) < DEADZONE) LX = 0;
  if (abs(LY) < DEADZONE) LY = 0;
  if (abs(RX) < DEADZONE) RX = 0;

  if (LX == 0 && LY == 0 && RX == 0) {
    lastSector = -1;
    activeSector = 0;
    return false;
  }

  if (RX != 0) {
    int spinSpeed = (RX > 0) ? ROTATION_SPEED : -ROTATION_SPEED;
    spinAll(spinSpeed);
    currentState = (RX > 0) ? "SPIN_CW_ALL" : "SPIN_CCW_ALL";
    lastSector = -1;
    activeSector = 0;
    return true;
  }

  long magSq = (long)LX * LX + (long)LY * LY;
  int  mag   = (int)sqrt((float)magSq);
  if (mag > 128) mag = 128;
  int  speed = (mag * currentSpeed) / 128;

  if (speed < MIN_STICK_SPEED) {
    lastSector = -1;
    activeSector = 0;
    return false;
  }

  float angleDeg = atan2((float)LX, (float)LY) * 180.0f / (float)PI;
  if (angleDeg < 0) angleDeg += 360.0f;

  int sector = (((int)((angleDeg + 22.5f) / 45.0f)) % 8) + 1;

  driveSector(sector, speed);
  currentState = sectorName(sector);

  if (sector != lastSector) {
    Serial.print(F("🎯 activeSector=")); Serial.print(sector);
    Serial.print(F(" (")); Serial.print(sectorName(sector));
    Serial.print(F(")  speed=")); Serial.println(speed);
    lastSector = sector;
  }

  return true;
}

// ═══════════════════════════════════════════
// MAIN INPUT PROCESSOR
// ═══════════════════════════════════════════
void processInput() {
  if (ps2_error != 0) return;

  ps2x.read_gamepad(false, 0);

  // START дангаар → robot toggle (SELECT+START нь M10-д очино)
  if (ps2x.ButtonPressed(PSB_START) && !ps2x.Button(PSB_SELECT)) {
    robotEnabled = !robotEnabled;
    stopAll();
    stopAllAux();
    wasMoving = false;
    directionState = DIR_NONE;
    lastSector = -1;
    activeSector = 0;
    testMode = TEST_OFF;
    if (robotEnabled) Serial.println(F("▶️ ROBOT ON"));
    else              Serial.println(F("⏸️ ROBOT OFF"));
  }

  if (!robotEnabled) {
    stopAll();
    driveAllAuxZero();
    directionState = DIR_NONE;
    activeSector = 0;
    return;
  }

  if (testMode != TEST_OFF) {
    if (ps2x.ButtonPressed(PSB_CIRCLE)) {
      testMode = TEST_OFF;
      stopAll();
      Serial.println(F("⛔ TEST CANCELLED"));
    }
    driveAllAuxZero();
    activeSector = 0;
    return;
  }

  // ━━━ Туслах мотор handler-ууд ━━━
  // M5, M6 — АШИГЛАХГҮЙ (зарлагдсан ч дуудагдахгүй)
  // handleM5();  ← idle, дуудахгүй
  // handleM6();  ← idle, дуудахгүй

  // M7-M10 идэвхтэй
  handleM7();    // L1 cycle (хайрцгийн мотор)
  handleM8();    // L2 cycle (дээш доош өргөгч)
  handleM9();    // R1 hold  (пневмо)
  handleM10();   // SELECT+START cycle (жадны гар + LED)
  handleM11();   // SELECT+R1 toggle (270° servo)

  // ━━━ Combo, тест, дугуй ━━━
  bool calibrated  = handleCalibration();
  bool paired      = handlePairCombos();
  bool testStarted = handleTestCombos();
  if (calibrated || paired || testStarted) return;

  handleSpeedButtons();

  bool moving = false;
  if (handleDPad()) {
    moving = true;
    lastSector = -1;
    activeSector = 0;
  } else if (handleSticks()) {
    moving = true;
    directionState = DIR_NONE;
  }

  if (!moving) {
    if (wasMoving) {
      stopAll();
      Serial.println(F("⛔ Idle — STOP"));
    }
    directionState = DIR_NONE;
    lastSector = -1;
    activeSector = 0;
  }
  wasMoving = moving;
}

// ═══════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════
void setup() {
  // PWM motor pins (M1-M6)
  safePinInit(M1_RPWM); safePinInit(M1_LPWM);
  safePinInit(M2_RPWM); safePinInit(M2_LPWM);
  safePinInit(M3_RPWM); safePinInit(M3_LPWM);
  safePinInit(M4_RPWM); safePinInit(M4_LPWM);
  safePinInit(M5_RPWM); safePinInit(M5_LPWM);
  safePinInit(M6_RPWM); safePinInit(M6_LPWM);

  // Digital motor pins (M7-M10) + LED
  safePinInit(M7_RPWM);  safePinInit(M7_LPWM);
  safePinInit(M8_RPWM);  safePinInit(M8_LPWM);
  safePinInit(M9_RPWM);  safePinInit(M9_LPWM);
  safePinInit(M10_RPWM); safePinInit(M10_LPWM);
  safePinInit(M10_LED);

  // M11 — Servo (270°). attach(pin, min_us, max_us)
  m11Servo.attach(M11_PIN, M11_HOME_US, M11_ROTATED_US);
  driveM11(false);   // эхлээд HOME байрлал
  m11Rotated = false;

  delay(200);
  Serial.begin(115200);
  delay(300);

  Serial.println(F("\n═══════════════════════════════════════"));
  Serial.println(F("  R2bot — 8-direction + 4 aux digital motors"));
  Serial.println(F("═══════════════════════════════════════\n"));

  initPS2();
  if (ps2_error == 0) {
    Serial.print(F("Controller type: ")); Serial.println(ps2_type);
  }

  Serial.println(F("\n━━━ MOTOR LAYOUT ━━━"));
  Serial.println(F("  M1=FL (D2,D3)    M2=FR (D4,D5)"));
  Serial.println(F("  M4=RL (D8,D9)    M3=RR (D6,D7)"));
  Serial.println(F("  M5/M6 (D10-D13) → ЗАРЛАГДСАН ГЭХДЭЭ АШИГЛАХГҮЙ"));
  Serial.println(F("  M7  (D23/D25) → L1 cycle (хайрцгийн мотор)"));
  Serial.println(F("  M8  (D27/D29) → L2 cycle (дээш доош өргөгч)"));
  Serial.println(F("  M9  (D31/D30) → R1 hold (пневмо целиндер)"));
  Serial.println(F("  M10 (D33/D32) → SELECT+START cycle (жадны гар)"));
  Serial.println(F("  M10 LED (D34)"));
  Serial.println(F("  M11 (D37)     → SELECT+R1 toggle (270° servo)"));

  Serial.println(F("\n━━━ ҮНДСЭН ХЯНАЛТ ━━━"));
  Serial.println(F("  START         → ON/OFF (SELECT-гүйгээр)"));
  Serial.println(F("  Зүүн стик     → 8-direction translate"));
  Serial.println(F("  Баруун стик   → spin"));
  Serial.println(F("  D-Pad ⬆⬇      → fwd/back (M1+M3)"));
  Serial.println(F("  D-Pad ⬅➡      → strafe   (M2+M4)"));
  Serial.println(F("  △+20 ×−20 ○MAX □MIN"));

  Serial.println(F("\n━━━ ТУСЛАХ МОТОРУУД ━━━"));
  Serial.println(F("  L1 → M7 cycle  (4-state: STOP/CW/STOP/CCW)"));
  Serial.println(F("  L2 → M8 cycle  (4-state: STOP/UP/STOP/DOWN)"));
  Serial.println(F("  R1 → M9 hold   (дараастай үед асна, SELECT-гүйгээр)"));
  Serial.println(F("  SELECT+START → M10 cycle, LED on D34"));
  Serial.println(F("  SELECT+R1   → M11 servo toggle (HOME ↔ 270°)"));

  Serial.println(F("\n━━━ 🌀 PAIR (wheels) ━━━"));
  Serial.println(F("  L1+L2 → M1+M3 / M2+M4 toggle"));
  Serial.println(F("  R1+R2 → all 4"));

  Serial.println(F("\n━━━ 🧪 TEST ━━━"));
  Serial.println(F("  L2+R2    → seq test (зэрэг даравал)"));
  Serial.println(F("  SELECT+△ → pair test"));
  Serial.println(F("  ○        → cancel"));

  Serial.println(F("\n━━━ 🔧 CALIB ━━━"));
  Serial.println(F("  L1+⬆⬇ M1    R1+⬆⬇ M2"));
  Serial.println(F("  R2+⬆⬇ M3    L2+⬆⬇ M4"));
  Serial.println(F("  SELECT+○ reset    SELECT+□ print"));

  printAllOffsets();
  Serial.println(F("⏸️ ROBOT OFF — press START\n"));
  Serial.println(F("R2BOT_READY\n"));
}

// ═══════════════════════════════════════════
// MAIN LOOP
// ═══════════════════════════════════════════
void loop() {
  static unsigned long lastRead = 0;
  unsigned long now = millis();

  if (now - lastRead >= PS2_READ_INTERVAL) {
    lastRead = now;
    processInput();
  }

  if (testMode != TEST_OFF) {
    runMotorTest();
  }
}
