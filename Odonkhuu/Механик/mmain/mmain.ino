/*
 * ═══════════════════════════════════════════════════════════════════════════
 * R2bot — 8-direction Stick + D-Pad (4 моторт) + 4 туслах мотор + 1 servo
 *         M1, M2: H-bridge DC мотор    M3, M4: 24V BLDC (42GM-4260)
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *   ⭐ ШИНЭ ӨӨРЧЛӨЛТҮҮД ⭐
 *     • M3, M4 нь 24V BLDC (дотроо driver-тэй): SPEED + DIR + BRAKE
 *         M3: D12 SPEED (PWM) + D40 DIR + D41 BRAKE
 *         M4: D13 SPEED (PWM) + D42 DIR + D43 BRAKE
 *     • Хуучин M3, M4 H-bridge pin (D6-D9) — RESERVE (зарлагдсан, ашиглагдахгүй)
 *     • M6 кодоос устгасан (D12, D13 нь BLDC-д ашиглагдсан)
 *     • D-Pad ⬆⬇ — БҮХ 4 моторыг ажиллуулна
 *     • Мотор тус бүрд дээд хязгаар: M1, M2 → 160     M3, M4 → 240
 *     • M3, M4 анхны offset −15
 *
 *   START          → Toggle ON/OFF
 *
 *   🕹️ Зүүн STICK    → 8 чиглэл (F/FR/R/BR/B/BL/L/FL)
 *   🕹️ Баруун STICK  → spin (4 мотор АДИЛ ЧИГЛЭЛД)
 *
 *   D-Pad ⬆⬇        → forward/backward (БҮХ 4 мотор)
 *   D-Pad ⬅➡        → strafe          (M2+M4)
 *
 *   △ +20  × −20  ○ MAX  □ MIN(50)
 *
 * ━━━ ТУСЛАХ МОТОРУУД ━━━
 *   L1 → M7  (D23/D25) digital cycle:  STOP→CW→STOP→CCW→STOP...
 *   L2 → M8  (D27/D29) digital cycle:  STOP→UP→STOP→DOWN→STOP...
 *   R1 → M9  (D31/D30) digital hold:   дарвал асна, авбал унтрана (пневмо)
 *   R2 → M5  (D10/D11) PWM     cycle:  STOP→CW→STOP→CCW→STOP...
 *   SELECT+START → M10 (D33/D32) cycle, LED on D34
 *   SELECT+R1    → M11 (D37) 270° servo toggle (HOME ↔ 270°)
 *
 * ━━━ 🌀 PAIR DRIVE COMBO ━━━
 *   L1 + L2 → toggle pair (M1+M3 ↔ M2+M4)
 *   R1 + R2 → all 4 motors
 *
 * ━━━ 🧪 TEST COMBO ━━━
 *   L2 + R2  → 4 мотор дарааллаар
 *   SELECT+△ → diagonal pair test
 *   ○        → test/cancel
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
const bool MOTOR_INVERT_M1  = false;   // FL — H-bridge
const bool MOTOR_INVERT_M2  = false;   // FR — H-bridge
const bool MOTOR_INVERT_M3  = false;   // RR — BLDC
const bool MOTOR_INVERT_M4  = false;   // RL — BLDC

const bool MOTOR_INVERT_M5  = false;
const bool MOTOR_INVERT_M7  = false;
const bool MOTOR_INVERT_M8  = false;
const bool MOTOR_INVERT_M9  = false;
const bool MOTOR_INVERT_M10 = false;

// ═══════════════════════════════════════════
// 🔧 MOTOR CALIBRATION OFFSETS (live editable)
// ⭐ M3, M4 анхнаасаа −15 (бусдаас бага хурдтай)
// ═══════════════════════════════════════════
int MOTOR_OFFSET_M1 =   0;
int MOTOR_OFFSET_M2 =   0;
int MOTOR_OFFSET_M3 = -15;   // ⭐ default −15
int MOTOR_OFFSET_M4 = -15;   // ⭐ default −15

const int OFFSET_MIN  = -30;
const int OFFSET_MAX  = +30;
const int OFFSET_STEP = 5;

// ═══════════════════════════════════════════
// ⚡ МОТОР ТУС БҮРИЙН ДЭЭД ХУРДНЫ ХЯЗГААР
// ═══════════════════════════════════════════
const int MAX_SPEED_M1_M2 = 160;   // M1, M2 — хамгийн ихдээ 160
const int MAX_SPEED_M3_M4 = 240;   // M3, M4 — хамгийн ихдээ 240

// ═══════════════════════════════════════════
// 🎯 8 ЧИГЛЭЛИЙН МОТОР PATTERN ХҮСНЭГТ
// ═══════════════════════════════════════════
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
// ▼ M1, M2 — H-bridge DC мотор (PWM) ▼
#define M1_RPWM   2
#define M1_LPWM   3
#define M2_RPWM   4
#define M2_LPWM   5

// ▼ ⚠️ ХУУЧИН M3, M4 H-bridge — RESERVE, ашиглагдахгүй ▼
#define M3_RPWM   6     // ХООСОН — reserve (дараа ашиглаж болно)
#define M3_LPWM   7     // ХООСОН — reserve
#define M4_RPWM   8     // ХООСОН — reserve
#define M4_LPWM   9     // ХООСОН — reserve

// ▼ M5 — PWM (R2 cycle) ▼
#define M5_RPWM   10
#define M5_LPWM   11

// ▼ ⭐ M3, M4 — 24V BLDC мотор (42GM-4260, дотроо driver-тэй) ⭐ ▼
//   Red→+24V, Black→24V GND, Blue→PWM SPEED, White→DIR, Green→BRAKE, Yellow→NC
//   D12, D13 нь хуучин M6 PWM байсан (M6 устгасан)
#define M3_SPEED  12    // Blue  — PWM speed (hardware PWM)
#define M3_DIR    40    // White — direction (HIGH/LOW)
#define M3_BRAKE  41    // Green — brake (HIGH=off, LOW=on)
#define M4_SPEED  13    // Blue  — PWM speed (hardware PWM)
#define M4_DIR    42    // White — direction
#define M4_BRAKE  43    // Green — brake

// ▼ M7-M10 — DIGITAL ▼
#define M7_RPWM   23
#define M7_LPWM   25
#define M8_RPWM   27
#define M8_LPWM   29
#define M9_RPWM   31
#define M9_LPWM   30
#define M10_RPWM  33
#define M10_LPWM  32
#define M10_LED   34

// ▼ M11 — 270° SERVO (D37, SELECT+R1) ▼
#define M11_PIN   37
const int M11_HOME_US     = 500;
const int M11_ROTATED_US  = 2500;

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
const int M5_SPEED        = 230;
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
int       activeSector     = 2;

int       m5State          = 0;
int       m7State          = 0;
int       m8State          = 0;
int       m10State         = 0;

Servo     m11Servo;
bool      m11Rotated       = false;

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
// LOW-LEVEL MOTOR (PWM — M1, M2, M5)
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
// LOW-LEVEL DIGITAL MOTOR (M7-M10)
// ═══════════════════════════════════════════
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
// ⭐ DRIVE BLDC (M3, M4) — 24V мотор, дотроо driver-тэй ⭐
// ═══════════════════════════════════════════
//   speedPin: PWM (Blue) — 0-255 хурд
//   dirPin:   digital (White) — HIGH=ccw, LOW=cw (өөрчилж тестлэх)
//   brakePin: digital (Green) — HIGH=brake off, LOW=brake on
//   speed:    -255..+255 (тэмдэг = чиглэл, хэмжээ = хурд)
// ═══════════════════════════════════════════
void driveBLDC(int speedPin, int dirPin, int brakePin, int speed) {
  speed = constrain(speed, -SPEED_MAX, SPEED_MAX);

  if (speed == 0) {
    analogWrite(speedPin, 0);
    digitalWrite(brakePin, LOW);    // brake ENGAGED (мотор зогсох)
    return;
  }

  digitalWrite(brakePin, HIGH);     // brake RELEASED
  digitalWrite(dirPin, (speed > 0) ? LOW : HIGH);
  analogWrite(speedPin, abs(speed));
}

// ═══════════════════════════════════════════
// 🎯 4-МОТОРЫГ ХАМТ УДИРДАХ (Wheels)
// ═══════════════════════════════════════════
//   Дараалал: pair → offset → CAP → invert → drive
//   M1, M2 нь ±160-аас хэтрэхгүй
//   M3, M4 нь ±240-аас хэтрэхгүй
// ═══════════════════════════════════════════
void writeFourMotors(int m1, int m2, int m3, int m4) {
  switch (pairMode) {
    case PAIR_M1_M3: m2 = 0; m4 = 0; break;
    case PAIR_M2_M4: m1 = 0; m3 = 0; break;
    case PAIR_ALL:
    default: break;
  }

  // ▼ Offset (M3, M4 анхнаасаа −15) ▼
  m1 = applyOffset(m1, MOTOR_OFFSET_M1);
  m2 = applyOffset(m2, MOTOR_OFFSET_M2);
  m3 = applyOffset(m3, MOTOR_OFFSET_M3);
  m4 = applyOffset(m4, MOTOR_OFFSET_M4);

  // ▼ ⭐ Мотор тус бүрд CAP — M1, M2 = ±160; M3, M4 = ±240 ⭐
  m1 = constrain(m1, -MAX_SPEED_M1_M2, MAX_SPEED_M1_M2);
  m2 = constrain(m2, -MAX_SPEED_M1_M2, MAX_SPEED_M1_M2);
  m3 = constrain(m3, -MAX_SPEED_M3_M4, MAX_SPEED_M3_M4);
  m4 = constrain(m4, -MAX_SPEED_M3_M4, MAX_SPEED_M3_M4);

  // ▼ Invert ▼
  m1 = applyInvert(m1, MOTOR_INVERT_M1);
  m2 = applyInvert(m2, MOTOR_INVERT_M2);
  m3 = applyInvert(m3, MOTOR_INVERT_M3);
  m4 = applyInvert(m4, MOTOR_INVERT_M4);

  // ▼ Drive ▼
  driveMotor(M1_RPWM, M1_LPWM, m1);                            // M1 — H-bridge
  driveMotor(M2_RPWM, M2_LPWM, m2);                            // M2 — H-bridge
  driveBLDC(M3_SPEED, M3_DIR, M3_BRAKE, m3);                   // M3 — BLDC ⭐
  driveBLDC(M4_SPEED, M4_DIR, M4_BRAKE, m4);                   // M4 — BLDC ⭐
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
    case 2: driveBLDC(M3_SPEED, M3_DIR, M3_BRAKE, speed); break;
    case 3: driveBLDC(M4_SPEED, M4_DIR, M4_BRAKE, speed); break;
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
void stopAllRaw() {
  driveMotor(M1_RPWM, M1_LPWM, 0);
  driveMotor(M2_RPWM, M2_LPWM, 0);
  driveBLDC(M3_SPEED, M3_DIR, M3_BRAKE, 0);   // BLDC brake engaged
  driveBLDC(M4_SPEED, M4_DIR, M4_BRAKE, 0);   // BLDC brake engaged
}

void stopAll() {
  stopAllRaw();
  currentState = "STOPPED";
}

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

// ⭐ D-Pad ⬆⬇ — БҮХ 4 моторыг ажиллуулна
void driveTwoFwdBwd(int speed) { writeFourMotors(speed, speed, -speed, -speed); }

// D-Pad ⬅➡ — strafe (M2, M4 идэвхтэй)
void driveTwoStrafe(int speed) { writeFourMotors(0, speed, 0, -speed); }

void moveForward()   { driveTwoFwdBwd( currentSpeed); currentState = "FWD_4M"; }
void moveBackward()  { driveTwoFwdBwd(-currentSpeed); currentState = "BWD_4M"; }
void strafeLeft()    { driveTwoStrafe(-currentSpeed); currentState = "STRAFE_L_2M"; }
void strafeRight()   { driveTwoStrafe( currentSpeed); currentState = "STRAFE_R_2M"; }

// ═══════════════════════════════════════════
// ⚙️ M5 — R2 CYCLE МОТОР (D10/D11, PWM)
// ═══════════════════════════════════════════
void driveM5(int speed) {
  if (MOTOR_INVERT_M5) speed = -speed;
  speed = constrain(speed, -SPEED_MAX, SPEED_MAX);
  if (speed > 0)      { analogWrite(M5_RPWM, speed);  analogWrite(M5_LPWM, 0); }
  else if (speed < 0) { analogWrite(M5_RPWM, 0);      analogWrite(M5_LPWM, -speed); }
  else                { analogWrite(M5_RPWM, 0);      analogWrite(M5_LPWM, 0); }
}
void stopM5() { m5State = 0; driveM5(0); }

void handleM5() {
  bool comboActive = ps2x.Button(PSB_L2)
                  || ps2x.Button(PSB_R1)
                  || ps2x.Button(PSB_PAD_UP)
                  || ps2x.Button(PSB_PAD_DOWN);
  if (ps2x.ButtonPressed(PSB_R2) && !comboActive) {
    m5State = (m5State + 1) % 4;
    Serial.print(F("⚙️ M5 cycle: "));
    switch (m5State) {
      case 0: Serial.println(F("STOPPED")); break;
      case 1: Serial.println(F("CW"));      break;
      case 2: Serial.println(F("STOPPED")); break;
      case 3: Serial.println(F("CCW"));     break;
    }
  }
  int speed = 0;
  if      (m5State == 1) speed = +M5_SPEED;
  else if (m5State == 3) speed = -M5_SPEED;
  driveM5(speed);
}

// ═══════════════════════════════════════════
// 📦 M7 — L1 CYCLE (D23/D25, digital)
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
// ⬆⬇ M8 — L2 CYCLE (D27/D29, digital)
// ═══════════════════════════════════════════
void driveM8(int direction) {
  driveDigitalMotor(M8_RPWM, M8_LPWM, direction, MOTOR_INVERT_M8);
}
void stopM8() { m8State = 0; driveM8(0); }

void handleM8() {
  bool comboActive = ps2x.Button(PSB_L1)
                  || ps2x.Button(PSB_R2)
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
// 💨 M9 — R1 HOLD (D31/D30, пневмо)
// ═══════════════════════════════════════════
void driveM9(int direction) {
  driveDigitalMotor(M9_RPWM, M9_LPWM, direction, MOTOR_INVERT_M9);
}
void stopM9() { driveM9(0); }

void handleM9() {
  bool m9On = ps2x.Button(PSB_R1) && !ps2x.Button(PSB_SELECT);
  driveM9(m9On ? +1 : 0);
}

// ═══════════════════════════════════════════
// 🎯 M10 — SELECT+START CYCLE (D33/D32) + LED D34
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
void driveM11(bool rotated) {
  if (rotated) m11Servo.writeMicroseconds(M11_ROTATED_US);
  else         m11Servo.writeMicroseconds(M11_HOME_US);
}

void stopM11() {
  m11Rotated = false;
  driveM11(false);
}

void handleM11() {
  if (ps2x.Button(PSB_SELECT) && ps2x.ButtonPressed(PSB_R1)) {
    m11Rotated = !m11Rotated;
    Serial.print(F("🤖 M11 servo: "));
    Serial.println(m11Rotated ? F("ROTATED 270°") : F("HOME 0°"));
    driveM11(m11Rotated);
  }
}

void stopAllAux() {
  stopM5();
  stopM7(); stopM8();
  stopM9(); stopM10();
  stopM11();
}
void driveAllAuxZero() {
  driveM5(0);
  driveM7(0); driveM8(0);
  driveM9(0); driveM10(0);
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
    Serial.println(F("🔥 MAX SPEED (M1,M2→160 cap, M3,M4→240 cap)"));
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
    MOTOR_OFFSET_M1 = 0;
    MOTOR_OFFSET_M2 = 0;
    MOTOR_OFFSET_M3 = -15;   // ⭐ default −15 хадгална
    MOTOR_OFFSET_M4 = -15;
    Serial.println(F("🔄 OFFSETS RESET (M1,M2=0; M3,M4=−15)"));
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
// 🧪 MOTOR TEST COMBOS
// ═══════════════════════════════════════════
bool handleTestCombos() {
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
  angleDeg += 45.0f;
  if (angleDeg < 0)   angleDeg += 360.0f;
  if (angleDeg >= 360.0f) angleDeg -= 360.0f;

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

  handleM5();
  handleM7();
  handleM8();
  handleM9();
  handleM10();
  handleM11();

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
  // M1, M2 — H-bridge (PWM pins)
  safePinInit(M1_RPWM); safePinInit(M1_LPWM);
  safePinInit(M2_RPWM); safePinInit(M2_LPWM);

  // ⚠️ M3, M4 H-bridge pin (D6-D9) — RESERVE, init ХИЙХГҮЙ
  //    (зарлагдсан хэвээр, дараа ашиглах боломжтой)

  // M5 PWM
  safePinInit(M5_RPWM); safePinInit(M5_LPWM);

  // ⭐ M3, M4 BLDC — SPEED (PWM), DIR, BRAKE ⭐
  safePinInit(M3_SPEED); safePinInit(M3_DIR); safePinInit(M3_BRAKE);
  safePinInit(M4_SPEED); safePinInit(M4_DIR); safePinInit(M4_BRAKE);
  // safePinInit нь LOW → BRAKE LOW → brake ENGAGED анх (мотор зогссон)

  // Digital motor pins (M7-M10) + LED
  safePinInit(M7_RPWM);  safePinInit(M7_LPWM);
  safePinInit(M8_RPWM);  safePinInit(M8_LPWM);
  safePinInit(M9_RPWM);  safePinInit(M9_LPWM);
  safePinInit(M10_RPWM); safePinInit(M10_LPWM);
  safePinInit(M10_LED);

  // M11 — Servo (270°)
  m11Servo.attach(M11_PIN, M11_HOME_US, M11_ROTATED_US);
  driveM11(false);
  m11Rotated = false;

  delay(200);
  Serial.begin(115200);
  delay(300);

  Serial.println(F("\n═══════════════════════════════════════"));
  Serial.println(F("  R2bot — M1,M2 H-bridge + M3,M4 BLDC"));
  Serial.println(F("═══════════════════════════════════════\n"));

  initPS2();
  if (ps2_error == 0) {
    Serial.print(F("Controller type: ")); Serial.println(ps2_type);
  }

  Serial.println(F("\n━━━ MOTOR LAYOUT ━━━"));
  Serial.println(F("  M1=FL H-bridge (D2,D3)  — max 160"));
  Serial.println(F("  M2=FR H-bridge (D4,D5)  — max 160"));
  Serial.println(F("  M3=RR BLDC (D12 SPD, D40 DIR, D41 BRAKE) — max 240, offset −15"));
  Serial.println(F("  M4=RL BLDC (D13 SPD, D42 DIR, D43 BRAKE) — max 240, offset −15"));
  Serial.println(F("  D6,D7,D8,D9 — RESERVE (хуучин M3, M4 H-bridge байсан)"));
  Serial.println(F("  M5 (D10,D11) → R2 cycle"));
  Serial.println(F("  M7-M10 (digital), M11 (servo D37)"));

  Serial.println(F("\n━━━ D-PAD ━━━"));
  Serial.println(F("  ⬆⬇ → БҮХ 4 мотор (M1,M2 +s, M3,M4 −s)"));
  Serial.println(F("  ⬅➡ → M2 + M4 strafe"));

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
