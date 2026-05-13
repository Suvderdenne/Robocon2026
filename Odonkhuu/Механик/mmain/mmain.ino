/*
 * ═══════════════════════════════════════════════════════════════════════════
 * R2bot — Robust PS2 Edition
 *         M1, M2, M3: H-bridge DC мотор    M4: 24V BLDC
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *   ⭐ ЭНЭ ХУВИЛБАРЫН ШИНЭ ЗАСВАР ⭐
 *     • Boot: PS2 холболтыг 10 удаа progressive delay-тэйгээр оролдоно
 *     • Runtime: PS2 тасрахад 3сек тутамд автоматаар дахин холбогдоно
 *     • PS2 тасрах үед бүх мотор АВТОМАТААР зогсоно
 *     • Дахин холбогдох үед бүх state reset (m7-m10 cycle, m10 LED)
 *     • Init-ийн өмнө 1сек power-stabilization delay (wireless PS2-д тус)
 *
 *   Бусад зүйл (D-Pad pair, 8-чиглэл бүх 4 мотор, M4 slowdown, idle stopAll,
 *   M10 2-LED, servo, calibration) — өмнөх хувилбараас хэвээр.
 * ═══════════════════════════════════════════════════════════════════════════
 */

#include <PS2X_lib.h>
#include <Servo.h>
#include <math.h>

// ═══════════════════════════════════════════
// 🔄 INVERT
// ═══════════════════════════════════════════
const bool MOTOR_INVERT_M1  = false;
const bool MOTOR_INVERT_M2  = false;
const bool MOTOR_INVERT_M3  = false;
const bool MOTOR_INVERT_M4  = false;

const bool MOTOR_INVERT_M5  = false;
const bool MOTOR_INVERT_M7  = false;
const bool MOTOR_INVERT_M8  = false;
const bool MOTOR_INVERT_M9  = false;
const bool MOTOR_INVERT_M10 = false;

// ═══════════════════════════════════════════
// 🔧 OFFSETS
// ═══════════════════════════════════════════
int MOTOR_OFFSET_M1 =   0;
int MOTOR_OFFSET_M2 =   0;
int MOTOR_OFFSET_M3 =   0;
int MOTOR_OFFSET_M4 =   0;

const int OFFSET_MIN  = -50;
const int OFFSET_MAX  = +30;
const int OFFSET_STEP = 5;

// ═══════════════════════════════════════════
// ⚡ ДЭЭД ХУРД
// ═══════════════════════════════════════════
const int MAX_SPEED_M1_M2 = 160;
const int MAX_SPEED_M3 = 160;      // M3 хэвээр
const int MAX_SPEED_M4 = 80;  

// ═══════════════════════════════════════════
// ⭐ M4 SLOWDOWN
// ═══════════════════════════════════════════
const int M4_SLOWDOWN_FACTOR = 8;

// ═══════════════════════════════════════════
// 🎯 DIRECTION TABLE
// ═══════════════════════════════════════════
int DIRECTION_TABLE[9][4] = {
//      m1     m2     m3     m4
  {     0,     0,     0,     0 },  // 0 = NONE
  {  +100,   +50,  -100,   -50 },  // 1 = F
  {  +100,  +100,  -100,  -100 },  // 2 = FR
  {   +50,  +100,   -50,  -100 },  // 3 = R
  {  -100,  +100,  +100,  -100 },  // 4 = BR
  {  -100,   -50,  +100,   +50 },  // 5 = B
  {  -100,  -100,  +100,  +100 },  // 6 = BL
  {   -50,  -100,   +50,  +100 },  // 7 = L
  {  +100,  -100,  -100,  +100 },  // 8 = FL
};

// ═══════════════════════════════════════════
// PIN CONFIG
// ═══════════════════════════════════════════
#define M1_RPWM   2
#define M1_LPWM   3
#define M2_RPWM   4
#define M2_LPWM   5
#define M3_RPWM   6
#define M3_LPWM   7

#define M5_RPWM   10
#define M5_LPWM   11

#define M4_SPEED  13
#define M4_DIR    42
#define M4_BRAKE  43

#define M7_RPWM   31
#define M7_LPWM   30
#define M8_RPWM   33
#define M8_LPWM   32
#define M9_RPWM   23
#define M9_LPWM   25
#define M10_RPWM  27
#define M10_LPWM  29

#define M10_LED   34
#define M10_LED2  35

#define M11_PIN   37
const int M11_HOME_US     = 500;
const int M11_ROTATED_US  = 2500;

// PS2
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

const int M5_HOLD_SPEED   = 240;

const unsigned long M10_LED_BOTH_AFTER_MS = 20000UL;
const unsigned long M10_LED_STOP_HOLD_MS  =  1000UL;

const unsigned long PS2_READ_INTERVAL  = 50;
const unsigned long TEST_PHASE_MS      = 1000;

// ⭐ PS2 ROBUSTNESS ⭐
const int           PS2_INIT_MAX_ATTEMPTS    = 10;       // 5 → 10
const unsigned long PS2_INIT_BASE_DELAY_MS   = 200;      // эхний оролдлогын дараа
const unsigned long PS2_RECONNECT_INTERVAL_MS = 3000UL;  // runtime reconnect

// ═══════════════════════════════════════════
// ENUMS
// ═══════════════════════════════════════════
enum Direction { DIR_NONE=0, DIR_UP, DIR_DOWN, DIR_LEFT, DIR_RIGHT };
enum Sector    { SEC_NONE=0, SEC_F, SEC_FR, SEC_R, SEC_BR, SEC_B, SEC_BL, SEC_L, SEC_FL };
enum PairMode  { PAIR_ALL=0, PAIR_M1_M3, PAIR_M2_M4 };
enum TestMode  { TEST_OFF=0, TEST_SEQUENCE, TEST_PAIRS };

// ═══════════════════════════════════════════
// GLOBAL STATE
// ═══════════════════════════════════════════
PS2X ps2x;
int   ps2_error = 1;
byte  ps2_type  = 0;
unsigned long ps2LastReconnectAttempt = 0;   // ⭐ runtime reconnect timer

bool   robotEnabled    = false;
int    currentSpeed    = DEFAULT_SPEED;
int    directionState  = DIR_NONE;
String currentState    = "STOPPED";
bool   wasMoving       = false;

PairMode pairMode      = PAIR_ALL;

TestMode testMode      = TEST_OFF;
int      testStep      = 0;
int      testPhase     = 0;
unsigned long testStart = 0;

int       lastSector   = -1;
int       activeSector = 0;

int       m7State      = 0;
int       m8State      = 0;
int       m9State      = 0;
int       m10State     = 0;

bool          m10HasBeenActivated = false;
unsigned long m10RunStartAt       = 0;
unsigned long m10StopStartAt      = 0;
bool          m10IsStopping       = false;

Servo     m11Servo;
bool      m11Rotated   = false;

// ═══════════════════════════════════════════
// SAFE PIN INIT
// ═══════════════════════════════════════════
void safePinInit(int pin) {
  digitalWrite(pin, LOW);
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

// ═══════════════════════════════════════════
// OFFSET / INVERT
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
// LOW-LEVEL DRIVERS
// ═══════════════════════════════════════════
void driveMotor(int rpwm, int lpwm, int speed) {
  speed = constrain(speed, -SPEED_MAX, SPEED_MAX);
  if (speed > 0)      { analogWrite(rpwm, speed);  analogWrite(lpwm, 0); }
  else if (speed < 0) { analogWrite(rpwm, 0);      analogWrite(lpwm, -speed); }
  else                { analogWrite(rpwm, 0);      analogWrite(lpwm, 0); }
}

void driveDigitalMotor(int rpwm, int lpwm, int direction, bool invert) {
  if (invert) direction = -direction;
  if (direction > 0)      { digitalWrite(rpwm, HIGH); digitalWrite(lpwm, LOW);  }
  else if (direction < 0) { digitalWrite(rpwm, LOW);  digitalWrite(lpwm, HIGH); }
  else                    { digitalWrite(rpwm, LOW);  digitalWrite(lpwm, LOW);  }
}

void driveBLDC(int speedPin, int dirPin, int brakePin, int speed) {
  speed = constrain(speed, -SPEED_MAX, SPEED_MAX);
  if (speed == 0) {
    analogWrite(speedPin, 0);
    digitalWrite(brakePin, LOW);
    return;
  }
  digitalWrite(brakePin, HIGH);
  digitalWrite(dirPin, (speed > 0) ? LOW : HIGH);
  analogWrite(speedPin, abs(speed));
}

// ═══════════════════════════════════════════
// 4-MOTOR HARNESS
// ═══════════════════════════════════════════
void writeFourMotors(int m1, int m2, int m3, int m4) {
  switch (pairMode) {
    case PAIR_M1_M3: m2 = 0; m4 = 0; break;
    case PAIR_M2_M4: m1 = 0; m3 = 0; break;
    case PAIR_ALL:
    default: break;
  }

  if (M4_SLOWDOWN_FACTOR > 1) m4 = m4 / M4_SLOWDOWN_FACTOR;

  m1 = applyOffset(m1, MOTOR_OFFSET_M1);
  m2 = applyOffset(m2, MOTOR_OFFSET_M2);
  m3 = applyOffset(m3, MOTOR_OFFSET_M3);
  m4 = applyOffset(m4, MOTOR_OFFSET_M4);

  m1 = constrain(m1, -MAX_SPEED_M1_M2, MAX_SPEED_M1_M2);
  m2 = constrain(m2, -MAX_SPEED_M1_M2, MAX_SPEED_M1_M2);
m3 = constrain(m3, -MAX_SPEED_M3, MAX_SPEED_M3);
m4 = constrain(m4, -MAX_SPEED_M4, MAX_SPEED_M4);

  m1 = applyInvert(m1, MOTOR_INVERT_M1);
  m2 = applyInvert(m2, MOTOR_INVERT_M2);
  m3 = applyInvert(m3, MOTOR_INVERT_M3);
  m4 = applyInvert(m4, MOTOR_INVERT_M4);

  driveMotor(M1_RPWM, M1_LPWM, m1);
  driveMotor(M2_RPWM, M2_LPWM, m2);
  driveMotor(M3_RPWM, M3_LPWM, m3);
  driveBLDC (M4_SPEED, M4_DIR, M4_BRAKE, m4);
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
    case 0: driveMotor(M1_RPWM, M1_LPWM, speed);            break;
    case 1: driveMotor(M2_RPWM, M2_LPWM, speed);            break;
    case 2: driveMotor(M3_RPWM, M3_LPWM, speed);            break;
    case 3: driveBLDC (M4_SPEED, M4_DIR, M4_BRAKE, speed);  break;
  }
}

const char* motorName(int idx) {
  switch (idx) {
    case 0: return "M1 (FL)"; case 1: return "M2 (FR)";
    case 2: return "M3 (RR)"; case 3: return "M4 (RL/BLDC)";
  }
  return "?";
}

// ═══════════════════════════════════════════
// STOP / SECTOR
// ═══════════════════════════════════════════
void stopAllRaw() {
  driveMotor(M1_RPWM, M1_LPWM, 0);
  driveMotor(M2_RPWM, M2_LPWM, 0);
  driveMotor(M3_RPWM, M3_LPWM, 0);
  driveBLDC (M4_SPEED, M4_DIR, M4_BRAKE, 0);
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
// SPIN / D-PAD HELPERS
// ═══════════════════════════════════════════
void spinAll(int speed) { writeFourMotors(speed, speed, speed, speed); }

void driveTwoFwdBwd(int speed) {
  writeFourMotors(speed, 0, -speed, 0);   // M1+M3 only
}
void driveTwoStrafe(int speed) {
  writeFourMotors(0, speed, 0, -speed);   // M2+M4 only
}

void moveForward()   { driveTwoFwdBwd( currentSpeed); currentState = "FWD_M1_M3"; }
void moveBackward()  { driveTwoFwdBwd(-currentSpeed); currentState = "BWD_M1_M3"; }
void strafeLeft()    { driveTwoStrafe(-currentSpeed); currentState = "STRAFE_L_M2_M4"; }
void strafeRight()   { driveTwoStrafe( currentSpeed); currentState = "STRAFE_R_M2_M4"; }

// ═══════════════════════════════════════════
// M5 — SELECT+START hold
// ═══════════════════════════════════════════
void driveM5(int speed) {
  if (MOTOR_INVERT_M5) speed = -speed;
  driveMotor(M5_RPWM, M5_LPWM, speed);
}
void stopM5() { driveM5(0); }

void handleM5() {
  bool m5On = ps2x.Button(PSB_SELECT) && ps2x.Button(PSB_START);
  driveM5(m5On ? M5_HOLD_SPEED : 0);
}

// ═══════════════════════════════════════════
// M7-M10 cycle motors
// ═══════════════════════════════════════════
void driveM7(int direction) { driveDigitalMotor(M7_RPWM, M7_LPWM, direction, MOTOR_INVERT_M7); }
void stopM7() { m7State = 0; driveM7(0); }
void handleM7() {
  bool comboActive = ps2x.Button(PSB_L2) || ps2x.Button(PSB_PAD_UP) || ps2x.Button(PSB_PAD_DOWN);
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
  if (m7State == 1) dir = +1; else if (m7State == 3) dir = -1;
  driveM7(dir);
}

void driveM8(int direction) { driveDigitalMotor(M8_RPWM, M8_LPWM, direction, MOTOR_INVERT_M8); }
void stopM8() { m8State = 0; driveM8(0); }
void handleM8() {
  bool comboActive = ps2x.Button(PSB_L1) || ps2x.Button(PSB_R2)
                  || ps2x.Button(PSB_PAD_UP) || ps2x.Button(PSB_PAD_DOWN);
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
  if (m8State == 1) dir = +1; else if (m8State == 3) dir = -1;
  driveM8(dir);
}

void driveM9(int direction) { driveDigitalMotor(M9_RPWM, M9_LPWM, direction, MOTOR_INVERT_M9); }
void stopM9() { m9State = 0; driveM9(0); }
void handleM9() {
  bool comboActive = ps2x.Button(PSB_L1) || ps2x.Button(PSB_L2) || ps2x.Button(PSB_R2)
                  || ps2x.Button(PSB_SELECT)
                  || ps2x.Button(PSB_PAD_UP) || ps2x.Button(PSB_PAD_DOWN);
  if (ps2x.ButtonPressed(PSB_R1) && !comboActive) {
    m9State = (m9State + 1) % 4;
    Serial.print(F("💨 M9 cycle: "));
    switch (m9State) {
      case 0: Serial.println(F("STOPPED")); break;
      case 1: Serial.println(F("UP"));      break;
      case 2: Serial.println(F("STOPPED")); break;
      case 3: Serial.println(F("DOWN"));    break;
    }
  }
  int dir = 0;
  if (m9State == 1) dir = +1; else if (m9State == 3) dir = -1;
  driveM9(dir);
}

// ═══════════════════════════════════════════
// M10 + 2 LED
// ═══════════════════════════════════════════
void driveM10(int direction) { driveDigitalMotor(M10_RPWM, M10_LPWM, direction, MOTOR_INVERT_M10); }
void m10SetLeds(bool led1, bool led2) {
  digitalWrite(M10_LED,  led1 ? HIGH : LOW);
  digitalWrite(M10_LED2, led2 ? HIGH : LOW);
}
void stopM10() {
  m10State = 0;
  driveM10(0);
  m10IsStopping = false;
  m10SetLeds(false, false);
}

void handleM10() {
  bool comboActive = ps2x.Button(PSB_L1) || ps2x.Button(PSB_L2) || ps2x.Button(PSB_R1)
                  || ps2x.Button(PSB_SELECT)
                  || ps2x.Button(PSB_PAD_UP) || ps2x.Button(PSB_PAD_DOWN);
  if (ps2x.ButtonPressed(PSB_R2) && !comboActive) {
    int prev = m10State;
    m10State = (m10State + 1) % 4;
    Serial.print(F("🎯 M10 cycle: "));
    switch (m10State) {
      case 0: Serial.println(F("STOPPED")); break;
      case 1: Serial.println(F("CW"));      break;
      case 2: Serial.println(F("STOPPED")); break;
      case 3: Serial.println(F("CCW"));     break;
    }
    bool prevRunning = (prev == 1)    || (prev == 3);
    bool nowRunning  = (m10State == 1) || (m10State == 3);
    m10HasBeenActivated = true;
    if (nowRunning) {
      m10RunStartAt  = millis();
      m10IsStopping  = false;
    } else if (prevRunning) {
      m10IsStopping  = true;
      m10StopStartAt = millis();
    }
  }

  int dir = 0;
  if (m10State == 1) dir = +1; else if (m10State == 3) dir = -1;
  driveM10(dir);

  unsigned long now = millis();
  if (m10IsStopping) {
    if (now - m10StopStartAt < M10_LED_STOP_HOLD_MS) m10SetLeds(true, true);
    else { m10IsStopping = false; m10SetLeds(false, false); }
  } else if (dir != 0) {
    unsigned long runElapsed = now - m10RunStartAt;
    if (runElapsed < M10_LED_BOTH_AFTER_MS) {
      if (dir > 0) m10SetLeds(true, false);
      else         m10SetLeds(false, true);
    } else {
      m10SetLeds(true, true);
    }
  } else {
    if (!m10HasBeenActivated) m10SetLeds(true, true);
    else                      m10SetLeds(false, false);
  }
}

// ═══════════════════════════════════════════
// M11 Servo
// ═══════════════════════════════════════════
void driveM11(bool rotated) {
  if (rotated) m11Servo.writeMicroseconds(M11_ROTATED_US);
  else         m11Servo.writeMicroseconds(M11_HOME_US);
}
void stopM11() { m11Rotated = false; driveM11(false); }

void handleM11() {
  if (ps2x.Button(PSB_SELECT) && ps2x.ButtonPressed(PSB_R1)) {
    m11Rotated = !m11Rotated;
    Serial.print(F("🤖 M11 servo: "));
    Serial.println(m11Rotated ? F("ROTATED 270°") : F("HOME 0°"));
    driveM11(m11Rotated);
  }
}

// ═══════════════════════════════════════════
// AUX STOP / ZERO
// ═══════════════════════════════════════════
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
  m10SetLeds(false, false);
}

// ═══════════════════════════════════════════
// ⭐ PS2 — ROBUST INIT ⭐
// ═══════════════════════════════════════════
//   10 удаа progressive delay-тэйгээр оролдоно (200ms → 1100ms)
//   Тэгсэн ч холбогдоогүй бол runtime-д үргэлжлүүлэн оролдоно.
// ═══════════════════════════════════════════
bool initPS2() {
  for (int attempt = 1; attempt <= PS2_INIT_MAX_ATTEMPTS; attempt++) {
    ps2_error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, PRESSURES, RUMBLE);
    if (ps2_error == 0) {
      ps2_type = ps2x.readType();
      Serial.print(F("✅ PS2 connected (attempt "));
      Serial.print(attempt);
      Serial.print(F("), type="));
      Serial.println(ps2_type);
      return true;
    }
    Serial.print(F("⚠️ PS2 attempt "));
    Serial.print(attempt); Serial.print('/');
    Serial.print(PS2_INIT_MAX_ATTEMPTS);
    Serial.print(F(" failed (err="));
    Serial.print(ps2_error);
    Serial.println(F("), retrying..."));
    // Progressive delay: 300, 400, 500, ..., 1200ms
    delay(PS2_INIT_BASE_DELAY_MS + (unsigned long)attempt * 100UL);
  }
  Serial.println(F("❌ PS2 init failed after all attempts — will retry at runtime"));
  return false;
}

// ⭐ RUNTIME RECONNECT ⭐
//   3 сек тутамд config_gamepad-г дахин дуудаж үзнэ.
//   Амжилттай бол state-уудыг reset хийнэ.
bool tryReconnectPS2() {
  unsigned long now = millis();
  if (now - ps2LastReconnectAttempt < PS2_RECONNECT_INTERVAL_MS) return false;
  ps2LastReconnectAttempt = now;

  Serial.println(F("🔄 PS2 reconnect attempting..."));
  ps2_error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, PRESSURES, RUMBLE);
  if (ps2_error == 0) {
    ps2_type = ps2x.readType();
    Serial.print(F("✅ PS2 reconnected, type="));
    Serial.println(ps2_type);
    // ⭐ Reset state — холболтын дараа гэнэтийн зан төлвөөс сэргийлнэ
    stopAll();
    stopAllAux();
    m7State = m8State = m9State = m10State = 0;
    m10HasBeenActivated = false;
    m10IsStopping = false;
    pairMode = PAIR_ALL;
    testMode = TEST_OFF;
    return true;
  } else {
    Serial.print(F("⚠️ Reconnect failed (err="));
    Serial.print(ps2_error);
    Serial.println(F(")"));
    return false;
  }
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
    Serial.println(F("🔥 MAX SPEED"));
  }
  if (ps2x.ButtonPressed(PSB_SQUARE)) {
    currentSpeed = 50;
    Serial.println(F("🐌 MIN SPEED (50)"));
  }
}

// ═══════════════════════════════════════════
// CALIBRATION
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
  Serial.print(F("  M4 slowdown: ÷")); Serial.println(M4_SLOWDOWN_FACTOR);
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
    MOTOR_OFFSET_M3 = 0;
    MOTOR_OFFSET_M4 = 0;
    Serial.println(F("🔄 OFFSETS RESET"));
    return true;
  }
  if (SEL && ps2x.ButtonPressed(PSB_SQUARE)) {
    printAllOffsets();
    return true;
  }
  return false;
}

// ═══════════════════════════════════════════
// PAIR / TEST
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
    Serial.println(F("🌀 PAIR MODE: ALL 4"));
    stopAll();
    return true;
  }
  return false;
}

bool handleTestCombos() {
  bool l2heldR2pressed = ps2x.Button(PSB_L2) && ps2x.ButtonPressed(PSB_R2);
  bool r2heldL2pressed = ps2x.Button(PSB_R2) && ps2x.ButtonPressed(PSB_L2);
  if (l2heldR2pressed || r2heldL2pressed) {
    testMode = TEST_SEQUENCE; testStep = 0; testPhase = 0; testStart = millis();
    Serial.println(F("\n🧪 MOTOR TEST: M1 → M2 → M3 → M4"));
    return true;
  }
  if (ps2x.Button(PSB_SELECT) && ps2x.ButtonPressed(PSB_TRIANGLE)) {
    testMode = TEST_PAIRS; testStep = 0; testPhase = 0; testStart = millis();
    Serial.println(F("\n🧪 PAIR TEST"));
    return true;
  }
  return false;
}

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
// D-PAD / STICKS
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

bool handleSticks() {
  int LX = ps2x.Analog(PSS_LX) - 128;
  int LY = 128 - ps2x.Analog(PSS_LY);
  int RX = ps2x.Analog(PSS_RX) - 128;

  if (abs(LX) < DEADZONE) LX = 0;
  if (abs(LY) < DEADZONE) LY = 0;
  if (abs(RX) < DEADZONE) RX = 0;

  if (LX == 0 && LY == 0 && RX == 0) {
    lastSector = -1; activeSector = 0;
    return false;
  }

  if (RX != 0) {
    int spinSpeed = (RX > 0) ? ROTATION_SPEED : -ROTATION_SPEED;
    spinAll(spinSpeed);
    currentState = (RX > 0) ? "SPIN_CW" : "SPIN_CCW";
    lastSector = -1; activeSector = 0;
    return true;
  }

  long magSq = (long)LX * LX + (long)LY * LY;
  int  mag   = (int)sqrt((float)magSq);
  if (mag > 128) mag = 128;
  int  speed = (mag * currentSpeed) / 128;

  if (speed < MIN_STICK_SPEED) {
    lastSector = -1; activeSector = 0;
    return false;
  }

  float angleDeg = atan2((float)LX, (float)LY) * 180.0f / (float)PI;
  angleDeg += 45.0f;
  if (angleDeg < 0)       angleDeg += 360.0f;
  if (angleDeg >= 360.0f) angleDeg -= 360.0f;
  int sector = (((int)((angleDeg + 22.5f) / 45.0f)) % 8) + 1;

  driveSector(sector, speed);
  currentState = sectorName(sector);

  if (sector != lastSector) {
    Serial.print(F("🎯 sector="));
    Serial.print(sector); Serial.print(F(" speed="));
    Serial.println(speed);
    lastSector = sector;
  }
  return true;
}

// ═══════════════════════════════════════════
// MAIN INPUT PROCESSOR — PS2 РОБУСТ
// ═══════════════════════════════════════════
void processInput() {
  // ⭐ PS2 disconnected ⇒ tryReconnect + бүх мотор зогсоо ⭐
  if (ps2_error != 0) {
    tryReconnectPS2();
    stopAll();
    stopAllAux();
    m10SetLeds(false, false);
    directionState = DIR_NONE;
    activeSector = 0;
    return;
  }

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
    m10HasBeenActivated = false;
    m10IsStopping       = false;
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
    lastSector = -1; activeSector = 0;
  } else if (handleSticks()) {
    moving = true;
    directionState = DIR_NONE;
  }

  if (!moving) {
    stopAll();
    if (wasMoving) Serial.println(F("⛔ Idle — STOP"));
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
  // Pin init
  safePinInit(M1_RPWM); safePinInit(M1_LPWM);
  safePinInit(M2_RPWM); safePinInit(M2_LPWM);
  safePinInit(M3_RPWM); safePinInit(M3_LPWM);
  safePinInit(M5_RPWM); safePinInit(M5_LPWM);

  pinMode(M4_SPEED, OUTPUT);  analogWrite(M4_SPEED, 0);
  pinMode(M4_DIR,   OUTPUT);  digitalWrite(M4_DIR, LOW);
  pinMode(M4_BRAKE, OUTPUT);  digitalWrite(M4_BRAKE, LOW);

  safePinInit(M7_RPWM);  safePinInit(M7_LPWM);
  safePinInit(M8_RPWM);  safePinInit(M8_LPWM);
  safePinInit(M9_RPWM);  safePinInit(M9_LPWM);
  safePinInit(M10_RPWM); safePinInit(M10_LPWM);
  safePinInit(M10_LED);
  safePinInit(M10_LED2);

  m11Servo.attach(M11_PIN, M11_HOME_US, M11_ROTATED_US);
  driveM11(false);
  m11Rotated = false;

  stopAll();
  stopAllAux();

  // ⭐ Power stabilization — wireless PS2-д хэрэгтэй ⭐
  delay(500);
  Serial.begin(115200);
  delay(500);

  Serial.println(F("\n═══════════════════════════════════════"));
  Serial.println(F("  R2bot — Robust PS2 Edition"));
  Serial.println(F("═══════════════════════════════════════\n"));

  // ⭐ PS2 init-ийн өмнө нэмэлт 1сек ⭐
  Serial.println(F("⏳ PS2 power stabilization (1s)..."));
  delay(1000);

  initPS2();

  Serial.println(F("\n━━━ PAIR LOGIC ━━━"));
  Serial.println(F("  D-Pad ⬆⬇ → M1+M3 only"));
  Serial.println(F("  D-Pad ⬅➡ → M2+M4 only"));
  Serial.println(F("  Stick 8-чиглэл → all 4 motors"));
  Serial.print  (F("  M4 slowdown: ÷")); Serial.println(M4_SLOWDOWN_FACTOR);

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

  if (testMode != TEST_OFF && ps2_error == 0 && robotEnabled) {
    runMotorTest();
  }
}
