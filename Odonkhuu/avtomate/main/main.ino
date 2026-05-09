/*
 * ═══════════════════════════════════════════════════════════════════════════
 * R2bot — 8-direction Left Stick + 2-motor D-Pad + Same-direction Spin
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *        ⬆ Forward
 *   M1 ╱           ╲ M2        M1 = FL (Front-Left)
 *      \           /           M2 = FR (Front-Right)
 *       \  ROBOT  /            M3 = RR (Rear-Right)
 *      /           \           M4 = RL (Rear-Left)
 *   M4 ╲           ╱ M3
 *        ⬇ Backward
 *
 *  Дугаалал ЦАГИЙН ЗҮҮНИЙ ДАГУУ: M1 → M2 → M3 → M4
 *
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *   START          → Toggle ON/OFF
 *
 *   🕹️ Зүүн STICK    → 8 чиглэл (F/FR/R/BR/B/BL/L/FL)
 *                       өнцөг → чиглэл, тулгасан хэмжээ → хурд
 *   🕹️ Баруун STICK  → spin (4 мотор АДИЛ ЧИГЛЭЛД)
 *
 *   ━ Зүүн стикийн 8 чиглэлийн мотор ━
 *      F   → M1 + M3              FR  → бүх 4 мотор
 *      R   → M2 + M4              BR  → бүх 4 мотор
 *      B   → M1 + M3              BL  → бүх 4 мотор
 *      L   → M2 + M4              FL  → бүх 4 мотор
 *
 *   D-Pad ⬆⬇        → forward/backward (зөвхөн M1 + M3 — diagonal pair)
 *   D-Pad ⬅➡        → strafe         (зөвхөн M2 + M4 — diagonal pair)
 *
 *   △ +20  × −20  ○ MAX(255)  □ MIN(50)
 *
 * ━━━ 🌀 PAIR DRIVE COMBO ━━━
 *   L1 + L2 → toggle pair (M1+M3 ↔ M2+M4)
 *   R1 + R2 → all 4 motors
 *
 * ━━━ 🧪 TEST COMBO ━━━
 *   SELECT + START → 4 моторыг дарааллаар (M1 → M2 → M3 → M4)
 *   SELECT + △     → diagonal pair test (M1+M3, дараа M2+M4)
 *   ○              → test зогсоох
 *
 * ━━━ 🔧 LIVE CALIBRATION ━━━
 *   L1+⬆/⬇ → M1 ±5    R1+⬆/⬇ → M2 ±5
 *   L2+⬆/⬇ → M4 ±5    R2+⬆/⬇ → M3 ±5
 *   SELECT+○ → reset     SELECT+□ → print
 *
 * ═══════════════════════════════════════════════════════════════════════════
 */

#include <PS2X_lib.h>
#include <math.h>

// ═══════════════════════════════════════════
// 🔄 MOTOR DIRECTION INVERT
// ═══════════════════════════════════════════
// Хэрэв мотор буруу чиглэлд эргэж байвал тухайн утгыг true болгоно
const bool MOTOR_INVERT_M1 = false;   // FL
const bool MOTOR_INVERT_M2 = false;   // FR
const bool MOTOR_INVERT_M3 = false;   // RR
const bool MOTOR_INVERT_M4 = false;   // RL

// ═══════════════════════════════════════════
// 🔧 MOTOR CALIBRATION OFFSETS (live editable)
// ═══════════════════════════════════════════
int MOTOR_OFFSET_M1 = 0;   // FL
int MOTOR_OFFSET_M2 = 0;   // FR
int MOTOR_OFFSET_M3 = 0;   // RR
int MOTOR_OFFSET_M4 = 0;   // RL

const int OFFSET_MIN  = -30;
const int OFFSET_MAX  = +30;
const int OFFSET_STEP = 5;

// ═══════════════════════════════════════════
// 🎯 8 ЧИГЛЭЛИЙН МОТОР PATTERN ХҮСНЭГТ
// ═══════════════════════════════════════════
// Зүүн стик нь өнцгөөс хамаарч `activeSector` хувьсагчийг 1..8 утгаар
// шинэчилнэ. Чиглэл бүрд тохирох M1, M2, M3, M4-ийн хүчийг доор
// тодорхойлно. Утга нь -100..+100 хувь:
//    +100 = бүтэн урагш (тухайн чиглэлийн speed-ийн 100%)
//       0 = унтраа
//    -100 = бүтэн хойш
//    +50  = тал хурдаар урагш гэх мэт
//
// Чиглэл буруу гарвал ЗӨВХӨН энэ хүснэгтийн тоонуудыг өөрчилнө.
// driveSector функцыг хүрэх шаардлагагүй.
//
// Sector ID: 1=F  2=FR  3=R  4=BR  5=B  6=BL  7=L  8=FL
// ═══════════════════════════════════════════
int DIRECTION_TABLE[9][4] = {
//      m1     m2     m3     m4
  {     0,     0,     0,     0 },  // 0 = NONE  (зогсолт)
  {  +100,     0,  -100,     0 },  // 1 = F     M1+M3 эсрэг чигт (forward)
  {  +100,  +100,  -100,  -100 },  // 2 = FR    бүх 4 мотор
  {     0,  +100,     0,  -100 },  // 3 = R     M2+M4 эсрэг чигт (right strafe)
  {  -100,  +100,  +100,  -100 },  // 4 = BR    бүх 4 мотор
  {  -100,     0,  +100,     0 },  // 5 = B     M1+M3 эсрэг чигт (backward)
  {  -100,  -100,  +100,  +100 },  // 6 = BL    бүх 4 мотор
  {     0,  -100,     0,  +100 },  // 7 = L     M2+M4 эсрэг чигт (left strafe)
  {  +100,  -100,  -100,  +100 },  // 8 = FL    бүх 4 мотор
};

// ═══════════════════════════════════════════
// PIN CONFIGURATION
// ═══════════════════════════════════════════
// M1 = FL (Front-Left)
#define M1_RPWM   2
#define M1_LPWM   3
// M2 = FR (Front-Right)
#define M2_RPWM   4
#define M2_LPWM   5
// M3 = RR (Rear-Right)
#define M3_RPWM   6
#define M3_LPWM   7
// M4 = RL (Rear-Left)
#define M4_RPWM   8
#define M4_LPWM   9
// M5 = Auxiliary motor (R1 toggle / L1 hold) — 230 PWM
#define M5_RPWM   10
#define M5_LPWM   11
// M6 = Auxiliary motor (R2 — 3-state cycle: FWD → REV → OFF) — 200 PWM
#define M6_RPWM   12
#define M6_LPWM   13

#define PS2_DAT  24
#define PS2_CMD  22
#define PS2_SEL  28
#define PS2_CLK  26
#define PRESSURES   false
#define RUMBLE      false

// ═══════════════════════════════════════════
// CONSTANTS
// ═══════════════════════════════════════════
const int DEADZONE       = 40;    // ↑ Стикийн drift/шумыг залгих (өмнө 25 байсан)
const int MIN_STICK_SPEED = 30;   // Энээс бага хурдыг тэг болгох (мотор зэвэрэхээс хамгаалах)
const int ROTATION_SPEED = 30;
const int SPEED_MIN      = 0;
const int SPEED_MAX      = 255;
const int DEFAULT_SPEED  = 80;
const int SPEED_STEP     = 20;
const int TEST_SPEED     = 150;
const int M5_SPEED       = 230;   // M5 туслах моторын ажиллах PWM
const int M6_SPEED       = 230;   // M6 туслах моторын ажиллах PWM
const unsigned long PS2_READ_INTERVAL  = 50;
const unsigned long TEST_PHASE_MS      = 1000;

// ═══════════════════════════════════════════
// DIRECTION STATE (D-Pad)
// ═══════════════════════════════════════════
enum Direction {
  DIR_NONE  = 0,
  DIR_UP    = 1,
  DIR_DOWN  = 2,
  DIR_LEFT  = 3,
  DIR_RIGHT = 4
};

// ═══════════════════════════════════════════
// 8-SECTOR DIRECTION (Left Stick)
// ═══════════════════════════════════════════
//  activeSector хувьсагч 1..8 утгатай. 0 = чиглэлгүй (зогссон).
//  1=F   2=FR   3=R   4=BR   5=B   6=BL   7=L   8=FL
enum Sector {
  SEC_NONE = 0,
  SEC_F    = 1,
  SEC_FR   = 2,
  SEC_R    = 3,
  SEC_BR   = 4,
  SEC_B    = 5,
  SEC_BL   = 6,
  SEC_L    = 7,
  SEC_FL   = 8
};

// ═══════════════════════════════════════════
// PAIR DRIVE MODE
// ═══════════════════════════════════════════
enum PairMode {
  PAIR_ALL    = 0,
  PAIR_M1_M3  = 1,    // diagonal: M1 (FL) + M3 (RR)
  PAIR_M2_M4  = 2     // diagonal: M2 (FR) + M4 (RL)
};

// ═══════════════════════════════════════════
// MOTOR TEST STATE
// ═══════════════════════════════════════════
enum TestMode {
  TEST_OFF       = 0,
  TEST_SEQUENCE  = 1,
  TEST_PAIRS     = 2
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

// Зүүн стикийн өмнөх sector (зөвхөн зөрөөтэй үед хэвлэх)
int       lastSector       = -1;

// 🎯 ЧИГЛЭЛ ТОДОРХОЙЛОГЧ ХУВЬСАГЧ (0=зогссон, 1..8=чиглэл)
//    1=F  2=FR  3=R  4=BR  5=B  6=BL  7=L  8=FL
int       activeSector     = 2;

// 5-р моторын toggle state (R1-ээр)
bool      m5Toggled        = false;

// 6-р моторын циклийн state (R2-ээр)
//   0 = STOP, 1 = CW (+M6_SPEED), 2 = STOP, 3 = CCW (-M6_SPEED)
//   Цикл: STOP → CW → STOP → CCW → STOP → CW → ... (чиглэл солихоос өмнө зогсоно)
int       m6State          = 0;

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
// LOW-LEVEL MOTOR
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
// 🎯 4-МОТОРЫГ ХАМТ УДИРДАХ ЦЭВЭР API
// ═══════════════════════════════════════════
// Параметрүүд: m1, m2, m3, m4 (тойроод цагийн зүүний дагуу)
void writeFourMotors(int m1, int m2, int m3, int m4) {
  // Pair mode-аар хэрэгцээгүй моторуудыг 0 болгоно
  switch (pairMode) {
    case PAIR_M1_M3: m2 = 0; m4 = 0; break;
    case PAIR_M2_M4: m1 = 0; m3 = 0; break;
    case PAIR_ALL:
    default: break;
  }

  // Offset
  m1 = applyOffset(m1, MOTOR_OFFSET_M1);
  m2 = applyOffset(m2, MOTOR_OFFSET_M2);
  m3 = applyOffset(m3, MOTOR_OFFSET_M3);
  m4 = applyOffset(m4, MOTOR_OFFSET_M4);

  // Invert
  m1 = applyInvert(m1, MOTOR_INVERT_M1);
  m2 = applyInvert(m2, MOTOR_INVERT_M2);
  m3 = applyInvert(m3, MOTOR_INVERT_M3);
  m4 = applyInvert(m4, MOTOR_INVERT_M4);

  driveMotor(M1_RPWM, M1_LPWM, m1);
  driveMotor(M2_RPWM, M2_LPWM, m2);
  driveMotor(M3_RPWM, M3_LPWM, m3);
  driveMotor(M4_RPWM, M4_LPWM, m4);
}

// Single motor (test-д ашиглана)
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
    case 0: return "M1 (FL)";
    case 1: return "M2 (FR)";
    case 2: return "M3 (RR)";
    case 3: return "M4 (RL)";
  }
  return "?";
}

// ═══════════════════════════════════════════
// 🎯 8-SECTOR DRIVE — зүүн стикийн чиглэл бүрд тохирох мотор pattern
// ═══════════════════════════════════════════
//
//  Cardinal (зөвхөн 2 мотор):
//     F  → M1 + M3            B  → M1 + M3 (эсрэг)
//     R  → M2 + M4            L  → M2 + M4 (эсрэг)
//
//  Diagonal (бүх 4 мотор — superposition):
//     FR = F + R              FL = F + L
//     BR = B + R              BL = B + L
//
// ═══════════════════════════════════════════
// 🎯 8-SECTOR DRIVE — DIRECTION_TABLE-аас pattern уншина
// ═══════════════════════════════════════════
//
//  sector: 0 = зогсолт,  1..8 = чиглэл (1=F, 2=FR, ..., 8=FL)
//  speed: 0..255
//
//  Бүх мотор pattern нь файлын дээд талын DIRECTION_TABLE-д байна.
//  driveSector функцийг өөрчлөх ШААРДЛАГАГҮЙ — зөвхөн хүснэгт засна.
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
    case 0: return "NONE";
    case 1: return "F";
    case 2: return "FR";
    case 3: return "R";
    case 4: return "BR";
    case 5: return "B";
    case 6: return "BL";
    case 7: return "L";
    case 8: return "FL";
  }
  return "?";
}

// ═══════════════════════════════════════════
// 🌀 SPIN — БҮХ 4 МОТОР АДИЛ ЧИГЛЭЛД
// ═══════════════════════════════════════════
void spinAll(int speed) {
  writeFourMotors(speed, speed, speed, speed);
}

// ═══════════════════════════════════════════
// 🚗 ЗӨВХӨН 2 МОТОР — D-PAD
// ═══════════════════════════════════════════
// Forward/Backward → M1 (FL) + M3 (RR) diagonal
// M1 ба M3 нь физикээр ЭСРЭГ чигт эргэнэ → spin биш, чигээр явах
void driveTwoFwdBwd(int speed) {
  writeFourMotors(speed, 0, -speed, 0);
}

// Strafe → M2 (FR) + M4 (RL) diagonal
void driveTwoStrafe(int speed) {
  // M1=0, M2=+speed, M3=0, M4=-speed
  writeFourMotors(0, speed, 0, -speed);
}

// ═══════════════════════════════════════════
// PUBLIC MOTION API
// ═══════════════════════════════════════════
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
// ⚙️ 5-Р МОТОР (M5) — ТУСЛАХ МОТОР (D10 / D11)
// ═══════════════════════════════════════════
//   R1  → toggle ON/OFF (нэг дарвал асна, дахин дарвал унтарна)
//   L1  → hold while pressed (барьж байх үед асна, тавихаар унтарна)
//   Аль аль нь үнэн → асна (хоёр аргыг зэрэг ашиглаж болно)
//   Робот OFF буюу TEST үед үргэлж унтрана.
// ═══════════════════════════════════════════
void driveM5(int speed) {
  speed = constrain(speed, -SPEED_MAX, SPEED_MAX);
  if (speed > 0) {
    analogWrite(M5_RPWM, speed);
    analogWrite(M5_LPWM, 0);
  } else if (speed < 0) {
    analogWrite(M5_RPWM, 0);
    analogWrite(M5_LPWM, -speed);
  } else {
    analogWrite(M5_RPWM, 0);
    analogWrite(M5_LPWM, 0);
  }
}

void stopM5() {
  m5Toggled = false;
  driveM5(0);
}

void handleM5() {
  // R1 → toggle (зөвхөн rising edge — дарах мөчид нэг л удаа)
  if (ps2x.ButtonPressed(PSB_R1)) {
    m5Toggled = !m5Toggled;
    Serial.print(F("⚙️ M5 toggle: "));
    Serial.println(m5Toggled ? F("ON") : F("OFF"));
  }
  // L1 → дараастай үед үргэлж асна
  bool m5HoldL1 = ps2x.Button(PSB_L1);

  bool m5On = m5Toggled || m5HoldL1;
  driveM5(m5On ? M5_SPEED : 0);
}

// ═══════════════════════════════════════════
// 🔁 6-Р МОТОР (M6) — ТУСЛАХ МОТОР (D12 / D13)
// ═══════════════════════════════════════════
//   R2 → 4 төлөвт цикл:
//     1-р дарах: CW   (+200 PWM, нар зөв)
//     2-р дарах: STOP (зогсоно)
//     3-р дарах: CCW  (-200 PWM, нар буруу)
//     4-р дарах: STOP (зогсоно)
//     5-р дарах: CW дахин эхэлнэ
//   → Чиглэл солихоос өмнө мотор зогсох нь аюулгүй (мех. шок багатай)
//   Робот OFF буюу TEST үед үргэлж унтрана.
// ═══════════════════════════════════════════
void driveM6(int speed) {
  speed = constrain(speed, -SPEED_MAX, SPEED_MAX);
  if (speed > 0) {
    analogWrite(M6_RPWM, speed);
    analogWrite(M6_LPWM, 0);
  } else if (speed < 0) {
    analogWrite(M6_RPWM, 0);
    analogWrite(M6_LPWM, -speed);
  } else {
    analogWrite(M6_RPWM, 0);
    analogWrite(M6_LPWM, 0);
  }
}

void stopM6() {
  m6State = 0;
  driveM6(0);
}

void handleM6() {
  // R2 → 4 төлөв цикл (rising edge)
  // 0=STOP → 1=CW → 2=STOP → 3=CCW → 0=STOP ...
  if (ps2x.ButtonPressed(PSB_R2)) {
    m6State = (m6State + 1) % 4;
    Serial.print(F("🔁 M6 cycle: "));
    switch (m6State) {
      case 0: Serial.println(F("STOPPED"));         break;
      case 1: Serial.println(F("CW (+200)"));       break;
      case 2: Serial.println(F("STOPPED"));         break;
      case 3: Serial.println(F("CCW (-200)"));      break;
    }
  }

  // Одоогийн state-д тохирох хурд
  // зөвхөн state 1 ба 3-д мотор ажиллана, 0 ба 2-т зогссон
  int m6Speed = 0;
  if      (m6State == 1) m6Speed = +M6_SPEED;
  else if (m6State == 3) m6Speed = -M6_SPEED;
  // state 0, 2 → m6Speed = 0 (зогссон)
  driveM6(m6Speed);
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

  // L1 → M1 (FL),  R1 → M2 (FR),  R2 → M3 (RR),  L2 → M4 (RL)
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
// 🧪 MOTOR TEST COMBOS
// ═══════════════════════════════════════════
bool handleTestCombos() {
  if (ps2x.Button(PSB_SELECT) && ps2x.ButtonPressed(PSB_START)) {
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
      if (testMode == TEST_SEQUENCE) {
        Serial.print(motorName(testStep));
      } else {
        Serial.print(testStep == 0 ? F("M1+M3") : F("M2+M4"));
      }
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
      // M1 + M3
      driveSingleMotor(0, speed);
      driveSingleMotor(1, 0);
      driveSingleMotor(2, speed);
      driveSingleMotor(3, 0);
    } else {
      // M2 + M4
      driveSingleMotor(0, 0);
      driveSingleMotor(1, speed);
      driveSingleMotor(2, 0);
      driveSingleMotor(3, speed);
    }
  }
  currentState = "TESTING";
}

// ═══════════════════════════════════════════
// D-PAD HANDLER (зөвхөн 2 мотор)
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
//  Зүүн стик: 8 чиглэл (өнцөг → sector, magnitude → speed)
//  Баруун стик: spin (бүх 4 мотор адил чиглэлд)
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

  // ━━━ Spin → 4 мотор адил чиглэлд (өмнөх логик хэвээр) ━━━
  if (RX != 0) {
    int spinSpeed = (RX > 0) ? ROTATION_SPEED : -ROTATION_SPEED;
    spinAll(spinSpeed);
    currentState = (RX > 0) ? "SPIN_CW_ALL" : "SPIN_CCW_ALL";
    lastSector = -1;
    activeSector = 0;   // spin нь чиглэл биш
    return true;
  }

  // ━━━ Translation → 8 чиглэл (зөвхөн зүүн стик) ━━━
  // 1) Magnitude → speed (хэр их тулгасанаас хамаарна)
  long magSq = (long)LX * LX + (long)LY * LY;
  int  mag   = (int)sqrt((float)magSq);
  if (mag > 128) mag = 128;
  int  speed = (mag * currentSpeed) / 128;

  // ⛔ Хэт жижиг дохио буюу мотор хөдлөх босгоос доош байвал зогсооно
  //    (стикийн drift/шум ороход дугуй зэвэрэхээс сэргийлнэ)
  if (speed < MIN_STICK_SPEED) {
    lastSector = -1;
    activeSector = 0;
    return false;
  }

  // 2) Angle → sector (1..8). 0 = чиглэлгүй.
  //    atan2(LX, LY): LY=урагш+ → 0°, LX=баруун+ → 90° (clockwise)
  //    Дараалал: 1=F, 2=FR, 3=R, 4=BR, 5=B, 6=BL, 7=L, 8=FL
  float angleDeg = atan2((float)LX, (float)LY) * 180.0f / (float)PI;
  if (angleDeg < 0) angleDeg += 360.0f;

  int sector = (((int)((angleDeg + 22.5f) / 45.0f)) % 8) + 1;

  // 3) Чиглэл бүрд тохирох мотор pattern гүйцэтгэх
  //    (driveSector нь activeSector-ыг шинэчилнэ)
  driveSector(sector, speed);
  currentState = sectorName(sector);

  // 4) Sector солигдсон үед serial-д товчхон мэдээлнэ
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
    stopM5();
    stopM6();
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
    driveM5(0);
    driveM6(0);
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
    driveM5(0);
    driveM6(0);
    activeSector = 0;
    return;
  }

  // M5 / M6 control (R1 toggle, L1 hold, R2 cycle) — combo-оос ӨМНӨ дуудаж байна
  // R1, R2-ийн rising edge зөв уншигдахын тулд
  handleM5();
  handleM6();

  bool calibrated  = handleCalibration();
  bool paired      = handlePairCombos();
  bool testStarted = handleTestCombos();
  if (calibrated || paired || testStarted) return;

  handleSpeedButtons();

  bool moving = false;
  if (handleDPad()) {
    moving = true;
    lastSector = -1;
    activeSector = 0;   // D-Pad нь sector систем биш
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
  safePinInit(M1_RPWM); safePinInit(M1_LPWM);
  safePinInit(M2_RPWM); safePinInit(M2_LPWM);
  safePinInit(M3_RPWM); safePinInit(M3_LPWM);
  safePinInit(M4_RPWM); safePinInit(M4_LPWM);
  safePinInit(M5_RPWM); safePinInit(M5_LPWM);
  safePinInit(M6_RPWM); safePinInit(M6_LPWM);

  delay(200);
  Serial.begin(115200);
  delay(300);

  Serial.println(F("\n═══════════════════════════════════════"));
  Serial.println(F("  R2bot — 8-direction Stick + 2-motor D-Pad"));
  Serial.println(F("  Motor numbering: clockwise"));
  Serial.println(F("═══════════════════════════════════════\n"));

  initPS2();
  if (ps2_error == 0) {
    Serial.print(F("Controller type: ")); Serial.println(ps2_type);
  }

  Serial.println(F("\n━━━ MOTOR LAYOUT (clockwise) ━━━"));
  Serial.println(F("  M1=FL (D2,D3)   M2=FR (D4,D5)"));
  Serial.println(F("  M4=RL (D8,D9)   M3=RR (D6,D7)"));

  Serial.println(F("\n━━━ INVERT ━━━"));
  Serial.print(F("  M1: ")); Serial.print(MOTOR_INVERT_M1 ? F("INV") : F("---"));
  Serial.print(F("  M2: ")); Serial.print(MOTOR_INVERT_M2 ? F("INV") : F("---"));
  Serial.print(F("  M3: ")); Serial.print(MOTOR_INVERT_M3 ? F("INV") : F("---"));
  Serial.print(F("  M4: ")); Serial.println(MOTOR_INVERT_M4 ? F("INV") : F("---"));

  Serial.println(F("\n━━━ CONTROLS ━━━"));
  Serial.println(F("  START         → ON/OFF"));
  Serial.println(F("  Left stick    → 8-direction (F/FR/R/BR/B/BL/L/FL)"));
  Serial.println(F("                  cardinal=2 motors, diagonal=4 motors"));
  Serial.println(F("                  magnitude → speed"));
  Serial.println(F("  Right stick   → 4-motor SPIN (same dir)"));
  Serial.println(F("  D-Pad ⬆⬇      → 2-motor (M1+M3)"));
  Serial.println(F("  D-Pad ⬅➡      → 2-motor (M2+M4)"));
  Serial.println(F("  △+20 ×−20 ○MAX □MIN"));

  Serial.println(F("\n━━━ ⚙️ M5 (D10/D11) ━━━"));
  Serial.println(F("  R1 → toggle ON/OFF (230 PWM)"));
  Serial.println(F("  L1 → hold while pressed"));

  Serial.println(F("\n━━━ 🔁 M6 (D12/D13) ━━━"));
  Serial.println(F("  R2 → cycle: STOP → CW → STOP → CCW → STOP (200 PWM)"));

  Serial.println(F("\n━━━ 🌀 PAIR ━━━"));
  Serial.println(F("  L1+L2 → M1+M3 / M2+M4 toggle"));
  Serial.println(F("  R1+R2 → all 4"));

  Serial.println(F("\n━━━ 🧪 TEST ━━━"));
  Serial.println(F("  SELECT+START → M1→M2→M3→M4"));
  Serial.println(F("  SELECT+△     → pair test"));
  Serial.println(F("  ○            → cancel"));

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
