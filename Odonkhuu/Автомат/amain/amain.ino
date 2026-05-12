#define M1_B  2   // Мотор 1 — чиглэлийн pin (PWM биш)
#define M1_A  3   // Мотор 1 — PWM pin
#define M2_B  4   // Мотор 2 — чиглэлийн pin (PWM биш)
#define M2_A  5   // Мотор 2 — PWM pin

int urugshHurd = 200;  // 0–255
int ergeltHurd = 180;

void setup() {
  pinMode(M1_A, OUTPUT);
  pinMode(M1_B, OUTPUT);
  pinMode(M2_A, OUTPUT);
  pinMode(M2_B, OUTPUT);

  urugshYavah(2000, urugshHurd);
  zogsoh(500);

  baruunErgeh(600, ergeltHurd);
  zogsoh(500);

  urugshYavah(2000, urugshHurd);
  zogsoh(500);

  zuunErgeh(600, ergeltHurd);
  zogsoh(500);

  urugshYavah(2000, urugshHurd);
  zogsoh(0);
}

void loop() {}

// ---------- Мотор тус бүрийн функц ----------
// hurd: эерэг = урагш, сөрөг = ухрах, 0 = зогсоох

void M1(int hurd) {
  if (hurd > 0) {
    digitalWrite(M1_A, LOW);
    analogWrite(M1_B, hurd);
  } else if (hurd < 0) {
    digitalWrite(M1_A, HIGH);
    analogWrite(M1_B, 255 + hurd);
  } else {
    digitalWrite(M1_A, LOW);
    digitalWrite(M1_B, LOW);
  }
}

void M2(int hurd) {
  if (hurd > 0) {
    digitalWrite(M2_A, LOW);
    analogWrite(M2_B, hurd);
  } else if (hurd < 0) {
    digitalWrite(M2_A, HIGH);
    analogWrite(M2_B, 255 + hurd);
  } else {
    digitalWrite(M2_A, LOW);
    digitalWrite(M2_B, LOW);
  }
}

// ---------- Хөдөлгөөний функцууд ----------

void urugshYavah(int hugatsaa, int hurd) {
  M1(hurd);
  M2(hurd);
  delay(hugatsaa); 
}

void zogsoh(int hugatsaa) {
  M1(0);
  M2(0);
  delay(hugatsaa);
}

void baruunE  rgeh(int hugatsaa, int hurd) {
  M1(hurd);    // M1 урагш
  M2(-hurd);   // M2 ухарна
  delay(hugatsaa);
}

void zuunErgeh(int hugatsaa, int hurd) {
  M1(-hurd);   // M1 ухарна
  M2(hurd);    // M2 урагш
  delay(hugatsaa);
}