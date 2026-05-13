// 42GM-4260 BLDC мотор + Arduino UNO
// Цагаан утас → Pin 8 (чиглэл)
// Цэнхэр утас → Pin 9 (PWM хурд)

const int DIR_PIN = 8;
const int PWM_PIN = 9;

void setup() {
  pinMode(DIR_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);

  Serial.begin(9600);
  Serial.println("Мотор тест эхэллээ");
}

void loop() {
  // CW чиглэлд 50% хурдаар 3 сек
  Serial.println("CW 50%");
  digitalWrite(DIR_PIN, LOW);
  analogWrite(PWM_PIN, 128);
  delay(3000);

  // Зогсох
  Serial.println("Зогсох");
  analogWrite(PWM_PIN, 0);
  delay(500);

  // CCW чиглэлд бүтэн хурдаар 3 сек
  Serial.println("CCW 100%");
  digitalWrite(DIR_PIN, HIGH);
  analogWrite(PWM_PIN, 255);
  delay(3000);

  // Зогсох
  Serial.println("Зогсох");
  analogWrite(PWM_PIN, 0);
  delay(500);
}