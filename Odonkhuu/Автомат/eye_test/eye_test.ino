// E18-D80NK мэдрэгчүүд — Arduino Mega
const int sensorPins[] = {22, 23, 24, 25, 26, 27, 28, 29};
const int N = sizeof(sensorPins) / sizeof(sensorPins[0]);

void setup() {
  Serial.begin(9600);
  for (int i = 0; i < N; i++) {
    pinMode(sensorPins[i], INPUT_PULLUP);
  }
  Serial.println(F("=== E18-D80NK мэдрэгчийн тест ==="));
  Serial.println(F("[X] = илрүүлсэн   [ ] = алга"));
  Serial.println(F("---------------------------------"));
}

void loop() {
  for (int i = 0; i < N; i++) {
    int state = digitalRead(sensorPins[i]);

    Serial.print(F("S"));
    Serial.print(i + 1);
    Serial.print(F("(p"));
    Serial.print(sensorPins[i]);
    Serial.print(F("):"));
    Serial.print(state == LOW ? F("[X]") : F("[ ]"));
    Serial.print(F("  "));
  }
  Serial.println();
  delay(200);
}