const int testPins[] = {30, 31, 33};

void setup() {
  Serial.begin(9600);
  for (int i = 0; i < 3; i++) pinMode(testPins[i], INPUT_PULLUP);
  Serial.println("Pin 30, 31, 33 тест. Товч дарж шалга.");
}

void loop() {
  for (int i = 0; i < 3; i++) {
    int val = digitalRead(testPins[i]);
    Serial.print("P");
    Serial.print(testPins[i]);
    Serial.print("=");
    Serial.print(val);
    Serial.print("  ");
  }
  Serial.println();
  delay(150);
}