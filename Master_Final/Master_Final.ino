void setup() {
  pinMode(25, OUTPUT); // o el LED que tenga tu RP2 Nano
}

void loop() {
  digitalWrite(25, !digitalRead(25));
  delay(500);
}
