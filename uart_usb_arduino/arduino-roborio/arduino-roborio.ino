void setup() {
  Serial.begin(9600);
}

void loop() {
  if(Serial.available()) {
    byte value = Serial.read();
    if(value == 0x12) {
      Serial.println("Arduino Recived 0x12");
    }
  }
  delay(50);
}