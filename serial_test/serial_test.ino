
HardwareSerial Serial1(PB7, PB6);

union FloatBytes {
  uint8_t asBytes[4];
  float asFloat;
};

FloatBytes float1;
FloatBytes float2;

void setup() {

  

  Serial.begin(115200);
  Serial1.begin(115200);

  Serial.println("Setup");
  Serial.println("Very Complete.");


//  delay(1000);

}



void loop() {

  if (Serial.available() > 0) {
    Serial.println("Reading new float from PC...");
    float1.asFloat = Serial.parseFloat();
    Serial.println(float1.asFloat);
    for (int i = 0; i < 4; i ++) {
      Serial1.write(float1.asBytes[i]);
      float2.asBytes[i] = float1.asBytes[i];
    }
    Serial.println(float2.asFloat);
  }
  if (Serial1.available() >= 4) {
    Serial.println("Reading new float from other MCU!");
    for (int i = 0; i < 4; i ++) {
      float1.asBytes[i] = Serial1.read();
    }
    Serial.println(float1.asFloat);
  }
}
