void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

char read_bytes[5];

void loop() {
  Serial.println(Serial.available());
  delay(250);
  Serial.readBytes(read_bytes, 5);
  Serial.println(read_bytes);
  Serial.println(Serial.available());
}
