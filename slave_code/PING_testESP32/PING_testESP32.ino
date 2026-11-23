void setup() {
  Serial.begin(115200);
  delay(1000); // Give the serial monitor a second to connect
  Serial.println("ESP32 Connected!");
}

void loop() {
  Serial.println("Ping");
  delay(1000); // Wait 1 second between pings
}