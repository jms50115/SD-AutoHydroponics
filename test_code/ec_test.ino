void setup() {
  Serial.begin(9600);     // USB console
  Serial1.begin(9600);    // EZO-EC UART (RX1 / TX1)

  delay(1000);            // Allow EZO-EC to boot

  Serial.println("EC Logger Started");
  Serial.println("Units: uS/cm");
}

void loop() {
  // Read incoming EC data from EZO-EC
  if (Serial1.available()) {
    String ecReading = Serial1.readStringUntil('\r');

    // Basic validation
    if (ecReading.length() > 0 && isDigit(ecReading[0])) {
      Serial.print("EC: ");
      Serial.print(ecReading);
      Serial.println(" uS/cm");
    }
  }
}