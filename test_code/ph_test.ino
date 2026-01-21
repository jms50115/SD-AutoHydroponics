/*
  Atlas Scientific EZO-pH
  Arduino Mega
  Logs pH every 1 second to Serial Monitor
*/

#include <Arduino.h>

void setup() {
  Serial.begin(9600);     // USB serial monitor
  Serial1.begin(9600);    // EZO-pH UART (default baud)

  delay(1000);

  // Put EZO-pH into continuous reading mode (1 sec)
  Serial1.print("C,1\r");

  Serial.println("EZO-pH initialized");
  Serial.println("Logging pH every 1 second...");
}

void loop() {
  if (Serial1.available()) {
    String response = Serial1.readStringUntil('\r');
    response.trim();

    // Ignore empty lines and *OK messages
    if (response.length() > 0 && response != "*OK") {
      Serial.print("pH: ");
      Serial.println(response);
    }
  }
}