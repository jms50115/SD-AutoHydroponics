#include <Wire.h>

#define HTU21D_ADDR 0x40
#define TEMP_PIN A0

const float Vref = 5.0;   // Arduino Mega ADC reference

void setup() {
  Serial.begin(9600);
  Wire.begin();
  delay(100);

  Serial.println("Logging Temperature (RTD) and Humidity (HTU21D)");
}

void loop() {
  // -------- Surveyor RTD Temperature --------
  int rawADC = analogRead(TEMP_PIN);
  float voltage = rawADC * (Vref / 1023.0);
  float tempRTD = (voltage - 1.058) / 0.009;   // From datasheet

  // -------- HTU21D Humidity --------
  float humidity = readHumidity();

  // -------- Output --------
  Serial.print("RTD Temp: ");
  Serial.print(tempRTD, 2);
  Serial.print(" °C | Humidity: ");
  Serial.print(humidity, 2);
  Serial.println(" %");

  delay(2000); // 2 seconds
}

// ---------------- HTU21D Functions ----------------

float readHumidity() {
  Wire.beginTransmission(HTU21D_ADDR);
  Wire.write(0xF5); // Trigger humidity measurement (No Hold Master)
  Wire.endTransmission();

  delay(20); // Max measurement time

  Wire.requestFrom(HTU21D_ADDR, 2);
  if (Wire.available() < 2) return NAN;

  uint16_t raw = Wire.read() << 8;
  raw |= Wire.read();
  raw &= 0xFFFC; // Clear status bits

  // RH formula from datasheet
  return -6.0 + 125.0 * ((float)raw / 65536.0);
}