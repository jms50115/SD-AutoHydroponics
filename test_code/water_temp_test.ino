const int tempPin = A0;      // Analog pin connected to Surveyor "A"
const float Vref = 5.0;     // Arduino Mega analog reference (default = 5V)

void setup() {
  Serial.begin(9600);
  Serial.println("Surveyor RTD Temperature Readings");
}

void loop() {
  int rawADC = analogRead(tempPin);

  // Convert ADC reading to voltage
  float voltage = rawADC * (Vref / 1023.0);

  // Convert voltage to temperature (°C)
  float temperatureC = (voltage - 1.058) / 0.009;

  // Print to Serial Monitor
  Serial.print("Voltage: ");
  Serial.print(voltage, 3);
  Serial.print(" V | Temperature: ");
  Serial.print(temperatureC, 2);
  Serial.println(" °C");

  delay(1000); // 1 second interval
}