/*
  Temporary pH + EC monitor
  For Arduino Mega 2560

  Assumes Atlas sensors are wired the same as your other program:
    pH sensor -> Serial1
    EC sensor -> Serial2

  On the PC side, open COM3 in Serial Monitor / Serial Plotter / any serial terminal.
*/

String phReading = "N/A";
String ecReading = "N/A";

void setup() {
  // USB serial to PC
  Serial.begin(115200);

  // Atlas EZO sensors in UART mode
  Serial1.begin(9600);   // pH
  Serial2.begin(9600);   // EC

  delay(2000);

  // Clear anything sitting in buffers
  clearBuffer(Serial1);
  clearBuffer(Serial2);

  Serial.println("Starting temporary pH/EC monitor...");
  Serial.println("Reading once per second.");
  Serial.println();
}

void loop() {
  // Ask both sensors for a reading at the same time
  sendCommand(Serial1, "R");
  sendCommand(Serial2, "R");

  // Wait long enough for both sensors to finish
  delay(1000);

  // Read responses
  phReading = readAtlasResponse(Serial1);
  ecReading = readAtlasResponse(Serial2);

  // Print to USB serial (this is what you see on COM3)
  Serial.print("pH: ");
  Serial.print(phReading);
  Serial.print("    EC: ");
  Serial.println(ecReading);

  // Small extra delay so output stays close to once per second visually
  delay(50);
}

void sendCommand(HardwareSerial &sensorPort, const char *cmd) {
  sensorPort.print(cmd);
  sensorPort.print('\r');
}

String readAtlasResponse(HardwareSerial &sensorPort) {
  String response = "";

  unsigned long start = millis();
  while (millis() - start < 300) {
    while (sensorPort.available()) {
      char c = sensorPort.read();

      // Atlas UART responses end with carriage return
      if (c == '\r' || c == '\n') {
        if (response.length() > 0) {
          response.trim();

          // Common Atlas response codes:
          // "*OK" or "OK" for success messages
          // readings are returned as numeric strings
          if (response == "*OK" || response == "OK") {
            return "OK";
          }

          return response;
        }
      } else {
        response += c;
      }
    }
  }

  response.trim();

  if (response.length() == 0) {
    return "No response";
  }

  return response;
}

void clearBuffer(HardwareSerial &sensorPort) {
  while (sensorPort.available()) {
    sensorPort.read();
  }
}