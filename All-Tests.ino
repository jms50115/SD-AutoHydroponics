#include <Wire.h>

// ================= USER SETTINGS =================
#define PRINT_FAHRENHEIT 1

// Measure your actual 5V rail if possible
const float VREF = 5.00;

// ---- RTD CALIBRATION (YOU MUST SET THESE) ----
// Start with these placeholders.
// After calibration, replace them with computed values.
float RTD_M = 100.0;   // °C per volt  (placeholder)
float RTD_B = -100.0;  // °C offset    (placeholder)

// ================= PINS / ADDRESSES =================
const int RTD_PIN = A0;
#define HTU21D_ADDR 0x40

// ================= LAST VALUES =================
float lastPH = NAN;
float lastEC = NAN;
float lastWaterTempC = NAN;
float lastAirTempC = NAN;
float lastHumidity = NAN;

// ================= SERIAL BUFFERS =================
String bufEC;
String bufPH;

// ================= TIMING =================
unsigned long lastPrintMs = 0;
const unsigned long PRINT_INTERVAL_MS = 1000;

// ================= UTILS =================
static bool isNumericStart(char c) {
  return (c >= '0' && c <= '9') || c == '-' || c == '+' || c == '.';
}

// ================= HTU CRC =================
static uint8_t htu_crc8(const uint8_t *data, int len) {
  uint8_t crc = 0x00;
  for (int i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

// ================= SETUP =================
void setup() {
  Serial.begin(9600);

  // Arduino Mega UARTs
  Serial1.begin(9600);   // EC
  Serial2.begin(9600);   // pH

  Wire.begin();
  delay(1200);

  // Put EZO boards into continuous mode
  Serial1.print("C,0\r"); delay(200);
  Serial2.print("C,0\r"); delay(200);
  Serial1.print("C,1\r"); delay(200);
  Serial2.print("C,1\r"); delay(200);

  Serial.println("LOGGER READY");
  Serial.println("EC=Serial1 | pH=Serial2 | RTD=A0 | HTU21D=I2C");
  Serial.println("FORMAT: pH | EC | Water | Air | RH");
}

// ================= LOOP =================
void loop() {
  pollEzoSerial(Serial1, bufEC, lastEC);
  pollEzoSerial(Serial2, bufPH, lastPH);

  unsigned long now = millis();
  if (now - lastPrintMs >= PRINT_INTERVAL_MS) {
    lastPrintMs = now;

    lastWaterTempC = readRTDTempC();

    float t, h;
    if (readHTU21D(t, h)) {
      lastAirTempC = t;
      lastHumidity = h;
    }

    Serial.print("pH: ");
    printVal(lastPH, 2);

    Serial.print(" | EC: ");
    printVal(lastEC, 0);
    Serial.print(" uS/cm");

    Serial.print(" | Water: ");
    printTemp(lastWaterTempC);

    Serial.print(" | Air: ");
    printTemp(lastAirTempC);

    Serial.print(" | RH: ");
    printVal(lastHumidity, 2);
    Serial.println(" %");
  }
}

// ================= EZO PARSER =================
void pollEzoSerial(HardwareSerial &port, String &buffer, float &lastValue) {
  while (port.available()) {
    char c = (char)port.read();

    if (c == '\r') {
      buffer.trim();
      if (buffer.length() && buffer != "*OK" && isNumericStart(buffer[0])) {
        lastValue = buffer.toFloat();
      }
      buffer = "";
    } else if (c != '\n') {
      buffer += c;
      if (buffer.length() > 40) buffer = "";
    }
  }
}

// ================= RTD =================
float readRTDTempC() {
  int raw = analogRead(RTD_PIN);
  float voltage = raw * (VREF / 1023.0f);

  // CALIBRATED linear conversion
  return RTD_M * voltage + RTD_B;
}

// ================= HTU21D =================
bool readHTU21D(float &tempC, float &rh) {
  float t = readHTU_TempC();
  float h = readHTU_RH();
  if (isnan(t) || isnan(h)) return false;
  tempC = t;
  rh = h;
  return true;
}

float readHTU_TempC() {
  Wire.beginTransmission(HTU21D_ADDR);
  Wire.write(0xF3);
  if (Wire.endTransmission() != 0) return NAN;

  delay(50);
  Wire.requestFrom(HTU21D_ADDR, 3);
  if (Wire.available() < 3) return NAN;

  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  uint8_t crc = Wire.read();

  uint8_t buf[2] = { msb, lsb };
  if (htu_crc8(buf, 2) != crc) return NAN;

  uint16_t raw = ((uint16_t)msb << 8) | lsb;
  raw &= 0xFFFC;

  return -46.85f + 175.72f * ((float)raw / 65536.0f);
}

float readHTU_RH() {
  Wire.beginTransmission(HTU21D_ADDR);
  Wire.write(0xF5);
  if (Wire.endTransmission() != 0) return NAN;

  delay(50);
  Wire.requestFrom(HTU21D_ADDR, 3);
  if (Wire.available() < 3) return NAN;

  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  uint8_t crc = Wire.read();

  uint8_t buf[2] = { msb, lsb };
  if (htu_crc8(buf, 2) != crc) return NAN;

  uint16_t raw = ((uint16_t)msb << 8) | lsb;
  raw &= 0xFFFC;

  return -6.0f + 125.0f * ((float)raw / 65536.0f);
}

// ================= PRINT HELPERS =================
void printVal(float v, int d) {
  if (isnan(v)) Serial.print("--");
  else Serial.print(v, d);
}

void printTemp(float tempC) {
  if (isnan(tempC)) {
    Serial.print("--");
    return;
  }
#if PRINT_FAHRENHEIT
  Serial.print(tempC * 9.0f / 5.0f + 32.0f, 2);
  Serial.print(" F");
#else
  Serial.print(tempC, 2);
  Serial.print(" C");
#endif
}
