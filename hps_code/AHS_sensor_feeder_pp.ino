#include <Arduino.h>
#include <math.h>
#include <Servo.h>
#include <Wire.h>
#include "Adafruit_HTU21DF.h"

const int TEMP_PIN = A0;
const float ADC_VREF = 5.0;

// =============================
// Actuator pins
// =============================
const int LIGHT_PIN = 7;

const int PUMP_PH_UP_PIN   = 22;
const int PUMP_PH_DOWN_PIN = 23;
const int PUMP_BASE_A_PIN  = 24;
const int PUMP_BASE_B_PIN  = 25;

// =============================
// Servo pump settings
// =============================
const int PUMP_STOP_CMD = 90;

// Change any of these to 180 if that pump runs the wrong direction
const int PUMP_PH_UP_RUN_CMD   = 0;
const int PUMP_PH_DOWN_RUN_CMD = 0;
const int PUMP_BASE_A_RUN_CMD  = 0;
const int PUMP_BASE_B_RUN_CMD  = 0;

// =============================
// Control timing
// =============================
const unsigned long CONTROL_INTERVAL_MS = 300000UL;   // 5 min
const unsigned long LOG_INTERVAL_MS     = 300000UL;   // 5 min
const unsigned long SENSOR_INTERVAL_MS  = 300000UL;   // 5 min
const unsigned long PUMP_GAP_MS         = 2000UL;
const unsigned long MANUAL_PUMP_RUN_MS  = 3000UL;

// Start small and calibrate
const unsigned long PH_DOSE_TIME_MS = 1500UL;
const unsigned long EC_DOSE_TIME_MS = 1500UL;

// Optional safety hard limits
const float ABSOLUTE_MIN_PH = 2.0;
const float ABSOLUTE_MAX_PH = 9.0;
const float ABSOLUTE_MAX_EC = 4.0;

// -----------------------------
// Plant profile data structure
// -----------------------------
struct PlantProfile {
  const char* name;
  float phMin;
  float phMax;
  float ecMin;
  float ecMax;
  float tempMin;
  float tempMax;
  int lightHours;
};

// Example plant profiles
PlantProfile plantProfiles[] = {
  {"Lettuce", 5.5, 6.5, 0.8, 1.2, 18.0, 24.0, 14},
  {"Basil",   5.5, 6.5, 1.0, 1.6, 20.0, 26.0, 14},
  {"Peppers", 5.8, 6.5, 2.0, 3.5, 20.0, 28.0, 14}
};

const int NUM_PLANTS = sizeof(plantProfiles) / sizeof(plantProfiles[0]);

// Current active plant profile
PlantProfile currentProfile = {"Lettuce", 5.5, 6.5, 0.8, 1.2, 18.0, 24.0, 14};

// =============================
// Light mode state
// =============================
enum LightMode {
  LIGHT_MODE_AUTO,
  LIGHT_MODE_MANUAL_ON
};

LightMode lightMode = LIGHT_MODE_AUTO;
bool timerWantsLightOn = false;
bool lightOn = false;

// =============================
// Runtime state
// =============================
unsigned long lastControlTime = 0;
unsigned long lastLogTime = 0;
unsigned long lastSensorTime = 0;

// cached latest readings
float lastTempC = NAN;
float lastPH = NAN;
float lastEC = NAN;

// HTU21DF variables
Adafruit_HTU21DF htu = Adafruit_HTU21DF();
float airTempC = NAN;
float humidity = NAN;

// last action text for dashboard/debug
String lastAction = "none";

// dosing info for current logging interval
String intervalDoseSolution = "no solution";
String intervalDoseMode = "no dose";

// =============================
// Pump servo objects
// =============================
Servo pumpPHUp;
Servo pumpPHDown;
Servo pumpBaseA;
Servo pumpBaseB;

// Forward declarations
void sendSensorAndStatusJson(float tempC, float ph, float ec, float airTempC, float humidity);
void sendLightStatusJson();

// -----------------------------
// Relay helpers (LIGHT ONLY)
// -----------------------------
void relayOn(int pin) {
  digitalWrite(pin, HIGH);
}

void relayOff(int pin) {
  digitalWrite(pin, LOW);
}

// -----------------------------
// Pump helpers
// -----------------------------
void stopPump(Servo &pump) {
  pump.write(PUMP_STOP_CMD);
}

void runPump(Servo &pump, int runCmd) {
  pump.write(runCmd);
}

void dosePump(Servo &pump, int runCmd, unsigned long durationMs) {
  runPump(pump, runCmd);
  delay(durationMs);
  stopPump(pump);
}

void recordDoseEvent(const char* solution, const char* mode) {
  intervalDoseSolution = solution;
  intervalDoseMode = mode;
}

void clearDoseEvent() {
  intervalDoseSolution = "no solution";
  intervalDoseMode = "no dose";
}

// -----------------------------
// Temperature reading
// -----------------------------
float readTemperatureC() {
  int adc = analogRead(TEMP_PIN);
  float voltage = (adc * ADC_VREF) / 1023.0;
  float tempC = (voltage - 1.058) / 0.009;
  return tempC;
}

// -----------------------------
// EZO UART helpers
// -----------------------------
void clearPort(HardwareSerial &port) {
  while (port.available() > 0) port.read();
}

void sendEzoCmd(HardwareSerial &port, const char *cmd) {
  clearPort(port);
  port.print(cmd);
  port.print("\r");
}

String readEzoLine(HardwareSerial &port, unsigned long timeout_ms = 1500) {
  String line = "";
  unsigned long start = millis();

  while (millis() - start < timeout_ms) {
    while (port.available() > 0) {
      char c = port.read();
      if (c == '\r') {
        line.trim();
        return line;
      }
      if (c != '\n') line += c;
    }
  }

  line.trim();
  return line;
}

String readEzoReading(HardwareSerial &port) {
  sendEzoCmd(port, "R");
  delay(950);

  String resp = readEzoLine(port);
  resp.trim();

  if (resp.length() == 0) return "";
  if (resp == "OK") return "";
  return resp;
}

float toFloatOrNaN(const String &s) {
  if (s.length() == 0) return NAN;
  return s.toFloat();
}

// -----------------------------
// Light helpers
// -----------------------------
const char* getLightModeString() {
  if (lightMode == LIGHT_MODE_MANUAL_ON) return "MANUAL_ON";
  return "AUTO";
}

const char* getLightStatusString() {
  if (lightMode == LIGHT_MODE_MANUAL_ON && lightOn) return "on_manual_override";
  if (lightMode == LIGHT_MODE_AUTO && lightOn) return "on_timer";
  return "off_timer";
}

void applyLightOutput() {
  bool shouldBeOn = false;

  if (lightMode == LIGHT_MODE_MANUAL_ON) {
    shouldBeOn = true;
  } else {
    shouldBeOn = timerWantsLightOn;
  }

  if (shouldBeOn) {
    relayOn(LIGHT_PIN);
    lightOn = true;
  } else {
    relayOff(LIGHT_PIN);
    lightOn = false;
  }
}

void setLightModeAuto() {
  lightMode = LIGHT_MODE_AUTO;
  applyLightOutput();
  lastAction = lightOn ? "light_auto_on_timer" : "light_auto_off_timer";
  sendLightStatusJson();
}

void setLightModeManualOn() {
  lightMode = LIGHT_MODE_MANUAL_ON;
  applyLightOutput();
  lastAction = "light_manual_on";
  sendLightStatusJson();
}

void setTimerLightState(bool on) {
  timerWantsLightOn = on;

  if (lightMode == LIGHT_MODE_AUTO) {
    applyLightOutput();
    lastAction = on ? "light_timer_on" : "light_timer_off";
  } else {
    lastAction = on ? "timer_on_ignored_manual" : "timer_off_ignored_manual";
  }

  sendLightStatusJson();
}

// -----------------------------
// Plant profile handling
// -----------------------------
bool setPlantProfileByName(String plantName) {
  plantName.trim();

  for (int i = 0; i < NUM_PLANTS; i++) {
    if (plantName.equalsIgnoreCase(plantProfiles[i].name)) {
      currentProfile = plantProfiles[i];

      lightMode = LIGHT_MODE_AUTO;
      timerWantsLightOn = false;
      applyLightOutput();

      lastAction = "plant_changed";
      return true;
    }
  }
  return false;
}

void sendCurrentProfileJson() {
  Serial.print("{\"status\":\"plant_set\",\"plant\":\"");
  Serial.print(currentProfile.name);
  Serial.print("\",\"phMin\":");
  Serial.print(currentProfile.phMin, 2);
  Serial.print(",\"phMax\":");
  Serial.print(currentProfile.phMax, 2);
  Serial.print(",\"ecMin\":");
  Serial.print(currentProfile.ecMin, 2);
  Serial.print(",\"ecMax\":");
  Serial.print(currentProfile.ecMax, 2);
  Serial.print(",\"tempMin\":");
  Serial.print(currentProfile.tempMin, 2);
  Serial.print(",\"tempMax\":");
  Serial.print(currentProfile.tempMax, 2);
  Serial.print(",\"lightHours\":");
  Serial.print(currentProfile.lightHours);
  Serial.print(",\"lightMode\":\"");
  Serial.print(getLightModeString());
  Serial.print("\",\"lightStatus\":\"");
  Serial.print(getLightStatusString());
  Serial.println("\"}");
}

// -----------------------------
// Pump control
// -----------------------------
void dosePHUpAuto() {
  dosePump(pumpPHUp, PUMP_PH_UP_RUN_CMD, PH_DOSE_TIME_MS);
  recordDoseEvent("pH Up", "automated dose");
  lastAction = "dose_ph_up";
}

void dosePHDownAuto() {
  dosePump(pumpPHDown, PUMP_PH_DOWN_RUN_CMD, PH_DOSE_TIME_MS);
  recordDoseEvent("pH Down", "automated dose");
  lastAction = "dose_ph_down";
}

void doseBaseAAuto() {
  dosePump(pumpBaseA, PUMP_BASE_A_RUN_CMD, EC_DOSE_TIME_MS);
  recordDoseEvent("Base A", "automated dose");
  lastAction = "dose_base_a";
}

void doseBaseBAuto() {
  dosePump(pumpBaseB, PUMP_BASE_B_RUN_CMD, EC_DOSE_TIME_MS);
  recordDoseEvent("Base B", "automated dose");
  lastAction = "dose_base_b";
}

void doseNutrientsAuto() {
  dosePump(pumpBaseA, PUMP_BASE_A_RUN_CMD, EC_DOSE_TIME_MS);
  delay(PUMP_GAP_MS);
  dosePump(pumpBaseB, PUMP_BASE_B_RUN_CMD, EC_DOSE_TIME_MS);

  recordDoseEvent("Base A + Base B", "automated dose");
  lastAction = "dose_base_a_b";
}

void dosePHUpManual() {
  dosePump(pumpPHUp, PUMP_PH_UP_RUN_CMD, PH_DOSE_TIME_MS);
  recordDoseEvent("pH Up", "manual dose");
  lastAction = "manual_dose_ph_up";
}

void dosePHDownManual() {
  dosePump(pumpPHDown, PUMP_PH_DOWN_RUN_CMD, PH_DOSE_TIME_MS);
  recordDoseEvent("pH Down", "manual dose");
  lastAction = "manual_dose_ph_down";
}

void doseBaseAManual() {
  dosePump(pumpBaseA, PUMP_BASE_A_RUN_CMD, EC_DOSE_TIME_MS);
  recordDoseEvent("Base A", "manual dose");
  lastAction = "manual_dose_a";
}

void doseBaseBManual() {
  dosePump(pumpBaseB, PUMP_BASE_B_RUN_CMD, EC_DOSE_TIME_MS);
  recordDoseEvent("Base B", "manual dose");
  lastAction = "manual_dose_b";
}

void runPHUpManual5s() {
  dosePump(pumpPHUp, PUMP_PH_UP_RUN_CMD, MANUAL_PUMP_RUN_MS);
  recordDoseEvent("pH Up", "manual dose");
  lastAction = "manual_run_ph_up_5s";
}

void runPHDownManual5s() {
  dosePump(pumpPHDown, PUMP_PH_DOWN_RUN_CMD, MANUAL_PUMP_RUN_MS);
  recordDoseEvent("pH Down", "manual dose");
  lastAction = "manual_run_ph_down_5s";
}

void runBaseAManual5s() {
  dosePump(pumpBaseA, PUMP_BASE_A_RUN_CMD, MANUAL_PUMP_RUN_MS);
  recordDoseEvent("Base A", "manual dose");
  lastAction = "manual_run_base_a_5s";
}

void runBaseBManual5s() {
  dosePump(pumpBaseB, PUMP_BASE_B_RUN_CMD, MANUAL_PUMP_RUN_MS);
  recordDoseEvent("Base B", "manual dose");
  lastAction = "manual_run_base_b_5s";
}

// -----------------------------
// Dosing control
// -----------------------------
void updateDosingControl(float ph, float ec) {
  if (!isnan(ph)) {
    if (ph < ABSOLUTE_MIN_PH || ph > ABSOLUTE_MAX_PH) {
      lastAction = "ph_out_of_absolute_range";
      return;
    }

    if (ph < currentProfile.phMin) {
      dosePHUpAuto();
      return;
    }

    if (ph > currentProfile.phMax) {
      dosePHDownAuto();
      return;
    }
  }

  if (!isnan(ec)) {
    if (ec > ABSOLUTE_MAX_EC) {
      lastAction = "ec_above_absolute_max";
      return;
    }

    if (ec < currentProfile.ecMin) {
      doseNutrientsAuto();
      return;
    }

    if (ec > currentProfile.ecMax) {
      lastAction = "ec_high_dilution_needed";
      return;
    }
  }

  lastAction = "stable";
}

// -----------------------------
// Incoming command parsing
// -----------------------------
void checkForNodeRedCommands() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.startsWith("PLANT:")) {
      String plantName = cmd.substring(6);
      plantName.trim();

      bool ok = setPlantProfileByName(plantName);

      if (ok) {
        sendCurrentProfileJson();
      } else {
        Serial.print("{\"status\":\"error\",\"message\":\"unknown plant\",\"input\":\"");
        Serial.print(plantName);
        Serial.println("\"}");
      }
    }
    else if (cmd == "GET_PROFILE") {
      sendCurrentProfileJson();
    }
    else if (cmd == "LIGHT_MODE:AUTO") {
      setLightModeAuto();
    }
    else if (cmd == "LIGHT_MODE:MANUAL_ON") {
      setLightModeManualOn();
    }
    else if (cmd == "TIMER_LIGHT:ON") {
      setTimerLightState(true);
    }
    else if (cmd == "TIMER_LIGHT:OFF") {
      setTimerLightState(false);
    }
    else if (cmd == "DOSE_PH_UP") {
      dosePHUpManual();
    }
    else if (cmd == "DOSE_PH_DOWN") {
      dosePHDownManual();
    }
    else if (cmd == "DOSE_A") {
      doseBaseAManual();
    }
    else if (cmd == "DOSE_B") {
      doseBaseBManual();
    }
    else if (cmd == "RUN_PH_UP_5S") {
      runPHUpManual5s();
    }
    else if (cmd == "RUN_PH_DOWN_5S") {
      runPHDownManual5s();
    }
    else if (cmd == "RUN_BASE_A_5S") {
      runBaseAManual5s();
    }
    else if (cmd == "RUN_BASE_B_5S") {
      runBaseBManual5s();
    }
  }
}

// -----------------------------
// JSON output
// -----------------------------
void sendLightStatusJson() {
  Serial.print("{\"lightOn\":");
  Serial.print(lightOn ? "true" : "false");

  Serial.print(",\"lightMode\":\"");
  Serial.print(getLightModeString());
  Serial.print("\"");

  Serial.print(",\"lightStatus\":\"");
  Serial.print(getLightStatusString());
  Serial.print("\"");

  Serial.print(",\"timerWantsLightOn\":");
  Serial.print(timerWantsLightOn ? "true" : "false");

  Serial.print(",\"lastAction\":\"");
  Serial.print(lastAction);
  Serial.print("\"");

  Serial.println("}");
}

void sendSensorAndStatusJson(float tempC, float ph, float ec, float airTempC, float humidity) {
  Serial.print("{\"tempC\":");
  Serial.print(tempC, 2);

  Serial.print(",\"ph\":");
  if (isnan(ph)) Serial.print("null");
  else Serial.print(ph, 3);

  Serial.print(",\"ec\":");
  if (isnan(ec)) Serial.print("null");
  else Serial.print(ec, 3);

  Serial.print(",\"solution\":\"");
  Serial.print(intervalDoseSolution);
  Serial.print("\"");

  Serial.print(",\"mode\":\"");
  Serial.print(intervalDoseMode);
  Serial.print("\"");

  Serial.print(",\"plant\":\"");
  Serial.print(currentProfile.name);
  Serial.print("\"");

  Serial.print(",\"phMin\":");
  Serial.print(currentProfile.phMin, 2);

  Serial.print(",\"phMax\":");
  Serial.print(currentProfile.phMax, 2);

  Serial.print(",\"ecMin\":");
  Serial.print(currentProfile.ecMin, 2);

  Serial.print(",\"ecMax\":");
  Serial.print(currentProfile.ecMax, 2);

  Serial.print(",\"tempMin\":");
  Serial.print(currentProfile.tempMin, 2);

  Serial.print(",\"tempMax\":");
  Serial.print(currentProfile.tempMax, 2);

  Serial.print(",\"lightHours\":");
  Serial.print(currentProfile.lightHours);

  Serial.print(",\"lightOn\":");
  Serial.print(lightOn ? "true" : "false");

  Serial.print(",\"lightMode\":\"");
  Serial.print(getLightModeString());
  Serial.print("\"");

  Serial.print(",\"lightStatus\":\"");
  Serial.print(getLightStatusString());
  Serial.print("\"");

  Serial.print(",\"timerWantsLightOn\":");
  Serial.print(timerWantsLightOn ? "true" : "false");

  Serial.print(",\"airTempC\":");
  if (isnan(airTempC)) Serial.print("null");
  else Serial.print(airTempC, 2);

  Serial.print(",\"humidity\":");
  if (isnan(humidity)) Serial.print("null");
  else Serial.print(humidity, 2);

  Serial.print(",\"lastAction\":\"");
  Serial.print(lastAction);
  Serial.print("\"");

  Serial.println("}");
}

void setup() {
  Serial.begin(57600);

  Serial1.begin(9600);
  Serial2.begin(9600);

  pinMode(LIGHT_PIN, OUTPUT);
  relayOff(LIGHT_PIN);

  Wire.begin();
  if (!htu.begin()) {
    Serial.println("{\"error\":\"HTU21D_not_found\"}");
  }

  pumpPHUp.attach(PUMP_PH_UP_PIN);
  pumpPHDown.attach(PUMP_PH_DOWN_PIN);
  pumpBaseA.attach(PUMP_BASE_A_PIN);
  pumpBaseB.attach(PUMP_BASE_B_PIN);

  stopPump(pumpPHUp);
  stopPump(pumpPHDown);
  stopPump(pumpBaseA);
  stopPump(pumpBaseB);

  lightMode = LIGHT_MODE_AUTO;
  timerWantsLightOn = false;
  applyLightOutput();

  lastControlTime = 0;
  lastLogTime = 0;
  lastSensorTime = 0;cm

  clearDoseEvent();

  delay(1000);

  sendEzoCmd(Serial1, "C,0"); delay(50); readEzoLine(Serial1);
  sendEzoCmd(Serial1, "E,0"); delay(50); readEzoLine(Serial1);

  sendEzoCmd(Serial2, "C,0"); delay(50); readEzoLine(Serial2);
  sendEzoCmd(Serial2, "E,0"); delay(50); readEzoLine(Serial2);

  Serial.println("{\"status\":\"boot\"}");
  sendCurrentProfileJson();
  sendLightStatusJson();
}

void loop() {
  checkForNodeRedCommands();

  unsigned long now = millis();

  if (now - lastSensorTime >= SENSOR_INTERVAL_MS || lastSensorTime == 0) {
    lastSensorTime = now;

    lastTempC = readTemperatureC();

    String phStr = readEzoReading(Serial1);
    String ecStr = readEzoReading(Serial2);

    airTempC = htu.readTemperature();
    humidity = htu.readHumidity();

    lastPH = toFloatOrNaN(phStr);
    lastEC = toFloatOrNaN(ecStr) / 1000.0;
  }

  if (now - lastControlTime >= CONTROL_INTERVAL_MS || lastControlTime == 0) {
    lastControlTime = now;
    updateDosingControl(lastPH, lastEC);
  }

  if (now - lastLogTime >= LOG_INTERVAL_MS || lastLogTime == 0) {
    lastLogTime = now;
    sendSensorAndStatusJson(lastTempC, lastPH, lastEC, airTempC, humidity);
    clearDoseEvent();
  }

  delay(10);
}