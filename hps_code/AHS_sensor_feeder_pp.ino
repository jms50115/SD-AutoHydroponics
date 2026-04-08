#include <Arduino.h>
#include <math.h>
#include <Servo.h>

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
const unsigned long CONTROL_INTERVAL_MS = 5000UL;    // evaluate controls every 5 sec
const unsigned long MIX_DELAY_MS        = 120000UL;  // wait 2 min after any dose
const unsigned long PUMP_GAP_MS         = 2000UL;    // gap between Base A and Base B
const unsigned long MANUAL_PUMP_RUN_MS = 3000UL;  // 3 seconds

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
// Runtime state
// =============================
bool lightOn = false;
unsigned long lastLightToggle = 0;
unsigned long lastDoseTime = 0;
unsigned long lastControlTime = 0;

// cached latest readings
float lastTempC = NAN;
float lastPH = NAN;
float lastEC = NAN;

// last action text for dashboard/debug
String lastAction = "none";

// manual light override state
bool manualLightOverride = false;

// =============================
// Pump servo objects
// =============================
Servo pumpPHUp;
Servo pumpPHDown;
Servo pumpBaseA;
Servo pumpBaseB;

// -----------------------------
// Relay helpers (LIGHT ONLY)
// -----------------------------
// If your light relay board is ACTIVE LOW:
//   relayOn  -> digitalWrite(pin, LOW)
//   relayOff -> digitalWrite(pin, HIGH)
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
// Plant profile handling
// -----------------------------
bool setPlantProfileByName(String plantName) {
  plantName.trim();

  for (int i = 0; i < NUM_PLANTS; i++) {
    if (plantName.equalsIgnoreCase(plantProfiles[i].name)) {
      currentProfile = plantProfiles[i];

      // restart light schedule when plant changes
      manualLightOverride = false;
      lightOn = false;
      relayOff(LIGHT_PIN);
      lastLightToggle = millis();

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
  Serial.println("}");
}

// -----------------------------
// Pump control
// -----------------------------
void dosePHUp() {
  dosePump(pumpPHUp, PUMP_PH_UP_RUN_CMD, PH_DOSE_TIME_MS);
  lastDoseTime = millis();
  lastAction = "dose_ph_up";
}

void dosePHDown() {
  dosePump(pumpPHDown, PUMP_PH_DOWN_RUN_CMD, PH_DOSE_TIME_MS);
  lastDoseTime = millis();
  lastAction = "dose_ph_down";
}

void doseBaseA() {
  dosePump(pumpBaseA, PUMP_BASE_A_RUN_CMD, EC_DOSE_TIME_MS);
  lastDoseTime = millis();
  lastAction = "manual_dose_a";
}

void doseBaseB() {
  dosePump(pumpBaseB, PUMP_BASE_B_RUN_CMD, EC_DOSE_TIME_MS);
  lastDoseTime = millis();
  lastAction = "manual_dose_b";
}

void doseNutrients() {
  dosePump(pumpBaseA, PUMP_BASE_A_RUN_CMD, EC_DOSE_TIME_MS);
  delay(PUMP_GAP_MS);
  dosePump(pumpBaseB, PUMP_BASE_B_RUN_CMD, EC_DOSE_TIME_MS);

  lastDoseTime = millis();
  lastAction = "dose_base_a_b";
}

// -----------------------------
// Light control
// -----------------------------
void setLightState(bool on, const String &actionName) {
  if (on) {
    relayOn(LIGHT_PIN);
  } else {
    relayOff(LIGHT_PIN);
  }

  lightOn = on;
  lastLightToggle = millis();
  lastAction = actionName;
}

void updateLightControl() {
  if (manualLightOverride) {
    return;
  }

  unsigned long now = millis();

  unsigned long onTimeMs  = (unsigned long)currentProfile.lightHours * 60UL * 60UL * 1000UL;
  unsigned long offTimeMs = (unsigned long)(24 - currentProfile.lightHours) * 60UL * 60UL * 1000UL;

  if (lightOn) {
    if (now - lastLightToggle >= onTimeMs) {
      setLightState(false, "light_off");
    }
  } else {
    if (now - lastLightToggle >= offTimeMs) {
      setLightState(true, "light_on");
    }
  }
}

// -----------------------------
// Dosing control
// -----------------------------
void updateDosingControl(float ph, float ec) {
  unsigned long now = millis();

  // wait after any dose for mixing
  if (now - lastDoseTime < MIX_DELAY_MS) {
    return;
  }

  // ---------- pH control ----------
  if (!isnan(ph)) {
    if (ph < ABSOLUTE_MIN_PH || ph > ABSOLUTE_MAX_PH) {
      lastAction = "ph_out_of_absolute_range";
      return;
    }

    if (ph < currentProfile.phMin) {
      dosePHUp();
      return;
    }

    if (ph > currentProfile.phMax) {
      dosePHDown();
      return;
    }
  }

  // ---------- EC control ----------
  if (!isnan(ec)) {
    if (ec > ABSOLUTE_MAX_EC) {
      lastAction = "ec_above_absolute_max";
      return;
    }

    if (ec < currentProfile.ecMin) {
      doseNutrients();
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
// Supported:
//   PLANT:Lettuce
//   GET_PROFILE
//   LIGHT_ON
//   LIGHT_OFF
//   LIGHT:ON
//   LIGHT:OFF
//   AUTO_LIGHT
//   DOSE_PH_UP
//   DOSE_PH_DOWN
//   DOSE_A
//   DOSE_B
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
    else if (cmd == "LIGHT_ON" || cmd == "LIGHT:ON") {
      manualLightOverride = true;
      setLightState(true, "manual_light_on");
    }
    else if (cmd == "LIGHT_OFF" || cmd == "LIGHT:OFF") {
      manualLightOverride = true;
      setLightState(false, "manual_light_off");
    }
    else if (cmd == "AUTO_LIGHT") {
      manualLightOverride = false;
      lastLightToggle = millis();
      lastAction = "auto_light_enabled";
    }

    // Existing short manual dose commands
    else if (cmd == "DOSE_PH_UP") {
      dosePHUp();
    }
    else if (cmd == "DOSE_PH_DOWN") {
      dosePHDown();
    }
    else if (cmd == "DOSE_A") {
      doseBaseA();
    }
    else if (cmd == "DOSE_B") {
      doseBaseB();
    }

    // New 5-second manual pump run commands
    else if (cmd == "RUN_PH_UP_5S") {
      dosePump(pumpPHUp, PUMP_PH_UP_RUN_CMD, MANUAL_PUMP_RUN_MS);
      lastDoseTime = millis();
      lastAction = "manual_run_ph_up_5s";
    }
    else if (cmd == "RUN_PH_DOWN_5S") {
      dosePump(pumpPHDown, PUMP_PH_DOWN_RUN_CMD, MANUAL_PUMP_RUN_MS);
      lastDoseTime = millis();
      lastAction = "manual_run_ph_down_5s";
    }
    else if (cmd == "RUN_BASE_A_5S") {
      dosePump(pumpBaseA, PUMP_BASE_A_RUN_CMD, MANUAL_PUMP_RUN_MS);
      lastDoseTime = millis();
      lastAction = "manual_run_base_a_5s";
    }
    else if (cmd == "RUN_BASE_B_5S") {
      dosePump(pumpBaseB, PUMP_BASE_B_RUN_CMD, MANUAL_PUMP_RUN_MS);
      lastDoseTime = millis();
      lastAction = "manual_run_base_b_5s";
    }
  }
}

// -----------------------------
// JSON output
// -----------------------------
void sendSensorAndStatusJson(float tempC, float ph, float ec) {
  Serial.print("{\"tempC\":");
  Serial.print(tempC, 2);

  Serial.print(",\"ph\":");
  if (isnan(ph)) Serial.print("null");
  else Serial.print(ph, 3);

  Serial.print(",\"ec\":");
  if (isnan(ec)) Serial.print("null");
  else Serial.print(ec, 3);

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

  Serial.print(",\"manualLightOverride\":");
  Serial.print(manualLightOverride ? "true" : "false");

  Serial.print(",\"lastAction\":\"");
  Serial.print(lastAction);
  Serial.print("\"");

  Serial.println("}");
}

void setup() {
  // Match Node-RED flow serial config
  Serial.begin(57600);

  Serial1.begin(9600);
  Serial2.begin(9600);

  pinMode(LIGHT_PIN, OUTPUT);
  relayOff(LIGHT_PIN);

  // Attach pump servos
  pumpPHUp.attach(PUMP_PH_UP_PIN);
  pumpPHDown.attach(PUMP_PH_DOWN_PIN);
  pumpBaseA.attach(PUMP_BASE_A_PIN);
  pumpBaseB.attach(PUMP_BASE_B_PIN);

  // Stop all pumps at startup
  stopPump(pumpPHUp);
  stopPump(pumpPHDown);
  stopPump(pumpBaseA);
  stopPump(pumpBaseB);

  lightOn = false;
  manualLightOverride = false;
  lastLightToggle = millis();
  lastDoseTime = 0;
  lastControlTime = 0;

  delay(1000);

  sendEzoCmd(Serial1, "C,0"); delay(50); readEzoLine(Serial1);
  sendEzoCmd(Serial1, "E,0"); delay(50); readEzoLine(Serial1);

  sendEzoCmd(Serial2, "C,0"); delay(50); readEzoLine(Serial2);
  sendEzoCmd(Serial2, "E,0"); delay(50); readEzoLine(Serial2);

  Serial.println("{\"status\":\"boot\"}");
  sendCurrentProfileJson();
}

void loop() {
  checkForNodeRedCommands(); 

  float tempC = readTemperatureC();

  String phStr = readEzoReading(Serial1);
  String ecStr = readEzoReading(Serial2);

  float ph = toFloatOrNaN(phStr);
  float ec = toFloatOrNaN(ecStr) / 1000.0;  // convert μS/cm → mS/cm


  lastTempC = tempC;
  lastPH = ph;
  lastEC = ec;

  updateLightControl();

  unsigned long now = millis();
  if (now - lastControlTime >= CONTROL_INTERVAL_MS) {
    lastControlTime = now;
    updateDosingControl(ph, ec);
  }

  sendSensorAndStatusJson(tempC, ph, ec);

  delay(2000);
}