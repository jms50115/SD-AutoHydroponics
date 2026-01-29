#include <Wire.h>
#include "Adafruit_HTU21DF.h"

// ---------------------------
// USER CONFIG
// ---------------------------

// I2C addresses (confirm yours)
#define EZO_PH_ADDR 0x63
#define EZO_EC_ADDR 0x64

// Control pins (recommended: two independent control signals)
// If you only have ONE Adafruit 4-outlet module, you can only use ONE control pin.
#define PIN_RELAY_FAN   6
#define PIN_RELAY_LIGHT 7

// Read/Control timing
#define SENSOR_PERIOD_MS        5000UL   // read sensors every 5s
#define CONTROL_PERIOD_MS       5000UL   // evaluate control every 5s
#define DOSING_COOLDOWN_MS     60000UL   // wait 60s between dosing actions
#define MIXING_WAIT_MS         30000UL   // after dosing, wait 30s before trusting readings

// ---------------------------
// BASIL TARGETS (defaults)
// ---------------------------

// pH
#define PH_MIN 5.8
#define PH_MAX 6.2
#define PH_HYST 0.05   // hysteresis to prevent chatter

// EC (mS/cm)
#define EC_MIN 1.0
#define EC_MAX 1.4
#define EC_HYST 0.05

// Water temperature (°F)
#define WATER_F_TARGET 72.0
#define WATER_F_HYST    2.0   // +/- band

// Air temperature & humidity
#define AIR_F_MAX  80.0   // choose based on your enclosure; basil commonly ~65–70F ideal
#define AIR_F_MIN  75.0   // fan turns off below this (hysteresis band)
#define RH_MAX     65.0   // fan on if humidity too high
#define RH_MIN     60.0   // fan off below this

// Light schedule (default basil-friendly)
#define LIGHTS_ON_HOURS   14    // Johnny’s suggests minimum 14h/day for basil
#define LIGHTS_OFF_HOURS  10
// For testing you can change to e.g. 6/18

// ---------------------------
// GLOBALS
// ---------------------------
Adafruit_HTU21DF htu = Adafruit_HTU21DF();

float phVal = NAN;
float ecVal = NAN;          // mS/cm
float airTempC = NAN;
float airTempF = NAN;
float relHum = NAN;

// TODO: replace with your Atlas Surveyor water temp reading.
// For now we’ll stub it out; you’ll paste your actual conversion here.
float waterTempF = NAN;

bool fanOn = false;
bool lightOn = false;

unsigned long lastSensorMs = 0;
unsigned long lastControlMs = 0;
unsigned long lastDoseMs = 0;
unsigned long lastDoseActionMs = 0;

// For a simple light schedule without RTC, we track "time since boot".
// Node-RED can later send real time-of-day if you want accurate schedules.
unsigned long lightCycleStartMs = 0;

// ---------------------------
// ATLAS EZO I2C HELPERS
// ---------------------------

// Send an ASCII command to EZO device
void ezoSendCommand(uint8_t addr, const char* cmd) {
  Wire.beginTransmission(addr);
  while (*cmd) Wire.write((uint8_t)(*cmd++));
  Wire.endTransmission();
}

// Read response from EZO into buffer; returns true if got something
bool ezoReadResponse(uint8_t addr, char* out, size_t outLen) {
  // EZO typically returns: [status][ASCII...]
  // status: 1 = success, 2 = fail, 254 = pending, 255 = no data
  Wire.requestFrom((int)addr, (int)outLen);
  if (!Wire.available()) return false;

  uint8_t status = Wire.read();
  size_t i = 0;

  while (Wire.available() && i < outLen - 1) {
    char c = (char)Wire.read();
    if (c == 0) break;
    out[i++] = c;
  }
  out[i] = '\0';

  return (status == 1);
}

// Request a reading and parse float
bool ezoReadFloat(uint8_t addr, float &value) {
  // Request a new reading
  ezoSendCommand(addr, "R");
  // Typical read time: ~900ms for many EZO circuits (varies by circuit/config)
  delay(1000);

  char buf[32];
  if (!ezoReadResponse(addr, buf, sizeof(buf))) return false;

  value = atof(buf);
  return true;
}

// ---------------------------
// PLACEHOLDER: WATER TEMP
// ---------------------------
// TODO: Replace this with your actual Surveyor analog temperature probe logic.
// If you already have working code that returns waterTempF, paste it here.
void readWaterTempStub() {
  // Example stub:
  // waterTempF = 72.0;
}

// ---------------------------
// LIGHT SCHEDULE (no RTC)
// ---------------------------
void updateLightSchedule() {
  unsigned long now = millis();
  unsigned long onMs  = (unsigned long)LIGHTS_ON_HOURS  * 60UL * 60UL * 1000UL;
  unsigned long offMs = (unsigned long)LIGHTS_OFF_HOURS * 60UL * 60UL * 1000UL;
  unsigned long cycleMs = onMs + offMs;

  if (lightCycleStartMs == 0) lightCycleStartMs = now;

  unsigned long elapsed = (now - lightCycleStartMs) % cycleMs;
  bool shouldBeOn = (elapsed < onMs);

  if (shouldBeOn != lightOn) {
    lightOn = shouldBeOn;
    digitalWrite(PIN_RELAY_LIGHT, lightOn ? HIGH : LOW);
  }
}

// ---------------------------
// CONTROL LOGIC
// ---------------------------

bool dosingCooldownReady() {
  return (millis() - lastDoseActionMs) >= DOSING_COOLDOWN_MS;
}

bool mixingWaitOver() {
  return (millis() - lastDoseActionMs) >= MIXING_WAIT_MS;
}

void controlFan() {
  // Turn fan ON if too hot OR too humid
  bool shouldOn = fanOn;

  if (!isnan(airTempF) && airTempF >= AIR_F_MAX) shouldOn = true;
  if (!isnan(relHum)   && relHum   >= RH_MAX)   shouldOn = true;

  // Turn fan OFF only if back below both "min" thresholds
  if (!isnan(airTempF) && !isnan(relHum)) {
    if (airTempF <= AIR_F_MIN && relHum <= RH_MIN) shouldOn = false;
  } else if (!isnan(airTempF)) {
    if (airTempF <= AIR_F_MIN) shouldOn = false;
  } else if (!isnan(relHum)) {
    if (relHum <= RH_MIN) shouldOn = false;
  }

  if (shouldOn != fanOn) {
    fanOn = shouldOn;
    digitalWrite(PIN_RELAY_FAN, fanOn ? HIGH : LOW);
  }
}

void controlDosing() {
  // Don’t dose too often, and don’t trust readings immediately after a dose
  if (!dosingCooldownReady()) return;
  if (!mixingWaitOver()) return;

  // pH control
  if (!isnan(phVal)) {
    if (phVal > (PH_MAX + PH_HYST)) {
      // TODO: dispense pH DOWN (servo/valve/pump)
      // Example placeholder:
      // dispenseSolution(PH_DOWN, amount);
      lastDoseActionMs = millis();
      return;
    } else if (phVal < (PH_MIN - PH_HYST)) {
      // TODO: dispense pH UP
      lastDoseActionMs = millis();
      return;
    }
  }

  // EC control (nutrient strength)
  if (!isnan(ecVal)) {
    if (ecVal < (EC_MIN - EC_HYST)) {
      // TODO: dispense nutrient concentrate (A/B)
      lastDoseActionMs = millis();
      return;
    } else if (ecVal > (EC_MAX + EC_HYST)) {
      // TODO: add water / dilute (if you have that capability)
      lastDoseActionMs = millis();
      return;
    }
  }

  // Water temp control (if you later add heater/chiller)
  if (!isnan(waterTempF)) {
    // TODO: optional heater/chiller control
  }
}

// ---------------------------
// JSON OUTPUT FOR NODE-RED
// ---------------------------
void printJsonLine() {
  Serial.print("{\"ph\":");      if (isnan(phVal)) Serial.print("null"); else Serial.print(phVal, 2);
  Serial.print(",\"ec\":");      if (isnan(ecVal)) Serial.print("null"); else Serial.print(ecVal, 2);
  Serial.print(",\"waterF\":");  if (isnan(waterTempF)) Serial.print("null"); else Serial.print(waterTempF, 2);
  Serial.print(",\"airF\":");    if (isnan(airTempF)) Serial.print("null"); else Serial.print(airTempF, 2);
  Serial.print(",\"rh\":");      if (isnan(relHum)) Serial.print("null"); else Serial.print(relHum, 1);

  Serial.print(",\"fan\":");   Serial.print(fanOn ? "true" : "false");
  Serial.print(",\"light\":"); Serial.print(lightOn ? "true" : "false");

  // include targets so Node-RED can display them too
  Serial.print(",\"targets\":{");
  Serial.print("\"ph\":["); Serial.print(PH_MIN,2); Serial.print(","); Serial.print(PH_MAX,2); Serial.print("],");
  Serial.print("\"ec\":["); Serial.print(EC_MIN,2); Serial.print(","); Serial.print(EC_MAX,2); Serial.print("],");
  Serial.print("\"airFmax\":"); Serial.print(AIR_F_MAX,1);
  Serial.print(",\"rhmax\":");  Serial.print(RH_MAX,1);
  Serial.print("}}");
  Serial.println();
}

// ---------------------------
// SETUP / LOOP
// ---------------------------
void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(PIN_RELAY_FAN, OUTPUT);
  pinMode(PIN_RELAY_LIGHT, OUTPUT);

  // Start OFF (depends on your relay module logic; adjust if active-low)
  digitalWrite(PIN_RELAY_FAN, LOW);
  digitalWrite(PIN_RELAY_LIGHT, LOW);

  if (!htu.begin()) {
    Serial.println("{\"error\":\"HTU21D not found\"}");
  }

  lightCycleStartMs = millis();
}

void loop() {
  unsigned long now = millis();

  // Light schedule always updates
  updateLightSchedule();

  // Periodic sensor reads
  if (now - lastSensorMs >= SENSOR_PERIOD_MS) {
    lastSensorMs = now;

    // Air sensor
    relHum = htu.readHumidity();
    airTempC = htu.readTemperature();
    airTempF = airTempC * 9.0 / 5.0 + 32.0;

    // Water sensors (Atlas EZO via I2C)
    // If a read fails, keep previous value or set NAN; here we set NAN
    if (!ezoReadFloat(EZO_PH_ADDR, phVal)) phVal = NAN;
    if (!ezoReadFloat(EZO_EC_ADDR, ecVal)) ecVal = NAN;

    // Water temp (stub)
    readWaterTempStub();

    // Send to Node-RED
    printJsonLine();
  }

  // Periodic control evaluation
  if (now - lastControlMs >= CONTROL_PERIOD_MS) {
    lastControlMs = now;
    controlFan();
    controlDosing();
  }

  // TODO (later): read commands from Serial (Node-RED) to change setpoints/profile.
}