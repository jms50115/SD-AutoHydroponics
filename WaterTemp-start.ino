/*
  Temperature Control State Machine (non-blocking)
  - Reads RTD analog input periodically
  - Filters temperature
  - Decides whether to COOL / HEAT / OK based on percent error band
  - "Actuates" fans via stub functions (non-functioning placeholders)
  - Uses a settle/mix time after any fan action before re-evaluating

  NOTE: Fan functions are stubs. Replace bodies with real pin/PWM control later.
*/

#include <Arduino.h>
#include <math.h>

// -------------------- Hardware / sensor settings --------------------
const int   tempPin = A0;
const float Vref    = 5.0;

// Voltage->Temp model (your existing linear conversion)
const float V_OFFSET = 1.058;
const float V_SLOPE  = 0.009;   // V per °C

// -------------------- Control settings --------------------
const float setpointF        = 78.0;     // desired temperature °F
const float tolerancePercent = 5.0;      // allowable +/- percent of setpoint

// Timing (non-blocking)
const unsigned long sampleIntervalMs = 1000;   // sensor sample rate
const unsigned long controlPeriodMs  = 5000;  // control decision rate
const unsigned long settleTimeMs     = 5000;  // wait after "fan action" before re-evaluating

// Fan staging thresholds (percent error above setpoint)
const float stage1Pct = 2.0;   // just above tolerance
const float stage2Pct = 5.0;
const float stage3Pct = 8.0;

// Filter strength (EMA). 0.0 = no update, 1.0 = no filtering
const float emaAlpha = 0.25;

// -------------------- State machine --------------------
enum ControlState {
  ST_INIT = 0,
  ST_IDLE,
  ST_SAMPLE,
  ST_EVALUATE,
  ST_ACTUATE,
  ST_SETTLE,
  ST_FAULT
};

enum Action {
  ACT_OK = 0,
  ACT_COOL,
  ACT_HEAT
};

ControlState state = ST_INIT;
// -------------------- Timing variables --------------------
unsigned long lastSampleMs  = 0;
unsigned long lastControlMs = 0;
unsigned long stateEnterMs  = 0;

// -------------------- Measurements / derived values --------------------
float tempF_raw      = NAN;
float tempF_filtered = NAN;
float percentError   = NAN;

// Decision outputs
Action action = ACT_OK;
int fanStage = 0; // 0=off, 1..3 stages

// -------------------- Helper: enter a state --------------------
void enterState(ControlState s) {
  state = s;
  stateEnterMs = millis();
}

// -------------------- Sensor read / conversion --------------------
float readTemperatureF() {
  int rawADC = analogRead(tempPin);
  float voltage = rawADC * (Vref / 1023.0);
  float tempC = (voltage - V_OFFSET) / V_SLOPE;
  float tempF = tempC * 9.0 / 5.0 + 32.0;
  return tempF;
}

// -------------------- Filtering --------------------
void updateFilter(float newTempF) {
  tempF_raw = newTempF;

  if (isnan(tempF_filtered)) {
    tempF_filtered = newTempF; // initialize
  } else {
    tempF_filtered = emaAlpha * newTempF + (1.0 - emaAlpha) * tempF_filtered;
  }
}

// -------------------- Fault detection (simple bounds check) --------------------
bool sensorFault(float tf) {
  // Adjust to your expected operating range
  if (isnan(tf)) return true;
  if (tf < -40.0 || tf > 200.0) return true; // unrealistic -> sensor/wiring issue
  return false;
}

// -------------------- Decision logic --------------------
void computeDecision() {
  percentError = ((tempF_filtered - setpointF) / setpointF) * 100.0;
  float absErr = fabs(percentError);

  fanStage = 0;

  if (absErr <= tolerancePercent) {
    action = ACT_OK;
    fanStage = 0;
    return;
  }

  if (percentError > 0) {
    // Too hot -> COOL
    action = ACT_COOL;

    // Stage the fan based on how far above setpoint we are (percent)
    if (percentError >= stage3Pct) fanStage = 3;
    else if (percentError >= stage2Pct) fanStage = 2;
    else fanStage = 1;

  } else {
    // Too cold -> HEAT (placeholder for future)
    action = ACT_HEAT;
    fanStage = 0;
  }
}

// -------------------- UART output --------------------
void printStatus(const char* tag) {
  Serial.print(tag);
  Serial.print(" | Traw=");
  Serial.print(tempF_raw, 2);
  Serial.print("F Tflt=");
  Serial.print(tempF_filtered, 2);
  Serial.print("F SP=");
  Serial.print(setpointF, 2);
  Serial.print("F Err=");
  Serial.print(percentError, 2);
  Serial.print("% ");

  if (action == ACT_OK) {
    Serial.println("- OK");
  } else if (action == ACT_COOL) {
    Serial.print("v COOL stage=");
    Serial.println(fanStage);
  } else {
    Serial.println("^ HEAT (not implemented)");
  }
}

// -------------------- Fan control stubs (non-functioning) --------------------
void fansOff() {
  // TODO: implement later (e.g., analogWrite(FAN_PWM_PIN, 0))
}

void fansSetStage(int stage) {
  // TODO: implement later
  // Example mapping when you implement:
  // stage 1 -> 35% PWM
  // stage 2 -> 65% PWM
  // stage 3 -> 100% PWM
  (void)stage;
}

// -------------------- Setup --------------------
void setup() {
  Serial.begin(9600);
  Serial.println("Temp Control FSM (fans stubbed)");
  enterState(ST_INIT);
}

// -------------------- Main loop (non-blocking) --------------------
void loop() {
  unsigned long now = millis();

  switch (state) {
    case ST_INIT: {
      // Initialize measurements
      float tf = readTemperatureF();
      updateFilter(tf);

      if (sensorFault(tempF_raw)) {
        enterState(ST_FAULT);
      } else {
        lastSampleMs  = now;
        lastControlMs = now;
        enterState(ST_IDLE);
        Serial.println("INIT done -> IDLE");
      }
      break;
    }

    case ST_IDLE: {
      // Periodic sampling (fast loop)
      if (now - lastSampleMs >= sampleIntervalMs) {
        lastSampleMs = now;
        enterState(ST_SAMPLE);
      }
      // Periodic control decision (slow loop)
      else if (now - lastControlMs >= controlPeriodMs) {
        lastControlMs = now;
        enterState(ST_EVALUATE);
      }
      break;
    }

    case ST_SAMPLE: {
      float tf = readTemperatureF();
      updateFilter(tf);

      if (sensorFault(tempF_raw)) {
        enterState(ST_FAULT);
      } else {
        // Return immediately to IDLE (non-blocking)
        enterState(ST_IDLE);
      }
      break;
    }

    case ST_EVALUATE: {
      // Decide based on filtered temperature
      computeDecision();
      printStatus("EVAL");
      enterState(ST_ACTUATE);
      break;
    }

    case ST_ACTUATE: {
      // Take action, but only via stub functions for now
      if (action == ACT_OK) {
        fansOff();
      } else if (action == ACT_COOL) {
        fansSetStage(fanStage);
      } else { // ACT_HEAT
        // Future: heater control
        fansOff();
      }

      // After action, wait settle time before allowing next evaluation
      enterState(ST_SETTLE);
      break;
    }

    case ST_SETTLE: {
      // Still sample while settling (so logs update and fault detection works)
      if (now - lastSampleMs >= sampleIntervalMs) {
        lastSampleMs = now;
        float tf = readTemperatureF();
        updateFilter(tf);

        if (sensorFault(tempF_raw)) {
          enterState(ST_FAULT);
          break;
        }
      }

      // Stay here until settle time passes, then go back to IDLE
      if (now - stateEnterMs >= settleTimeMs) {
        Serial.println("SETTLE complete -> IDLE");
        enterState(ST_IDLE);
      }
      break;
    }

    case ST_FAULT: {
      // Safe response on fault
      fansOff();
      Serial.println("FAULT: temperature sensor out of range / disconnected");

      // Try to recover periodically without blocking
      // (Every 5 seconds attempt a read; if normal, re-init filter and return to IDLE)
      if (now - stateEnterMs >= 5000) {
        float tf = readTemperatureF();
        updateFilter(tf);
        if (!sensorFault(tempF_raw)) {
          Serial.println("FAULT cleared -> IDLE");
          lastSampleMs  = now;
          lastControlMs = now;
          enterState(ST_IDLE);
        } else {
          // stay in fault, reset timer for next retry
          enterState(ST_FAULT);
        }
      }
      break;
    }

    default:
      enterState(ST_FAULT);
      break;
  }
}
