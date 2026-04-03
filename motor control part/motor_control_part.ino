#include <avr/pgmspace.h>
#include "lookup_table.h"

// ==========================================================
//  Auto-Mandhala — CNC Shield V3 + A4988 + NEMA 17
// ==========================================================
//
//  Wiring (CNC Shield V3, fixed pinout):
//    X-axis: STEP = D2,  DIR = D5
//    Y-axis: STEP = D3,  DIR = D6
//    EN (global enable) = D8  (LOW = active)
//
//  Motor: 17HS4401j (NEMA 17, 1.8°/step, 200 steps/rev)
//  A4988 microstepping set by jumpers under shield:
//    No jumpers = full step (200 steps/rev)
//    MS1        = half step (400)
//    MS1+MS2    = 1/4 step (800)
//    All        = 1/16 step (3200) ← default, smoothest
//
//  Serial commands (115200 baud):
//    Angle mode:  90,45        → motor1 to 90°, motor2 to 45°
//    Field mode:  field:30,45  → lookup: 30uT magnitude, 45° direction
// ==========================================================

// ---------- CNC Shield V3 Pin Definitions ----------

const int PIN_EN     = 8;   // Global enable (LOW = active)

const int PIN_X_STEP = 2;   // X-axis step
const int PIN_X_DIR  = 5;   // X-axis direction

const int PIN_Y_STEP = 3;   // Y-axis step
const int PIN_Y_DIR  = 6;   // Y-axis direction

// ---------- Motor Parameters ----------

// ★ A4988 microstepping (set by CNC Shield jumper caps):
//   No jumpers = full step (200)
//   MS1 = half step (400)
//   MS1+MS2 = 1/4 step (800)
//   All = 1/16 step (3200)
//   Current config: no jumpers = full step
const long STEPS_PER_REV = 200 * 3;

// Pulse interval (microseconds). Full step recommended: 1000~2000us
const int STEP_DELAY_US = 1500;

// ---------- Motor Pin Arrays (for indexing) ----------

const int stepPin[2] = {PIN_X_STEP, PIN_Y_STEP};
const int dirPin[2]  = {PIN_X_DIR,  PIN_Y_DIR};

// ---------- Motor State (current position in steps) ----------

long currentStep[2] = {0, 0};

// ---------- Target Flags ----------

// Angle mode: "90,45"
bool  hasAngleTarget = false;
float targetDeg[2]   = {0.0, 0.0};

// Field mode: "field:30,45"
bool  hasFieldTarget  = false;
float targetMagnitude = 0.0;
float targetDirection = 0.0;

// ---------- Low-level: Single Step Pulse ----------

void stepOnce(int motorId, int dir) {
  // dir: +1 = clockwise, -1 = counter-clockwise
  digitalWrite(dirPin[motorId], (dir > 0) ? HIGH : LOW);
  digitalWrite(stepPin[motorId], HIGH);
  delayMicroseconds(STEP_DELAY_US);
  digitalWrite(stepPin[motorId], LOW);
  delayMicroseconds(STEP_DELAY_US);

  currentStep[motorId] += dir;

  // Keep within [0, STEPS_PER_REV) range
  if (currentStep[motorId] >= STEPS_PER_REV) currentStep[motorId] -= STEPS_PER_REV;
  if (currentStep[motorId] < 0)              currentStep[motorId] += STEPS_PER_REV;
}

// ---------- Move to Target Step (shortest path) ----------

void moveToStep(int motorId, long targetStep) {
  // Normalize target
  while (targetStep < 0)              targetStep += STEPS_PER_REV;
  while (targetStep >= STEPS_PER_REV) targetStep -= STEPS_PER_REV;

  long diff = targetStep - currentStep[motorId];

  // Choose shortest path
  if (diff >  STEPS_PER_REV / 2) diff -= STEPS_PER_REV;
  if (diff < -STEPS_PER_REV / 2) diff += STEPS_PER_REV;

  int  direction = (diff >= 0) ? +1 : -1;
  long steps     = labs(diff);

  Serial.print(F("[Motor "));
  Serial.print(motorId == 0 ? "X" : "Y");
  Serial.print(F("] Moving "));
  Serial.print(steps);
  Serial.print(F(" steps, dir="));
  Serial.println(direction > 0 ? "CW" : "CCW");

  // Enable driver (power saving: only power on during movement)
  digitalWrite(PIN_EN, LOW);
  delay(5);  // Wait for driver to stabilize

  for (long i = 0; i < steps; i++) {
    stepOnce(motorId, direction);
    delay(100);
  }

  delay(10); // Wait for motor to settle
  // Disable driver, release holding current (motor no longer locked)
  digitalWrite(PIN_EN, HIGH);

  Serial.print(F("[Motor "));
  Serial.print(motorId == 0 ? "X" : "Y");
  Serial.print(F("] Done. Position=step "));
  Serial.println(currentStep[motorId]);
}

// ---------- Move to Angle ----------

void moveToAngle(int motorId, float angleDeg) {
  long target = lround((angleDeg / 360.0) * (double)STEPS_PER_REV);
  moveToStep(motorId, target);
}

// ---------- Lookup Table ----------

void findBestAngles(float targetMag, float targetDir,
                    int &outerAngle, int &innerAngle) {
  float bestError = 9999999.0;

  for (int i = 0; i < TABLE_SIZE; i++) {
    TableEntry e = getEntry(i);

    float magError = abs(e.magnitude_uT - targetMag);
    float dirDiff  = abs(e.direction_deg - targetDir);
    if (dirDiff > 180.0) dirDiff = 360.0 - dirDiff;
    float totalError = magError + dirDiff;

    if (totalError < bestError) {
      bestError  = totalError;
      outerAngle = e.outerDeg;
      innerAngle = e.innerDeg;
    }
  }

  Serial.print(F("Best match: outer="));
  Serial.print(outerAngle);
  Serial.print(F("  inner="));
  Serial.print(innerAngle);
  Serial.print(F("  error="));
  Serial.println(bestError);
}

// ---------- Serial Input Parser ----------
//
//  Angle mode:  90,45          → X to 90°, Y to 45°
//  Field mode:  field:30,45    → lookup table for best angles
//

void handleSerialInput() {
  static String input = "";

  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (input.length() > 0) {

        // --- Field mode: starts with "field:" ---
        if (input.startsWith("field:")) {
          String params   = input.substring(6);
          int    commaIdx = params.indexOf(',');

          if (commaIdx > 0) {
            float mag = params.substring(0, commaIdx).toFloat();
            float dir = params.substring(commaIdx + 1).toFloat();

            dir = fmod(dir, 360.0);
            if (dir > 180.0)  dir -= 360.0;
            if (dir < -180.0) dir += 360.0;

            targetMagnitude = mag;
            targetDirection = dir;
            hasFieldTarget  = true;

            Serial.print(F("Field mode: mag="));
            Serial.print(targetMagnitude);
            Serial.print(F(" uT, dir="));
            Serial.print(targetDirection);
            Serial.println(F(" deg"));

          } else {
            Serial.println(F("Invalid. Format: field:magnitude,direction  e.g. field:30,45"));
          }

        // --- Angle mode: default, comma-separated angles ---
        } else {
          int commaIdx = input.indexOf(',');

          if (commaIdx > 0) {
            float deg1 = input.substring(0, commaIdx).toFloat();
            float deg2 = input.substring(commaIdx + 1).toFloat();

            // Normalize to 0~360
            deg1 = fmod(deg1, 360.0);
            deg2 = fmod(deg2, 360.0);
            if (deg1 < 0) deg1 += 360.0;
            if (deg2 < 0) deg2 += 360.0;

            targetDeg[0]   = deg1;
            targetDeg[1]   = deg2;
            hasAngleTarget = true;

            Serial.print(F("Angle mode: X="));
            Serial.print(targetDeg[0]);
            Serial.print(F(" deg, Y="));
            Serial.print(targetDeg[1]);
            Serial.println(F(" deg"));

          } else {
            Serial.println(F("Invalid input."));
            Serial.println(F("  Angle mode:  deg1,deg2        e.g. 90,45"));
            Serial.println(F("  Field mode:  field:mag,dir    e.g. field:30,45"));
          }
        }

        input = "";
      }

    } else {
      if (isAlphaNumeric(c) || c == '.' || c == '-' || c == '+' || c == ',' || c == ':') {
        input += c;
      }
    }
  }
}

// ---------- Execute Field Mode ----------

void goToField(float magnitude, float direction) {
  int outerAngle = 0;
  int innerAngle = 0;

  Serial.println(F("Searching lookup table..."));
  findBestAngles(magnitude, direction, outerAngle, innerAngle);

  Serial.print(F("Moving X (outer) to "));
  Serial.print(outerAngle);
  Serial.println(F(" deg"));
  moveToAngle(0, outerAngle);

  Serial.print(F("Moving Y (inner) to "));
  Serial.print(innerAngle);
  Serial.println(F(" deg"));
  moveToAngle(1, innerAngle);

  Serial.println(F("Done."));
}

// ---------- Execute Angle Mode ----------

void goToAngles(float deg1, float deg2) {
  Serial.print(F("Moving X to "));
  Serial.print(deg1);
  Serial.println(F(" deg"));
  moveToAngle(0, deg1);

  Serial.print(F("Moving Y to "));
  Serial.print(deg2);
  Serial.println(F(" deg"));
  moveToAngle(1, deg2);

  Serial.println(F("Done."));
}

// ---------- Setup ----------

void setup() {
  Serial.begin(115200);
  Serial.println(F("=== Auto-Mandhala (CNC Shield + A4988) ==="));

  // Configure pins
  pinMode(PIN_EN,     OUTPUT);
  pinMode(PIN_X_STEP, OUTPUT);
  pinMode(PIN_X_DIR,  OUTPUT);
  pinMode(PIN_Y_STEP, OUTPUT);
  pinMode(PIN_Y_DIR,  OUTPUT);

  // Default: disable driver (EN=HIGH), enable only during movement
  digitalWrite(PIN_EN, HIGH);

  Serial.print(F("Steps/rev = "));
  Serial.println(STEPS_PER_REV);
  Serial.println(F("Ready."));
  Serial.println(F("  Angle mode:  deg1,deg2        e.g. 90,45"));
  Serial.println(F("  Field mode:  field:mag,dir    e.g. field:30,45"));
}

// ---------- Loop ----------

void loop() {
  handleSerialInput();

  if (hasAngleTarget) {
    goToAngles(targetDeg[0], targetDeg[1]);
    hasAngleTarget = false;
  }

  if (hasFieldTarget) {
    goToField(targetMagnitude, targetDirection);
    hasFieldTarget = false;
  }

  delay(10);
}
