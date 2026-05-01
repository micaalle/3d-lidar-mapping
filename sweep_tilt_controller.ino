#include <Wire.h>
#include <Preferences.h>

const int STEP_PIN = 2;
const int DIR_PIN  = 3;
const int EN_PIN   = 4;
const int HOME_PIN = 5;

const uint8_t AS5600_ADDR = 0x36;
Preferences prefs;

// calibration
const float ENCODER_SIGN = -1.0f;
const bool DIR_HIGH_INCREASES_ANGLE = true;
const bool HOME_DIR_HIGH = false;

// slip safetly 
const float SOFT_LIMIT_NEG_DEG = -360.0f;
const float SOFT_LIMIT_POS_DEG = 360.0f;

const float ANGLE_TOLERANCE_DEG = 0.35f;
const int STABLE_COUNT_REQUIRED = 5;
const unsigned long GOTO_TIMEOUT_MS = 30000;

const int HOME_BACKOFF_STEPS = 20;
const int HOME_MAX_FAST_STEPS = 1500;
const int HOME_MAX_SLOW_STEPS = 200;

const unsigned long TELEMETRY_INTERVAL_US = 20000; // 50 Hz

float zeroOffsetDeg = 0.0f;
bool hasZero = false;

bool sweepActive = false;
float sweepStartDeg = 0.0f;
float sweepEndDeg = 0.0f;
float sweepSpeedDps = 0.0f;
unsigned long sweepStartUs = 0;
unsigned long lastTelemetryUs = 0;
bool sweepBeginSent = false;

float wrap180(float deg) {
  while (deg > 180.0f) deg -= 360.0f;
  while (deg < -180.0f) deg += 360.0f;
  return deg;
}

bool homePressed() {
  return digitalRead(HOME_PIN) == LOW;
}

void enableDriver() { digitalWrite(EN_PIN, LOW); }
void disableDriver() { digitalWrite(EN_PIN, HIGH); }

uint16_t readAS5600Raw() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0C);
  if (Wire.endTransmission(false) != 0) return 0;

  Wire.requestFrom(AS5600_ADDR, (uint8_t)2);
  if (Wire.available() < 2) return 0;

  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  return ((uint16_t)hi << 8) | lo;
}

float rawToDeg(uint16_t raw) {
  return (raw * 360.0f) / 4096.0f;
}

float readRawAngleDeg() {
  return rawToDeg(readAS5600Raw());
}

float readCorrectedAngleDeg() {
  float rawDeg = readRawAngleDeg();
  float corrected = ENCODER_SIGN * (rawDeg - zeroOffsetDeg);
  return wrap180(corrected);
}

void singleStep(bool dirHigh, int delayUs) {
  digitalWrite(DIR_PIN, dirHigh ? HIGH : LOW);
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(delayUs);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(delayUs);
}

int chooseStepDelayUs(float absErrorDeg) {
  if (absErrorDeg > 20.0f) return 1600;
  if (absErrorDeg > 8.0f)  return 2400;
  if (absErrorDeg > 3.0f)  return 3400;
  return 5200;
}

void loadZeroState() {
  prefs.begin("scanner", true);
  zeroOffsetDeg = prefs.getFloat("zero", 0.0f);
  hasZero = prefs.getBool("hasZero", false);
  prefs.end();
}

void saveZeroState() {
  prefs.begin("scanner", false);
  prefs.putFloat("zero", zeroOffsetDeg);
  prefs.putBool("hasZero", hasZero);
  prefs.end();
}

void printStatus() {
  float rawDeg = readRawAngleDeg();
  float corrected = hasZero ? readCorrectedAngleDeg() : 0.0f;

  Serial.print("OK STATUS raw=");
  Serial.print(rawDeg, 3);
  Serial.print(" corrected=");
  Serial.print(corrected, 3);
  Serial.print(" zero=");
  Serial.print(zeroOffsetDeg, 3);
  Serial.print(" home=");
  Serial.print(homePressed() ? "PRESSED" : "OPEN");
  Serial.print(" sweep=");
  Serial.println(sweepActive ? "ON" : "OFF");
}

void jogSteps(int n, int delayUs = 3000) {
  enableDriver();
  bool dirHigh = (n > 0);
  int count = abs(n);

  for (int i = 0; i < count; i++) {
    singleStep(dirHigh, delayUs);
  }

  Serial.print("OK JOG ");
  Serial.println(n);
}

bool doHome() {
  enableDriver();

  if (homePressed()) {
    for (int i = 0; i < HOME_BACKOFF_STEPS; i++) {
      singleStep(!HOME_DIR_HIGH, 2500);
    }
    delay(80);
  }

  int fastSteps = 0;
  while (!homePressed() && fastSteps < HOME_MAX_FAST_STEPS) {
    singleStep(HOME_DIR_HIGH, 1800);
    fastSteps++;
  }

  if (!homePressed()) {
    Serial.println("ERR HOME_NOT_FOUND");
    return false;
  }

  for (int i = 0; i < HOME_BACKOFF_STEPS; i++) {
    singleStep(!HOME_DIR_HIGH, 2500);
  }
  delay(80);

  int slowSteps = 0;
  while (!homePressed() && slowSteps < HOME_MAX_SLOW_STEPS) {
    singleStep(HOME_DIR_HIGH, 3800);
    slowSteps++;
  }

  if (!homePressed()) {
    Serial.println("ERR HOME_REAPPROACH");
    return false;
  }

  zeroOffsetDeg = readRawAngleDeg();
  hasZero = true;
  saveZeroState();

  Serial.print("OK HOME ");
  Serial.println(zeroOffsetDeg, 3);
  return true;
}

bool moveToAngle(float targetDeg) {
  if (!hasZero) {
    Serial.println("ERR NO_ZERO");
    return false;
  }

  if (targetDeg < SOFT_LIMIT_NEG_DEG || targetDeg > SOFT_LIMIT_POS_DEG) {
    Serial.println("ERR SOFT_LIMIT");
    return false;
  }

  enableDriver();

  unsigned long t0 = millis();
  int stableCount = 0;
  int stepBudget = 5000;

  while (millis() - t0 < GOTO_TIMEOUT_MS && stepBudget > 0) {
    float currentDeg = readCorrectedAngleDeg();
    float errorDeg = wrap180(targetDeg - currentDeg);
    float absError = fabs(errorDeg);

    if (absError <= ANGLE_TOLERANCE_DEG) {
      stableCount++;
      if (stableCount >= STABLE_COUNT_REQUIRED) {
        Serial.print("OK GOTO ");
        Serial.println(targetDeg, 3);
        return true;
      }
      delay(20);
      continue;
    }

    stableCount = 0;
    bool wantIncrease = (errorDeg > 0.0f);
    bool dirHigh = DIR_HIGH_INCREASES_ANGLE ? wantIncrease : !wantIncrease;

    int delayUs = chooseStepDelayUs(absError);
    singleStep(dirHigh, delayUs);
    stepBudget--;

    float newDeg = readCorrectedAngleDeg();
    float newErr = wrap180(targetDeg - newDeg);
    float newAbsErr = fabs(newErr);

    if (newAbsErr <= ANGLE_TOLERANCE_DEG || newAbsErr <= 0.5f) {
      Serial.print("OK GOTO ");
      Serial.println(targetDeg, 3);
      return true;
    }

    if (newAbsErr > absError + 1.0f) {
      Serial.println("ERR DIVERGING");
      return false;
    }
  }

  Serial.println("ERR GOTO_TIMEOUT");
  return false;
}

void stopSweep(const char* reason = nullptr) {
  sweepActive = false;
  if (reason) {
    Serial.print("INFO,SWEEP_DONE,");
    Serial.print(micros());
    Serial.print(",");
    Serial.print(readCorrectedAngleDeg(), 4);
    Serial.print(",");
    Serial.println(reason);
  }
}

void beginSweep(float startDeg, float endDeg, float speedDps) {
  sweepStartDeg = startDeg;
  sweepEndDeg = endDeg;
  sweepSpeedDps = fabs(speedDps);
  sweepStartUs = micros();
  lastTelemetryUs = 0;
  sweepBeginSent = false;
  sweepActive = true;
  enableDriver();

  Serial.print("OK SWEEP ");
  Serial.print(startDeg, 3);
  Serial.print(" ");
  Serial.print(endDeg, 3);
  Serial.print(" ");
  Serial.println(speedDps, 3);
}

void serviceSweep() {
  if (!sweepActive) return;
  if (!hasZero) {
    stopSweep("NO_ZERO");
    return;
  }

  unsigned long nowUs = micros();

  if (!sweepBeginSent) {
    Serial.print("INFO,SWEEP_BEGIN,");
    Serial.print(nowUs);
    Serial.print(",");
    Serial.print(sweepStartDeg, 4);
    Serial.print(",");
    Serial.print(sweepEndDeg, 4);
    Serial.print(",");
    Serial.println(sweepSpeedDps, 4);
    sweepBeginSent = true;
  }

  float currentDeg = readCorrectedAngleDeg();

  if (nowUs - lastTelemetryUs >= TELEMETRY_INTERVAL_US) {
    lastTelemetryUs = nowUs;
    Serial.print("ANG,");
    Serial.print(nowUs);
    Serial.print(",");
    Serial.println(currentDeg, 4);
  }

  float signDir = (sweepEndDeg >= sweepStartDeg) ? 1.0f : -1.0f;
  float totalDist = fabs(sweepEndDeg - sweepStartDeg);
  float elapsedSec = (nowUs - sweepStartUs) / 1000000.0f;
  float desiredTravel = sweepSpeedDps * elapsedSec;
  if (desiredTravel > totalDist) desiredTravel = totalDist;

  float targetDeg = sweepStartDeg + signDir * desiredTravel;
  if (targetDeg < SOFT_LIMIT_NEG_DEG || targetDeg > SOFT_LIMIT_POS_DEG) {
    stopSweep("SOFT_LIMIT");
    return;
  }

  float errorDeg = wrap180(targetDeg - currentDeg);
  float absError = fabs(errorDeg);

  if (desiredTravel >= totalDist && absError <= ANGLE_TOLERANCE_DEG) {
    stopSweep("OK");
    return;
  }

  if (absError <= ANGLE_TOLERANCE_DEG) {
    return;
  }

  bool wantIncrease = (errorDeg > 0.0f);
  bool dirHigh = DIR_HIGH_INCREASES_ANGLE ? wantIncrease : !wantIncrease;

  int delayUs = chooseStepDelayUs(absError);
  singleStep(dirHigh, delayUs);
}

void handleCommand(String s) {
  s.trim();
  if (s.length() == 0) return;

  if (s.equalsIgnoreCase("PING")) {
    Serial.println("OK PONG");
    return;
  }

  if (s.equalsIgnoreCase("ENABLE")) {
    enableDriver();
    Serial.println("OK ENABLE");
    return;
  }

  if (s.equalsIgnoreCase("DISABLE")) {
    disableDriver();
    Serial.println("OK DISABLE");
    return;
  }

  if (s.equalsIgnoreCase("STATUS")) {
    printStatus();
    return;
  }

  if (s.equalsIgnoreCase("RAW?")) {
    Serial.print("OK RAW ");
    Serial.println(readRawAngleDeg(), 3);
    return;
  }

  if (s.equalsIgnoreCase("ANGLE?")) {
    if (!hasZero) {
      Serial.println("ERR NO_ZERO");
      return;
    }
    Serial.print("OK ANGLE ");
    Serial.println(readCorrectedAngleDeg(), 3);
    return;
  }

  if (s.equalsIgnoreCase("ZEROHERE")) {
    zeroOffsetDeg = readRawAngleDeg();
    hasZero = true;
    saveZeroState();
    Serial.print("OK ZEROHERE ");
    Serial.println(zeroOffsetDeg, 3);
    return;
  }

  if (s.equalsIgnoreCase("HOME")) {
    doHome();
    return;
  }

  if (s.equalsIgnoreCase("STOP")) {
    stopSweep("STOP");
    Serial.println("OK STOP");
    return;
  }

  if (s.startsWith("JOG ")) {
    int n = s.substring(4).toInt();
    if (abs(n) > 32) {
      Serial.println("ERR JOG_TOO_LARGE");
      return;
    }
    jogSteps(n);
    return;
  }

  if (s.startsWith("GOTO ")) {
    float targetDeg = s.substring(5).toFloat();
    moveToAngle(targetDeg);
    return;
  }

  if (s.startsWith("SWEEP ")) {
    if (!hasZero) {
      Serial.println("ERR NO_ZERO");
      return;
    }

    int firstSpace = s.indexOf(' ');
    int secondSpace = s.indexOf(' ', firstSpace + 1);
    int thirdSpace = s.indexOf(' ', secondSpace + 1);

    if (firstSpace < 0 || secondSpace < 0 || thirdSpace < 0) {
      Serial.println("ERR BAD_SWEEP_ARGS");
      return;
    }

    float startDeg = s.substring(firstSpace + 1, secondSpace).toFloat();
    float endDeg = s.substring(secondSpace + 1, thirdSpace).toFloat();
    float speedDps = s.substring(thirdSpace + 1).toFloat();

    if (startDeg < SOFT_LIMIT_NEG_DEG || startDeg > SOFT_LIMIT_POS_DEG ||
        endDeg < SOFT_LIMIT_NEG_DEG || endDeg > SOFT_LIMIT_POS_DEG) {
      Serial.println("ERR SOFT_LIMIT");
      return;
    }

    if (speedDps <= 0.0f || speedDps > 20.0f) {
      Serial.println("ERR BAD_SWEEP_SPEED");
      return;
    }

    beginSweep(startDeg, endDeg, speedDps);
    return;
  }

  Serial.println("ERR UNKNOWN");
}

void setup() {
  Serial.begin(115200);

  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 5000) {
    delay(10);
  }

  Wire.begin();
  loadZeroState();

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(HOME_PIN, INPUT_PULLUP);

  digitalWrite(STEP_PIN, LOW);
  enableDriver();

  delay(300);
  Serial.println("READY CONT_SWEEP_V1");
}

void loop() {
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    handleCommand(s);
  }

  if (sweepActive) {
    serviceSweep();
  }
}
