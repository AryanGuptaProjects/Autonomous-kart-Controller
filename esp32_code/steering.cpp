#include "steering.h"

// ============================================================
// Direction Conventions
// ============================================================

// Calibrated straight-ahead dead center count
static uint32_t deadCenterCount = DEFAULT_DEAD_CENTER;

// Direction convention: 1 = Normal, -1 = Inverted
static int angleRotationDirection = 1;

void invertSteeringDirection() {
  angleRotationDirection = -angleRotationDirection;
  resetEncoderFilter();
}

int getSteeringDirection() {
  return angleRotationDirection;
}

void setSteeringDirection(int dir) {
  angleRotationDirection = (dir < 0) ? -1 : 1;
  resetEncoderFilter();
}

// Maps physical pin states to logical left/right steering directions
constexpr int DIR_THAT_DECREASES_COUNT = HIGH;
constexpr int DIR_LEFT = DIR_THAT_DECREASES_COUNT;
constexpr int DIR_RIGHT = !DIR_THAT_DECREASES_COUNT;

// ============================================================
// PWM Settings
// ============================================================

constexpr int pwmFreq = 5000;    // 5 kHz PWM frequency
constexpr int pwmResolution = 8; // 8-bit resolution (0-255)

// ============================================================
// Limits & Deadband
// ============================================================
int minPWM = 25;  // Minimum PWM to overcome mechanical static friction
int maxPWM = 150; // Maximum safe PWM cap
int deadband =
    18200; // Acceptable count error (~1800 counts is ~0.30 deg, eliminates backlash hunting/jitter!)

// ============================================================
// PID Tuning Parameters (Scaled for 21-bit MT6835 with 1:28 gearbox)
// ============================================================
float Kp = 0.012f; // Proportional response
float Ki = 0.0f;   // Set to 0 to ELIMINATE slow integral windup creep!
float Kd = 0.0018f; // Damping to prevent overshoot and stop jitter

// ============================================================
// Integral Limits
// ============================================================
float maxIntegral = 0.0f; // Not needed with Ki = 0

// ============================================================
// State Variables
// ============================================================

volatile long targetPosition = 0;
bool pidEnabled = false;
bool manualUnrestricted = false;

// ============================================================
// Internal PID States
// ============================================================

static float lastPosF = 0.0f;
static float currentPwm = 0.0f;
static float lastError = 0.0f;
static float integralError = 0.0f;
static unsigned long lastTime = 0;

// ============================================================
// Encoder Health Monitoring
// ============================================================

static bool encoderFault = false;
static long lastEncoderCount = 0;
static unsigned long lastMotionTime = 0;

// If motor is commanded but encoder doesn't move for this duration,
// trigger a safety fault.
constexpr unsigned long ENCODER_TIMEOUT_MS = 100;

// ============================================================
// SETUP STEERING
// ============================================================

void setupSteering() {
  pinMode(PIN_DIR, OUTPUT);
  digitalWrite(PIN_DIR, LOW);
  ledcAttach(PIN_PWM, pwmFreq, pwmResolution);
  ledcWrite(PIN_PWM, 0); // Immediately guarantee motor power is 0 on boot!

  // Initialize Chip Select
  pinMode(PIN_ENC_CS, OUTPUT);
  digitalWrite(PIN_ENC_CS, HIGH);

  // Start SPI bus with custom pin mapping
  // SPI.begin(SCK, MISO, MOSI, CS)
  SPI.begin(PIN_ENC_SCK, PIN_ENC_MISO, PIN_ENC_MOSI, PIN_ENC_CS);

  // Prime first read
  readRawEncoderAngle();
}

// ============================================================
// GET RAW ENCODER COUNT
// ============================================================

long getRawEncoderCount() {
  uint32_t raw = readRawEncoderAngle();
  int32_t diff = (int32_t)raw - (int32_t)deadCenterCount;

  // Handle circular wrap across 180-degree boundary
  if (diff > HALF_ENC_COUNTS) {
    diff -= TOTAL_ENC_COUNTS;
  } else if (diff < -HALF_ENC_COUNTS) {
    diff += TOTAL_ENC_COUNTS;
  }

  return (long)(diff * angleRotationDirection);
}

// ============================================================
// LOW-PASS FILTER STATE & FUNCTIONS
// ============================================================

float encoderLpfAlpha = DEFAULT_ENCODER_LPF_ALPHA;
static float filteredEncoderCount = 0.0f;
static bool filterInitialized = false;
static unsigned long lastEncoderReadMicros = 0;

void setEncoderFilterAlpha(float alpha) {
  encoderLpfAlpha = constrain(alpha, 0.01f, 1.0f);
}

float getEncoderFilterAlpha() {
  return encoderLpfAlpha;
}

void resetEncoderFilter() {
  filterInitialized = false;
  lastEncoderReadMicros = 0;
}

float getFilteredEncoderCount() {
  getEncoderCount(); // Ensure the filter holds the most recent sample
  return filteredEncoderCount;
}

// ============================================================
// GET FILTERED ENCODER COUNT
// ============================================================

long getEncoderCount() {
  unsigned long nowMicros = micros();

  // Rate-limit SPI reads and filter steps to at most once per 1000us (1ms / 1kHz).
  // This prevents back-to-back calls in the same loop cycle from applying duplicate
  // filter advances or overloading the SPI bus.
  if (!filterInitialized || (nowMicros - lastEncoderReadMicros >= 1000)) {
    lastEncoderReadMicros = nowMicros;
    long raw = getRawEncoderCount();

    if (!filterInitialized) {
      filteredEncoderCount = (float)raw;
      filterInitialized = true;
    } else {
      // 1st-Order Exponential Moving Average (EMA) Low-Pass Filter:
      // y[k] = y[k-1] + alpha * (x[k] - y[k-1])
      filteredEncoderCount += encoderLpfAlpha * ((float)raw - filteredEncoderCount);
    }
  }

  return (long)roundf(filteredEncoderCount);
}

static bool magnetDetected = false;
static uint8_t lastRawStatus = 0;
static uint8_t lastRawB0 = 0, lastRawB1 = 0, lastRawB2 = 0;

uint32_t readRawEncoderAngle() {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  digitalWrite(PIN_ENC_CS, LOW);

  // Command to read starting at angle register 0x003
  SPI.transfer(0xA0);
  SPI.transfer(0x03);

  uint8_t b0 = SPI.transfer(0x00); // Angle bits [20:13]
  uint8_t b1 = SPI.transfer(0x00); // Angle bits [12:5]
  uint8_t b2 = SPI.transfer(0x00); // Angle bits [4:0] (top 5 bits) + status

  digitalWrite(PIN_ENC_CS, HIGH);
  SPI.endTransaction();

  lastRawB0 = b0;
  lastRawB1 = b1;
  lastRawB2 = b2;

  // Status bits are in bits 2..0 of b2
  lastRawStatus = b2 & 0x07;
  bool allZero = (b0 == 0 && b1 == 0 && b2 == 0);
  bool allFF   = (b0 == 0xFF && b1 == 0xFF && b2 == 0xFF);

  // Valid as long as SPI wires are active and delivering real data
  magnetDetected = !allZero && !allFF;

  return ((uint32_t)b0 << 13) | ((uint32_t)b1 << 5) | (b2 >> 3);
}

bool isMagnetValid() {
  return magnetDetected;
}

uint8_t getMagnetStatusBits() {
  return lastRawStatus;
}

void getRawSpiBytes(uint8_t &b0, uint8_t &b1, uint8_t &b2) {
  b0 = lastRawB0;
  b1 = lastRawB1;
  b2 = lastRawB2;
}

// ============================================================
// ZERO ENCODER
// ============================================================

void zeroEncoderCount() {
  // Save the current physical position as the straight-ahead reference
  deadCenterCount = readRawEncoderAngle();
  resetEncoderFilter();
  clearEncoderFault();
  float rawDeg = (deadCenterCount * 360.0f) / (float)TOTAL_ENC_COUNTS;
  Serial.printf("\n>>> [ENCODER] Calibrated new Dead Center: %u (%.2f deg)\n", deadCenterCount, rawDeg);
  Serial.printf(">>> Set 'DEFAULT_DEAD_CENTER = %u;' in steering.h to make permanent.\n\n", deadCenterCount);
}

// ============================================================
// MAGNET TELEMETRY HELPERS
// ============================================================

uint32_t getRawMagnetCount() {
  return readRawEncoderAngle();
}

float getRawMagnetDegrees() {
  return (readRawEncoderAngle() * 360.0f) / (float)TOTAL_ENC_COUNTS;
}

uint32_t getDeadCenterCount() {
  return deadCenterCount;
}

// ============================================================
// ENCODER FAULT STATUS
// ============================================================

bool isEncoderFault() { return encoderFault; }

// ============================================================
// CLEAR ENCODER FAULT
// ============================================================

void clearEncoderFault() {

  encoderFault = false;

  lastMotionTime = millis();

  lastEncoderCount = getEncoderCount();
}

// ============================================================
// ENABLE PID
// ============================================================

void enablePID(long newTarget) {
  // Do not engage PID if encoder has a fault
  if (encoderFault) {
    return;
  }

  manualUnrestricted = false;

  // Clamp target to safe mechanical boundaries
  long clampedTarget = constrain(newTarget, MIN_POS_LIMIT, MAX_POS_LIMIT);

  // Initialize PID state ONLY if PID was previously disabled.
  // Note: We do NOT wipe integralError or reset timers when targetPosition moves,
  // allowing smooth continuous tracking during active steering!
  if (!pidEnabled) {
    integralError = 0.0f;
    lastPosF = getFilteredEncoderCount();
    lastTime = millis();
    lastMotionTime = millis();
    lastEncoderCount = getEncoderCount();
    currentPwm = 0.0f;
    pidEnabled = true;
  }

  targetPosition = clampedTarget;
}

// ============================================================
// DISABLE PID
// ============================================================

void disablePID() {
  pidEnabled = false;
  integralError = 0.0f;
  currentPwm = 0.0f;
  ledcWrite(PIN_PWM, 0);
}

// ============================================================
// PID UPDATE
// ============================================================

void updatePID() {
  // Skip if PID disabled or encoder fault
  if (!pidEnabled || encoderFault) {
    return;
  }

  // Safety cutoff: If magnet is missing or SPI is disconnected, kill motor immediately!
  if (!magnetDetected) {
    ledcWrite(PIN_PWM, 0);
    currentPwm = 0.0f;
    return;
  }

  unsigned long now = millis();

  // Calculate delta time
  float dt = (now - lastTime) / 1000.0f;

  // Minimum update interval = 5 ms (up to 200 Hz update rate)
  if (dt < 0.005f) {
    return;
  }

  lastTime = now;

  // --------------------------------------------------------
  // Read filtered encoder position
  // --------------------------------------------------------
  float currentPosF = getFilteredEncoderCount();
  long currentPos = (long)roundf(currentPosF);

  // Calculate position error
  float error = (float)(targetPosition - currentPosF);
  float absError = abs(error);

  // --------------------------------------------------------
  // Soft Deadband Check
  // --------------------------------------------------------
  if (absError <= (float)deadband) {
    ledcWrite(PIN_PWM, 0);
    currentPwm = 0.0f;
    lastMotionTime = now;
    lastEncoderCount = currentPos;
    lastPosF = currentPosF;
    return;
  }

  // --------------------------------------------------------
  // Encoder Safety Check
  // --------------------------------------------------------
  if (abs(currentPos - lastEncoderCount) > 10) {
    lastEncoderCount = currentPos;
    lastMotionTime = now;
  } else if (now - lastMotionTime > ENCODER_TIMEOUT_MS) {
    encoderFault = true;
    stopSteeringMotor();
    Serial.println("[SAFETY ALERT] Encoder not responding! "
                   "Steering disabled.");
    return;
  }

  // --------------------------------------------------------
  // Integral with anti-windup
  // --------------------------------------------------------
  integralError += error * dt;
  integralError = constrain(integralError, -maxIntegral, maxIntegral);

  // --------------------------------------------------------
  // Derivative on Measurement (Prevents derivative kick on setpoint moves)
  // --------------------------------------------------------
  float dPos = (currentPosF - lastPosF) / dt;
  lastPosF = currentPosF;
  float derivative = -dPos; // Damping opposing motion

  // --------------------------------------------------------
  // PID Output
  // --------------------------------------------------------
  float output = (Kp * error) + (Ki * integralError) + (Kd * derivative);

  // --------------------------------------------------------
  // Direction with anti-backlash transition
  // --------------------------------------------------------
  int desiredDir = (output > 0) ? DIR_RIGHT : DIR_LEFT;
  static int activeDir = DIR_RIGHT;
  if (activeDir != desiredDir) {
    // Briefly drop PWM to 0 on direction change to protect gearbox and motor driver
    ledcWrite(PIN_PWM, 0);
    currentPwm = 0.0f;
    digitalWrite(PIN_DIR, desiredDir);
    activeDir = desiredDir;
  }

  // --------------------------------------------------------
  // Smooth Friction Compensation (Transition Band)
  // --------------------------------------------------------
  // Linearly ramps from 0 to minPWM over TRANSITION_BAND counts
  // Eliminates hard 0 -> 25 PWM step cliff when exiting deadband
  constexpr float TRANSITION_BAND = 1000.0f;
  float frictionBoost = 0.0f;
  if (absError < ((float)deadband + TRANSITION_BAND)) {
    frictionBoost = (float)minPWM * ((absError - (float)deadband) / TRANSITION_BAND);
  } else {
    frictionBoost = (float)minPWM;
  }

  // --------------------------------------------------------
  // Desired PWM with Slew-Rate Limiter (Prevents current/torque shocks)
  // --------------------------------------------------------
  float desiredPwm = abs(output) + frictionBoost;
  desiredPwm = constrain(desiredPwm, 0.0f, (float)maxPWM);

  // Maximum slew rate: 600 PWM units/sec (~250ms for full 0 to 150 ramp)
  float maxPwmStep = 600.0f * dt;
  if (desiredPwm > currentPwm + maxPwmStep) {
    currentPwm += maxPwmStep;
  } else if (desiredPwm < currentPwm - maxPwmStep) {
    currentPwm -= maxPwmStep;
  } else {
    currentPwm = desiredPwm;
  }

  int finalPwm = (int)roundf(currentPwm);

  // --------------------------------------------------------
  // Software Limit Safety
  // --------------------------------------------------------
  if ((currentPos >= MAX_POS_LIMIT && output > 0) ||
      (currentPos <= MIN_POS_LIMIT && output < 0)) {
    ledcWrite(PIN_PWM, 0);
    currentPwm = 0.0f;
    return;
  }

  // --------------------------------------------------------
  // Hard Runaway Protection
  // If position exceeds +/-75 degrees, immediately trip safety cutoff
  // to protect the linkage and prevent endless rotation.
  // --------------------------------------------------------
  constexpr long HARD_STOP_LIMIT = degToCounts(75.0f);
  if (abs(currentPos) > HARD_STOP_LIMIT) {
    encoderFault = true;
    stopSteeringMotor();
    Serial.println("\n[SAFETY ALERT] Steering runaway detected (exceeded +/-75 deg)! Motor STOPPED.");
    Serial.println(">>> Motor was likely driving in the wrong direction (Positive Feedback).");
    Serial.println(">>> Press 'I' to invert steering direction, then 'C' to re-center or 'Z' to re-calibrate.\n");
    return;
  }

  ledcWrite(PIN_PWM, finalPwm);
}

// ============================================================
// MANUAL LIMIT CHECK
// ============================================================

void checkManualLimits() {

  // Don't interfere with PID
  // or unrestricted calibration
  if (pidEnabled || manualUnrestricted) {
    return;
  }

  long currentPos = getEncoderCount();

  int currentDir = digitalRead(PIN_DIR);

  // --------------------------------------------------------
  // Left limit
  // --------------------------------------------------------

  if (currentPos >= MAX_POS_LIMIT && currentDir == DIR_RIGHT) {

    ledcWrite(PIN_PWM, 0);
  }

  // --------------------------------------------------------
  // Right limit
  // --------------------------------------------------------

  else if (currentPos <= MIN_POS_LIMIT && currentDir == DIR_LEFT) {

    ledcWrite(PIN_PWM, 0);
  }
}

// ============================================================
// JOG LEFT
// ============================================================

void jogLeft(bool unrestricted) {

  disablePID();

  manualUnrestricted = unrestricted;

  // Move if unrestricted OR not at left limit
  if (unrestricted || getEncoderCount() > MIN_POS_LIMIT) {

    digitalWrite(PIN_DIR, DIR_LEFT);

    ledcWrite(PIN_PWM, minPWM + 30);

    Serial.println(unrestricted ? "Calibration LEFT..."
                                : "Manual LEFT (Limit Protected)...");
  }

  else {

    ledcWrite(PIN_PWM, 0);

    Serial.println("Manual LEFT blocked: "
                   "At/Past MIN limit");
  }
}

// ============================================================
// JOG RIGHT
// ============================================================

void jogRight(bool unrestricted) {

  disablePID();

  manualUnrestricted = unrestricted;

  // Move if unrestricted OR not at right limit
  if (unrestricted || getEncoderCount() < MAX_POS_LIMIT) {

    digitalWrite(PIN_DIR, DIR_RIGHT);

    ledcWrite(PIN_PWM, minPWM + 30);

    Serial.println(unrestricted ? "Calibration RIGHT..."
                                : "Manual RIGHT (Limit Protected)...");
  }

  else {

    ledcWrite(PIN_PWM, 0);

    Serial.println("Manual RIGHT blocked: "
                   "At/Past MAX limit");
  }
}

// ============================================================
// STOP STEERING MOTOR
// ============================================================

void stopSteeringMotor() {

  disablePID();

  manualUnrestricted = false;

  ledcWrite(PIN_PWM, 0);
}

float getSteeringAngleDegrees() {
  return (getFilteredEncoderCount() * 360.0f) / (float)TOTAL_ENC_COUNTS;
}
