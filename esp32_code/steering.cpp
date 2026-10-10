#include "steering.h"

// ============================================================
// Direction Conventions
// ============================================================

// Calibrated straight-ahead dead center count
static uint32_t deadCenterCount = DEFAULT_DEAD_CENTER;

// Direction convention: 1 = Normal, -1 = Inverted
constexpr int ANGLE_ROTATION_DIRECTION = 1;

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
    150; // Acceptable count error (~150 counts is only 0.025 degrees!)

// ============================================================
// PID Tuning Parameters (Scaled down for 21-bit MT6835)
// ============================================================
float Kp = 0.012f;   // Down from 0.20 (scaled down ~16x for 21-bit counts)
float Ki = 0.00005f; // Down from 0.001
float Kd = 0.0015f;  // Down from 0.02

// ============================================================
// Integral Limits
// ============================================================
float maxIntegral = 10000.0f; // Adjusted for larger count error accumulation

// ============================================================
// State Variables
// ============================================================

volatile long targetPosition = 0;
bool pidEnabled = false;
bool manualUnrestricted = false;

// ============================================================
// Internal PID States
// ============================================================

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
  ledcAttach(PIN_PWM, pwmFreq, pwmResolution);

  // Initialize Chip Select
  pinMode(PIN_ENC_CS, OUTPUT);
  digitalWrite(PIN_ENC_CS, HIGH);

  // Start SPI bus with custom pin mapping
  // SPI.begin(SCK, MISO, MOSI, CS)
  SPI.begin(PIN_ENC_SCK, PIN_ENC_MISO, PIN_ENC_MOSI, PIN_ENC_CS);
}

// ============================================================
// SETUP NEW PCNT DRIVER
// ============================================================

// ============================================================
// GET ENCODER COUNT
// ============================================================

long getEncoderCount() {
  uint32_t raw = readRawEncoderAngle();
  int32_t diff = (int32_t)raw - (int32_t)deadCenterCount;

  // Handle circular wrap across 180-degree boundary
  if (diff > HALF_ENC_COUNTS) {
    diff -= TOTAL_ENC_COUNTS;
  } else if (diff < -HALF_ENC_COUNTS) {
    diff += TOTAL_ENC_COUNTS;
  }

  return (long)(diff * ANGLE_ROTATION_DIRECTION);
}

// ============================================================
// MT6835 SPI Read Protocol
// ============================================================

uint32_t readRawEncoderAngle() {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  digitalWrite(PIN_ENC_CS, LOW);

  // Command to read angle register 0x003
  SPI.transfer(0xA0);
  SPI.transfer(0x03);

  uint8_t b0 = SPI.transfer(0x00); // Angle bits [20:13]
  uint8_t b1 = SPI.transfer(0x00); // Angle bits [12:5]
  uint8_t b2 = SPI.transfer(0x00); // Angle bits [4:0] (top 5 bits) + status

  digitalWrite(PIN_ENC_CS, HIGH);
  SPI.endTransaction();

  return ((uint32_t)b0 << 13) | ((uint32_t)b1 << 5) | (b2 >> 3);
}

// ============================================================
// ZERO ENCODER
// ============================================================

void zeroEncoderCount() {
  // Save the current physical position as the straight-ahead reference
  deadCenterCount = readRawEncoderAngle();
  clearEncoderFault();
  Serial.printf("[ENCODER] Calibrated new Dead Center: %u\n", deadCenterCount);
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

  // Reinitialize PID if target changed
  // or PID was previously disabled
  if (!pidEnabled || targetPosition != clampedTarget) {

    integralError = 0.0f;

    lastError = (float)(clampedTarget - getEncoderCount());

    lastTime = millis();

    lastMotionTime = millis();

    lastEncoderCount = getEncoderCount();

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

  unsigned long now = millis();

  // Calculate delta time
  float dt = (now - lastTime) / 1000.0f;

  // Minimum update interval = 5 ms
  if (dt < 0.005f) {
    return;
  }

  lastTime = now;

  // --------------------------------------------------------
  // Read encoder
  // --------------------------------------------------------

  long currentPos = getEncoderCount();

  // Calculate position error
  float error = (float)(targetPosition - currentPos);

  // --------------------------------------------------------
  // Deadband
  // --------------------------------------------------------

  if (abs(error) <= deadband) {

    ledcWrite(PIN_PWM, 0);

    lastMotionTime = now;

    lastEncoderCount = currentPos;

    return;
  }

  // --------------------------------------------------------
  // Encoder Safety Check
  // --------------------------------------------------------

  if (abs(currentPos - lastEncoderCount) > 10) {
    lastEncoderCount = currentPos;
    lastMotionTime = now;
  }

  else if (now - lastMotionTime > ENCODER_TIMEOUT_MS) {

    encoderFault = true;

    stopSteeringMotor();

    Serial.println("[SAFETY ALERT] Encoder not responding! "
                   "Steering disabled.");

    return;
  }

  // --------------------------------------------------------
  // Integral
  // --------------------------------------------------------

  integralError += error * dt;

  integralError = constrain(integralError, -maxIntegral, maxIntegral);

  // --------------------------------------------------------
  // Derivative
  // --------------------------------------------------------

  float derivative = (error - lastError) / dt;

  lastError = error;

  // --------------------------------------------------------
  // PID Output
  // --------------------------------------------------------

  float output = (Kp * error) + (Ki * integralError) + (Kd * derivative);

  // --------------------------------------------------------
  // Direction
  // --------------------------------------------------------

  if (output > 0) {

    digitalWrite(PIN_DIR, DIR_RIGHT);

  } else {

    digitalWrite(PIN_DIR, DIR_LEFT);
  }

  // --------------------------------------------------------
  // PWM Magnitude
  // --------------------------------------------------------

  int pwmVal = (int)abs(output);

  // Minimum PWM to overcome friction
  if (pwmVal < minPWM) {
    pwmVal = minPWM;
  }

  // Maximum safe PWM
  pwmVal = constrain(pwmVal, 0, maxPWM);

  // --------------------------------------------------------
  // Software Limit Safety
  // --------------------------------------------------------

  if ((currentPos >= MAX_POS_LIMIT && output > 0) ||
      (currentPos <= MIN_POS_LIMIT && output < 0)) {

    ledcWrite(PIN_PWM, 0);

  } else {

    ledcWrite(PIN_PWM, pwmVal);
  }
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
  return (getEncoderCount() * 360.0f) / (float)TOTAL_ENC_COUNTS;
}
