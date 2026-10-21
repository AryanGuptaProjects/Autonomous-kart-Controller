#ifndef STEERING_H
#define STEERING_H
#include <SPI.h>

#include <Arduino.h>

// --- Pin Definitions for Steering Control ---
constexpr int PIN_PWM   = 26;
constexpr int PIN_DIR   = 27;

// --- MT6835 SPI Pin Definition ---
constexpr int PIN_ENC_CS = 4;
constexpr int PIN_ENC_SCK = 14;
constexpr int PIN_ENC_MISO = 35;
constexpr int PIN_ENC_MOSI = 23;

// --- MT6835 21-Bit Resolution Constants ---
constexpr uint32_t TOTAL_ENC_COUNTS = 2097152;          // 2^21 counts per 360 deg
constexpr int32_t  HALF_ENC_COUNTS  = TOTAL_ENC_COUNTS / 2; // 1,048,576
constexpr uint32_t DEFAULT_DEAD_CENTER = 1823624;      // Calibrated straight-ahead point
constexpr int32_t degToCounts(float deg) {
    return (int32_t)((deg * (float)TOTAL_ENC_COUNTS) / 360.0f);
}
// Steering mechanical limits (e.g., +/- 45 deg or +/- 60 deg)
constexpr long MAX_POS_LIMIT = degToCounts(60.0f);  // ~+349,525 counts
constexpr long MIN_POS_LIMIT = degToCounts(-60.0f); // ~-349,525 counts

// --- Global State Variables ---
extern volatile long targetPosition;
extern bool pidEnabled;
extern bool manualUnrestricted;

// --- Initialization Functions ---
void setupSteering();
uint32_t readRawEncoderAngle();
float getSteeringAngleDegrees();
uint32_t getRawMagnetCount();
float getRawMagnetDegrees();
uint32_t getDeadCenterCount();
void invertSteeringDirection();
int getSteeringDirection();
void setSteeringDirection(int dir);
bool isMagnetValid();
uint8_t getMagnetStatusBits();
void getRawSpiBytes(uint8_t &b0, uint8_t &b1, uint8_t &b2);


// --- Low-Pass Filter Configuration ---
constexpr float DEFAULT_ENCODER_LPF_ALPHA = 0.15f; // Filter smoothing factor (0.01 = heavy smoothing, 1.0 = raw)
extern float encoderLpfAlpha;

void setEncoderFilterAlpha(float alpha);
float getEncoderFilterAlpha();
void resetEncoderFilter();
float getFilteredEncoderCount();

// --- Encoder Functions ---
long getEncoderCount();
long getRawEncoderCount();
void zeroEncoderCount();

// --- PID Controller Functions ---
void enablePID(long newTarget);
void disablePID();
void updatePID();
void checkManualLimits();

// --- Manual Control Functions ---
void jogLeft(bool unrestricted);
void jogRight(bool unrestricted);
void stopSteeringMotor();

// --- Safety & Encoder Diagnostics ---
bool isEncoderFault();
void clearEncoderFault();

#endif