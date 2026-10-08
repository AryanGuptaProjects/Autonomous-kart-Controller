#include <SPI.h>

const int CS_PIN = 10;

// Encoder resolution constants (21-bit)
const uint32_t TOTAL_COUNTS    = 2097152;          // Total counts in 360 deg (2^21)
const int32_t  HALF_COUNTS     = TOTAL_COUNTS / 2; // Counts in 180 deg (1,048,576)

// Calibrated straight-ahead dead center (346.5300 deg = 2018873 counts)
const uint32_t DEAD_CENTER = 2018873; 

// Direction convention: Anticlockwise = Positive (+), Clockwise = Negative (-)
const int ANGLE_ROTATION_DIRECTION = 1;


// 360 deg ---> 2097152 counts
// 1 deg ---> 5825.422 counts
// Helper to convert degree limits to exact integer counts
constexpr int32_t degToCounts(float deg) {
  return (int32_t)((deg * (float)TOTAL_COUNTS) / 360.0f);
}

// Steering limits stored and enforced strictly in raw counts (e.g. +/- 60.0 deg = +/- 203884 counts)
const int32_t MAX_STEERING_COUNTS = degToCounts(60.0f);  // Anticlockwise limit (+)
const int32_t MIN_STEERING_COUNTS = degToCounts(-60.0f); // Clockwise limit (-)

void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  
  SPI.begin();
  Serial.println("--- MT6835 Steering Angle Measurement ---");
}

uint32_t readRawAngle() {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  
  digitalWrite(CS_PIN, LOW);
  
  // Command to read register 0x003
  SPI.transfer(0xA0); 
  SPI.transfer(0x03);
  
  uint8_t b0 = SPI.transfer(0x00); // ANGLE[20:13]
  uint8_t b1 = SPI.transfer(0x00); // ANGLE[12:5]
  uint8_t b2 = SPI.transfer(0x00); // ANGLE[4:0] (top 5 bits) + status
  
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();

  // Extract 21-bit position value (0 to 2097151)
  return ((uint32_t)b0 << 13) | ((uint32_t)b1 << 5) | (b2 >> 3);
}

// Computes steering position in raw counts relative to dead center (100% integer math)
int32_t getSteeringCounts(uint32_t raw) {
  // 1. Difference from dead center
  int32_t diff = (int32_t)raw - (int32_t)DEAD_CENTER;

  // 2. Handle 360-degree boundary wrap-around in integer count space
  if (diff > HALF_COUNTS) {
    diff -= TOTAL_COUNTS;
  } else if (diff < -HALF_COUNTS) {
    diff += TOTAL_COUNTS;
  }

  // 3. Apply direction: Anticlockwise (+), Clockwise (-)
  return diff * ANGLE_ROTATION_DIRECTION;
}

void loop() {
  uint32_t raw = readRawAngle();

  // 1. Calculate steering offset from dead center in raw ticks (dead center = 0)
  int32_t steeringCounts = getSteeringCounts(raw);

  // 2. Enforce limits strictly in raw counts (integer arithmetic)
  int32_t clampedCounts = constrain(steeringCounts, MIN_STEERING_COUNTS, MAX_STEERING_COUNTS);
  bool isAtLimit = (steeringCounts > MAX_STEERING_COUNTS) || (steeringCounts < MIN_STEERING_COUNTS);

  // --- VISUAL PRESENTATION ONLY (convert to degrees for human monitoring) ---
  float displayAngleDeg = (clampedCounts * 360.0f) / (float)TOTAL_COUNTS;
  float sensorDegrees   = (raw * 360.0f) / (float)TOTAL_COUNTS;

  Serial.print("Raw: ");
  Serial.print(raw);
  Serial.print(" | Sensor: ");
  Serial.print(sensorDegrees, 4);
  Serial.print(" deg | Clamped Counts: ");
  Serial.print(clampedCounts);
  Serial.print(" | Angle: ");
  Serial.print(displayAngleDeg, 4);
  Serial.print(" deg");

  if (isAtLimit) {
    Serial.print(" [LIMIT REACHED]");
  }

  Serial.println();

  delay(100);
}