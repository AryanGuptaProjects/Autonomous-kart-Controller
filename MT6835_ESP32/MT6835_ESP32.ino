#include <SPI.h>

// --- ESP32 Custom SPI Pins ---
const int CS_PIN   = 4;
const int SCK_PIN  = 14;
const int MISO_PIN = 35;  // (or 33)
const int MOSI_PIN = 23;

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
  
  // Initialize ESP32 SPI with your custom routed pins
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
  
  Serial.println("--- MT6835 Steering Angle Measurement (ESP32) ---");
}


// Returns raw angle and sets 'magnetPresent' to false if the magnet is missing
uint32_t readRawAngle(bool &magnetPresent) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  
  digitalWrite(CS_PIN, LOW);
  
  // Command to read starting at register 0x003
  SPI.transfer(0xA0); 
  SPI.transfer(0x03);
  
  uint8_t b0 = SPI.transfer(0x00); // ANGLE[20:13]
  uint8_t b1 = SPI.transfer(0x00); // ANGLE[12:5]
  uint8_t b2 = SPI.transfer(0x00); // ANGLE[4:0] (bits 7..3) + STATUS (bits 2..0)
  
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();

  // Extract status bits (bits 2..0 of b2)
  uint8_t status = b2 & 0x07;
  
  // Bit 1 (0x02) triggers high when the magnetic field is too weak or missing
  bool lowMagField = (status & 0x02) != 0;
  
  // Magnet is present if low magnetic field warning is NOT active
  magnetPresent = !lowMagField; 

  // Extract 21-bit angle position
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
  bool isMagnetValid = true;
  uint32_t raw = readRawAngle(isMagnetValid);

  int32_t steeringCounts = 0;
  int32_t clampedCounts  = 0;
  float displayAngleDeg  = 0.0f;

  if (!isMagnetValid) {
    // Force output to 0 if magnet is missing or out of range
    steeringCounts  = 0;
    clampedCounts   = 0;
    displayAngleDeg = 0.0f;

    Serial.println("[ERROR] Magnet Not Detected / Removed! Output forced to 0.");
  } else {
    // Normal operation when magnet is detected
    steeringCounts = getSteeringCounts(raw);
    clampedCounts  = constrain(steeringCounts, MIN_STEERING_COUNTS, MAX_STEERING_COUNTS);
    
    bool isAtLimit      = (steeringCounts > MAX_STEERING_COUNTS) || (steeringCounts < MIN_STEERING_COUNTS);
    displayAngleDeg     = (clampedCounts * 360.0f) / (float)TOTAL_COUNTS;
    float sensorDegrees = (raw * 360.0f) / (float)TOTAL_COUNTS;

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
  }

  delay(100);
}
