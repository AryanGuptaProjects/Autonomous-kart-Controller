#include <SPI.h>

const int CS_PIN = 10;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  
  SPI.begin();
  Serial.println("--- MT6835 Angle Measurement ---");
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

  // Extract 21-bit position value
  return ((uint32_t)b0 << 13) | ((uint32_t)b1 << 5) | (b2 >> 3);
}

void loop() {
  uint32_t raw = readRawAngle();
  float degrees = (raw * 360.0f) / 2097152.0f;

  Serial.print("Raw Count: ");
  Serial.print(raw);
  Serial.print(" | Angle: ");
  Serial.print(degrees, 4);
  Serial.println(" deg");

  delay(100);
}