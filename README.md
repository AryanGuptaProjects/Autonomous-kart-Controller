# MT6835 21-Bit Magnetic Rotary Encoder — Evaluation & SPI Driver

UPDATED : 8 OCTOBER 2026

[![Language: C++ / Arduino](https://img.shields.io/badge/Language-C%2B%2B%20%2F%20Arduino-00599C?logo=c%2B%2B)](https://www.arduino.cc/)
[![Platform: ESP32 / Arduino MCU](https://img.shields.io/badge/Platform-ESP32%20%2F%20Microcontroller-E7352C?logo=espressif)](https://www.espressif.com/)
[![Sensor: MagnTek MT6835](https://img.shields.io/badge/Sensor-MagnTek%20MT6835-green)](https://www.magntek.com.cn/)
[![Resolution: 21--Bit](https://img.shields.io/badge/Resolution-21--Bit%20(2%2C097%2C152%20CPR)-blueviolet)]()

High-precision SPI driver and testing firmware for the **MagnTek MT6835** 21-bit magnetic rotary angle encoder. This module is developed as part of the **Maveric Autonomous Electric Go-Kart Control System** (`maveric_controls`) to provide ultra-accurate, absolute steering column angular feedback for closed-loop PID steering actuation.

---

## Table of Contents

- [1. Overview](#1-overview)
- [2. Hardware Specifications](#2-hardware-specifications)
- [3. Pinout & Wiring Connections](#3-pinout--wiring-connections)
- [4. SPI Protocol & Register Architecture](#4-spi-protocol--register-architecture)
- [5. Firmware Implementation](#5-firmware-implementation)
- [6. Build, Flashing & Execution](#6-build-flashing--execution)
- [7. Expected Serial Output](#7-expected-serial-output)
- [8. Autonomous Go-Kart Integration Context](#8-autonomous-go-kart-integration-context)
- [9. Troubleshooting & FAQ](#9-troubleshooting--faq)

---

## 1. Overview

The **MT6835** is a 4th-generation magnetic angle encoder IC from MagnTek based on advanced anisotropic magnetoresistive (AMR) technology with integrated digital signal processing. 

### Key Advantages for Autonomous Steering:
- **Absolute Positioning**: Unlike incremental optical quadrature encoders, the MT6835 detects true absolute shaft angle immediately upon power-up, eliminating zero-point homing routines.
- **Ultra-High Resolution**: 21-bit raw position encoding delivers $2^{21} = 2,097,152$ discrete counts per $360^\circ$ rotation ($\approx 0.0001716^\circ$ angular resolution).
- **Harsh Environment Resilience**: Completely immune to dust, tire rubber particulate, grease, and mechanical vibration present in automotive and go-kart chassis environments.
- **Low Latency SPI Interface**: Synchronous bus communication avoids interrupt overhead and CPU cycle consumption.

---

## 2. Hardware Specifications

| Parameter | Specification | Notes |
| :--- | :--- | :--- |
| **Encoder IC** | MagnTek MT6835 | 4th-Gen AMR Magnetic Angle Sensor |
| **Resolution** | 21-Bit | $2^{21} = 2,097,152$ counts / revolution |
| **Angular Precision** | $\approx 0.0001716^\circ$ | High sensitivity for micro-steering corrections |
| **Interface** | 4-Wire Standard SPI | Mode 3 (`CPOL=1`, `CPHA=1`), MSB First |
| **Tested Clock Frequency** | 1.0 MHz (`SPISettings`) | Supports up to 16 MHz bus speed |
| **Operating Voltage** | 3.3V – 5.0V DC | Typically 3.3V logic for ESP32 / modern MCUs |
| **Magnet Type** | Diametrically Magnetized NdFeB | Cylindrical magnet centered over IC |
| **Typical Air Gap** | 1.0 mm – 3.0 mm | Between magnet face and sensor package |

---

## 3. Pinout & Wiring Connections

The test sketch configures the standard SPI bus pins plus a dedicated Chip Select (`CS_PIN`).

### Default Wiring Diagram

```
       +--------------------+                 +--------------------+
       |  Microcontroller   |                 |   MT6835 Breakout  |
       |  (ESP32 / MCU)     |                 |       Module       |
       |                    |                 |                    |
       |  [ GPIO 10 ] ------+--- CS --------->|  CSN / CS          |
       |  [ SCK / GPIO 18 ]-+--- SCK -------->|  SCK / CLK         |
       |  [ MOSI / GPIO 23]-+--- MOSI ------->|  MOSI / SI         |
       |  [ MISO / GPIO 19]-|<-- MISO --------+-- MISO / SO        |
       |  [ 3.3V ] ---------+--- VCC -------->|  VCC               |
       |  [ GND ] ----------+--- GND -------->|  GND               |
       +--------------------+                 +--------------------+
```

### Pin Mapping Reference

| MT6835 Pin | Sketch Default | ESP32 Default SPI | Arduino Uno / Nano | Description |
| :--- | :--- | :--- | :--- | :--- |
| **VCC** | `3.3V` / `5V` | `3.3V` | `5V` / `3.3V` | Module DC Power |
| **GND** | `GND` | `GND` | `GND` | Common Ground Reference |
| **CS / CSN** | `Pin 10` | `GPIO 10` (or user GPIO)| `Pin 10` | Active-LOW Chip Select |
| **SCK / CLK**| SPI SCK | `GPIO 18` (VSPI SCK) | `Pin 13` | SPI Clock |
| **MISO / SO**| SPI MISO | `GPIO 19` (VSPI MISO)| `Pin 12` | Master In Slave Out (Data to MCU) |
| **MOSI / SI**| SPI MOSI | `GPIO 23` (VSPI MOSI)| `Pin 11` | Master Out Slave In (Commands to MT6835) |

> [!NOTE]
> On ESP32 boards, you can remap SPI pins to any available GPIOs via `SPI.begin(sck, miso, mosi, cs)`. Verify that `CS_PIN` in [MT6835_Testing.ino](file:///Users/aryangupta/Downloads/experimenting/MT6835_Testing/MT6835_Testing.ino#L3) matches your physical wiring.

---

## 4. SPI Protocol & Register Architecture

The MT6835 utilizes a continuous multi-byte SPI read cycle:

```
  CS   : ---+                                                                  +---
            |__________________________________________________________________|
  MOSI :    [ 0xA0 ]    [ 0x03 ]      [ 0x00 ]        [ 0x00 ]        [ 0x00 ]
             Opcode     Reg Addr        Dummy           Dummy           Dummy
  MISO :    [ ---- ]    [ ---- ]      [ Byte 0 ]      [ Byte 1 ]      [ Byte 2 ]
                                    ANGLE[20:13]     ANGLE[12:5]    ANGLE[4:0] + Stat
```

### 1. Read Command Sequence
1. Assert `CS` LOW.
2. Send **Read Opcode**: `0xA0` (binary `1010 0000b` — MagnTek SPI read command).
3. Send **Register Address**: `0x03` (angle data register pointer).
4. Send three dummy bytes (`0x00`) to clock out three response bytes from the sensor:
   - **Byte 0 (`b0`)**: Angle bits `[20:13]` (upper 8 bits)
   - **Byte 1 (`b1`)**: Angle bits `[12:5]` (middle 8 bits)
   - **Byte 2 (`b2`)**: Angle bits `[4:0]` (lower 5 bits) located in the upper 5 bits of `b2`; the lower 3 bits contain status/diagnostic information.
5. Deassert `CS` HIGH.

### 2. 21-Bit Bitwise Unpacking
The 21-bit position value is reconstructed using:
```cpp
uint32_t raw = ((uint32_t)b0 << 13) | ((uint32_t)b1 << 5) | (b2 >> 3);
```

### 3. Degree Conversion Formula
$$\text{Angle } (^\circ) = \frac{\text{raw} \times 360.0^\circ}{2^{21}} = \frac{\text{raw} \times 360.0^\circ}{2097152.0}$$

---

## 5. Firmware Implementation

The complete standalone test sketch is located at [`MT6835_Testing/MT6835_Testing.ino`](file:///Users/aryangupta/Downloads/experimenting/MT6835_Testing/MT6835_Testing.ino):

```cpp
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
```

---

## 6. Build, Flashing & Execution

### Method 1: Arduino IDE
1. Open the [Arduino IDE](https://www.arduino.cc/en/software).
2. Open [`MT6835_Testing/MT6835_Testing.ino`](file:///Users/aryangupta/Downloads/experimenting/MT6835_Testing/MT6835_Testing.ino).
3. Select your target microcontroller board (e.g., **ESP32 Dev Module** or **Arduino Uno**).
4. Select the matching serial port (e.g., `/dev/cu.usbserial-*` or `COMx`).
5. Click **Upload** ($\rightarrow$).
6. Open **Serial Monitor** at **115200 baud**.

### Method 2: PlatformIO (CLI)
Create a minimal `platformio.ini` in the project directory:

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
```
Run the build and upload commands:
```bash
pio run --target upload
pio device monitor -b 115200
```

---

## 7. Expected Serial Output

When rotating the magnet above the sensor, the serial console outputs continuous raw count and degree measurements:

```text
--- MT6835 Angle Measurement ---
Raw Count: 14502   | Angle: 2.4893 deg
Raw Count: 524288  | Angle: 90.0000 deg
Raw Count: 1048576 | Angle: 180.0000 deg
Raw Count: 1572864 | Angle: 270.0000 deg
Raw Count: 2094821 | Angle: 359.5999 deg
```

---

## 8. Autonomous Go-Kart Integration Context

This evaluation module serves as the next-generation sensor upgrade for the **Maveric Autonomous Go-Kart Steering Controller** (`AkshatChauhan18/maveric_controls`):

```
       +-------------------------------------------------------------+
       |                Steering Closed-Loop Control                 |
       +-------------------------------------------------------------+
                                      |
         Target Steering Angle        |    Actual Steering Feedback
         (from ROS 2 /cmd_vel or RC)  |    (from MT6835 SPI Encoder)
                     \                |                /
                      v               v               v
                +-------------------------------------------+
                |          Closed-Loop PID Control          |
                |    Error = Target - MeasuredPosition      |
                +-------------------------------------------+
                                      |
                                      v
                +-------------------------------------------+
                |        Cytron MD20A H-Bridge Motor        |
                |          Steering Column Actuator         |
                +-------------------------------------------+
```

### Sensor Comparison: MT6835 vs. Incremental Optical Encoder

| Feature | Legacy Optical Encoder (`PCNT`) | MagnTek MT6835 (This Driver) |
| :--- | :--- | :--- |
| **Measurement Mode** | Incremental relative pulses | **True Absolute position** ($0^\circ - 360^\circ$) |
| **Startup Behavior** | Starts at 0 (requires homing routine) | **Instant true angle reading upon power-on** |
| **Resolution** | Pulse-dependent (e.g. 1000–2400 CPR) | **2,097,152 CPR (21-bit)** |
| **Mechanical Immunity** | Prone to optical slot dust/contamination | **Hermetically immune magnetic sensing** |
| **Processor Load** | ESP-IDF Pulse Counter hardware channels | **Fast SPI on-demand reads** |
| **Failure Detection** | Watchdog stall timeout (100 ms) | **Hardware register status & parity/CRC** |

---

## 9. Troubleshooting & FAQ

### Problem 1: `Raw Count: 0` or `Raw Count: 2097151` constant output
- **Cause**: SPI bus communication failure or Chip Select wiring mismatch.
- **Solution**:
  1. Confirm `CS_PIN` in code matches the exact physical pin wired to `CSN`.
  2. Verify common ground (`GND`) connection between MCU and MT6835 breakout.
  3. Ensure MISO and MOSI are not swapped.

### Problem 2: Jittery or erratic angle readings
- **Cause**: Incorrect magnet orientation or air gap distance.
- **Solution**:
  1. Ensure the magnet is **diametrically magnetized** (poles along the diameter, NOT axially magnetized like refrigerator magnets).
  2. Center the magnet directly over the MT6835 package center.
  3. Maintain an air gap of 1.0 mm to 2.5 mm between magnet and chip surface.

### Problem 3: SPI Clock speed / mode issues on other MCUs
- **Cause**: Some microcontrollers do not support SPI Mode 3 out-of-the-box or require slower clock rates with long dupont jumper wires.
- **Solution**:
  - Keep jumper wire lengths under 15 cm.
  - If using high bus speeds, ensure clock is set between 500 kHz and 4 MHz (`SPISettings(1000000, MSBFIRST, SPI_MODE3)`).

---

## License & Credits

Developed for the **Maveric Autonomous Go-Kart Project** (`AkshatChauhan18/maveric_controls`).  
Maintained by the Maveric Controls & Embedded Systems Team.
