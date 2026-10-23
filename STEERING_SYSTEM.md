# Steering Subsystem: Architecture, Logic & Control Specification

**Document Version:** 1.0  
**Target Platform:** ESP32 (Arduino-ESP32 Framework)  
**Actuator:** High-Torque DC Steering Motor driven via BTS7960 H-Bridge  
**Sensor Interface:** Absolute Rotary Angle Feedback (Zero-Referenced)  
**Primary Source Code References:**
- [steering.h](file:///Users/aryangupta/Downloads/experimenting/esp32_code/steering.h)
- [steering.cpp](file:///Users/aryangupta/Downloads/experimenting/esp32_code/steering.cpp)
- [esp32_code.ino](file:///Users/aryangupta/Downloads/experimenting/esp32_code/esp32_code.ino)

---

## Table of Contents
1. [Executive Summary & System Purpose](#1-executive-summary--system-purpose)
2. [Hardware Interface & Pin Mapping](#2-hardware-interface--pin-mapping)
3. [Coordinate System, Units & Calibration](#3-coordinate-system-units--calibration)
4. [Target Angle Generation & Input Modes](#4-target-angle-generation--input-modes)
5. [Sensor Feedback & Filtering Pipeline](#5-sensor-feedback--filtering-pipeline)
6. [Closed-Loop Controller Logic (PID)](#6-closed-loop-controller-logic-pid)
7. [Motor Drive Translation & Actuation](#7-motor-drive-translation--actuation)
8. [Safety Systems & Fault Interlocks](#8-safety-systems--fault-interlocks)
9. [Calibration & Manual Teleop Commands](#9-calibration--manual-teleop-commands)
10. [Analysis of Current Implementation Flaws](#10-analysis-of-current-implementation-flaws)
11. [Blueprint for Clean-Sheet Rewrite](#11-blueprint-for-clean-sheet-rewrite)

---

## 1. Executive Summary & System Purpose

The steering system converts target steering commands from either a manual RC transmitter (Channel 1) or an autonomous host (ROS 2 / laptop serial) into physical road wheel angles using an electric DC motor connected to an Ackermann steering linkage.

Because steering involves significant static friction (tire contact patch on pavement, kingpin bushings, reduction gearbox) and severe mechanical inertia, the motor must act as a **closed-loop position servo**:
- Continuously measure current road wheel orientation.
- Compare measured angle against commanded target angle.
- Modulate motor speed (PWM) and direction (DIR) to minimize error.
- Enforce mechanical safety limits and automatically shut down if stalled or in runaway.

```
               +------------------------------------------------------+
               |                  Command Sources                     |
               |   - RC Channel 1 Joystick (-1.0 to +1.0)             |
               |   - Laptop / ROS 2 Serial Packet (-100 to +100)      |
               +--------------------------+---------------------------+
                                          |
                                          v
               +------------------------------------------------------+
               |          Input Pre-Processing & Scaling              |
               |   - Joystick deadband & EMA smoothing                |
               |   - Scale to physical travel limits (±60.0 deg)      |
               +--------------------------+---------------------------+
                                          | Target Angle
                                          v
+--------------+               +----------------------+
| Position     |  Feedback     |  Position Error      |
| Sensor       |-------------->|  e(t) = Target - Pos |
+--------------+               +----------+-----------+
                                          |
                                          v
                               +----------------------+
                               |    PID Controller    |
                               |    - Proportional    |
                               |    - Viscous Damping |
                               +----------+-----------+
                                          |
                                          v
                               +----------------------+
                               |   Drive Formatting   |
                               |   - Direction switch |
                               |   - Friction boost   |
                               |   - Slew limiter     |
                               +----------+-----------+
                                          |
                                          v
                               +----------------------+
                               |  Safety Interlocks   |
                               |  - Stall detection   |
                               |  - Travel limits     |
                               |  - Runaway cutoff    |
                               +----------+-----------+
                                          | PWM + DIR
                                          v
                               +----------------------+
                               |  BTS7960 Driver &    |
                               |  DC Steering Motor   |
                               +----------------------+
```

---

## 2. Hardware Interface & Pin Mapping

### 2.1 Microcontroller & Motor Driver Connections
The ESP32 interfaces with a high-current BTS7960 H-bridge motor driver module:

| Signal | ESP32 Pin | Function | Configuration / Hardware Characteristics |
| :--- | :--- | :--- | :--- |
| **PIN_PWM** | **GPIO 26** | Motor Speed / Torque | LEDC hardware PWM timer, 5 kHz carrier frequency, 8-bit resolution (duty cycle: `0` to `255`). |
| **PIN_DIR** | **GPIO 27** | Motor Direction | Standard GPIO digital output (`HIGH` vs `LOW`). |

### 2.2 Direction Pin Logic
The physical direction signal is defined relative to position count changes:
- `DIR_THAT_DECREASES_COUNT = HIGH` $\rightarrow$ Turns steering **LEFT**.
- `!DIR_THAT_DECREASES_COUNT = LOW` $\rightarrow$ Turns steering **RIGHT**.

---

## 3. Coordinate System, Units & Calibration

### 3.1 Units and Scaling
Physical angles are represented internally in high-resolution encoder counts:
- **Full Circle ($360^\circ$):** $2^{21} = 2,097,152\text{ counts}$
- **Resolution:** $\approx 5,825.42\text{ counts per degree}$ ($\approx 0.00017^\circ\text{ per count}$)
- **Conversion Equation:**
  $$\text{counts} = \frac{\text{degrees} \times 2,097,152}{360.0}$$
  $$\text{degrees} = \frac{\text{counts} \times 360.0}{2,097,152}$$

### 3.2 Straight-Ahead Zero Reference (`deadCenterCount`)
- Absolute physical straight-ahead is defined as `0` counts / `0.0°`.
- When calibrated, the sensor's raw orientation reading is saved to `deadCenterCount` (default `1823624`).
- Any measured position relative to center is:
  $$\text{relativeCount} = (\text{rawSensorCount} - \text{deadCenterCount}) \times \text{angleRotationDirection}$$
- Circular wrapping across the $180^\circ$ ($\pm 1,048,576\text{ counts}$) boundary is unwrapped with modular arithmetic so the value smoothly spans positive and negative angles.

### 3.3 Travel & Safety Limits
| Parameter | Value in Degrees | Value in Counts | Description |
| :--- | :--- | :--- | :--- |
| **MAX_POS_LIMIT** | $+60.0^\circ$ | $+349,525$ | Maximum right steering travel lock. |
| **MIN_POS_LIMIT** | $-60.0^\circ$ | $-349,525$ | Maximum left steering travel lock. |
| **HARD_STOP_LIMIT** | $\pm 75.0^\circ$ | $\pm 436,906$ | Emergency runaway threshold; trips motor power off immediately. |

---

## 4. Target Angle Generation & Input Modes

The steering accepts target inputs from two distinct modes managed in [esp32_code.ino](file:///Users/aryangupta/Downloads/experimenting/esp32_code/esp32_code.ino):

### 4.1 Manual RC Mode (Transmitter Channel 1)
When RC Mode is active (Channel 6 switch DOWN), steering targets are derived from Channel 1:
1. **Raw Normalization:** Channel pulse (typically $1000\ \mu\text{s}$ to $2000\ \mu\text{s}$) is converted to normalized float:
   $$\text{rawSteerNorm} \in [-1.0, +1.0]$$
2. **Transmitter Deadband:** If $|\text{rawSteerNorm}| < 0.05$, it is clamped to `0.0` to eliminate trim drift around center.
3. **Input Smoothing (EMA Filter):**
   $$\text{filteredSteerNorm}_k = \text{filteredSteerNorm}_{k-1} + 0.20 \times (\text{rawSteerNorm}_k - \text{filteredSteerNorm}_{k-1})$$
4. **Auto-Centering Snap:** When the driver releases the stick into the center deadband, the filtered value decays and snaps to exactly `0.0` (dead center).
5. **Target Position Computation:**
   $$\text{targetPos} = \text{filteredSteerNorm} \times \text{MAX\_POS\_LIMIT}$$

### 4.2 Autonomous / Laptop Mode (Binary Protocol)
When Autonomous Mode is active (Channel 6 switch UP), steering is commanded via a 3-byte binary struct over USB-UART:
```cpp
struct KartCommand {
    int8_t  steering;  // -100 (Full Left) to +100 (Full Right)
    uint8_t throttle;  // 0 to 100%
    uint8_t brake;     // 0 = released, 1 = engaged
};
```
1. **Normalization:** $\text{steerNorm} = \frac{\text{cmd.steering}}{100.0}$
2. **Target Position Computation:**
   $$\text{targetPos} = \text{steerNorm} \times \text{MAX\_POS\_LIMIT}$$

### 4.3 Target Clamping Gate
All target commands pass through `enablePID(target)`:
$$\text{targetPosition} = \mathrm{constrain}(\text{newTarget},\ \text{MIN\_POS\_LIMIT},\ \text{MAX\_POS\_LIMIT})$$

---

## 5. Sensor Feedback & Filtering Pipeline

To prevent sensor noise and mechanical vibration from jittering the derivative term of the PID loop, the position reading passes through a dedicated smoothing stage:

### 5.1 Sampling Rate-Limiter
- The position feedback is rate-limited to update at most once every $1,000\ \mu\text{s}$ ($1\text{ kHz}$).
- Consecutive calls within the same millisecond loop iteration return the cached value to prevent excessive filtering overhead.

### 5.2 1st-Order Low-Pass Filter (EMA)
The filtered position $y[k]$ is updated from the raw unwrapped position $x[k]$:
$$y[k] = y[k-1] + \alpha \cdot (x[k] - y[k-1])$$
- Default Filter Alpha ($\alpha$): `0.15`
- Low $\alpha$ ($< 0.05$): Heavy smoothing, higher phase lag.
- High $\alpha$ ($> 0.50$): Responsive, but passes high-frequency mechanical vibration.

---

## 6. Closed-Loop Controller Logic (PID)

The primary control loop is implemented in `updatePID()` and runs at up to $200\text{ Hz}$ ($\Delta t \ge 5\text{ ms}$).

### 6.1 Position Error & Deadband
$$\text{error} = \text{targetPosition} - \text{currentPosition}$$
$$\text{absError} = |\text{error}|$$

- **Backlash Deadband (`deadband = 18,200 counts` $\approx 3.12^\circ$):**
  - Steering linkages, tie rods, and reduction gearboxes have physical mechanical play (backlash).
  - If the motor attempts to correct within the backlash window, it will continuously hunt back and forth and buzz.
  - If $\text{absError} \le \text{deadband}$, the motor output is cut immediately:
    $$\text{PWM} = 0,\quad \text{currentPwm} = 0$$

### 6.2 Proportional Term ($K_p$)
$$P = K_p \times \text{error} \quad (K_p = 0.012)$$
- Directly scales motor command with distance to target.
- A $10^\circ$ error ($\approx 58,254\text{ counts}$) produces a raw proportional term of $\approx 699$.

### 6.3 Integral Term ($K_i$)
$$I = K_i \times \int \text{error}\ dt \quad (K_i = 0.0)$$
- **Why $K_i$ is 0:** In steering systems, tire scrub resistance creates steady-state resistance. An active integrator accumulates windup while fighting ground friction, causing sluggish overshoot, delayed direction reversal, and creeping.

### 6.4 Derivative on Measurement ($K_d$)
Rather than calculating derivative on error ($\frac{d(error)}{dt}$, which causes an explosive "derivative kick" whenever the operator moves the joystick), the derivative is calculated purely on **measured velocity**:
$$\text{velocity} = \frac{\text{currentPos} - \text{lastPos}}{\Delta t}$$
$$D = -K_d \times \text{velocity} \quad (K_d = 0.0018)$$
- Acts as pure viscous damping / electronic shock absorber.
- Opposes rapid rotation to arrest momentum as the wheels approach target position.

### 6.5 Raw Controller Output
$$\text{output} = P + I + D$$

---

## 7. Motor Drive Translation & Actuation

The raw mathematical PID output must be conditioned for the physical motor driver:

### 7.1 Direction Selection & Back-EMF Transition
$$\text{desiredDir} = (\text{output} > 0)\ ?\ \text{DIR\_RIGHT} : \text{DIR\_LEFT}$$
- **Anti-Shock Zeroing:** If `desiredDir != activeDir`, the controller forces $\text{PWM} = 0$ for one cycle before asserting the new direction pin state. This protects the H-bridge MOSFETs from shoot-through and prevents instantaneous gearbox torque reversals.

### 7.2 Friction Compensation (Stiction Boost)
Due to static friction in the steering column, gearbox, and tires, a DC motor will not rotate at low PWM values (e.g., $< 25$). To prevent motor stall and deadzone:
- Minimum starting PWM: `minPWM = 25`
- A transition band ($\text{TRANSITION\_BAND} = 1,000\text{ counts}$) linearly blends the stiction boost to avoid a harsh step when leaving the deadband:
  $$\text{frictionBoost} = \begin{cases} 
  \text{minPWM} \times \frac{\text{absError} - \text{deadband}}{\text{TRANSITION\_BAND}} & \text{if } \text{deadband} < \text{absError} < \text{deadband} + \text{TRANSITION\_BAND} \\
  \text{minPWM} & \text{if } \text{absError} \ge \text{deadband} + \text{TRANSITION\_BAND}
  \end{cases}$$

### 7.3 Target PWM & Slew-Rate Limiter
$$\text{desiredPwm} = \mathrm{constrain}(|\text{output}| + \text{frictionBoost},\ 0,\ \text{maxPWM}) \quad (\text{maxPWM} = 150)$$
To prevent destructive current spikes and mechanical jerk, PWM change per second is rate-limited:
- **Maximum Slew Rate:** $600\text{ PWM units / sec}$
- At $200\text{ Hz}$ ($\Delta t = 5\text{ ms}$), the maximum change per step is:
  $$\Delta \text{PWM}_{\max} = 600 \times 0.005 = 3\text{ PWM counts per cycle}$$
- Full acceleration from $0$ to $150$ PWM takes $\approx 250\text{ ms}$.

---

## 8. Safety Systems & Fault Interlocks

The steering subsystem includes five independent safety watchdogs:

```
+-------------------------------------------------------------------------+
|                         Safety Interlock Matrix                         |
+-------------------------------------------------------------------------+
| Condition                             | Action Taken                    |
+---------------------------------------+---------------------------------+
| Measured position >= +60° and moving R| PWM forced to 0 (Software Limit)|
| Measured position <= -60° and moving L| PWM forced to 0 (Software Limit)|
| Measured position exceeds +/- 75°     | Immediate Shutdown (Runaway)    |
| Commanded PWM > 0 but no motion >100ms| Immediate Shutdown (Stall Fault)|
| RC Transmitter Signal Lost (Failsafe) | Steering PID disabled, PWM = 0  |
| Laptop Watchdog timeout (> 500ms)     | Steering PID disabled, PWM = 0  |
+-------------------------------------------------------------------------+
```

### 8.1 Software Travel Limits
Prevents the motor from forcing the steering linkage past its mechanical tie-rod stops:
- If $\text{currentPos} \ge \text{MAX\_POS\_LIMIT}$ and $\text{output} > 0$: $\text{PWM} \rightarrow 0$.
- If $\text{currentPos} \le \text{MIN\_POS\_LIMIT}$ and $\text{output} < 0$: $\text{PWM} \rightarrow 0$.
- Reverse motion away from the limit is always permitted.

### 8.2 Runaway Protection (Positive Feedback Detection)
If the motor driver polarity is inverted or the sensor direction multiplier is wrong, the system experiences positive feedback (error increases $\rightarrow$ motor drives harder in wrong direction).
- If $|\text{currentPos}| > \text{HARD\_STOP\_LIMIT}$ ($\pm 75^\circ$):
  - Sets `encoderFault = true`.
  - Calls `stopSteeringMotor()` (cuts PWM to 0).
  - Serial alert triggers warning the user of runaway positive feedback.

### 8.3 Motor Stall & Disconnection Watchdog
Protects the DC motor and BTS7960 driver from burnout if the wheels are wedged against a curb or if the sensor fails:
- While driving, if the position does not change by $> 10\text{ counts}$ for $\ge 100\text{ ms}$:
  - Asserts `encoderFault = true`.
  - Disables steering PID and cuts throttle to 0 V.

---

## 9. Calibration & Manual Teleop Commands

Interactive serial calibration is supported via single-character ASCII commands over USB-UART:

| Key | Function | Implementation Details |
| :---: | :--- | :--- |
| **`Z`** | **Zero / Calibrate Dead Center** | Records current physical position as straight-ahead (`deadCenterCount`). Resets filters and clears faults. |
| **`C`** | **Center Steering** | Enables PID targeting position `0` (drives wheels straight ahead). |
| **`I`** | **Invert Direction Multiplier** | Flips `angleRotationDirection` between `+1` and `-1` to correct positive feedback runaways. |
| **`M`** | **Diagnostic Report** | Prints current raw counts, calibrated center, degrees, and magnet health. |
| **`S`** | **Emergency Stop** | Immediately disables PID and forces PWM to 0. |

---

## 10. Analysis of Current Implementation Flaws

If you are rewriting the steering code from scratch, address these architectural weaknesses present in the existing code:

### 1. Cascaded Lag from Compounding Filters & Slew Limiters
- The system currently runs:
  1. Input EMA filter on RC joystick.
  2. EMA low-pass filter on sensor position feedback.
  3. Derivative term ($K_d$) on measurement.
  4. Slew-rate limiter on the output PWM.
- **Problem:** Every filter and slew limiter adds **phase lag** to the feedback loop. In closed-loop position control, phase lag causes delayed reaction, ringing, and persistent oscillation.

### 2. Output Slew Limiting vs. Setpoint Profiling
- **Current Approach:** Slew-rate limiting the *PWM motor output*. When the error is large, the controller wants high PWM, but the slew limiter delays it. When the system arrives at the target, the slew limiter delays cutting PWM, leading to overshoot.
- **Correct Solution:** Slew-rate limit the **target setpoint** (trapezoidal profile) so the target moves smoothly, allowing the PID controller to respond instantaneously to setpoint error without artificial output delay.

### 3. Discontinuous Deadband "Cliff"
- Jumping from $\text{PWM} = 0$ to $\text{PWM} \ge \text{minPWM}$ when exiting the deadband causes mechanical clicking/buzzing at the deadband threshold. A polynomial or smooth cubic spline transition avoids this chatter.

### 4. Scattered Direction Multiplier
- The variable `angleRotationDirection` is applied inside `getRawEncoderCount()`, but manual limits check physical pin reads, and jog functions apply hardcoded pin values.
- In a clean rewrite, define **one unambiguous coordinate frame**:
  - `+Angle` = Right, `-Angle` = Left.
  - Direction logic should only be resolved at the final hardware pin write.

---

## 11. Blueprint for Clean-Sheet Rewrite

When designing the new steering module, structure it around four clean stages:

```
+------------------------------------------------------------------------------+
|                         Clean Rewrite Architecture                           |
+------------------------------------------------------------------------------+
| 1. Profiled Setpoint Generator                                               |
|    - Target angle input (deg)                                                |
|    - Rate limiter on setpoint (deg/s) -> Generates smooth current target     |
+------------------------------------------------------------------------------+
                                       |
                                       v
+------------------------------------------------------------------------------+
| 2. Clean State Observer                                                      |
|    - Read sensor angle                                                       |
|    - Apply calibration offset -> true physical angle (deg)                   |
|    - Lightweight velocity estimator                                          |
+------------------------------------------------------------------------------+
                                       |
                                       v
+------------------------------------------------------------------------------+
| 3. Feedforward + PD Controller                                               |
|    - Smooth deadband window (quadratic or cubic blend)                       |
|    - Proportional tracking on angle error                                    |
|    - Damping on measured velocity                                            |
|    - Output normalized torque request (-1.0 to +1.0)                         |
+------------------------------------------------------------------------------+
                                       |
                                       v
+------------------------------------------------------------------------------+
| 4. Hardware Actuator & Safety Bridge                                         |
|    - Check software limits, stall timers, runaway bounds                     |
|    - Map torque (-1.0 to +1.0) to (DIR_PIN, PWM_DUTY)                        |
|    - Apply stiction offset only when outside smooth deadband                 |
+------------------------------------------------------------------------------+
```
