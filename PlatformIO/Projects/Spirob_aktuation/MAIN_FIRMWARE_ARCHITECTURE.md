# Main Firmware Architecture (src/main.cpp)

## Overview

The new `main.cpp` is a clean, dual-motor force control firmware designed after the successful `test_one_motor_force_control.cpp` pattern. It manages two independent actuators with different sensor types using polymorphic abstraction.

## Hardware Configuration

### Motor 0
- **Servo ID**: 1
- **Sensor Type**: HX711 (load cell amplifier)
- **Pins**: GPIO 12 (DOUT), GPIO 13 (SCK)
- **Calibration**: `FORCE_OFFSET_0`, `FORCE_SCALE_0`

### Motor 1
- **Servo ID**: 2
- **Sensor Type**: NAU78025 (Qwiic Scale via I2C)
- **I2C Address**: 0x2A
- **Multiplexer Channel**: 2 (on TCA9548A @ 0x70)
- **Calibration**: `FORCE_OFFSET_1`, `FORCE_SCALE_1`

### Common
- **Servo UART**: Serial2 (pins 16/17) @ 1Mbps
- **Debug Serial**: Serial @ 115200 baud
- **I2C**: Wire (standard pins for ESP32)

## Firmware Structure

### 1. Initialization (setup())

```
Wire.begin()
Serial.begin(115200)
Serial2.begin(1Mbps)
  ↓
for each motor:
  - Create MotorDriver
  - Create ForceSensor (HX711 or ANU78025)
  - Load calibration constants
  - Create ForceControlLoop
  - Configure PID tunings
  ↓
Print status
```

### 2. Main Loop (loop())

```
Every 5ms (200Hz loop rate):
  1. sensors[i]->update()           # Non-blocking polling
  2. if running: controlLoops[i]->update(dt)  # PID control
  3. Process serial commands
  4. Every 100ms: Print status (10Hz)
```

### 3. Command Processing

**Format**: `<command> <id> [params...]`

**Commands**:
- `set <id> <kg>` - Set force setpoint
- `start <id>` - Begin force regulation
- `stop <id>` - Stop regulation
- `pid <id> <kp> <ki> <kd>` - Tune PID coefficients
- `tare <id>` - Zero the force sensor
- `status` - Print current state
- `help` - Show command list

**Special**: Use `id = "all"` to broadcast to both motors

### 4. Control Modes

- **MODE_MANUAL_POSITION**: Direct servo position (not used in main.cpp)
- **MODE_PID_FORCE**: Force regulation (default, PID drives motor speed)
- **MODE_SPEED_CONTROL**: Direct speed command (available but rarely used)

## Design Decisions

### Why This Architecture?

1. **Polymorphic Sensors**: `ForceSensor* sensors[2]` allows mixing HX711 and ANU78025 without code duplication
2. **Static Calibration**: No NVS storage complexity; calibration values hardcoded via `#define`
3. **No Safety Manager**: Simplified firmware focuses on robust control; safety features are optional
4. **Test-Driven Pattern**: Mirrors successful `test_one_motor_force_control.cpp` design
5. **Non-blocking Updates**: All sensors use polling; no blocking I2C/SPI calls in main loop

### What's NOT Included

- ❌ SafetyManager (soft limits, alarm states)
- ❌ ConfigManager (NVS persistence)
- ❌ Distance sensors (VL6180X)
- ❌ Python GUI integration
- ❌ Runtime calibration

These are available in isolated test environments if needed.

## Calibration

### Static Calibration Method

1. **Tare** (zero-point):
   ```
   Sensor raw value at zero load = OFFSET
   ```

2. **Known weight** (scale factor):
   ```
   Sensor raw value with known mass = RAW_LOADED
   SCALE = (RAW_LOADED - OFFSET) / mass_kg
   ```

3. **Update constants**:
   ```cpp
   #define FORCE_OFFSET_0  <offset>
   #define FORCE_SCALE_0   <scale>
   ```

4. **Use test environments** for calibration:
   - `test_hx711` for Motor 0
   - `test_anu78025` for Motor 1

### Online Tare

Even with static calibration, you can re-tare at runtime:
```
> tare 0    # Tare Motor 0 sensor
> tare all  # Tare both sensors
```

## Status Output Format

Every 100ms (10Hz):
```
M0: Force=0.123/2.500 kg | Pos= 1234 | Speed=  500 | RUN
M1: Force=1.456/0.000 kg | Pos= 5678 | Speed=    0 | STOP
```

Fields:
- `Force=actual/setpoint` - Current vs. target force in kg
- `Pos` - Motor position (raw servo value)
- `Speed` - Motor speed (raw servo value)
- `RUN/STOP` - Regulation active or idle

## PID Tuning

Default values (configured in setup()):
```cpp
Kp = 300.0    // Proportional gain
Ki = 0.5      // Integral gain
Kd = 50.0     // Derivative gain
```

Live tuning via serial:
```
> pid 0 350 0.8 45    # Motor 0: Kp=350, Ki=0.8, Kd=45
> pid all 300 0.5 50  # Both motors: standard tuning
```

## Key Files

- [src/main.cpp](src/main.cpp) - Dual-motor production firmware
- [src/test_one_motor_force_control.cpp](src/test_one_motor_force_control.cpp) - Single motor tuning
- [src/test_hx711.cpp](src/test_hx711.cpp) - HX711 sensor calibration
- [src/test_anu78025.cpp](src/test_anu78025.cpp) - ANU78025 sensor calibration
- [src/ForceControlLoop.h/cpp](src/ForceControlLoop.h) - PID + mode management
- [src/ForceSensor.h](src/ForceSensor.h) - Polymorphic sensor interface

## Typical Workflow

1. **Calibrate sensors** (offline, once):
   ```bash
   pio run -e test_hx711 -t upload
   # Calibrate Motor 0, record OFFSET_0 and SCALE_0
   
   pio run -e test_anu78025 -t upload
   # Calibrate Motor 1, record OFFSET_1 and SCALE_1
   ```

2. **Update calibration constants** in `main.cpp`:
   ```cpp
   #define FORCE_OFFSET_0  270331L
   #define FORCE_SCALE_0   225.64403f
   #define FORCE_OFFSET_1  520000L
   #define FORCE_SCALE_1   423.11273f
   ```

3. **Tune PID** (if needed):
   ```bash
   pio run -e production -t upload
   # Open serial monitor at 115200 baud
   > set 0 2.5        # Request 2.5 kg on Motor 0
   > start 0
   > pid 0 320 0.6 45 # Adjust tuning
   ```

4. **Deploy**:
   ```bash
   pio run -e production -t upload
   ```

## Troubleshooting

### Motor Not Responding
- Check servo ID (1 or 2)
- Verify Serial2 pins (16/17) and baud (1Mbps)
- Check power supply to servo

### Sensor Not Responding
- **HX711**: Check GPIO pins (12/13), power
- **ANU78025**: Check I2C wiring, multiplexer address (0x70), channel (0-7)

### Force Readings Wrong
- Verify calibration: Is `(raw - offset) / scale ≈ actual_mass`?
- Check sensor is level and load is centered
- Re-tare: `> tare 0` or `> tare 1`

### PID Oscillation
- Reduce Kp (proportional gain)
- Increase Kd (derivative gain) for damping
- Increase Ki slowly if steady-state error appears
