# Firmware Documentation for Spirob Aktuation

## Overview
This ESP32 firmware implements force-based control for robotic actuators using PID regulation. It supports multiple force sensor types (HX711, ANU78025) and includes safety features with soft limits and alarm states.

## Module Responsibilities

### ForceSensor (Abstract Base Class)
- Defines interface for force sensors
- Methods: begin(), tare(), setScale(), readRaw(), getForce(), isReady(), update()

### ForceSensorHX711
- Implements HX711 load cell amplifier
- Non-blocking polling with moving average filtering
- Calibration via tare() and setScale()

### ForceSensorAnu78025
- Implements Qwiic Scale (NAU7802) over I2C with multiplexer
- Handles TCA9548A multiplexer selection
- Calibration and filtering similar to HX711

### PidController
- Reusable PID controller with anti-windup
- Configurable tunings (Kp, Ki, Kd) and output limits
- Updates based on measurement and time

### DistanceSensorVL6180X
- VL6180X time-of-flight sensor for distance measurement
- Used for limit calibration and diagnostics
- Moving window averaging for stable readings

### MotorController
- Combines servo control with PID regulation
- Modes: MANUAL_POSITION, PID_FORCE, WHEEL
- In PID_FORCE mode, regulates force to setpoint using PID output as speed command

### SafetyManager
- Monitors force and distance limits
- Enters ALARM state on violations, stops motors
- Allows reset when conditions are safe

### ConfigManager
- Stores sensor configurations and PID tunings in NVS flash
- Persists calibration data across reboots

## Calibration Process
1. **Tare**: Send `cal_tare:<actuator>` to zero the sensor
2. **Set Known Weight**: Send `cal_weight:<actuator>:<weight>` (e.g., 5.0 kg)
3. **Place Load**: Apply the known weight to the sensor
4. **Save Calibration**: Send `cal_save:<actuator>` to compute and store scale factor

## Serial Commands
- `mode:<actuator>:<mode>` - Set mode (manual, pid_force, wheel)
- `setpoint:<actuator>:<force>` - Set force setpoint for PID
- `cal_tare:<actuator>` - Tare force sensor
- `cal_weight:<actuator>:<weight>` - Set known weight for calibration
- `cal_save:<actuator>` - Save calibration
- `alarm_reset` - Reset alarm if safe

## Timing
- Sensor updates: Continuous, non-blocking
- PID updates: Every 10ms if sensor data available
- Safety checks: Continuous
- Reporting: Every 100ms (timestamp,force0,distance0,position0,force1,distance1,position1,state)

## Safety Features
- Soft limits on force and distance
- Alarm state stops all motor activity
- Manual reset required to resume operation