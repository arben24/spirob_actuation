# AI Coding Agent Instructions for Spirob Aktuation

## Project Overview
ESP32-based robotics actuation system with force/position control. Integrates dual-actuator servo control with force sensors (HX711/ANU78025), PID regulation, safety limits, and a Python GUI for user control. Designed for LeRobot AI integration.

## Firmware Architecture (main.cpp)

**Dual-motor setup** with mixed sensor types:
- **Motor 0**: Servo ID 1 + HX711 (GPIO 12/13)
- **Motor 1**: Servo ID 2 + NAU78025 via I2C Multiplexer (0x70), channel 2

**Polymorphic sensor array**: `ForceSensor* sensors[2]` accepts both HX711 and ANU78025 implementations

**Initialization sequence** (setup()):
1. Serial/Wire/Serial2 init
2. Per-motor: MotorDriver → ForceSensor (polymorphic) → ForceControlLoop
3. Load static calibration from `#define` constants
4. Print status

**Main loop** (loop()):
1. `sensors[i]->update()` - Non-blocking polling for all sensors
2. `controlLoops[i]->update(dt)` - PID control if running
3. Serial command parsing (CLI)
4. Status output 10Hz

**Key design decisions**:
- No SafetyManager/ConfigManager in main.cpp (simplified production firmware)
- Static calibration via `#define` (no NVS persistence needed)
- Isolated test environments for tuning (`test_one_motor_force_control`, `test_hx711`, `test_anu78025`)

## Critical System Patterns

**Initialization Sequence** (`setup()`):
1. Serial/Wire init, servo serial (Serial2) at 1Mbps
2. Per-actuator: MotorDriver → ForceSensor → DistanceSensor → ForceControlLoop
3. ConfigManager loads PID tunings from NVS; SafetyManager registers all components with limits

**Control Modes** (set via `m <id> <mode>` command):
- `0` = MANUAL_POSITION: Direct position setpoint sent to servo
- `1` = PID_FORCE: Force setpoint; PID output drives servo speed
- `2` = WHEEL: Continuous rotation mode

**Force Sensor Workflow**:
- Calibration: `cal_tare:<id>` → zero reference, `cal_weight:<id>:<kg>` → set known load, `cal_save:<id>` → compute scale factor
- Data: Non-blocking polling; moving-window averaging (10 samples); scale factor applied during `getForce()` call
- Units: Always kg (scale factor is "kg per ADC count")

**Safety System** (`SafetyManager::checkLimits()`):
- Monitors force (soft limit = 1.2 × nominal load) and distance (20-65mm range)
- Enters STATE_ALARM on violation → all motors stop
- `alarm_reset` command clears alarm if limits now safe

## Serial Command API

**Quick Reference** (supported by all environments):
```
set <id> <kg>            # Set force setpoint (id: 0-1, or 'all')
start <id>               # Start force regulation
stop <id>                # Stop regulation (motor stops)
pid <id> <kp> <ki> <kd>  # Live tune PID
tare <id>                # Tare sensor (zero-point calibration)
status                   # Print current status
help                     # Show command help
```

**Examples**:
```
set 0 2.5      # Motor 0: setpoint = 2.5 kg
start all      # Start both motors
pid 1 350 0.8 45  # Motor 1: Kp=350, Ki=0.8, Kd=45
stop all       # Stop both motors
```

**Output** (10Hz):
```
M0: Force=0.123/2.500 kg | Pos= 1234 | Speed=  500 | RUN
M1: Force=1.456/0.000 kg | Pos= 5678 | Speed=    0 | STOP
```

## Hardware Integration
- **Force Sensors**: HX711 (pins 12-15 on actuator 0; 14-15 on actuator 1) OR ANU78025 (I2C 0x2A on multiplexer)
- **Distance Sensors**: VL6180X on TCA9548A multiplexer (channel 0/1 per actuator)
- **Servo Comms**: Pins 16/17 @ 1Mbps (Serial2); expects servo driver responding to position/speed commands

## Development Patterns

**Sensor Abstraction** - `ForceSensor` base class:
- Both HX711 and ANU78025 inherit from abstract `ForceSensor` (see [src/ForceSensor.h](src/ForceSensor.h))
- Unified interface: `update()`, `getForce()`, `getWeight()`, `isReady()`, `tare()`, `setCalibration()`
- Enables polymorphic arrays: `ForceSensor* sensors[2] = {new ForceSensorHX711(...), new ForceSensorAnu78025(...)}`
- Both implementations use identical filtering (moving-window average) and unit conventions (kg, N)

**Non-blocking Architecture**: All sensors use `update()` calls in main loop; state returned via getter methods (`getForce()`, `getDistance()`)

**Sensor Data Flow**:
1. `sensor->update()` - Non-blocking poll; if data available, read raw → compute mass → apply filter
2. `sensor->getWeight()` - Returns filtered mass in kg (moving-window average, 10 samples)
3. `sensor->getForce()` - Returns F = mass_kg × 9.81 (in Newtons)
4. Filtering ensures stable setpoint and PID convergence

**PID with Anti-windup** (`PidController::update()`):
- Integral clamped to prevent accumulation; output range limited; anti-windup on initialization

**Modular Test Builds** (`platformio.ini`):
- `test_hx711`, `test_anu78025`, `test_vl6180x`, `test_motor`, `test_config` use `build_src_filter` for isolated testing
- `test_one_motor_force_control`: Single force control loop (Motor ID 2 + NAU78025) for tuning

## Build & Debug Workflow

| Task | Command |
|------|---------|
| Build firmware | `pio run -e production` |
| Upload & monitor | `pio run -e production -t upload && pio device monitor` |
| Test single motor+sensor | `pio run -e test_one_motor_force_control -t upload` |
| Test HX711 sensor | `pio run -e test_hx711 -t upload` |
| Test ANU78025 sensor | `pio run -e test_anu78025 -t upload` |

**PlatformIO Environments**: 
- `production`: Main firmware (dual motor, HX711 + ANU78025)
- `test_one_motor_force_control`: Isolated Motor 2 + NAU78025 tuning
- `test_hx711`, `test_anu78025`, etc.: Individual sensor testing

## Key Files by Function
- `src/main.cpp`: System orchestration, loop cycle, command parsing
- `src/ForceControlLoop.h/cpp`: Mode-switching logic, PID integration
- `src/SafetyManager.h/cpp`: Limit checking, alarm state machine
- `src/ConfigManager.h/cpp`: NVS persistence (PID, calibration)
- `main.py`: GUI event handlers, serial reader thread</content>
<parameter name="filePath">/home/student/Documents/PlatformIO/Projects/Spirob_aktuation/.github/copilot-instructions.md