# AI Coding Agent Instructions for Spirob Aktuation

## Project Overview
ESP32-based dual-motor servo control system with real-time force regulation via PID loops. Features polymorphic sensor abstraction (HX711 + NAU7802 load cells), ServoSerial UART communication, CLI command parsing, and CustomTkinter GUI for remote operation. Architecture emphasizes non-blocking main loop design and compile-time calibration via `#define` constants.

## Critical Architecture Decisions

**Index vs ID Confusion (MUST understand):**
- Code uses array indices `[0]`, `[1]` everywhere BUT serial protocol uses Servo IDs `1`, `2`
- Index 0 → Servo ID 1 (HX711) | Index 1 → Servo ID 2 (NAU7802)
- All CLI commands and internal calls use indices; ServoSerial protocol uses IDs
- MotorDriver constructor takes Servo ID, but stored in `motors[array_index]`

**Hardware Wiring:**
- **Motor[0]**: Servo ID 1 + HX711 (GPIO 26=data, 27=clock) 
- **Motor[1]**: Servo ID 2 + NAU7802 (I2C 0x2A, mux channel 2)
- **Servo UART**: Serial2 @ 1Mbps (pins 16/17)
- **Debug UART**: 115200 baud (production) or 460800 (system ID fw)

**Design Philosophy:**
- **Calibration**: Static `#define` offsets + scale factors in `main.cpp` (compiled once, not runtime NVS)
- **Sensor Abstraction**: Polymorphic `ForceSensor* sensors[2]` (unified interface for HX711 + NAU7802)
- **Non-blocking**: All sensor `update()` calls return immediately; data via getters (`getForce()`, `getWeight()`)
- **State Decoupling**: `isRunning[i]` flag independent from control loop state; motor won't auto-stop

## Main Loop Execution (loop())

**Tight update cycle (lines 196-220 in main.cpp):**
1. `sensors[i]->update()` - Non-blocking poll; if new ADC data, applies scale & 10-sample moving-window filter
2. If `isRunning[i] == true`: `controlLoops[i]->update()` returns speed → `motors[i]->setSpeed(speed)`
3. Serial parse (blocking on `Serial.available()` + `readStringUntil('\n')`)
4. Status print throttled at 10Hz via `lastStatusPrint` + `STATUS_INTERVAL = 1000ms`

**State Tracking:**
- `isRunning[i]` is independent ON/OFF flag (set by `start`/`stop` CLI commands)
- Control loop only executes when `isRunning[i] == true`; motor WON'T auto-stop after regulation ends
- Force setpoint updates via `set` command work regardless of `isRunning` state

**Sensor Data Flow** (both HX711 + NAU7802):
- Raw ADC → apply `FORCE_OFFSET_X`, `FORCE_SCALE_X` (loaded at compile) → moving-avg filter → cache in class
- `getWeight()` returns mass_kg; `getForce()` returns force_N = mass_kg × 9.81
- Filter prevents PID oscillation; 10-sample window = ~100ms of history

**Safety (test builds only):**
- `SafetyManager::checkLimits()` monitors force + distance, triggers STATE_ALARM → motors stop
- Production main.cpp does NOT include SafetyManager (simplification)

## Serial Command API

**Implemented Commands** (from [src/main.cpp](src/main.cpp#L193-L395)):
```
set <id> <N>             # Force setpoint in Newtons (id: 0-1, or 'all')
start <id>               # Begin force regulation; sets motor MODE_WHEEL
stop <id>                # Stop regulation; calls motor->stop()
pid <id> <kp> <ki> <kd>  # Live-tune PID gains (no persistence; resets on restart)
fast_print               # Enable 10Hz continuous status streaming
status                   # Print single status snapshot
help                     # Show available commands
```

**Notes:**
- `<id>` is 0-1 (array index), NOT servo ID; `all` broadcasts to both motors
- Force setpoint changes apply immediately (doesn't require `start` to take effect)
- PID tunings are RAM-only; flash with tuned values after testing
- Status output format: `M0: Force= X.XXX / Y.YYY N | Pos=  XXXXX | Speed=   XXXX | RUN/STOP`

**Examples:**
```
set 0 24.5          # Motor 0: setpoint = 24.5 N (≈ 2.5 kg)
start all           # Start both motors
pid 1 350 0.8 45    # Motor 1: Kp=350, Ki=0.8, Kd=45 (tuning)
stop all            # Emergency stop both
```

## Development Patterns

**Sensor Abstraction** - `ForceSensor` base class:
- Both HX711 and NAU7802 inherit from abstract `ForceSensor` (see [src/ForceSensor.h](src/ForceSensor.h))
- Unified interface: `update()`, `getForce()`, `getWeight()`, `isReady()`, `tare()`, `setCalibration()`
- Enables polymorphic arrays: `ForceSensor* sensors[2] = {new ForceSensorHX711(...), new ForceSensorAnu78025(...)}`
- Both implementations use identical filtering (10-sample moving-window average) and unit conventions (kg → N)
- Production build includes BOTH sensor implementations; choose which motor uses which at runtime

**Non-blocking Architecture**:
- All sensors use `update()` calls in main loop; state returned via getter methods
- Motor polling via `getPosition()`, `getSpeed()` (no blocking I/O)
- Serial read is the only blocking operation (uses `readStringUntil('\n')`)

**PID with Anti-windup** ([src/PidController.h](src/PidController.h)):
- Integral term clamped; output limited to `[-maxSpeed, +maxSpeed]`
- Anti-windup engaged on initialization to prevent integrator saturation

**MotorDriver Communications**:
- Uses SC Servo serial protocol @ 1Mbps (ServoSerial library)
- Position is 0-4095 (wheel rotation) or 0-1023 (servo mode)
- Commands: `setSpeed()`, `setPosition()`, `getPosition()`, `getSpeed()`, `stop()`
- Mode switching: `setMode(MODE_WHEEL)` for force control, `MODE_SERVO_POSITION` for position hold

## Build & Debug Workflow

| Task | Command |
|------|---------|
| Build firmware | `pio run -e production` |
| Upload & monitor | `pio run -e production -t upload && pio device monitor` |
| Test single motor+sensor | `pio run -e test_one_motor_force_control -t upload` |
| Test HX711 sensor | `pio run -e test_hx711 -t upload` |
| Test NAU7802 sensor | `pio run -e test_anu78025 -t upload` |
| System identification | `pio run -e one_motor_system_id -t upload` → `uv run system_identification.py` |
| Launch Python GUI | `python3 main.py` (or `uv run main.py`) |

**PlatformIO Environments** (from [platformio.ini](platformio.ini)): 
- `production`: Main firmware (dual motor, HX711 + NAU7802) - **DEFAULT**
- `one_motor_system_id`: Step response data collection @ 460800 baud (binary protocol)
- `test_one_motor_force_control`: Isolated Motor 2 + NAU7802 tuning
- `test_hx711`, `test_anu78025`, `test_vl6180x`, `test_motor`, `test_config`: Individual component testing

**System Identification Workflow** (for PID tuning):
1. Flash `one_motor_system_id` firmware (uses binary protocol @ 460800 baud)
2. Run `system_identification.py` to collect step response data
3. Script applies step input, records force/position, analyzes response
4. Exports data as Polars DataFrame + CSV for analysis
5. Results guide PID coefficient selection for ForceControlLoop

## Key Files by Function
- [src/main.cpp](src/main.cpp): System orchestration, loop cycle, command parsing (lines 1-416)
- [src/main_SystemIdentification.cpp](src/main_SystemIdentification.cpp): Binary telemetry for step response data collection
- [src/ForceControlLoop.h/cpp](src/ForceControlLoop.h): Mode-switching logic, PID integration
- [src/ForceSensor.h](src/ForceSensor.h): Abstract base class (polymorphic sensor interface)
- [src/ForceSensorHX711.h/cpp](src/ForceSensorHX711.h): HX711 load cell implementation
- [src/ForceSensorAnu78025.h/cpp](src/ForceSensorAnu78025.h): NAU7802 Qwiic scale implementation
- [src/PidController.h/cpp](src/PidController.h): Anti-windup PID control
- [src/MotorDriver.h/cpp](src/MotorDriver.h): Servo communication via Serial2
- [src/SafetyManager.h/cpp](src/SafetyManager.h): Limit checking, alarm state machine (used in test builds)
- [src/ConfigManager.h/cpp](src/ConfigManager.h): NVS persistence (not used in production main.cpp)
- [main.py](main.py): CustomTkinter GUI for command dispatch + real-time status display
- [system_identification.py](system_identification.py): Step response data collector with binary protocol parsing
- [step_response_automation.py](step_response_automation.py): Parametric test suite framework (TestSuite, TestPoint, StepResponseAutomation runner)

## Python Development (main.py, system_identification.py & step_response_automation.py)

**Dependencies** (managed via `uv` + `pyproject.toml`):
- `pyserial>=3.5` - Serial communication with ESP32
- `customtkinter>=5.0` - Modern GUI framework
- `polars>=1.38.1` - DataFrame operations (system ID analysis)
- `matplotlib>=3.10.8` - Plotting step response data

**main.py - CustomTkinter GUI**:
- `SerialReaderThread`: Non-blocking serial read in separate thread
- `CommandQueue`: Thread-safe command dispatch (rate limited: 1 cmd per 50ms)
- UI: Force sliders (0-50N), start/stop buttons, emergency stop, real-time status display
- Status parsing from firmware (M0/M1 format): Force (actual/setpoint), position, speed, run state
- Launch: `python3 main.py` or `uv run main.py`

**system_identification.py - Binary Protocol Parser**:
- Connects @ 460800 baud to `one_motor_system_id` firmware
- Syncs to header `0xAA 0x55`, reads packed status structs (12 bytes each)
- Applies step inputs via serial commands (e.g., `step 500 2000` = speed 500 for 2000ms)
- Exports response data as Polars DataFrame + CSV for PID tuning analysis

**step_response_automation.py - Parametric Test Framework** (NEW):
- `StepResponseAutomation` class orchestrates multi-test campaigns with automated reset between runs
- Helper functions: `create_speed_sweep()`, `create_force_sweep()`, `create_parametric_grid()` for test suite generation
- `TestSuite` & `TestPoint` dataclasses define test configurations
- Reads binary protocol @ 460800 baud; generates Parquet files + PNG plots for each test
- Key workflow: `automation.connect()` → `automation.run_test_suite(suite)` → auto-saves results with timestamps
- Automatic `reset_to_zero()` between tests monitors cable length until < 1mm (max 30s timeout)
- Example usage in `__main__`: 15-point speed sweep (100-1500) with 100s duration + 10N max force

**Key Features**:
- Emergency stop broadcasts `stop all` command
- Real-time feedback panel shows all ESP32 responses + sent commands
- Thread-safe with proper signal handling

## Common Pitfalls & Conventions

**Index/ID Confusion (frequent bug source):**
- CLI & internal code use **array indices** (0-1) for motor selection
- ServoSerial protocol uses **Servo IDs** (1-2) in packet headers
- When adding new commands, always validate: which context am I in?
- Example: `set 0 25.0` = Motor at array index 0 = Servo ID 1

**Calibration Workflow:**
- All sensor calibration is **compile-time only** (no NVS runtime configuration in production)
- To change `FORCE_OFFSET_0` or `FORCE_SCALE_0`, edit `main.cpp` lines 16-24, rebuild, and flash
- Never assume ConfigManager or NVS persistence; production main.cpp doesn't use it

**Force Setpoint Semantics:**
- `set <id> <N>` updates `forceSetpoint[id]` and takes immediate effect in next control loop iteration
- Motor doesn't need to be running (`isRunning[id] == true`) for setpoint to apply
- `start`/`stop` commands only control whether `controlLoops[i]->update()` executes

**Thread Safety in main.py:**
- Use `CommandQueue` for all serial commands; never call `serial_port.write()` directly
- `SerialReaderThread` runs in separate thread; all queue operations are thread-safe
- GUI updates must happen in main thread; use queue callbacks if needed

**Motor Mode Semantics:**
- `MODE_WHEEL`: Continuous rotation for force control (via PID speed regulation)
- `MODE_SERVO_POSITION`: Fixed position hold (not typically used in force control)
- `setMode()` only affects motor driver behavior; doesn't affect `ForceControlLoop` enum

**Test Environment Selection:**
- Use `build_src_filter` to isolate components; don't manually comment out code
- `test_*` environments are mutually exclusive (each has its own main)
- System identification environment uses different serial protocol (binary @ 460800 baud)</content>
<parameter name="filePath">/home/student/Documents/PlatformIO/Projects/Spirob_aktuation/.github/copilot-instructions.md