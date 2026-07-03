# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ESP32 firmware + Python tooling for a dual-motor rope-driven actuator (SpiRob) with force control, binary telemetry, and a MuJoCo digital twin. Full architecture doc (German): [README.md](README.md). Protocol spec: [COMMUNICATION_PROTOCOL.md](COMMUNICATION_PROTOCOL.md).

## Build & Flash (PlatformIO)

`platformio.ini` uses `build_src_filter` per environment to compile only the sources that environment needs — **when adding a new `.cpp` file, you must add it to every relevant environment's filter or it silently won't build/link.**

```bash
# Compile only (no upload)
pio run -e production
pio run -e one_motor_system_id

# Flash + open monitor
pio run -e production -t upload && pio device monitor -b 115200
pio run -e one_motor_system_id -t upload && pio device monitor -b 460800
```

Key environments (all `espressif32`/`esp32dev`/`arduino`):
| Env | Source | Baud | Purpose |
|---|---|---|---|
| `production` (default) | `main.cpp` | 115200 | Dual-motor PID force control, ASCII CLI |
| `one_motor_system_id` | `main_SystemIdentification.cpp` | 460800 | System ID, homing, 600 Hz binary telemetry, step-response |
| `object_force` | `main_object_force.cpp` | 115200 | Force sensing only, no motor |
| `test_hx711`, `test_anu78025`, `test_vl6180x`, `test_motor`, `test_config`, `test_one_motor_force_control` | `test_*.cpp` | 115200 | Isolated component tests |

Python side uses `uv`:
```bash
uv run <script>.py       # e.g. uv run system_identification.py
```
Deps declared in `pyproject.toml` (pyserial, polars, matplotlib, scipy, pyqt6/pyqtgraph, mujoco, customtkinter).

## Architecture

Two independent, mutually-exclusive firmware entry points, selected via PlatformIO environment — not a runtime switch:

```
main.cpp / main_SystemIdentification.cpp   (pick one per build)
├── ForceControlLoop      PID or direct-speed force regulation
│   ├── IMotorDriver       abstract motor interface (setMode/setPosition/setSpeed/stop/...)
│   │   ├── MotorDriver           SC-Servo UART protocol (Feetech, servo ID 1 or 2) — test_motor, test_one_motor_force_control, test_config only
│   │   └── MotorDriverRobStride  RobStride/CyberGear over CAN (MIT/CTRL_MODE frame) — production, one_motor_system_id
│   ├── ForceSensor       abstract base; impls: ForceSensorHX711, ForceSensorAnu78025 (NAU7802 over I2C mux)
│   └── PidController     generic PID with anti-windup
└── EndStopSwitch         debounced limit switch (safety)
```

As of 2026-07, `production` and `one_motor_system_id` drive RobStride/CyberGear CAN motors via `MotorDriverRobStride` (wraps the vendored `TWAI_CAN_MI_Motor.h/.cpp` low-level driver); `test_motor`, `test_one_motor_force_control`, and `test_config` still target the original Feetech SC-servos via `MotorDriver` and are unaffected. Both driver classes implement `IMotorDriver` so `ForceControlLoop` doesn't care which one it's driving.

`MotorDriverRobStride` keeps the motor permanently in `CTRL_MODE` (MIT frame `Motor_ControlMode(torque, position, speed, kp, kd)`); the force-PID's output is sent as a **speed target tracked via the frame's `kd` damping term** (`kp=0`, `torque=0`) — the same PID→speed cascade the Feetech WHEEL mode used, just carried over CAN. Its `setPosition()`/`setSpeed()` take `int16_t` in **centi-rad / centi-rad-s** (value/100 = physical units) to stay a drop-in match for `IMotorDriver`; `getPositionRad()` (not part of the interface) returns full-precision float radians for anything that integrates position over time (e.g. rope-length tracking). `poll()` must be called once per loop iteration (and inside any blocking loop like homing/step-response) — the motor only replies to received frames, so this is what keeps cached telemetry fresh; it is the CAN analogue of `ForceSensor::update()`.

`ConfigManager`, `SafetyManager`, and `MotorController` exist in `src/` (and `src/FIRMWARE_README.md` documents them as if they were the live architecture) but **are not wired into any current build environment** except `ConfigManager` in `test_config`. Treat `src/FIRMWARE_README.md` as describing a different/earlier design, not the current one — trust `platformio.ini` build filters and `main.cpp`/`main_SystemIdentification.cpp` over it.

### Critical conventions
- **Motor index vs. servo/CAN ID**: Python/CLI use index `0`/`1`; the driver constructors take servo/CAN ID `1`/`2`, not the index. Motor 0 → ID 1 → mux channel 0 → endstop GPIO 26 (direction inverted); Motor 1 → ID 2 → mux channel 1 → endstop GPIO 27.
- `ForceControlLoop::ControlMode` (PID/Speed/Manual) and `IMotorDriver::DriverMode` (`MODE_SERVO_POSITION`/`MODE_WHEEL`) are different enums — don't cross-cast.
- `ForceSensor` stores mass internally in kg; `getForce()` returns Newtons. `update()` is non-blocking — call `update()` then `getForce()` within the same loop iteration to get a fresh cached value.
- `EndStopSwitch::isRawTriggered()` is immediate (use in blocking loops like homing/step-response for e-stop); `isTriggered()` is debounced (~5 ms) for normal loop use.
- Force sensor calibration (`FORCE_OFFSET_n`, `FORCE_SCALE_n`) is compile-time `#define`s at the top of each `main_*.cpp` — no NVS/EEPROM persistence. `FORCE_SCALE_1`/`FORCE_OFFSET_1` in `main.cpp` are placeholders and must be recalibrated after any hardware change. Same status applies to the new RobStride-era placeholders in both `main_*.cpp` (`DEFAULT_KP/KI/KD`, `MOTOR_HOLD_KP/KD`, `MOTOR_SPEED_KD`, `DEFAULT_MAX_SPEED`, `HOMING_SPEED`) — not yet field-tuned.
- Rope drum, `production`: not tracked (no rope-length output in this env). Rope drum, `one_motor_system_id`: Ø 44 mm drum → `MM_PER_RAD = 22 mm/rad` (arc length = radius × angle); `calculateRopeLength()` unwraps `MotorDriverRobStride::getPositionRad()`'s ±4π wrap the same way the old code unwrapped Feetech's 4096-steps/rev wrap.
- ASCII CLI syntax differs between `production` (`set`, `start`, `stop`, `pid`, `status`, `fast_print`) and `one_motor_system_id` (`f`, `start`, `pid`, `v`, `step`, `stop`, `c`, `r`, `n`, `fl`, `help`) — check the right main file before assuming a command exists.

### Telemetry (ESP32 → Python, binary, ~600 Hz)
- Status packet (22 bytes): `0xAA 0x55 | uint32 ts_us | float force0_N | float force1_N | float rope0_mm | float rope1_mm`
- Step-end packet (3 bytes): `0xBB 0x66 | uint8 motor_index`
- Full spec in [COMMUNICATION_PROTOCOL.md](COMMUNICATION_PROTOCOL.md).

### Python tooling
- Dual-motor Parquet schema (used by `system_identification.py`, `step_response_automation.py`): `timestamp_s`, `timestamp_us`, `force0_N`, `force1_N`, `rope0_mm`, `rope1_mm`.
- `analyze_step_response.py` still assumes the old single-motor schema — don't assume it works against current dual-motor Parquet files without checking.
- `spirob_digital_twin.py` opens the MuJoCo viewer on `spiral_chain.xml`; moving actuator sliders sends `f <id> <N>` force setpoints to the real hardware over serial (`PORT`, `BAUDRATE=460800`, `SEND_HZ=50`, `FORCE_DEADBAND=0.05N` configured at top of script). Closing the viewer sends `stop`.
