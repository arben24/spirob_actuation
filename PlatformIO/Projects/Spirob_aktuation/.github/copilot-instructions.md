# AI Coding Agent Instructions for Spirob Aktuation

## Big Picture
- ESP32 firmware + Python tooling for a dual-motor actuator with force control and telemetry.
- Firmware layers: `ForceSensor` (HX711/NAU7802), `MotorDriver` (SC servo UART@1Mbps), `ForceControlLoop`, `EndStopSwitch` safety (see [src](src)).
- Entry points: production loop in [src/main.cpp](src/main.cpp) and system-identification loop in [src/main_SystemIdentification.cpp](src/main_SystemIdentification.cpp).

## Critical Conventions
- Motor indices are 0/1 in CLI & Python; servo IDs are 1/2. `MotorDriver` takes servo ID, not index.
- `ForceControlLoop::ControlMode` is **not** `MotorDriver::DriverMode`.
- `ForceSensor::update()` is non-blocking; call `update()` then `getForce()` in the same loop for fresh cached values.
- `ForceSensor` stores kg internally; `getForce()` returns Newtons. Calibration is compile-time defines in sensor files.
- `EndStopSwitch::isTriggered()` is debounced logic; `isRawTriggered()` is immediate for blocking loops (e.g., homing).

## Build & Run (PlatformIO)
- Environments isolate sources via `build_src_filter`; add new sources to the relevant env in [platformio.ini](platformio.ini).
- Key envs: `production` (115200), `one_motor_system_id` (binary telemetry, 460800), component tests (`test_*`).

## Protocols & Telemetry
- ASCII CLI commands differ between production and system ID; see [src/main.cpp](src/main.cpp) and [src/main_SystemIdentification.cpp](src/main_SystemIdentification.cpp).
- Binary status packets (600Hz) and step-end packets are specified in [COMMUNICATION_PROTOCOL.md](COMMUNICATION_PROTOCOL.md).

## Python Tooling & Data
- Scripts: [system_identification.py](system_identification.py), [step_response_automation.py](step_response_automation.py), [live_monitor_simple.py](live_monitor_simple.py), [main.py](main.py), [spirob_digital_twin.py](spirob_digital_twin.py).
- Dependencies live in [pyproject.toml](pyproject.toml) (includes `pyserial`, `polars`, `pyqtgraph`, `mujoco`).
- Dual-motor Parquet schema: `timestamp_s`, `timestamp_us`, `force0_N`, `force1_N`, `rope0_mm`, `rope1_mm`.

## Known Pitfalls
- `analyze_step_response.py` still assumes a single-motor schema.
- Blocking `step` tests should use `isRawTriggered()` for emergency stop.
