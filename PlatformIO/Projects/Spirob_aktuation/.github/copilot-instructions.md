# AI Coding Agent Instructions for Spirob Aktuation

## Project Overview
This is a robotics actuation system combining AI-powered control (via LeRobot) with custom embedded hardware. The project integrates:
- **Firmware (C++):** ESP32-based servo control with force sensors (HX711/ANU78025) using PID feedback, modular architecture with safety limits
- **Control Software (Python):** Tkinter GUI for manual/PID mode switching and setpoint control

## Architecture
- **Embedded Layer:** `src/main.cpp` orchestrates modular components: MotorController, ForceSensor, SafetyManager, ConfigManager
- **Communication:** Serial protocol between Python GUI (`main.py`) and ESP32 firmware

## Key Patterns & Conventions
- **Servo Control Modes:** MANUAL_POSITION (direct position), PID_FORCE (force regulation), WHEEL (continuous rotation)
- **Sensor Integration:** Force sensors with tare/calibration, moving-window averaging (10 samples), non-blocking updates
- **PID Tuning:** Kp=300, Ki=0.5, Kd=50 defaults; anti-windup, output limits; updates every 10ms when sensor ready
- **Force Units:** Configurable nominal loads (e.g., 5kg, 20kg); scale factors computed during calibration
- **Safety Limits:** Soft limits on force (1.2x nominal) and distance (20-65mm); alarm state halts motors
- **Calibration Workflow:** Tare → set known weight → save scale factor; stored in NVS flash
- **Output Format:** CSV lines like "ts,force0,distance0,pos0,force1,distance1,pos1,state"

## Build & Development Workflow
- **Firmware:** Use PlatformIO (`pio run -e esp32dev`) with Arduino framework; upload via `pio run -e esp32dev -t upload`
- **Python Deps:** Use `uv` for dependency management (`uv sync`); activate venv with `source .venv/bin/activate`
- **Testing:** Connect ESP32 via USB; run `python main.py` for GUI control; monitor serial at 115200 baud
- **Debugging:** Firmware logs force/distance/position; use serial commands for calibration and mode switching

## Integration Points
- **Hardware Wiring:** UART TX/RX (17/16) for servos, I2C for sensors/multiplexer; HX711 uses GPIO pins
- **Sensor Multiplexing:** TCA9548A for multiple I2C devices (VL6180X, ANU78025)
- **CLI Commands:** Firmware accepts commands like "mode:0:pid_force", "setpoint:0:5.0", "cal_tare:0", "alarm_reset"

## Code Examples
- **PID Update:** See `PidController::update()` for anti-windup and clamping
- **Sensor Averaging:** Circular buffer in `ForceSensorHX711::update()`
- **Calibration:** State machine in `processSerialCommand()` for tare/weight/save
- **Safety Check:** `SafetyManager::checkLimits()` monitors and halts on violations

Reference: `platformio.ini` for lib_deps (HX711, SparkFun_Qwiic_Scale), `src/FIRMWARE_README.md` for module details.</content>
<parameter name="filePath">/home/student/Documents/PlatformIO/Projects/Spirob_aktuation/.github/copilot-instructions.md