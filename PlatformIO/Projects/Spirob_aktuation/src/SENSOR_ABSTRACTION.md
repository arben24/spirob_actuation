# Force Sensor Abstraction Layer

## Overview

This directory contains an abstraction layer for force sensor implementations, allowing the firmware to work with multiple sensor types (HX711, ANU78025/NAU7802) through a unified interface.

## Architecture

### Base Class: `ForceSensor` (src/ForceSensor.h)

Abstract interface defining the contract for all force sensor implementations:

```cpp
class ForceSensor {
public:
    virtual bool begin() = 0;
    virtual void update() = 0;           // Non-blocking polling
    virtual void tare() = 0;
    virtual void setCalibration(long offset, float scale) = 0;
    virtual void setOffset(long offset) = 0;
    virtual void setScale(float scale) = 0;
    virtual bool isReady() = 0;
    virtual long readRaw() = 0;
    virtual float getWeight() = 0;       // kg
    virtual float getForce() = 0;        // Newtons (weight * 9.81)
};
```

### Implementations

#### ForceSensorHX711
- **Hardware**: HX711 load cell amplifier (SPI-like protocol)
- **Data pins**: Configurable GPIO (dataPin, clockPin)
- **Filtering**: Moving-window average (10 samples, configurable)
- **Units**: Internally uses grams; converts to kg on `getWeight()`
- **File**: src/ForceSensorHX711.h / src/ForceSensorHX711.cpp

#### ForceSensorAnu78025
- **Hardware**: NAU7802 (Qwiic Scale) via I2C
- **I2C Address**: 0x2A (default)
- **Multiplexer**: TCA9548A (automatic channel selection)
- **Filtering**: Moving-window average (10 samples)
- **Units**: Stores directly in kg
- **File**: src/ForceSensorAnu78025.h / src/ForceSensorAnu78025.cpp

## Usage

### Single Sensor

```cpp
// HX711 Example
ForceSensor* sensor = new ForceSensorHX711(12, 13);  // data_pin=12, clock_pin=13
sensor->begin();
sensor->setCalibration(520000L, 423.11f);  // offset, scale (raw counts per kg)

// ANU78025 Example
ForceSensor* sensor = new ForceSensorAnu78025(0x2A, 2);  // address, mux_channel
sensor->begin();
sensor->setCalibration(520000L, 423.11f);
```

### Multiple Sensors (Polymorphic Array)

```cpp
ForceSensor* sensors[2];
sensors[0] = new ForceSensorHX711(12, 13);
sensors[1] = new ForceSensorAnu78025(0x2A, 1);

for (int i = 0; i < 2; i++) {
    sensors[i]->begin();
    sensors[i]->setCalibration(offset[i], scale[i]);
}

// Main loop
while (true) {
    for (int i = 0; i < 2; i++) {
        sensors[i]->update();
        if (sensors[i]->isReady()) {
            float force_N = sensors[i]->getForce();
            float mass_kg = sensors[i]->getWeight();
        }
    }
}
```

## Calibration Workflow

### 1. Tare (Zero-Point Calibration)

```cpp
sensor->tare();  // Call with no load applied
// Stores: offset = raw_value_at_zero_load
```

### 2. Measure Known Weight

```cpp
// Apply known mass (e.g., 5.0 kg) to sensor
// Wait for readings to stabilize (a few seconds)
long raw_with_load = sensor->readRaw();
```

### 3. Compute Scale Factor

```cpp
// scale = (raw_with_load - offset) / known_mass_kg
// Example: (550000 - 520000) / 5.0 = 6000 raw counts per kg
float scale = (float)(raw_with_load - offset) / 5.0;
```

### 4. Load Calibration

```cpp
sensor->setCalibration(offset, scale);
// Or separately:
sensor->setOffset(offset);
sensor->setScale(scale);
```

## Data Flow & Filtering

1. **Poll**: `sensor->update()` → reads raw ADC value (if available)
2. **Calculate**: `(raw - offset) / scale` → mass in kg
3. **Filter**: Add to 10-sample circular buffer, compute moving average
4. **Convert**: `mass_kg * 9.81` → force in Newtons
5. **Retrieve**: `getForce()` / `getWeight()` return filtered values

**Why filtering matters**: 
- Sensor noise is reduced by ~√10 ≈ 3x through averaging
- PID controllers converge better with smooth setpoints
- Safety limits have hysteresis tolerance

## Integration with ForceControlLoop

The `ForceControlLoop` class uses `ForceSensor*` (polymorphic pointer):

```cpp
ForceControlLoop controlLoop(motorDriver, sensor);  // sensor can be HX711 or ANU78025
controlLoop->setMode(MODE_PID_FORCE);
controlLoop->setForceSetpoint(5.0);  // 5.0 kg
controlLoop->update(dt);  // PID calls sensor->getForce() internally
```

## Testing

Each sensor type has an isolated test environment:

```bash
# Test HX711
pio run -e test_hx711 -t upload

# Test ANU78025 with multiplexer
pio run -e test_anu78025 -t upload

# Test complete force control loop
pio run -e test_one_motor_force_control -t upload
```

## Common Issues

### Sensor Not Responding (begin() fails)
- **HX711**: Check GPIO pins (dataPin, clockPin), power supply
- **ANU78025**: Check I2C wiring, multiplexer address (0x70), channel selection

### Readings Unstable
- Verify calibration offset and scale are correct
- Increase filter window (max 255 samples, currently 10)
- Check mechanical stability (loose load cell, vibration)

### Force Values Wrong
- Verify `getWeight()` returns correct mass (kg)
- Check calibration: does `(raw - offset) / scale` match known mass?
- Ensure load cell is level and strain-free

### ANU78025 Not Found
- Verify TCA9548A multiplexer I2C address (0x70)
- Check channel number (0-7)
- Verify NAU7802 I2C address (default 0x2A)

## Files

- [src/ForceSensor.h](src/ForceSensor.h) - Abstract base class and structs
- [src/ForceSensorHX711.h/cpp](src/ForceSensorHX711.h) - HX711 implementation
- [src/ForceSensorAnu78025.h/cpp](src/ForceSensorAnu78025.h) - NAU7802 implementation
- [src/test_hx711.cpp](src/test_hx711.cpp) - HX711 test sketch
- [src/test_anu78025.cpp](src/test_anu78025.cpp) - ANU78025 test sketch
- [src/test_one_motor_force_control.cpp](src/test_one_motor_force_control.cpp) - Force control loop test
