#ifndef FORCE_SENSOR_H
#define FORCE_SENSOR_H

#include <Arduino.h>

enum ForceSensorType { FORCE_NONE, FORCE_HX711, FORCE_ANU78025 };

struct ForceSensorConfig {
    ForceSensorType sensorType;
    float nominalLoad; // in kg
    float scaleFactor;
    float offset;
    // For HX711
    int hx711DataPin;
    int hx711ClockPin;
    uint8_t hx711Gain;
    // For ANU78025
    uint8_t i2cAddress;
    uint8_t multiplexerChannel;
};

class ForceSensor {
public:
    virtual ~ForceSensor() {}
    virtual bool begin() = 0;
    virtual void tare() = 0;
    virtual void setScale(float scale) = 0;
    virtual long readRaw() = 0;
    virtual float getForce() = 0;
    virtual bool isReady() = 0; // Check if new data is available
    virtual void update() = 0; // Poll for new data
};

#endif