#ifndef FORCE_SENSOR_ANU78025_H
#define FORCE_SENSOR_ANU78025_H

#include <Arduino.h>
#include <Wire.h>
#include "ForceSensor.h"
#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h"

/**
 * @class ForceSensorAnu78025
 * @brief NAU7802 (Qwiic Scale) implementation via I2C with TCA9548A multiplexer
 * 
 * Provides NAU7802-specific hardware control while conforming to ForceSensor interface.
 * Non-blocking polling with moving-window averaging filter.
 * Handles I2C multiplexer selection automatically.
 */
class ForceSensorAnu78025 : public ForceSensor {
public:
    /**
     * @brief Construct ANU78025 sensor
     * @param i2cAddress I2C address of NAU7802 (typically 0x2A)
     * @param multiplexerChannel TCA9548A channel for this sensor
     */
    ForceSensorAnu78025(uint8_t i2cAddress = 0x2A, uint8_t multiplexerChannel = 0, uint8_t sampleRate = NAU7802_SPS_80);

    // === ForceSensor Interface Implementation ===
    bool begin() override;
    void update() override;
    void tare() override;
    void setCalibration(long offset, float scale) override;
    void setOffset(long offset) override;
    void setScale(float scale) override;
    bool isReady() override;
    long readRaw() override;
    float getWeight() override;
    float getForce() override;

    // === ANU78025-specific accessors ===
    float getScale() const { return scaleFactor; }
    long getOffset() const { return offset; }

private:
    NAU7802 scale;
    uint8_t i2cAddress;
    uint8_t multiplexerChannel;
    uint8_t sampleRate;

    // Calibration data
    float scaleFactor;
    long offset;

    // Status
    bool ready;
    long currentRaw;

    // Filter: moving-window averaging
    static const uint8_t filterSize = 1;
    float forceBuffer[filterSize];
    uint8_t bufferIndex;
    float filteredWeight; // in kg

    // Helper
    void selectChannel(uint8_t channel);
};

#endif // FORCE_SENSOR_ANU78025_H
