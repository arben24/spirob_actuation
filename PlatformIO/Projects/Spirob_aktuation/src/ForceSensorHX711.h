#ifndef FORCE_SENSOR_HX711_H
#define FORCE_SENSOR_HX711_H

#include "ForceSensor.h"
#include <HX711.h>

/**
 * @class ForceSensorHX711
 * @brief HX711 load cell amplifier implementation
 * 
 * Provides HX711-specific hardware control while conforming to ForceSensor interface.
 * Non-blocking polling with moving-window averaging filter.
 */
class ForceSensorHX711 : public ForceSensor {
private:
    HX711 scale;
    float scaleFactor;
    long offset;
    uint8_t gain;
    bool ready;
    float filteredWeight;
    const uint8_t filterSize;
    float weightBuffer[10];
    uint8_t bufferIndex;
    int dataPin;
    int clockPin;
    long currentRawValue;

public:
    /**
     * @brief Construct HX711 sensor
     * @param dataPin GPIO pin for DOUT
     * @param clockPin GPIO pin for SCK
     * @param gain ADC gain (128, 64, or 32)
     * @param filterSize Moving average filter window size
     */
    ForceSensorHX711(int dataPin, int clockPin, uint8_t gain = 128, const uint8_t filterSize = 1);

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

    // === HX711-specific accessors ===
    long getOffset() const { return offset; }
};

#endif // FORCE_SENSOR_HX711_H