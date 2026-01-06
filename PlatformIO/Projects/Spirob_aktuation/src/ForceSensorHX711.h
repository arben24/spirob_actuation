#ifndef FORCE_SENSOR_HX711_H
#define FORCE_SENSOR_HX711_H

#include "ForceSensor.h"
#include <HX711.h>

class ForceSensorHX711 : public ForceSensor {
private:
    HX711 scale;
    float scaleFactor;
    long offset;
    uint8_t gain;
    bool ready;
    float filteredForce;
    const uint8_t filterSize = 10;
    float forceBuffer[10];
    uint8_t bufferIndex;
    int dataPin;
    int clockPin;

public:
    ForceSensorHX711(int dataPin, int clockPin, uint8_t gain = 128);
    bool begin() override;
    void tare() override;
    void setScale(float scale) override;
    long readRaw() override;
    float getForce() override;
    bool isReady() override;
    void update() override;
};

#endif