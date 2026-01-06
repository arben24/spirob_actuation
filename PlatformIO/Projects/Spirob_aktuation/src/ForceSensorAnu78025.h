#ifndef FORCE_SENSOR_ANU78025_H
#define FORCE_SENSOR_ANU78025_H

#include "ForceSensor.h"
#include <SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h>

class ForceSensorAnu78025 : public ForceSensor {
private:
    NAU7802 scale;
    float scaleFactor;
    long offset;
    uint8_t multiplexerChannel;
    bool ready;
    float filteredForce;
    const uint8_t filterSize = 10;
    float forceBuffer[10];
    uint8_t bufferIndex;

    void selectChannel(uint8_t channel);

public:
    ForceSensorAnu78025(uint8_t i2cAddress, uint8_t multiplexerChannel);
    bool begin() override;
    void tare() override;
    void setScale(float scale) override;
    long readRaw() override;
    float getForce() override;
    bool isReady() override;
    void update() override;
};

#endif