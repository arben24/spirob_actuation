#ifndef DISTANCE_SENSOR_VL6180X_H
#define DISTANCE_SENSOR_VL6180X_H

#include <Adafruit_VL6180X.h>
#include <Wire.h>

class DistanceSensorVL6180X {
private:
    Adafruit_VL6180X sensor;
    uint8_t multiplexerChannel;
    const uint8_t windowSize = 10;
    uint16_t window[10];
    uint8_t windowIndex;
    uint8_t windowCount;
    float filteredDistance;
    uint8_t lastStatus;

    bool selectChannel(uint8_t channel);
    void pushSample(uint16_t v);
    void computeStats(float &mean, float &stddev);

public:
    DistanceSensorVL6180X(uint8_t channel);
    bool begin();
    void update();
    float getDistance();
    bool isReady();
    uint8_t getLastStatus();
    uint8_t getWindowCount();
};

#endif