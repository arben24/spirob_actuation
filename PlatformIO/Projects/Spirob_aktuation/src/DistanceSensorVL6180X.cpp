#include "DistanceSensorVL6180X.h"

DistanceSensorVL6180X::DistanceSensorVL6180X(uint8_t channel) : multiplexerChannel(channel), windowIndex(0), windowCount(0), filteredDistance(0.0) {
    for (uint8_t i = 0; i < windowSize; i++) {
        window[i] = 0;
    }
}

void DistanceSensorVL6180X::selectChannel(uint8_t channel) {
    Wire.beginTransmission(0x70);
    Wire.write(1 << channel);
    Wire.endTransmission();
}

bool DistanceSensorVL6180X::begin() {
    selectChannel(multiplexerChannel);
    if (!sensor.begin()) {
        return false;
    }
    // Set recommended registers
    Wire.beginTransmission(0x29);
    Wire.write(0x00); Wire.write(0x11); Wire.write(0x10);
    Wire.write(0x00); Wire.write(0x10); Wire.write(0x30);
    Wire.write(0x00); Wire.write(0x0F); Wire.write(0x46);
    Wire.write(0x00); Wire.write(0x31); Wire.write(0xFF);
    Wire.write(0x00); Wire.write(0x41); Wire.write(0x63);
    Wire.write(0x00); Wire.write(0x2E); Wire.write(0x01);
    Wire.write(0x00); Wire.write(0x14); Wire.write(0x24);
    Wire.endTransmission();
    sensor.startRangeContinuous(10);
    return true;
}

void DistanceSensorVL6180X::pushSample(uint16_t v) {
    window[windowIndex] = v;
    windowIndex = (windowIndex + 1) % windowSize;
    if (windowCount < windowSize) windowCount++;
}

void DistanceSensorVL6180X::computeStats(float &mean, float &stddev) {
    if (windowCount == 0) {
        mean = 0.0; stddev = 0.0; return;
    }
    float sum = 0.0;
    for (uint8_t j = 0; j < windowCount; j++) sum += window[j];
    mean = sum / windowCount;
    float var = 0.0;
    for (uint8_t j = 0; j < windowCount; j++) {
        float d = window[j] - mean;
        var += d * d;
    }
    var /= windowCount;
    stddev = sqrt(var);
}

void DistanceSensorVL6180X::update() {
    selectChannel(multiplexerChannel);
    if (sensor.isRangeComplete()) {
        uint8_t range = sensor.readRangeResult();
        uint8_t status = sensor.readRangeStatus();
        if (status == VL6180X_ERROR_NONE && range != 0) {
            pushSample(range);
            float mean, stddev;
            computeStats(mean, stddev);
            filteredDistance = mean;
        }
    }
}

float DistanceSensorVL6180X::getDistance() {
    return filteredDistance;
}

bool DistanceSensorVL6180X::isReady() {
    return windowCount > 0;
}