#include "DistanceSensorVL6180X.h"

DistanceSensorVL6180X::DistanceSensorVL6180X(uint8_t channel) : multiplexerChannel(channel), windowIndex(0), windowCount(0), filteredDistance(0.0), lastStatus(0) {
    for (uint8_t i = 0; i < windowSize; i++) {
        window[i] = 0;
    }
}

bool DistanceSensorVL6180X::selectChannel(uint8_t channel) {
    Wire.beginTransmission(0x70); // TCA9548A I2C multiplexer address
    Wire.write(1 << channel);
    return Wire.endTransmission() == 0; // Return true if successful
}

bool DistanceSensorVL6180X::begin() {
    if (!selectChannel(multiplexerChannel)) {
        return false; // Multiplexer not reachable
    }
    if (!sensor.begin()) {
        return false;
    }
    // Set Max Convergence Time to 50ms to fix VL6180X_ERROR_EARLY_CONVERGENCE_ESTIMATE (Status 6)
    Wire.beginTransmission(0x29); // VL6180X I2C address
    Wire.write(0x00); Wire.write(0x1C); Wire.write(0x32); // Register 0x001C = 0x32 (50ms)
    Wire.endTransmission();
    // Start continuous ranging with 50ms interval to avoid timing issues
    sensor.startRangeContinuous(50);
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
    if (!selectChannel(multiplexerChannel)) {
        return; // Skip if multiplexer not accessible
    }
    if (sensor.isRangeComplete()) {
        uint8_t status = sensor.readRangeStatus();
        lastStatus = status; // Always update last status for debugging
        if (status == VL6180X_ERROR_NONE) {
            uint8_t range = sensor.readRangeResult();
            if (range != 0) { // Ensure valid range value
                pushSample(range);
                float mean, stddev;
                computeStats(mean, stddev);
                filteredDistance = mean;
            }
        }
    }
}

float DistanceSensorVL6180X::getDistance() {
    return filteredDistance;
}

bool DistanceSensorVL6180X::isReady() {
    return windowCount >= 5; // Require at least 5 samples to avoid initial noise
}

uint8_t DistanceSensorVL6180X::getLastStatus() {
    return lastStatus;
}

uint8_t DistanceSensorVL6180X::getWindowCount() {
    return windowCount;
}