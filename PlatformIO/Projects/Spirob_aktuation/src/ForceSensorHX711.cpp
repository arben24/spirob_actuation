#include "ForceSensorHX711.h"

ForceSensorHX711::ForceSensorHX711(int dataPin, int clockPin, uint8_t gain)
    : scaleFactor(1.0), offset(0), gain(gain), ready(false), filteredForce(0.0), bufferIndex(0), dataPin(dataPin), clockPin(clockPin) {
    for (uint8_t i = 0; i < filterSize; i++) {
        forceBuffer[i] = 0.0;
    }
}

bool ForceSensorHX711::begin() {
    scale.begin((byte)dataPin, (byte)clockPin, gain);
    if (scale.is_ready()) {
        tare();
        return true;
    }
    return false;
}

void ForceSensorHX711::tare() {
    if (scale.is_ready()) {
        scale.tare();
        offset = scale.get_offset();
    }
}

void ForceSensorHX711::setScale(float scale) {
    scaleFactor = scale;
}

long ForceSensorHX711::readRaw() {
    if (scale.is_ready()) {
        return scale.read();
    }
    return 0;
}

float ForceSensorHX711::getForce() {
    return filteredForce;
}

bool ForceSensorHX711::isReady() {
    return ready;
}

void ForceSensorHX711::update() {
    if (scale.is_ready()) {
        long raw = scale.read();
        float force = (raw - offset) * scaleFactor;
        forceBuffer[bufferIndex] = force;
        bufferIndex = (bufferIndex + 1) % filterSize;
        float sum = 0.0;
        for (uint8_t i = 0; i < filterSize; i++) {
            sum += forceBuffer[i];
        }
        filteredForce = sum / filterSize;
        ready = true;
    } else {
        ready = false;
    }
}