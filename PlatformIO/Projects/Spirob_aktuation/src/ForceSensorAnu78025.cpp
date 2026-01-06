#include "ForceSensorAnu78025.h"
#include <Wire.h>

ForceSensorAnu78025::ForceSensorAnu78025(uint8_t i2cAddress, uint8_t multiplexerChannel)
    : scaleFactor(1.0), offset(0), multiplexerChannel(multiplexerChannel), ready(false), filteredForce(0.0), bufferIndex(0) {
    for (uint8_t i = 0; i < filterSize; i++) {
        forceBuffer[i] = 0.0;
    }
}

void ForceSensorAnu78025::selectChannel(uint8_t channel) {
    Wire.beginTransmission(0x70); // TCA9548A address
    Wire.write(1 << channel);
    Wire.endTransmission();
}

bool ForceSensorAnu78025::begin() {
    selectChannel(multiplexerChannel);
    if (scale.begin() == false) {
        return false;
    }
    scale.setGain(NAU7802_GAIN_128);
    scale.setSampleRate(NAU7802_SPS_80);
    scale.calibrateAFE();
    tare();
    return true;
}

void ForceSensorAnu78025::tare() {
    selectChannel(multiplexerChannel);
    if (scale.available()) {
        scale.calculateZeroOffset(64);
        offset = scale.getZeroOffset();
    }
}

void ForceSensorAnu78025::setScale(float scale) {
    scaleFactor = scale;
}

long ForceSensorAnu78025::readRaw() {
    selectChannel(multiplexerChannel);
    if (scale.available()) {
        return scale.getReading();
    }
    return 0;
}

float ForceSensorAnu78025::getForce() {
    return filteredForce;
}

bool ForceSensorAnu78025::isReady() {
    return ready;
}

void ForceSensorAnu78025::update() {
    selectChannel(multiplexerChannel);
    if (scale.available()) {
        long raw = scale.getReading();
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