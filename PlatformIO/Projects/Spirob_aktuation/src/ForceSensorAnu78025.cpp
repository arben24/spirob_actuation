#include "ForceSensorAnu78025.h"

#define TCA9548A_ADDR 0x70

ForceSensorAnu78025::ForceSensorAnu78025(uint8_t i2cAddress, uint8_t multiplexerChannel, uint8_t sampleRate)
    : i2cAddress(i2cAddress), multiplexerChannel(multiplexerChannel), 
      scaleFactor(1.0), offset(0), ready(false), currentRaw(0), 
      filteredWeight(0.0), bufferIndex(0), sampleRate(sampleRate)
{
    for (uint8_t i = 0; i < filterSize; i++) {
        forceBuffer[i] = 0.0;
    }
}

    /*  NAU7802_SPS_320 = 0b111 = 7,
  NAU7802_SPS_80 = 0b011 = 3,
  NAU7802_SPS_40 = 0b010 = 2,
  NAU7802_SPS_20 = 0b001 = 1,
  NAU7802_SPS_10 = 0b000 = 0;*/

void ForceSensorAnu78025::selectChannel(uint8_t channel) {
    Wire.beginTransmission(TCA9548A_ADDR);
    Wire.write(1 << channel);
    Wire.endTransmission();
}

bool ForceSensorAnu78025::begin() {
    selectChannel(multiplexerChannel);
    
    if (scale.begin() == false) {
        return false;
    }

    scale.setGain(NAU7802_GAIN_128);
    scale.setSampleRate(sampleRate);
    
    // Calibrate AFE (Analog Front End)
    scale.calibrateAFE();
    
    return true;
}

void ForceSensorAnu78025::tare() {
    selectChannel(multiplexerChannel);
    // Calculate zero offset with 64 samples for stable tare
    scale.calculateZeroOffset(64);
    offset = scale.getZeroOffset();
    Serial.print("ANU78025 Tare Offset: ");
    Serial.println(offset);
}

void ForceSensorAnu78025::setCalibration(long offsetVal, float scaleVal) {
    setOffset(offsetVal);
    setScale(scaleVal);
}

void ForceSensorAnu78025::setOffset(long offsetVal) {
    this->offset = offsetVal;
    selectChannel(multiplexerChannel);
    scale.setZeroOffset(offsetVal);
}

void ForceSensorAnu78025::setScale(float scaleVal) {
    this->scaleFactor = scaleVal;
    selectChannel(multiplexerChannel);
    scale.setCalibrationFactor(scaleVal);
}

long ForceSensorAnu78025::readRaw() {
    selectChannel(multiplexerChannel);
    if (scale.available()) {
        return scale.getReading();
    }
    return currentRaw;
}

float ForceSensorAnu78025::getWeight() {
    // Return filtered mass in kg
    return filteredWeight;
}

float ForceSensorAnu78025::getForce() {
    // F = m * g (mass in kg * 9.81 m/s²)
    return (filteredWeight/1000) * 9.81f;
}

bool ForceSensorAnu78025::isReady() {
    return ready;
}

void ForceSensorAnu78025::update() {
    selectChannel(multiplexerChannel);
    
    if (scale.available()) {
        long raw = scale.getReading();
        currentRaw = raw;
        
        // Calculate mass in kg: (raw - offset) / scaleFactor
        // scaleFactor is "raw counts per kg"
        float weight_kg = (float)(raw - offset) / scaleFactor;
        
        // Apply moving-window filter
        forceBuffer[bufferIndex] = weight_kg;
        bufferIndex = (bufferIndex + 1) % filterSize;
        
        float sum = 0.0f;
        for (uint8_t i = 0; i < filterSize; i++) {
            sum += forceBuffer[i];
        }
        filteredWeight = sum / filterSize;
        
        ready = true;
    }
}
