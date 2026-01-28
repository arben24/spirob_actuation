#include "ForceSensorHX711.h"

ForceSensorHX711::ForceSensorHX711(int dataPin, int clockPin, uint8_t gain, const uint8_t filterSize)
    : scaleFactor(1.0), offset(0), gain(gain), ready(false), 
      filteredWeight(0.0), bufferIndex(0), dataPin(dataPin), 
      clockPin(clockPin), currentRawValue(0), filterSize(filterSize) 
{
    for (uint8_t i = 0; i < filterSize; i++) {
        weightBuffer[i] = 0.0;
    }
}

bool ForceSensorHX711::begin() {
    scale.begin((byte)dataPin, (byte)clockPin, gain);
    // Initialize pins; hardware check done in update() loop when isReady()
    return true;
}

void ForceSensorHX711::tare() {
    // Tare requires sensor ready. Best called from loop when isReady() is true.
    if (scale.wait_ready_timeout(1000)) { // Wait max 1 second
        scale.tare();
        offset = scale.get_offset();
        Serial.print("HX711 Tare executed. Offset: ");
        Serial.println(offset);
    } else {
        Serial.println("HX711 Tare failed (Timeout)");
    }
}

void ForceSensorHX711::setOffset(long offsetVal) {
    this->offset = offsetVal;
    scale.set_offset(offsetVal);
}

void ForceSensorHX711::setScale(float scaleVal) {
    scaleFactor = scaleVal;
}

void ForceSensorHX711::setCalibration(long offsetVal, float scaleVal) {
    setOffset(offsetVal);
    setScale(scaleVal);
}

long ForceSensorHX711::readRaw() {
    // Return last read value (non-blocking)
    return currentRawValue;
}

float ForceSensorHX711::getWeight() {
    // Return filtered mass in kg
    return filteredWeight / 1000.0f; // Convert from grams to kg
}

float ForceSensorHX711::getForce() {
    // F = m * g (mass in kg * 9.81 m/s²)
    return (filteredWeight / 1000.0f) * 9.80665f;
}

bool ForceSensorHX711::isReady() {
    return scale.is_ready();
}

void ForceSensorHX711::update() {
    // Non-blocking polling: read if data available
    if (scale.is_ready()) {
        long raw = scale.read();
        currentRawValue = raw;
        
        // Calculate mass in grams: (raw - offset) / scaleFactor
        // scaleFactor is "raw counts per gram"
        float weight_g = (float)(raw - offset) / scaleFactor;
        
        // Apply moving-window filter
        weightBuffer[bufferIndex] = weight_g;
        bufferIndex = (bufferIndex + 1) % filterSize;
        
        float sum = 0.0f;
        for (uint8_t i = 0; i < filterSize; i++) {
            sum += weightBuffer[i];
        }
        filteredWeight = sum / filterSize;
        ready = true;
    }
}
