#ifndef FORCE_SENSOR_H
#define FORCE_SENSOR_H

#include <Arduino.h>

// ============================================================================
// ENUMS AND STRUCTS
// ============================================================================

/**
 * @enum ForceSensorType
 * @brief Supported force sensor types
 */
enum ForceSensorType {
    FORCE_HX711,      // HX711 load cell amplifier
    FORCE_ANU78025    // NAU7802 (Qwiic Scale, ANU78025)
};

/**
 * @struct ForceSensorConfig
 * @brief Configuration for force sensors
 */
struct ForceSensorConfig {
    ForceSensorType type;
    long offset;       // Zero-load raw ADC value
    float scale;       // Raw counts per kg
    float nominalLoad; // Expected load (for safety limits)
};

// ============================================================================
// ABSTRACT BASE CLASS
// ============================================================================

/**
 * @class ForceSensor
 * @brief Abstract interface for force sensors
 * 
 * Provides unified interface for different sensor types (HX711, ANU78025).
 * Implementations must override all pure virtual methods.
 * 
 * Usage:
 *   ForceSensor* sensor = new ForceSensorHX711(...);
 *   sensor->begin();
 *   sensor->setCalibration(offset, scale);
 *   
 *   while(1) {
 *       sensor->update();
 *       if (sensor->isReady()) {
 *           float force_N = sensor->getForce();
 *           float mass_kg = sensor->getWeight();
 *       }
 *   }
 */
class ForceSensor {
public:
    virtual ~ForceSensor() = default;

    // ========================================================================
    // INITIALIZATION
    // ========================================================================

    /**
     * @brief Initialize the sensor hardware
     * @return true if successful, false if sensor not responding
     */
    virtual bool begin() = 0;

    /**
     * @brief Poll sensor for new data (non-blocking)
     * 
     * Should be called regularly in the main loop. Updates internal
     * state and applies filtering. No blocking operations.
     */
    virtual void update() = 0;

    // ========================================================================
    // CALIBRATION
    // ========================================================================

    /**
     * @brief Zero the sensor (tare operation)
     * 
     * Captures current reading as the zero point. Call when no load
     * is applied. Updates offset internally.
     */
    virtual void tare() = 0;

    /**
     * @brief Set both offset and scale factor
     * @param offset Raw ADC value at zero load
     * @param scale Raw counts per kg (computed during calibration)
     */
    virtual void setCalibration(long offset, float scale) = 0;

    /**
     * @brief Set the zero-load offset
     * @param offset Raw ADC value at zero load
     */
    virtual void setOffset(long offset) = 0;

    /**
     * @brief Set the scale factor (raw counts per kg)
     * @param scale Raw ADC counts per kilogram
     */
    virtual void setScale(float scale) = 0;

    // ========================================================================
    // DATA RETRIEVAL
    // ========================================================================

    /**
     * @brief Check if new sensor data is available
     * @return true if sensor has data, false otherwise
     */
    virtual bool isReady() = 0;

    /**
     * @brief Read raw ADC value
     * @return Raw sensor reading (unscaled)
     */
    virtual long readRaw() = 0;

    /**
     * @brief Get mass in kilograms
     * @return Mass in kg (filtered)
     */
    virtual float getWeight() = 0;

    /**
     * @brief Get force in Newtons
     * @return Force in N (mass_kg * 9.81, filtered)
     */
    virtual float getForce() = 0;
};

#endif // FORCE_SENSOR_H
