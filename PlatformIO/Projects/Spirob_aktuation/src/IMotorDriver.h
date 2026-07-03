#ifndef I_MOTOR_DRIVER_H
#define I_MOTOR_DRIVER_H

#include <Arduino.h>

// Shared by MotorDriver (Feetech/SC-Servo) and MotorDriverRobStride (CAN/MIT) so
// ForceControlLoop can drive either actuator generation without knowing which one
// is wired up.
enum DriverMode { MODE_SERVO_POSITION = 0, MODE_WHEEL = 1 };

class IMotorDriver {
public:
    virtual ~IMotorDriver() {}
    virtual void setMode(DriverMode mode) = 0;
    virtual DriverMode getMode() = 0;
    virtual void setPosition(int16_t position) = 0;
    virtual void setSpeed(int16_t speed) = 0;
    virtual void stop() = 0;
    virtual int16_t getPosition() = 0;
    virtual int getSpeed() = 0;
    virtual void setReverseDirection(bool reverse) = 0;
};

#endif
