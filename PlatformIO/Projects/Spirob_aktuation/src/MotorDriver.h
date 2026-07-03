#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>
#include <SMS_STS.h>
#include "IMotorDriver.h"

class MotorDriver : public IMotorDriver {
private:
    uint8_t servoId;
    SMS_STS servo;
    HardwareSerial* serial;
    DriverMode currentMode;
    int8_t directionFactor = 1; // 1 or -1 for direction inversion

public:
    MotorDriver(uint8_t id, HardwareSerial* serial);
    void setMode(DriverMode mode) override;
    DriverMode getMode() override;
    void setPosition(int16_t position) override;
    void setSpeed(int16_t speed) override;
    void stop() override;
    int16_t getPosition() override;
    int getSpeed() override;
    int16_t getLoad();
    int getTemperature();
    int getVoltage();
    int getCurrent();
    int getMove();
    void setReverseDirection(bool reverse) override;
};

#endif