#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>
#include <SMS_STS.h>

enum DriverMode { MODE_SERVO_POSITION = 0, MODE_WHEEL = 1 };

class MotorDriver {
private:
    uint8_t servoId;
    SMS_STS servo;
    HardwareSerial* serial;
    DriverMode currentMode;
    int8_t directionFactor = 1; // 1 or -1 for direction inversion

public:
    MotorDriver(uint8_t id, HardwareSerial* serial);
    void setMode(DriverMode mode);
    DriverMode getMode();
    void setPosition(int16_t position);
    void setSpeed(int16_t speed);
    void stop();
    int16_t getPosition();
    int getSpeed();
    int16_t getLoad();
    int getTemperature();
    int getVoltage();
    int getCurrent();
    int getMove();
    void setReverseDirection(bool reverse);
};

#endif