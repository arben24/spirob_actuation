#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "PidController.h"
#include "ForceSensor.h"
#include "DistanceSensorVL6180X.h"
#include <SMS_STS.h>

enum MotorMode { MODE_MANUAL_POSITION, MODE_PID_FORCE, MODE_WHEEL };

class MotorController {
private:
    uint8_t servoId;
    SMS_STS servo;
    PidController pid;
    ForceSensor* forceSensor;
    DistanceSensorVL6180X* distanceSensor;
    MotorMode mode;
    float manualPosition;
    float wheelSpeed;
    float forceSetpoint;
    float maxSpeed;

public:
    MotorController(uint8_t id, HardwareSerial* serial);
    void setForceSensor(ForceSensor* fs);
    void setDistanceSensor(DistanceSensorVL6180X* ds);
    void setMode(MotorMode m);
    void setManualPosition(float pos);
    void setWheelSpeed(float speed);
    void setForceSetpoint(float sp);
    void setPidTunings(float kp, float ki, float kd);
    void setMaxSpeed(float max);
    void update(float now);
    int getPosition();
    float getForce();
    float getDistance();
};

#endif