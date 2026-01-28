#ifndef SAFETY_MANAGER_H
#define SAFETY_MANAGER_H

#include "MotorDriver.h"
#include "ForceSensor.h"
#include "DistanceSensorVL6180X.h"

enum SystemState { STATE_NORMAL, STATE_ALARM };

class SafetyManager {
private:
    MotorDriver* motorDrivers[2];
    ForceSensor* forceSensors[2];
    DistanceSensorVL6180X* distanceSensors[2];
    float maxForce[2];
    float minDistance[2];
    float maxDistance[2];
    SystemState state;
    String alarmReason;

public:
    SafetyManager();
    void setMotorDriver(int index, MotorDriver* motorDriver);
    void setSensors(int index, ForceSensor* fs, DistanceSensorVL6180X* ds);
    void setLimits(int index, float maxF, float minD, float maxD);
    void checkLimits();
    SystemState getState();
    String getAlarmReason();
    bool resetAlarm();
};

#endif