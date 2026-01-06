#ifndef SAFETY_MANAGER_H
#define SAFETY_MANAGER_H

#include "MotorController.h"

enum SystemState { STATE_NORMAL, STATE_ALARM };

class SafetyManager {
private:
    MotorController* motors[2];
    float maxForce[2];
    float minDistance[2];
    float maxDistance[2];
    SystemState state;
    String alarmReason;

public:
    SafetyManager();
    void setMotor(int index, MotorController* motor);
    void setLimits(int index, float maxF, float minD, float maxD);
    void checkLimits();
    SystemState getState();
    String getAlarmReason();
    bool resetAlarm();
};

#endif