#include "SafetyManager.h"

SafetyManager::SafetyManager() : state(STATE_NORMAL), alarmReason("") {}

void SafetyManager::setMotorDriver(int index, MotorDriver* motorDriver) {
    if (index < 2) motorDrivers[index] = motorDriver;
}

void SafetyManager::setSensors(int index, ForceSensor* fs, DistanceSensorVL6180X* ds) {
    if (index < 2) {
        forceSensors[index] = fs;
        distanceSensors[index] = ds;
    }
}

void SafetyManager::setLimits(int index, float maxF, float minD, float maxD) {
    if (index < 2) {
        maxForce[index] = maxF;
        minDistance[index] = minD;
        maxDistance[index] = maxD;
    }
}

void SafetyManager::checkLimits() {
    if (state == STATE_ALARM) return;
    for (int i = 0; i < 2; i++) {
        if (forceSensors[i] && distanceSensors[i]) {
            float force = forceSensors[i]->getForce();
            float dist = distanceSensors[i]->getDistance();
            if (force > maxForce[i]) {
                state = STATE_ALARM;
                alarmReason = "Force exceeded on motor " + String(i);
                motorDrivers[i]->stop();
                return;
            }
            if (dist < minDistance[i] || dist > maxDistance[i]) {
                state = STATE_ALARM;
                alarmReason = "Distance out of range on motor " + String(i);
                motorDrivers[i]->stop();
                return;
            }
        }
    }
}

SystemState SafetyManager::getState() {
    return state;
}

String SafetyManager::getAlarmReason() {
    return alarmReason;
}

bool SafetyManager::resetAlarm() {
    // Check if conditions are safe
    bool safe = true;
    for (int i = 0; i < 2; i++) {
        if (forceSensors[i] && distanceSensors[i]) {
            float force = forceSensors[i]->getForce();
            float dist = distanceSensors[i]->getDistance();
            if (force > maxForce[i] || dist < minDistance[i] || dist > maxDistance[i]) {
                safe = false;
            }
        }
    }
    if (safe) {
        state = STATE_NORMAL;
        alarmReason = "";
        return true;
    }
    return false;
}