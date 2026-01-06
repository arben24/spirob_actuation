#include "SafetyManager.h"

SafetyManager::SafetyManager() : state(STATE_NORMAL), alarmReason("") {}

void SafetyManager::setMotor(int index, MotorController* motor) {
    if (index < 2) motors[index] = motor;
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
        if (motors[i]) {
            float force = motors[i]->getForce();
            float dist = motors[i]->getDistance();
            if (force > maxForce[i]) {
                state = STATE_ALARM;
                alarmReason = "Force exceeded on motor " + String(i);
                // Stop motors
                motors[i]->setMode(MODE_MANUAL_POSITION);
                motors[i]->setManualPosition(0);
                return;
            }
            if (dist < minDistance[i] || dist > maxDistance[i]) {
                state = STATE_ALARM;
                alarmReason = "Distance out of range on motor " + String(i);
                motors[i]->setMode(MODE_MANUAL_POSITION);
                motors[i]->setManualPosition(0);
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
        if (motors[i]) {
            float force = motors[i]->getForce();
            float dist = motors[i]->getDistance();
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