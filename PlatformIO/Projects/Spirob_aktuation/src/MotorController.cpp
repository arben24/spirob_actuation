#include "MotorController.h"

MotorController::MotorController(uint8_t id, HardwareSerial* serial)
    : servoId(id), servo(0), forceSensor(nullptr), distanceSensor(nullptr), mode(MODE_MANUAL_POSITION), manualPosition(0.0), wheelSpeed(0.0), forceSetpoint(0.0), maxSpeed(3000.0) {
    servo.pSerial = serial;
}

void MotorController::setForceSensor(ForceSensor* fs) {
    forceSensor = fs;
}

void MotorController::setDistanceSensor(DistanceSensorVL6180X* ds) {
    distanceSensor = ds;
}

void MotorController::setMode(MotorMode m) {
    mode = m;
    if (mode == MODE_PID_FORCE) {
        servo.WheelMode(servoId);
        pid.reset();
    } else if (mode == MODE_WHEEL) {
        servo.WheelMode(servoId);
    }
}

void MotorController::setManualPosition(float pos) {
    manualPosition = pos;
    if (mode == MODE_MANUAL_POSITION) {
        servo.WritePosEx(servoId, (int16_t)pos, 0, 0);
    }
}

void MotorController::setWheelSpeed(float speed) {
    wheelSpeed = speed;
    if (mode == MODE_WHEEL) {
        servo.WriteSpe(servoId, (int16_t)speed, 0);
    }
}

void MotorController::setForceSetpoint(float sp) {
    forceSetpoint = sp;
}

void MotorController::setPidTunings(float kp, float ki, float kd) {
    pid.setTunings(kp, ki, kd);
}

void MotorController::setMaxSpeed(float max) {
    maxSpeed = max;
    pid.setOutputLimits(-max, max);
}

void MotorController::update(float now) {
    if (mode == MODE_PID_FORCE && forceSensor && forceSensor->isReady()) {
        float force = forceSensor->getForce();
        float output = pid.update(force, now);
        servo.WriteSpe(servoId, (int16_t)output, 0);
    }
}

int MotorController::getPosition() {
    servo.FeedBack(servoId);
    return servo.ReadPos(servoId);
}

float MotorController::getForce() {
    return forceSensor ? forceSensor->getForce() : 0.0;
}

float MotorController::getDistance() {
    return distanceSensor ? distanceSensor->getDistance() : 0.0;
}