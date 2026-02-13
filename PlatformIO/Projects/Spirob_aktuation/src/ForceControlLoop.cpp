#include "ForceControlLoop.h"

ForceControlLoop::ForceControlLoop(MotorDriver* md, ForceSensor* fs)
    : motorDriver(md), forceSensor(fs), mode(MODE_MANUAL_POSITION), manualPosition(0.0), speedSetpoint(0.0), forceSetpoint(0.0), maxSpeed(3000.0) {
    pid.setOutputLimits(-maxSpeed, maxSpeed);
}

void ForceControlLoop::setMode(ControlMode m) {
    mode = m;
    motorDriver->setMode(MODE_SERVO_POSITION);  // Default mode
    
    if (mode == MODE_PID_FORCE) {
        pid.reset();
        motorDriver->setMode(MODE_WHEEL);  // PID output is speed command
    } else if (mode == MODE_SPEED_CONTROL) {
        motorDriver->setMode(MODE_WHEEL);
    } else if (mode == MODE_MANUAL_POSITION) {
        motorDriver->setMode(MODE_SERVO_POSITION);
    }
}

void ForceControlLoop::setManualPosition(float pos) {
    manualPosition = pos;
    if (mode == MODE_MANUAL_POSITION) {
        motorDriver->setPosition((int16_t)pos);
    }
}

void ForceControlLoop::setSpeedSetpoint(float speed) {
    speedSetpoint = speed;
    if (mode == MODE_SPEED_CONTROL) {
        motorDriver->setSpeed((int16_t)speed);
    }
}

void ForceControlLoop::setForceSetpoint(float sp) {
    forceSetpoint = sp;
    pid.setSetpoint(sp);
}

void ForceControlLoop::setPidTunings(float kp, float ki, float kd) {
    pid.setTunings(kp, ki, kd);
}

void ForceControlLoop::setMaxSpeed(float max) {
    maxSpeed = max;
    pid.setOutputLimits(-max, max);
}

void ForceControlLoop::setSampleTime(float sampleTime) {
    this->sampleTime = sampleTime;
    pid.setSampleTime(sampleTime);
}

float ForceControlLoop::update() {
    if (mode == MODE_PID_FORCE && forceSensor && forceSensor->isReady()) {
        float force = forceSensor->getForce();
        Output = pid.update(force);
        //Serial.printf("PID Update: Setpoint=%.3f N, Measured=%.3f N, Output=%.1f\n", forceSetpoint, force, Output);
    }
    return Output;
}

float ForceControlLoop::getForce() {
    return forceSensor ? forceSensor->getForce() : 0.0;
}