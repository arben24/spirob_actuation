#include "PidController.h"

PidController::PidController(float kp, float ki, float kd, float outMin, float outMax, float sampleTime)
    : Kp(kp), Ki(ki), Kd(kd), setpoint(0.0), integral(0.0), prevError(0.0), lastTime(0), outMin(outMin), outMax(outMax), sampleTime(sampleTime), lastOutput(0.0) {}

void PidController::setTunings(float kp, float ki, float kd) {
    Kp = kp;
    Ki = ki;
    Kd = kd;
}

void PidController::setOutputLimits(float min, float max) {
    outMin = min;
    outMax = max;
}

void PidController::setSetpoint(float sp) {
    setpoint = sp;
}

void PidController::setSampleTime(float sampleTime) {
    this->sampleTime = sampleTime;    //in ms
}

float PidController::update(float measurement) {

    unsigned long currentTime = millis();

    // On first call after reset(), seed lastTime so the first
    // update fires exactly one sampleTime later rather than immediately.
    if (lastTime == 0) {
        lastTime = currentTime;
        return lastOutput;
    }

    if (currentTime - lastTime >= (unsigned long)sampleTime) {

        float dt = (currentTime - lastTime) / 1000.0f;  // ms → s
        lastTime = currentTime;

        float error = setpoint - measurement;

        float P = Kp * error;

        integral += Ki * error * dt;
        float iLimit = fabs(outMax) * 10.0f;
        if (integral >  iLimit) integral =  iLimit;
        if (integral < -iLimit) integral = -iLimit;

        float dError = (dt > 0.0f) ? (error - prevError) / dt : 0.0f;
        float D = Kd * dError;
        prevError = error;

        float output = P + integral + D;
        if (output > outMax) output = outMax;
        if (output < outMin) output = outMin;

        lastOutput = output;
    }

    return lastOutput;
}

void PidController::reset() {
    integral = 0.0;
    prevError = 0.0;
    lastTime = 0;
    lastOutput = 0.0;
}