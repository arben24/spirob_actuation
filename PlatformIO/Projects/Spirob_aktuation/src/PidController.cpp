#include "PidController.h"

PidController::PidController(float kp, float ki, float kd, float outMin, float outMax)
    : Kp(kp), Ki(ki), Kd(kd), setpoint(0.0), integral(0.0), prevError(0.0), lastTime(0.0), outMin(outMin), outMax(outMax) {}

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

float PidController::update(float measurement, float now) {
    float error = setpoint - measurement;
    Serial.printf("PID Update: Setpoint=%.3f, Measurement=%.3f, Error=%.3f\n", setpoint, measurement, error);
    float dt = (lastTime > 0) ? (now - lastTime) / 1000.0f : 0.0f;
    if (dt < 0.001f) dt = 0.001f;
    lastTime = now;

    float P = Kp * error;
    integral += Ki * error * dt;
    float iLimit = fabs(outMax) * 10.0f;
    if (integral > iLimit) integral = iLimit;
    if (integral < -iLimit) integral = -iLimit;
    float I = integral;

    float dError = (dt > 0) ? (error - prevError) / dt : 0.0f;
    float D = Kd * dError;
    prevError = error;

    float output = P + I + D;
    if (output > outMax) output = outMax;
    if (output < outMin) output = outMin;

    return output;
}

void PidController::reset() {
    integral = 0.0;
    prevError = 0.0;
    lastTime = 0.0;
}