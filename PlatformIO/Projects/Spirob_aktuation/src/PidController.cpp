#include "PidController.h"

PidController::PidController(float kp, float ki, float kd, float outMin, float outMax, float sampleTime)
    : Kp(kp), Ki(ki), Kd(kd), setpoint(0.0), integral(0.0), prevError(0.0), lastTime(0.0), outMin(outMin), outMax(outMax), sampleTime(sampleTime) {}

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
    //Serial.printf("CurrentTime: %lu, LastTime: %lu, SampleTime: %.2f\n", currentTime, lastTime, sampleTime);
    //Serial.println((float)(currentTime - lastTime));

    if((float)(currentTime - lastTime) >= this->sampleTime) {
        //Serial.println("PID Update triggered");
        float error = setpoint - measurement;
        //Serial.printf("PID Update: Setpoint=%.3f, Measurement=%.3f, Error=%.3f\n", setpoint, measurement, error);
        float dt = (currentTime - lastTime) / 1000.0f; // Convert to seconds

        lastTime = currentTime;

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
        //Serial.println(currentTime - lastTime);

        return output;
    }
}

void PidController::reset() {
    integral = 0.0;
    prevError = 0.0;
    lastTime = 0.0;
}