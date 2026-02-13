#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

class PidController {
private:
    float Kp, Ki, Kd;
    float setpoint;
    float integral;
    float prevError;
    unsigned long lastTime;
    float outMin, outMax;
    float sampleTime;

public:
    PidController(float kp = 10.0, float ki = 0.0, float kd = 0.0, float outMin = -1000.0, float outMax = 1000.0, float sampleTime = 100.0);
    void setTunings(float kp, float ki, float kd);
    float getKp() { return Kp; }
    float getKi() { return Ki; }
    float getKd() { return Kd; }
    void setOutputLimits(float min, float max);
    void setSetpoint(float sp);
    void setSampleTime(float dt);
    float update(float measurement);
    void reset();
};

#endif