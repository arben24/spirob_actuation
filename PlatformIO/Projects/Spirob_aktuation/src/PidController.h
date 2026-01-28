#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

class PidController {
private:
    float Kp, Ki, Kd;
    float setpoint;
    float integral;
    float prevError;
    float lastTime;
    float outMin, outMax;

public:
    PidController(float kp = 10.0, float ki = 0.0, float kd = 0.0, float outMin = -1000.0, float outMax = 1000.0);
    void setTunings(float kp, float ki, float kd);
    void setOutputLimits(float min, float max);
    void setSetpoint(float sp);
    float update(float measurement, float now);
    void reset();
};

#endif