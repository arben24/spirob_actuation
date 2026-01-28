#ifndef FORCE_CONTROL_LOOP_H
#define FORCE_CONTROL_LOOP_H

#include "MotorDriver.h"
#include "ForceSensor.h"
#include "PidController.h"

/**
 * @enum ControlMode
 * @brief Control modes for ForceControlLoop (distinct from MotorDriver::DriverMode)
 */
enum ControlMode { 
    MODE_MANUAL_POSITION,   // Direct position setpoint
    MODE_PID_FORCE,         // Force regulation via PID
    MODE_SPEED_CONTROL      // Direct speed setpoint
};

class ForceControlLoop {
private:
    MotorDriver* motorDriver;
    ForceSensor* forceSensor;
    PidController pid;
    ControlMode mode;
    float manualPosition;
    float speedSetpoint;
    float forceSetpoint;
    float maxSpeed;

public:
    ForceControlLoop(MotorDriver* md, ForceSensor* fs);
    void setMode(ControlMode m);
    void setManualPosition(float pos);
    void setSpeedSetpoint(float speed);
    void setForceSetpoint(float sp);
    void setPidTunings(float kp, float ki, float kd);
    void setMaxSpeed(float max);
    void update(float dt);
    float getForce();
};

#endif