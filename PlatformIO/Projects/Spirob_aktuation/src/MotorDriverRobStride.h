#ifndef MOTOR_DRIVER_ROBSTRIDE_H
#define MOTOR_DRIVER_ROBSTRIDE_H

#include <Arduino.h>
#include "IMotorDriver.h"
#include "TWAI_CAN_MI_Motor.h"

// Drives a RobStride/Xiaomi CyberGear motor over CAN (see TWAI_CAN_MI_Motor.h).
// The motor stays in CTRL_MODE (MIT frame) for its whole life; setMode() only
// changes which fields of the MIT frame are driven by setPosition()/setSpeed():
//   MODE_SERVO_POSITION -> position hold: kp=holdKp, kd=holdKd, torque=0, speed=0
//   MODE_WHEEL           -> speed command: kp=0, kd=speedKd, torque=0, position=0
// (Force control loop confirmed: PID output is a speed target, tracked via the
// kd damping term of the MIT frame - not a torque feed-forward.)
//
// getPosition()/getSpeed() never touch the CAN bus: poll() decodes whatever
// status frame is queued and caches it, mirroring ForceSensor::update()+getForce().
// Call poll() once per loop iteration for each motor.
//
// Units: the interface is int16_t for drop-in compatibility with the Feetech
// MotorDriver/ForceControlLoop, so position/speed are passed as centi-rad and
// centi-rad/s (value/100.0f = physical rad or rad/s).
class MotorDriverRobStride : public IMotorDriver {
private:
    MI_Motor_ motor;
    uint8_t motorId;
    DriverMode currentMode;
    int8_t directionFactor = 1;
    bool hasData = false;
    unsigned long lastFeedbackRequest = 0;

    float holdKp, holdKd;   // MIT gains while holding a position (MODE_SERVO_POSITION)
    float speedKd;          // MIT damping gain while tracking a speed target (MODE_WHEEL)
    float torqueLimitNm;
    float currentLimitA;

    void sendControlFrame(float torque, float positionRad, float speedRadS, float kp, float kd);

public:
    MotorDriverRobStride(uint8_t id, float holdKp = 10.0f, float holdKd = 1.0f,
                         float speedKd = 1.0f, float torqueLimitNm = 6.0f, float currentLimitA = 10.0f);

    // Runs the mandatory init sequence (id -> zero -> CTRL_MODE -> limits -> enable)
    // and waits for a first status frame. Returns false if the motor never answers.
    bool begin(uint32_t timeoutMs = 500);

    // Pumps the CAN RX queue and refreshes cached telemetry. Must be called every
    // loop iteration regardless of mode - the motor only replies to received frames,
    // so this also pings Motor_Request_Feedback() on an interval when nothing else
    // is being sent (e.g. while stopped) to keep getPosition()/getSpeed() fresh.
    void poll();

    void setMode(DriverMode mode) override;
    DriverMode getMode() override;
    void setPosition(int16_t position) override;  // centi-rad
    void setSpeed(int16_t speed) override;         // centi-rad/s
    void stop() override;
    int16_t getPosition() override;                // centi-rad, cached
    int getSpeed() override;                        // centi-rad/s, cached
    void setReverseDirection(bool reverse) override;

    // Extra real telemetry (not part of IMotorDriver, only genuinely available on
    // this motor - do not add getVoltage()/getCurrent()/getMove() here, the
    // underlying driver has no working generic parameter read for those).
    float getTorqueNm();
    float getTemperatureC();
    bool hasFault();

    // Cached mechanical angle in rad, full float precision (cur_angle wraps at
    // +-4*PI - see main_SystemIdentification.cpp's calculateRopeLength() for the
    // unwrap logic). Prefer this over getPosition() for anything that integrates
    // position over time; getPosition()'s centi-rad rounding accumulates drift.
    float getPositionRad();
};

#endif
