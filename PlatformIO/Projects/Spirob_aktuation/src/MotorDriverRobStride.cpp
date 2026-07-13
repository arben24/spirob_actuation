#include "MotorDriverRobStride.h"

MotorDriverRobStride::MotorDriverRobStride(uint8_t id, float holdKp, float holdKd,
                                           float speedKd, float torqueLimitNm, float currentLimitA)
    : motorId(id), currentMode(MODE_SERVO_POSITION),
      holdKp(holdKp), holdKd(holdKd), speedKd(speedKd),
      torqueLimitNm(torqueLimitNm), currentLimitA(currentLimitA) {}

void MotorDriverRobStride::sendControlFrame(float torque, float positionRad, float speedRadS, float kp, float kd) {
    motor.Motor_ControlMode(torque, positionRad, speedRadS, kp, kd);
}

bool MotorDriverRobStride::begin(uint32_t timeoutMs) {
    motor.Motor_Con_Init(motorId);
    delay(20);

    // Deliberately does NOT call Motor_Set_Zero() here (unlike the vendor's
    // usual init order) and does NOT send a position-hold frame below: neither
    // main.cpp nor main_SystemIdentification.cpp depends on an absolute
    // position reference across reboots (system-id's rope-length tracking
    // seeds its own zero from whatever position is read right after begin()).
    // Re-zeroing + holding at that fresh zero with a stiff kp is exactly what
    // made the motor visibly snap to a fixed "start position" on every boot.

    motor.Change_Mode(CTRL_MODE);
    delay(20);
    motor.Set_SingleParameter(LIMIT_TORQUE, torqueLimitNm);
    delay(20);
    motor.Set_SingleParameter(LIMIT_CUR, currentLimitA);
    delay(20);

    motor.Motor_Enable();
    delay(50);

    // Fully passive (torque=0, kp=0, kd=0): no position/speed target, so the
    // motor just stays wherever it physically is. It only starts actively
    // holding/moving once setMode()/setSpeed() is called.
    sendControlFrame(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

    unsigned long start = millis();
    while (millis() - start < timeoutMs) {
        motor.Motor_Request_Feedback();
        uint8_t err = motor.Motor_Data_Updata(20);
        if (err == 0) {
            hasData = true;
            return true;
        }
    }
    return false;
}

void MotorDriverRobStride::poll() {
    for (int attempt = 0; attempt < 3; attempt++) {
        uint8_t err = motor.Motor_Data_Updata(0);
        if (err == 2) {
            break; // nothing pending
        }
        if (err == 0) {
            hasData = true;
            break;
        }
        // err == 1: frame belonged to the other motor sharing the bus, retry
    }

    // The motor only replies to a received frame, so while stopped (no MIT
    // frames going out) telemetry would otherwise go stale.
    unsigned long now = millis();
    if (now - lastFeedbackRequest >= 100) {
        motor.Motor_Request_Feedback();
        lastFeedbackRequest = now;
    }
}

void MotorDriverRobStride::setMode(DriverMode mode) {
    if (currentMode == mode) return;
    currentMode = mode;

    if (mode == MODE_SERVO_POSITION) {
        // Hold the last known position rather than coasting at the last speed command.
        sendControlFrame(0.0f, motor.motor_rx_data.cur_angle, 0.0f, holdKp, holdKd);
    } else if (mode == MODE_WHEEL) {
        sendControlFrame(0.0f, 0.0f, 0.0f, 0.0f, speedKd);
    }
}

DriverMode MotorDriverRobStride::getMode() {
    return currentMode;
}

void MotorDriverRobStride::setPosition(int16_t position) {
    if (currentMode != MODE_SERVO_POSITION) {
        Serial.print("Warning: Motor ");
        Serial.print(motorId);
        Serial.println(" not in SERVO_POSITION mode. Call setMode(MODE_SERVO_POSITION) first.");
        return;
    }
    sendControlFrame(0.0f, position / 100.0f, 0.0f, holdKp, holdKd);
}

void MotorDriverRobStride::setSpeed(int16_t speed) {
    if (currentMode != MODE_WHEEL) {
        Serial.print("Warning: Motor ");
        Serial.print(motorId);
        Serial.println(" not in WHEEL mode. Call setMode(MODE_WHEEL) first.");
        return;
    }
    float speedRadS = (directionFactor * speed) / 100.0f;
    sendControlFrame(0.0f, 0.0f, speedRadS, 0.0f, speedKd);
}

void MotorDriverRobStride::stop() {
    motor.Motor_Reset(); // disables torque - misleadingly named in the vendor driver, not a reboot
}

int16_t MotorDriverRobStride::getPosition() {
    return (int16_t)(motor.motor_rx_data.cur_angle * 100.0f);
}

int MotorDriverRobStride::getSpeed() {
    return (int)(directionFactor * motor.motor_rx_data.cur_speed * 100.0f);
}

void MotorDriverRobStride::setReverseDirection(bool reverse) {
    directionFactor = reverse ? -1 : 1;
}

float MotorDriverRobStride::getTorqueNm() {
    return motor.motor_rx_data.cur_torque;
}

float MotorDriverRobStride::getTemperatureC() {
    return motor.motor_rx_data.cur_temp;
}

bool MotorDriverRobStride::hasFault() {
    return hasData && motor.motor_rx_data.err_sta;
}

float MotorDriverRobStride::getPositionRad() {
    return motor.motor_rx_data.cur_angle;
}
