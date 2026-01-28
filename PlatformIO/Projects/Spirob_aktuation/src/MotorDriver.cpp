#include "MotorDriver.h"

MotorDriver::MotorDriver(uint8_t id, HardwareSerial* serial)
    : servoId(id), servo(0), serial(serial), currentMode(MODE_SERVO_POSITION) {
    servo.pSerial = serial;
}

void MotorDriver::setReverseDirection(bool reverse) {
    directionFactor = reverse ? -1 : 1;
}

void MotorDriver::setMode(DriverMode mode) {
    if (currentMode == mode) return; // Already in this mode
    
    currentMode = mode;
    if (mode == MODE_WHEEL) {
        servo.WheelMode(servoId);
    } else if (mode == MODE_SERVO_POSITION) {

    }
}

DriverMode MotorDriver::getMode() {
    return currentMode;
}

void MotorDriver::setPosition(int16_t position) {
    if (currentMode != MODE_SERVO_POSITION) {
        Serial.print("Warning: Motor ");
        Serial.print(servoId);
        Serial.println(" not in SERVO_POSITION mode. Call setMode(MODE_SERVO_POSITION) first.");
        return;
    }
    servo.WritePosEx(servoId, position, 0, 0);
}

void MotorDriver::setSpeed(int16_t speed) {
    if (currentMode != MODE_WHEEL) {
        Serial.print("Warning: Motor ");
        Serial.print(servoId);
        Serial.println(" not in WHEEL mode. Call setMode(MODE_WHEEL) first.");
        return;
    }
    servo.WriteSpe(servoId, speed * directionFactor, 0);
}

void MotorDriver::stop() {
    servo.WriteSpe(servoId, 0, 0);
}

int16_t MotorDriver::getPosition() {
    servo.FeedBack(servoId);
    return servo.ReadPos(servoId);
}

int16_t MotorDriver::getLoad() {
    servo.FeedBack(servoId);
    return servo.ReadLoad(servoId);
}

int MotorDriver::getSpeed() {
    servo.FeedBack(servoId);
    return directionFactor * servo.ReadSpeed(servoId);
}

int MotorDriver::getTemperature() {
    servo.FeedBack(servoId);
    return servo.ReadTemper(servoId);
}

int MotorDriver::getVoltage() {
    servo.FeedBack(servoId);
    return servo.ReadVoltage(servoId);
}

int MotorDriver::getCurrent() {
    servo.FeedBack(servoId);
    return servo.ReadCurrent(servoId);
}

int MotorDriver::getMove() {
    servo.FeedBack(servoId);
    return servo.ReadMove(servoId);
}

