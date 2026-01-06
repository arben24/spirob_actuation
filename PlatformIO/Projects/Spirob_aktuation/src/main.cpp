#include <Arduino.h>
#include <Wire.h>
#include "ForceSensorHX711.h"
#include "ForceSensorAnu78025.h"
#include "PidController.h"
#include "DistanceSensorVL6180X.h"
#include "MotorController.h"
#include "SafetyManager.h"
#include "ConfigManager.h"

const uint8_t NUM_ACTUATORS = 2;
const uint8_t SERVO_IDS[NUM_ACTUATORS] = {1, 2};

MotorController* motors[NUM_ACTUATORS];
ForceSensor* forceSensors[NUM_ACTUATORS];
DistanceSensorVL6180X* distanceSensors[NUM_ACTUATORS];
SafetyManager safetyManager;
ConfigManager configManager;

enum CalibrationState { CAL_IDLE, CAL_TARE, CAL_WAIT_WEIGHT, CAL_COMPUTE };
CalibrationState calStates[NUM_ACTUATORS] = {CAL_IDLE};
float knownWeights[NUM_ACTUATORS] = {0.0};

void setup() {
    Serial.begin(115200);
    Wire.begin();

    Serial2.begin(1000000, SERIAL_8N1, 16, 17); // Servo serial

    for (uint8_t i = 0; i < NUM_ACTUATORS; i++) {
        motors[i] = new MotorController(SERVO_IDS[i], &Serial2);
        distanceSensors[i] = new DistanceSensorVL6180X(i);
        distanceSensors[i]->begin();

        // Load config and create force sensor
        ForceSensorConfig fsConfig = configManager.loadForceSensorConfig(i);
        if (fsConfig.sensorType == FORCE_HX711) {
            forceSensors[i] = new ForceSensorHX711(fsConfig.hx711DataPin, fsConfig.hx711ClockPin, fsConfig.hx711Gain);
        } else if (fsConfig.sensorType == FORCE_ANU78025) {
            forceSensors[i] = new ForceSensorAnu78025(fsConfig.i2cAddress, fsConfig.multiplexerChannel);
        } else {
            forceSensors[i] = nullptr;
        }
        if (forceSensors[i]) {
            forceSensors[i]->begin();
            forceSensors[i]->setScale(fsConfig.scaleFactor);
        }
        motors[i]->setForceSensor(forceSensors[i]);
        motors[i]->setDistanceSensor(distanceSensors[i]);

        float kp, ki, kd;
        configManager.loadPidTunings(i, kp, ki, kd);
        motors[i]->setPidTunings(kp, ki, kd);

        safetyManager.setMotor(i, motors[i]);
        safetyManager.setLimits(i, fsConfig.nominalLoad * 1.2, 20.0, 65.0); // Example limits
    }

    Serial.println("System initialized");
}

void processSerialCommand(String cmd) {
    cmd.trim();
    if (cmd.length() == 0) return;

    if (cmd.startsWith("mode:")) {
        // mode:<actuator>:<mode> e.g. mode:0:pid_force
        int first = cmd.indexOf(':');
        int second = cmd.indexOf(':', first + 1);
        uint8_t act = cmd.substring(first + 1, second).toInt();
        String modeStr = cmd.substring(second + 1);
        MotorMode mode;
        if (modeStr == "manual") mode = MODE_MANUAL_POSITION;
        else if (modeStr == "pid_force") mode = MODE_PID_FORCE;
        else if (modeStr == "wheel") mode = MODE_WHEEL;
        else return;
        motors[act]->setMode(mode);
        Serial.println("OK: mode set");
    } else if (cmd.startsWith("setpoint:")) {
        // setpoint:<actuator>:<value>
        int first = cmd.indexOf(':');
        int second = cmd.indexOf(':', first + 1);
        uint8_t act = cmd.substring(first + 1, second).toInt();
        float val = cmd.substring(second + 1).toFloat();
        motors[act]->setForceSetpoint(val);
        Serial.println("OK: setpoint set");
    } else if (cmd.startsWith("cal_tare:")) {
        uint8_t act = cmd.substring(9).toInt();
        if (forceSensors[act]) {
            forceSensors[act]->tare();
            calStates[act] = CAL_TARE;
            Serial.println("OK: tare done");
        }
    } else if (cmd.startsWith("cal_weight:")) {
        // cal_weight:<actuator>:<weight>
        int colon = cmd.indexOf(':');
        uint8_t act = cmd.substring(colon + 1, cmd.indexOf(':', colon + 1)).toInt();
        float weight = cmd.substring(cmd.lastIndexOf(':') + 1).toFloat();
        knownWeights[act] = weight;
        calStates[act] = CAL_WAIT_WEIGHT;
        Serial.println("OK: weight set, place load and send cal_save");
    } else if (cmd.startsWith("cal_save:")) {
        uint8_t act = cmd.substring(9).toInt();
        if (calStates[act] == CAL_WAIT_WEIGHT && forceSensors[act]) {
            long raw = forceSensors[act]->readRaw();
            ForceSensorConfig config = configManager.loadForceSensorConfig(act);
            config.offset = raw; // Assuming tare was done
            config.scaleFactor = knownWeights[act] / (raw - config.offset);
            configManager.saveForceSensorConfig(act, config);
            forceSensors[act]->setScale(config.scaleFactor);
            calStates[act] = CAL_IDLE;
            Serial.println("OK: calibration saved");
        }
    } else if (cmd == "alarm_reset") {
        if (safetyManager.resetAlarm()) {
            Serial.println("OK: alarm reset");
        } else {
            Serial.println("ERR: conditions not safe");
        }
    }
}

void loop() {
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        processSerialCommand(cmd);
    }

    uint32_t now = millis();

    // Update sensors
    for (uint8_t i = 0; i < NUM_ACTUATORS; i++) {
        if (forceSensors[i]) forceSensors[i]->update();
        if (distanceSensors[i]) distanceSensors[i]->update();
    }

    // Check safety
    safetyManager.checkLimits();

    // Update motors if not in alarm
    if (safetyManager.getState() == STATE_NORMAL) {
        for (uint8_t i = 0; i < NUM_ACTUATORS; i++) {
            motors[i]->update(now / 1000.0);
        }
    }

    // Report every 100ms
    static uint32_t lastReport = 0;
    if (now - lastReport >= 100) {
        Serial.print(now);
        for (uint8_t i = 0; i < NUM_ACTUATORS; i++) {
            Serial.print(",");
            Serial.print(motors[i]->getForce());
            Serial.print(",");
            Serial.print(motors[i]->getDistance());
            Serial.print(",");
            Serial.print(motors[i]->getPosition());
        }
        Serial.print(",");
        Serial.print(safetyManager.getState() == STATE_NORMAL ? "NORMAL" : "ALARM");
        Serial.println();
        lastReport = now;
    }

    delay(10);
}