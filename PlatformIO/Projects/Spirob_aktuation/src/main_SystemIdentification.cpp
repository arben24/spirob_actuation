#include <Arduino.h>
#include <Wire.h>
#include "MotorDriver.h"
#include "ForceSensorAnu78025.h"

#define MOTOR_1_ID                2
#define MOTOR_1_REVERSE_DIRECTION  false
#define MOTOR_1_SENSOR_TYPE       FORCE_ANU78025
#define ANU78025_I2C_ADDR         0x2A
#define ANU78025_MUX_CHANNEL      2
#define FORCE_OFFSET_1            520000L
#define FORCE_SCALE_1             423.11273f

#define WINCH_DIAMETER_MM         44.0f  // Winch drum diameter in mm; configurable for different drum sizes

#define NUM_ACTUATORS             2
#define MOTOR_RX_PIN              16
#define MOTOR_TX_PIN              17
#define DEFAULT_MAX_SPEED         3000

// ============================================================================
// ROPE LENGTH CALCULATION (precomputed constants for efficiency)
// ============================================================================

// Precompute mm per step: circumference / 4096 steps
constexpr float MM_PER_STEP = (3.14159265358979323846f * WINCH_DIAMETER_MM) / 4096.0f;

// Global rope length tracking (cumulative, supports multiple rotations)
volatile float totalRopeLength_mm = 0.0f;
volatile int prevMotorPos = 0;

// Force limit for step response safety
volatile float maxTargetForce_N = 100.0f;

HardwareSerial servoSerial(2);              // Serial2 for servo
MotorDriver* motor;
ForceSensor* sensor;

struct __attribute__((packed)) status{
    uint32_t timestamp_us;
    float tendon_force;
    float ropeLength_mm;
};

volatile status currentStatus;

const uint8_t header[2] = {0xAA, 0x55};
const uint8_t step_end_header[2] = {0xBB, 0x66};

// End reason codes
//const uint8_t END_REASON_FORCE = 0x01;
//const uint8_t END_REASON_TIME = 0x02;
//const uint8_t END_REASON_MANUAL = 0x03;

// ============================================================================
// ROPE LENGTH CALCULATION (wrap-around aware, cumulative)
// ============================================================================

inline __attribute__((always_inline)) float calculateRopeLength(int currentMotorPos) {
    // Calculate delta with wrap-around handling (4096 encoder steps per rotation)
    int16_t deltaSteps = (int16_t)(currentMotorPos - prevMotorPos);
    
    // Handle encoder wrap (optimal wrap detection)
    if (deltaSteps > 2047) {
        deltaSteps -= 4096;
    } else if (deltaSteps < -2047) {
        deltaSteps += 4096;
    }
    
    // Accumulate rope length (mm)
    totalRopeLength_mm += deltaSteps * MM_PER_STEP;
    
    // Update previous position
    prevMotorPos = currentMotorPos;
    
    return totalRopeLength_mm;
}

void sendStepEndMessage() {
    Serial.write(step_end_header, sizeof(step_end_header));
    //Serial.write(&reason, 1);
    delay(200);  // Ensure message is sent before any new data
}

void printStatus_binary() {
    currentStatus.timestamp_us = micros();
    currentStatus.tendon_force = sensor->getForce();
    int currPos = motor->getPosition();
    currentStatus.ropeLength_mm = calculateRopeLength(currPos);

    Serial.write(header, sizeof(header));
    Serial.write((uint8_t*)&currentStatus, sizeof(currentStatus));
    }

void printStatus_human() {
    currentStatus.timestamp_us = micros();
    currentStatus.tendon_force = sensor->getForce();
    int currPos = motor->getPosition();
    currentStatus.ropeLength_mm = calculateRopeLength(currPos);
    Serial.printf("Time = %lu | Force= %.3f N | RopeLen= %.1f mm | MaxF= %.1f N | size=%lu\n", currentStatus.timestamp_us, currentStatus.tendon_force, currentStatus.ropeLength_mm, maxTargetForce_N, sizeof(currentStatus));
}

void print_timing_analysis() {
    uint32_t currentTime = micros();
    currentStatus.timestamp_us = micros();
    Serial.printf("timestamp:%lu,", micros() - currentStatus.timestamp_us);
    currentTime = micros();
    currentStatus.tendon_force = sensor->getForce();
    Serial.printf("force_reading:%lu,", micros() - currentTime);
    currentTime = micros();
    int motorPos = motor->getPosition();
    Serial.printf("position_reading:%lu,", micros() - currentTime);
    currentTime = micros();
    currentStatus.ropeLength_mm = calculateRopeLength(motorPos);
    Serial.printf("rope_calc:%lu,", micros() - currentTime);
    currentTime = micros();
    int motorSpeed = motor->getSpeed();
    Serial.printf("speed_reading:%lu \n", micros() - currentTime);
}

void stepResponse_forceLimited(int speed, uint32_t max_duration_ms) {
    motor->setMode(MODE_WHEEL);
    unsigned long start_time = millis();

    motor->setSpeed(speed);

    while (millis() - start_time < max_duration_ms) {
        sensor->update();
        float current_force = sensor->getForce();
        
        // Check force limit
        if (current_force >= maxTargetForce_N) {
            motor->setSpeed(0);
            delay(10);
            //Serial.println("step_end:force_limit");
            sendStepEndMessage();
            //Serial.println("STEP_END: Force limit reached");
            return;
        }
        
        printStatus_binary();
        //delayMicroseconds(100);
    }
    
    motor->setSpeed(0);
    delay(10);
    //sendStepEndMessage(END_REASON_TIME);
    //Serial.println("STEP_END: Duration limit reached");
    sendStepEndMessage();
}

void setup(){
    Serial.begin(460800);
    delay(1000);
    
    Wire.begin();  // I2C mit 400 kHz
    Wire.setClock(400000);  // I2C mit 400 kHz
    
    servoSerial.begin(1000000, SERIAL_8N1, MOTOR_RX_PIN, MOTOR_TX_PIN);
    delay(100);

    Serial.println("--- Motor 1 Initialization ---");
    
    Serial.print("MotorDriver ID 2...");
    motor = new MotorDriver(MOTOR_1_ID, &servoSerial);
    delay(50);
    int pos = motor->getPosition();
    if (pos == -1) {
        Serial.println(" FAILED");
        while (1) delay(1000);
    }
    motor->setReverseDirection(MOTOR_1_REVERSE_DIRECTION);
    Serial.println(" OK");
    
    // Initialize rope length tracking
    prevMotorPos = motor->getPosition();
    totalRopeLength_mm = 0.0f;

    //========================================================================
    // Initialize Force Sensor
    //========================================================================

    Serial.print("ForceSensorAnu78025 (I2C 0x2A, Mux Ch 2)...");
    sensor = new ForceSensorAnu78025(ANU78025_I2C_ADDR, ANU78025_MUX_CHANNEL, NAU7802_SPS_320);
    int retryCount = 0;
    while (!sensor->begin()) {
        Serial.print(".");
        delay(500);
        retryCount++;
        if (retryCount > 10) {
            Serial.println(" FAILED");
            while (1) delay(1000);
        }
    }
    sensor->setCalibration(FORCE_OFFSET_1, FORCE_SCALE_1);
    
    Serial.println(" OK");
}

void processSerialCommands() {
    while (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        
        if (cmd.startsWith("step ")) {
            int spaceIdx = cmd.indexOf(' ', 5);
            float speed = cmd.substring(5, spaceIdx).toFloat();
            float duration = cmd.substring(spaceIdx + 1).toFloat();
            stepResponse_forceLimited((int)speed, (uint32_t)duration);
            Serial.println("Force-limited step completed");
        }
        else if (cmd.startsWith("f ")) {
            maxTargetForce_N = cmd.substring(2).toFloat();
            Serial.printf("Max force set to %.2f N\n", maxTargetForce_N);
        }
        else if (cmd == "r") {
            Serial.println("Returning to rope length 0...");
            motor->setMode(MODE_WHEEL);
            
            // Bestimme Richtung + ABSOLUTE Speed
            float target_length = 0.0f;
            float tolerance_mm = 0.2f;  // Präzision ±0.5mm
            
            // Richtung: Rückwärts wenn zu lang
            int return_speed = (totalRopeLength_mm > 0) ? -800 : 800;  // Höhere Speed
            motor->setSpeed(return_speed);
            
            unsigned long timeout_ms = millis() + 30000;  // 30s Timeout
            
            while (abs(totalRopeLength_mm - target_length) > tolerance_mm && millis() < timeout_ms) {
                sensor->update();
                int curr_pos = motor->getPosition();
                calculateRopeLength(curr_pos);
                
                // Dynamische Speed-Reduktion nahe Target
                float error_mm = abs(totalRopeLength_mm - target_length);
                if (error_mm < 5.0f) {
                    motor->setSpeed(return_speed / 10);  // Finale Präzision
                    printStatus_binary();
                }
                
                printStatus_binary();
                //delayMicroseconds(1667);  // ~600Hz
            }
            
            motor->setSpeed(0);  // HARTE STOP
            
            // HARDCODE Nullstellung (sicherer als totalRopeLength_mm=0!)
            totalRopeLength_mm = 0.0f;
            prevMotorPos = motor->getPosition();
            
            Serial.printf("✅ Home OK ");
        }

        else if (cmd == "stop") {
            motor->setSpeed(0);
            //sendStepEndMessage();
            Serial.println("STEP_END: Manual stop");
        }
        else if (cmd == "n") {
            totalRopeLength_mm = 0.0f;
            prevMotorPos = motor->getPosition();
            Serial.println("Rope length nulled");
        }
        else {
            Serial.println("Unbekannter Befehl");
        }
    }
}

void loop(){
    processSerialCommands();
    printStatus_binary();
    //printStatus_human();
    //print_timing_analysis();
    sensor->update();  // ForceSensorAnu78025 benötigt regelmäßige Updates, um die Daten zu aktualisieren
}

