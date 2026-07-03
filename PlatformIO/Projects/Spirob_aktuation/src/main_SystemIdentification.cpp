/*
 * RobStride/CyberGear CAN motors (see MotorDriverRobStride.h) - same control
 * mapping as env:production: motor stays in CTRL_MODE, force-PID output is a
 * speed target tracked via the MIT frame's kd damping term.
 */

#include <Arduino.h>
#include <Wire.h>
#include "MotorDriverRobStride.h"
#include "ForceSensorAnu78025.h"
#include "ForceControlLoop.h"
#include "EndStopSwitch.h"

// ============================================================================
// HARDWARE CONFIGURATION - Motor 0 (Servo ID 1)
// ============================================================================

#define MOTOR_0_ID                1
#define MOTOR_0_REVERSE_DIRECTION true
#define MOTOR_0_MUX_CHANNEL       0
#define FORCE_OFFSET_0            153859L
#define FORCE_SCALE_0             105.13830f
#define ENDSTOP_0_PIN             27      // Endstop for motor 0

// ============================================================================
// HARDWARE CONFIGURATION - Motor 1 (Servo ID 2)
// ============================================================================

#define MOTOR_1_ID                2
#define MOTOR_1_REVERSE_DIRECTION false   
#define MOTOR_1_MUX_CHANNEL       1
#define FORCE_OFFSET_1            -108743L  
#define FORCE_SCALE_1             104.81184f
#define ENDSTOP_1_PIN             26      // Endstop for motor 1

// ============================================================================
// HARDWARE CONFIGURATION - Common
// ============================================================================

#define ANU78025_I2C_ADDR         0x2A
#define WINCH_DIAMETER_MM         44.0f
#define NUM_ACTUATORS             2

// PID output / motor speed target, in centi-rad/s (see MotorDriverRobStride.h
// unit convention: value/100.0f = rad/s). Bounds the force-PID's own authority;
// the "v"/"step" commands below send whatever raw centi-rad/s the user asks
// for, independent of this cap. Placeholder - not yet tuned on hardware.
#define DEFAULT_MAX_SPEED         300
// Slow, deliberate approach speed for homing - independent constant, not a
// fraction of DEFAULT_MAX_SPEED. Placeholder - verify on hardware before trusting.
#define HOMING_SPEED              30

// RobStride MIT-frame gains, shared by both motors (placeholders, see main.cpp).
#define MOTOR_HOLD_KP             10.0f
#define MOTOR_HOLD_KD             1.0f
#define MOTOR_SPEED_KD            1.0f
#define MOTOR_TORQUE_LIMIT_NM     6.0f
#define MOTOR_CURRENT_LIMIT_A     10.0f

// ============================================================================
// ROPE LENGTH CALCULATION (precomputed constants for efficiency)
// ============================================================================

// mm of rope per radian of drum rotation: arc length s = radius * angle
constexpr float MM_PER_RAD = WINCH_DIAMETER_MM / 2.0f;

// Per-motor state arrays
volatile float totalRopeLength_mm[NUM_ACTUATORS] = {0.0f, 0.0f};
volatile float prevMotorPos_rad[NUM_ACTUATORS]    = {0.0f, 0.0f};
const bool     reverseDirection[NUM_ACTUATORS]    = {MOTOR_0_REVERSE_DIRECTION, MOTOR_1_REVERSE_DIRECTION};

// Force limit for step response safety (shared across both motors)
volatile float maxTargetForce_N = 100.0f;

// Endstop per motor
EndStopSwitch endstop0(ENDSTOP_0_PIN);
EndStopSwitch endstop1(ENDSTOP_1_PIN);
EndStopSwitch* endstops[NUM_ACTUATORS] = {&endstop0, &endstop1};

// ============================================================================
// FORCE CONTROL (PID regulation per motor)
// ============================================================================

// Re-derived for the new +-300 centi-rad/s output range (previously +-3000 raw
// servo-speed steps) - starting point only, needs field retuning against the
// actual RobStride motors, same as main.cpp's DEFAULT_KP/KI/KD.
#define DEFAULT_KP              5.0f
#define DEFAULT_KI              0.05f
#define DEFAULT_KD              1.0f
#define DEFAULT_PID_SAMPLE_TIME 10.0f   // ms

ForceControlLoop* controlLoops[NUM_ACTUATORS];
bool isForceControlRunning[NUM_ACTUATORS] = {false, false};
float forceSetpoint_N[NUM_ACTUATORS] = {0.0f, 0.0f};

MotorDriverRobStride* motors[NUM_ACTUATORS];
ForceSensor*          sensors[NUM_ACTUATORS];

// ============================================================================
// BINARY TELEMETRY PROTOCOL (both motors in one packet)
// ============================================================================
// Python: struct.unpack('<I ff ff', data)  →  (ts_us, f0, f1, rope0, rope1)
// Total: 2 header + 20 payload = 22 bytes per packet

struct __attribute__((packed)) status_dual {
    uint32_t timestamp_us;
    float tendon_force[NUM_ACTUATORS];
    float ropeLength_mm[NUM_ACTUATORS];
};

volatile status_dual currentStatus;

const uint8_t header[2]          = {0xAA, 0x55};
const uint8_t step_end_header[2] = {0xBB, 0x66};

// End reason codes
//const uint8_t END_REASON_FORCE = 0x01;
//const uint8_t END_REASON_TIME = 0x02;
//const uint8_t END_REASON_MANUAL = 0x03;

// ============================================================================
// ROPE LENGTH CALCULATION (wrap-around aware, cumulative, per motor)
// ============================================================================

inline float calculateRopeLength(int idx, float currentMotorPosRad) {
    float deltaRad = currentMotorPosRad - prevMotorPos_rad[idx];
    if (reverseDirection[idx]) deltaRad = -deltaRad;

    // cur_angle wraps at +-4*PI (16-bit fixed-point register behind it, see
    // TWAI_CAN_MI_Motor.cpp's INT2ANGLE) - same idea as the old Feetech
    // 4096-steps/rev wrap, just a bigger span (2 mechanical turns instead of 1).
    if (deltaRad > 4.0f * PI)       deltaRad -= 8.0f * PI;
    else if (deltaRad < -4.0f * PI) deltaRad += 8.0f * PI;

    totalRopeLength_mm[idx] += deltaRad * MM_PER_RAD;
    prevMotorPos_rad[idx] = currentMotorPosRad;
    return totalRopeLength_mm[idx];
}

// ============================================================================
// BINARY OUTPUT
// ============================================================================

void sendStepEndMessage(int idx) {
    Serial.write(step_end_header, sizeof(step_end_header));
    uint8_t motor_byte = (uint8_t)idx;
    Serial.write(&motor_byte, 1);        // 1 extra byte: which motor finished
    delay(200);
}

void printStatus_binary() {
    currentStatus.timestamp_us = micros();
    for (int i = 0; i < NUM_ACTUATORS; i++) {
        currentStatus.tendon_force[i]  = sensors[i]->getForce();
        currentStatus.ropeLength_mm[i] = calculateRopeLength(i, motors[i]->getPositionRad());
    }
    Serial.write(header, sizeof(header));
    Serial.write((uint8_t*)&currentStatus, sizeof(currentStatus));
}

void printStatus_human() {
    currentStatus.timestamp_us = micros();
    for (int i = 0; i < NUM_ACTUATORS; i++) {
        currentStatus.tendon_force[i]  = sensors[i]->getForce();
        currentStatus.ropeLength_mm[i] = calculateRopeLength(i, motors[i]->getPositionRad());
    }
    Serial.printf("Time=%lu | M0: F=%.3f/%.1fN L=%.1fmm %s | M1: F=%.3f/%.1fN L=%.1fmm %s | FlLim=%.1fN\n",
        currentStatus.timestamp_us,
        currentStatus.tendon_force[0], forceSetpoint_N[0], currentStatus.ropeLength_mm[0],
        isForceControlRunning[0] ? "PID" : "OFF",
        currentStatus.tendon_force[1], forceSetpoint_N[1], currentStatus.ropeLength_mm[1],
        isForceControlRunning[1] ? "PID" : "OFF",
        maxTargetForce_N);
}

// ============================================================================
// STEP RESPONSE (force-limited, single motor, blocking)
// ============================================================================

void stepResponse_forceLimited(int idx, int speed, uint32_t max_duration_ms) {
    motors[idx]->setMode(MODE_WHEEL);
    motors[idx]->setSpeed(speed);
    unsigned long start_time = millis();

    while (millis() - start_time < max_duration_ms) {
        for (int i = 0; i < NUM_ACTUATORS; i++) sensors[i]->update();
        for (int i = 0; i < NUM_ACTUATORS; i++) motors[i]->poll();

        // Force limit on the active motor
        if (sensors[idx]->getForce() >= maxTargetForce_N) {
            motors[idx]->setSpeed(0);
            delay(10);
            sendStepEndMessage(idx);
            return;
        }

        // Endstop limit on the active motor
        if (endstops[idx]->isRawTriggered()) {
            motors[idx]->setSpeed(0);
            delay(10);
            sendStepEndMessage(idx);
            return;
        }

        printStatus_binary();
    }

    motors[idx]->setSpeed(0);
    delay(10);
    sendStepEndMessage(idx);
}

// ============================================================================
// HOMING (Referenzfahrt, single motor, blocking)
// ============================================================================

void performHoming(int idx) {
    Serial.printf("Homing motor %d: moving toward endstop (pin %d)...\n", idx, endstops[idx]->pin());

    if (endstops[idx]->isRawTriggered()) {
        totalRopeLength_mm[idx] = 0.0f;
        prevMotorPos_rad[idx] = motors[idx]->getPositionRad();
        Serial.printf("Motor %d: already at endstop -- position zeroed\n", idx);
        return;
    }

    motors[idx]->setMode(MODE_WHEEL);
    motors[idx]->setSpeed(-HOMING_SPEED);

    const unsigned long deadline = millis() + 30000;

    while (millis() < deadline) {
        endstops[idx]->update();
        for (int i = 0; i < NUM_ACTUATORS; i++) sensors[i]->update();
        for (int i = 0; i < NUM_ACTUATORS; i++) motors[i]->poll();
        printStatus_binary();

        if (endstops[idx]->isRawTriggered()) {
            motors[idx]->setSpeed(0);
            delay(10);
            totalRopeLength_mm[idx] = 0.0f;
            prevMotorPos_rad[idx] = motors[idx]->getPositionRad();
            Serial.printf("Motor %d: HOME found -- position zeroed\n", idx);
            return;
        }
    }

    motors[idx]->setSpeed(0);
    Serial.printf("Motor %d: homing timeout (30s)\n", idx);
}

// ============================================================================
// RETURN TO ZERO (single motor, blocking)
// ============================================================================

void returnToZero(int idx) {
    Serial.printf("Motor %d: returning to rope length 0...\n", idx);
    motors[idx]->setMode(MODE_WHEEL);

    float tolerance_mm = 0.2f;
    int return_speed = (totalRopeLength_mm[idx] > 0) ? -80 : 80;
    motors[idx]->setSpeed(return_speed);

    const unsigned long deadline = millis() + 30000;

    while (fabsf(totalRopeLength_mm[idx]) > tolerance_mm && millis() < deadline) {
        for (int i = 0; i < NUM_ACTUATORS; i++) sensors[i]->update();
        for (int i = 0; i < NUM_ACTUATORS; i++) motors[i]->poll();
        calculateRopeLength(idx, motors[idx]->getPositionRad());

        if (fabsf(totalRopeLength_mm[idx]) < 5.0f) {
            motors[idx]->setSpeed(return_speed / 10);
        }

        // Safety: stop if endstop triggers during return
        if (endstops[idx]->isRawTriggered()) {
            motors[idx]->setSpeed(0);
            Serial.printf("Motor %d: endstop hit during return\n", idx);
            break;
        }

        printStatus_binary();
    }

    motors[idx]->setSpeed(0);
    totalRopeLength_mm[idx] = 0.0f;
    prevMotorPos_rad[idx] = motors[idx]->getPositionRad();
    Serial.printf("Motor %d: home OK\n", idx);
}

// ============================================================================
// SETUP
// ============================================================================

void initMotor(int idx, uint8_t servoId, bool reverse) {
    Serial.printf("MotorDriverRobStride CAN ID %d...", servoId);
    motors[idx] = new MotorDriverRobStride(servoId, MOTOR_HOLD_KP, MOTOR_HOLD_KD,
                                            MOTOR_SPEED_KD, MOTOR_TORQUE_LIMIT_NM, MOTOR_CURRENT_LIMIT_A);
    if (!motors[idx]->begin()) {
        Serial.println(" FAILED");
        while (1) delay(1000);
    }
    motors[idx]->setReverseDirection(reverse);
    prevMotorPos_rad[idx] = motors[idx]->getPositionRad();
    totalRopeLength_mm[idx] = 0.0f;
    Serial.printf(" OK (pos=%.3f rad)\n", prevMotorPos_rad[idx]);
}

void initSensor(int idx, uint8_t muxChannel, long offset, float scale) {
    Serial.printf("ForceSensorAnu78025 (mux ch %d)...", muxChannel);
    sensors[idx] = new ForceSensorAnu78025(ANU78025_I2C_ADDR, muxChannel, NAU7802_SPS_320);
    int retryCount = 0;
    while (!sensors[idx]->begin()) {
        Serial.print(".");
        delay(500);
        if (++retryCount > 10) {
            Serial.println(" FAILED");
            while (1) delay(1000);
        }
    }
    sensors[idx]->setCalibration(offset, scale);
    Serial.println(" OK");
}

void setup() {
    Serial.begin(460800);
    delay(1000);

    Wire.begin();
    Wire.setClock(400000);

    Motor_CAN_Init();
    delay(200);

    // Motors
    Serial.println("=== Motor Initialization ===");
    initMotor(0, MOTOR_0_ID, MOTOR_0_REVERSE_DIRECTION);
    initMotor(1, MOTOR_1_ID, MOTOR_1_REVERSE_DIRECTION);

    // Sensors
    Serial.println("=== Sensor Initialization ===");
    initSensor(0, MOTOR_0_MUX_CHANNEL, FORCE_OFFSET_0, FORCE_SCALE_0);
    initSensor(1, MOTOR_1_MUX_CHANNEL, FORCE_OFFSET_1, FORCE_SCALE_1);

    // Endstops
    Serial.println("=== Endstop Initialization ===");
    for (int i = 0; i < NUM_ACTUATORS; i++) {
        Serial.printf("Endstop %d (pin %d)...", i, endstops[i]->pin());
        endstops[i]->begin();
        Serial.printf(" OK (%s)\n", endstops[i]->isTriggered() ? "TRIGGERED" : "open");
    }

    // Force Control Loops
    Serial.println("=== Force Control Initialization ===");
    for (int i = 0; i < NUM_ACTUATORS; i++) {
        controlLoops[i] = new ForceControlLoop(motors[i], sensors[i]);
        controlLoops[i]->setForceSetpoint(0.0f);
        controlLoops[i]->setMaxSpeed(DEFAULT_MAX_SPEED);
        controlLoops[i]->setSampleTime(DEFAULT_PID_SAMPLE_TIME);
        controlLoops[i]->setPidTunings(DEFAULT_KP, DEFAULT_KI, DEFAULT_KD);
        Serial.printf("  Motor %d: PID(%.1f, %.3f, %.1f) SampleTime=%.0fms\n", 
                      i, DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, DEFAULT_PID_SAMPLE_TIME);
    }

    Serial.println("\n=== System Ready (2 motors) ===");
    Serial.println("Commands:  f <m> <N>  |  start <m>  |  pid <m> <kp> <ki> <kd>  |  v <m> <speed>  |  step <m> <v> <ms>  |  stop [m]");
}

// ============================================================================
// COMMAND PARSER
// ============================================================================

// Parse motor index: "0", "1", or "all" (returns -1)
bool parseMotorId(const String& s, int& id) {
    if (s == "all") { id = -1; return true; }
    id = s.toInt();
    if (s.charAt(0) != '0' && id == 0) { // toInt returns 0 on parse failure
        Serial.printf("Error: motor must be 0-%d or 'all'\n", NUM_ACTUATORS - 1);
        return false;
    }
    if (id < 0 || id >= NUM_ACTUATORS) {
        Serial.printf("Error: motor must be 0-%d or 'all'\n", NUM_ACTUATORS - 1);
        return false;
    }
    return true;
}

void processSerialCommands() {
    while (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd.length() == 0) continue;

        // Tokenize: split into words
        String tokens[5];
        int nTokens = 0;
        int start = 0;
        while (nTokens < 5 && start < (int)cmd.length()) {
            int sp = cmd.indexOf(' ', start);
            if (sp == -1) sp = cmd.length();
            tokens[nTokens++] = cmd.substring(start, sp);
            start = sp + 1;
        }

        // ---- step <motor> <speed> <duration_ms> ----
        if (tokens[0] == "step" && nTokens >= 4) {
            int id;
            if (!parseMotorId(tokens[1], id) || id < 0) {
                Serial.println("Error: step requires specific motor (0 or 1)");
                continue;
            }
            isForceControlRunning[id] = false;  // Disable force ctrl during step
            int speed       = tokens[2].toInt();
            uint32_t dur_ms = (uint32_t)tokens[3].toFloat();
            stepResponse_forceLimited(id, speed, dur_ms);
            Serial.printf("Motor %d: step complete\n", id);
        }

        // ---- v <motor> <speed> ---- direct speed (disables force control)
        else if (tokens[0] == "v" && nTokens >= 3) {
            int id;
            if (!parseMotorId(tokens[1], id)) continue;
            int speed = tokens[2].toInt();
            if (id == -1) {
                for (int i = 0; i < NUM_ACTUATORS; i++) {
                    isForceControlRunning[i] = false;
                    motors[i]->setMode(MODE_WHEEL);
                    motors[i]->setSpeed(speed);
                }
                Serial.printf("All motors: speed=%d (force ctrl off)\n", speed);
            } else {
                isForceControlRunning[id] = false;
                motors[id]->setMode(MODE_WHEEL);
                motors[id]->setSpeed(speed);
                Serial.printf("Motor %d: speed=%d (force ctrl off)\n", id, speed);
            }
        }

        // ---- c <motor|all> ---- homing (disables force control)
        else if (tokens[0] == "c" && nTokens >= 2) {
            int id;
            if (!parseMotorId(tokens[1], id)) continue;
            if (id == -1) {
                for (int i = 0; i < NUM_ACTUATORS; i++) {
                    isForceControlRunning[i] = false;
                    performHoming(i);
                }
            } else {
                isForceControlRunning[id] = false;
                performHoming(id);
            }
        }

        // ---- r <motor> ---- return to zero (disables force control)
        else if (tokens[0] == "r" && nTokens >= 2) {
            int id;
            if (!parseMotorId(tokens[1], id)) continue;
            if (id == -1) {
                for (int i = 0; i < NUM_ACTUATORS; i++) {
                    isForceControlRunning[i] = false;
                    returnToZero(i);
                }
            } else {
                isForceControlRunning[id] = false;
                returnToZero(id);
            }
        }

        // ---- f <motor|all> <force_N> ---- set force setpoint per motor
        else if (tokens[0] == "f" && nTokens >= 3) {
            int id;
            if (!parseMotorId(tokens[1], id)) continue;
            float force = tokens[2].toFloat();
            if (id == -1) {
                for (int i = 0; i < NUM_ACTUATORS; i++) {
                    forceSetpoint_N[i] = force;
                    controlLoops[i]->setForceSetpoint(force);
                }
                Serial.printf("All motors: force setpoint=%.2f N\n", force);
            } else {
                forceSetpoint_N[id] = force;
                controlLoops[id]->setForceSetpoint(force);
                Serial.printf("Motor %d: force setpoint=%.2f N\n", id, force);
            }
        }

        // ---- fl <force> ---- global force limit (step response safety)
        else if (tokens[0] == "fl" && nTokens >= 2) {
            maxTargetForce_N = tokens[1].toFloat();
            Serial.printf("Max force limit set to %.2f N\n", maxTargetForce_N);
        }

        // ---- stop [motor] ---- stop motor(s) and disable force control
        else if (tokens[0] == "stop") {
            if (nTokens >= 2) {
                int id;
                if (!parseMotorId(tokens[1], id)) continue;
                if (id == -1) {
                    for (int i = 0; i < NUM_ACTUATORS; i++) {
                        motors[i]->setSpeed(0);
                        isForceControlRunning[i] = false;
                    }
                    Serial.println("All motors stopped (force ctrl off)");
                } else {
                    motors[id]->setSpeed(0);
                    isForceControlRunning[id] = false;
                    Serial.printf("Motor %d stopped (force ctrl off)\n", id);
                }
            } else {
                for (int i = 0; i < NUM_ACTUATORS; i++) {
                    motors[i]->setSpeed(0);
                    isForceControlRunning[i] = false;
                }
                Serial.println("All motors stopped (force ctrl off)");
            }
        }

        // ---- n <motor> ---- null rope length
        else if (tokens[0] == "n" && nTokens >= 2) {
            int id;
            if (!parseMotorId(tokens[1], id)) continue;
            if (id == -1) {
                for (int i = 0; i < NUM_ACTUATORS; i++) {
                    totalRopeLength_mm[i] = 0.0f;
                    prevMotorPos_rad[i] = motors[i]->getPositionRad();
                }
                Serial.println("All rope lengths nulled");
            } else {
                totalRopeLength_mm[id] = 0.0f;
                prevMotorPos_rad[id] = motors[id]->getPositionRad();
                Serial.printf("Motor %d: rope length nulled\n", id);
            }
        }

        // ---- pid <motor> <kp> <ki> <kd> ---- tune PID per motor
        else if (tokens[0] == "pid" && nTokens >= 5) {
            int id;
            if (!parseMotorId(tokens[1], id) || id < 0) {
                Serial.println("Error: pid requires specific motor (0 or 1)");
                continue;
            }
            float kp = tokens[2].toFloat();
            float ki = tokens[3].toFloat();
            float kd = tokens[4].toFloat();
            controlLoops[id]->setPidTunings(kp, ki, kd);
            Serial.printf("Motor %d: PID(%.2f, %.4f, %.2f)\n", id, kp, ki, kd);
        }

        // ---- start <motor|all> ---- start force regulation
        else if (tokens[0] == "start" && nTokens >= 2) {
            int id;
            if (!parseMotorId(tokens[1], id)) continue;
            if (id == -1) {
                for (int i = 0; i < NUM_ACTUATORS; i++) {
                    controlLoops[i]->setMode(MODE_PID_FORCE);
                    isForceControlRunning[i] = true;
                }
                Serial.printf("All motors: force control started\n");
            } else {
                controlLoops[id]->setMode(MODE_PID_FORCE);
                isForceControlRunning[id] = true;
                Serial.printf("Motor %d: force control started (setpoint=%.2f N)\n", id, forceSetpoint_N[id]);
            }
        }

        // ---- help ----
        else if (tokens[0] == "help") {
            Serial.println("\n=== Commands ===");
            Serial.println("-- Force Control --");
            Serial.println("f <m|all> <N>          Set force setpoint (e.g. 'f 0 5.0', 'f all 3')");
            Serial.println("start <m|all>          Start force regulation (PID)");
            Serial.println("pid <m> <kp> <ki> <kd> Tune PID (e.g. 'pid 0 30 0.5 0')");
            Serial.println("-- Direct Control --");
            Serial.println("v <m|all> <speed>      Direct speed (disables force ctrl)");
            Serial.println("step <m> <speed> <ms>  Step response (e.g. 'step 0 500 2000')");
            Serial.println("stop [m]               Stop motor(s) + force ctrl (default: all)");
            Serial.println("-- Referencing --");
            Serial.println("c <m|all>              Homing / reference run");
            Serial.println("r <m|all>              Return to rope length 0");
            Serial.println("n <m|all>              Null rope length tracking");
            Serial.println("-- Settings --");
            Serial.println("fl <N>                 Force limit for step response (e.g. 'fl 15')");
            Serial.println("help                   Show this help\n");
        }

        else {
            Serial.println("Unknown command (type 'help')");
        }
    }
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    // 1. Update endstop debounce state machines
    for (int i = 0; i < NUM_ACTUATORS; i++) {
        endstops[i]->update();

        // Safety: stop motor on first debounced trigger (rising edge)
        if (endstops[i]->risingEdge()) {
            motors[i]->setSpeed(0);
            isForceControlRunning[i] = false;
            Serial.printf("Motor %d: endstop triggered -- stopped\n", i);
        }
    }

    // 2. Update sensors (non-blocking ADC poll)
    for (int i = 0; i < NUM_ACTUATORS; i++) {
        sensors[i]->update();
    }

    // 2b. Drain CAN feedback frames (non-blocking)
    for (int i = 0; i < NUM_ACTUATORS; i++) {
        motors[i]->poll();
    }

    // 3. Force control loop update (PID → motor speed)
    for (int i = 0; i < NUM_ACTUATORS; i++) {
        if (isForceControlRunning[i]) {
            float output = controlLoops[i]->update();
            motors[i]->setSpeed((int16_t)output);
        }
    }

    // 4. Process serial commands
    processSerialCommands();

    // 5. Send telemetry (both motors in one packet)
    printStatus_binary();
}

