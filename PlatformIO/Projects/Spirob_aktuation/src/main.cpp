#include <Arduino.h>
#include <Wire.h>
#include "MotorDriver.h"
#include "ForceSensorHX711.h"
#include "ForceSensorAnu78025.h"
#include "ForceControlLoop.h"

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================

void printStatus();
void printHelp();

// ============================================================================
// HARDWARE CONFIGURATION - Motor 0 (HX711)

#define MOTOR_0_ID                1
#define MOTOR_0_REVERSE_DIRECTION  true
#define MOTOR_0_SENSOR_TYPE       FORCE_HX711
#define HX711_DATA_PIN_0          26
#define HX711_CLOCK_PIN_0         27
#define FORCE_OFFSET_0            270331L
#define FORCE_SCALE_0             225.64403f

// ============================================================================
// HARDWARE CONFIGURATION - Motor 1 (ANU78025)
// ============================================================================

#define MOTOR_1_ID                2
#define MOTOR_1_REVERSE_DIRECTION  false
#define MOTOR_1_SENSOR_TYPE       FORCE_ANU78025
#define ANU78025_I2C_ADDR         0x2A
#define ANU78025_MUX_CHANNEL      2
#define FORCE_OFFSET_1            520000L
#define FORCE_SCALE_1             423.11273f

// ============================================================================
// HARDWARE CONFIGURATION - Common
// ============================================================================

#define NUM_ACTUATORS             2
#define MOTOR_RX_PIN              16
#define MOTOR_TX_PIN              17
#define DEFAULT_MAX_SPEED         3000

// Default PID tunings (tunable via CLI)
#define DEFAULT_KP                30.0
#define DEFAULT_KI                0.5
#define DEFAULT_KD                0.0

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

HardwareSerial servoSerial(2);              // Serial2 for servo
MotorDriver* motors[NUM_ACTUATORS];
ForceSensor* sensors[NUM_ACTUATORS];
ForceControlLoop* controlLoops[NUM_ACTUATORS];

// ============================================================================
// CONTROL STATE
// ============================================================================

bool isRunning[NUM_ACTUATORS] = {false, false};
bool fast_print = false;
float forceSetpoint[NUM_ACTUATORS] = {0.0f, 0.0f};
unsigned long lastStatusPrint = 0;
const unsigned long STATUS_INTERVAL = 1000;  // 10Hz output

// ============================================================================
// INITIALIZATION
// ============================================================================

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Wire.begin();
    
    servoSerial.begin(1000000, SERIAL_8N1, MOTOR_RX_PIN, MOTOR_TX_PIN);
    delay(100);

    Serial.println("\n=== Spirob Aktuation - Dual Motor Force Control ===");
    Serial.println("Motor 0: Servo ID 1 (HX711)");
    Serial.println("Motor 1: Servo ID 2 (ANU78025)");
    Serial.println();

    // === Initialize Motor 0 (HX711) ===
    Serial.println("--- Motor 0 Initialization ---");
    
    Serial.print("MotorDriver ID 1...");
    motors[0] = new MotorDriver(MOTOR_0_ID, &servoSerial);
    delay(50);
    int pos = motors[0]->getPosition();
    if (pos == -1) {
        Serial.println(" FAILED");
        while (1) delay(1000);
    }
    motors[0]->setReverseDirection(MOTOR_0_REVERSE_DIRECTION); // Invert direction if needed
    Serial.println(" OK");

    Serial.print("ForceSensorHX711 (pins 12/13)...");
    sensors[0] = new ForceSensorHX711(HX711_DATA_PIN_0, HX711_CLOCK_PIN_0, 128);
    if (!sensors[0]->begin()) {
        Serial.println(" FAILED");
        while (1) delay(1000);
    }
    sensors[0]->setCalibration(FORCE_OFFSET_0, FORCE_SCALE_0);
    Serial.println(" OK");

    Serial.print("ForceControlLoop 0...");
    controlLoops[0] = new ForceControlLoop(motors[0], sensors[0]);
    controlLoops[0]->setMode(MODE_PID_FORCE);
    controlLoops[0]->setForceSetpoint(0.0f);
    controlLoops[0]->setMaxSpeed(DEFAULT_MAX_SPEED);
    controlLoops[0]->setPidTunings(DEFAULT_KP, DEFAULT_KI, DEFAULT_KD);
    controlLoops[0]->setSampleTime(100.0f);  // 100 ms sample time
    Serial.println(" OK");

    // === Initialize Motor 1 (ANU78025) ===
    Serial.println("--- Motor 1 Initialization ---");
    
    Serial.print("MotorDriver ID 2...");
    motors[1] = new MotorDriver(MOTOR_1_ID, &servoSerial);
    delay(50);
    pos = motors[1]->getPosition();
    if (pos == -1) {
        Serial.println(" FAILED");
        while (1) delay(1000);
    }
    motors[1]->setReverseDirection(MOTOR_1_REVERSE_DIRECTION);
    Serial.println(" OK");

    Serial.print("ForceSensorAnu78025 (I2C 0x2A, Mux Ch 2)...");
    sensors[1] = new ForceSensorAnu78025(ANU78025_I2C_ADDR, ANU78025_MUX_CHANNEL);
    int retryCount = 0;
    while (!sensors[1]->begin()) {
        Serial.print(".");
        delay(500);
        retryCount++;
        if (retryCount > 10) {
            Serial.println(" FAILED");
            while (1) delay(1000);
        }
    }
    sensors[1]->setCalibration(FORCE_OFFSET_1, FORCE_SCALE_1);
    Serial.println(" OK");

    Serial.print("ForceControlLoop 1...");
    controlLoops[1] = new ForceControlLoop(motors[1], sensors[1]);
    controlLoops[1]->setMode(MODE_PID_FORCE);
    controlLoops[1]->setForceSetpoint(0.0f);
    controlLoops[1]->setMaxSpeed(DEFAULT_MAX_SPEED);
    controlLoops[1]->setPidTunings(DEFAULT_KP, DEFAULT_KI, DEFAULT_KD);
    controlLoops[1]->setSampleTime(100.0f);  // 100 ms sample time
    Serial.println(" OK");

    Serial.println("\n=== System Ready ===");
    printHelp();
}

// ============================================================================
// STATUS OUTPUT
// ============================================================================

void printStatus() {
    for (int i = 0; i < NUM_ACTUATORS; i++) {
        float forceActual = sensors[i]->getForce();
        int motorPos = motors[i]->getPosition();
        int motorSpeed = motors[i]->getSpeed();
        
        Serial.printf(" M%d: Force= %.3f / %.3f N | Pos= %5d | Speed= %5d | %s",
            i, forceActual, forceSetpoint[i], motorPos, motorSpeed,
            isRunning[i] ? "RUN" : "STOP");
    }
    Serial.println();
}

void printHelp() {
    Serial.println("\n=== Commands ===");
    Serial.println("set <id> <kg>            - Set force setpoint (0-1, or 'all')");
    Serial.println("start <id>               - Start regulation (0-1, or 'all')");
    Serial.println("stop <id>                - Stop regulation (0-1, or 'all')");
    Serial.println("pid <id> <kp> <ki> <kd> - Tune PID");
    //Serial.println("tare <id>                - Tare sensor (0-1, or 'all')");
    Serial.println("status                   - Print status");
    Serial.println("help                     - Show this help");
    Serial.println();
}

// ============================================================================
// COMMAND PROCESSING
// ============================================================================

bool parseMotorId(String idStr, int& id) {
    if (idStr == "all") {
        id = -1;  // Special value for "all"
        return true;
    }
    id = idStr.toInt();
    if (id < 0 || id >= NUM_ACTUATORS) {
        Serial.print("Error: Motor ID must be 0-");
        Serial.print(NUM_ACTUATORS - 1);
        Serial.println(" or 'all'");
        return false;
    }
    return true;
}

void processCommand(String cmd) {
    cmd.trim();
    if (cmd.length() == 0) return;

    int spaceIdx = cmd.indexOf(' ');
    String token = (spaceIdx == -1) ? cmd : cmd.substring(0, spaceIdx);
    String args = (spaceIdx == -1) ? "" : cmd.substring(spaceIdx + 1);

    // === set <id> <kg> - Set force setpoint ===
    if (token == "set") {
        int firstSpace = args.indexOf(' ');
        if (firstSpace == -1) {
            Serial.println("Error: set <id> <N>");
            return;
        }
        String idStr = args.substring(0, firstSpace);
        float newSetpoint = args.substring(firstSpace + 1).toFloat();
        
        int id;
        if (!parseMotorId(idStr, id)) return;

        if (id == -1) {
            for (int i = 0; i < NUM_ACTUATORS; i++) {
                forceSetpoint[i] = newSetpoint;
                controlLoops[i]->setForceSetpoint(newSetpoint);
            }
            Serial.print("All motors: setpoint = ");
            Serial.print(newSetpoint, 3);
            Serial.println(" N");
        } else {
            forceSetpoint[id] = newSetpoint;
            controlLoops[id]->setForceSetpoint(newSetpoint);
            Serial.print("Motor ");
            Serial.print(id);
            Serial.print(": setpoint = ");
            Serial.print(newSetpoint, 3);
            Serial.println(" N");
        }
    }

    // === start <id> - Start regulation ===
    else if (token == "start") {
        int id;
        if (!parseMotorId(args, id)) return;

        if (id == -1) {
            for (int i = 0; i < NUM_ACTUATORS; i++) {
                isRunning[i] = true;
                motors[i]->setMode(MODE_WHEEL);
            }
            Serial.println("All motors: regulation started");
        } else {
            isRunning[id] = true;
            motors[id]->setMode(MODE_WHEEL);
            Serial.print("Motor ");
            Serial.print(id);
            Serial.println(": regulation started");
        }
    }

    // === stop <id> - Stop regulation ===
    else if (token == "stop") {
        int id;
        if (!parseMotorId(args, id)) return;

        if (id == -1) {
            for (int i = 0; i < NUM_ACTUATORS; i++) {
                isRunning[i] = false;
                motors[i]->stop();
            }
            Serial.println("All motors: stopped");
        } else {
            isRunning[id] = false;
            motors[id]->stop();
            Serial.print("Motor ");
            Serial.print(id);
            Serial.println(": stopped");
        }
    }

    // === pid <id> <kp> <ki> <kd> - Tune PID ===
    else if (token == "pid") {
        int idx1 = args.indexOf(' ');
        if (idx1 == -1) {
            Serial.println("Error: pid <id> <kp> <ki> <kd>");
            return;
        }
        int idx2 = args.indexOf(' ', idx1 + 1);
        if (idx2 == -1) {
            Serial.println("Error: pid <id> <kp> <ki> <kd>");
            return;
        }
        int idx3 = args.indexOf(' ', idx2 + 1);
        if (idx3 == -1) {
            Serial.println("Error: pid <id> <kp> <ki> <kd>");
            return;
        }

        String idStr = args.substring(0, idx1);
        float kp = args.substring(idx1 + 1, idx2).toFloat();
        float ki = args.substring(idx2 + 1, idx3).toFloat();
        float kd = args.substring(idx3 + 1).toFloat();

        int id;
        if (!parseMotorId(idStr, id)) return;

        if (id == -1) {
            for (int i = 0; i < NUM_ACTUATORS; i++) {
                controlLoops[i]->setPidTunings(kp, ki, kd);
            }
            Serial.printf("All motors: PID = Kp=%.1f, Ki=%.3f, Kd=%.1f\n", kp, ki, kd);
        } else {
            controlLoops[id]->setPidTunings(kp, ki, kd);
            Serial.printf("Motor %d: PID = Kp=%.1f, Ki=%.3f, Kd=%.1f\n", id, kp, ki, kd);
        }
    }

    else if(token == "sampletime") {
        int firstSpace = args.indexOf(' ');
        if (firstSpace == -1) {
            Serial.println("Error: sampletime <id> <ms>");
            return;
        }
        String idStr = args.substring(0, firstSpace);
        float newSampleTime = args.substring(firstSpace + 1).toFloat();
        
        int id;
        if (!parseMotorId(idStr, id)) return;

        if (id == -1) {
            for (int i = 0; i < NUM_ACTUATORS; i++) {
                controlLoops[i]->setSampleTime(newSampleTime);
            }
            //Serial.print("All motors: sample time = ");
            //Serial.print(newSampleTime, 3);
            //Serial.println(" ms");
        } else {
            controlLoops[id]->setSampleTime(newSampleTime);
            //Serial.print("Motor ");
            //Serial.print(id);
            //Serial.print(": sample time = ");
            //Serial.print(newSampleTime, 3);
            //Serial.println(" ms");
        }
    }

    else if (token == "fast_print") {
        fast_print = true;
        Serial.println("Fast status printing enabled");
    }

    // === status - Print status ===
    else if (token == "status") {
        printStatus();
    }

    // === help - Show help ===
    else if (token == "help") {
        printHelp();
    }

    else {
        Serial.print("Unknown command: '");
        Serial.print(token);
        Serial.println("' - Type 'help' for commands");
    }
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    unsigned long now = micros();

    // --- 1. Update all sensors (non-blocking) ---
    for (int i = 0; i < NUM_ACTUATORS; i++) {
        sensors[i]->update();
    }
    
    for (int i = 0; i < NUM_ACTUATORS; i++) {
        if (isRunning[i]) {
            float output = controlLoops[i]->update();
            motors[i]->setSpeed((int16_t)output);
        }
    }

    // --- 3. Serial command processing ---
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        processCommand(cmd);
    }

    // --- 4. Status output (10Hz) ---
    if ((now - lastStatusPrint >= STATUS_INTERVAL)&&(fast_print==false)) {
        //printStatus();
        lastStatusPrint = now;
    }
    if (fast_print==true){
        printStatus();
        lastStatusPrint = now;
    }

    //Serial.println(micros() - now); // print loop time for debugging
}
