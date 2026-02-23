#include <Arduino.h>
#include <Wire.h>
#include "MotorDriver.h"
#include "ForceSensorAnu78025.h"
#include "ForceControlLoop.h"

// ============================================================================
// HARDWARE CONFIGURATION (Static - adjust as needed)
// ============================================================================

#define MOTOR_ID             1          // Servo ID for this motor
#define MOTOR_RX_PIN         16         // Servo UART RX
#define MOTOR_TX_PIN         17         // Servo UART TX
#define FORCE_SENSOR_I2C_ADDR  0x2A     // NAU7802 I2C address
#define FORCE_SENSOR_MUX_CH    0        // TCA9548A multiplexer channel for Motor 2

// ============================================================================
// CALIBRATION CONSTANTS (Calibrate and fill in these values)
// ============================================================================

// Example calibration values - replace with YOUR calibrated values
// Tare at zero load, then place known weight to get offset/scale
#define FORCE_SENSOR_OFFSET  153859L   // Raw ADC value at zero load (tare)
#define FORCE_SENSOR_SCALE   105.13830 // Raw counts per kg: (raw_loaded - offset) / mass_kg

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

HardwareSerial servoSerial(2);         // Serial2 for servo (pins 16/17)
MotorDriver* motor = nullptr;
ForceSensorAnu78025* forceSensor = nullptr;
ForceControlLoop* controlLoop = nullptr;

// ============================================================================
// CONTROL STATE
// ============================================================================

bool isRunning = false;
float forceSetpoint = 0.0f;  // Target force in kg
unsigned long lastStatusPrint = 0;
const unsigned long STATUS_INTERVAL = 1000; // Print status every 100ms (10Hz)

// ============================================================================
// INITIALIZATION
// ============================================================================

void setup() {
    Serial.begin(115200);
    delay(1000); // Wait for serial to stabilize
    
    Wire.begin(); // I2C for sensors
    
    // Initialize servo serial (1Mbps on pins 16/17)
    servoSerial.begin(1000000, SERIAL_8N1, MOTOR_RX_PIN, MOTOR_TX_PIN);
    delay(100);

    Serial.println("\n=== Single Motor Force Control Test ===");
    Serial.print("Motor ID: "); Serial.println(MOTOR_ID);
    Serial.print("Force Sensor: NAU78025 (I2C 0x"); Serial.print(FORCE_SENSOR_I2C_ADDR, HEX);
    Serial.print(", Mux Channel "); Serial.println(FORCE_SENSOR_MUX_CH);
    Serial.println();

    // --- Initialize Motor Driver ---
    Serial.print("Initializing MotorDriver ID ");
    Serial.print(MOTOR_ID);
    Serial.print("...");
    motor = new MotorDriver(MOTOR_ID, &servoSerial);
    delay(100);
    
    int pos = motor->getPosition();
    if (pos == -1) {
        Serial.println(" FAILED - Motor not responding!");
        Serial.println("Check: Wiring, Motor ID, Power, Baud Rate");
        while (1) delay(1000);
    }
    Serial.print(" OK (Position="); Serial.print(pos); Serial.println(")");

    motor->setReverseDirection(true); // Set to true if motor moves in opposite direction

    // --- Initialize Force Sensor ---
    Serial.print("Initializing ForceSensorAnu78025 on Mux Channel ");
    Serial.print(FORCE_SENSOR_MUX_CH);
    Serial.print("...");
    forceSensor = new ForceSensorAnu78025(FORCE_SENSOR_I2C_ADDR, FORCE_SENSOR_MUX_CH);
    
    int retryCount = 0;
    while (!forceSensor->begin()) {
        Serial.print(".");
        delay(500);
        retryCount++;
        if (retryCount > 10) {
            Serial.println(" FAILED - Sensor not responding!");
            Serial.println("Check: I2C Wiring, Multiplexer, Address, Channel");
            while (1) delay(1000);
        }
    }
    Serial.println(" OK");

    // Load calibration
    forceSensor->setCalibration(FORCE_SENSOR_OFFSET, FORCE_SENSOR_SCALE);
    Serial.print("Calibration loaded: Offset="); Serial.print(FORCE_SENSOR_OFFSET);
    Serial.print(", Scale="); Serial.println(FORCE_SENSOR_SCALE);

    // --- Initialize Force Control Loop ---
    Serial.print("Initializing ForceControlLoop...");
    controlLoop = new ForceControlLoop(motor, forceSensor);
    controlLoop->setMode(MODE_PID_FORCE);  // Force regulation mode
    controlLoop->setForceSetpoint(5.0f);   // Start at zero force
    controlLoop->setMaxSpeed(3000);        // Max motor speed
    controlLoop->setSampleTime(200.0f);     // PID sample time in ms
    Serial.println(" OK");

    Serial.println("\n=== Commands ===");
    Serial.println("set <kg>      - Set force setpoint (e.g., 'set 2.5')");
    Serial.println("start         - Start force regulation");
    Serial.println("stop          - Stop regulation (motor stops)");
    Serial.println("pid <kp> <ki> <kd> - Tune PID online (e.g., 'pid 300 0.5 50')");
    Serial.println("tare          - Zero the force sensor");
    Serial.println("status        - Print current status");
    Serial.println("help          - Show this help");
    Serial.println();
}

// ============================================================================
// STATUS OUTPUT
// ============================================================================

void printStatus() {
    float forceActual = forceSensor->getForce();
    int motorPos = motor->getPosition();
    int motorSpeed = motor->getSpeed();
    
    Serial.printf("Force: %.3f/%.3f kg | MotorPos: %d | MotorSpeed: %d | Running: %s\n",
        forceActual, forceSetpoint, motorPos, motorSpeed,
        isRunning ? "YES" : "NO");
}

// ============================================================================
// COMMAND PROCESSING
// ============================================================================

void processCommand(String cmd) {
    cmd.trim();
    if (cmd.length() == 0) return;

    int spaceIdx = cmd.indexOf(' ');
    String token = (spaceIdx == -1) ? cmd : cmd.substring(0, spaceIdx);
    String args = (spaceIdx == -1) ? "" : cmd.substring(spaceIdx + 1);

    // --- set <kg> - Set force setpoint ---
    if (token == "set") {
        float newSetpoint = args.toFloat();
        forceSetpoint = newSetpoint;
        controlLoop->setForceSetpoint(forceSetpoint);
        Serial.print("Force setpoint set to: ");
        Serial.print(forceSetpoint, 3);
        Serial.println(" kg");
    }

    // --- start - Start regulation ---
    else if (token == "start") {
        isRunning = true;
        motor->setMode(MODE_WHEEL);  // Enable wheel mode for speed control
        Serial.println("Force regulation started");
    }

    // --- stop - Stop regulation ---
    else if (token == "stop") {
        isRunning = false;
        motor->stop();
        Serial.println("Force regulation stopped");
    }

    // --- pid <kp> <ki> <kd> - Tune PID ---
    else if (token == "pid") {
        // Parse: "300 0.5 50" format
        int idx1 = args.indexOf(' ');
        if (idx1 == -1) {
            Serial.println("Error: pid <kp> <ki> <kd>");
            return;
        }
        int idx2 = args.indexOf(' ', idx1 + 1);
        if (idx2 == -1) {
            Serial.println("Error: pid <kp> <ki> <kd>");
            return;
        }

        float kp = args.substring(0, idx1).toFloat();
        float ki = args.substring(idx1 + 1, idx2).toFloat();
        float kd = args.substring(idx2 + 1).toFloat();

        controlLoop->setPidTunings(kp, ki, kd);
        Serial.printf("PID tunings updated: Kp=%.1f, Ki=%.3f, Kd=%.1f\n", kp, ki, kd);
    }

    // --- tare - Zero the sensor ---
    else if (token == "tare") {
        forceSensor->tare();
        Serial.println("Force sensor tared (offset updated)");
    }

    // --- status - Print current status ---
    else if (token == "status") {
        printStatus();
    }

    // --- help - Show help ---
    else if (token == "help") {
        Serial.println("\n=== Commands ===");
        Serial.println("set <kg>      - Set force setpoint (e.g., 'set 2.5')");
        Serial.println("start         - Start force regulation");
        Serial.println("stop          - Stop regulation (motor stops)");
        Serial.println("pid <kp> <ki> <kd> - Tune PID online (e.g., 'pid 300 0.5 50')");
        Serial.println("tare          - Zero the force sensor");
        Serial.println("status        - Print current status");
        Serial.println("help          - Show this help");
        Serial.println();
    }

    else {
        Serial.print("Unknown command: '");
        Serial.print(token);
        Serial.println("' - Type 'help' for available commands");
    }
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    unsigned long now = millis();

    // --- 1. Update sensor (non-blocking polling) ---
    forceSensor->update();

    // --- 2. Update control loop (if running) ---
    if (isRunning) {
        // dt in seconds (typically ~10ms)
        static unsigned long lastLoopTime = 0;
        if (lastLoopTime == 0) lastLoopTime = now;
        
        float dt = (now - lastLoopTime) / 1000.0f;
        lastLoopTime = now;
        
        // Clamp dt to reasonable range (prevent large jumps)
        if (dt > 0.1f) dt = 0.01f;
        if (dt < 0.001f) dt = 0.001f;
        
    
        float output = controlLoop->update();
        motor->setSpeed((int16_t)output);

    }

    // --- 3. Serial command processing ---
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        processCommand(cmd);
    }

    // --- 4. Status output (10Hz) ---
    if (now - lastStatusPrint >= STATUS_INTERVAL) {
        printStatus();
        lastStatusPrint = now;
    }

    delay(5); // Small delay to prevent overwhelming the loop
}
