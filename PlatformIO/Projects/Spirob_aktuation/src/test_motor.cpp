#include "MotorDriver.h"
#include <HardwareSerial.h>

// Hardware configuration
#define RX_PIN 16
#define TX_PIN 17
#define SERVO_ID_0 1
#define SERVO_ID_1 2

HardwareSerial servoSerial(1); // Use Serial1 for servo communication
MotorDriver* motors[2]; // Array for both motors

void setup() {
    // Initialize serial for debug output
    Serial.begin(115200);
    delay(1000); // Wait for serial to initialize

    // Initialize servo serial (1Mbit/s, 8N1)
    servoSerial.begin(1000000, SERIAL_8N1, RX_PIN, TX_PIN);

    // Create MotorDriver instances for both motors
    motors[0] = new MotorDriver(SERVO_ID_0, &servoSerial);
    motors[1] = new MotorDriver(SERVO_ID_1, &servoSerial);

    // Connection check
    Serial.println("Checking motor connections...");
    bool allConnected = true;
    for (int i = 0; i < 2; i++) {
        int pos = motors[i]->getPosition();
        if (pos == -1) {
            Serial.print("Motor ");
            Serial.print(i);
            Serial.println(" not answering - check wiring/ID/power");
            allConnected = false;
        } else {
            Serial.print("Motor ");
            Serial.print(i);
            Serial.print(" connected, initial position: ");
            Serial.println(pos);
        }
    }

    Serial.println("\n=== Motor Test Ready ===");
    Serial.println("Mode Commands:");
    Serial.println("  mode <id> 0     - Set SERVO_POSITION mode");
    Serial.println("  mode <id> 1     - Set WHEEL mode");
    Serial.println("\nSingle Motor Commands:");
    Serial.println("  m <id> <pos>    - Set position (e.g., m 0 1000)");
    Serial.println("  w <id> <speed>  - Set speed (e.g., w 0 500)");
    Serial.println("  s <id>          - Stop motor (e.g., s 0)");
    Serial.println("  p <id>          - Get position");
    Serial.println("  stat <id>       - Get all status (pos, load, speed, temp, volt, current, move)");
    Serial.println("  t <id> <0/1>    - Torque on/off");
    Serial.println("\nBroadcast Commands (all motors):");
    Serial.println("  m a <pos>       - Move all motors to position");
    Serial.println("  s a             - Stop all motors");
    Serial.println("\nGeneral:");
    Serial.println("  i               - Info on both motors");
    Serial.println("  h               - Help (this message)");
}


void printMotorStatus(int id) {
    if (id < 0 || id > 1) {
        Serial.println("Error: Only ID 0 and 1 supported");
        return;
    }
    
    MotorDriver* m = motors[id];
    Serial.print("Motor ");
    Serial.print(id);
    Serial.print(": Pos=");
    Serial.print(m->getPosition());
    Serial.print(" Load=");
    Serial.print(m->getLoad());
    Serial.print(" Speed=");
    Serial.print(m->getSpeed());
    Serial.print(" Temp=");
    Serial.print(m->getTemperature());
    Serial.print("C Volt=");
    Serial.print(m->getVoltage());
    Serial.print("V Current=");
    Serial.print(m->getCurrent());
    Serial.print("mA Move=");
    Serial.println(m->getMove());
}

void processMotorCommand(String cmd) {
    cmd.trim();
    if (cmd.length() == 0) return;

    // Parse first token
    int spaceIdx = cmd.indexOf(' ');
    String token = (spaceIdx == -1) ? cmd : cmd.substring(0, spaceIdx);
    String args = (spaceIdx == -1) ? "" : cmd.substring(spaceIdx + 1);

    // === Mode Commands ===
    if (token == "mode") {
        // mode <id> <0/1> - Set servo mode
        int firstSpace = args.indexOf(' ');
        if (firstSpace == -1) {
            Serial.println("Error: mode <id> <0/1>");
            return;
        }
        int id = args.substring(0, firstSpace).toInt();
        int modeVal = args.substring(firstSpace + 1).toInt();

        if (id == 0 || id == 1) {
            DriverMode mode = (modeVal == 0) ? MODE_SERVO_POSITION : MODE_WHEEL;
            motors[id]->setMode(mode);
            Serial.print("Motor ");
            Serial.print(id);
            Serial.print(" mode set to: ");
            Serial.println(modeVal == 0 ? "SERVO_POSITION" : "WHEEL");
        } else {
            Serial.println("Error: Only ID 0 and 1 supported");
        }

    } else if (token == "m") {
        // m <id> <pos> - Set position
        int firstSpace = args.indexOf(' ');
        if (firstSpace == -1) {
            Serial.println("Error: m <id> <pos>");
            return;
        }
        String idStr = args.substring(0, firstSpace);
        String paramStr = args.substring(firstSpace + 1);

        Serial.println("Debug: Received m command");
        Serial.print("Debug: id = ");
        Serial.println(idStr);
        Serial.print("Debug: pos = ");
        Serial.println(paramStr);

        if (idStr == "a") {
            // BROADCAST FALL
            int pos = paramStr.toInt();
            for (int i = 0; i < 2; i++) {
                motors[i]->setPosition(pos);
            }
            Serial.print("All motors set to position ");
            Serial.println(pos);
        } else {
            // EINZEL FALL
            int id = idStr.toInt();
            int pos = paramStr.toInt();
            
            if (id >= 0 && id < 2) {
                motors[id]->setPosition(pos);
                Serial.printf("Motor %d set to position %d\n", id, pos);
            } else {
                Serial.println("Error: Only ID 0 and 1 supported");
            }
        }

    } else if (token == "w") {
        // w <id> <speed> - Set speed
        int firstSpace = args.indexOf(' ');
        if (firstSpace == -1) {
            Serial.println("Error: w <id> <speed>");
            return;
        }
        int id = args.substring(0, firstSpace).toInt();
        int speed = args.substring(firstSpace + 1).toInt();

        if (id == 0 || id == 1) {
            motors[id]->setSpeed(speed);
            Serial.print("Motor ");
            Serial.print(id);
            Serial.print(" speed set to ");
            Serial.println(speed);
        } else {
            Serial.println("Error: Only ID 0 and 1 supported");
        }

    } else if (token == "s") {
        // s <id> or s a - Stop motor(s)
        if (args == "a") {
            // Broadcast: s a - Stop all
            for (int i = 0; i < 2; i++) {
                motors[i]->stop();
            }
            Serial.println("All motors stopped");
        } else {
            int id = args.toInt();
            if (id == 0 || id == 1) {
                motors[id]->stop();
                Serial.print("Motor ");
                Serial.print(id);
                Serial.println(" stopped");
            } else {
                Serial.println("Error: Only ID 0 and 1 supported");
            }
        }

    } else if (token == "p") {
        // p <id> - Get position
        int id = args.toInt();
        if (id == 0 || id == 1) {
            int pos = motors[id]->getPosition();
            Serial.print("Motor ");
            Serial.print(id);
            Serial.print(" position: ");
            Serial.println(pos);
        } else {
            Serial.println("Error: Only ID 0 and 1 supported");
        }

    } else if (token == "stat") {
        // stat <id> - Get all status
        int id = args.toInt();
        printMotorStatus(id);

    } else if (token == "i") {
        // i - Info on both motors
        Serial.println("=== Motor Information ===");
        for (int i = 0; i < 2; i++) {
            printMotorStatus(i);
        }

    } else if (token == "h") {
        // h - Help
        Serial.println("\n=== Motor Test Commands ===");
        Serial.println("Mode Commands:");
        Serial.println("  mode <id> 0     - Set SERVO_POSITION mode");
        Serial.println("  mode <id> 1     - Set WHEEL mode");
        Serial.println("\nSingle Motor Commands:");
        Serial.println("  m <id> <pos>    - Set position (e.g., m 0 1000)");
        Serial.println("  w <id> <speed>  - Set speed (e.g., w 0 500)");
        Serial.println("  s <id>          - Stop motor (e.g., s 0)");
        Serial.println("  p <id>          - Get position");
        Serial.println("  stat <id>       - Get all status");
        Serial.println("  t <id> <0/1>    - Torque on/off");
        Serial.println("\nBroadcast Commands (all motors):");
        Serial.println("  m a <pos>       - Move all motors to position");
        Serial.println("  s a             - Stop all motors");
        Serial.println("\nGeneral:");
        Serial.println("  i               - Info on both motors");
        Serial.println("  h               - Help (this message)");

    } else {
        Serial.println("Unknown command. Type 'h' for help.");
    }
}

void loop() {
    // Handle serial commands
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        processMotorCommand(cmd);
    }

    delay(10); // Small delay to prevent overwhelming the loop
}