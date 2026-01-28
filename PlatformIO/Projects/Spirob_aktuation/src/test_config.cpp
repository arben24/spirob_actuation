#include "ConfigManager.h"
#include "ForceSensor.h"

ConfigManager configManager;

void clearTestData() {
    // Clear the NVS namespace to start fresh
    Preferences prefs;
    prefs.begin("actuation", false);
    prefs.clear();
    prefs.end();
    Serial.println("✓ Test data cleared from NVS");
}

void savePidTest(int channel, float kp, float ki, float kd) {
    configManager.savePidTunings(channel, kp, ki, kd);
    Serial.print("✓ Saved PID Ch ");
    Serial.print(channel);
    Serial.print(": kp=");
    Serial.print(kp);
    Serial.print(", ki=");
    Serial.print(ki);
    Serial.print(", kd=");
    Serial.println(kd);
}

void loadPidTest(int channel) {
    float kp, ki, kd;
    configManager.loadPidTunings(channel, kp, ki, kd);
    Serial.print("✓ Loaded PID Ch ");
    Serial.print(channel);
    Serial.print(": kp=");
    Serial.print(kp);
    Serial.print(", ki=");
    Serial.print(ki);
    Serial.print(", kd=");
    Serial.println(kd);
}

void saveSensorTest(int channel, int type, float scale) {
    ForceSensorConfig config;
    config.type = (ForceSensorType)type;
    config.scale = scale;
    config.offset = 0;
    config.nominalLoad = 5.0;
    
    configManager.saveForceSensorConfig(channel, config);
    Serial.print("✓ Saved Sensor Config Ch ");
    Serial.print(channel);
    Serial.print(": type=");
    Serial.print(type);
    Serial.print(", scale=");
    Serial.println(scale);
}

void loadSensorTest(int channel) {
    ForceSensorConfig config = configManager.loadForceSensorConfig(channel);
    Serial.print("✓ Loaded Sensor Config Ch ");
    Serial.print(channel);
    Serial.print(": type=");
    Serial.print(config.type);
    Serial.print(", scale=");
    Serial.print(config.scale);
    Serial.print(", nominal=");
    Serial.println(config.nominalLoad);
}

void stressTest() {
    Serial.println("\n=== Starting Stress Test ===");
    bool allPassed = true;
    
    // Test PID Tunings
    Serial.println("\n--- PID Tunings Test ---");
    float testKp = 2.5, testKi = 0.1, testKd = 0.0;
    for (int ch = 0; ch < 2; ch++) {
        Serial.print("Ch ");
        Serial.print(ch);
        Serial.print(": Writing kp=");
        Serial.print(testKp);
        Serial.print(", ki=");
        Serial.print(testKi);
        Serial.print(", kd=");
        Serial.print(testKd);
        Serial.print(" ... ");
        
        configManager.savePidTunings(ch, testKp, testKi, testKd);
        
        float readKp, readKi, readKd;
        configManager.loadPidTunings(ch, readKp, readKi, readKd);
        
        if (readKp == testKp && readKi == testKi && readKd == testKd) {
            Serial.println("PASS");
        } else {
            Serial.print("FAIL - Got kp=");
            Serial.print(readKp);
            Serial.print(", ki=");
            Serial.print(readKi);
            Serial.print(", kd=");
            Serial.println(readKd);
            allPassed = false;
        }
    }
    
    // Test Sensor Configs
    Serial.println("\n--- Sensor Config Test ---");
    int testType = FORCE_HX711;
    float testScale = 0.00025;
    for (int ch = 0; ch < 2; ch++) {
        Serial.print("Ch ");
        Serial.print(ch);
        Serial.print(": Writing type=");
        Serial.print(testType);
        Serial.print(", scale=");
        Serial.print(testScale);
        Serial.print(" ... ");
        
        ForceSensorConfig config;
        config.type = (ForceSensorType)testType;
        config.scale = testScale;
        config.offset = 1000;
        config.nominalLoad = 5.0;
        
        configManager.saveForceSensorConfig(ch, config);
        
        ForceSensorConfig readConfig = configManager.loadForceSensorConfig(ch);
        
        if (readConfig.type == testType && 
            readConfig.scale == testScale &&
            readConfig.nominalLoad == 5.0) {
            Serial.println("PASS");
        } else {
            Serial.print("FAIL - Got type=");
            Serial.print(readConfig.type);
            Serial.print(", scale=");
            Serial.println(readConfig.scale);
            allPassed = false;
        }
    }
    
    // Summary
    Serial.println("\n--- Stress Test Result ---");
    if (allPassed) {
        Serial.println("✓✓✓ ALL TESTS PASSED ✓✓✓");
    } else {
        Serial.println("✗✗✗ SOME TESTS FAILED ✗✗✗");
    }
    Serial.println();
}

void processCommand(String cmd) {
    cmd.trim();
    if (cmd.length() == 0) return;
    
    int spaceIdx = cmd.indexOf(' ');
    String token = (spaceIdx == -1) ? cmd : cmd.substring(0, spaceIdx);
    String args = (spaceIdx == -1) ? "" : cmd.substring(spaceIdx + 1);
    
    if (token == "clear") {
        clearTestData();
    } else if (token == "save_pid") {
        // save_pid <ch> <kp> <ki> <kd>
        int idx1 = args.indexOf(' ');
        int idx2 = args.indexOf(' ', idx1 + 1);
        int idx3 = args.indexOf(' ', idx2 + 1);
        
        if (idx1 == -1 || idx2 == -1 || idx3 == -1) {
            Serial.println("Error: save_pid <ch> <kp> <ki> <kd>");
            return;
        }
        
        int ch = args.substring(0, idx1).toInt();
        float kp = args.substring(idx1 + 1, idx2).toFloat();
        float ki = args.substring(idx2 + 1, idx3).toFloat();
        float kd = args.substring(idx3 + 1).toFloat();
        
        savePidTest(ch, kp, ki, kd);
    } else if (token == "load_pid") {
        // load_pid <ch>
        int ch = args.toInt();
        loadPidTest(ch);
    } else if (token == "save_sensor") {
        // save_sensor <ch> <type> <scale>
        int idx1 = args.indexOf(' ');
        int idx2 = args.indexOf(' ', idx1 + 1);
        
        if (idx1 == -1 || idx2 == -1) {
            Serial.println("Error: save_sensor <ch> <type> <scale>");
            return;
        }
        
        int ch = args.substring(0, idx1).toInt();
        int type = args.substring(idx1 + 1, idx2).toInt();
        float scale = args.substring(idx2 + 1).toFloat();
        
        saveSensorTest(ch, type, scale);
    } else if (token == "load_sensor") {
        // load_sensor <ch>
        int ch = args.toInt();
        loadSensorTest(ch);
    } else if (token == "stress") {
        stressTest();
    } else if (token == "h" || token == "help") {
        Serial.println("\n=== ConfigManager Test Commands ===");
        Serial.println("clear                       - Clear all test data from NVS");
        Serial.println("save_pid <ch> <kp> <ki> <kd> - Save PID tuning");
        Serial.println("load_pid <ch>               - Load and display PID tuning");
        Serial.println("save_sensor <ch> <type> <scale> - Save sensor config");
        Serial.println("load_sensor <ch>            - Load and display sensor config");
        Serial.println("stress                      - Run comprehensive stress test");
        Serial.println("h, help                     - Show this help message");
        Serial.println();
    } else {
        Serial.println("Unknown command. Type 'h' for help.");
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000); // Wait for serial to initialize
    configManager.begin();
    
    Serial.println("\n=== ConfigManager Test Suite ===");
    Serial.println("Testing ESP32 NVS Preferences for Spirob Aktuation");
    Serial.println("Type 'h' for help or 'stress' to run all tests");
    Serial.println();
}

void loop() {
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        processCommand(cmd);
    }
    
    delay(10);
}
