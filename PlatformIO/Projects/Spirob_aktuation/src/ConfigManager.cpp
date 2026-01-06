#include "ConfigManager.h"

ConfigManager::ConfigManager() {
    prefs.begin("actuation", false);
}

ConfigManager::~ConfigManager() {
    prefs.end();
}

void ConfigManager::saveForceSensorConfig(int channel, ForceSensorConfig config) {
    String key = "fs" + String(channel);
    prefs.putBytes(key.c_str(), &config, sizeof(ForceSensorConfig));
}

ForceSensorConfig ConfigManager::loadForceSensorConfig(int channel) {
    ForceSensorConfig config;
    String key = "fs" + String(channel);
    prefs.getBytes(key.c_str(), &config, sizeof(ForceSensorConfig));
    return config;
}

void ConfigManager::savePidTunings(int channel, float kp, float ki, float kd) {
    String key = "pid" + String(channel);
    prefs.putFloat((key + "kp").c_str(), kp);
    prefs.putFloat((key + "ki").c_str(), ki);
    prefs.putFloat((key + "kd").c_str(), kd);
}

void ConfigManager::loadPidTunings(int channel, float &kp, float &ki, float &kd) {
    String key = "pid" + String(channel);
    kp = prefs.getFloat((key + "kp").c_str(), 300.0);
    ki = prefs.getFloat((key + "ki").c_str(), 0.5);
    kd = prefs.getFloat((key + "kd").c_str(), 50.0);
}