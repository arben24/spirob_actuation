#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include "ForceSensor.h"
#include <Preferences.h>

class ConfigManager {
private:
    Preferences prefs;

public:
    ConfigManager();
    ~ConfigManager();
    void saveForceSensorConfig(int channel, ForceSensorConfig config);
    ForceSensorConfig loadForceSensorConfig(int channel);
    void savePidTunings(int channel, float kp, float ki, float kd);
    void loadPidTunings(int channel, float &kp, float &ki, float &kd);
};

#endif