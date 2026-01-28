#include <Arduino.h>
#include <Wire.h>
#include "DistanceSensorVL6180X.h"

// VL6180X Parameters (adjust to your hardware)
#define MULTIPLEXER_CHANNEL1  0     // Channel on TCA9548A multiplexer
#define MULTIPLEXER_CHANNEL2  1     // Another channel if needed

DistanceSensorVL6180X* sensor1;
DistanceSensorVL6180X* sensor2;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    delay(1000);
    Serial.println("VL6180X Unit Test Starting...");

    sensor1 = new DistanceSensorVL6180X(MULTIPLEXER_CHANNEL1);
    sensor2 = new DistanceSensorVL6180X(MULTIPLEXER_CHANNEL2);

    if (sensor1->begin()) {
        Serial.println("VL6180X initialized successfully");
        Serial.println("Ready for testing.");
    } else {
        Serial.println("ERROR: VL6180X initialization failed");
        while (1) delay(1000);
    }
    if(sensor2->begin()) {
        Serial.println("Second VL6180X initialized successfully");
    } else {
        Serial.println("ERROR: Second VL6180X initialization failed");
    }
}

void loop() {
    sensor1->update();
    if (sensor1->isReady()) {
        float distance = sensor1->getDistance();
        Serial.print("Distance: ");
        Serial.print(distance, 2);
        Serial.print(" mm");
    } else {
        Serial.print("sensor1 not ready, status: ");
        Serial.print(sensor1->getLastStatus());
        Serial.print(", samples: ");
        Serial.println(sensor1->getWindowCount());
    }
    sensor2->update();
    if (sensor2->isReady()) {
        float distance2 = sensor2->getDistance();
        Serial.print("Distance2: ");
        Serial.print(distance2, 2);
        Serial.println(" mm");
    } else {
        Serial.print("sensor2 not ready, status: ");
        Serial.print(sensor2->getLastStatus());
        Serial.print(", samples: ");
        Serial.println(sensor2->getWindowCount());
    }

    delay(100);
}