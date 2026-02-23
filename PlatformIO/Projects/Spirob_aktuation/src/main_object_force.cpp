
#include <Arduino.h>
#include <Wire.h>

#include "ForceSensorAnu78025.h"

// =========================
// Configuration
// =========================
#define SERIAL_BAUD 460800
#define SENSOR_COUNT 8

// Calibration (raw offset + scale factor per sensor)
// Update these values after calibration.
static long SENSOR_OFFSET[SENSOR_COUNT] = {
	22218, -615736, 116445, -163101, -294077, -263772, -42757, -409674
};

static float SENSOR_SCALE[SENSOR_COUNT] = {
	440.94781f, 391.46408f, 450.66455f, 446.10040f, 448.80185f, 422.53790f, 423.74002f, 433.52628f
};

// =========================
// Binary frame definition
// =========================
// Frame: 0xAA 0x55 | uint32_t timestamp_us | float forceN[8]
static const uint8_t FRAME_HEADER[2] = {0xAA, 0x55};

struct __attribute__((packed)) SensorFrame {
	uint8_t header[2];
	uint32_t timestamp_us;
	float forceN[SENSOR_COUNT];
};

// =========================
// Globals
// =========================
static ForceSensorAnu78025* sensors[SENSOR_COUNT];
static SensorFrame frame;

// Profiling / timing
static bool profilingEnabled = false;
static uint32_t lastReportMs = 0;
static uint32_t loopCount = 0;
static uint32_t loopTimeSumUs = 0;
static uint32_t sensorTimeSumUs[SENSOR_COUNT] = {0};
static uint32_t sensorTimeMaxUs[SENSOR_COUNT] = {0};

// Simple line buffer for CLI commands
static char cmdBuffer[32];
static uint8_t cmdIndex = 0;

// =========================
// Helpers
// =========================
static void handleCommand(const char* cmdLine) {
	// Supported commands:
	// "t"        -> tare sensor 0
	// "t a"      -> tare all sensors
	// "t <idx>"  -> tare sensor idx (0..7 or 1..8)
	if (cmdLine[0] == 't') {
		// Skip leading 't'
		const char* p = cmdLine + 1;
		while (*p == ' ') { p++; }

		if (*p == '\0') {
			// Default: tare sensor 0
			sensors[0]->tare();
			return;
		}

		if (*p == 'a' || *p == 'A') {
			for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
				sensors[i]->tare();
			}
			return;
		}

		int idx = atoi(p);
		if (idx >= 1 && idx <= SENSOR_COUNT) {
			idx -= 1; // allow 1-based index
		}
		if (idx >= 0 && idx < SENSOR_COUNT) {
			sensors[idx]->tare();
		}
	}

	// Profiling toggle: "p" or "p on/off"
	if (cmdLine[0] == 'p') {
		const char* p = cmdLine + 1;
		while (*p == ' ') { p++; }
		if (*p == '\0') {
			profilingEnabled = !profilingEnabled;
		} else if (strncmp(p, "on", 2) == 0) {
			profilingEnabled = true;
		} else if (strncmp(p, "off", 3) == 0) {
			profilingEnabled = false;
		}
		Serial.print("profiling: ");
		Serial.println(profilingEnabled ? "on" : "off");
	}
}

static void pollSerial() {
	while (Serial.available() > 0) {
		char c = (char)Serial.read();
		if (c == '\n' || c == '\r') {
			if (cmdIndex > 0) {
				cmdBuffer[cmdIndex] = '\0';
				handleCommand(cmdBuffer);
				cmdIndex = 0;
			}
		} else {
			if (cmdIndex < (sizeof(cmdBuffer) - 1)) {
				cmdBuffer[cmdIndex++] = c;
			}
		}
	}
}

// =========================
// Arduino entry points
// =========================
void setup() {
	Serial.begin(SERIAL_BAUD);
	Wire.begin();
    Wire.setClock(400000);

	// Initialize sensors on mux channels 0..7
	for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
		sensors[i] = new ForceSensorAnu78025(0x2A, i, NAU7802_SPS_320);
		sensors[i]->begin();
		sensors[i]->setCalibration(SENSOR_OFFSET[i], SENSOR_SCALE[i]);
	}

	frame.header[0] = FRAME_HEADER[0];
	frame.header[1] = FRAME_HEADER[1];
}

void loop() {
	uint32_t loopStartUs = micros();

	// Update sensors and collect per-sensor timing
	for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
		uint32_t sStart = micros();
		sensors[i]->update();
		uint32_t sDur = micros() - sStart;

		sensorTimeSumUs[i] += sDur;
		if (sDur > sensorTimeMaxUs[i]) {
			sensorTimeMaxUs[i] = sDur;
		}

		frame.forceN[i] = sensors[i]->getForce();
	}

	// Send binary frame as fast as possible when not profiling
	if (!profilingEnabled) {
		frame.timestamp_us = micros();
		Serial.write(reinterpret_cast<uint8_t*>(&frame), sizeof(SensorFrame));
	}

	// Handle CLI commands (non-blocking)
	pollSerial();

	// Update loop timing stats
	uint32_t loopDurUs = micros() - loopStartUs;
	loopTimeSumUs += loopDurUs;
	loopCount++;

	// Report once per second in readable text when profiling enabled
	if (profilingEnabled && (millis() - lastReportMs) >= 1000) {
		uint32_t elapsedMs = millis() - lastReportMs;
		float elapsedS = elapsedMs / 1000.0f;
		float loopHz = loopCount / elapsedS;
		uint32_t avgLoopUs = (loopCount > 0) ? (loopTimeSumUs / loopCount) : 0;

		Serial.println("--- timing ---");
		Serial.print("loop_hz: "); Serial.println(loopHz, 2);
		Serial.print("loop_avg_us: "); Serial.println(avgLoopUs);
		for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
			uint32_t avgUs = (loopCount > 0) ? (sensorTimeSumUs[i] / loopCount) : 0;
			Serial.print("sensor "); Serial.print(i);
			Serial.print(" avg_us: "); Serial.print(avgUs);
			Serial.print(" max_us: "); Serial.println(sensorTimeMaxUs[i]);
		}

		// reset counters
		lastReportMs = millis();
		loopCount = 0;
		loopTimeSumUs = 0;
		for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
			sensorTimeSumUs[i] = 0;
			sensorTimeMaxUs[i] = 0;
		}
	}
}
