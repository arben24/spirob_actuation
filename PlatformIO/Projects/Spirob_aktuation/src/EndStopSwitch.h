#pragma once
#include <Arduino.h>

/**
 * @brief Software-debounced end-stop / limit switch
 *
 * Provides two readout modes:
 *   isRawTriggered()  — direct digitalRead(), zero latency
 *                       → use for safety-critical immediate motor stops
 *   isTriggered()     — debounced stable state (≤debounceUs latency)
 *                       → use for logic decisions
 *
 * Wiring: momentary NO button between pin and GND, INPUT_PULLUP active
 *         → activeLow = true (default)
 *
 * Usage:
 *   EndStopSwitch sw(26);
 *   sw.begin();                    // call once in setup()
 *   sw.update();                   // call every loop() — ~1 µs
 *   if (sw.isRawTriggered()) ...   // immediate safety stop
 *   if (sw.risingEdge())     ...   // one-shot on first debounced trigger
 */
class EndStopSwitch {
public:
    /**
     * @param pin        GPIO pin number
     * @param activeLow  true = triggered when pin reads LOW (default, INPUT_PULLUP)
     * @param debounceUs Debounce window in microseconds (default: 5000 = 5 ms)
     */
    EndStopSwitch(uint8_t pin, bool activeLow = true, uint32_t debounceUs = 5000);

    /**
     * Configure pin as INPUT_PULLUP and sample initial state immediately.
     * Call once in setup().
     */
    void begin();

    /**
     * Advance the debounce state machine.
     * Call exactly ONCE per loop() iteration. Execution time: ~1 µs.
     * Read edge flags AFTER calling this in the same loop() cycle.
     */
    void update();

    /** Debounced stable trigger state — noise-immune, ≤debounceUs latency. */
    bool isTriggered() const { return _stable; }

    /**
     * Direct digitalRead() — no debounce delay.
     * Use inside fast blocking loops (step response, homing) for immediate stops.
     */
    bool isRawTriggered() const;

    /**
     * True for exactly ONE update() cycle after debounced trigger onset.
     * Call after update() in the same loop().
     */
    bool risingEdge()  const { return _risingEdge; }

    /**
     * True for exactly ONE update() cycle after debounced release.
     */
    bool fallingEdge() const { return _fallingEdge; }

    uint8_t pin() const { return _pin; }

private:
    const uint8_t  _pin;
    const bool     _activeLow;
    const uint32_t _debounceUs;

    bool     _stable;           ///< Last accepted debounced state
    bool     _candidate;        ///< Candidate state waiting to be confirmed
    uint32_t _candidateTimeUs;  ///< micros() when candidate was first seen
    bool     _risingEdge;       ///< Set for one update() cycle on trigger onset
    bool     _fallingEdge;      ///< Set for one update() cycle on release
};
