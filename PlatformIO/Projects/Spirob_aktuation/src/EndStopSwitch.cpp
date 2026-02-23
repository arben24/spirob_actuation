#include "EndStopSwitch.h"

EndStopSwitch::EndStopSwitch(uint8_t pin, bool activeLow, uint32_t debounceUs)
    : _pin(pin), _activeLow(activeLow), _debounceUs(debounceUs),
      _stable(false), _candidate(false), _candidateTimeUs(0),
      _risingEdge(false), _fallingEdge(false) {}

void EndStopSwitch::begin() {
    pinMode(_pin, INPUT_PULLUP);
    // Bootstrap from actual pin state — accept without debounce at startup
    _stable          = isRawTriggered();
    _candidate       = _stable;
    _candidateTimeUs = micros();
    _risingEdge      = false;
    _fallingEdge     = false;
}

bool EndStopSwitch::isRawTriggered() const {
    return _activeLow ? (digitalRead(_pin) == LOW)
                      : (digitalRead(_pin) == HIGH);
}

void EndStopSwitch::update() {
    // Edge flags are valid for exactly ONE update() cycle
    _risingEdge  = false;
    _fallingEdge = false;

    const bool raw = isRawTriggered();

    if (raw != _candidate) {
        // Pin changed: start fresh debounce window for the new candidate
        _candidate       = raw;
        _candidateTimeUs = micros();
    } else if (raw != _stable) {
        // Candidate is holding — check whether debounce window has elapsed
        if ((micros() - _candidateTimeUs) >= _debounceUs) {
            _stable      = raw;
            _risingEdge  =  raw;   // true  → just triggered (onset)
            _fallingEdge = !raw;   // true  → just released
        }
    }
    // raw == _stable: already settled, nothing to do
}
