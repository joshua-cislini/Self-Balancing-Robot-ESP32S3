#include "buzzer.h"

// 1. Use an initializer list (faster and safer than assigning in the body)
Buzzer::Buzzer(uint8_t pin)
  : _pin(pin), _stopTime(0), _isBuzzing(false), _isEnabled(true) {}

void Buzzer::begin() {
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, LOW);
}

void Buzzer::buzz(uint16_t duration_ms) {
  if (_isEnabled) {
    _stopTime = millis() + duration_ms;

    // Only call digitalWrite if it's not already on
    if (!_isBuzzing) {
      digitalWrite(_pin, HIGH);
      _isBuzzing = true;
    }
  }
}

void Buzzer::buzzBlocking(uint16_t duration_ms) {
  if (_isEnabled) {
    digitalWrite(_pin, HIGH);
    delay(duration_ms);
    digitalWrite(_pin, LOW);
    _isBuzzing = false;  // Reset state
  }
}

void Buzzer::disable() {
  _isEnabled = false;
  digitalWrite(_pin, LOW);  // Force off immediately just in case it was buzzing
  _isBuzzing = false;
}

void Buzzer::enable() {
  _isEnabled = true;
}

bool Buzzer::isEnabled() const {
  return _isEnabled;
}

void Buzzer::doUpdate() {
  // 2. Only execute if currently buzzing AND time has expired
  if (_isBuzzing && millis() >= _stopTime) {
    digitalWrite(_pin, LOW);
    _isBuzzing = false;  // Prevents digitalWrite from being called again until next buzz
  }
}