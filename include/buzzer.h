#ifndef BUZZER_H
#define BUZZER_H

#include <Arduino.h>

class Buzzer {
  public:
	Buzzer(uint8_t pin);

	void begin();
	void blocking(uint16_t duration_ms);
	void buzz(uint16_t duration_ms);
	void update();
	void disable();
	void enable();
	bool isEnabled() const; // Marked const because it doesn't modify object state

  private:
	uint8_t _pin;
	uint32_t _stopTime; // Replaces _counter to use true milliseconds
	bool _isBuzzing;	// Tracks state to eliminate redundant digitalWrites
	bool _isEnabled;
};

#endif