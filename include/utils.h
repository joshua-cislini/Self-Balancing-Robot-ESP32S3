#ifndef UTILS_H
#define UTILS_H

#include "config.h"
#include <Arduino.h>

// --- Hardware Timers for ESP32 ---
extern hw_timer_t *timerR;
extern hw_timer_t *timerL;
extern portMUX_TYPE timerMuxR;
extern portMUX_TYPE timerMuxL;

// --- Compile-Time Timer Calculations & Safety ---
#define STEPPER_TIMER_BASE_FREQ 2000000UL // Target 2 MHz tick rate (0.5 us per tick)
#define TIMER_INIT_OCR 65535U

// --- Stringizing Macros ---
#define STRINGIFY_INDIR(x) #x
#define STRINGIFY(x) STRINGIFY_INDIR(x)

// Inline Build Info Function
inline void printBuildInfo() {
	Serial.println(F("============================================="));
	Serial.print(F("Board Target: "));
	Serial.println(F("LilyGo T-Display-S3 AMOLED (ESP32-S3)"));
	Serial.print(F("CPU Speed:    "));
	Serial.print(F(STRINGIFY(F_CPU)));
	Serial.println(F(" Hz"));
	Serial.print(F("Timer Freq:   "));
	Serial.print(F(STRINGIFY(STEPPER_TIMER_BASE_FREQ)));
	Serial.println(F(" Hz"));
	Serial.print(F("Build Date:   "));
	Serial.print(F(__DATE__));
	Serial.print(F(" "));
	Serial.println(F(__TIME__));
	Serial.println(F("============================================="));
}

// Fast I/O helpers (Standard ESP32 digitalRead/Write are extremely fast at 240MHz)
inline int fastDigitalRead(uint8_t pin) {
	return digitalRead(pin);
}

inline void fastDigitalWrite(uint8_t pin, uint8_t val) {
	digitalWrite(pin, val);
}

// --- IR Distance Sensor Conversion (Sharp GP2Y0A21YK0F) ---
inline float readSensorDistanceMeters(uint8_t pin) {
	uint16_t rawADC = analogRead(pin);

	// ESP32 uses a 12-bit ADC (0-4095) at 3.3V.
	// Convert this back to the 10-bit 5V equivalent used in the original math:
	float voltage = ((float)rawADC / 4095.0f) * 3.3f;
	float equivalent10BitADC = (voltage / 5.0f) * 1023.0f;

	if(equivalent10BitADC < 80.0f) return 0.80f;
	float distanceMeters = (67.87f / (equivalent10BitADC - 3.0f)) - 0.04f;
	return constrain(distanceMeters, 0.10f, 0.80f);
}

// Direction variables extern mapping
extern volatile int8_t directionMotorR;
extern volatile int8_t directionMotorL;

// Declarations
void timerSetup();
void printLoopCycleTime();

#endif // UTILS_H