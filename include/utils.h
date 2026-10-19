#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>

#include "config.h"

extern volatile uint32_t stepCountR;
extern volatile uint32_t stepCountL;

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
	Serial.println(F("========================================"));
	Serial.print(F("Board Target: "));
	Serial.println(F("LilyGo T-Display-S3 AMOLED"));
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
	Serial.println(F("========================================"));
}

// Direction variables extern mapping
extern volatile int8_t directionMotorR;
extern volatile int8_t directionMotorL;

// Declarations
void timerSetup();
void printLoopCycleTime();
float readSensorDistanceMeters(uint8_t pin);

#endif // UTILS_H