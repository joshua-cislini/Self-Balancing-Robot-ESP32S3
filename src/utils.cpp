#include "utils.h"
#include "rom/ets_sys.h" // Required for esp_rom_delay_us()

volatile uint32_t stepCountR = 0;
volatile uint32_t stepCountL = 0;

hw_timer_t *timerR = NULL;
hw_timer_t *timerL = NULL;
portMUX_TYPE timerMuxR = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMuxL = portMUX_INITIALIZER_UNLOCKED;

// ISR for Timer 0 (Right Motor)
void IRAM_ATTR onTimerR() {
	if(directionMotorR == 0) return;
	stepCountR += 1; // Replaces stepCountR++ to satisfy C++20 -Wvolatile rules
	digitalWrite(STEPPER_R_STEP_PIN, HIGH);
	esp_rom_delay_us(1);
	digitalWrite(STEPPER_R_STEP_PIN, LOW);
}

// ISR for Timer 1 (Left Motor)
void IRAM_ATTR onTimerL() {
	if(directionMotorL == 0) return;
	stepCountL += 1; // Replaces stepCountL++
	digitalWrite(STEPPER_L_STEP_PIN, HIGH);
	esp_rom_delay_us(1);
	digitalWrite(STEPPER_L_STEP_PIN, LOW);
}

void timerSetup() {
	// Core 3.x API: Pass frequency in Hz directly (no manual prescaler needed)

	// Timer 0 (Right Motor)
	timerR = timerBegin(2000000); // 2 MHz base clock
	timerAttachInterrupt(timerR, &onTimerR);
	timerAlarm(timerR, TIMER_INIT_OCR, true, 0); // (timer, alarm_value, autoreload, reload_count)

	// Timer 1 (Left Motor)
	timerL = timerBegin(2000000);
	timerAttachInterrupt(timerL, &onTimerL);
	timerAlarm(timerL, TIMER_INIT_OCR, true, 0);
}

// --- IR Distance Sensor Conversion (Sharp GP2Y0A21YK0F) ---
float readSensorDistanceMeters(uint8_t pin) {
	uint16_t rawADC = analogRead(pin);

	// Scale 12-bit 3.3V ADC reading to 10-bit 5V equivalent
	float equivalent10BitADC = (float)rawADC * 0.164884f;

	if(equivalent10BitADC < 80.0f) return 0.80f;
	float distanceMeters = (67.87f / (equivalent10BitADC - 3.0f)) - 0.04f;
	return constrain(distanceMeters, 0.10f, 0.80f);
}

void printLoopCycleTime() {
	static uint32_t lastExecutionTime = 0;
	static uint32_t lastPrintTime = 0;
	static uint32_t maxDeltaTime = 0;
	static uint32_t minDeltaTime = 0xFFFFFFFF;
	static uint32_t totalDeltaTime = 0;
	static uint32_t loopCounter = 0;

	uint32_t nowMicros = micros();

	if(lastExecutionTime > 0) {
		uint32_t deltaTime = nowMicros - lastExecutionTime;
		totalDeltaTime += deltaTime;
		loopCounter++;

		if(deltaTime > maxDeltaTime) maxDeltaTime = deltaTime;
		if(deltaTime < minDeltaTime) minDeltaTime = deltaTime;
	}
	lastExecutionTime = nowMicros;

	if(millis() - lastPrintTime >= 1000) {
		lastPrintTime = millis();

		if(loopCounter > 0) {
			uint32_t avgDeltaTime = totalDeltaTime / loopCounter;
			float frequencyHz = (float)loopCounter * (1000000.0f / (float)totalDeltaTime);

			Serial.print(F("[LOOP METRICS] Avg: "));
			Serial.print(avgDeltaTime);
			Serial.print(F(" us ("));
			Serial.print(frequencyHz, 1);
			Serial.print(F(" Hz) | Min: "));
			Serial.print(minDeltaTime);
			Serial.print(F(" us | Max: "));
			Serial.print(maxDeltaTime);
			Serial.println(F(" us"));
		}

		totalDeltaTime = 0;
		loopCounter = 0;
		maxDeltaTime = 0;
		minDeltaTime = 0xFFFFFFFF;
	}
}