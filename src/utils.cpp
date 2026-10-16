#include "utils.h"
#include "esp_rom_sys.h" // Required for esp_rom_delay_us()

hw_timer_t *timerR = NULL;
hw_timer_t *timerL = NULL;
portMUX_TYPE timerMuxR = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMuxL = portMUX_INITIALIZER_UNLOCKED;

void timerSetup() {
	// ESP32 Arduino Core 2.x Timer API
	timerR = timerBegin(0, 40, true);			   // Timer 0 (Right Motor) - 80MHz / 40 = 2MHz base clock
	timerAttachInterrupt(timerR, &onTimerR, true); // true = edge triggered
	timerAlarmWrite(timerR, TIMER_INIT_OCR, true); // true = autoreload
	timerAlarmEnable(timerR);

	timerL = timerBegin(1, 40, true); // Timer 1 (Left Motor)
	timerAttachInterrupt(timerL, &onTimerL, true);
	timerAlarmWrite(timerL, TIMER_INIT_OCR, true);
	timerAlarmEnable(timerL);
}

// ISR for Timer 0 (Right Motor Step Generation)
void IRAM_ATTR onTimerR() {
	if(directionMotorR == 0) return;
	digitalWrite(STEPPER_R_STEP_PIN, HIGH);
	esp_rom_delay_us(1); // Safely block for 1 microsecond (replaces the 16 NOPs)
	digitalWrite(STEPPER_R_STEP_PIN, LOW);
}

// ISR for Timer 1 (Left Motor Step Generation)
void IRAM_ATTR onTimerL() {
	if(directionMotorL == 0) return;
	digitalWrite(STEPPER_L_STEP_PIN, HIGH);
	esp_rom_delay_us(1); // Safely block for 1 microsecond
	digitalWrite(STEPPER_L_STEP_PIN, LOW);
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