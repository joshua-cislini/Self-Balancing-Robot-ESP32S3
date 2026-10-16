#include "buzzer.h"
#include "complementaryFilter.h"
#include "config.h"
#include "mpu6050.h"
#include "pid.h"
#include "utils.h"
#include <Wire.h>

ComplementaryFilter angleFilter;
Mpu6050 IMU(MPU6050_ADDRESS);
Pid anglePID(ANGLE_P, ANGLE_I, ANGLE_D, ANGLE_I_LIMIT);
Pid speedPID(SPEED_P, SPEED_I, SPEED_D, SPEED_I_LIMIT);
Buzzer buzzer(BUZZER_PIN);

void batteryCheck();
void behaviour();
void avoid();
void follow();
void balance();
void setMotorSpeed(int16_t tspeed, int8_t motorID);

RobotState currentState = RobotState::DISARMED;

uint32_t lastLoopTime = 0; // [us][cite: 10]
float deltaTime = 0.0f;	   // [s][cite: 10]

// Explicit Steps/Sec Motion Variables
float motorSpeedSps = 0.0f;
float targetSpeedSps = 0.0f;
float turningSpeedSps = 0.0f;
float actualTargetSpeedSps = 0.0f;
float actualTurningSpeedSps = 0.0f;

volatile int8_t directionMotorR = 0;
volatile int8_t directionMotorL = 0;

void setup() {
	timerSetup();

	Serial.begin(500000);
	while(!Serial);
	printBuildInfo();

	// analogReference(INTERNAL) removed: ESP32 uses 3.3V reference by default
	// Set ESP32 ADC resolution explicitly to match our math in config.h and utils.h
	analogReadResolution(12);

	randomSeed(analogRead(BATTERY_SENSE_PIN) ^ micros());

	pinMode(SWITCH_1_PIN, INPUT_PULLUP);
	pinMode(SWITCH_2_PIN, INPUT_PULLUP);
	pinMode(STEPPER_R_STEP_PIN, OUTPUT);
	pinMode(STEPPER_R_DIR_PIN, OUTPUT);
	pinMode(STEPPER_L_STEP_PIN, OUTPUT);
	pinMode(STEPPER_L_DIR_PIN, OUTPUT);
	pinMode(STEPPER_ENABLE_PIN, OUTPUT);
	pinMode(STEPPER_SLEEP_PIN, OUTPUT);
	pinMode(RX_LED_PIN, OUTPUT);

	fastDigitalWrite(STEPPER_ENABLE_PIN, HIGH);
	fastDigitalWrite(STEPPER_SLEEP_PIN, LOW);
	fastDigitalWrite(RX_LED_PIN, HIGH);

	// Initialize I2C. Note: On custom ESP32 boards, you may need to map SDA/SCL pins here:
	// Wire.begin(SDA_PIN, SCL_PIN);
	Wire.begin();
	Wire.setClock(400000);

	IMU.begin();
	IMU.calibrateGyro();
	IMU.updateIMUdata();
	float initialAccAngle = atan2f(IMU.getAccelY(), IMU.getAccelZ()) * RAD_TO_DEG;
	angleFilter.setInitialAngle(initialAccAngle);
	IMU.serialPrintDebug();
	Serial.flush();

	lastLoopTime = micros();
}

void loop() {
	uint32_t currentLoopTime = micros();
	deltaTime = (float)(currentLoopTime - lastLoopTime) * 0.000001f;
	lastLoopTime = currentLoopTime;
	printLoopCycleTime();

	buzzer.doUpdate();
	batteryCheck();

	if(currentState != RobotState::LOW_BATTERY) {
		if(currentState != RobotState::DISARMED) {
			bool sw1 = fastDigitalRead(SWITCH_1_PIN);
			bool sw2 = fastDigitalRead(SWITCH_2_PIN);
			if(sw1 && sw2) currentState = RobotState::IDLE;
			else if(!sw1 && sw2) currentState = RobotState::FOLLOW;
			else if(sw1 && !sw2) currentState = RobotState::CUSTOM_MODE;
			else currentState = RobotState::AVOID;
		}
		IMU.updateIMUdata();
		behaviour();
		balance();
	} else {
		setMotorSpeed(0, 1);
		setMotorSpeed(0, 2);
		fastDigitalWrite(STEPPER_ENABLE_PIN, HIGH);
		fastDigitalWrite(STEPPER_SLEEP_PIN, LOW);
	}

	uint16_t ledPhase = millis() & 1023;
	switch(currentState) {
		case RobotState::LOW_BATTERY: fastDigitalWrite(RX_LED_PIN, (ledPhase & 64) ? LOW : HIGH); break;
		case RobotState::DISARMED: fastDigitalWrite(RX_LED_PIN, LOW); break;
		case RobotState::IDLE: fastDigitalWrite(RX_LED_PIN, (ledPhase < 64 || (ledPhase > 128 && ledPhase < 192)) ? LOW : HIGH); break;
		case RobotState::AVOID: fastDigitalWrite(RX_LED_PIN, ledPhase > 960 ? LOW : HIGH); break;
		case RobotState::FOLLOW: fastDigitalWrite(RX_LED_PIN, ledPhase < 960 ? LOW : HIGH); break;
		case RobotState::CUSTOM_MODE: fastDigitalWrite(RX_LED_PIN, (ledPhase & 512) ? LOW : HIGH); break;
	}
}

void batteryCheck() {
	static uint32_t lastBatteryCheckTime = 0;
	static uint32_t lastBatteryWarnBeepTime = 0;
	static uint8_t lowVoltageCounter = 0;
	uint32_t now = millis();

	if(now - lastBatteryCheckTime >= 2000) {
		lastBatteryCheckTime = now;
		if(now < 3000) return;

		uint16_t rawADC = analogRead(BATTERY_SENSE_PIN);
		float voltage = (float)rawADC * VOLTAGE_DIVIDER_MULT;

		if(voltage < MIN_BAT_VOLTAGE) {
			lowVoltageCounter++;
			if(lowVoltageCounter >= 5) {
				currentState = RobotState::LOW_BATTERY;
			}
		} else {
			lowVoltageCounter = 0;

			if(currentState == RobotState::LOW_BATTERY && voltage > WARN_BAT_VOLTAGE) {
				currentState = RobotState::DISARMED;
				fastDigitalWrite(RX_LED_PIN, HIGH);
			}

			if(currentState != RobotState::LOW_BATTERY && voltage < WARN_BAT_VOLTAGE) {
				if(now - lastBatteryWarnBeepTime > 5000) {
					lastBatteryWarnBeepTime = now;
					buzzer.buzz(100);
				}
			}
		}
	}
}

void balance() {
	float accAngle = atan2f(IMU.getAccelY(), IMU.getAccelZ()) * RAD_TO_DEG;
	float currentLeanAngle = angleFilter.calculate(accAngle, IMU.getGyroX(), deltaTime);

	if(currentState == RobotState::DISARMED) {
		setMotorSpeed(0, 1);
		setMotorSpeed(0, 2);
		fastDigitalWrite(STEPPER_ENABLE_PIN, HIGH);
		fastDigitalWrite(STEPPER_SLEEP_PIN, LOW);

		if(millis() > 3000 && fabsf(currentLeanAngle) < START_ANGLE_ERROR) {
			fastDigitalWrite(STEPPER_SLEEP_PIN, HIGH);
			delay(1);

			currentState = RobotState::IDLE;
			actualTargetSpeedSps = 0.0f;
			actualTurningSpeedSps = 0.0f;
			motorSpeedSps = 0.0f;
			turningSpeedSps = 0.0f;
			anglePID.resetPID();
			speedPID.resetPID();
			buzzer.buzz(10);
		}
		return;
	}

	actualTargetSpeedSps += constrain(targetSpeedSps - actualTargetSpeedSps, -SPEED_ACCELERATION, SPEED_ACCELERATION);
	actualTurningSpeedSps += constrain(turningSpeedSps - actualTurningSpeedSps, -TURN_ACCELERATION, TURN_ACCELERATION);

	float targetAngle = speedPID.updatePID(actualTargetSpeedSps, motorSpeedSps, deltaTime);
	motorSpeedSps = -anglePID.updatePID(targetAngle, currentLeanAngle, deltaTime);

	if(fabsf(currentLeanAngle - targetAngle) > MAX_ACCEPTABLE_ANGLE_ERROR ||
	   fabsf(currentLeanAngle) > MAX_ACCEPTABLE_ANGLE) {
		currentState = RobotState::DISARMED;
		fastDigitalWrite(STEPPER_ENABLE_PIN, HIGH);
		fastDigitalWrite(STEPPER_SLEEP_PIN, LOW);
		buzzer.buzz(10);
		return;
	}

	float rawSpeedMps = spsToMps(motorSpeedSps);
	float absSpeedMps = fabsf(rawSpeedMps);

	if(absSpeedMps > OVERSPEED_THRESHOLD) {
		float excessSpeedMps = absSpeedMps - OVERSPEED_THRESHOLD;
		float brakeDir = (rawSpeedMps > 0.0f) ? -1.0f : 1.0f;

		float brakeAmountMps = brakeDir * (OVERSPEED_BRAKE_AMOUNT * excessSpeedMps);
		actualTargetSpeedSps = mpsToSps(brakeAmountMps);
	}

	int16_t leftMotorSpeed = (motorSpeedSps + actualTurningSpeedSps) * MOTOR_SPEED_MULTIPLIER;
	int16_t rightMotorSpeed = (motorSpeedSps - actualTurningSpeedSps) * MOTOR_SPEED_MULTIPLIER;

	setMotorSpeed(rightMotorSpeed, 1);
	setMotorSpeed(leftMotorSpeed, 2);
	fastDigitalWrite(STEPPER_ENABLE_PIN, LOW);
}

void behaviour() {
	switch(currentState) {
		case RobotState::AVOID:
			avoid();
			break;

		case RobotState::FOLLOW:
			follow();
			break;

		case RobotState::CUSTOM_MODE:
			targetSpeedSps = 0.0f;
			turningSpeedSps = 0.0f;
			break;

		case RobotState::IDLE:
		case RobotState::DISARMED:
		case RobotState::LOW_BATTERY:
		default:
			targetSpeedSps = 0.0f;
			turningSpeedSps = 0.0f;
			break;
	}
}

void avoid() {
	static uint32_t turnEndTime = 0;
	uint32_t now = millis();

	if(now < turnEndTime) {
		targetSpeedSps = 0.0f;
		return;
	}

	float leftDistM = readSensorDistanceMeters(LEFTSENSOR_PIN);
	float rightDistM = readSensorDistanceMeters(RIGHTSENSOR_PIN);

	if(leftDistM < AVOID_TRIGGER_DIST_M || rightDistM < AVOID_TRIGGER_DIST_M) {
		targetSpeedSps = 0.0f;

		float turnDir = (rightDistM < leftDistM) ? 1.0f : -1.0f;
		turningSpeedSps = dpsToSps(turnDir * TURN_SPEED_DPS);

		turnEndTime = now + TURN_BASE_TIME + random(0, TURN_RANDOM_AMOUNT);
		return;
	}

	targetSpeedSps = mpsToSps(CRUISE_SPEED_MPS);

	if(leftDistM < AVOID_ADJUST_DIST_M || rightDistM < AVOID_ADJUST_DIST_M) {
		float distanceDiffM = rightDistM - leftDistM;
		float turnRateDps = distanceDiffM * (TURN_SPEED_DPS * 2.0f);
		turningSpeedSps = dpsToSps(turnRateDps);
	} else {
		turningSpeedSps = 0.0f;
	}
}

void follow() {
	float leftDistM = readSensorDistanceMeters(LEFTSENSOR_PIN);
	float rightDistM = readSensorDistanceMeters(RIGHTSENSOR_PIN);
	float closestDistM = min(leftDistM, rightDistM);

	if(closestDistM > FOLLOW_MAX_DIST_M) {
		targetSpeedSps = 0.0f;
		turningSpeedSps = 0.0f;
		return;
	}

	float steerDiffM = leftDistM - rightDistM;
	float turnRateDps = steerDiffM * (TURN_SPEED_DPS * 2.5f);
	turningSpeedSps = dpsToSps(turnRateDps);

	float distanceErrorM = closestDistM - FOLLOW_TARGET_DIST_M;
	float speedMps = constrain(distanceErrorM * 1.5f, REVERSE_SPEED_MPS, CRUISE_SPEED_MPS);
	targetSpeedSps = mpsToSps(speedMps);
}

/**
 * @brief Set the Motor Speed object
 *
 * @param tspeed
 * @param motorID
 */
void setMotorSpeed(int16_t tspeed, int8_t motorID) {
	uint32_t timer_period = 65535;
	int8_t dir = 0;

	if(tspeed != 0) {
		int16_t absSpeed = abs(tspeed);
		timer_period = 2000000UL / absSpeed;
		if(timer_period > 65535) timer_period = 65535;
		dir = (tspeed > 0) ? 1 : -1;
	}

	// 2. HARDWARE TIMER UPDATES (Protected by Spinlocks)
	if(motorID == 1) { // Right Motor
		portENTER_CRITICAL(&timerMuxR);
		directionMotorR = dir;
		if(dir == 1) fastDigitalWrite(STEPPER_R_DIR_PIN, LOW);
		else if(dir == -1) fastDigitalWrite(STEPPER_R_DIR_PIN, HIGH);

		// Core 2.x API for updating alarm limits and resetting counters
		timerAlarmWrite(timerR, timer_period, true);
		timerWrite(timerR, 0);
		portEXIT_CRITICAL(&timerMuxR);

	} else if(motorID == 2) { // Left Motor
		portENTER_CRITICAL(&timerMuxL);
		directionMotorL = dir;
		if(dir == 1) fastDigitalWrite(STEPPER_L_DIR_PIN, HIGH);
		else if(dir == -1) fastDigitalWrite(STEPPER_L_DIR_PIN, LOW);

		timerAlarmWrite(timerL, timer_period, true);
		timerWrite(timerL, 0);
		portEXIT_CRITICAL(&timerMuxL);
	}
}