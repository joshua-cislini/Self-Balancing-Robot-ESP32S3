#include <Wire.h>

#include "buzzer.h"
#include "complementaryFilter.h"
#include "config.h"
#include "mpu6050.h"
#include "pid.h"
#include "utils.h"

ComplementaryFilter angleFilter;
Mpu6050 IMU(MPU6050_ADDRESS);
Pid anglePID(ANGLE_P, ANGLE_I, ANGLE_D, ANGLE_I_LIMIT);
Pid speedPID(SPEED_P, SPEED_I, SPEED_D, SPEED_I_LIMIT);
Buzzer buzzer(BUZZER_PIN);

void batteryCheck();
void avoid();
void follow();
void balance();
void setMotorSpeed_Sps(int16_t speedSps, int8_t motorID);

RobotState currentState = RobotState::DISARMED;

uint32_t lastLoopTime = 0; // [us]
float deltaTime = 0.0f;	   // [s]

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
	while(!Serial && millis() < 3000);
	printBuildInfo();

	// analogReference(INTERNAL) removed: ESP32 uses 3.3V reference by default
	analogReadResolution(12); // Set ESP32 ADC resolution explicitly to match our math in config.h and utils.h
	randomSeed(analogRead(BATTERY_SENSE_PIN) ^ micros());
	batteryCheck();

	pinMode(SWITCH_1_PIN, INPUT_PULLUP);
	pinMode(SWITCH_2_PIN, INPUT_PULLUP);
	pinMode(STEPPER_R_STEP_PIN, OUTPUT);
	pinMode(STEPPER_R_DIR_PIN, OUTPUT);
	pinMode(STEPPER_L_STEP_PIN, OUTPUT);
	pinMode(STEPPER_L_DIR_PIN, OUTPUT);
	pinMode(STEPPER_ENABLE_PIN, OUTPUT);
	pinMode(STEPPER_SLEEP_PIN, OUTPUT);

	digitalWrite(STEPPER_ENABLE_PIN, HIGH); // !ENABLE = 1 --> DISABLED
	digitalWrite(STEPPER_SLEEP_PIN, LOW);	// !SLEEP = 0 --> SLEEPING

	// Initialize I2C. Note: On custom ESP32 boards, you may need to map SDA/SCL pins here:
	Wire.begin(); // Wire.begin(SDA_PIN, SCL_PIN);
	Wire.setClock(400000);

	IMU.begin();
	IMU.calibrateGyro();
	IMU.updateIMUdata();
	float initialAccAngle = atan2f(IMU.getAccelY(), IMU.getAccelZ()) * RAD_TO_DEG;
	angleFilter.setInitialAngle_DEG(initialAccAngle);
	IMU.serialPrintDebug();

	Serial.flush();
	lastLoopTime = micros();
}

void loop() {
	uint32_t currentLoopTime = micros();
	deltaTime = (float)(currentLoopTime - lastLoopTime) * 0.000001f; // [ms] --> [s]
	lastLoopTime = currentLoopTime;
	printLoopCycleTime();

	buzzer.update();
	batteryCheck();

	if(currentState != RobotState::LOW_BATTERY) {
		if(currentState != RobotState::DISARMED) {
			bool sw1 = digitalRead(SWITCH_1_PIN);
			bool sw2 = digitalRead(SWITCH_2_PIN);
			if(sw1 && sw2) {
				currentState = RobotState::IDLE;
				targetSpeedSps = 0.0f;
				turningSpeedSps = 0.0f;
			} else if(!sw1 && sw2) {
				currentState = RobotState::AVOID;
				avoid();
			} else if(sw1 && !sw2) {
				currentState = RobotState::FOLLOW;
				follow();
			} else {
				currentState = RobotState::CUSTOM_MODE; // Do whatever
				targetSpeedSps = 0.0f;
				turningSpeedSps = 0.0f;
			}
		}
		balance();
	} else {
		setMotorSpeed_Sps(0, 1);
		setMotorSpeed_Sps(0, 2);
		digitalWrite(STEPPER_ENABLE_PIN, HIGH);
		digitalWrite(STEPPER_SLEEP_PIN, LOW);
	}
}

void batteryCheck() {
	static uint32_t lastBatteryCheckTime = 0;
	static uint32_t lastBatteryWarnBeepTime = 0;
	static uint8_t lowVoltageCounter = 0;

	const uint32_t now = millis();
	if(now - lastBatteryCheckTime >= 1000) {
		lastBatteryCheckTime = now;

		uint16_t rawADC = analogRead(BATTERY_SENSE_PIN);
		float voltage = (float)rawADC * VOLTAGE_MULTIPLIER;

		if(voltage < MIN_BAT_VOLTAGE) {
			lowVoltageCounter++;
			if(lowVoltageCounter >= 5) {
				currentState = RobotState::LOW_BATTERY;
			}
		} else {
			lowVoltageCounter = 0;

			if(currentState == RobotState::LOW_BATTERY && voltage > WARN_BAT_VOLTAGE) {
				currentState = RobotState::DISARMED;
			} else if(currentState != RobotState::LOW_BATTERY && voltage < WARN_BAT_VOLTAGE) {
				if(now - lastBatteryWarnBeepTime > 5000) {
					lastBatteryWarnBeepTime = now;
					buzzer.buzz(100);
				}
			}
		}
	}
}

void balance() {
	IMU.updateIMUdata();
	const float accAngle = atan2f(IMU.getAccelY(), IMU.getAccelZ()) * RAD_TO_DEG;
	const float currentLeanAngle = angleFilter.calculate(accAngle, IMU.getGyroX(), deltaTime);

	const float targetAngle = speedPID.updatePID(actualTargetSpeedSps, motorSpeedSps, deltaTime);
	motorSpeedSps = -anglePID.updatePID(targetAngle, currentLeanAngle, deltaTime);

	if(fabsf(currentLeanAngle - targetAngle) > MAX_ACCEPTABLE_ANGLE_ERROR || fabsf(currentLeanAngle) > MAX_ACCEPTABLE_ANGLE) {
		currentState = RobotState::DISARMED;
		digitalWrite(STEPPER_ENABLE_PIN, HIGH);
		digitalWrite(STEPPER_SLEEP_PIN, LOW);
		buzzer.buzz(10);

		if(millis() > 3000 && fabsf(currentLeanAngle) < START_ANGLE_ERROR) {
			digitalWrite(STEPPER_SLEEP_PIN, HIGH);
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

	const float rawSpeedMps = SPS_To_MPS(motorSpeedSps);
	const float absSpeedMps = fabsf(rawSpeedMps);

	if(absSpeedMps > OVERSPEED_THRESHOLD) {
		const float excessSpeedMps = absSpeedMps - OVERSPEED_THRESHOLD;
		const float brakeDir = (rawSpeedMps > 0.0f) ? -1.0f : 1.0f;

		const float brakeAmountMps = brakeDir * (OVERSPEED_BRAKE_AMOUNT * excessSpeedMps);
		actualTargetSpeedSps = MPS_To_SPS(brakeAmountMps);
	}

	const int16_t leftMotorSpeedSPS = (motorSpeedSps + actualTurningSpeedSps) * MOTOR_SPEED_MULTIPLIER;
	const int16_t rightMotorSpeedSPS = (motorSpeedSps - actualTurningSpeedSps) * MOTOR_SPEED_MULTIPLIER;

	setMotorSpeed_Sps(rightMotorSpeedSPS, 1);
	setMotorSpeed_Sps(leftMotorSpeedSPS, 2);
	digitalWrite(STEPPER_ENABLE_PIN, LOW);
}

void avoid() {
	static uint32_t turnEndTime = 0;
	const uint32_t now = millis();

	if(now < turnEndTime) {
		targetSpeedSps = 0.0f;
		return;
	}

	const float leftDistM = readSensDist_M(LEFTSENSOR_PIN);
	const float rightDistM = readSensDist_M(RIGHTSENSOR_PIN);

	if(leftDistM < AVOID_TRIGGER_DIST_M || rightDistM < AVOID_TRIGGER_DIST_M) {
		targetSpeedSps = 0.0f;

		const float turnDir = (rightDistM < leftDistM) ? 1.0f : -1.0f;
		turningSpeedSps = DPS_To_SPS(turnDir * TURN_SPEED_DPS);

		turnEndTime = now + TURN_BASE_TIME + random(0, TURN_RANDOM_AMOUNT);
		return;
	}

	targetSpeedSps = MPS_To_SPS(CRUISE_SPEED_MPS);

	if(leftDistM < AVOID_ADJUST_DIST_M || rightDistM < AVOID_ADJUST_DIST_M) {
		const float distanceDiffM = rightDistM - leftDistM;
		const float turnRateDps = distanceDiffM * (TURN_SPEED_DPS * 2.0f);
		turningSpeedSps = DPS_To_SPS(turnRateDps);
	} else {
		turningSpeedSps = 0.0f;
	}
}

void follow() {
	const float leftDistM = readSensDist_M(LEFTSENSOR_PIN);
	const float rightDistM = readSensDist_M(RIGHTSENSOR_PIN);
	const float closestDistM = min(leftDistM, rightDistM);

	if(closestDistM > FOLLOW_MAX_DIST_M) {
		targetSpeedSps = 0.0f;
		turningSpeedSps = 0.0f;
		return;
	}

	const float steerDiffM = leftDistM - rightDistM;
	const float turnRateDps = steerDiffM * (TURN_SPEED_DPS * 2.5f);
	turningSpeedSps = DPS_To_SPS(turnRateDps);

	const float distanceErrorM = closestDistM - FOLLOW_TARGET_DIST_M;
	const float speedMps = constrain(distanceErrorM * 1.5f, REVERSE_SPEED_MPS, CRUISE_SPEED_MPS);
	targetSpeedSps = MPS_To_SPS(speedMps);
}

void setMotorSpeed_Sps(int16_t speedSps, int8_t motorID) {
	uint32_t timerPeriod = 65535;
	int8_t dir = 0;

	if(speedSps != 0) {
		int16_t absSpeed = abs(speedSps);
		timerPeriod = 2000000UL / absSpeed;
		if(timerPeriod > 65535) timerPeriod = 65535;
		dir = (speedSps > 0) ? 1 : -1;
	}

	// 2. HARDWARE TIMER UPDATES (Protected by Spinlocks)
	if(motorID == 1) { // Right Motor
		portENTER_CRITICAL(&timerMuxR);
		directionMotorR = dir;
		if(dir == 1) digitalWrite(STEPPER_R_DIR_PIN, LOW);
		else if(dir == -1) digitalWrite(STEPPER_R_DIR_PIN, HIGH);

		timerAlarm(timerR, timerPeriod, true, 0); // Core 3.x API for updating alarms and clearing hardware counters
		timerRestart(timerR);
		portEXIT_CRITICAL(&timerMuxR);
	} else if(motorID == 2) { // Left Motor
		portENTER_CRITICAL(&timerMuxL);
		directionMotorL = dir;
		if(dir == 1) digitalWrite(STEPPER_L_DIR_PIN, HIGH);
		else if(dir == -1) digitalWrite(STEPPER_L_DIR_PIN, LOW);

		timerAlarm(timerL, timerPeriod, true, 0);
		timerRestart(timerL);
		portEXIT_CRITICAL(&timerMuxL);
	}
}