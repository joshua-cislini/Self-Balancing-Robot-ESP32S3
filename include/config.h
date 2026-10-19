#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.2957795f
#endif

// --- Robot System States ---
enum class RobotState : uint8_t {
	LOW_BATTERY = 0,
	DISARMED = 1,
	IDLE = 2,
	AVOID = 3,
	FOLLOW = 4,
	CUSTOM_MODE = 5
};

// --- Wheel & Motion Geometry ---
constexpr float PI_F = (float)M_PI;
constexpr float WHEEL_RADIUS_M = 0.065f / 2.0f;
constexpr float TRACK_WIDTH_M = 0.095f;
constexpr float WHEEL_CIRCUMFERENCE_M = 2.0f * PI_F * WHEEL_RADIUS_M;
constexpr float STEPS_PER_REV = 3200.0f;

// --- Speed Conversion Multipliers ---
constexpr float MPS_TO_STEPS_PER_SEC = STEPS_PER_REV / WHEEL_CIRCUMFERENCE_M;
constexpr float STEPS_PER_SEC_TO_MPS = WHEEL_CIRCUMFERENCE_M / STEPS_PER_REV;

// Pin configuration (LilyGo T-Display-S3 AMOLED):
// TODO: Re-map these to the actual physical GPIOs you use, avoiding AMOLED/PSRAM pins
constexpr uint8_t STEPPER_R_STEP_PIN = 1;
constexpr uint8_t STEPPER_R_DIR_PIN = 2;
constexpr uint8_t STEPPER_L_STEP_PIN = 3;
constexpr uint8_t STEPPER_L_DIR_PIN = 4;
constexpr uint8_t STEPPER_ENABLE_PIN = 5;
constexpr uint8_t STEPPER_SLEEP_PIN = 6;
constexpr uint8_t BUZZER_PIN = 7;
constexpr uint8_t SWITCH_1_PIN = 8;
constexpr uint8_t SWITCH_2_PIN = 9;
constexpr uint8_t RX_LED_PIN = 10;
constexpr uint8_t BATTERY_SENSE_PIN = 11; // Must be an ADC1 channel
constexpr uint8_t RIGHTSENSOR_PIN = 12;	  // Must be an ADC1 channel
constexpr uint8_t LEFTSENSOR_PIN = 13;	  // Must be an ADC1 channel

// I2C address of the IMU
constexpr uint8_t MPU6050_ADDRESS = 0x68;

// PID constants for the angle PID controller
constexpr float ANGLE_P = 18.0f;
constexpr float ANGLE_I = 1.5f;
constexpr float ANGLE_D = 5.0f;
constexpr float ANGLE_I_LIMIT = 200.0f;

// PID constants for the speed PID controller
constexpr float SPEED_P = 900.0f;
constexpr float SPEED_I = 2.0f;
constexpr float SPEED_D = 0.1f;
constexpr float SPEED_I_LIMIT = 200.0f;

constexpr float MAX_ACCEPTABLE_ANGLE_ERROR = 35.0f;
constexpr float MAX_ACCEPTABLE_ANGLE = 40.0f;
constexpr float START_ANGLE_ERROR = 1.0f;

// Battery monitor configuration
constexpr float WARN_BAT_VOLTAGE = 10.5f;
constexpr float MIN_BAT_VOLTAGE = 10.0f;

// Re-calculated for ESP32 12-bit ADC (3.3V reference) replacing the 1.0f/36.80f AVR value
constexpr float VOLTAGE_MULTIPLIER = 0.00448f; // VOLTAGE_MULTIPLIER = Vactual/rawADC

// Motor acceleration
constexpr float SPEED_ACCELERATION = 0.008f;
constexpr float TURN_ACCELERATION = 0.005f;

constexpr float OVERSPEED_THRESHOLD = 1.2f;
constexpr float OVERSPEED_BRAKE_AMOUNT = 0.40f;

constexpr uint16_t TURN_BASE_TIME = 400;
constexpr uint16_t TURN_RANDOM_AMOUNT = 1200;
constexpr uint16_t MOTOR_SPEED_MULTIPLIER = 3600;

// --- IR Distance Sensor Thresholds (in Meters) ---
constexpr float AVOID_TRIGGER_DIST_M = 0.20f;
constexpr float AVOID_ADJUST_DIST_M = 0.30f;
constexpr float FOLLOW_MAX_DIST_M = 0.60f;
constexpr float FOLLOW_TARGET_DIST_M = 0.25f;

// --- Physical Speed Targets ---
constexpr float CRUISE_SPEED_MPS = 0.15f;
constexpr float REVERSE_SPEED_MPS = -0.15f;
constexpr float TURN_SPEED_DPS = 25.0f;

// --- Conversion Helper Functions ---
inline float mpsToSps(float speedMps) {
	return speedMps * MPS_TO_STEPS_PER_SEC;
}

inline float spsToMps(float speedSps) {
	return speedSps * STEPS_PER_SEC_TO_MPS;
}

inline float dpsToSps(float turnRateDps) {
	float radPerSec = turnRateDps * (PI_F / 180.0f);
	float wheelSpeedMps = -radPerSec * (TRACK_WIDTH_M * 0.5f);
	return mpsToSps(wheelSpeedMps);
}

inline float spsToDps(float turnSpeedSps) {
	float wheelSpeedMps = spsToMps(-turnSpeedSps);
	float radPerSec = wheelSpeedMps / (TRACK_WIDTH_M * 0.5f);
	return radPerSec * (180.0f / PI_F);
}

#endif // CONFIG_H