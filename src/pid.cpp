#include "pid.h"

Pid::Pid(float p, float i, float d, float iLimit)
	: _kp(p), _ki(i), _kd(d), _iLimit(iLimit), _integratedError(0.0f), _lastError(0.0f) {}

float Pid::updatePID(float target, float current, float deltaTime) {
	if(deltaTime <= 0.0f) return 0.0f;

	float error = target - current;

	// Proportional term
	float pPart = _kp * error;

	// Integral term
	_integratedError += error * deltaTime;
	_integratedError = constrain(_integratedError, -_iLimit, _iLimit);
	float iPart = _ki * _integratedError;

	// Derivative term
	float dPart = _kd * ((error - _lastError) / deltaTime);
	_lastError = error;

	return pPart + iPart + dPart;
}

void Pid::resetPID() {
	_integratedError = 0.0f;
	_lastError = 0.0f;
}

void Pid::setP(float p) {
	_kp = p;
}
void Pid::setI(float i) {
	_ki = i;
}
void Pid::setD(float d) {
	_kd = d;
}
void Pid::setIlimit(float limit) {
	_iLimit = limit;
}

float Pid::getP() const {
	return _kp;
}
float Pid::getI() const {
	return _ki;
}
float Pid::getD() const {
	return _kd;
}
float Pid::getIlimit() const {
	return _iLimit;
}