#ifndef PID_H
#define PID_H

#include <Arduino.h>

class Pid {
  public:
	Pid(float p, float i, float d, float iLimit);

	float updatePID(float target, float current, float deltaTime);
	void resetPID();

	void setP(float p);
	void setI(float i);
	void setD(float d);
	void setIlimit(float limit);

	float getP() const;
	float getI() const;
	float getD() const;
	float getIlimit() const;

  private:
	float _kp;
	float _ki;
	float _kd;
	float _iLimit;
	float _integratedError;
	float _lastError;
};

#endif