#ifndef COMPLEMENTARYFILTER_H
#define COMPLEMENTARYFILTER_H

class ComplementaryFilter {
  public:
	ComplementaryFilter();

	float calculate(float newAngle, float newRate, float dt);
	void resetValues();
	void setInitialAngle(float angle);

  private:
	// Self-documenting filter coefficients
	static constexpr float GYRO_WEIGHT = 0.995f;
	static constexpr float ACCEL_WEIGHT = 0.005f;

	float _angle;
};

#endif