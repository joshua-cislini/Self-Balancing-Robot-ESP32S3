#include "complementaryFilter.h"

// Use initializer list for clean memory allocation
ComplementaryFilter::ComplementaryFilter()
  : _angle(0.0f) {}

float ComplementaryFilter::calculate(float newAngle, float newRate, float dt) {
  // Execute filter math using the named compile-time constants
  _angle = GYRO_WEIGHT * (_angle + newRate * dt) + ACCEL_WEIGHT * newAngle;
  return _angle;
}

void ComplementaryFilter::resetValues() {
  _angle = 0.0f;
}

void ComplementaryFilter::setInitialAngle(float angle) {
  _angle = angle;
}