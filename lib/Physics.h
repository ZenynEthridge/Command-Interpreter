#pragma once
#include <cmath>
#include <stdexcept>
#include <iostream>

namespace physics {
	float accel_time(float vi, float vt, float cd, float m, float f);
	float accel_dist(float vi, float vt, float cd, float m, float f, float t);
}
