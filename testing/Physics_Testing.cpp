#include "Physics.h"
#include <gtest/gtest.h>


TEST(PhysicsTest, accel_time) {

	float cd = 40;
	float m = 5;
	float F[6] =  { 30,  50, -30, 0, 0 };
	float v0[6] = { 0,   0,    0, 5, 1 };
	float vt[6] = { 0.5, 0,    1, 1, 5 };
	float t[6];
	
	for (int i = 0; i < 6; i++) { t[i] = accel_time(v0[i], vt[i], cd, m, F[i]); }

	ASSERT_TRUE(t[0] > 0);
	ASSERT_TRUE(t[1] == 0);
	ASSERT_TRUE(t[2] < 0);
	ASSERT_TRUE(t[3] > 0);
	ASSERT_TRUE(t[4] < 0);
}