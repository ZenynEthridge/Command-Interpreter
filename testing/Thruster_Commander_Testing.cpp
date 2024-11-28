#include "Thruster_Commander.h"
#include <gtest/gtest.h>


TEST(ThrusterCommanderTest, Accel_Time_From_Zero_x) {
	auto control = new Thruster_Commander();
	
	float v = 0.9 * control->top_speed_x(true);
	float t = control->accel_time_from_zero_x(v);

    // math makes no sense currently
    delete control;
}
TEST(ThrusterCommanderTest, Top_Speed_X) {
	auto control = new Thruster_Commander();
	float max_fwd = control->top_speed_x(true);
	float max_rev = control->top_speed_x(false);

    // TODO: calculate asserts based on config file
	ASSERT_TRUE(max_fwd > 0);
	ASSERT_TRUE(max_rev < 0);
}

TEST(ThrusterCommanderTest, Thrust_Compute_Fx) {
	auto control = new Thruster_Commander();

    // TODO: decrease tolerances by using vertical thrusters to correct mx, my
	six_axis tolerances = { 0.001, 0.001, 0.001, 0.5, 0.5, 0.001 };
    six_axis desired = { 3,0,0,0,0,0 };
    thruster_set thrusts = control->thrust_compute_fx(desired(0));
    six_axis result = control->net_force_from_thrusters(thrusts);
	for (int i = 0; i < 6; i++) {
		ASSERT_NEAR(result(i), desired(i), tolerances(i));
	}
    delete control;
}
TEST(ThrusterCommanderTest, Thrust_Compute_Fy) {
    auto control = new Thruster_Commander();
    
    // TODO: decrease tolerances by using vertical thrusters to correct mx, my
    six_axis tolerances = { 0.001, 0.001, 0.001, 0.5, 0.5, 0.1 };
    six_axis desired = { 0,3,0,0,0,0 };
    thruster_set thrusts = control->thrust_compute_fy(desired(1));
    six_axis result = control->net_force_from_thrusters(thrusts);
    for (int i = 0; i < 6; i++) {
        ASSERT_NEAR(result(i), desired(i), tolerances(i));
    }
    delete control;
}
TEST(ThrusterCommanderTest, Thrust_Compute_Fz) {
    auto control = new Thruster_Commander();

    six_axis tolerances = { 0.001, 0.001, 0.001, 0.1, 0.1, 0.001 };
    six_axis desired = { 0,0,3,0,0,0 };
    thruster_set thrusts = control->thrust_compute_fz(desired(2));
    six_axis result = control->net_force_from_thrusters(thrusts);

    for (int i = 0; i < 6; i++) {
        ASSERT_NEAR(result(i), desired(i), tolerances(i));
    }
    delete control;
}
TEST(ThrusterCommanderTest, GetPwmNoInterpolation) {
    auto thrusterCommander = new Thruster_Commander();
    double result = thrusterCommander->get_pwm(1, -3.52);
    ASSERT_DOUBLE_EQ(result, 1100);

    result = thrusterCommander->get_pwm(1, -3);
    ASSERT_DOUBLE_EQ(result, 1148);

    result = thrusterCommander->get_pwm(1, -0.03);
    ASSERT_DOUBLE_EQ(result, 1464);

    result = thrusterCommander->get_pwm(1, 0);
    ASSERT_DOUBLE_EQ(result, 1468);

    result = thrusterCommander->get_pwm(1, 0.05);
    ASSERT_DOUBLE_EQ(result, 1536);

    result = thrusterCommander->get_pwm(1, 1);
    ASSERT_DOUBLE_EQ(result, 1652);

    result = thrusterCommander->get_pwm(1, 4.52);
    ASSERT_DOUBLE_EQ(result, 1900);

    result = thrusterCommander->get_pwm(1, 4.53);
    ASSERT_DOUBLE_EQ(result, 1896);

    delete thrusterCommander;
}

TEST(ThrusterCommanderTest, GetPwmWithInterpolation) {
    auto thrusterCommander = new Thruster_Commander();
    double result = thrusterCommander->get_pwm(1, -3.51);
    ASSERT_NEAR(result, 1102.0000, 0.1);

    result = thrusterCommander->get_pwm(1, -3.46);
    ASSERT_NEAR(result, 1111.0000, 0.1);

    result = thrusterCommander->get_pwm(1, -2.99);
    ASSERT_NEAR(result, 1148.6667, 0.1);

    result = thrusterCommander->get_pwm(1, -0.264);
    ASSERT_NEAR(result, 1419.4667, 0.1);

    result = thrusterCommander->get_pwm(1, -0.029);
    ASSERT_NEAR(result, 1464.1333, 0.1);

    result = thrusterCommander->get_pwm(1, -0.01);
    ASSERT_NEAR(result, 1466.6667, 0.1);

    result = thrusterCommander->get_pwm(1, 0.02);
    ASSERT_NEAR(result, 1533.6000, 0.1);

    result = thrusterCommander->get_pwm(1, 0.051);
    ASSERT_NEAR(result, 1536.4000, 0.1);

    result = thrusterCommander->get_pwm(1, 1.1);
    ASSERT_NEAR(result, 1660.8000, 0.1);

    result = thrusterCommander->get_pwm(1, 4.5);
    ASSERT_NEAR(result, 1891.5556, 0.1);

    result = thrusterCommander->get_pwm(1, 4.525);
    ASSERT_NEAR(result, 1898.0000, 0.1);

    delete thrusterCommander;
}

TEST(ThrusterCommanderTest, GetPwmOutOfBounds) {
    testing::internal::CaptureStderr();
    auto thrusterCommander = new Thruster_Commander();
    double result = thrusterCommander->get_pwm(1, -3.521);
    ASSERT_DOUBLE_EQ(result, 1100);

    result = thrusterCommander->get_pwm(1, -500);
    ASSERT_DOUBLE_EQ(result, 1100);

    result = thrusterCommander->get_pwm(1, 4.531);
    ASSERT_DOUBLE_EQ(result, 1896);

    result = thrusterCommander->get_pwm(1, 500);
    ASSERT_DOUBLE_EQ(result, 1896);

    delete thrusterCommander;
}


TEST(ThrusterCommandTest, BasicSequenceTo) {
    auto thrusterCommander = new Thruster_Commander();
    six_axis target = { 5, 5, 5, 0, 0, 0 };
    command_sequence sequence = thrusterCommander->basic_sequence(target);

    // TODO: run pencil and paper calulations to determine if the sequence is correct
    // TODO: add asserts
    delete thrusterCommander;
}
TEST(ThrusterCommanderTest, Thrust_Compute_fz) {
    testing::internal::CaptureStdout();
    Thruster_Commander thruster_commander = Thruster_Commander();
    thruster_commander.print_info();
    thruster_commander.thrust_compute_fz(5);

    std::string output = testing::internal::GetCapturedStdout(); // TODO: check that output matches expected output
}