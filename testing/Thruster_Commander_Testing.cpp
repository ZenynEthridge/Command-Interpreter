#include "Thruster_Commander.cpp"
#include <gtest/gtest.h>

TEST(ThrusterCommanderTest, SimpleVertical) {
    testing::internal::CaptureStdout();
    Thruster_Commander thruster_commander = Thruster_Commander();
    thruster_commander.print_info();
    thruster_commander.thrust_compute_fz(5);

    std::string output = testing::internal::GetCapturedStdout(); // TODO: check that output matches expected output
}

TEST(ThrusterCommanderTest, GetPwm) {
    auto thrusterCommander = new Thruster_Commander();
    int result = thrusterCommander->get_pwm(1, -3);
    ASSERT_EQ(result, 1148);
    result = thrusterCommander->get_pwm(1, 1);
    ASSERT_EQ(result, 1652);
    delete thrusterCommander;
}
