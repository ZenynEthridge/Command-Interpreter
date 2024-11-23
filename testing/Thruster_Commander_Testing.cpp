#include "Thruster_Commander.cpp"
#include <gtest/gtest.h>

TEST(ThrusterCommanderTest, SimpleVertical) {
    testing::internal::CaptureStdout();
    Thruster_Commander thruster_commander = Thruster_Commander();
    thruster_commander.print_info();
    thruster_commander.thrust_compute_fz(5);

    std::string output = testing::internal::GetCapturedStdout(); // TODO: check that output matches expected output
}