#include "Thruster_Commander.cpp"
#include <gtest/gtest.h>

TEST(ThrusterCommanderTest, SimpleVertical) {
    Thruster_Commander thruster_commander = Thruster_Commander();
    thruster_commander.print_info();
    thruster_commander.simple_vertical_forces(5);
    //TODO: Add assertion
}