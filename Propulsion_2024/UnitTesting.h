#include <iostream>
#include <vector>
#include <stdexcept>
#include <cmath>
#include "Thruster_Commander.h"
#include "eigen-3.4.0/Eigen/Dense"
#include "Command.h"
#include "Command_Interpreter.h"


// a value of zero indicates success
// please write and save your tests here


int command_interpreter_test_0() 
{
    //Begin command/command interpreter testing
    struct Command command = { 1500, 1900, 1100, 1250, 1300, 1464, 1535, 1536, 2 };
    auto pin1 = new PwmPin(0);
    auto pin2 = new PwmPin(1);
    auto pin3 = new PwmPin(2);
    auto pin4 = new PwmPin(3);
    auto pin5 = new PwmPin(4);
    auto pin6 = new PwmPin(5);
    auto pin7 = new PwmPin(6);
    auto pin8 = new PwmPin(7);
    auto pins = std::vector<PwmPin*>{ pin1, pin2, pin3, pin4, pin5, pin6, pin7, pin8 };
    auto interpreter = new Command_Interpreter_RPi5(pins, std::vector<DigitalPin*>{});
    interpreter->execute(command);
    delete pin1;
    delete pin2;
    delete pin3;
    delete pin4;
    delete pin5;
    delete pin6;
    delete pin7;
    delete pin8;
    delete interpreter;
    //End command/command interpreter testing

    return 0;
}

int thruster_commander_test_0()
{
	Thruster_Commander thruster_commander = Thruster_Commander();
	thruster_commander.print_info();
	return 0;
}