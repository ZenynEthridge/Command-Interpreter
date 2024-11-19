#include "Command_Interpreter.cpp"
#include <gtest/gtest.h>

TEST(CommandInterpreterTest, CreateCommandInterpreter) {
    testing::internal::CaptureStdout();

    auto pin1 = new PwmPin(0);
    auto pin2 = new PwmPin(1);
    auto pin3 = new PwmPin(2);
    auto pin4 = new PwmPin(3);
    auto pin5 = new PwmPin(4);
    auto pin6 = new PwmPin(5);
    auto pin7 = new PwmPin(6);
    auto pin8 = new PwmPin(7);
    auto pins = std::vector<PwmPin *>{pin1, pin2, pin3, pin4, pin5, pin6, pin7, pin8};


    auto interpreter = new Command_Interpreter_RPi5(pins, std::vector<DigitalPin *>{});
    std::string output = testing::internal::GetCapturedStdout(); // TODO: check that output matches expected output

    delete interpreter;
}

TEST(CommandInterpreterTest, CreateCommandInterpreterWithDigitalPins) {
    testing::internal::CaptureStdout();

    auto pin1 = new PwmPin(0);
    auto pin2 = new PwmPin(1);
    auto pin3 = new PwmPin(2);
    auto pin4 = new PwmPin(3);
    auto pin5 = new PwmPin(4);
    auto pin6 = new PwmPin(5);
    auto pin7 = new PwmPin(6);
    auto pin8 = new PwmPin(7);
    auto pwmPins = std::vector<PwmPin *>{pin1, pin2, pin3, pin4, pin5, pin6, pin7, pin8};

    auto digital1 = new DigitalPin(8, ActiveLow);
    auto digital2 = new DigitalPin(9, ActiveHigh);
    auto digitalPins = std::vector<DigitalPin*>{digital1, digital2};

    auto interpreter = new Command_Interpreter_RPi5(pwmPins, digitalPins);
    std::string output = testing::internal::GetCapturedStdout(); // TODO: check that output matches expected output

    delete interpreter;
}

TEST(CommandInterpreterTest, ExecuteCommand) {
    testing::internal::CaptureStdout();

    struct Command command = {1500, 1900, 1100, 1250, 1300, 1464, 1535, 1536, 2};
    auto pin1 = new PwmPin(0);
    auto pin2 = new PwmPin(1);
    auto pin3 = new PwmPin(2);
    auto pin4 = new PwmPin(3);
    auto pin5 = new PwmPin(4);
    auto pin6 = new PwmPin(5);
    auto pin7 = new PwmPin(6);
    auto pin8 = new PwmPin(7);
    auto pins = std::vector<PwmPin *>{pin1, pin2, pin3, pin4, pin5, pin6, pin7, pin8};

    auto interpreter = new Command_Interpreter_RPi5(pins, std::vector<DigitalPin *>{});
    interpreter->execute(command);
    std::string output = testing::internal::GetCapturedStdout(); // TODO: check that output matches expected output


    delete interpreter;
}