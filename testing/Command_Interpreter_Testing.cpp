#include "Command_Interpreter.cpp"
#include "Parser.h"
#include <gtest/gtest.h>

TEST(CommandInterpreterTest, CreateCommandInterpreter) {
    testing::internal::CaptureStdout();

    auto expectedPinSetEvents = std::vector<PinSetEvent>{PinSetEvent{0,1}, PinSetEvent{1,1},
                                                         PinSetEvent{2,1}, PinSetEvent{3,1},
                                                         PinSetEvent{4,1}, PinSetEvent{5,1},
                                                         PinSetEvent{6,1}, PinSetEvent{7,1}};
    auto expectedResult = ParseInterpreterResult{true, expectedPinSetEvents, std::vector<PinWriteEvent>{}};


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
    interpreter->initializePins();
    std::string output = testing::internal::GetCapturedStdout();
    auto result = parseInterpreterOutput(output);

    delete interpreter;

    ASSERT_TRUE(result.wiringPiSetUp);
    ASSERT_EQ(result, expectedResult);
}

TEST(CommandInterpreterTest, CreateCommandInterpreterWithDigitalPins) {
    testing::internal::CaptureStdout();

    auto expectedPinSetEvents = std::vector<PinSetEvent>{PinSetEvent{0,1}, PinSetEvent{1,1},
                                                         PinSetEvent{2,1}, PinSetEvent{3,1},
                                                         PinSetEvent{4,1}, PinSetEvent{5,1},
                                                         PinSetEvent{6,1}, PinSetEvent{7,1},
                                                         PinSetEvent{8,0}, PinSetEvent{9,0}};
    auto expectedResult = ParseInterpreterResult{true, expectedPinSetEvents, std::vector<PinWriteEvent>{}};


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
    interpreter->initializePins();
    std::string output = testing::internal::GetCapturedStdout();
    auto result = parseInterpreterOutput(output);

    delete interpreter;

    ASSERT_TRUE(result.wiringPiSetUp);
    ASSERT_EQ(result, expectedResult);

}

TEST(CommandInterpreterTest, ExecuteCommand) {
    testing::internal::CaptureStdout();
    auto expectedPinSetEvents = std::vector<PinSetEvent>{PinSetEvent{0,1}, PinSetEvent{1,1},
                                                         PinSetEvent{2,1}, PinSetEvent{3,1},
                                                         PinSetEvent{4,1}, PinSetEvent{5,1},
                                                         PinSetEvent{6,1}, PinSetEvent{7,1}};
    auto expectedPinWriteEvents = std::vector<PinWriteEvent>{PinWriteEvent{0,0}, PinWriteEvent{1,1023},
                                                             PinWriteEvent{2,1023}, PinWriteEvent{3,603},
                                                             PinWriteEvent{4,463}, PinWriteEvent{5,3},
                                                             PinWriteEvent{6,0}, PinWriteEvent{7,2}};
    auto expectedResult = ParseInterpreterResult{true, expectedPinSetEvents, expectedPinWriteEvents};

    struct Command command = {1500, 1900, 1100,
            1250, 1300, 1464, 1535,
            1536, 2};
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
    interpreter->initializePins();
    interpreter->execute(command);
    std::string output = testing::internal::GetCapturedStdout();
    auto result = parseInterpreterOutput(output);

    delete interpreter;

    ASSERT_TRUE(result.wiringPiSetUp);
    ASSERT_EQ(result, expectedResult);
}