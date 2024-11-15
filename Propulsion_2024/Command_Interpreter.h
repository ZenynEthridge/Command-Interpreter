#pragma once
#include "command.h"
#include <vector>

// The purpose of this class is toggle the GPIO pins on the Raspberry Pi based on a command object
// Requires information about wiring, ect.
enum PinStatus {Enabled, Disabled};
enum EnableType {ActiveHigh, ActiveLow};

class Pin {
private:
    int gpioNumber;
    PinStatus pinStatus;
    EnableType enableType;
public:
    Pin(int gpioNumber);
    Pin(int gpioNumber, EnableType enableType);
    void enable();
    void disable(); //if we are using PWM, we'll want to change this to "set pwm value" rather than enable/disable
    bool enabled();
    friend class Command_Interpreter_RPi5;
};

class Command_Interpreter_RPi5 {
private:
    std::vector<Pin> pins;
public:
    Command_Interpreter_RPi5();
    Command_Interpreter_RPi5(std::vector<Pin> pins);
    void execute(Command command);
    void initializePins();
};
