// William Barber
#include "Command_Interpreter.h"
#include <wiringPi.h> // this needs to be installed on the pi
#include <iostream>


DigitalPin::DigitalPin(int gpioNumber, EnableType enableType): Pin(gpioNumber), pinStatus(Disabled), enableType(enableType) {}

void DigitalPin::initialize() {
    pinMode(gpioNumber, OUTPUT);
}

void DigitalPin::enable() {
    switch (enableType) {
        case ActiveHigh:
            digitalWrite(gpioNumber, HIGH);
            break;
        case ActiveLow:
            digitalWrite(gpioNumber, LOW);
            break;
        default:
            std::cerr << "Unknown pin mode!" << std::endl;
    }
    pinStatus = Enabled;
}

void DigitalPin::disable() {
    switch (enableType) {
        case ActiveHigh:
            digitalWrite(gpioNumber, LOW);
            break;
        case ActiveLow:
            digitalWrite(gpioNumber, HIGH);
            break;
        default:
            std::cerr << "Unknown pin mode!" << std::endl;
    }
    pinStatus = Disabled;
}

bool DigitalPin::enabled() {
    return pinStatus == Enabled;
}


PwmPin::PwmPin(int gpioNumber): Pin(gpioNumber), currentPwm(0) {}

void PwmPin::initialize() {
    pinMode(gpioNumber, PWM_OUTPUT);
}

void PwmPin::enable() {
    setPowerAndDirection(255, Forwards);
}

void PwmPin::disable() {
    setPowerAndDirection(0, Forwards);
}

bool PwmPin::enabled() {
    return currentPwm != 0;
}

void PwmPin::setPowerAndDirection(int pwmValue, Direction direction) {
    //TODO: set direction (not sure how to do this w/ pwm. High/low?)
    pwmWrite(gpioNumber, pwmValue);
}



Command_Interpreter_RPi5::Command_Interpreter_RPi5(): pins(std::vector<Pin>{}) {}
Command_Interpreter_RPi5::Command_Interpreter_RPi5(std::vector<Pin> pins): pins(std::move(pins)) {}

void Command_Interpreter_RPi5::initializePins() {
    if (wiringPiSetupGpio() == -1) {
        std::cerr << "Failure to configure GPIO pins through wiringPi!" << std::endl;
    }
    for (Pin& pin : pins) {
        pin.initialize();
    }
}

void Command_Interpreter_RPi5::execute(Command command) {
    //TODO
}
