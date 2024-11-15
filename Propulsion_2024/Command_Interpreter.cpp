// William Barber
#include "Command_Interpreter.h"
#include <wiringPi.h> // this needs to be installed on the pi
#include <iostream>
#include <utility>

Pin::Pin(int gpioNumber): gpioNumber(gpioNumber), enableType(ActiveLow), pinStatus(Disabled) {}
Pin::Pin(int gpioNumber, EnableType enableType): gpioNumber(gpioNumber), enableType(enableType), pinStatus(Disabled) {}

void Pin::enable() {
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

void Pin::disable() {
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

bool Pin::enabled() {
    return pinStatus == Enabled;
}

Command_Interpreter_RPi5::Command_Interpreter_RPi5(): pins(std::vector<Pin>{}) {}
Command_Interpreter_RPi5::Command_Interpreter_RPi5(std::vector<Pin> pins): pins(std::move(pins)) {}

void Command_Interpreter_RPi5::initializePins() {
    wiringPiSetupGpio();
    for (Pin pin : pins) {
        pinMode(pin.gpioNumber, OUTPUT); // OUTPUT may be PWM_OUTPUT
    }
}
