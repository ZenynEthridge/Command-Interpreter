// William Barber
#include <iostream>
#include <fstream>
#include <ctime>
#include <utility>
#include "Serial.h"
#include "Command_Interpreter.h"
#include "Wiring.h"

void DigitalPin::initialize(WiringControl &wiringControl) {
    switch (enableType) {
        case ActiveLow:
            wiringControl.setPinType(gpioNumber, DigitalActiveLow);
            break;
        case ActiveHigh:
            wiringControl.setPinType(gpioNumber, DigitalActiveHigh);
            break;
        default:
            errorLog << "Impossible digital pin type " << enableType << "! Exiting." << std::endl;
            exit(42);
    }
}

void DigitalPin::enable(WiringControl &wiringControl) {
    switch (enableType) {
        case ActiveHigh:
            wiringControl.digitalWrite(gpioNumber, High);
            break;
        case ActiveLow:
            wiringControl.digitalWrite(gpioNumber, Low);
            break;
        default:
            errorLog << "Impossible pin mode!" << std::endl;
            exit(42);
    }
}

void DigitalPin::disable(WiringControl &wiringControl) {
    switch (enableType) {
        case ActiveHigh:
            wiringControl.digitalWrite(gpioNumber, Low);
            break;
        case ActiveLow:
            wiringControl.digitalWrite(gpioNumber, High);
            break;
        default:
            errorLog << "Impossible pin mode!" << std::endl;
            exit(42);
    }
}

bool DigitalPin::enabled(WiringControl &wiringControl) {
    switch (enableType) {
        case ActiveHigh:
            return wiringControl.digitalRead(gpioNumber) == High;
        case ActiveLow:
            return wiringControl.digitalRead(gpioNumber) == Low;
        default:
            errorLog << "Impossible pin enable type! Exiting." << std::endl;
            exit(42);
    }
}

int DigitalPin::read(WiringControl &wiringControl) {
    return wiringControl.digitalRead(gpioNumber);
}

void PwmPin::setPwm(int pulseWidth, WiringControl &wiringControl) {
    setPowerAndDirection(pulseWidth, wiringControl);
    std::time_t currentTime = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    outLog << "Current time: " << std::ctime(&currentTime) << std::endl;
    outLog << "Thruster at pin " << gpioNumber << ": " << pulseWidth << std::endl;
}


void HardwarePwmPin::initialize(WiringControl &wiringControl) {
    wiringControl.setPinType(gpioNumber, HardwarePWM);
}

void HardwarePwmPin::enable(WiringControl &wiringControl) {
    wiringControl.pwmWriteMaximum(gpioNumber);
}

void HardwarePwmPin::disable(WiringControl &wiringControl) {
    wiringControl.pwmWriteOff(gpioNumber);
}

bool HardwarePwmPin::enabled(WiringControl &wiringControl) {
    return wiringControl.pwmRead(gpioNumber).pulseWidth != 1500;
}

void HardwarePwmPin::setPowerAndDirection(int pwmValue, WiringControl &wiringControl) {
    wiringControl.pwmWrite(gpioNumber, pwmValue);
}

int HardwarePwmPin::read(WiringControl &wiringControl) {
    return wiringControl.pwmRead(gpioNumber).pulseWidth;
}


void SoftwarePwmPin::initialize(WiringControl &wiringControl) {
    wiringControl.setPinType(gpioNumber, SoftwarePWM);
}

void SoftwarePwmPin::enable(WiringControl &wiringControl) {
    wiringControl.pwmWriteMaximum(gpioNumber);
}

void SoftwarePwmPin::disable(WiringControl &wiringControl) {
    wiringControl.pwmWriteOff(gpioNumber);
}

bool SoftwarePwmPin::enabled(WiringControl &wiringControl) {
    return wiringControl.pwmRead(gpioNumber).pulseWidth != 1500;
}

void SoftwarePwmPin::setPowerAndDirection(int pwmValue, WiringControl &wiringControl) {
    wiringControl.pwmWrite(gpioNumber, pwmValue);
}

int SoftwarePwmPin::read(WiringControl &wiringControl) {
    return wiringControl.pwmRead(gpioNumber).pulseWidth;
}

Command_Interpreter_RPi5::Command_Interpreter_RPi5(std::vector<PwmPin *> thrusterPins,
                                                   std::vector<DigitalPin *> digitalPins,
                                                   const WiringControl &wiringControl, std::ostream &output,
                                                   std::ostream &outLog, std::ostream &errorLog) :
        thrusterPins(std::move(thrusterPins)), digitalPins(std::move(digitalPins)), wiringControl(wiringControl),
        errorLog(errorLog), outLog(outLog), output(output), isInterruptBlind_Execute(false) {
    if (this->thrusterPins.size() != 8) {
        errorLog << "Incorrect number of thruster pwm pins given! Need 8, given " << this->thrusterPins.size()
                 << std::endl;
        exit(42);
    }
}

std::vector<Pin *> Command_Interpreter_RPi5::allPins() {
    auto allPins = std::vector<Pin *>{};
    allPins.insert(allPins.end(), this->thrusterPins.begin(), this->thrusterPins.end());
    allPins.insert(allPins.end(), this->digitalPins.begin(), this->digitalPins.end());
    return allPins;
}

void Command_Interpreter_RPi5::initializePins() {
    if (!wiringControl.initializeSerial()) {
        errorLog << "Failure to configure serial!" << std::endl;
        exit(42);
    }
    for (Pin *pin: allPins()) {
        pin->initialize(wiringControl);
    }
}

std::vector<int> Command_Interpreter_RPi5::readPins() {
    std::vector<int> pinValues;
    for (auto pin: allPins()) {
        pinValues.push_back(pin->read(wiringControl));
    }
    return pinValues;
}

Command_Interpreter_RPi5::~Command_Interpreter_RPi5() {
    for (auto pin: allPins()) {
        delete pin;
    }
}

void Command_Interpreter_RPi5::blind_execute(const CommandComponent &commandComponent) {
    isInterruptBlind_Execute = false;
    auto endTime = std::chrono::system_clock::now() + commandComponent.duration;
    auto currentTime = std::chrono::system_clock::now();
    untimed_execute(commandComponent.thruster_pwms);
    while (currentTime < endTime && !isInterruptBlind_Execute) {
        currentTime = std::chrono::system_clock::now();
    }
    isInterruptBlind_Execute = false;
}

void Command_Interpreter_RPi5::untimed_execute(pwm_array thrusterPwms) {
    int i = 0;
    for (int pulseWidth: thrusterPwms.pwm_signals) {
        thrusterPins.at(i)->setPwm(pulseWidth, wiringControl);
        i++;
    }
}

void Command_Interpreter_RPi5::untimed_execute(const std::array<int, 8>& pwms){
    pwm_array a;
    std::copy(pwms.begin(), pwms.end(), a.pwm_signals);
    untimed_execute(a);

}
