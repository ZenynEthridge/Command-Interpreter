// William Barber
#include "Command_Interpreter.h"
#include <iostream>
#include <utility>

// When compiling for non-RPI devices which cannot run wiringPi library,  
// use -MOCK_RPI flag to enable mock functions
#ifndef MOCK_RPI
#include <wiringPi.h>  // Include wiringPi library by default

#else
#define PWM_OUTPUT 1
#define OUTPUT 0
#define HIGH 1
#define LOW 0

int wiringPiSetupGpio() {
    std::cout << "[Mock] wiringPi GPIO set up!" << std::endl;
    return 0;
}

void pinMode(int pinNumber, int mode) {
    std::cout << "[Mock] pinMode: pin " << pinNumber << " set to mode " << mode << std::endl;
}

void digitalWrite(int pinNumber, int voltage) {
    std::cout << "[Mock] digitalWrite: pin " << pinNumber << ", voltage " << voltage << std::endl;
}

void pwmWrite(int pinNumber, int pwm) {
    std::cout << "[Mock] pwmWrite: pin " << pinNumber << ", PWM " << pwm << std::endl;
}

#endif

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
            std::cerr << "Impossible pin mode!" << std::endl; //throw exception?
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
            std::cerr << "Impossible pin mode!" << std::endl; //throw exception?
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
    setPowerAndDirection(MAX_PWM_VALUE, Forwards);
    currentPwm = MAX_PWM_VALUE;
}

void PwmPin::disable() {
    setPowerAndDirection(0, Forwards);
    currentPwm = 0;
}

bool PwmPin::enabled() {
    return currentPwm != 0;
}

void PwmPin::setPowerAndDirection(int pwmValue, Direction direction) {
    //TODO: set direction (not sure how to do this w/ pwm. High/low?)
    pwmWrite(gpioNumber, pwmValue);
    currentPwm = pwmValue;
}



Command_Interpreter_RPi5::Command_Interpreter_RPi5(std::vector<PwmPin*> thrusterPins, std::vector<DigitalPin*> digitalPins):
                                                thrusterPins(std::move(thrusterPins)), digitalPins(std::move(digitalPins)) {
    if (this->thrusterPins.size() != 8) {
        std::cerr << "Incorrect number of thruster pwm pins given! Need 8, given " << this->thrusterPins.size() << std::endl;
    }
    allPins = std::vector<Pin*>{};
    allPins.insert(allPins.end(), this->thrusterPins.begin(), this->thrusterPins.end());
    allPins.insert(allPins.end(), this->digitalPins.begin(), this->digitalPins.end());
    initializePins();
}

void Command_Interpreter_RPi5::initializePins() {
    if (wiringPiSetupGpio() == -1) {
        std::cerr << "Failure to configure GPIO pins through wiringPi!" << std::endl; //throw exception?
    }
    for (Pin* pin : allPins) {
        pin->initialize();
    }
}

Command_Interpreter_RPi5::ThrusterSpec Command_Interpreter_RPi5::convertPwmValue(int pwmFrequency) {
    /*
     * Converts a pwm frequency between 1100 and 1900 into a Thruster Spec.
     * @param pwmFrequency: a pwm frequency between 1100 and 1900
     * @return: a Thruster Spec with a pwm magnitude between 0 and MAX_PWM_VALUE and a Direction
     */
    // 1100 - 1464 = negative
    // 1536 - 1900 = positive
    const int minPosPwmFrequency = 1535; // slightly less than 1536 so that 1536 is not zero
    const int maxPosPwmFrequency = 1900;
    const int minNegPwmFrequency = 1100;
    const int maxNegPwmFrequency = 1465; // slightly greater than 1464 so that 1464 is not zero
    int pwmMagnitude;
    double multiplier;

    if (pwmFrequency >= 1465 && pwmFrequency <= 1535) {
        return ThrusterSpec{0, Forwards};
    }
    if (pwmFrequency <= 1464) {
        multiplier = ((double)(MAX_PWM_VALUE)/(double)(maxNegPwmFrequency - minNegPwmFrequency));
        pwmMagnitude = MAX_PWM_VALUE - (int)((double)(pwmFrequency - minNegPwmFrequency) * multiplier);
        return ThrusterSpec{pwmMagnitude, Backwards};
    }
    else {
        multiplier = ((double)(MAX_PWM_VALUE)/(double)(maxPosPwmFrequency - minPosPwmFrequency));
        pwmMagnitude = (int)((double)(pwmFrequency - minPosPwmFrequency) * multiplier);
        return ThrusterSpec{pwmMagnitude, Forwards};
    }
}

void Command_Interpreter_RPi5::execute(const Command& command) {
    int i = 0;
    for (int frequency : command.thruster_pwms.pwm_signals) {
        ThrusterSpec thrusterSpec = convertPwmValue(frequency);
        thrusterPins.at(i)->setPowerAndDirection(thrusterSpec.pwm_value, thrusterSpec.direction);
        i++;
    }
    //TODO: duration
}
