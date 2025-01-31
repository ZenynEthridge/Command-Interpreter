// William Barber
#include "Command_Interpreter.h"
#include <fstream>
#include <ctime>
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
//TODO: wrapper class for wiringPi GTest?
#include <map>
#include <iostream>
std::map<int,int> pinStatus = {};

int wiringPiSetupGpio() {
    std::cout << "[Mock] wiringPi GPIO set up!" << std::endl;
    return 0;
}

void softPwmCreate(int pinNumber, int startValue, int max) {
    std::cout << "Create software pwm pin number " << pinNumber << " in range [0," << max << "] and starting at "
    << startValue << std::endl;
}

void pinMode(int pinNumber, int mode) {
    if (pinStatus.find(pinNumber) == pinStatus.end()) {
        pinStatus[pinNumber] = 0;
    }
    std::cout << "[Mock] pinMode: pin " << pinNumber << " set to mode " << mode << std::endl;
}

void digitalWrite(int pinNumber, int voltage) {
    pinStatus[pinNumber] = voltage;
    std::cout << "[Mock] digitalWrite: pin " << pinNumber << ", voltage " << voltage << std::endl;
}

void pwmWrite(int pinNumber, int pwm) {
    pinStatus[pinNumber] = pwm;
    std::cout << "[Mock] pwmWrite: pin " << pinNumber << ", PWM " << pwm << std::endl;
}

void softPwmWrite(int pinNumber, int pwm) {
    pinStatus[pinNumber] = pwm;
    std::cout << "[Mock] softPwmWrite: pin" << pinNumber << ", PWM " << pwm << std::endl;
}

int digitalRead(int pinNumber) {
    return (pinStatus.find(pinNumber) != pinStatus.end() || pinStatus[pinNumber] > 1) ? pinStatus[pinNumber] : -1;
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
            std::cerr << "Impossible pin mode!" << std::endl;
            exit(42);
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
            std::cerr << "Impossible pin mode!" << std::endl;
            exit(42);
    }
    pinStatus = Disabled;
}

bool DigitalPin::enabled() {
    return pinStatus == Enabled;
}

int DigitalPin::read() {
    return digitalRead(gpioNumber);
}

void PwmPin::setPwm(int frequency, std::ofstream &logFile) {
    ThrusterSpec thrusterSpec = convertPwmToThrusterSpec(frequency);
    setPowerAndDirection(thrusterSpec);
    std::time_t currentTime = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    logFile << "Current time: " << std::ctime(&currentTime) << std::endl;
    logFile << "Thruster at pin " << gpioNumber << ": " << thrusterSpec.pwm_value << std::endl;
}

HardwarePwmPin::HardwarePwmPin(int gpioNumber): PwmPin(gpioNumber) {}

void HardwarePwmPin::initialize() {
    pinMode(gpioNumber, PWM_OUTPUT);
}

void HardwarePwmPin::enable() {
    setPowerAndDirection(ThrusterSpec{MAX_HARDWARE_PWM_VALUE, Forwards});
}

void HardwarePwmPin::disable() {
    setPowerAndDirection(ThrusterSpec{0, Forwards});
}

bool HardwarePwmPin::enabled() {
    return currentPwm != 0;
}

void HardwarePwmPin::setPowerAndDirection(ThrusterSpec thrusterSpec) {
    //TODO: set direction (not sure how to do this w/ pwm. High/low?)
    pwmWrite(gpioNumber, thrusterSpec.pwm_value);
    currentPwm = thrusterSpec.pwm_value;
}

int HardwarePwmPin::read() {
    return currentPwm;
}

ThrusterSpec HardwarePwmPin::convertPwmToThrusterSpec(int pwm) {
    double pwmFrequency = pwm;
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
        multiplier = ((double)(MAX_HARDWARE_PWM_VALUE) / (double)(maxNegPwmFrequency - minNegPwmFrequency));
        pwmMagnitude = MAX_HARDWARE_PWM_VALUE - (int)((double)(pwmFrequency - minNegPwmFrequency) * multiplier);
        return ThrusterSpec{pwmMagnitude, Backwards};
    }
    else {
        multiplier = ((double)(MAX_HARDWARE_PWM_VALUE) / (double)(maxPosPwmFrequency - minPosPwmFrequency));
        pwmMagnitude = (int)((double)(pwmFrequency - minPosPwmFrequency) * multiplier);
        return ThrusterSpec{pwmMagnitude, Forwards};
    }
}


SoftwarePwmPin::SoftwarePwmPin(int gpioNumber): PwmPin(gpioNumber) {}

void SoftwarePwmPin::initialize() {
    pinMode(gpioNumber, OUTPUT);
    softPwmCreate(gpioNumber, 0, 100); //TODO: Verify that this is correct
}

void SoftwarePwmPin::enable() {
    setPowerAndDirection(ThrusterSpec{MAX_SOFTWARE_PWM_VALUE, Forwards});
}

void SoftwarePwmPin::disable() {
    setPowerAndDirection(ThrusterSpec{0, Forwards});
}

bool SoftwarePwmPin::enabled() {
    return currentPwm != 0;
}

void SoftwarePwmPin::setPowerAndDirection(ThrusterSpec thrusterSpec) {
    softPwmWrite(gpioNumber, thrusterSpec.pwm_value);
    currentPwm = thrusterSpec.pwm_value;
    //TODO: work out how to do direction
}

int SoftwarePwmPin::read() {
    return currentPwm;
}

ThrusterSpec SoftwarePwmPin::convertPwmToThrusterSpec(int pwm) {
    double pwmFrequency = pwm;
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
        multiplier = ((double)(MAX_SOFTWARE_PWM_VALUE) / (double)(maxNegPwmFrequency - minNegPwmFrequency));
        pwmMagnitude = MAX_SOFTWARE_PWM_VALUE - (int)((double)(pwmFrequency - minNegPwmFrequency) * multiplier);
        return ThrusterSpec{pwmMagnitude, Backwards};
    }
    else {
        multiplier = ((double)(MAX_SOFTWARE_PWM_VALUE) / (double)(maxPosPwmFrequency - minPosPwmFrequency));
        pwmMagnitude = (int)((double)(pwmFrequency - minPosPwmFrequency) * multiplier);
        return ThrusterSpec{pwmMagnitude, Forwards};
    }
}


Command_Interpreter_RPi5::Command_Interpreter_RPi5(std::vector<PwmPin*> thrusterPins, std::vector<DigitalPin*> digitalPins):
                                                thrusterPins(std::move(thrusterPins)), digitalPins(std::move(digitalPins)) {
    if (this->thrusterPins.size() != 8) {
        std::cerr << "Incorrect number of thruster pwm pins given! Need 8, given " << this->thrusterPins.size() << std::endl;
        exit(42);
    }
}

std::vector<Pin*> Command_Interpreter_RPi5::allPins() {
    auto allPins = std::vector<Pin*>{};
    allPins.insert(allPins.end(), this->thrusterPins.begin(), this->thrusterPins.end());
    allPins.insert(allPins.end(), this->digitalPins.begin(), this->digitalPins.end());
    return allPins;
}

void Command_Interpreter_RPi5::initializePins() {
    if (wiringPiSetupGpio() == -1) {
        std::cerr << "Failure to configure GPIO pins through wiringPi!" << std::endl;
        exit(42);
    }
    for (Pin* pin : allPins()) {
        pin->initialize();
    }
}

void Command_Interpreter_RPi5::execute(pwm_array thrusterPwms, std::ofstream& logFile) {
    int i = 0;
    for (int frequency : thrusterPwms.pwm_signals) {
        thrusterPins.at(i)->setPwm(frequency, logFile);
        i++;
    }
}

std::vector<int> Command_Interpreter_RPi5::readPins() {
    std::vector<int> pinValues;
    for (auto pin : allPins()) {
        pinValues.push_back(pin->read());
    }
    return pinValues;
}

Command_Interpreter_RPi5::~Command_Interpreter_RPi5() {
    for (auto pin: allPins()) {
        delete pin;
    }
}

//move_forwards(float distance, std::ofstream logfile) {
//    command = calculate_stuff(distance, current_robot_data);
//    blind_execute(command.accel_command, logfile); //accel
//    blind_execute(command.ss_command, logfile); //stead-state
//    blind_execute(command.dec_command, logfile); //decel
//    // At this point, a new thread/command sends new commands. This can be stop, or it can be a new direction, etc.
//}

void Command_Interpreter_RPi5::blind_execute(const CommandComponent& commandComponent, std::ofstream& logfile) {
    auto endTime = std::chrono::system_clock::now() + commandComponent.duration;
    auto currentTime = std::chrono::system_clock::now();
    execute(commandComponent.thruster_pwms, logfile);
    while (currentTime < endTime) {
        currentTime = std::chrono::system_clock::now();
    }
}

//void Command_Interpreter_RPi5::corrective_execute(command_component command, std::ofstream logfile) {
//    adjusted_command = copy(command)
//    while (!time_is_up()) {
//        adjusted_command = correct_command();
//        execute(command);
//    }
//}
