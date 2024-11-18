// William Barber
#pragma once
#include "Command.h"
#include <vector>

// The purpose of this class is toggle the GPIO pins on the Raspberry Pi based on a command object
// Requires information about wiring, ect.
enum PinStatus {Enabled, Disabled};
enum EnableType {ActiveHigh, ActiveLow};
const int MAX_PWM_VALUE = 1023; //TODO: check pwm range (currently assuming 0-1023)

/*
 * NOTE: We may not need DigitalPin, in which case both DigitalPin and abstract Pin classes are not useful, and can
 * be replaced with just the PwmPin class (probably renamed to Pin). This would also necessitate the removal of allPins
 * and digitalPins data members from Command_Interpreter_RPi5
 */
class Pin {
protected:
    int gpioNumber{};
public:
    virtual void initialize()= 0;
    virtual void enable() = 0;
    virtual void disable() = 0;
    virtual bool enabled() = 0;
    explicit Pin(int gpioNumber) : gpioNumber(gpioNumber) {}
};

class DigitalPin : public Pin {
private:
    EnableType enableType;
    PinStatus pinStatus;
public:
    void initialize() override;
    void enable() override;
    void disable() override;
    bool enabled() override;
    DigitalPin(int gpioNumber, EnableType enableType);
};

class PwmPin : public Pin {
private:
    int currentPwm;
public:
    void initialize() override;
    void enable() override;
    void disable() override;
    bool enabled() override;
    void setPowerAndDirection(int pwmValue, Direction direction);
    explicit PwmPin(int gpioNumber);
};

class Command_Interpreter_RPi5 {
private:
    struct ThrusterSpec {
        int pwm_value;
        Direction direction;
    };
    std::vector<Pin*> allPins;
    std::vector<PwmPin*> thrusterPins;
    std::vector<DigitalPin*> digitalPins;
    static ThrusterSpec convertPwmValue(int pwmFrequency);
public:
    explicit Command_Interpreter_RPi5(std::vector<PwmPin*> thrusterPins, std::vector<DigitalPin*> digitalPins);
    void execute(const Command& command);
    void initializePins();
};