#pragma once
#include "command.h"
#include <vector>

// The purpose of this class is toggle the GPIO pins on the Raspberry Pi based on a command object
// Requires information about wiring, ect.
enum PinStatus {Enabled, Disabled};
enum EnableType {ActiveHigh, ActiveLow};
enum Direction {Forwards, Backwards};

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
    std::vector<Pin> pins;
public:
    Command_Interpreter_RPi5();
    Command_Interpreter_RPi5(std::vector<Pin> pins);
    void execute(Command command);
    void initializePins();
};