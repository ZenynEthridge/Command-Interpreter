// William Barber
#pragma once
#include "Command.h"
#include <vector>
#include <fstream>

/// @brief Whether a pin is currently in an enabled (powered) or disabled (unpowered) state
enum PinStatus {Enabled, Disabled};

///@brief Whether a digital pin is active high or active low
enum EnableType {ActiveHigh, ActiveLow};

/// @brief The spec for a thruster command to be sent to GPIO
struct ThrusterSpec {
    int pwm_value;
    Direction direction;
};

const int MAX_HARDWARE_PWM_VALUE = 1023; //TODO: check pwm range (currently assuming 0-1023)
const int MAX_SOFTWARE_PWM_VALUE = 100; //TODO: check pwm range for software

/*
 * NOTE: We may not need DigitalPin, in which case both DigitalPin and abstract Pin classes are not useful, and can
 * be replaced with just the HardwarePwmPin class (probably renamed to Pin). This would also necessitate the removal of allPins
 * and digitalPins data members from Command_Interpreter_RPi5
 */

/// @brief A Raspberry Pi 5 GPIO pin, as specified by its GPIO pin number (see https://pinout.xyz)
class Pin {
protected:
    int gpioNumber{};
public:
    /// @brief Initializes pin through WiringPi (i.e. to output, PWM, etc.)
    virtual void initialize()= 0;

    /// @brief Sets pin to maximum (positive) power
    virtual void enable() = 0;

    /// @brief Sets pin to be unpowered (stopped)
    virtual void disable() = 0;

    /// @brief Whether a pin is currently powered (whether its power level is not zero)
    /// @return True if power is not zero, false otherwise
    virtual bool enabled() = 0;

    /// @brief The pin's current state
    /// @return The current pin status
    virtual int read() = 0;
    explicit Pin(int gpioNumber) : gpioNumber(gpioNumber) {}
    virtual ~Pin() = default;
};

/// @brief A digital (two-state) Raspberry Pi 5 GPIO pin
class DigitalPin : public Pin {
private:
    EnableType enableType;
    PinStatus pinStatus; //TODO: can remove this and just rely on digitalReads from the pins directly
public:
    void initialize() override;
    void enable() override;
    void disable() override;
    bool enabled() override;
    int read() override;
    DigitalPin(int gpioNumber, EnableType enableType);
};

/// @brief A PWM pin which may or may not be hardware-supported
class PwmPin : public Pin {
protected:
    int currentPwm;
    /// @brief Converts a pwm frequency between 1100 and 1900 into a Thruster Spec.
    /// @param pwm a pwm frequency between 1100 and 1900
    /// @return A Thruster Spec with a pwm magnitude between 0 and the max pwm value, and a Direction
    virtual ThrusterSpec convertPwmToThrusterSpec(int pwm) = 0;

    /// @brief Sets pin to the specified pwm value and direction
    /// @param thrusterSpec a spec dictating the power and direction for the thruster
    virtual void setPowerAndDirection(ThrusterSpec thrusterSpec) = 0;

public:
    /// @brief Sets pin to given pwm frequency
    /// @param frequency the desired frequency, between 1100 and 1900
    /// @param logFile a file stream to write activity to for post-match analysis
    virtual void setPwm(int frequency, std::ofstream& logFile);
    explicit PwmPin(int gpioNumber) : Pin(gpioNumber), currentPwm(0) {}
    virtual ~PwmPin() = default;
};

/// @brief a pwm-capable Raspberry Pi 5 GPIO pin (supports analogue output)
class HardwarePwmPin : public PwmPin {
protected:
    /// @brief Converts a pwm frequency between 1100 and 1900 into a Thruster Spec.
    /// @param pwm a pwm frequency between 1100 and 1900
    /// @return A Thruster Spec with a pwm magnitude between 0 and MAX_HARDWARE_PWM_VALUE and a Direction
    ThrusterSpec convertPwmToThrusterSpec(int pwm) override;

    /// @brief Sets pin to the specified pwm value and direction
    /// @param thrusterSpec the pin specifications, with a pwm magnitude between 0 and MAX_HARDWARE_PWM_VALUE and a Direction
    void setPowerAndDirection(ThrusterSpec thrusterSpec) override;
public:
    void initialize() override;
    void enable() override;
    void disable() override;
    bool enabled() override;

    int read() override;
    explicit HardwarePwmPin(int gpioNumber);
};

/// @brief a Raspberry Pi 5 GPIO pin that doesn't natively support PWM, but that will simulate analogue output
/// through software pwm control.
class SoftwarePwmPin : public PwmPin {
protected:
    /// @brief Converts a pwm frequency between 1100 and 1900 into a Thruster Spec.
    /// @param pwm a pwm frequency between 1100 and 1900
    /// @return A Thruster Spec with a pwm magnitude between 0 and MAX_HARDWARE_PWM_VALUE and a Direction
    ThrusterSpec convertPwmToThrusterSpec(int pwm) override;
public:
    void initialize() override;
    void enable() override;
    void disable() override;
    bool enabled() override;

    /// @brief Sets pin to the specified pwm value and direction
    /// @param thrusterSpec the pin specifications, with a pwm magnitude between 0 and MAX_HARDWARE_PWM_VALUE and a Direction
    void setPowerAndDirection(ThrusterSpec thrusterSpec) override;
    int read() override;
    explicit SoftwarePwmPin(int gpioNumber);
};

/// @brief The purpose of this class is toggle the GPIO pins on the Raspberry Pi based on a command object.
/// Requires information about wiring, etc.
class Command_Interpreter_RPi5 {
private:
    std::vector<Pin*> allPins();
    std::vector<PwmPin*> thrusterPins;
    std::vector<DigitalPin*> digitalPins;

public:
    explicit Command_Interpreter_RPi5(std::vector<PwmPin*> thrusterPins, std::vector<DigitalPin*> digitalPins);

    /// @brief Initializes the pins in allPins through the WiringPi library to be either PWM or digital pins
    /// depending on their types
    void initializePins();

    /// @brief Executes a command by sending the specified pwm values to the Pi's GPIO for the specified duration
    /// @param command a command struct with a C-style array of pwm frequency integers and a duration float
    /// @param logFile a file stream to write activity to for post-match analysis
    void execute(const CommandComponent& command, std::ofstream& logfile);

    /// @brief Executes a command without self-correction. Sets pwm values for the duration specified. Does not stop
    /// thrusters after execution.
    /// @param command a command struct with three sub-components: the acceleration, steady-state, and deceleration.
    /// @param logFile a file stream to write activity to for post-match analysis
    void blind_execute(const CommandComponent& command, std::ofstream& logfile);

    /// @brief Get the current pwm values of all the pins.
    /// @return A vector containing the current value of all pins. PWM pins will return a value in their range, which
    /// varies depending on the type.
    std::vector<int> readPins();

    ~Command_Interpreter_RPi5(); //TODO this also deletes all its pins. Not sure if this is desirable or not?
};

