// William Barber

#include "Wiring.h"

// When compiling for non-RPI devices which cannot run wiringPi library,
// use -MOCK_RPI flag to enable mock functions
#ifdef MOCK_RPI

#include "Mock_GPIO.h"

const int MAX_HARDWARE_PWM_VALUE = 1023;
const int MAX_SOFTWARE_PWM_VALUE = 100;

int scalePwm(int pwm, int maxPwmValue) {
    double pulseWidth = pwm;
    // 1100 - 1464 = negative
    // 1536 - 1900 = positive
    const int minPosPulseWidth = 1535; // slightly less than 1536 so that 1536 is not zero
    const int maxPosPulseWidth = 1900;
    const int minNegPulseWidth = 1100;
    const int maxNegPulseWidth = 1465; // slightly greater than 1464 so that 1464 is not zero
    int pwmMagnitude;
    double multiplier;

    if (pulseWidth >= 1465 && pulseWidth <= 1535) {
        return 0;
    }
    if (pulseWidth <= 1464) {
        multiplier = ((double)(maxPwmValue) / (double)(maxNegPulseWidth - minNegPulseWidth));
        pwmMagnitude = maxPwmValue - (int)((double)(pulseWidth - minNegPulseWidth) * multiplier);
    }
    else {
        multiplier = ((double)(maxPwmValue) / (double)(maxPosPulseWidth - minPosPulseWidth));
        pwmMagnitude = (int)((double)(pulseWidth - minPosPulseWidth) * multiplier);
    }
    return pwmMagnitude;
}


bool WiringControl::initializeGPIO() {
    if (mockSetupGpio() == 0) {
        return true;
    }
    else {
        return false;
    }
}

void WiringControl::setPinType(int pinNumber, PinType pinType) {
    switch (pinType) {
        case Digital:
            mockPinMode(pinNumber, OUTPUT);
            digitalPinStatuses[pinNumber] = Low;
            break;
        case HardwarePWM:
            pwmPinStatuses[pinNumber] = PwmPinStatus {scalePwm(1500, MAX_HARDWARE_PWM_VALUE), 300, 50};
            mockPinMode(pinNumber, PWM_OUTPUT);
            break;
        case SoftwarePWM:
            pwmPinStatuses[pinNumber] = PwmPinStatus {scalePwm(1500, MAX_SOFTWARE_PWM_VALUE), 300, 50};
            mockSoftPwmCreate(pinNumber, 0, 100);
            break;
        default:
            std::cerr << "Impossible pin type " << pinType << "! Exiting." << std::endl;
            exit(42);
    }
    pinTypes[pinNumber] = pinType;
}

void WiringControl::digitalWrite(int pinNumber, DigitalPinStatus digitalPinStatus) {
    switch (digitalPinStatus) {
        case Low:
            mockDigitalWrite(pinNumber, LOW);
            break;
        case High:
            mockDigitalWrite(pinNumber, HIGH);
            break;
        default:
            std::cerr << "Impossible digital pin status " << digitalPinStatus << "! Exiting." << std::endl;
            exit(42);
    }
    digitalPinStatuses[pinNumber] = digitalPinStatus;
}

DigitalPinStatus WiringControl::digitalRead(int pinNumber) {
    return digitalPinStatuses[pinNumber];
}

void WiringControl::pwmWrite(int pinNumber, int pwmFrequency) {
    switch (pinTypes[pinNumber]) {
        case HardwarePWM:
            mockPwmWrite(pinNumber, scalePwm(pwmFrequency, MAX_HARDWARE_PWM_VALUE));
            pwmPinStatuses[pinNumber].pulseWidth = scalePwm(pwmFrequency, MAX_HARDWARE_PWM_VALUE);
            break;
        case SoftwarePWM:
            mockSoftPwmWrite(pinNumber, scalePwm(pwmFrequency, MAX_SOFTWARE_PWM_VALUE));
            pwmPinStatuses[pinNumber].pulseWidth = scalePwm(pwmFrequency, MAX_SOFTWARE_PWM_VALUE);
            break;
        case Digital:
            std::cerr << "Invalid pin type \"Digital\". Digital pin type cannot be used for PWM. Exiting." << std::endl;
            exit(42);
            break;
        default:
            std::cerr << "Impossible pin type " << pinTypes[pinNumber] << "! Exiting." << std::endl;
            exit(42);
    }
}

void WiringControl::pwmWriteMaximum(int pinNumber) {
    pwmWrite(pinNumber, 1900);
}

void WiringControl::pwmWriteOff(int pinNumber) {
    pwmWrite(pinNumber, 1500);
}

PwmPinStatus WiringControl::pwmRead(int pinNumber) {
    return pwmPinStatuses[pinNumber];
}

#else

// William Barber

#include "Wiring.h"

namespace wiringPi {
#include <wiringPi.h>
#include <wiringSerial.h>
}
#include <iostream>
#include <string>

void printToSerial(std::string message, int serial) {
    if (serial == -1) {
        std::cout << message;
    }
    for (int i = 0; i < message.length(); i++) {
        serialPutChar(serial, message[i]);
    }
}


bool WiringControl::initializeGPIO() {
    if (wiringPi::wiringPiSetupGpio() < 0 || (serial = serialOpen("/dev/ttyAMA0", 115200) < 0)) {
        return false;
    }
}

void WiringControl::setPinType(int pinNumber, PinType pinType) {
    std::string message = "Set pin ";
    message.append(pinNumber);
    switch (pinType) {
        case Digital:scalePwm
            message.append(" to mode Digital\n");
            digitalWrite(pinNumber, Low);
            digitalPinStatuses[pinNumber] = Low;
            break;
        case HardwarePWM:
            message.append(" to mode HardPwm\n");
            pwmWrite(pinNumber, 1500);
            pwmPinStatuses[pinNumber] = PwmPinStatus {1500, 0};
            break;
        case SoftwarePWM:
            message.append(" to mode SoftPwm\n");
            pwmWrite(pinNumber, 1500);
            pwmPinStatuses[pinNumber] = PwmPinStatus {1500, 0};
            break;
        default:
            std::cerr << "Impossible pin type " << pinType << "! Exiting." << std::endl;
            exit(42);
    }
    printToSerial(message, serial);
    pinTypes[pinNumber] = pinType;
}

void WiringControl::digitalWrite(int pinNumber, DigitalPinStatus digitalPinStatus) {
    std::string message = "Set pin ";
    message.append(pinNumber);
    switch (digitalPinStatus) {
        case Low:
            message.append(" to digital Low\n")
            break;
        case High:
            message.append(" to digital High\n")
            wiringPi::digitalWrite(pinNumber, HIGH);
            break;
        default:
            std::cerr << "Impossible digital pin status " << digitalPinStatus << "! Exiting." << std::endl;
            exit(42);
    }
    printToSerial(message, serial);
    digitalPinStatuses[pinNumber] = digitalPinStatus;
}

DigitalPinStatus WiringControl::digitalRead(int pinNumber) {
    return digitalPinStatuses[pinNumber];
}

void WiringControl::pwmWrite(int pinNumber, int dutyCycle) {
    std::string message = "Set pin ";
    message.append(pinNumber);
    switch (pinTypes[pinNumber]) {
        case HardwarePWM:
        case SoftwarePWM:
            message.append(" to PWM duty cycle ")
            message.append(dutyCycle);
            message.append("\n");
            pwmPinStatuses[pinNumber].dutyCycle = dutyCycle;
            break;
        case Digital:
            std::cerr << "Invalid pin type \"Digital\". Digital pin type cannot be used for PWM. Exiting." << std::endl;
            exit(42);
        default:
            std::cerr << "Impossible pin type " << pinTypes[pinNumber] << "! Exiting." << std::endl;
            exit(42);
    }
    printToSerial(message, serial);
}

PwmPinStatus WiringControl::pwmRead(int pinNumber) {
    return pwmPinStatuses[pinNumber];
}

void WiringControl::pwmWriteMaximum(int pinNumber) {
    pwmWrite(pinNumber, 1900);
}

void WiringControl::pwmWriteOff(int pinNumber) {
    pwmWrite(pinNumber, 1500);
}

#endif