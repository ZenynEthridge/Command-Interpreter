// William Barber

#include "Wiring.h"

#include <iostream>
#include <string>

// When compiling for non-RPI devices which cannot run wiringPi library,
// use -MOCK_RPI flag to enable mock functions
#ifdef MOCK_RPI

#else

#include <wiringPi.h>
#include <wiringSerial.h>

bool WiringControl::initializeGPIO() {
    if (wiringPi::wiringPiSetupGpio() < 0 || (serial = serialOpen("/dev/ttyAMA0", 115200) < 0)) {
        return false;
    }
}

void printToSerial(std::string message, int serial) {
    if (serial == -1) {
        std::cout << message;
    }
    else {
        for (int i = 0; i < message.length(); i++) {
            serialPutChar(serial, message[i]);
        }
    }
}

#endif

bool WiringControl::initializeGPIO() {
    return true;
}


void printToSerial(std::string message, int serial) {
    if (serial == -1) {
        std::cout << message;
    }
    else {
        for (int i = 0; i < message.length(); i++) {
            putchar(message[i]);
        }
    }
}

void WiringControl::setPinType(int pinNumber, PinType pinType) {
    std::string message = "Set pin ";
    message.append(std::to_string(pinNumber));
    switch (pinType) {
        case Digital:
            message.append(" to mode Digital\n");
            printToSerial(message, serial);
            pinTypes[pinNumber] = Digital;
            digitalWrite(pinNumber, Low);
            digitalPinStatuses[pinNumber] = Low;
            break;
        case HardwarePWM:
            message.append(" to mode HardPwm\n");
            printToSerial(message, serial);
            pinTypes[pinNumber] = HardwarePWM;
            pwmWrite(pinNumber, 1500);
            pwmPinStatuses[pinNumber] = PwmPinStatus {1500, 0};
            break;
        case SoftwarePWM:
            message.append(" to mode SoftPwm\n");
            printToSerial(message, serial);
            pinTypes[pinNumber] = SoftwarePWM;
            pwmWrite(pinNumber, 1500);
            pwmPinStatuses[pinNumber] = PwmPinStatus {1500, 0};
            break;
        default:
            std::cerr << "Impossible pin type " << pinType << "! Exiting." << std::endl;
            exit(42);
    }
    pinTypes[pinNumber] = pinType;
}

void WiringControl::digitalWrite(int pinNumber, DigitalPinStatus digitalPinStatus) {
    std::string message = "Set pin ";
    message.append(std::to_string(pinNumber));
    switch (digitalPinStatus) {
        case Low:
            message.append(" to digital Low\n");
            printToSerial(message, serial);
            digitalPinStatuses[pinNumber] = Low;
            break;
        case High:
            message.append(" to digital High\n");
            printToSerial(message, serial);
            digitalPinStatuses[pinNumber] = High;
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

void WiringControl::pwmWrite(int pinNumber, int pulseWidth) {
    std::string message = "Set pin ";
    message.append(std::to_string(pinNumber));
    switch (pinTypes[pinNumber]) {
        case HardwarePWM:
        case SoftwarePWM:
            message.append(" to PWM pulse width ");
            message.append(std::to_string(pulseWidth));
            message.append("\n");
            printToSerial(message, serial);
            pwmPinStatuses[pinNumber].pulseWidth = pulseWidth;
            break;
        case Digital:
            std::cerr << "Invalid pin type \"Digital\". Digital pin type cannot be used for PWM. Exiting." << std::endl;
            exit(42);
        default:
            std::cerr << "Impossible pin type " << pinTypes[pinNumber] << "! Exiting." << std::endl;
            exit(42);
    }
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