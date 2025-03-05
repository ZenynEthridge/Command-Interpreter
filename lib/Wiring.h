// William Barber

#pragma once

#include <unordered_map>

enum PinType {Digital, HardwarePWM, SoftwarePWM};
enum DigitalPinStatus {Low, High};

struct PwmPinStatus {
    int pulseWidth;
    int frequency;
    int dutyCycle;
};

class WiringControl {
private:
    static int serial = -1;
    std::unordered_map<int, PinType> pinTypes;
    std::unordered_map<int, PwmPinStatus> pwmPinStatuses;
    std::unordered_map<int, DigitalPinStatus> digitalPinStatuses;
public:
   bool initializeGPIO();
   void setPinType(int pinNumber, PinType pinType);
   void digitalWrite(int pinNumber, DigitalPinStatus digitalPinStatus);
   DigitalPinStatus digitalRead(int pinNumber);
   void pwmWrite(int pinNumber, int pwmFrequency);
   PwmPinStatus pwmRead(int pinNumber);
   void pwmWriteMaximum(int pinNumber);
   void pwmWriteOff(int pinNumber);
   WiringControl() = default;
};
