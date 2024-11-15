// William Barber
#include "Command_Interpreter.h"
#include <wiringPi.h> // this needs to be installed on the pi

void Command_Interpreter_RPi5::initializePins() {
    wiringPiSetupGpio();
    for (Pin pin : pins) {
        pinMode(pin.gpioNumber, OUTPUT); // OUTPUT may be PWM_OUTPUT
    }
}
