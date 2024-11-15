#pragma once
#include "command.h"

 // The purpose of this class is toggle the GPIO pins on the Raspberry Pi bassed on a command object
// Requires information about wiring, ect.

class Command_Interpreter_RPi5
{
	void execute(Command command);

};

