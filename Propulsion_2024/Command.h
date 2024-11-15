#pragma once

// A command is a simple instruction to the vehicle
// Commands will be combined in sequencing to create more complex instructions
class Command
{
public:
	int thruster_pwms[8]; // PWM values for each thruster
	float duration;       // Duration of the command in seconds
	Command();
	~Command();


};

