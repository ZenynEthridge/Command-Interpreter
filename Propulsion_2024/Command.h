#pragma once
#include <map>

// A command is a simple instruction to the vehicle
// Commands will be combined in sequencing to create more complex instructions

enum Direction {Forwards, Backwards};

struct pwm_array {
		int pwm_signals[8];
	};
struct Command
{
public:
    std::map<int, Direction>thruster_values; // map of pwm values to thruster directions
    float duration;       // Duration of the command in seconds
};

