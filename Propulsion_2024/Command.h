#pragma once

// A command is a simple instruction to the vehicle
// Commands will be combined in sequencing to create more complex instructions

enum Direction {Forwards, Backwards};

struct ThrusterSpec {
    int pwm_frequency;
    Direction direction;
};

struct ThrusterSpecArray {
    ThrusterSpec thruster_specs[8]; // array of thruster frequencies and directions
};

struct Command {
    ThrusterSpecArray thruster_specs; // array of thruster frequencies and directions
    float duration;       // Duration of the command in seconds
};

