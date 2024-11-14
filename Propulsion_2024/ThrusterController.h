#pragma once
#include "eigen-3.4.0/Eigen/Dense"
#include "command.h"
#include <vector>


// The purpose of this class is to generate command objects
// A command object is a simple instruction to the vehicle
class Thruster_Commander
{
protected:

	Eigen::Matrix<float, 1, 3> mass_center;
	Eigen::Matrix<float, 1, 3> volume_center;
	Eigen::Matrix<float, 8, 3> thruster_positions;
	Eigen::Matrix<float, 8, 3> thruster_moment_arms; // Distance from thruster to mass center (m)
	Eigen::Matrix<float, 8, 3> thruster_directions; // Direction of thruster force (unit vector)
	Eigen::Matrix<float, 8, 3> thruster_torques;   // Force of thruster (N)
	Eigen::Matrix<float, 8, 1> thruster_voltages;  // Torque of thruster (N*m)

	int num_thrusters; // Number of thrusters on the vehicle
	float mass;       // Mass of the vehicle (kg)
	float volume;    // Displacement volume of the vehicle (m^3)

	
	// Returns the PWM value for a given thruster and force
	int get_pwm(int thruster_num, float force);

public:
	Thruster_Commander();
	~Thruster_Commander();
	void set_mass_center(Eigen::Matrix<float, 1, 3> mass_center);
	void set_volume_center(Eigen::Matrix<float, 1, 3> volume_center);
	void set_thruster_position(Eigen::Matrix<float, 6, 3> thruster_position);
	Eigen::Matrix<float, 1, 3> get_mass_center();
	Eigen::Matrix<float, 1, 3> get_volume_center();
	Eigen::Matrix<float, 6, 3> get_thruster_position();

	void print_info();

	Command simple_vertical(float distance);
	Command simple_forward(float distance);
	Command simple_sideways(float distance);
	Command simple_z_rotation(float angle);

};

