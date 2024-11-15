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
	float rho_water = 1025; // Density of water (kg/m^3)

	
	// Returns the PWM value for a given thruster and force
	int get_pwm(int thruster_num, float force);


	// Drag force = 0.5 * rho_water * v^2 * Cd * A
	// rho water is known, A can be found as a function of direction, v is input, Cd is unknown, but can be estimated or found through trail and error
	Eigen::Matrix<float, 1, 3> predict_drag_forces(
		float x_velocity, float y_velocity, float z_velocity);
	
	// <drag_torque> = <r> x <drag_force> = <r> x < 0.5 * rho_water * (r*omega)^2 * Cd * A >
	// this will be a little trickier to calculate, might require integration
	Eigen::Matrix<float, 1, 3> predict_drag_torques(
		float x_velocity, float y_velocity, float z_velocity, float x_angular_velocity, 
		float y_angular_velocity, float z_angular_velocity);

public:
	Thruster_Commander();
	~Thruster_Commander();

	// mostly for debugging purposes
	void print_info();

	// functions begining with 'simple_' make the assumption that the vehicle is stable about the x and y axes
	// these functions only need to consider forces in the x, y, and z directions, and moments about the z axis
	Command simple_vertical(float distance);
	Command simple_forward(float distance);
	Command simple_sideways(float distance);
	Command simple_horizontal(float x_distance, float y_distance);
	
	Command simple_rotation(float angle);
	Command simple_rotating_vertical(float distance, float angle);
	Command simple_rotating_forward(float distance, float angle);
	Command simple_rotating_sideways(float distance, float angle);
	Command simple_rotating_horizontal(float x_distance, float y_distance, float angle);

	Command simple_3d(float x_distance, float y_distance, float z_distance);
	Command simple_rotating_3d(float x_distance, float y_distance, float z_distance, float angle);

};

