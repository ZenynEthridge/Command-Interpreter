#pragma once
#include "eigen-3.4.0/Eigen/Dense"

class Controls
{
protected:

	Eigen::Matrix<float, 1, 3> mass_center;
	Eigen::Matrix<float, 1, 3> volume_center;
	Eigen::Matrix<float, 8, 3> thruster_positions;
	Eigen::Matrix<float, 8, 3> thruster_moment_arms; // Distance from thruster to mass center (m)
	Eigen::Matrix<float, 8, 3> thruster_directions; // Direction of thruster force (unit vector)
	Eigen::Matrix<float, 8, 3> thruster_torques;   // Force of thruster (N)

	int num_thrusters; // Number of thrusters on the vehicle
	float mass;       // Mass of the vehicle (kg)
	float volume;    // Displacement volume of the vehicle (m^3)

public:
	Controls();
	~Controls();
	void set_mass_center(Eigen::Matrix<float, 1, 3> mass_center);
	void set_volume_center(Eigen::Matrix<float, 1, 3> volume_center);
	void set_thruster_position(Eigen::Matrix<float, 6, 3> thruster_position);
	Eigen::Matrix<float, 1, 3> get_mass_center();
	Eigen::Matrix<float, 1, 3> get_volume_center();
	Eigen::Matrix<float, 6, 3> get_thruster_position();

	void print_info();

};

