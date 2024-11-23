// Zenyn Ethridge, 2024
// zjethridge@ucdavis.edu

#include <iostream>
#include <cmath>
#include <string>
#include "Thruster_Commander.h"
#include "eigen-3.4.0/Eigen/Dense"

Thruster_Commander::Thruster_Commander()
{
	// TODO: Move all the hardcoded values in this constructor to a config file
	//       this will make unit testing simpler

	// Values come from Onshape 2024 Vehicle V10 11/12/24
	num_thrusters = 8;
	three_axis mass_center_inches = {0.466, 0, 1.561};
	mass_center = mass_center_inches * 0.0254; // convert to meters

	volume_center = mass_center; // volume center, currently, is a complete guess
	volume_center(0, 2) += 0.1;	 // add 0.1 meters to z coordinate to account for the volume center being slightly above the mass center

	// avg(max distance, min distance) of motor part 4 cylindrical surface to orgin
	// front left top, front right top, rear left top, rear right top, front left bottom, front right bottom, rear left bottom, rear right bottom
	// x, y, z coordinates here are how the appear on onshape. May need to be corrected to match surge, sway, heave

	thruster_positions = thruster_set_3D::Zero();
	thruster_positions.row(0) << .2535, -.2035, .042;
	thruster_positions.row(1) << .2535, .2035, -.042;
	thruster_positions.row(2) << -.2545, -.2035, .042;
	thruster_positions.row(3) << -.2545, .2035, .042;
	thruster_positions.row(4) << .167, -.1375, -.049;
	thruster_positions.row(5) << .167, .1375, -.049;
	thruster_positions.row(6) << -.1975, -.1165, -.049;
	thruster_positions.row(7) << -.1975, .1165, -.049;

	// torques will be calulated about the center of mass
	thruster_moment_arms = thruster_positions - mass_center.replicate(8, 1);

	// directionality recorded as the direction the front of thruster is facing
	// force direction will be reversed
	const double PI = 3.141592653589793;
	float sin45 = sin(45 * PI / 180);
	thruster_directions = thruster_set_3D::Zero();
	thruster_directions.row(0) << 0, 0, 1;
	thruster_directions.row(1) << 0, 0, 1;
	thruster_directions.row(2) << 0, 0, 1;
	thruster_directions.row(3) << 0, 0, 1;
	thruster_directions.row(4) << -sin45, -sin45, 0;
	thruster_directions.row(5) << -sin45, sin45, 0;
	thruster_directions.row(6) << -sin45, sin45, 0;
	thruster_directions.row(7) << -sin45, -sin45, 0;

	thruster_torques = Eigen::Matrix<float, 8, 3>::Zero();
	for (int i = 0; i < num_thrusters; i++)
	{
		thruster_torques.row(i) = thruster_moment_arms.row(i).cross(thruster_directions.row(i));
	}

	float volume_inches = 449.157;			 // volume of vehicle in inches^3, from onshape. This is likley less than the displacement volume and should be corrected
	volume = volume_inches * pow(0.0254, 3); // convert to meters^3
	mass = 5.51;							 // mass of vehicle in kg, from onshape
	gravity = -9.81;
	rho_water = 1025; // Density of water (kg/m^3)
	weight_magnitude = mass * gravity;
	buoyant_magnitude = -rho_water * gravity * volume;
	// all zeros for now
	position = three_axis::Zero();
	orientation = three_axis::Zero();
	velocity = three_axis::Zero();
	angular_velocity = three_axis::Zero();
	acceleration = three_axis::Zero();
	angular_acceleration = three_axis::Zero();
}
Thruster_Commander::~Thruster_Commander() {}

void Thruster_Commander::print_info()
{

	std::cout << "Mass Center: \n"
			  << mass_center << std::endl;
	std::cout << "Volume Center: \n"
			  << volume_center << std::endl;
	std::cout << "Thruster Positions: \n"
			  << thruster_positions << std::endl;
	std::cout << "Thruster Moment Arms: \n"
			  << thruster_moment_arms << std::endl;
	std::cout << "Thruster Directions: \n"
			  << thruster_directions << std::endl;
	std::cout << "Thruster Torques: \n"
			  << thruster_torques << std::endl;
	std::cout << "Mass: \n"
			  << mass << std::endl;
	std::cout << "Volume: \n"
			  << volume << std::endl;
}
int Thruster_Commander::get_pwm(int thruster_num, float force)
{
	//   std::ifstream dataset("data/14V_PWM_Correlation.csv"); // Replace with your CSV file name
	//   std::string line;
	// std::string PWM;
	// int PWM_value;

	//   // Get the header line (if exists)
	//   if (std::getline(file, line)) {
	//	for( auto s: line){
	//	while(s != ','){
	//		PWM += s;
	//	}
	//	}
	//	PWM_value = stoi(PWM);
	//       while (low <= high) {
	//       int mid = low + (high - low) / 2;

	//       // Check if x is present at mid
	//       if (arr[mid] == x)
	//           return mid;

	//       // If x greater, ignore left half
	//       if (arr[mid] < x)
	//           low = mid + 1;

	//       // If x is smaller, ignore right half
	//       else
	//           high = mid - 1;
	//   }
	return 1500;
}

six_axis Thruster_Commander::weight_force(three_axis orientation)
{
	six_axis result = six_axis::Zero();
	result.segment(0, 3) = weight_magnitude * three_axis::UnitZ();
	std::cout << "Weight force: \n"
			  << result << std::endl;

	result.segment(0, 3) *= -Eigen::AngleAxisf(orientation(0),
											   Eigen::Vector3f::UnitX())
								 .toRotationMatrix();
	result.segment(0, 3) *= -Eigen::AngleAxisf(orientation(1),
											   Eigen::Vector3f::UnitY())
								 .toRotationMatrix();
	result.segment(0, 3) *= -Eigen::AngleAxisf(orientation(2),
											   Eigen::Vector3f::UnitZ())
								 .toRotationMatrix();

	std::cout << "Weight force: \n"
			  << result << std::endl;
	return result;
}
six_axis Thruster_Commander::bouyant_force(three_axis orientation)
{
	six_axis result = six_axis::Zero();

	three_axis bouyant_force_linear = buoyant_magnitude * three_axis::UnitZ();
	bouyant_force_linear *= -Eigen::AngleAxisf(orientation(0),
											   Eigen::Vector3f::UnitX())
								 .toRotationMatrix();
	bouyant_force_linear *= -Eigen::AngleAxisf(orientation(1),
											   Eigen::Vector3f::UnitY())
								 .toRotationMatrix();
	bouyant_force_linear *= -Eigen::AngleAxisf(orientation(2),
											   Eigen::Vector3f::UnitZ())
								 .toRotationMatrix();

	three_axis bouyant_force_rotational =
		bouyant_force_linear.cross(volume_center - mass_center);

	result.segment(0, 3) = bouyant_force_linear;
	result.segment(3, 3) = bouyant_force_rotational;
	return result;
}

six_axis Thruster_Commander::graviational_forces(three_axis orientation)
{
	six_axis result = six_axis::Zero();
	result += bouyant_force(orientation);
	result += weight_force(orientation);
	std::cout << "Gravitational forces: \n"
			  << result << std::endl;
	return result;
}

three_axis Thruster_Commander::compute_forces(force_array forces)
{
	three_axis total_force = three_axis::Zero();
	for (int i = 0; i < num_thrusters; i++)
	{
		total_force += forces.forces[i] * thruster_directions.row(i);
	}
	return total_force;
}
three_axis Thruster_Commander::compute_torques(force_array forces)
{
	three_axis total_torque = three_axis::Zero();
	for (int i = 0; i < num_thrusters; i++)
	{
		total_torque += forces.forces[i] * thruster_torques.row(i);
	}
	return total_torque;
}

six_axis Thruster_Commander::predict_drag_forces(six_axis velocity)
{
	six_axis drag_force = six_axis::Zero();

	// explanation on notion:
	// https://www.notion.so/crsucd/
	// Rotational-drag-analyssi-1478a3eca2f0801d86f2e0c8fb675c0d
	// theses values are estimates and should be improveded experimentally
	float c_inf[6] = {0.041, 0.05, 0.125, 0.005, 0.005, 0.005};

	for (int i = 0; i < 6; i++)
	{
		drag_force(0, i) = rho_water * pow(velocity(i), 2) * c_inf[i];
	}

	std::cout << "Drag Force: " << drag_force << std::endl;
	return drag_force;
}
six_axis Thruster_Commander::net_env_forces(six_axis velocity, three_axis orientation)
{
	six_axis result = six_axis::Zero();
	result += predict_drag_forces(velocity);
	result += graviational_forces(orientation);
	std::cout << "Net Environmental Forces: \n"
			  << result << std::endl;
	return result;
}
void Thruster_Commander::test_force_functions()
{
	// check thrust_compute_fz
	// check thrust_compute_fy
	// check thrust_compute_fx
	// check thrust_compute_fx_fy
	// check thrust_compute_mz
	// check thrust_compute_fz_mz
	// check thrust_compute_fy_mz
	// check thrust_compute_fx_mz
	// check thrust_compute_fx_fy_mz
	// check thrust_compute_fx_fy_fz
	// check thrust_compute_fx_fy_fz_mz
	// check thrust_compute_fx_fy_fz_mx_my_mz
}
// this one is done and seems to work
thruster_set Thruster_Commander::thrust_compute_fz(float z_force)
{

	// Force is euqally divided by the 4 vertical thrusters for this function

	float force_per_thruster = z_force / 4;
	thruster_set thrusters = thruster_set();
	for (int i = 0; i < num_thrusters; i++)
	{
		if (i < 4)
		{
			thrusters(i) = force_per_thruster;
		}
		else
		{
			thrusters(i) = 0;
		}
		std::cout << i << " " << thrusters(i) << std::endl;
	}
	return thrusters;
}
thruster_set Thruster_Commander::thrust_compute_fy(float y_force)
{

	// fx, fz and mz should be zero
	// my and mz should be small enough to keep the vehicle stable
	float force_per_thruster = y_force / 4;
	thruster_set forces = thruster_set();
	for (int i = 0; i < 4; i++)
	{
		forces(i) = 0;
	}
	for (int i = 4; i < 8; i++)
	{
		forces(i) = force_per_thruster;
	}
	// Print the thruster set
	for (int i = 0; i < num_thrusters; i++)
	{
		std::cout << i << " " << forces(i) << std::endl;
	}
	return forces;
}
thruster_set Thruster_Commander::thrust_compute_fx(float x_force)
{
	// fy, fz and mz should be zero
	// mx and my should be small enough to keep the vehicle stable
	thruster_set forces;
	return forces;
}
thruster_set Thruster_Commander::thrust_compute_fx_fy(float x_force, float y_force)
{
	// fz and mz should be zero
	// mx and my should be small enough to keep the vehicle stable
	thruster_set forces;
	return forces;
}
thruster_set Thruster_Commander::thrust_compute_mz(float z_torque)
{
	// fx, fy and fz should be zero
	// mx and my should be small enough to keep the vehicle stable
	thruster_set forces;
	return forces;
}
thruster_set Thruster_Commander::thrust_compute_fz_mz(float z_force, float z_torque)
{
	// fx and fy should be zero
	// mx and my should be small enough to keep the vehicle stable
	thruster_set forces;
	return forces;
}
thruster_set Thruster_Commander::thrust_compute_fy_mz(float y_force, float z_torque)
{
	// fx and fz should be zero
	// mx and my should be small enough to keep the vehicle stable
	thruster_set forces;
	return forces;
}
thruster_set Thruster_Commander::thrust_compute_fx_mz(float x_force, float z_torque)
{
	// fy and fz should be zero
	// mx and my should be small enough to keep the vehicle stable
	thruster_set forces;
	return forces;
}
thruster_set Thruster_Commander::thrust_compute_fx_fy_mz(float x_force, float y_force, float z_torque)
{
	// fz should be zero
	// mx and my should be small enough to keep the vehicle stable
	thruster_set forces;
	return forces;
}
thruster_set Thruster_Commander::thrust_compute_fx_fy_fz(float x_force, float y_force)
{
	// mz should be zero
	// mx and my should be small enough to keep the vehicle stable
	thruster_set forces;
	return forces;
}
thruster_set Thruster_Commander::thrust_compute_fx_fy_fz_mz(float x_distance, float y_force, float z_force, float z_torque)
{
	// mx and my should be small enough to keep the vehicle stable
	thruster_set forces;
	return forces;
}
thruster_set Thruster_Commander::thrust_compute_fx_fy_fz_mx_my_mz(six_axis force_torques)
{
	// this is the most general case
	// all forces and torques are specified
	thruster_set forces;
	return forces;
}
thruster_set Thruster_Commander::thrust_compute(six_axis force_torque, bool simple)
{
	// this is a general case force function
	// it will call other force functions depending on what forces and torques are specified
	// if simple=true, mz and my will be neglected
	thruster_set forces;
	return forces;
}
std::vector<Command> Thruster_Commander::sequence_to(six_axis target_position)
{
	Eigen::Matrix<float, 1, 6> start_position;
	start_position << position, orientation;
	std::cout << "Start Position: " << start_position << std::endl;
	std::cout << "Target Position: " << target_position << std::endl;

	Eigen::Matrix<float, 1, 6> distance = target_position - start_position;
	std::cout << "Distance: " << distance << std::endl;
	Eigen::Matrix<float, 1, 6> direction = distance.normalized();
	std::cout << "Direction: " << direction << std::endl;

	// we need a function to compute the max thrust possible in the prefered direction
	// then we need to calculate the max speed possible in that direction
	// should the robot reorient first?
	// in which orientation is the robot the fastest?

	std::vector<Command> commands;
	return commands;
}