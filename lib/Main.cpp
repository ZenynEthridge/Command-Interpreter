#include <iostream>
#include "Thruster_Commander.h"
#include "eigen-3.4.0/Eigen/Dense"
#include "eigen-3.4.0/Eigen/Geometry"
#include <yaml-cpp/yaml.h>

int main()
{
//    auto data_file = YAML::LoadFile("jfdklfjlkdsf");
    Thruster_Commander control = Thruster_Commander();
    control.print_info();
	/*
	control.print_info();
    control.predict_drag_forces({0.5, 0.5, 0.1, 0.1, 0.1, 0.1});	control.weight_force({ 3.14/2, 0, 0 });
    control.bouyant_force({ 3.14/2, 0, 0 });
    control.gravitational_forces({ 3.14 / 2, 0, 0 });
    control.net_env_forces({ 0.5, 0.5, 0, 0, 0, 0 }, { 3.14/2, 0, 0 });
    control.thrust_compute_fz(5);
    */
	// Optionally, you can still call other functions or print additional info
	// control.print_info();

	Thruster_Commander commander;
	float x_force = 20.0f;
	float y_force = 50.0f;
	float z_torque = 12.0f;

	thruster_set forces = commander.thrust_compute_fx_fy_mz(x_force, y_force, z_torque);

	std::cout << "Thruster outputs:" << std::endl;
	for (int i = 0; i < forces.size(); ++i) {
		std::cout << "T" << i << ": " << forces(i) << std::endl;
	}

	return 0;

}

