#include <iostream>
#include "Thruster_Commander.h"
#include "eigen-3.4.0/Eigen/Dense"
#include "eigen-3.4.0/Eigen/Geometry"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <filesystem>
#include "Relative_Path_fix.h"



int main()
{
//    auto data_file = YAML::LoadFile("jfdklfjlkdsf");
   /* Thruster_Commander control = Thruster_Commander();
    control.print_info();
	control.print_info();
    control.predict_drag_forces({0.5, 0.5, 0.1, 0.1, 0.1, 0.1});	control.weight_force({ 3.14/2, 0, 0 });
    control.bouyant_force({ 3.14/2, 0, 0 });
    control.gravitational_forces({ 3.14 / 2, 0, 0 });
    control.net_env_forces({ 0.5, 0.5, 0, 0, 0, 0 }, { 3.14/2, 0, 0 });
    control.thrust_compute_fz(5);*/
    std::ifstream file("data\\14V_Correlation.csv");

    if (file.is_open()) {
        std::cout << "Test file";
    }
	std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string value;
        std::vector<std::string> row;

        while (std::getline(ss, value, ',')) {
            row.push_back(value);
        }

        // Process the row data
        for (const auto& item : row) {
            std::cout << item << " ";
        }
        std::cout << std::endl;
    }
	const int MAX_PATH = 260;
    char buffer[MAX_PATH];

	std::cout << "Current working directory: " << get_current_dir() << std::endl;

    return 0;
}

