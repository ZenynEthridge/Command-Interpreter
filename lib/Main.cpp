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
    control.thrust_compute_fz(5);
    return 0;*/

    std::string filename = "data\\14V_Correlation.csv";
    std::ifstream inFile(filename);
    if (inFile.is_open()) {
        std::string line;
        while (std::getline(inFile, line)) {
            std::cout << line << std::endl;
        }
        inFile.close();
    }
    else {
        std::cerr << "Error opening file for reading." << std::endl;
    }
}

