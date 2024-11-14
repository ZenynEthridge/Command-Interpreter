#include <iostream>
#include <vector>
#include <stdexcept>
#include <cmath>
#include "ThrusterController.h"
#include "eigen-3.4.0/Eigen/Dense"



int main() {
    

	Controls Thruster = Controls();
	Thruster.print_info();
    return 0;
}

