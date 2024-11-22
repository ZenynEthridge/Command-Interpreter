#include <iostream>
#include "Thruster_Commander.h"

int main()
{
	std::cout << "Running main" << std::endl;
	Thruster_Commander thruster_commander = Thruster_Commander();
	thruster_commander.print_info();
    return 0;
}

