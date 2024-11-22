#include <iostream>
#include "Thruster_Commander.h"

int main()
{
	Thruster_Commander control = Thruster_Commander();
	control.print_info();
	control.predict_drag_forces({0.5, 0.5, 0, 0, 0, 0});
    return 0;
}

