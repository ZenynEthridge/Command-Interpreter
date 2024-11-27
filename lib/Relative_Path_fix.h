#include <iostream>
#include <direct.h> // For _getcwd
#include <cstdlib>  // For MAX_PATH

std::string get_current_dir() {
	const int MAX_PATH = 260;
	char buffer[MAX_PATH];
	// Get the current working directory
	if (_getcwd(buffer, MAX_PATH) != nullptr) {
		return buffer;
	}
	else {
		return "Error: Unable to get the current working directory!";
	}
}
