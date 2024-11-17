cmake_minimum_required(VERSION 3.28)
project(Propulsion_2024)

set(CMAKE_CXX_STANDARD 14)
add_compile_options("-DMOCK_RPI")

include_directories(.)

add_executable(Propulsion_2024
        Command.h
        Command_Interpreter.cpp
        Command_Interpreter.h
        Main.cpp
        Thruster_Commander.cpp
        Thruster_Commander.h)
