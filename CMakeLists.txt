cmake_minimum_required(VERSION 3.28)
project(Propulsion_2024)

set(CMAKE_CXX_STANDARD 14)

# Optional mocking flag
option(MOCK_RPI "Use mock Pico" OFF)

if (MOCK_RPI)
    add_compile_options("-DMOCK_RPI")
endif()

# Include paths
include_directories(
    ${PROJECT_SOURCE_DIR}/lib
    ${PROJECT_SOURCE_DIR}/testing
)

# Enable GTest
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/d144031940543e15423a25ae5a8a74141044862f.zip
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

# Define test target
add_executable(propulsion_test
    testing/Command_Interpreter_Testing.cpp
    lib/Command.h
    lib/Command_Interpreter.cpp
    lib/Command_Interpreter.h
    lib/Wiring.cpp
    lib/Wiring.h
    lib/Serial.cpp
    lib/Serial.h
)

# Always link GTest
target_link_libraries(propulsion_test GTest::gtest_main)

# Define mock or real behavior
if(MOCK_RPI)
    message(STATUS "Building with MOCK_RPI=ON: Using mock WiringPi (no external linkage).")
    target_compile_definitions(propulsion_test PRIVATE MOCK_RPI)
else()
    message(STATUS "Building with MOCK_RPI=OFF: Linking to real WiringPi.")
    target_link_libraries(propulsion_test wiringPi)
endif()
add_library(PropulsionFunctions
        lib/Command.h
        lib/Command_Interpreter.cpp
        lib/Command_Interpreter.h
        lib/Wiring.cpp
        lib/Wiring.h
)
include(GoogleTest)

gtest_discover_tests(propulsion_test)

