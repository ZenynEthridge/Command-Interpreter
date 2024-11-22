# Propulsion 2024

## Developer Resources

## Running 

### Using CMake to build code

1. Install CMake. This varies from system to system.
2. Open the terminal, and `cd ~/build` (`~` is the project root)
3. `cmake ..`
4. `make`. You should get two executables: `Propulsion_2024`, and `run_tests`.
5. Run with `./run_tests` or `./Propulsion_2024`.

## Running Unit Tests
> Before you push code to the repo, you should make sure that you pass all the unit tests. Here's how:
1. Follow the instructions above to build the code (Using Cmake to build code)
2. Run `./run_tests`
3. Confirm that all tests pass (are green). If there are any failed (red) tests, check why they're failing and get them fixed!
