# At LEAST 2.8 but newer is better
cmake_minimum_required(VERSION 3.2 FATAL_ERROR)
project(CpmCircleExample VERSION 0.1 LANGUAGES CXX)

# Define C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Include all needed paths and libs
include(CpmDefinitions)

# Adding executables
add_executable(cpm_bin
        src/VehiclePlanner.cpp
        src/MultiVehiclePlanner.cpp
        src/main.cpp)
target_link_libraries(cpm_bin cpm_libs)
