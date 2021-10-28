# At LEAST 3.2 to get INTERFACE keyword
cmake_minimum_required(VERSION 3.2 FATAL_ERROR)
project(command_executor VERSION 0.1 LANGUAGES CXX)

# Must use GNUInstallDirs to install libraries into correct
# locations on all platforms.
include(GNUInstallDirs)

# Define C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find cpm_lib
find_package(Cpm REQUIRED)
message("Cpm directory " ${CPM_DIR})
include_directories(SYSTEM "${CPM_INCLUDE_DIR}")

# Find Fast-DDS
find_package(fastrtps REQUIRED)
message("fastrtps version: " ${fastrtps_VERSION})

# exe
include_directories(src)

add_executable(cpm_bin
    src/main.cpp
    include/lane_graph_full/lane_graph.hpp
    src/lane_graph_tools.hpp
    src/lane_graph_tools.cpp
    src/VehicleTrajectoryPlanningState.hpp
    src/VehicleTrajectoryPlanningState.cpp
    src/MultiVehicleTrajectoryPlanner.hpp
    src/MultiVehicleTrajectoryPlanner.cpp
)
target_include_directories(cpm_bin PUBLIC
    include/lane_graph_full)
target_link_libraries(cpm_bin cpm fastrtps)
target_compile_options(cpm_bin PRIVATE -Wall -Wextra -Werror=return-type -O2)

