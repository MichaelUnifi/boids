cmake_minimum_required(VERSION 3.10)
project(boids-bartoloni)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find SFML
find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)

# Enable OpenMP
find_package(OpenMP REQUIRED)

# Add executable for sequential version
add_executable(boids_visualization boids_visualization.cpp
        constants.h
        structures.h)
target_link_libraries(boids_visualization sfml-graphics sfml-window sfml-system)

add_executable(sequential sequential.cpp
        constants.h
        structures.h
)

add_executable(parallel parallel.cpp
        constants.h
        structures.h)
target_link_libraries(parallel ${OpenMP_CXX_FLAGS})
