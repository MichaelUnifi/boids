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


# Add executable for parallel version
#add_executable(parallel parallel.cpp)
#target_link_libraries(parallel sfml-graphics sfml-window sfml-system ${OpenMP_CXX_FLAGS})


add_executable(boids_seq_benchmark boids_seq_benchmark.cpp
        constants.h
        structures.h
)
add_executable(boids_parallel_benchmark boids_parallel_benchmark.cpp
        constants.h
        structures.h)
target_link_libraries(boids_parallel_benchmark ${OpenMP_CXX_FLAGS})
