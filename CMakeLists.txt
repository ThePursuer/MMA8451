cmake_minimum_required(VERSION 3.10)

project(MMA8451)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build MMA8451 driver shared library
add_library(mma8451 SHARED MMA8451.cpp)

# Build MMA8451 example program
add_executable(MMA8451_example main.cpp)
target_link_libraries(MMA8451_example mma8451)