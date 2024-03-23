### ODrive Can Interface Simulator ###

cmake_minimum_required(VERSION 3.16)

project(CanSimulator VERSION 0.1.0 LANGUAGES C CXX)

# Global configurations, compiler flags, etc.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(${PROJECT_NAME} STATIC
        can_simulator.cpp
        can_simulator.h
)

add_executable(RunCanSimulator main.cpp)
target_link_libraries(RunCanSimulator ${PROJECT_NAME})