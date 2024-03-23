### ODrive Can Interface ###

cmake_minimum_required(VERSION 3.16)

project(ODriveCanInterface VERSION 0.1.0 LANGUAGES C CXX)

# Global configurations, compiler flags, etc.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(${PROJECT_NAME} STATIC
        src/odrive_can_interface.cpp
)

# Optionally, add options for building tests, examples, etc.
option(BUILD_EXAMPLES "Build examples" OFF)
set(BUILD_EXAMPLES ON) # Use to override BUILD_EXAMPLES option
if (BUILD_EXAMPLES)
    add_subdirectory(examples)
endif ()

add_subdirectory(tools/can_simulator)

option(BUILD_TESTS "Build tests" OFF)
#set(BUILD_TESTS ON) # Uncomment to default to building tests
if (BUILD_TESTS)
    enable_testing()  # Enables CTest, which is included with CMake

    find_package(GTest REQUIRED) # Make sure Google Test is installed on your system
    include_directories(${GTEST_INCLUDE_DIRS}) # Include Google Test's headers

    add_subdirectory(tests)  # Assumes you have a 'tests' directory with a CMakeLists.txt
endif ()

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
