cmake_minimum_required(VERSION 3.10)
project(RoboControlMath)

set(CMAKE_CXX_STANDARD 17)

add_executable(kinematics_test forward_kinematics.cpp)