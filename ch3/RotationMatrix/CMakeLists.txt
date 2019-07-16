cmake_minimum_required(VERSION 3.12)
project(exercise1)

set(CMAKE_CXX_STANDARD 14)

add_executable(RotationMatrix main.cpp)

include_directories("/usr/include/eigen3")