cmake_minimum_required(VERSION 3.12)
project(useGeometry)

set(CMAKE_CXX_STANDARD 14)

add_executable(useGeometry main.cpp)
include_directories("/usr/include/eigen3")