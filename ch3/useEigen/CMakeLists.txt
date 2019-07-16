cmake_minimum_required(VERSION 3.12)
project(useEigen)

set(CMAKE_CXX_STANDARD 14)

add_executable(useEigen main.cpp)

include_directories("/usr/include/eigen3")