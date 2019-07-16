cmake_minimum_required(VERSION 3.13)
project(Assignment5)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

add_executable(Assignment5 main.cpp)