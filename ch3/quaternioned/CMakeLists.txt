cmake_minimum_required(VERSION 3.12)
project(quaternioned)

set(CMAKE_CXX_STANDARD 11)

add_executable(quaternioned main.cpp)

include_directories("/usr/include/eigen3")