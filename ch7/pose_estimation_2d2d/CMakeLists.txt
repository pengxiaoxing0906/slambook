cmake_minimum_required(VERSION 2.8)
project(pose_estimation_2d2d)

set(CMAKE_CXX_STANDARD 11)

add_executable(pose_estimation_2d2d  main.cpp)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(pose_estimation_2d2d ${OpenCV_LIBS})