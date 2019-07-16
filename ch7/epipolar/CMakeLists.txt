cmake_minimum_required(VERSION 3.12)
project(epipolar)

set(CMAKE_CXX_STANDARD 11)

add_executable(epipolar main.cpp)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(epipolar ${OpenCV_LIBS})