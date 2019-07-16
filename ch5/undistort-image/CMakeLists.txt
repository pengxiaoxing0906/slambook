cmake_minimum_required(VERSION 3.12)
project(undistort_image)

set(CMAKE_CXX_STANDARD 11)

add_executable(undistort_image main.cpp)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

target_link_libraries(undistort_image ${OpenCV_LIBS})