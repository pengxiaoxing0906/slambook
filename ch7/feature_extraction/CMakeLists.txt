cmake_minimum_required(VERSION 2.8)
project(feature_extraction)

set(CMAKE_CXX_STANDARD 11)
add_executable(feature_extraction main.cpp)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(feature_extraction ${OpenCV_LIBS})





