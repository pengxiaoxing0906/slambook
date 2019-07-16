cmake_minimum_required(VERSION 2.8)
project(trianglation)

set(CMAKE_CXX_STANDARD 11)

add_executable(trianglation main.cpp)


find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(trianglation ${OpenCV_LIBS})