cmake_minimum_required(VERSION 3.12)
project(useLK)

set(CMAKE_CXX_STANDARD 11)

add_executable(useLK  main.cpp)


find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(useLK ${OpenCV_LIBS})