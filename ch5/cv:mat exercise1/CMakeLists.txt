cmake_minimum_required(VERSION 3.12)
project(cv_mat_exercise1)

set(CMAKE_CXX_STANDARD 11)

add_executable(cv_mat_exercise1 main.cpp)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

target_link_libraries(cv_mat_exercise1 ${OpenCV_LIBS})