cmake_minimum_required(VERSION 3.12)
project(dense_monocular)

set(CMAKE_CXX_STANDARD 11)

#Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

#Eigen
include_directories("/usr/include/eigen3")

#opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(dense_monocular main.cpp)
target_link_libraries(dense_monocular ${OpenCV_LIBS} ${Sophus_LIBRARIES})