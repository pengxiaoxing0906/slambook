cmake_minimum_required(VERSION 3.12)
project(pose_estimation_3d3d)

set(CMAKE_CXX_STANDARD 11)

list(APPEND CMAKE_MODULE_PATH "/home/pxx/g2o/cmake_modules")

find_package(OpenCV REQUIRED)
find_package(G2O REQUIRED)
find_package(CSparse REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} ${G2O_INCLUDE_DIRS} ${CSPARSE_INCLUDE_DIRS} "/usr/include/eigen3/")

add_executable(pose_estimation_3d3d main.cpp)
target_link_libraries(pose_estimation_3d3d ${OpenCV_LIBS} ${CSPARSE_LIBRARY} g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension)