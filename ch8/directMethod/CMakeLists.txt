cmake_minimum_required(VERSION 3.12)
project(directMethod)

set(CMAKE_CXX_STANDARD 11)


list(APPEND CMAKE_MODULE_PATH /home/pxx/g2o/cmake_modules)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(G2O)
include_directories(${G2O_INCLUDE_DIRS})

include_directories("/usr/include/eigen3")

set(G2O_LIBS g2o_core g2o_types_sba g2o_solver_csparse g2o_stuff g2o_csparse_extension)


add_executable(directMethod   direct_semidense.cpp)
target_link_libraries(directMethod ${OpenCV_LIBS} ${G2O_LIBS})