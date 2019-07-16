cmake_minimum_required(VERSION 3.12)
project(curve_fitting_g2o)

set(CMAKE_CXX_STANDARD 11)


add_executable(curve_fitting_g2o main.cpp)

#添加cmake模块以使用ceres库
list(APPEND CMAKE_MODULE_PATH /home/pxx/g2o/cmake_modules )
set(G2O_ROOT /usr/local/include/g2o)
find_package(G2O REQUIRED)
include_directories(${G20_INCLUDE_DIRS} )
include_directories("/usr/include/eigen3")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

target_link_libraries(curve_fitting_g2o ${OpenCV_LIBS} g2o_core g2o_stuff)


