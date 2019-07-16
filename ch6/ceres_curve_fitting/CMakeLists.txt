cmake_minimum_required(VERSION 3.12)
project(ceres_curve_fitting)

set(CMAKE_CXX_STANDARD 11)

add_executable(ceres_curve_fitting main.cpp)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#添加cmake模块以使用ceres库
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})
target_link_libraries(ceres_curve_fitting ${OpenCV_LIBS} ${CERES_LIBRARIES})