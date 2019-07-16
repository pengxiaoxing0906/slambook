cmake_minimum_required(VERSION 3.12)
project(imageBasics)

set(CMAKE_CXX_STANDARD 11)

add_executable(imageBasics main.cpp)

#添加C++11标准支持
set(CMAKE_CXX_FLAGES"-std=c++11")

#寻找opencv library
find_package(OpenCV REQUIRED)

#添加头文件
include_directories(${OpenCV_INCLUDE_DIRS})

#link OpenCV library
target_link_libraries(imageBasics ${OpenCV_LIBS})