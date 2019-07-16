cmake_minimum_required(VERSION 3.12)
project(visualiseGeometry)

set(CMAKE_CXX_STANDARD 14)



include_directories( "/usr/include/eigen3" )

# 添加Pangolin依赖
find_package( Pangolin )
include_directories( ${Pangolin_INCLUDE_DIRS} )

add_executable(visualiseGeometry main.cpp)
target_link_libraries( visualiseGeometry ${Pangolin_LIBRARIES} )