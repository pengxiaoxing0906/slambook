cmake_minimum_required(VERSION 3.12)
project(pcl_test)

set(CMAKE_CXX_STANDARD 11)


find_package(PCL  REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

add_executable(pcl_test main.cpp)
target_link_libraries (pcl_test ${PCL_LIBRARIES})

install(TARGETS pcl_test RUNTIME DESTINATION bin)


