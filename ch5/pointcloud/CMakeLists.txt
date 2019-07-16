cmake_minimum_required(VERSION 2.8)
project(pointcloud)

#set(CMAKE_CXX_STANDARD 11)
set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )



#eigen
include_directories("/usr/include/eigen3/")

#opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})


#PCL
find_package(PCL REQUIRED COMPONENT common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
add_executable(pointcloud main.cpp)
target_link_libraries(pointcloud ${OpenCV_LIBS} ${PCL_LIBRARIES})

