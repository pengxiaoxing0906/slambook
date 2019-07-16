cmake_minimum_required(VERSION 3.12)
project(dense_RGBD)

set(CMAKE_CXX_STANDARD 11)

# opencv
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# eigen
include_directories( "/usr/include/eigen3/" )

# pcl
find_package( PCL REQUIRED COMPONENT common io filters )
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

#octomap
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})

add_executable(dense_RGBD main.cpp )
target_link_libraries( dense_RGBD ${OpenCV_LIBS} ${PCL_LIBRARIES} )

add_executable(octo_mapping octo_mapping.cpp)
target_link_libraries(octo_mapping ${OpenCV_LIBS} ${PCL_LIBRARIES}  ${OCTOMAP_LIBRARIES})


