cmake_minimum_required(VERSION 3.12)
project(pose_graph)

set(CMAKE_CXX_STANDARD 11)

list( APPEND CMAKE_MODULE_PATH /home/pxx/g2o/cmake_modules)


#Eigen
include_directories("/usr/include/eigen3")

#Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})


#G2O
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

find_package(Cholmod REQUIRED)
include_directories(${CHOLMOD_INCLUDE_DIR})

#gtsam
find_package(GTSAMCMakeTools)
find_package(GTSAM REQUIRED)

#ceres
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

add_executable(pose_graph_g2o_SE3  pose_graph_g2o_SE3.cpp)
target_link_libraries(pose_graph_g2o_SE3  g2o_core g2o_stuff g2o_types_slam3d ${CHOLMOD_LIBRARIES})


add_executable(pose_graph_g2o_lie_algebra  pose_graph_g2o_lie_algebra.cpp pose_graph_gtsam.cpp)
target_link_libraries(pose_graph_g2o_lie_algebra g2o_core g2o_stuff ${CHOLMOD_LIBRARIES} ${Sophus_LIBRARIES})

add_executable(pose_graph_gtsam pose_graph_gtsam.cpp)
target_link_libraries(pose_graph_gtsam ${CHOLMOD_LIBRARIES} gtsam)