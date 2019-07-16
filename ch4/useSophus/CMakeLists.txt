cmake_minimum_required(VERSION 3.12)
project(useSophus)

set(CMAKE_CXX_STANDARD 11)

add_executable(useSophus main.cpp)

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

target_link_libraries(useSophus ${Sophus_LIBRARIES})