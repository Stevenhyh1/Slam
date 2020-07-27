cmake_minimum_required(VERSION 3.1.0)
project(hw8)

find_package(Sophus REQUIRED)
find_package(Pangolin 0.4 REQUIRED)

include_directories("/usr/include/eigen3")
include_directories(${Sophus_INCLUDE_DIRS})
include_directories(${Pangolin_INCLUDE_DIRS})

add_executable(drawtrajectory draw_trajectory.cpp)
target_link_libraries(drawtrajectory ${Sophus_LIBRARIES} ${Pangolin_LIBRARIES})