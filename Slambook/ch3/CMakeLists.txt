cmake_minimum_required(VERSION 3.10)

include_directories("/usr/include/eigen3")

project(eigen)

add_executable(eigenMatrix eigenMatrix.cpp)
add_executable(eigenGeometry eigenGeometry.cpp)