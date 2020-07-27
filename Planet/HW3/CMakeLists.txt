cmake_minimum_required(VERSION 3.1.0)

project(hw3)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(cppnewfeature cppNewFeather.cpp)
target_link_libraries(cppnewfeature ${OpenCV_LIBS})