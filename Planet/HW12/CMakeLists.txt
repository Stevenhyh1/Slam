cmake_minimum_required(VERSION 3.1.0)

project(hw12)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(homography virtual-billboard.cpp)

target_link_libraries(homography ${OpenCV_LIBRARIES})