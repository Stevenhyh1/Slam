cmake_minimum_required( VERSION 3.1.0)
project(hw9)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(undistort undistort_image.cpp)
target_link_libraries(undistort ${OpenCV_LIBRARIES})