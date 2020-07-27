cmake_minimum_required(VERSION 2.8.0)

project(HW1)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

add_executable(ImageIO ImageIO.cpp)
target_link_libraries(ImageIO ${OpenCV_LIBS} Boost::filesystem)