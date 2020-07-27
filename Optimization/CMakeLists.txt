cmake_minimum_required(VERSION 2.8)

project(cmake_tutorials)

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

add_executable(helloword helloworld.cpp)
target_link_libraries(helloword ${CERES_LIBRARIES})

add_executable(powell powell_function.cc)
target_link_libraries(powell ${CERES_LIBRARIES})