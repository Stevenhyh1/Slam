cmake_minimum_required( VERSION 2.8 )
project(non_linear_optimization)

set( CMAKE_BUILD_TYPE "Debug" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake )

find_package( OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package( G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

include_directories("/usr/include/eigen3")

add_executable( curvefitting curve_fitting.cpp)
target_link_libraries( curvefitting ${OpenCV_LIBS} ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY})