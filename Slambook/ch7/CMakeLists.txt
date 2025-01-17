cmake_minimum_required( VERSION 2.8)
project(ch7)

set ( CMAKE_BUILD_TYPE "Debug" )
set ( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(OpenCV 3 REQUIRED)
find_package(G2O REQUIRED)
find_package(Sophus REQUIRED)

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS}
        "/usr/include/eigen3/"
)

add_executable(orb_cv orb_cv.cpp)
target_link_libraries(orb_cv ${OpenCV_LIBS})

add_executable(2d2d pose_estimation_2d2d.cpp)
target_link_libraries(2d2d ${OpenCV_LIBS})

add_executable(3d3d pose_estimation_3d3d.cpp)
target_link_libraries(3d3d ${OpenCV_LIBS} ${Sophus_LIBRARIES} g2o_core g2o_stuff)