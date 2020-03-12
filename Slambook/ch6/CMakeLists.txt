cmake_minimum_required( VERSION 2.8 )
project( ceres_curve_fitting )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# 添加cmake模块以使用ceres库
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake )

# 寻找Ceres库并添加它的头文件
find_package( Ceres REQUIRED )
include_directories( ${CERES_INCLUDE_DIRS} )

# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_DIRS} )

find_package( G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

include_directories("/usr/include/eigen3")

# 与Ceres和OpenCV链接
add_executable( ceresfitting ceresfitting.cpp )
target_link_libraries( ceresfitting ${CERES_LIBRARIES} ${OpenCV_LIBS} )


add_executable( g2ofitting g2ofitting.cpp)
target_link_libraries( g2ofitting ${OpenCV_LIBS} ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY})