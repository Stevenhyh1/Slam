cmake_minimum_required(VERSION 2.8.3)
project(myslam)

SET(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++11)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_ros
  roscpp
  rospy
  std_msgs
  tf
)

set(ALL_TARGET_LIBRARIES "")

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

include(cmake/eigen.cmake)
include(cmake/opencv.cmake)
include(cmake/glog.cmake)
include(cmake/geographic.cmake)
include(cmake/YAML.cmake)
include(cmake/FindG2O.cmake)

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake )
find_package( G2O REQUIRED)

file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE NODE_SRCS "src/*_node.cpp")
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})

add_executable(data_pretreat_node src/apps/data_pretreat_node.cpp ${ALL_SRCS})
add_dependencies(data_pretreat_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(data_pretreat_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(front_end_node src/apps/front_end_node.cpp ${ALL_SRCS})
add_dependencies(front_end_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(front_end_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(back_end_node src/apps/back_end_node.cpp ${ALL_SRCS})
add_dependencies(back_end_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(back_end_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})