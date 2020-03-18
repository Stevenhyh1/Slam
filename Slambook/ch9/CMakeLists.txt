cmake_minimum_required( VERSION 3.0)
project( visual_odometry)

list (APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set ( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set ( LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

find_package( Sophus REQUIRED)
find_package( OpenCV REQUIRED)
find_package( G2O REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${Sophus_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${G2O_INCLUDE_DIRS}
    "/usr/include/eigen3"
)

add_subdirectory(src)
add_subdirectory(app)