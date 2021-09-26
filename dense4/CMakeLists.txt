cmake_minimum_required(VERSION 3.0)
project(dense3)

include_directories("/usr/include/eigen3")

#OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(dense4 dense4.cpp)
target_link_libraries(dense4 ${OpenCV_LIBS} ${PCL_LIBRARIES})
