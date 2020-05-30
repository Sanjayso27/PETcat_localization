cmake_minimum_required(VERSION 2.8.3)
project(depth_map)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE Release)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

cs_add_library(depth_lib src/depth_map.cpp)
cs_add_executable(depth_map_node src/depth_map_node.cpp)

target_link_libraries(depth_map_node depth_lib)

target_link_libraries(depth_map_node ${OpenCV_LIBRARIES})

cs_install()
cs_export()