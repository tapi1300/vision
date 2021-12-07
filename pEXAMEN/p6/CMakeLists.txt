cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(MY_PCL_PROJECT)

find_package(PCL 1.4 REQUIRED COMPONENTS)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcl main.cpp)
target_link_libraries(pcl ${PCL_LIBRARIES})