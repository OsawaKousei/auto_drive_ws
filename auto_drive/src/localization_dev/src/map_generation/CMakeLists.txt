cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(map_generator)

find_package(PCL 1.3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(map_generator src/map_generator.cpp)
target_link_libraries(map_generator ${PCL_LIBRARIES})