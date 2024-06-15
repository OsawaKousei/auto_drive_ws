cmake_minimum_required(VERSION 3.22)
project(pathplanner_cpp)


# C++ Version
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)  # 17じゃなかったらエラー


# OMPL
find_package(Boost REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(ompl REQUIRED)


# Library
add_library(pathplanner_cpp STATIC
        ${PROJECT_SOURCE_DIR}/src/planning.cpp
        ${PROJECT_SOURCE_DIR}/src/spline_utils.cpp
        ${PROJECT_SOURCE_DIR}/src/util_functions.cpp
        ${PROJECT_SOURCE_DIR}/src/planner_defines.cpp
)

target_include_directories(pathplanner_cpp PUBLIC  # PUBLICなので、そとからhppをinclude出来る
        ${CMAKE_CURRENT_SOURCE_DIR}/inc
        ${OMPL_INCLUDE_DIRS}
)

target_link_libraries(pathplanner_cpp ${OMPL_LIBRARIES} ${Boost_LIBRARIES} Eigen3::Eigen)