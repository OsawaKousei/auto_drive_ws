cmake_minimum_required(VERSION 3.22)
project(PathTest)
set(CMAKE_CXX_STANDARD 17)

# matplotlibcpp
find_package(matplotlibcpp17)
find_package(Python3 COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)
add_subdirectory(pathplanner_cpp)

add_executable(PathTest main.cpp)
target_include_directories(PathTest PUBLIC ${Python3_INCLUDE_DIRS})
target_link_libraries(PathTest matplotlibcpp17::matplotlibcpp17
        ${Python3_LIBRARIES} pybind11::embed ${OMPL_LIBRARIES} ${Boost_LIBRARIES} pathplanner_cpp
)