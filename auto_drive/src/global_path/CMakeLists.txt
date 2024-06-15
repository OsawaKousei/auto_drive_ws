cmake_minimum_required(VERSION 3.22)
project(global_path)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# matplotlibcpp
find_package(matplotlibcpp17)
find_package(Python3 COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)


# OMPL
find_package(Boost REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(ompl REQUIRED)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(global_planning src/global_planning.cpp)


ament_auto_add_executable(global_path src/main.cpp
        src/global_planning.cpp
)

target_include_directories(global_planning PUBLIC
        ${OMPL_INCLUDE_DIRS}
        include
)

target_link_libraries(global_path matplotlibcpp17::matplotlibcpp17
        ${Python3_LIBRARIES} pybind11::embed ${OMPL_LIBRARIES} ${Boost_LIBRARIES} global_planning
)