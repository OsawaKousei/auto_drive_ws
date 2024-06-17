cmake_minimum_required(VERSION 3.22)
project(local_path)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# OMPL
find_package(Boost REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(ompl REQUIRED)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

# localization_test
ament_auto_add_library(path_publisher src/path_publisher.cpp)
rclcpp_components_register_nodes(path_publisher "local_path::PathPublisher")
ament_auto_add_executable(
local_path_node src/local_path_node.cpp
  src/path_publisher.cpp
)

install(
  DIRECTORY
    config
    params
    launch
    maps
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()