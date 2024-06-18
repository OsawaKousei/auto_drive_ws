cmake_minimum_required(VERSION 3.22)
project(path_pursuit)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

# ament_auto_add_library(path_publisher src/path_publisher.cpp)
# ament_auto_add_library(local_planner src/local_planner.cpp)
# rclcpp_components_register_nodes(path_publisher "local_path::PathPublisher")
# rclcpp_components_register_nodes(local_planner "local_path::LocalPlanner")
# ament_auto_add_executable(
# local_path_node src/local_path_node.cpp
#   src/path_publisher.cpp
#   src/local_planner.cpp
# )

ament_auto_add_executable(
  pid_test_node src/pid_test_node.cpp
)

install(
  DIRECTORY
    config
    params
    launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()