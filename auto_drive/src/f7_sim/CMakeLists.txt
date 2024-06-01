cmake_minimum_required(VERSION 3.8)
project(f7_sim)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# set(CMAKE_AUTOMOC ON)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

# f7_sim
ament_auto_add_library(f7_sim_node src/local_path/f7_sim_node.cpp)
ament_auto_add_library(noisy_odom_node src/local_path/noisy_odom_node.cpp)
rclcpp_components_register_nodes(f7_sim_node "f7_sim::F7_SIMNode")
rclcpp_components_register_nodes(noisy_odom_node "f7_sim::NoisyOdomNode")
ament_auto_add_executable(
  f7_sim_exec src/local_path/f7_sim.cpp
  src/local_path/f7_sim_node.cpp
  src/local_path/noisy_odom_node.cpp
)
# # system plugin
# ament_auto_add_executable(
#   f7_sim_node src/local_path/f7_sim_node.cpp
# )

install(
  DIRECTORY
    launch
    config
    params
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()