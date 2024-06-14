cmake_minimum_required(VERSION 3.8)
project(f7_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(f7_sim src/f7_sim.cpp)
ament_auto_add_library(noisy_odom src/noisy_odom.cpp)
rclcpp_components_register_nodes(f7_sim "f7_sim::F7Sim")
rclcpp_components_register_nodes(noisy_odom "f7_sim::NoisyOdom")
ament_auto_add_executable(
f7_sim_node src/f7_sim_node.cpp
  src/f7_sim.cpp
  src/noisy_odom.cpp
)

install(
  DIRECTORY
    # launch
    # config
    # params
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
