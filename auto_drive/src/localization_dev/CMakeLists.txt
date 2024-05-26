cmake_minimum_required(VERSION 3.8)
project(localization_dev)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(local_accuracy src/local_accuracy.cpp)
ament_auto_add_library(localization src/localization.cpp)
ament_auto_add_library(noisy_odom src/noisy_odom.cpp)
rclcpp_components_register_nodes(local_accuracy "holonomic_sim::LocalAccuracy")
rclcpp_components_register_nodes(localization "holonomic_sim::Localization")
rclcpp_components_register_nodes(noisy_odom "holonomic_sim::NoisyOdom")
ament_auto_add_executable(
localization_test_node src/localization_test_node.cpp
  src/local_accuracy.cpp
  src/localization.cpp
  src/noisy_odom.cpp
)

install(
  DIRECTORY
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
