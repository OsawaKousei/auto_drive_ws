cmake_minimum_required(VERSION 3.8)
project(nav2_localization)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

find_package(PCL 1.1 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
ament_auto_find_build_dependencies()

# localization_test
ament_auto_add_library(noisy_odom src/noisy_odom.cpp)
ament_auto_add_library(nav2_accuracy src/nav2_accuracy.cpp)
rclcpp_components_register_nodes(nav2_accuracy "localization_dev::Nav2Accuracy")
rclcpp_components_register_nodes(noisy_odom "localization_dev::NoisyOdom")

ament_auto_add_executable(
nav2_amcl_node src/nav2_amcl_node.cpp
  src/nav2_accuracy.cpp
  src/noisy_odom.cpp
)

install(
  DIRECTORY
    launch
    config
    params
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
