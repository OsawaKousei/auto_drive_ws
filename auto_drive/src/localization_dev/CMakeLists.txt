cmake_minimum_required(VERSION 3.8)
project(localization_dev)

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
ament_auto_add_library(local_accuracy src/localization_test/local_accuracy.cpp)
ament_auto_add_library(localization src/localization_test/localization.cpp)
ament_auto_add_library(noisy_odom src/localization_test/noisy_odom.cpp)
ament_auto_add_library(map_publisher src/localization_test/map_publisher.cpp)
ament_auto_add_library(odom_modifier src/localization_test/odom_modifier.cpp)
rclcpp_components_register_nodes(local_accuracy "localization_dev::LocalAccuracy")
rclcpp_components_register_nodes(localization "localization_dev::Localization")
rclcpp_components_register_nodes(noisy_odom "localization_dev::NoisyOdom")
rclcpp_components_register_nodes(map_publisher "localization_dev::MapPublisher")
rclcpp_components_register_nodes(odom_modifier "localization_dev::OdomModifier")
ament_auto_add_executable(
localization_test_node src/localization_test/localization_test_node.cpp
  src/localization_test/local_accuracy.cpp
  src/localization_test/localization.cpp
  src/localization_test/noisy_odom.cpp
  src/localization_test/map_publisher.cpp
  src/pc_mapping/odom_tf.cpp
  src/pc_mapping/pc2octmap.cpp
  src/localization_test/odom_modifier.cpp
)

# pc_mapping
ament_auto_add_library(pc2_filter src/pc_mapping/pc2_filter.cpp)
ament_auto_add_library(pc2_mapping src/pc_mapping/pc2_mapping.cpp)
ament_auto_add_library(scan2pc2 src/pc_mapping/scan2pc2.cpp)
ament_auto_add_library(odom_tf src/pc_mapping/odom_tf.cpp)
ament_auto_add_library(save_pc src/pc_mapping/save_pc.cpp)
ament_auto_add_library(pc2octmap src/pc_mapping/pc2octmap.cpp)
rclcpp_components_register_nodes(pc2_filter "localization_dev::Pc2Filter")
rclcpp_components_register_nodes(pc2_mapping "localization_dev::Pc2Mapping")
rclcpp_components_register_nodes(scan2pc2 "localization_dev::Scan2pc2")
rclcpp_components_register_nodes(odom_tf "localization_dev::OdomTf")
rclcpp_components_register_nodes(save_pc "localization_dev::SavePc")
rclcpp_components_register_nodes(pc2octmap "localization_dev::Pc2octmap")
ament_auto_add_executable(
pc_mapping_node src/pc_mapping/pc_mapping_node.cpp
  src/pc_mapping/pc2_filter.cpp
  src/pc_mapping/pc2_mapping.cpp
  src/pc_mapping/scan2pc2.cpp
  src/pc_mapping/odom_tf.cpp
  src/pc_mapping/save_pc.cpp
  src/pc_mapping/pc2octmap.cpp
)

# settig to use PCL in ros2 node
target_link_libraries(pc2_filter ${PCL_LIBRARIES})
ament_target_dependencies(pc2_filter
 pcl_conversions
)

target_link_libraries(pc2_mapping ${PCL_LIBRARIES})
ament_target_dependencies(pc2_mapping
 pcl_conversions
)

target_link_libraries(map_publisher ${PCL_LIBRARIES})

target_include_directories(map_publisher
  PUBLIC pcp_util/include
)
ament_target_dependencies(map_publisher
 pcl_conversions
)

target_link_libraries(pc2octmap ${PCL_LIBRARIES})
ament_target_dependencies(pc2octmap
 pcl_conversions
)

# test node in pc_mapping
ament_auto_add_executable(dammy_scan_node src/pc_mapping/dammy_scan_node.cpp)

install(
  DIRECTORY
    launch
    config
    map
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
