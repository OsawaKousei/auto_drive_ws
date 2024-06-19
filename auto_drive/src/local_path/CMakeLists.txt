cmake_minimum_required(VERSION 3.22)
project(local_path)
set(CMAKE_CXX_STANDARD 17)

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

# OMPL
find_package(Boost REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(ompl REQUIRED)

find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(PCL 1.1 REQUIRED)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

# localization_test
ament_auto_add_library(local_planning 
  src/local_planning.cpp
  src/util_functions.cpp
)

target_include_directories(local_planning PUBLIC
        ${OMPL_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/include/local_path
        include
)

ament_auto_add_library(path_publisher src/path_publisher.cpp)
ament_auto_add_library(local_planner src/local_planner.cpp)
rclcpp_components_register_nodes(path_publisher "local_path::PathPublisher")
rclcpp_components_register_nodes(local_planner "local_path::LocalPlanner")
ament_auto_add_executable(
local_path_node src/local_path_node.cpp
  src/path_publisher.cpp
  src/local_planner.cpp
)

ament_auto_add_executable(
static_local_planner_node src/static_local_planner_node.cpp
  src/util_functions.cpp
  src/util/pcp.cpp
)

ament_auto_add_library(pcp src/util/pcp.cpp)

target_include_directories(pcp PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include/local_path
        ${CMAKE_CURRENT_SOURCE_DIR}/include/util
        # include
        # Eigen3::Eigen
        ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(pcp
 pcl_conversions
)

target_link_libraries(pcp ${PCL_LIBRARIES})

ament_auto_add_executable(
corner_detect_node src/corner_detect_node.cpp
  src/path_publisher.cpp
  src/util/pcp.cpp
)

target_include_directories(static_local_planner_node PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include/local_path
        ${CMAKE_CURRENT_SOURCE_DIR}/include/util
        # include
        # Eigen3::Eigen
        ${PCL_INCLUDE_DIRS}
        pcp
)

target_include_directories(corner_detect_node PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include/local_path
        ${CMAKE_CURRENT_SOURCE_DIR}/include/util
        # include
        # Eigen3::Eigen
        ${PCL_INCLUDE_DIRS}
)

target_link_libraries(corner_detect_node ${PCL_LIBRARIES})
target_link_libraries(static_local_planner_node 
  ${PCL_LIBRARIES}
  pcp
)

ament_target_dependencies(corner_detect_node
 pcl_conversions
)

ament_target_dependencies(static_local_planner_node
 pcl_conversions
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