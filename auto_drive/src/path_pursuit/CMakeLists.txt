cmake_minimum_required(VERSION 3.22)
project(path_pursuit)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(pure_pursuit src/pure_pursuit.cpp)
ament_auto_add_library(simple_pursuit src/simple_pursuit.cpp)
rclcpp_components_register_nodes(pure_pursuit "path_pursuit::PurePursuit")
rclcpp_components_register_nodes(simple_pursuit "path_pursuit::SimplePursuit")
ament_auto_add_executable(
path_pursuit_node src/path_pursuit_node.cpp
  src/pure_pursuit.cpp
  src/simple_pursuit.cpp
)

ament_auto_add_executable(
  pid_test_node src/pid_test_node.cpp
)

target_include_directories(pure_pursuit PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include/path_pursuit
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