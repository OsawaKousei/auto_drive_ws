cmake_minimum_required(VERSION 3.8)
project(f7_sim)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# set(CMAKE_AUTOMOC ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# system plugin
ament_auto_add_executable(
  f7_sim_node src/local_path/f7_sim_node.cpp
)
# ament_auto_add_executable(
#   local_path_node src/local_path/local_path_node.cpp
# )
# ament_auto_add_executable(
#   pos_certain_node src/local_path/pos_certain_node.cpp
# )

install(
  DIRECTORY
    launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()