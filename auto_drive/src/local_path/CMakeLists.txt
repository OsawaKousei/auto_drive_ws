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
# ament_auto_find_build_dependencies()

# ament_auto_add_library(global_planning src/global_planning.cpp)

# ament_auto_add_executable(global_path_node src/global_path_node.cpp
#         src/global_planning.cpp
# )

# target_include_directories(global_planning PUBLIC
#         ${OMPL_INCLUDE_DIRS}
#         include
# )

# target_link_libraries(global_path_node
#         ${OMPL_LIBRARIES} ${Boost_LIBRARIES} global_planning
# )

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