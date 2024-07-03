cmake_minimum_required(VERSION 3.22)
project(pcp_util)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Eigen3 REQUIRED NO_MODULE)

find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(PCL 1.1 REQUIRED)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(pcp_util SHARED src/pcp_util.cpp)

target_include_directories(pcp_util PUBLIC
        ${PCL_INCLUDE_DIRS}
)

target_link_libraries(pcp_util ${PCL_LIBRARIES})

ament_target_dependencies(pcp_util
 pcl_conversions
)

install(
  DIRECTORY
    include/
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()