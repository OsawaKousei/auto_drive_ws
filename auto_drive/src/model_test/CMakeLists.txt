cmake_minimum_required(VERSION 3.8)
project(model_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
find_package(ignition-gazebo6 REQUIRED)

include_directories(SYSTEM
  ${IGNITION-COMMON_INCLUDE_DIRS}
  ${IGNITION-GUI_INCLUDE_DIRS}
)

# gui plugin
qt5_add_resources(resources_rcc src/switch_panel/SwitchPanel.qrc)
add_library(SwitchPanel SHARED
  src/switch_panel/SwitchPanel.cpp
  ${resources_rcc}
)
target_link_libraries(SwitchPanel
  ${IGNITION-COMMON_LIBRARY_DIRS}
  ${IGNITION-GUI_LIBRARIES}
)

qt5_add_resources(resources_rcc src/holonomic_panel/HolonomicPanel.qrc)
add_library(HolonomicPanel SHARED
  src/holonomic_panel/HolonomicPanel.cpp
  ${resources_rcc}
)
target_link_libraries(HolonomicPanel
  ${IGNITION-COMMON_LIBRARY_DIRS}
  ${IGNITION-GUI_LIBRARIES}
)


# system plugin
add_library(RotateAxis SHARED
  src/rotate_axis/rotate_axis.cpp
)
target_link_libraries(RotateAxis
  ${IGNITION-PLUGIN_LIBRARIES}
  ${IGNITION-GAZEBO_LIBRARIES}
)

add_library(HolonomicTest SHARED
  src/holonomic_test/holonomic_test.cpp
)
target_link_libraries(HolonomicTest
  ${IGNITION-PLUGIN_LIBRARIES}
  ${IGNITION-GAZEBO_LIBRARIES}
)

# install plugin
install(
  TARGETS SwitchPanel RotateAxis HolonomicPanel HolonomicTest
  LIBRARY DESTINATION lib
)

# install launch file and other files to share directory
install(
  DIRECTORY
    launch
    models
    worlds
  DESTINATION share/${PROJECT_NAME}
)

# gazeboがpluginを探すパスを通す？
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
