cmake_minimum_required(VERSION 3.8)
project(capstone)

# Required version
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compile with Release
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "RelWithDebInfo" "MinSizeRel")
endif()

# Find ROS2 packages; REQUIRED for immediate error detection
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(custom_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# ------------------------------------------------------------------------------ #
# Core library
set(CAPSTONE_LIBRARY_SOURCES
  src/data/config/control_config.cpp
  src/data/config/port_config.cpp
  src/bridge/serial.cpp
  src/controller/fsm.cpp
  src/controller/controller.cpp
  src/ros/bridge_node.cpp
  src/ros/ctrl_node.cpp
)

add_library(capstone_library ${CAPSTONE_LIBRARY_SOURCES})
target_include_directories(capstone_library PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(capstone_library
  Eigen3::Eigen
  yaml-cpp
)

ament_target_dependencies(capstone_library
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  custom_msgs
)

# ------------------------------------------------------------------------------ #
add_executable(control_fsm_sanity_test src/controller/fsm_sanity_test.cpp)
target_link_libraries(control_fsm_sanity_test capstone_library)
ament_target_dependencies(control_fsm_sanity_test
  rclcpp
  std_msgs
  custom_msgs
)
add_test(NAME control_fsm_sanity_test COMMAND control_fsm_sanity_test)


add_executable(ctrl_node src/ctrl.cpp)
target_link_libraries(ctrl_node capstone_library)
ament_target_dependencies(ctrl_node
  rclcpp
  std_msgs
  sensor_msgs
  custom_msgs
)

add_executable(bridge_node src/bridge.cpp)
target_link_libraries(bridge_node capstone_library)
ament_target_dependencies(bridge_node
  rclcpp
  std_msgs
  sensor_msgs
  custom_msgs
)

# ------------------------------------------------------------------------------ #
# Install; locate compiled files into the destination
install(TARGETS
  capstone_library
  ctrl_node
  bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# Python simulator; PROGRAMS stands for executable scripts such as Python
install(PROGRAMS
  scripts/vision.py
  scripts/track_test.py
  scripts/vision_covariance.py
  scripts/prev_test/vision_test.py
  scripts/prev_test/test_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
)

# Launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Config and model files
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

# ------------------------------------------------------------------------------ #
# Export interfaces
ament_export_include_directories(include)
ament_export_libraries(capstone_library)
ament_export_dependencies(rclcpp std_msgs geometry_msgs sensor_msgs custom_msgs)

ament_package()
