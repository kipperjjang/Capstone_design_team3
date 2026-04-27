cmake_minimum_required(VERSION 3.8)
project(capstone)

# Required version
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Not using pinocchio for python
option(BUILD_FULL_CAPSTONE "Build controller and bridge executables" OFF)

# Compile with Release
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "RelWithDebInfo" "MinSizeRel")
endif()

# For debug
# if(NOT CMAKE_CONFIGURATION_TYPES)
#   if(NOT CMAKE_BUILD_TYPE)
#     set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
#     set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "RelWithDebInfo" "MinSizeRel")
#   endif()
# endif()

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
# Estimator test library
set(CAPSTONE_LIBRARY_SOURCES
  src/data/config/estimator_config.cpp
  src/estimator/estimator.cpp
  src/estimator/kalman_filter.cpp
  src/ros/test_node.cpp
)

if(BUILD_FULL_CAPSTONE)
  list(APPEND CAPSTONE_LIBRARY_SOURCES
    src/bridge/serial.cpp
    src/data/config/port_config.cpp
    src/data/config/control_config.cpp
    src/controller/controller.cpp
    src/controller/fsm.cpp
    src/ros/bridge_node.cpp
    src/ros/ctrl_node.cpp
  )
endif()

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
# Test executable
add_executable(test_node src/test.cpp)
target_link_libraries(test_node capstone_library)
ament_target_dependencies(test_node
  rclcpp
  std_msgs
  custom_msgs
)

if(BUILD_FULL_CAPSTONE)
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
endif()

# ------------------------------------------------------------------------------ #
# Install; locate compiled files into the destination
install(TARGETS
  capstone_library
  test_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_FULL_CAPSTONE)
  install(TARGETS
    ctrl_node
    bridge_node
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(DIRECTORY include/
  DESTINATION include/
)

# Python simulator; PROGRAMS stands for executable scripts such as Python
install(PROGRAMS
  scripts/vision.py
  scripts/vision_test.py
  scripts/test_visualizer.py
  scripts/test_kalman_evo.py
  scripts/prediction/sample_training_data.py
  scripts/prediction/generate_synthetic_data.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY scripts/prediction/
  DESTINATION lib/${PROJECT_NAME}/prediction
  FILES_MATCHING
    PATTERN "*.py"
    PATTERN "*.md"
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
