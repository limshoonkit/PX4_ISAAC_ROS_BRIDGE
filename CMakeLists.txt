cmake_minimum_required(VERSION 3.5)
project(isaac_ros_vslam_mavros_px4 LANGUAGES C CXX)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

#############################################
# Dependencies
find_package(mavros REQUIRED)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

###############################################################################
# INCLUDES and LIBS
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

###############################################################################
# SOURCES
set(PX4_ISAAC_ROS_SRC
  ${CMAKE_CURRENT_SOURCE_DIR}/src/px4_isaac_ros_bridge_node.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/px4_isaac_ros_bridge.cpp
)

###############################################################################
# Bin and Install

add_executable(px4_isaac_ros_bridge
  ${PX4_ISAAC_ROS_SRC}
)
ament_target_dependencies(px4_isaac_ros_bridge
  rclcpp
  rclcpp_components
)
install(TARGETS px4_isaac_ros_bridge
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install PARAMS files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install URDF files
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install LAUNCH files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
