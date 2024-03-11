cmake_minimum_required(VERSION 3.5)
project(px4_isaac_ros_bridge LANGUAGES C CXX)

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
# find_package(mavros REQUIRED)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)

###############################################################################
# INCLUDES and LIBS
include_directories(
  /include
)

###############################################################################
# SOURCES
# px4_isaac_ros_bridge_node
ament_auto_add_library(
  px4_isaac_ros_bridge_node SHARED
  src/px4_isaac_ros_bridge_node.cpp
)

###############################################################################
# Install
ament_auto_add_executable(${PROJECT_NAME}
  src/px4_isaac_ros_bridge_main.cpp
)
target_link_libraries(${PROJECT_NAME} px4_isaac_ros_bridge_node)

ament_target_dependencies(px4_isaac_ros_bridge_node
  rclcpp
  rclcpp_components
  # mavros
)

ament_auto_package(INSTALL_TO_SHARE config launch urdf)
