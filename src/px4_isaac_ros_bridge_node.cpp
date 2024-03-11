#include "px4_isaac_ros_bridge.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv); 

  auto node = std::make_shared<rclcpp::Node>("px4_isaac_ros_bridge_node");

  // Create instance of your PX4_ISAAC_ROS_BRIDGE class
  auto px4_isaac_ros_bridge = std::make_shared<PX4_ISAAC_ROS_BRIDGE>(node); 

  rclcpp::spin(node); 
  px4_isaac_ros_bridge.worker_.join(); 
  
  rclcpp::shutdown();

  return 0;
}