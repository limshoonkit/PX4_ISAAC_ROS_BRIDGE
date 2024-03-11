#include "px4_isaac_ros_bridge_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::executors::MultiThreadedExecutor exec;
    
    auto px4_isaac_ros_bridge_node = std::make_shared<isaac_ros::px4_bridge::PX4IsaacRosBridge>();
    exec.add_node(px4_isaac_ros_bridge_node);
    exec.spin();
    px4_isaac_ros_bridge_node->worker_.join();

    rclcpp::shutdown();
    return 0;
}