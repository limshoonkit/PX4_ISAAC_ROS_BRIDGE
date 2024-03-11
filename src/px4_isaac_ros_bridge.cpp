#include "px4_isaac_ros_bridge.hpp"

PX4_ISAAC_ROS_BRIDGE::PX4_ISAAC_ROS_BRIDGE(const rclcpp::Node::SharedPtr &node)
    : node_(node)
{

    // initialize subscribers
    odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
        "/camera/odom/sample_throttled", 30,
        std::bind(&PX4_Realsense_Bridge::odomCallback, this, std::placeholders::_1));

    // publishers
    mavros_odom_pub_ = node->create_publisher<nav_msgs::msg::Odometry>(
        "/mavros/odometry/out", 10);
    mavros_system_status_pub_ = node_->create_publisher<mavros_msgs::msg::CompanionProcessStatus>(
        "/mavros/companion_process/status", 1);

    last_callback_time = node->get_clock()->now();

    status_mutex_ = std::make_unique<std::mutex>();
    worker_thread_ = std::thread(&PX4_ISAAC_ROS_BRIDGE::publishSystemStatus, this);
};

PX4_ISAAC_ROS_BRIDGE::~PX4_ISAAC_ROS_BRIDGE() {}