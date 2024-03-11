#ifndef PX4_ISAAC_ROS_BRIDGE
#define PX4_ISAAC_ROS_BRIDGE

#include <nav_msgs/msg/odometry.hpp>
#include <mavros_msgs/msg/companion_process_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <thread>
#include <mutex>
#include <memory>

// https://mavlink.io/en/messages/common.html
enum class MAV_STATE
{
    MAV_STATE_UNINIT,
    MAV_STATE_BOOT,
    MAV_STATE_CALIBRATIN,
    MAV_STATE_STANDBY,
    MAV_STATE_ACTIVE,
    MAV_STATE_CRITICAL,
    MAV_STATE_EMERGENCY,
    MAV_STATE_POWEROFF,
    MAV_STATE_FLIGHT_TERMINATION,
}

class PX4_ISAAC_ROS_BRIDGE
{
public:
    PX4_ISAAC_ROS_BRIDGE(const rclcpp::Node::SharedPtr& node);
    ~PX4_ISAAC_ROS_BRIDGE();

    void publishSystemStatus();
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    std::thread worker_;

private:
    rclcpp::Node::SharedPtr node_;
    // Subscribe from Isaac ROS vSlam node
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    // Publish to Mavros node
    rclcpp::Publisher<mavros_msgs::msg::CompanionProcessStatus>::SharedPtr mavros_odom_pub_;   
    rclcpp::Publisher<mavros_msgs::msg::CompanionProcessStatus>::SharedPtr mavros_system_status_pub_; 

    MAV_STATE system_status_{MAV_STATE::MAV_STATE_UNINIT};
    MAV_STATE last_system_status_{MAV_STATE::MAV_STATE_UNINIT};

    std::unique_ptr<std::mutex>
        status_mutex_;

    bool flag_first_pose_received{false};

    ros::Time last_callback_time;
};
#endif // PX4_ISAAC_ROS_BRIDGE