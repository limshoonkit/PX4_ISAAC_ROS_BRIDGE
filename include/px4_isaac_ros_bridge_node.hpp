#ifndef PX4_ISAAC_ROS_BRIDGE__PX4_ISAAC_ROS_BRIDGE_HPP_
#define PX4_ISAAC_ROS_BRIDGE__PX4_ISAAC_ROS_BRIDGE_HPP_

// System
#include <thread>
#include <mutex>
#include <memory>
// ROS2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
// Mavlink/Mavros
#include <nav_msgs/msg/odometry.hpp>
#include <mavros_msgs/msg/companion_process_status.hpp>

namespace isaac_ros
{
    namespace px4_bridge
    {

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
        };

        class PX4IsaacRosBridge : public rclcpp::Node
        {
        public:
            explicit PX4IsaacRosBridge(const rclcpp::NodeOptions options = rclcpp::NodeOptions());
            ~PX4IsaacRosBridge();

            std::thread worker_;

        private:

            // Flag to update mavros system status after receiving odometry message from isaac ros
            bool first_pose_received_ = false;

            // Subscribe odometry topic from isaac ros nodes
            // https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_visual_slam/index.html#quickstart
            const rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

            // Publish VIO topic for mavros
            // https://docs.px4.io/main/en/computer_vision/visual_inertial_odometry.html
            const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr mavros_odom_pub_;
            const rclcpp::Publisher<mavros_msgs::msg::CompanionProcessStatus>::SharedPtr mavros_system_status_pub_;

            MAV_STATE system_status_{MAV_STATE::MAV_STATE_UNINIT};
            MAV_STATE last_system_status_{MAV_STATE::MAV_STATE_UNINIT};

            void PublishSystemStatus();
            void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

            std::unique_ptr<std::mutex> status_mutex_;

            rclcpp::Time last_callback_time;
        };
    } // namespace px4_bridge
} // namespace isaac_ros

#endif // PX4_ISAAC_ROS_BRIDGE__PX4_ISAAC_ROS_BRIDGE_HPP_