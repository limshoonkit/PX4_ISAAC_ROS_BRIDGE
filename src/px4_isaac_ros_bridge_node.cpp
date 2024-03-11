#include "px4_isaac_ros_bridge_node.hpp"

namespace isaac_ros
{
  namespace px4_bridge
  {

    PX4IsaacRosBridge::PX4IsaacRosBridge(rclcpp::NodeOptions options)
        : Node("px4_isaac_ros_bridge", options),
          odom_sub_(create_subscription<nav_msgs::msg::Odometry>("visual_slam/tracking/odometry", rclcpp::QoS(100), std::bind(&PX4IsaacRosBridge::OdomCallback, this, std::placeholders::_1))),
          mavros_odom_pub_(create_publisher<nav_msgs::msg::Odometry>("/mavros/odometry/out", rclcpp::QoS(100))),
          mavros_system_status_pub_(create_publisher<mavros_msgs::msg::CompanionProcessStatus>("/mavros/companion_process/status", rclcpp::QoS(1)))
    {
      last_callback_time = this->now();
      status_mutex_.reset(new std::mutex);
      worker_ = std::thread(&PX4IsaacRosBridge::PublishSystemStatus, this);
      RCLCPP_INFO(this->get_logger(), "px4_isaac_ros_bridge launched");
    }

    PX4IsaacRosBridge::~PX4IsaacRosBridge()
    {
    }

    void PX4IsaacRosBridge::PublishSystemStatus()
    {
      rclcpp::Rate loop_rate(1.0); // 1Hz
      while (rclcpp::ok())
      {
        if (first_pose_received_)
        {
          const auto current_time = this->now();
          if (current_time - last_callback_time > rclcpp::Duration::from_seconds(0.5))
          {
            RCLCPP_WARN(this->get_logger(), "No odometry from isaac_ros");
            system_status_ = MAV_STATE::MAV_STATE_FLIGHT_TERMINATION;
          }
          auto mavros_status = std::make_unique<mavros_msgs::msg::CompanionProcessStatus>();
          mavros_status->header.stamp = current_time;
          mavros_status->set__component(197U); // MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY
          {
            std::lock_guard<std::mutex> status_guard(*(status_mutex_));
            mavros_status->set__state(static_cast<int>(system_status_));
            mavros_system_status_pub_->publish(std::move(mavros_status));
          }
        }
        loop_rate.sleep();
      }
    }

    void PX4IsaacRosBridge::OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      // Create a copy of the received message
      auto mavros_out = std::make_unique<nav_msgs::msg::Odometry>(*msg);
      mavros_out->header = msg->header;
      mavros_out->child_frame_id = msg->child_frame_id;
      mavros_odom_pub_->publish(std::move(mavros_out));

      first_pose_received_ = true;
      {
        std::lock_guard<std::mutex> status_guard(*(status_mutex_));
        last_system_status_ = system_status_;

        if (msg->pose.covariance[0] > 0.1)
          system_status_ = MAV_STATE::MAV_STATE_FLIGHT_TERMINATION;
        else if (msg->pose.covariance[0] == 0.1)
          system_status_ = MAV_STATE::MAV_STATE_CRITICAL;
        else if (msg->pose.covariance[0] == 0.01)
          system_status_ = MAV_STATE::MAV_STATE_ACTIVE;
        else
          RCLCPP_WARN(this->get_logger(), "Unexpected vision variance");
      }

      last_callback_time = this->now();
      if (last_system_status_ != system_status_)
      {
        auto mavros_status = std::make_unique<mavros_msgs::msg::CompanionProcessStatus>();
        mavros_status->header.stamp = last_callback_time;
        mavros_status->set__component(197U); // MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY
        mavros_status->set__state(static_cast<int>(system_status_));
        mavros_system_status_pub_->publish(std::move(mavros_status));
      }
    }

  } // namespace px4_bridge
} // namespace isaac_ros
