#pragma once

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include "domain/PIDController.hpp"
#include "domain/Pose.hpp"
#include "domain/Waypoint.hpp"

#include <memory>

namespace ddrobot::application {

using Twist = geometry_msgs::msg::Twist;
using Odometry = nav_msgs::msg::Odometry;

class WaypointFollowerNode : public rclcpp::Node {
public:
  WaypointFollowerNode();

private:
  void odomCallback(const Odometry::SharedPtr msg);
  void controlLoop();

  rclcpp::Publisher<Twist>::SharedPtr publisher_;
  rclcpp::Subscription<Odometry>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  ddrobot::domain::Pose current_pose_;
  ddrobot::domain::Waypoint target_;
  ddrobot::domain::PIDController linear_pid_;
  ddrobot::domain::PIDController angular_pid_;
  rclcpp::Time last_time_;
};

} // namespace ddrobot::application
