#include "application/WaypointFollowerNode.hpp"

#include "utils/MathUtils.hpp"

#include <chrono>
#include <cmath>
#include <functional>

namespace ddrobot::application {

using namespace std::chrono_literals;

WaypointFollowerNode::WaypointFollowerNode()
    : Node("waypoint_follower"), linear_pid_(5.0, 0.0, 0.1),
      angular_pid_(1.5, 0.0, 0.1), last_time_(this->now()) {

  this->declare_parameter("target_x", 0.0);
  this->declare_parameter("target_y", 0.0);

  target_.x = this->get_parameter("target_x").as_double();
  target_.y = this->get_parameter("target_y").as_double();

  publisher_ = this->create_publisher<Twist>("cmd_vel", 10);

  subscription_ = this->create_subscription<Odometry>(
      "odom", 10,
      std::bind(&WaypointFollowerNode::odomCallback, this,
                std::placeholders::_1));

  timer_ = this->create_wall_timer(
      100ms, std::bind(&WaypointFollowerNode::controlLoop, this));
}

void WaypointFollowerNode::odomCallback(const Odometry::SharedPtr msg) {
  current_pose_.x = msg->pose.pose.position.x;
  current_pose_.y = msg->pose.pose.position.y;

  double x_orientation = msg->pose.pose.orientation.x;
  double y_orientation = msg->pose.pose.orientation.y;
  double z_orientation = msg->pose.pose.orientation.z;
  double w_orientation = msg->pose.pose.orientation.w;

  double siny_cosp =
      2.0 * (w_orientation * z_orientation + x_orientation * y_orientation);
  double cosy_cosp = 1.0 - 2.0 * (y_orientation * y_orientation +
                                  z_orientation * z_orientation);

  current_pose_.theta = std::atan2(siny_cosp, cosy_cosp);
}

void WaypointFollowerNode::controlLoop() {
  rclcpp::Time now = this->now();
  double delta_time = (now - last_time_).seconds();

  if (delta_time <= 0.0)
    return;

  last_time_ = now;

  double distance_error = ddrobot::utils::distance(
      current_pose_.x, current_pose_.y, target_.x, target_.y);

  double dx = target_.x - current_pose_.x;
  double dy = target_.y - current_pose_.y;
  double desired_angle = std::atan2(dy, dx);
  double angular_error =
      ddrobot::utils::normalizeAngle(desired_angle - current_pose_.theta);

  double linear_velocity = linear_pid_.compute(distance_error, delta_time);
  double angular_velocity = angular_pid_.compute(angular_error, delta_time);

  auto twist = Twist();
  twist.linear.x = linear_velocity;
  twist.angular.z = angular_velocity;

  publisher_->publish(twist);
}

} // namespace ddrobot::application
