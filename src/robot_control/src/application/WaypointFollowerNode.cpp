#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include "domain/PIDController.hpp"
#include "domain/Pose.hpp"
#include "domain/Waypoint.hpp"

#include "utils/MathUtils.hpp"

#include <functional>
#include <memory>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>

namespace ddrobot::application {

using Twist = geometry_msgs::msg::Twist;
using Odometry = nav_msgs::msg::Odometry;

using namespace ::ddrobot::domain;
using namespace ::ddrobot::utils;
using namespace std::chrono_literals;

class WaypointFollowerNode : public rclcpp::Node {
public:
  WaypointFollowerNode()
      : Node("waypoint_follower"), linear_pid_(5.0, 0.0, 0.1),
        angular_pid_(1.5, 0.0, 0.1) {

    publisher_ = this->create_publisher<Twist>("cmd_vel", 10);
    subscription_ = this->create_subscription<Odometry>(
        "odom", 10,
        std::bind(&WaypointFollowerNode::odomCallback, this,
                  std::placeholders::_1));

    timer_ = this->create_wall_timer(
        100ms, std::bind(&WaypointFollowerNode::controlLoop, this));
  }

private:
  rclcpp::Publisher<Twist>::SharedPtr publisher_;
  rclcpp::Subscription<Odometry>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  Pose current_pose_;
  Waypoint target_;
  PIDController linear_pid_;
  PIDController angular_pid_;
  double delta_time = 0.1;

  void odomCallback(const Odometry::SharedPtr msg) {
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

    double theta = std::atan2(siny_cosp, cosy_cosp);

    current_pose_.theta = theta;
  }

  void controlLoop() {
    double distance_error =
        distance(current_pose_.x, current_pose_.y, target_.x, target_.y);

    double dx = target_.x - current_pose_.x;
    double dy = target_.y - current_pose_.y;
    double desired_angle = std::atan2(dy, dx);
    double angular_error = normalizeAngle(desired_angle - current_pose_.theta);

    double linear_velocity = linear_pid_.compute(distance_error, delta_time);
    double angular_velocity = angular_pid_.compute(angular_error, delta_time);

    auto twist = Twist();

    twist.linear.x = linear_velocity;
    twist.angular.z = angular_velocity;

    publisher_->publish(twist);
  }
};

} // namespace ddrobot::application

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ddrobot::application::WaypointFollowerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}