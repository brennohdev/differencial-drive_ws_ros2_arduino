#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include "domain/PIDController.hpp"
#include "domain/Pose.hpp"
#include "domain/Waypoint.hpp"
#include "utils/MathUtils.hpp"
#include <memory>

namespace ddrobot::application {

using Twist = geometry_msgs::msg::Twist;

class WaypointFollowerNode : public rclcpp::Node {
public:
  WaypointFollowerNode() : Node("waypoint_follower") {

    publisher_ = this->create_publisher<Twist>("cmd_vel", 10);
  }

private:
  rclcpp::Publisher<Twist>::SharedPtr publisher_;
};
} // namespace ddrobot::application

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ddrobot::application::WaypointFollowerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}