#include "application/WaypointFollowerNode.hpp"

#include "rclcpp/rclcpp.hpp"

#include <memory>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ddrobot::application::WaypointFollowerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
