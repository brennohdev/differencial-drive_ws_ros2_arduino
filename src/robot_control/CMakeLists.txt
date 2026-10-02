cmake_minimum_required(VERSION 3.8)
project(robot_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(waypoint_follower
    src/application/WaypointFollowerNode.cpp
    src/domain/PIDController.cpp
)

target_include_directories(waypoint_follower PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

ament_target_dependencies(waypoint_follower
  rclcpp
  geometry_msgs
  nav_msgs
)

install(TARGETS waypoint_follower
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
