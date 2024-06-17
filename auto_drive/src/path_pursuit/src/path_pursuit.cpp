#include "path_pursuit/path_pursuit.hpp"
#include <rclcpp_components/register_node_macro.hpp>

#include "pid.hpp"

#include <fstream>

using namespace std::chrono_literals;

namespace path_pursuit {
PathPursuit::PathPursuit(const rclcpp::NodeOptions &options)
    : rclcpp::Node("path_pursuit", options) {

  pid_x = PID(1, 0, 0);
  pid_y = PID(1, 0, 0);

  path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "path", 1, [this](const nav_msgs::msg::Path::SharedPtr msg) {
        path_ = *msg;
      });

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 1, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_callback(msg);
      });
  
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
}

PathPursuit::~PathPursuit() {}

// TODO : syncronize path and odom

void PathPursuit::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  // get current position
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;

  // pid control
  double target_x = path_.poses[0].pose.position.x;
  double target_y = path_.poses[0].pose.position.y;
  double cmd_x = pid_x.pid_ctrl(x, target_x);
  double cmd_y = pid_y.pid_ctrl(y, target_y);

  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = cmd_x;
  cmd_vel.linear.y = cmd_y;

  this->cmd_vel_pub_->publish(cmd_vel);
}

} // namespace path_pursuit

RCLCPP_COMPONENTS_REGISTER_NODE(path_pursuit::PathPursuit)
