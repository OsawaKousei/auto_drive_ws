#include "path_pursuit/path_pursuit.hpp"
#include <rclcpp_components/register_node_macro.hpp>

#include <fstream>

using namespace std::chrono_literals;

namespace path_pursuit {
PathPursuit::PathPursuit(const rclcpp::NodeOptions &options)
    : rclcpp::Node("path_pursuit", options) {

  path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) {
        path_callback(msg);
      });
  
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
}

PathPursuit::~PathPursuit() {}

void PathPursuit::path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
  RCLCPP_INFO(get_logger(), "Received path with %d poses", msg->poses.size());
}

} // namespace path_pursuit

RCLCPP_COMPONENTS_REGISTER_NODE(path_pursuit::PathPursuit)
