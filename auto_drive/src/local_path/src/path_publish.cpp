#include "local_path/path_publisher.hpp"
#include <rclcpp_components/register_node_macro.hpp>

using namespace std::chrono_literals;

namespace local_path {
PathPublisher::PathPublisher(const rclcpp::NodeOptions &options)
    : rclcpp::Node("path_publisher", options) {
  auto odom_callback = [this](const nav_msgs::msg::Odometry::SharedPtr msg) -> void {
    mutex_.lock();
    
    mutex_.unlock();
  };

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("odom", 10, odom_callback);
  path_pub_ = create_publisher<nav_msgs::msg::Path>("path", 10);
}

PathPublisher::~PathPublisher() {}

} // namespace local_path

RCLCPP_COMPONENTS_REGISTER_NODE(local_path::PathPublisher)
