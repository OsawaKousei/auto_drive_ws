#include "local_path/path_publisher.hpp"
#include <rclcpp_components/register_node_macro.hpp>

#include <fstream>

using namespace std::chrono_literals;

namespace local_path {
PathPublisher::PathPublisher(const rclcpp::NodeOptions &options)
    : rclcpp::Node("path_publisher", options) {

  // configure parameters
  declare_parameter("path_dir", "./global_path.csv");
  get_parameter("path_dir", path_dir_);
  std::cout << "path_dir: " << path_dir_ << std::endl;

  // read path from csv file
  std::ifstream ifs(path_dir_);
  if (!ifs.is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open file: %s", path_dir_.c_str());
    return;
  }

  std::string line;
  while (std::getline(ifs, line)) {
    std::istringstream iss(line);
    std::string token;
    std::vector<double> point;
    while (std::getline(iss, token, ',')) {
      point.push_back(std::stod(token));
    }
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = point[0];
    pose.pose.position.y = point[1];
    pose.pose.position.z = 0.0;
    path_.poses.push_back(pose);
  }
  path_.header.frame_id = "map";

  std::cout << "read path from csv file: " << path_dir_ << std::endl;
  std::cout << "path size: " << path_.poses.size() << std::endl;

  auto odom_callback = [this](const nav_msgs::msg::Odometry::SharedPtr msg) -> void {
    mutex_.lock();
    
    // publish path
    path_.header.stamp = now();
    path_pub_->publish(path_);

    mutex_.unlock();
  };

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("odom", 10, odom_callback);
  path_pub_ = create_publisher<nav_msgs::msg::Path>("path", 10);
}

PathPublisher::~PathPublisher() {}

} // namespace local_path

RCLCPP_COMPONENTS_REGISTER_NODE(local_path::PathPublisher)
