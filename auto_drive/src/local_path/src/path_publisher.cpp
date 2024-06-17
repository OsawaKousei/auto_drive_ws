#include "local_path/path_publisher.hpp"
#include <rclcpp_components/register_node_macro.hpp>

#include <fstream>

using namespace std::chrono_literals;

namespace local_path {
PathPublisher::PathPublisher(const rclcpp::NodeOptions &options)
    : rclcpp::Node("path_publisher", options) {

  // configure parameters
  declare_parameter("path_dir", "./global_path.csv");
  declare_parameter("distance_threshold", 0.01);
  get_parameter("path_dir", path_dir_);
  get_parameter("distance_threshold", distance_threshold_);
  std::cout << "path_dir: " << path_dir_ << std::endl;
  std::cout << "distance_threshold: " << distance_threshold_ << std::endl;

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

  // コールバックの呼び出しはこういう書き方もできるらしい
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&PathPublisher::odom_callback, this, std::placeholders::_1));
  path_pub_ = create_publisher<nav_msgs::msg::Path>("global_path", 10);
}

PathPublisher::~PathPublisher() {}

void PathPublisher::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  mutex_.lock();

  // check if the robot passed the next point
  double dx = path_.poses[0].pose.position.x - msg->pose.pose.position.x;
  double dy = path_.poses[0].pose.position.y - msg->pose.pose.position.y;
  double distance = std::sqrt(dx * dx + dy * dy);
  // std::cout << "distance: " << distance << std::endl;

  if (distance < distance_threshold_){
    // std::cout << "passed point: " << path_.poses[0].pose.position.x << ", " << path_.poses[0].pose.position.y << std::endl;
    // std::cout << "current position: " << msg->pose.pose.position.x << ", " << msg->pose.pose.position.y << std::endl;
    
    // update path
    path_.poses.erase(path_.poses.begin());

    //std::cout << "next point: " << path_.poses[0].pose.position.x << ", " << path_.poses[0].pose.position.y << std::endl;

    // TODO : ゴールに到達した際の適切な処理を追加
  }

  path_.header.stamp = now();
  path_pub_->publish(path_);

  mutex_.unlock();
}

} // namespace local_path

RCLCPP_COMPONENTS_REGISTER_NODE(local_path::PathPublisher)
