#include "local_path/local_planner.hpp"
#include <rclcpp_components/register_node_macro.hpp>

#include "local_path/local_planner.hpp"

#include <fstream>
#include <local_path/local_planning.hpp>

using namespace std::chrono_literals;

namespace local_path {
LocalPlanner::LocalPlanner(const rclcpp::NodeOptions &options)
    : rclcpp::Node("path_publisher", options) {

  path_sub_ = create_subscription<nav_msgs::msg::Path>("global_path", 1, [this](const nav_msgs::msg::Path::SharedPtr msg) { path_callback(msg); });
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("odom", 1, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { this->odom_ = *msg; });
  local_path_pub_ = create_publisher<nav_msgs::msg::Path>("local_path", 1);
}

LocalPlanner::~LocalPlanner() {}

void LocalPlanner::path_callback(const nav_msgs::msg::Path::SharedPtr msg){
  mutex_.lock();

  // pick the first 10 points
  path_.header = msg->header;
  path_.poses.clear();
  for (int i = 0; i < 10; i++){
    path_.poses.push_back(msg->poses[i]);
  }

  // add the current position
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position = odom_.pose.pose.position;
  pose.pose.orientation = odom_.pose.pose.orientation;
  path_.poses.insert(path_.poses.begin(), pose);

  xs.clear();
  ys.clear();

  for (auto& state : path_.poses) {
      xs.push_back(state.pose.position.x);
      ys.push_back(state.pose.position.y);
  }

  auto start_time = std::chrono::system_clock::now();
  auto [xs_new, ys_new] = spline_by_num(xs, ys, 20);  // スプライン補間
  auto end_time = std::chrono::system_clock::now();
  double elapsed_first = std::chrono::duration_cast<std::chrono::milliseconds>(end_time-start_time).count(); //処理に要した時間をミリ秒に変換

  auto local_path = nav_msgs::msg::Path();
  local_path.header = path_.header;
  for (int i = 0; i < xs_new.size(); i++){
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = xs_new[i];
    pose.pose.position.y = ys_new[i];
    pose.pose.position.z = 0.0;
    local_path.poses.push_back(pose);
  }

  // publish the local path
  local_path_pub_->publish(local_path);

  mutex_.unlock();
}

} // namespace local_path

RCLCPP_COMPONENTS_REGISTER_NODE(local_path::LocalPlanner)
