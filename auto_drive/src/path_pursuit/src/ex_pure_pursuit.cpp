#include "path_pursuit/ex_pure_pursuit.hpp"
#include <rclcpp_components/register_node_macro.hpp>

#include "pid.hpp"

#include <fstream>

using namespace std::chrono_literals;

namespace path_pursuit {
ExPurePursuit::ExPurePursuit(const rclcpp::NodeOptions &options)
    : rclcpp::Node("pure_pursuit", options) {

  // configure parameters
  declare_parameter("kp", 0.0);
  declare_parameter("ki", 0.0);
  declare_parameter("kd", 0.0);
  declare_parameter("target_dist", 0.0);
  declare_parameter("ctrl_priod", 0.1);
  get_parameter("kp", kp);
  get_parameter("ki", ki);
  get_parameter("kd", kd);
  get_parameter("target_dist", target_dist_);
  get_parameter("ctrl_priod", ctrl_priod_);
  std::cout << "kp: " << kp << " ki: " << ki << " kd: " << kd << std::endl;
  std::cout << "target_dist: " << target_dist_ << std::endl;
  std::cout << "ctrl_priod: " << ctrl_priod_ << std::endl;

  pid_x = PID(kp, ki, kd);
  pid_x.set_dt(ctrl_priod_);
  pid_y = PID(kp, ki, kd);
  pid_y.set_dt(ctrl_priod_);

  target_idx_ = 5;

  path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "local_path", 1, [this](const nav_msgs::msg::Path::SharedPtr msg) {
        path_ = *msg;
      });

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 1, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_callback(msg);
      });
  
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
}

ExPurePursuit::~ExPurePursuit() {}

// TODO : syncronize path and odom

void ExPurePursuit::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  // prev_odom_ = *msg;

  // get current position
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;

  // set current velocity
  // vel_ = sqrt(pow(x - prev_odom_.pose.pose.position.x, 2) + pow(y - prev_odom_.pose.pose.position.y, 2)) / ctrl_priod_ ;

  // check if path is empty
  if (path_.poses.size() == 0) {
    return;
  }

  // target_distより距離が遠い最小のindexを探す
  for (int i = 0; i < path_.poses.size(); i++) {
    double dx = path_.poses[i].pose.position.x - x;
    double dy = path_.poses[i].pose.position.y - y;
    double dist = sqrt(dx * dx + dy * dy);
    std::cout << "dist: " << dist << std::endl;
    if (dist > target_dist_) {
      if(!i == target_idx_){
        // pid_x.reset_integral();
        // pid_y.reset_integral();
      }
      target_idx_ = i;
      break;
    }
  }

  std::cout << "target_dist: " << target_dist_ << std::endl;

  std::cout << "target_idx: " << target_idx_ << std::endl;

  // pid control
  double target_x = path_.poses[target_idx_].pose.position.x;
  double target_y = path_.poses[target_idx_].pose.position.y;
  double cmd_x = pid_x.pid_ctrl(x, target_x);
  double cmd_y = pid_y.pid_ctrl(y, target_y);

  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = cmd_x;
  cmd_vel.linear.y = cmd_y;

  this->cmd_vel_pub_->publish(cmd_vel);
}

} // namespace path_pursuit

RCLCPP_COMPONENTS_REGISTER_NODE(path_pursuit::ExPurePursuit)
