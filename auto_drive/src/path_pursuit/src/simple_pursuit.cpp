#include "path_pursuit/simple_pursuit.hpp"
#include <rclcpp_components/register_node_macro.hpp>

#include "pid.hpp"

#include <fstream>

using namespace std::chrono_literals;

namespace path_pursuit {
SimplePursuit::SimplePursuit(const rclcpp::NodeOptions &options)
    : rclcpp::Node("simple_pursuit", options) {

  // configure parameters
  declare_parameter("kp", 0.0);
  declare_parameter("ki", 0.0);
  declare_parameter("kd", 0.0);
  declare_parameter("dt", 0.1);
  get_parameter("kp", kp);
  get_parameter("ki", ki);
  get_parameter("kd", kd);
  get_parameter("dt", ctrl_priod_);
  std::cout << "kp: " << kp << " ki: " << ki << " kd: " << kd << std::endl;
  std::cout << "dt: " << ctrl_priod_ << std::endl;

  pid_x = PID(kp, ki, kd);
  pid_x.set_dt(ctrl_priod_);
  pid_y = PID(kp, ki, kd);
  pid_y.set_dt(ctrl_priod_);

  path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "local_path", 1, [this](const nav_msgs::msg::Path::SharedPtr msg) {
        path_ = *msg;
        pid_x.reset_integral();
        pid_y.reset_integral();
      });

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 1, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_callback(msg);
      });
  
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
}

SimplePursuit::~SimplePursuit() {}

// TODO : syncronize path and odom

void SimplePursuit::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  // get current position
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;

  // check if path is empty
  if (path_.poses.size() == 0) {
    return;
  }

  // pid control
  double target_x = path_.poses[2].pose.position.x;
  double target_y = path_.poses[2].pose.position.y;
  double cmd_x = pid_x.pid_ctrl(x, target_x);
  double cmd_y = pid_y.pid_ctrl(y, target_y);

  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = cmd_x;
  cmd_vel.linear.y = cmd_y;

  this->cmd_vel_pub_->publish(cmd_vel);
}

} // namespace path_pursuit

RCLCPP_COMPONENTS_REGISTER_NODE(path_pursuit::SimplePursuit)
