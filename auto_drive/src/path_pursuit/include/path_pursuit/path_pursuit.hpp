#ifndef PATH_PURSUIT__PATH_PURSUIT_HPP_
#define PATH_PURSUIT__PATH_PURSUIT_HPP_

#include "path_pursuit/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "pid.hpp"

namespace path_pursuit {

class PathPursuit : public rclcpp::Node {
public:
  TUTORIAL_PUBLIC
  explicit PathPursuit(const rclcpp::NodeOptions &options);
  virtual ~PathPursuit();

private:
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  nav_msgs::msg::Path path_;
  PID pid_x;
  PID pid_y;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  std::mutex mutex_;
};

} // namespace path_pursuit

#endif // PATH_PURSUIT__PATH_PURSUIT_HPP_
